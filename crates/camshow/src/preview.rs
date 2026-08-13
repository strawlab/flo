//! The sending end of the preview link: webcam frames pushed to `flo`.
//!
//! A third TCP listener, separate from the control link in [`crate::server`]
//! and the video relay in [`crate::video_link`]. flo connects to it and reads
//! decimated, downscaled webcam frames to show in its browser UI. See
//! [`camshow_protocol::preview`] for the framing and why it exists.
//!
//! Nothing here may affect capture. The capture loop hands frames over through
//! a latest-wins [`watch`] channel and never waits: if flo is slow, or absent,
//! frames are simply overwritten. The loop also skips producing a preview frame
//! at all while nobody is connected, since stamping and downscaling cost real
//! CPU on the small computers this runs on.

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

use camshow_protocol::{
    preview::{PREVIEW_CLIENT_HELLO_LEN, PreviewFrameHeader, decode_client_hello},
    video::WirePixelFormat,
};
use eyre::{Result, WrapErr};
use machine_vision_formats::{ImageData, Stride, owned::OImage, pixel_format::RGB8};
use tokio::{
    io::{AsyncRead, AsyncReadExt, AsyncWriteExt},
    net::TcpListener,
    sync::watch,
};
use tracing::{debug, info, warn};

use crate::state::Timestamp;

/// One frame ready to go on the wire: already stamped and downscaled by the
/// capture loop, so this module only serializes.
pub(crate) struct PreviewFrame {
    pub(crate) image: OImage<RGB8>,
    pub(crate) timestamp: Timestamp,
}

/// The capture loop's handle on the preview link.
///
/// Cheap to consult every frame: [`Self::wanted`] is one relaxed atomic load,
/// which is the point — the expensive work of building a preview frame happens
/// only when it returns true.
#[derive(Clone)]
pub(crate) struct PreviewSink {
    tx: watch::Sender<Option<Arc<PreviewFrame>>>,
    connected: Arc<AtomicBool>,
}

impl PreviewSink {
    /// Whether anyone is reading the preview link right now.
    pub(crate) fn wanted(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }

    /// Offer a frame. Overwrites any frame not yet sent, and never blocks.
    pub(crate) fn send(&self, frame: PreviewFrame) {
        let _ = self.tx.send(Some(Arc::new(frame)));
    }
}

/// The serving end, held by the tokio side.
pub(crate) struct PreviewServer {
    rx: watch::Receiver<Option<Arc<PreviewFrame>>>,
    connected: Arc<AtomicBool>,
}

/// Build the two ends of the preview link.
pub(crate) fn channel() -> (PreviewSink, PreviewServer) {
    let (tx, rx) = watch::channel(None);
    let connected = Arc::new(AtomicBool::new(false));
    (
        PreviewSink {
            tx,
            connected: Arc::clone(&connected),
        },
        PreviewServer { rx, connected },
    )
}

impl PreviewServer {
    /// Serve preview frames until the listener fails to bind.
    ///
    /// A bind failure is fatal for the same reason it is on the other two
    /// links: it cannot be recovered from in place. Per-connection errors just
    /// end that connection and wait for flo to reconnect.
    pub(crate) async fn run(mut self, listen_addr: String) -> Result<()> {
        let listener = TcpListener::bind(&listen_addr)
            .await
            .with_context(|| format!("binding preview listener at {listen_addr}"))?;
        info!("preview link listening on {}", listener.local_addr()?);
        self.run_on_listener(listener).await
    }

    async fn run_on_listener(&mut self, listener: TcpListener) -> Result<()> {
        loop {
            let (mut stream, peer) = match listener.accept().await {
                Ok(pair) => pair,
                Err(e) => {
                    warn!("preview accept error: {e}; retrying");
                    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                    continue;
                }
            };
            if let Err(e) = read_client_hello(&mut stream).await {
                debug!("rejecting non-preview client {peer}: {e}");
                continue;
            }
            debug!("preview link connected from {peer}");
            self.connected.store(true, Ordering::Relaxed);
            // Only frames captured from here on: whatever is in the channel
            // now predates this connection and may be arbitrarily stale.
            self.rx.mark_unchanged();

            let result = Self::serve_connection(stream, &mut self.rx).await;
            self.connected.store(false, Ordering::Relaxed);
            match result {
                Ok(()) => debug!("preview link {peer} disconnected"),
                Err(e) => debug!("preview link {peer} ended: {e}"),
            }
        }
    }

    async fn serve_connection(
        mut stream: tokio::net::TcpStream,
        rx: &mut watch::Receiver<Option<Arc<PreviewFrame>>>,
    ) -> Result<()> {
        stream.set_nodelay(true).ok();
        loop {
            rx.changed().await?;
            let Some(frame) = rx.borrow_and_update().clone() else {
                continue;
            };

            let image = &frame.image;
            let header = PreviewFrameHeader::new(
                WirePixelFormat::Rgb8,
                image.width(),
                image.height(),
                image.stride().try_into()?,
                frame.timestamp.timestamp_nanos_opt().unwrap_or_default(),
            )?;
            // One write per frame would be two syscalls; the payload dominates
            // either way, but `write_all_vectored` is not on `AsyncWriteExt`,
            // so header then payload it is. A partial write mid-frame ends the
            // connection, which flo recovers from by reconnecting.
            stream.write_all(&header.encode()).await?;
            stream
                .write_all(&image.image_data()[..header.payload_len as usize])
                .await?;
        }
    }
}

/// A peer must identify itself promptly before it can activate preview work.
/// In particular, a browser's HTTP request is rejected here and the accept
/// loop remains available to FLO.
async fn read_client_hello<R>(stream: &mut R) -> Result<()>
where
    R: AsyncRead + Unpin,
{
    const HELLO_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);

    let mut hello = [0u8; PREVIEW_CLIENT_HELLO_LEN];
    tokio::time::timeout(HELLO_TIMEOUT, stream.read_exact(&mut hello))
        .await
        .wrap_err("preview client did not identify itself in time")??;
    decode_client_hello(&hello).wrap_err("invalid preview client hello")
}

/// Longest edge of a preview frame. flo re-encodes these to JPEG for a pane a
/// few hundred pixels wide, so anything larger is detail thrown away twice.
pub(crate) const PREVIEW_MAX_WIDTH: u32 = 640;

/// Reduce `src` by an integer factor so it is at most [`PREVIEW_MAX_WIDTH`]
/// wide, by dropping rows and columns.
///
/// Nearest-neighbour, not an area average: this is a monitoring view, the OSD
/// text stamped on it stays legible either way, and a box filter over a 1080p
/// frame is real work on a small computer. An integer factor keeps it to
/// indexing.
pub(crate) fn downscale_rgb8(src: &OImage<RGB8>) -> OImage<RGB8> {
    let factor = src.width().div_ceil(PREVIEW_MAX_WIDTH).max(1) as usize;
    if factor == 1 {
        return src.clone();
    }

    let src_stride = src.stride();
    let src_data = src.image_data();
    let dst_width = (src.width() as usize).div_ceil(factor);
    let dst_height = (src.height() as usize).div_ceil(factor);
    let dst_stride = dst_width * 3;

    let mut dst = vec![0u8; dst_stride * dst_height];
    for dst_row in 0..dst_height {
        let src_row_start = dst_row * factor * src_stride;
        let dst_row_start = dst_row * dst_stride;
        for dst_col in 0..dst_width {
            let s = src_row_start + dst_col * factor * 3;
            let d = dst_row_start + dst_col * 3;
            dst[d..d + 3].copy_from_slice(&src_data[s..s + 3]);
        }
    }

    OImage::new(dst_width as u32, dst_height as u32, dst_stride, dst)
        .expect("the buffer was sized from these dimensions")
}

#[cfg(test)]
mod tests {
    use tokio::io::AsyncWriteExt;

    use super::*;

    #[tokio::test]
    async fn a_browser_is_rejected_before_it_activates_the_preview() {
        let (mut browser, mut server) = tokio::io::duplex(64);
        browser.write_all(b"GET / HTTP/1.1\r\n\r\n").await.unwrap();

        assert!(read_client_hello(&mut server).await.is_err());
    }

    #[tokio::test]
    async fn a_browser_does_not_block_the_next_preview_client() {
        let (sink, mut server) = channel();
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server_task = tokio::spawn(async move { server.run_on_listener(listener).await });

        let mut browser = tokio::net::TcpStream::connect(addr).await.unwrap();
        browser.write_all(b"GET / HTTP/1.1\r\n\r\n").await.unwrap();

        let mut flo = tokio::net::TcpStream::connect(addr).await.unwrap();
        flo.write_all(&camshow_protocol::preview::encode_client_hello())
            .await
            .unwrap();
        tokio::time::timeout(std::time::Duration::from_secs(1), async {
            while !sink.wanted() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("the valid client should be accepted after the browser");

        server_task.abort();
    }

    /// `width` x `height` where each pixel's red channel is its column index,
    /// so a subsample can be checked by value.
    fn ramp(width: u32, height: u32) -> OImage<RGB8> {
        let stride = width as usize * 3;
        let mut data = vec![0u8; stride * height as usize];
        for row in 0..height as usize {
            for col in 0..width as usize {
                data[row * stride + col * 3] = col as u8;
            }
        }
        OImage::new(width, height, stride, data).unwrap()
    }

    #[test]
    fn a_small_frame_is_passed_through_unscaled() {
        let src = ramp(320, 240);
        let out = downscale_rgb8(&src);
        assert_eq!((out.width(), out.height()), (320, 240));
    }

    #[test]
    fn a_frame_at_the_limit_is_not_scaled() {
        let out = downscale_rgb8(&ramp(PREVIEW_MAX_WIDTH, 480));
        assert_eq!(out.width(), PREVIEW_MAX_WIDTH);
    }

    #[test]
    fn a_large_frame_is_reduced_below_the_limit() {
        let out = downscale_rgb8(&ramp(1920, 1080));
        assert_eq!((out.width(), out.height()), (640, 360), "1920/3 = 640");
        assert!(out.width() <= PREVIEW_MAX_WIDTH);
    }

    #[test]
    fn downscaling_samples_evenly_spaced_columns() {
        let out = downscale_rgb8(&ramp(1920, 4));
        // Factor 3, so output column n is input column 3n.
        let row = &out.image_data()[..out.stride()];
        assert_eq!(row[0], 0);
        assert_eq!(row[3], 3);
        assert_eq!(row[6], 6);
    }

    #[test]
    fn a_width_that_is_not_a_multiple_of_the_factor_still_fits() {
        // 1921 / 640 rounds up to a factor of 4, not 3.
        let out = downscale_rgb8(&ramp(1921, 1081));
        assert!(
            out.width() <= PREVIEW_MAX_WIDTH,
            "got width {}",
            out.width()
        );
        assert_eq!(out.stride(), out.width() as usize * 3);
    }

    #[test]
    fn a_sink_with_no_reader_is_not_wanted() {
        let (sink, _server) = channel();
        assert!(!sink.wanted());
        // Sending anyway is harmless, which is what the capture loop relies on
        // during the race between a disconnect and the next frame.
        sink.send(PreviewFrame {
            image: ramp(8, 8),
            timestamp: chrono::Local::now(),
        });
    }
}
