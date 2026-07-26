//! The receiving end of the video link: frames relayed from `flo-strand-cam`.
//!
//! A second TCP listener, separate from the control link in [`crate::server`].
//! `flo-strand-cam` connects to it and pushes tracking-camera frames so the
//! operator can switch the live view away from the FPV webcam mid-flight.
//!
//! Two properties of this module matter more than anything else it does:
//!
//! - **It never becomes the capture loop's clock.** The relayed frames are held
//!   in a single latest-wins slot that the loop *peeks at*, and deliberately
//!   not behind [`crate::source::FrameSource`], whose `next_frame` blocks. The
//!   webcam pull stays the loop's only blocking wait and the frame watchdog's
//!   only feed, so a stalled or dead tracking camera falls back to the webcam
//!   display instead of stalling the recording or killing the process.
//! - **It never displays a stale frame.** A relayed frame is only offered while
//!   it is fresh; past that the selector shows the webcam and says so.
//!
//! Received frames are normalized to tightly packed RGB8 here, once, because
//! that is what both outputs want: the GUI uploads RGB texture data, and
//! keeping one pixel format on the sink path means the display and the encoder
//! see exactly what the webcam path produces.

use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use camshow_protocol::video::{VIDEO_HEADER_LEN, VideoFrameHeader, WirePixelFormat};
use eyre::{Result, WrapErr};
use flo_core::DisplaySource;
use machine_vision_formats::{
    image_ref::ImageRef,
    owned::OImage,
    pixel_format::{Mono8, RGB8},
};
use strand_dynamic_frame::{DynamicFrame, DynamicFrameOwned};
use tokio::{io::AsyncReadExt, net::TcpListener};
use tracing::{debug, info, warn};

use crate::state::Frame;

/// A relayed frame older than this is not shown. Generous enough to bridge the
/// jitter of a decimated relay at ~30 fps, short enough that the operator never
/// looks at a frozen picture without the fallback kicking in.
pub(crate) const RELAYED_FRAME_STALE_AFTER: Duration = Duration::from_millis(500);

struct RelayedFrame {
    source: DisplaySource,
    frame: Frame,
    received_at: Instant,
}

/// The newest frame the video link has received, shared between the tokio task
/// that reads the socket and the capture thread that displays it.
///
/// One slot, not a queue: the operator wants the most recent view, so an older
/// frame that has not been displayed yet has no value and is simply overwritten.
#[derive(Clone)]
pub(crate) struct LatestRelayedFrame {
    slot: Arc<Mutex<Option<RelayedFrame>>>,
}

impl LatestRelayedFrame {
    pub(crate) fn new() -> Self {
        Self {
            slot: Arc::new(Mutex::new(None)),
        }
    }

    /// The newest frame from `source`, if one arrived recently enough to be
    /// worth showing. Leaves it in place: the webcam and the relay are not
    /// synchronized, so consecutive capture-loop iterations may legitimately
    /// display the same relayed frame twice.
    pub(crate) fn peek_fresh(&self, source: DisplaySource) -> Option<Frame> {
        let slot = self.lock();
        let relayed = slot.as_ref()?;
        (relayed.source == source && relayed.received_at.elapsed() < RELAYED_FRAME_STALE_AFTER)
            .then(|| Arc::clone(&relayed.frame))
    }

    fn store(&self, relayed: RelayedFrame) {
        *self.lock() = Some(relayed);
    }

    /// Forgets the current frame, so a dropped connection cannot leave the
    /// last frame of a dead relay sitting in the slot until it ages out.
    fn clear(&self) {
        *self.lock() = None;
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Option<RelayedFrame>> {
        // A panic while holding this lock could only come from the two trivial
        // accessors above, so the data is still consistent.
        self.slot.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// Accepts video-link connections and keeps `latest` up to date, until a bind
/// error ends the loop.
///
/// Only one relay connects at a time (there is one flo). A second connection is
/// served only after the first ends, matching the control link's behaviour.
pub(crate) async fn serve(listen_addr: String, latest: LatestRelayedFrame) -> Result<()> {
    let listener = TcpListener::bind(&listen_addr)
        .await
        .with_context(|| format!("binding video TCP listener at {listen_addr}"))?;
    info!("video link listening on {}", listener.local_addr()?);
    loop {
        let (stream, peer) = match listener.accept().await {
            Ok(pair) => pair,
            Err(e) => {
                warn!("video link accept error: {e}; retrying");
                tokio::time::sleep(Duration::from_millis(500)).await;
                continue;
            }
        };
        info!("video link connected from {peer}");

        match receive_frames(stream, &latest).await {
            Ok(()) => info!("video link {peer} disconnected cleanly"),
            Err(e) => warn!("video link {peer} ended: {e:?}"),
        }
        latest.clear();
    }
}

/// Reads frames until the connection ends or a header does not parse.
///
/// A bad header is fatal for the connection: the framing is fixed-size, so
/// there is no way to resynchronize mid-stream. Dropping the connection lets
/// flo reconnect and start from a known position.
async fn receive_frames(mut stream: tokio::net::TcpStream, latest: &LatestRelayedFrame) -> Result<()> {
    stream.set_nodelay(true).ok();
    let mut header_bytes = [0u8; VIDEO_HEADER_LEN];
    // Reused across frames: a steady stream of same-sized frames should not
    // allocate per frame.
    let mut payload: Vec<u8> = Vec::new();

    loop {
        match stream.read_exact(&mut header_bytes).await {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(e) => return Err(e).wrap_err("reading video frame header"),
        }
        let header = VideoFrameHeader::decode(&header_bytes).wrap_err("decoding video frame")?;

        payload.resize(header.payload_len as usize, 0);
        stream
            .read_exact(&mut payload)
            .await
            .wrap_err("reading video frame payload")?;

        match to_rgb8(&header, &payload) {
            Ok(frame) => latest.store(RelayedFrame {
                source: header.source,
                frame: Arc::new(DynamicFrameOwned::from_static(frame)),
                received_at: Instant::now(),
            }),
            // A conversion failure is about this frame, not the connection, so
            // keep reading rather than forcing a reconnect.
            Err(e) => debug!(
                "dropping relayed frame {} from {:?}: {e:?}",
                header.frame_number, header.source
            ),
        }
    }
}

/// Normalizes a received frame to tightly packed RGB8.
///
/// Mono8 is replicated across the three channels, which is exactly what nokhwa
/// does for the webcam, and any row padding the sending camera had is dropped.
fn to_rgb8(header: &VideoFrameHeader, payload: &[u8]) -> Result<OImage<RGB8>> {
    let (width, height, stride) = (header.width, header.height, header.stride as usize);
    let packed_stride = width as usize * 3;
    let mut dest = OImage::<RGB8>::zeros(width, height, packed_stride)
        .ok_or_else(|| eyre::eyre!("cannot allocate a {width}x{height} RGB8 frame"))?;

    // The source view borrows `payload` directly: no copy of the received bytes
    // beyond the single conversion pass into `dest`.
    match header.pixel_format {
        WirePixelFormat::Mono8 => {
            let image = ImageRef::<Mono8>::new(width, height, stride, payload)
                .ok_or_else(|| eyre::eyre!("Mono8 payload too small for {width}x{height}"))?;
            DynamicFrame::from_static_ref(&image).into_pixel_format_dest(&mut dest)
        }
        WirePixelFormat::Rgb8 => {
            let image = ImageRef::<RGB8>::new(width, height, stride, payload)
                .ok_or_else(|| eyre::eyre!("RGB8 payload too small for {width}x{height}"))?;
            DynamicFrame::from_static_ref(&image).into_pixel_format_dest(&mut dest)
        }
    }
    .wrap_err("converting a relayed frame to RGB8")?;

    Ok(dest)
}

#[cfg(test)]
mod tests {
    use machine_vision_formats::{ImageData, Stride};

    use super::*;

    fn header(pixel_format: WirePixelFormat, width: u32, height: u32, stride: u32) -> VideoFrameHeader {
        VideoFrameHeader::new(
            DisplaySource::StrandCamMain,
            pixel_format,
            width,
            height,
            stride,
            1,
            0,
        )
        .unwrap()
    }

    #[test]
    fn mono8_is_replicated_across_the_rgb_channels() {
        let header = header(WirePixelFormat::Mono8, 2, 2, 2);
        let rgb = to_rgb8(&header, &[10, 20, 30, 40]).unwrap();

        assert_eq!(rgb.stride(), 6);
        assert_eq!(
            rgb.image_data(),
            &[10, 10, 10, 20, 20, 20, 30, 30, 30, 40, 40, 40]
        );
    }

    #[test]
    fn row_padding_from_the_sending_camera_is_dropped() {
        // 2x2 Mono8 with a 4-byte stride: two padding bytes per row that must
        // not reach the display, whose texture upload assumes packed rows.
        let header = header(WirePixelFormat::Mono8, 2, 2, 4);
        let rgb = to_rgb8(&header, &[1, 2, 0xff, 0xff, 3, 4, 0xff, 0xff]).unwrap();

        assert_eq!(rgb.stride(), 6, "output rows are tightly packed");
        assert_eq!(rgb.image_data(), &[1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4]);
    }

    #[test]
    fn rgb8_passes_through() {
        let header = header(WirePixelFormat::Rgb8, 2, 1, 6);
        let rgb = to_rgb8(&header, &[1, 2, 3, 4, 5, 6]).unwrap();
        assert_eq!(rgb.image_data(), &[1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn a_short_payload_is_refused_rather_than_read_past() {
        let header = header(WirePixelFormat::Mono8, 4, 4, 4);
        assert!(to_rgb8(&header, &[0u8; 8]).is_err());
    }

    fn relayed(source: DisplaySource, received_at: Instant) -> RelayedFrame {
        let image = OImage::<RGB8>::new(1, 1, 3, vec![0u8; 3]).unwrap();
        RelayedFrame {
            source,
            frame: Arc::new(DynamicFrameOwned::from_static(image)),
            received_at,
        }
    }

    #[test]
    fn only_the_selected_source_is_offered() {
        let latest = LatestRelayedFrame::new();
        latest.store(relayed(DisplaySource::StrandCamSecondary, Instant::now()));

        assert!(
            latest.peek_fresh(DisplaySource::StrandCamMain).is_none(),
            "a frame from the other camera must not be shown"
        );
        assert!(
            latest
                .peek_fresh(DisplaySource::StrandCamSecondary)
                .is_some()
        );
    }

    #[test]
    fn a_fresh_frame_can_be_shown_more_than_once() {
        let latest = LatestRelayedFrame::new();
        latest.store(relayed(DisplaySource::StrandCamMain, Instant::now()));

        // The webcam clocks the loop and the relay is decimated, so the same
        // relayed frame legitimately covers several capture iterations.
        assert!(latest.peek_fresh(DisplaySource::StrandCamMain).is_some());
        assert!(latest.peek_fresh(DisplaySource::StrandCamMain).is_some());
    }

    #[test]
    fn a_stale_frame_is_withheld_so_the_selector_falls_back() {
        let latest = LatestRelayedFrame::new();
        let long_ago = Instant::now() - (RELAYED_FRAME_STALE_AFTER + Duration::from_millis(1));
        latest.store(relayed(DisplaySource::StrandCamMain, long_ago));

        assert!(latest.peek_fresh(DisplaySource::StrandCamMain).is_none());
    }

    #[test]
    fn a_dropped_connection_forgets_its_last_frame() {
        let latest = LatestRelayedFrame::new();
        latest.store(relayed(DisplaySource::StrandCamMain, Instant::now()));
        latest.clear();

        assert!(latest.peek_fresh(DisplaySource::StrandCamMain).is_none());
    }
}
