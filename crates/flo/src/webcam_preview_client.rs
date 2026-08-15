//! Reads webcam frames from camshow's preview link and keeps the web server's
//! newest-image slot filled.
//!
//! The receiving end of [`camshow_protocol::preview`]. camshow owns the FPV
//! webcam, so this is the only way flo can show it to a browser.
//!
//! Demand-driven, which is the whole design: this task is idle, and not even
//! connected, until someone opens the preview page. Every layer below it is
//! then idle too — with nobody connected camshow does not stamp, downscale or
//! send anything. Closing the page returns the whole path to rest within a few
//! seconds.

use camshow_protocol::preview::{PREVIEW_HEADER_LEN, PreviewFrameHeader, encode_client_hello};
use camshow_protocol::video::WirePixelFormat;
use color_eyre::eyre::{self, Result, WrapErr};
use flo_webserver::WebcamPreview;
use machine_vision_formats::{image_ref::ImageRef, pixel_format::RGB8};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpStream,
    sync::watch,
};
use tracing::{debug, info, warn};

/// How long to wait before reconnecting after the link fails or ends.
const RECONNECT_DELAY: std::time::Duration = std::time::Duration::from_secs(1);

/// How often to re-check for demand while idle. Short enough that opening the
/// preview page feels immediate.
const IDLE_POLL: std::time::Duration = std::time::Duration::from_millis(250);

/// JPEG quality for preview images. A monitoring view over a radio link: high
/// enough that the stamped OSD text stays sharp, low enough to keep each frame
/// to tens of kilobytes.
const JPEG_QUALITY: u8 = 70;

/// Connect to camshow's preview link whenever the preview is wanted, decode
/// frames, and hand them to `preview` as JPEG.
///
/// Runs until cancelled. Every failure here is non-fatal: camshow may be
/// absent, restarting, or built without the preview link, and flo carries on
/// regardless. That is why the only reporting is a log line.
pub(crate) async fn run(
    mut addr_rx: watch::Receiver<Option<std::net::SocketAddr>>,
    preview: WebcamPreview,
) -> Result<()> {
    loop {
        let addr = loop {
            if let Some(addr) = *addr_rx.borrow_and_update() {
                break addr;
            }
            addr_rx.changed().await?;
        };
        info!("webcam preview link target: {addr}");
        // Nothing to do until a browser asks. No connection, so camshow does
        // not produce frames either.
        while !preview.wanted() {
            tokio::time::sleep(IDLE_POLL).await;
        }

        match TcpStream::connect(addr).await {
            Ok(stream) => {
                debug!("connected to camshow preview link at {addr}");
                if let Err(e) = read_frames(stream, &preview).await {
                    debug!("camshow preview link to {addr} ended: {e}");
                }
                // Whatever was on screen belongs to a link that is now gone.
                preview.clear();
            }
            Err(e) => {
                debug!("camshow preview connect to {addr} failed: {e}");
            }
        }
        tokio::time::sleep(RECONNECT_DELAY).await;
    }
}

/// Read frames until the link fails or the preview stops being wanted.
async fn read_frames(mut stream: TcpStream, preview: &WebcamPreview) -> Result<()> {
    stream
        .write_all(&encode_client_hello())
        .await
        .context("sending preview client hello")?;
    let mut header_buf = [0u8; PREVIEW_HEADER_LEN];
    loop {
        stream
            .read_exact(&mut header_buf)
            .await
            .context("reading preview frame header")?;
        let header = PreviewFrameHeader::decode(&header_buf)?;

        let mut payload = vec![0u8; header.payload_len as usize];
        stream
            .read_exact(&mut payload)
            .await
            .context("reading preview frame payload")?;

        // Checked after the payload is consumed, not before: leaving a partly
        // read frame in the socket would desynchronize the stream, so the
        // link is only dropped on a frame boundary.
        if !preview.wanted() {
            debug!("preview no longer wanted; closing the link");
            return Ok(());
        }

        match encode_jpeg(&header, &payload) {
            Ok(jpeg) => preview.set_jpeg(jpeg),
            // A frame we cannot encode is worth saying so about once, but not
            // worth dropping the link over: the next one may be fine.
            Err(e) => warn!("could not encode preview frame: {e}"),
        }
    }
}

fn encode_jpeg(header: &PreviewFrameHeader, payload: &[u8]) -> Result<Vec<u8>> {
    // camshow converts to RGB8 before sending; anything else means the two
    // ends disagree about the wire format, which is a bug rather than a
    // condition to handle.
    if header.pixel_format != WirePixelFormat::Rgb8 {
        eyre::bail!("unexpected preview pixel format {:?}", header.pixel_format);
    }
    let image = ImageRef::<RGB8>::new(header.width, header.height, header.stride as usize, payload)
        .ok_or_else(|| eyre::eyre!("preview frame geometry does not match its payload"))?;
    let frame = strand_dynamic_frame::DynamicFrame::from_static_ref(&image);
    frame
        .to_encoded_buffer(convert_image::EncoderOptions::Jpeg(JPEG_QUALITY))
        .context("encoding preview frame as JPEG")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame whose red channel ramps across each row, so a decode can tell
    /// "these are the pixels we were handed" from "this is a valid JPEG".
    fn ramp_payload(width: u32, height: u32) -> (PreviewFrameHeader, Vec<u8>) {
        let stride = width as usize * 3;
        let mut payload = vec![0u8; stride * height as usize];
        for row in 0..height as usize {
            for col in 0..width as usize {
                payload[row * stride + col * 3] = (col * 255 / (width as usize - 1).max(1)) as u8;
            }
        }
        let header = PreviewFrameHeader::new(
            WirePixelFormat::Rgb8,
            width,
            height,
            stride as u32,
            1_760_000_000_000_000_000,
        )
        .unwrap();
        (header, payload)
    }

    #[test]
    fn an_encoded_preview_frame_keeps_its_pixels() {
        let (header, payload) = ramp_payload(640, 360);
        let jpeg = encode_jpeg(&header, &payload).unwrap();

        let decoded = image::load_from_memory_with_format(&jpeg, image::ImageFormat::Jpeg)
            .unwrap()
            .to_rgb8();
        assert_eq!(decoded.dimensions(), (640, 360));
        // JPEG is lossy, so this checks the ramp survived in shape, not
        // exactly: a black frame would fail it by a mile.
        assert!(
            decoded.get_pixel(600, 180)[0] > decoded.get_pixel(20, 180)[0] + 100,
            "the ramp did not survive encoding: left={:?} right={:?}",
            decoded.get_pixel(20, 180),
            decoded.get_pixel(600, 180),
        );
    }

    #[test]
    fn a_payload_too_short_for_its_geometry_is_refused() {
        let (header, mut payload) = ramp_payload(64, 36);
        payload.truncate(payload.len() - 1);
        assert!(encode_jpeg(&header, &payload).is_err());
    }
}
