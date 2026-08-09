//! Where the pixels come from.
//!
//! Today the only source is a USB webcam via [`nokhwa`], whose capture call is
//! blocking — hence the capture loop's dedicated `std::thread`.
//!
//! The webcam is more than one source among equals: it is the source of the
//! clean video written to disk, always. The [`FrameSource`] trait is the seam
//! for adding sources for the *display and stream* (e.g. frames handed over
//! from strand-cam), which is a separate question from what gets recorded.

use eyre::{Result, WrapErr};
use machine_vision_formats::{owned::OImage, pixel_format};
use nokhwa::{
    Camera,
    pixel_format::RgbFormat,
    utils::{RequestedFormat, RequestedFormatType},
};
use tracing::{info, warn};

use crate::state::Timestamp;

/// One clean (no OSD) frame plus the capture time the source assigns to it.
pub(crate) struct SourceFrame {
    pub(crate) image: OImage<pixel_format::RGB8>,
    pub(crate) timestamp: Timestamp,
}

/// A source of camera frames for the capture loop.
///
/// Recording is deliberately not expressed through this trait: the capture loop
/// holds a concrete [`WebcamSource`] for the frames it writes to disk, so
/// selecting a different source for the display and stream cannot change what
/// is recorded.
///
/// Not `Send`: a source is opened and used entirely on the capture thread
/// (nokhwa's `Camera` is not `Send`), so `run` opens it there rather than
/// receiving one ready-made.
pub(crate) trait FrameSource {
    /// Blocks until the next frame is available. An `Err` is treated as
    /// transient: the capture loop logs it, backs off briefly, and asks again.
    /// A source that never recovers is caught by the loop's frame watchdog.
    fn next_frame(&mut self) -> Result<SourceFrame>;

    /// Called once, after the capture loop exits.
    fn close(&mut self);
}

/// A USB webcam, opened by human-readable name or by "first one found".
pub(crate) struct WebcamSource {
    camera: Camera,
}

impl WebcamSource {
    pub(crate) fn open(preferred_human_name: Option<&str>) -> Result<Self> {
        let mut camera = open_camera(preferred_human_name)?;
        info!("camera opened: {}", camera.info());
        camera.open_stream().context("opening camera stream")?;
        Ok(Self { camera })
    }
}

impl FrameSource for WebcamSource {
    fn next_frame(&mut self) -> Result<SourceFrame> {
        let frame = self.camera.frame().context("camera read failed")?;
        // Stamped as soon as the frame is in hand, before the (potentially
        // several millisecond) decode below.
        let timestamp = chrono::Local::now();
        let decoded = frame
            .decode_image::<RgbFormat>()
            .context("camera decode failed")?;
        Ok(SourceFrame {
            image: image_buf_to_oimage(decoded),
            timestamp,
        })
    }

    fn close(&mut self) {
        if let Err(e) = self.camera.stop_stream() {
            warn!("error stopping camera stream: {e}");
        }
    }
}

fn open_camera(preferred_human_name: Option<&str>) -> Result<Camera> {
    let (started_tx, started_rx) = std::sync::mpsc::channel();
    nokhwa::nokhwa_initialize(move |_| {
        let _ = started_tx.send(());
    });
    started_rx
        .recv()
        .map_err(|e| eyre::eyre!("nokhwa_initialize never completed: {e}"))?;

    let backend =
        nokhwa::native_api_backend().ok_or_else(|| eyre::eyre!("could not open nokhwa backend"))?;
    let devices = nokhwa::query(backend)?;
    if devices.is_empty() {
        eyre::bail!("no webcams found");
    }

    let mut chosen = 0;
    for (i, device) in devices.iter().enumerate() {
        info!("webcam #{i}: {}", device.human_name());
        if let Some(target) = preferred_human_name
            && device.human_name() == target
        {
            chosen = i;
        }
    }
    if let Some(target) = preferred_human_name
        && devices[chosen].human_name() != target
    {
        eyre::bail!("could not find preferred camera \"{target}\"");
    }

    let device = &devices[chosen];
    Camera::new(
        device.index().clone(),
        RequestedFormat::new::<RgbFormat>(RequestedFormatType::AbsoluteHighestFrameRate),
    )
    .with_context(|| format!("opening camera {}", device.human_name()))
}

fn image_buf_to_oimage(
    buf: image::ImageBuffer<image::Rgb<u8>, Vec<u8>>,
) -> OImage<pixel_format::RGB8> {
    let width = buf.width();
    let height = buf.height();
    let data = buf.into_flat_samples().samples;
    OImage::new(width, height, (width * 3) as usize, data)
        .expect("ImageBuffer dimensions are always valid for OImage")
}
