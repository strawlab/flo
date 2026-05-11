//! Webcam capture, OSD overlay, and recording.
//!
//! Runs on a dedicated `std::thread` because [`nokhwa`] capture is blocking.
//! Communicates with the rest of the app through tokio channels.

use std::{
    sync::{Arc, mpsc::Receiver as StdReceiver},
    time::Duration,
};

use eyre::{Result, WrapErr};
use machine_vision_formats::{owned::OImage, pixel_format};
use nokhwa::{
    Camera,
    pixel_format::RgbFormat,
    utils::{RequestedFormat, RequestedFormatType},
};
use osd_overlay::OsdFonts;
use srt_writer::BufferingSrtFrameWriter;
use strand_dynamic_frame::DynamicFrameOwned;
use tokio::sync::{mpsc, watch};
use tracing::{debug, error, info, warn};

use crate::state::{DisplayFrame, OsdSnapshot, RecordingCommand};

/// How long to keep showing the last received OSD canvas after flo stops
/// sending updates. After this many seconds, the overlay is blanked.
const OSD_STALE_AFTER: Duration = Duration::from_secs(2);

const WEBCAM_MP4_TEMPLATE: &str = "webcam%Y%m%d_%H%M%S.%f.mp4";

/// Bundled inputs to [`run`]. Each `Start` recording message carries the
/// codec config and target directory, so this struct only needs the bits
/// that come from the local environment (preferred camera) and inter-task
/// wiring.
pub(crate) struct CameraTask {
    pub(crate) fpv_cam_human_name: Option<String>,
    /// If `Some`, this canvas is overlaid whenever no fresh OSD update is
    /// arriving from flo.
    pub(crate) test_pattern: Option<osd_utils::OsdCache>,
    pub(crate) egui_ctx_rx: StdReceiver<eframe::egui::Context>,
    pub(crate) display_tx: watch::Sender<Option<DisplayFrame>>,
    pub(crate) osd_rx: watch::Receiver<Option<OsdSnapshot>>,
    pub(crate) recording_rx: mpsc::UnboundedReceiver<RecordingCommand>,
    pub(crate) shutdown_rx: tokio::sync::oneshot::Receiver<()>,
}

#[derive(serde::Serialize)]
struct SrtMsg {
    timestamp: chrono::DateTime<chrono::Local>,
    osd: osd_utils::OsdCache,
}

struct ActiveRecording {
    mp4: bg_movie_writer::BgMovieWriter,
    srt: BufferingSrtFrameWriter,
    srt_t0: chrono::DateTime<chrono::Local>,
}

impl ActiveRecording {
    fn new(
        creation_time: chrono::DateTime<chrono::Local>,
        cfg: &strand_cam_remote_control::RecordingConfig,
        data_dir: &camino::Utf8Path,
    ) -> Result<Self> {
        let fname = creation_time.format(WEBCAM_MP4_TEMPLATE).to_string();
        let mp4_path = data_dir.join(&fname);
        let mut srt_path = mp4_path.clone();
        srt_path.set_extension("osd.srt");

        info!("starting webcam recording to {mp4_path}");
        let mp4 = bg_movie_writer::BgMovieWriter::new(cfg.clone(), 100, mp4_path.clone().into());
        let srt_fd = std::fs::File::create(&srt_path)
            .with_context(|| format!("creating SRT file {srt_path}"))?;
        let srt = BufferingSrtFrameWriter::new(Box::new(srt_fd));

        Ok(Self {
            mp4,
            srt,
            srt_t0: creation_time,
        })
    }

    fn write(
        &mut self,
        frame: Arc<DynamicFrameOwned>,
        timestamp: chrono::DateTime<chrono::Local>,
        canvas: osd_utils::OsdCache,
    ) -> Result<()> {
        self.mp4.write(frame, timestamp)?;
        let pts = timestamp.signed_duration_since(self.srt_t0).to_std()?;
        let msg = SrtMsg {
            timestamp,
            osd: canvas,
        };
        self.srt.add_frame(pts, serde_json::to_string(&msg)?)?;
        Ok(())
    }

    fn finish(mut self) {
        if let Err(e) = self.mp4.finish() {
            error!("error finishing mp4 writer: {e:?}");
        }
        if let Err(e) = self.srt.close() {
            error!("error closing SRT writer: {e:?}");
        }
        info!("webcam recording closed");
    }
}

pub(crate) fn run(task: CameraTask) -> Result<()> {
    let CameraTask {
        fpv_cam_human_name,
        test_pattern,
        egui_ctx_rx,
        display_tx,
        mut osd_rx,
        mut recording_rx,
        shutdown_rx,
    } = task;

    let fonts = OsdFonts::load();
    let glyph_table = osd_overlay::build_glyph_table();

    let egui_ctx = egui_ctx_rx
        .recv()
        .map_err(|e| eyre::eyre!("never received egui context: {e}"))?;

    // Watch for the GUI exiting; close the egui window when the app is told
    // to shut down.
    let egui_ctx_for_shutdown = egui_ctx.clone();
    std::thread::Builder::new()
        .name("camshow-shutdown-watch".into())
        .spawn(move || {
            let _ = shutdown_rx.blocking_recv();
            egui_ctx_for_shutdown.send_viewport_cmd(eframe::egui::ViewportCommand::Close);
        })
        .ok();

    let mut camera = open_camera(fpv_cam_human_name.as_deref())?;
    info!("camera opened: {}", camera.info());
    camera.open_stream().context("opening camera stream")?;

    let mut active: Option<ActiveRecording> = None;

    loop {
        // Drain pending recording commands without blocking. Honor only the
        // last one to avoid getting stuck if commands queue up.
        loop {
            match recording_rx.try_recv() {
                Ok(cmd) => apply_command(cmd, &mut active),
                Err(mpsc::error::TryRecvError::Empty) => break,
                Err(mpsc::error::TryRecvError::Disconnected) => {
                    debug!("recording command channel closed; camera shutting down");
                    if let Some(rec) = active.take() {
                        rec.finish();
                    }
                    return Ok(());
                }
            }
        }

        let frame = match camera.frame() {
            Ok(f) => f,
            Err(e) => {
                error!("camera read failed: {e}");
                std::thread::sleep(Duration::from_millis(100));
                continue;
            }
        };
        let timestamp = chrono::Local::now();
        let decoded = match frame.decode_image::<RgbFormat>() {
            Ok(d) => d,
            Err(e) => {
                error!("camera decode failed: {e}");
                continue;
            }
        };

        let clean = image_buf_to_oimage(decoded);
        let mut overlaid = clean.clone();

        let osd_snapshot = osd_rx.borrow_and_update().clone();
        // TODO: investigate what happens when data is stale.
        // We probably want to show an error on the OSD like "STALE".
        let live_canvas = osd_snapshot
            .as_ref()
            .filter(|s| s.received_at.elapsed() < OSD_STALE_AFTER)
            .map(|s| &s.canvas);
        // Live OSD takes precedence; the test pattern is the fallback when
        // flo isn't pushing.
        let active_canvas = live_canvas.or(test_pattern.as_ref());

        if let Some(canvas) = active_canvas
            && let Err(e) = osd_overlay::stamp_canvas(&mut overlaid, canvas, &fonts, &glyph_table)
        {
            warn!("OSD stamp failed: {e}");
        }

        let clean_arc = Arc::new(DynamicFrameOwned::from_static(clean));
        let overlaid_arc = Arc::new(DynamicFrameOwned::from_static(overlaid));

        if let Some(rec) = active.as_mut() {
            // Log whatever was actually overlaid (or an empty canvas if
            // nothing was) so SRT playback matches what the operator saw.
            let canvas_for_recording = active_canvas
                .cloned()
                .unwrap_or_else(|| osd_utils::OsdCache::new(30, 16));
            if let Err(e) = rec.write(clean_arc, timestamp, canvas_for_recording) {
                error!("error writing recording frame, stopping recording: {e:?}");
                if let Some(rec) = active.take() {
                    rec.finish();
                }
            }
        }

        let display = DisplayFrame {
            frame: overlaid_arc,
            recording: active.is_some(),
        };
        if display_tx.send(Some(display)).is_err() {
            debug!("GUI dropped; camera thread exiting");
            break;
        }
        egui_ctx.request_repaint();
    }

    if let Some(rec) = active.take() {
        rec.finish();
    }
    if let Err(e) = camera.stop_stream() {
        warn!("error stopping camera stream: {e}");
    }
    Ok(())
}

fn apply_command(cmd: RecordingCommand, active: &mut Option<ActiveRecording>) {
    match cmd {
        RecordingCommand::Start(start) => {
            if active.is_some() {
                warn!("StartRecording received while already recording; ignoring");
                return;
            }
            match ActiveRecording::new(start.creation_time, &start.mp4_cfg, &start.data_dir) {
                Ok(rec) => *active = Some(rec),
                Err(e) => error!("failed to start recording: {e:?}"),
            }
        }
        RecordingCommand::Stop => {
            if let Some(rec) = active.take() {
                rec.finish();
            }
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
