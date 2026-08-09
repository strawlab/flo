//! The capture loop: pull a frame from the webcam, decide what the operator
//! should see, write the clean webcam frame to disk if flo asked for a
//! recording, and hand the chosen frame to every enabled output.
//!
//! The recording is always the clean webcam image — that is the archival
//! record, and it is not affected by what the display and stream happen to be
//! showing.
//!
//! What is displayed and streamed can be the webcam (with the OSD stamped on
//! it) or a tracking-camera frame relayed from flo over the video link (with no
//! OSD, since the canvas is calibrated for the FPV camera's geometry). The
//! webcam is pulled on every iteration either way: it clocks the loop, it is
//! what gets recorded, and it is the fallback whenever the selected relay has
//! nothing fresh.
//!
//! Runs on a dedicated `std::thread` because frame sources are blocking.

use std::{
    ops::ControlFlow,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use eyre::{Result, WrapErr};
use flo_core::DisplaySource;
use osd_overlay::OsdFonts;
use srt_writer::BufferingSrtFrameWriter;
use strand_dynamic_frame::DynamicFrameOwned;
use tokio::sync::{mpsc, watch};
use tracing::{debug, error, info, warn};

use crate::{
    APP_NAME,
    sink::{FrameSink, SinkConfig, Sinks},
    source::{FrameSource, WebcamSource},
    state::{Frame, OsdSnapshot, RecordingCommand, Timestamp},
    video_link::LatestRelayedFrame,
};

/// How long to keep showing the last received OSD canvas after flo stops
/// sending updates. After this many seconds, the overlay is blanked.
const OSD_STALE_AFTER: Duration = Duration::from_secs(2);

/// If the camera does not produce any frame for this long, terminate the
/// process so the systemd unit can restart it.
///
/// The app gets a longer grace period before the first frame arrives because
/// some cameras need time to warm up after the process starts.
const CAMERA_FRAME_WATCHDOG_STARTUP_GRACE: Duration = Duration::from_secs(30);

/// If the camera is already streaming and then stops producing frames, fail
/// fast so the systemd unit can restart it.
const CAMERA_FRAME_WATCHDOG_TIMEOUT: Duration = Duration::from_secs(1);

/// How long to wait before asking the source again after a failed frame.
const FRAME_RETRY_BACKOFF: Duration = Duration::from_millis(100);

/// How often to repeat the "selected source has no fresh frame, showing the
/// webcam" warning. At capture rate it would otherwise be tens of lines per
/// second for as long as the relay is down.
const FALLBACK_WARN_INTERVAL: Duration = Duration::from_secs(5);

const WEBCAM_MP4_TEMPLATE: &str = "webcam%Y%m%d_%H%M%S.%f.mp4";

/// Bundled inputs to [`run`]. Each `Start` recording message carries the
/// codec config and target directory, so this struct only needs the bits
/// that come from the local environment (preferred camera, the outputs) and
/// inter-task wiring.
pub(crate) struct CameraTask {
    pub(crate) fpv_cam_human_name: Option<String>,
    /// If `Some`, this canvas is overlaid whenever no fresh OSD update is
    /// arriving from flo.
    pub(crate) test_pattern: Option<osd_utils::OsdCache>,
    pub(crate) sinks: SinkConfig,
    pub(crate) osd_rx: watch::Receiver<Option<OsdSnapshot>>,
    pub(crate) recording_rx: mpsc::UnboundedReceiver<RecordingCommand>,
    pub(crate) rtp_bitrate_rx: mpsc::UnboundedReceiver<Option<u32>>,
    /// What the display and stream should show: the `--display-source` value to
    /// begin with, then whatever flo selects at runtime. Affects neither the
    /// recording nor the frame watchdog.
    pub(crate) display_source_rx: watch::Receiver<DisplaySource>,
    /// Frames relayed from a tracking camera, for the non-webcam sources.
    pub(crate) relayed: LatestRelayedFrame,
    /// Fires (or closes) when the process should shut down.
    pub(crate) shutdown_rx: mpsc::UnboundedReceiver<()>,
}

#[derive(serde::Serialize)]
struct SrtMsg {
    timestamp: Timestamp,
    osd: osd_utils::OsdCache,
}

struct ActiveRecording {
    mp4: bg_movie_writer::BgMovieWriter,
    srt: BufferingSrtFrameWriter,
    srt_t0: Timestamp,
}

impl ActiveRecording {
    fn new(
        creation_time: Timestamp,
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
        frame: Frame,
        timestamp: Timestamp,
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

/// Runs the capture loop until shutdown is requested or every output has gone
/// away.
pub(crate) fn run(task: CameraTask) -> Result<()> {
    let CameraTask {
        fpv_cam_human_name,
        test_pattern,
        sinks,
        mut osd_rx,
        mut recording_rx,
        mut rtp_bitrate_rx,
        mut display_source_rx,
        relayed,
        mut shutdown_rx,
    } = task;

    // Built before the watchdogs start below: with the GUI enabled this waits
    // for eframe to come up, which should not count against the camera's
    // startup grace period.
    let mut sinks = Sinks::build(sinks)?;

    let fonts = OsdFonts::load();
    let glyph_table = osd_overlay::build_glyph_table();

    let shutting_down = Arc::new(AtomicBool::new(false));
    let last_frame_at = Arc::new(Mutex::new(Instant::now()));
    let first_frame_seen = Arc::new(AtomicBool::new(false));
    let watchdog_started_at = Instant::now();

    let shutting_down_for_shutdown = Arc::clone(&shutting_down);
    let on_shutdown = sinks.take_on_shutdown();
    std::thread::Builder::new()
        .name(format!("{APP_NAME}-shutdown-watch"))
        .spawn(move || {
            // A send or every sender being dropped: either way, stop.
            let _ = shutdown_rx.blocking_recv();
            shutting_down_for_shutdown.store(true, Ordering::Release);
            if let Some(on_shutdown) = on_shutdown {
                on_shutdown();
            }
        })
        .ok();

    let shutting_down_for_watchdog = Arc::clone(&shutting_down);
    let last_frame_for_watchdog = Arc::clone(&last_frame_at);
    let first_frame_seen_for_watchdog = Arc::clone(&first_frame_seen);
    std::thread::Builder::new()
        .name(format!("{APP_NAME}-frame-watchdog"))
        .spawn(move || {
            loop {
                if shutting_down_for_watchdog.load(Ordering::Acquire) {
                    break;
                }

                let (elapsed, timeout) = if first_frame_seen_for_watchdog.load(Ordering::Acquire) {
                    (
                        match last_frame_for_watchdog.lock() {
                            Ok(last) => last.elapsed(),
                            Err(poisoned) => poisoned.into_inner().elapsed(),
                        },
                        CAMERA_FRAME_WATCHDOG_TIMEOUT,
                    )
                } else {
                    (watchdog_started_at.elapsed(), CAMERA_FRAME_WATCHDOG_STARTUP_GRACE)
                };

                if elapsed > timeout {
                    error!(
                        "camera frame watchdog tripped after {:?} without frames; exiting for restart",
                        elapsed
                    );
                    std::process::exit(1);
                }

                std::thread::sleep(Duration::from_millis(100));
            }
        })
        .ok();

    // Concrete, not a `Box<dyn FrameSource>`: what gets recorded is always the
    // webcam, so this source is never substituted. A future switch of the
    // displayed/streamed source adds a *second* source feeding the OSD/output
    // path below; it does not replace this one. See [`FrameSource`].
    let mut webcam = WebcamSource::open(fpv_cam_human_name.as_deref())?;

    let mut active: Option<ActiveRecording> = None;
    let mut last_fallback_warn: Option<Instant> = None;
    let mut display_source = *display_source_rx.borrow_and_update();

    loop {
        if shutting_down.load(Ordering::Acquire) {
            break;
        }

        // Drain pending recording commands without blocking. Honor only the
        // last one to avoid getting stuck if commands queue up.
        loop {
            match recording_rx.try_recv() {
                Ok(cmd) => apply_command(cmd, &mut active),
                Err(mpsc::error::TryRecvError::Empty) => break,
                Err(mpsc::error::TryRecvError::Disconnected) => {
                    debug!("recording command channel closed; capture loop shutting down");
                    if let Some(rec) = active.take() {
                        rec.finish();
                    }
                    sinks.finish();
                    webcam.close();
                    return Ok(());
                }
            }
        }

        // Same non-blocking, last-one-wins drain as recording commands
        // above. A closed channel isn't a shutdown signal here: the
        // recording-command drain above already handles that case for both
        // channels, since the server task drops them together.
        while let Ok(kbps) = rtp_bitrate_rx.try_recv() {
            sinks.set_rtp_bitrate_kbps(kbps);
        }

        // Latest-choice-wins, and cheap enough to consult every frame.
        let selected = *display_source_rx.borrow_and_update();
        if selected != display_source {
            info!("display source changed from {display_source:?} to {selected:?}");
            display_source = selected;
            // Report a missing relay for the newly-selected source right away
            // rather than waiting out the previous source's warning interval.
            last_fallback_warn = None;
        }

        // The webcam is the loop's clock: it is what gets recorded, so the
        // recording cadence is its cadence.
        let frame = match webcam.next_frame() {
            Ok(f) => f,
            Err(e) => {
                error!("webcam frame failed: {e:?}");
                std::thread::sleep(FRAME_RETRY_BACKOFF);
                continue;
            }
        };
        match last_frame_at.lock() {
            Ok(mut last) => *last = Instant::now(),
            Err(poisoned) => {
                *poisoned.into_inner() = Instant::now();
            }
        }
        first_frame_seen.store(true, Ordering::Release);

        // Two frames from here on, and they are not interchangeable:
        // `recorded` is the clean webcam image that goes to disk, and
        // `displayed_arc` below is whatever the operator should see, which may
        // come from an entirely different camera.
        let recorded = frame.image;
        let timestamp = frame.timestamp;

        let osd_snapshot = osd_rx.borrow_and_update().clone();
        // TODO: investigate what happens when data is stale.
        // We probably want to show an error on the OSD like "STALE".
        let live_canvas = osd_snapshot
            .as_ref()
            .filter(|s| s.received_at.elapsed() < OSD_STALE_AFTER)
            .map(|s| &s.canvas);
        // Live OSD takes precedence; the test pattern is the fallback when
        // flo isn't pushing. Note that this is the canvas flo currently *has*,
        // not necessarily the one that gets drawn: it is also what the
        // recording's SRT sidecar logs, and that must stay true even while a
        // tracking camera is displayed and nothing is stamped at all.
        let active_canvas = live_canvas.or(test_pattern.as_ref());

        // A relayed tracking-camera frame is shown as it came in: the OSD
        // canvas is calibrated for the FPV camera, so stamping it onto an IR
        // frame would put the tracking marks in the wrong place. When the
        // selected relay has nothing fresh — it is starting up, flo died, the
        // camera stalled — fall back to the webcam rather than freezing on the
        // last frame. That is a display event, not a reason to stop: the frame
        // watchdog is fed by the webcam pull above and only by that.
        let relayed_frame = match display_source {
            DisplaySource::Webcam => None,
            source => {
                let fresh = relayed.peek_fresh(source);
                match &fresh {
                    // Rearm, so a later outage is reported straight away
                    // rather than waiting out the interval.
                    Some(_) => last_fallback_warn = None,
                    None => {
                        if last_fallback_warn
                            .is_none_or(|at| at.elapsed() >= FALLBACK_WARN_INTERVAL)
                        {
                            warn!("no fresh frame for {source:?}; showing the webcam instead");
                            last_fallback_warn = Some(Instant::now());
                        }
                    }
                }
                fresh
            }
        };

        let displayed_arc = match relayed_frame {
            Some(relayed_frame) => relayed_frame,
            None => {
                let mut displayed = recorded.clone();
                if let Some(canvas) = active_canvas
                    && let Err(e) =
                        osd_overlay::stamp_canvas(&mut displayed, canvas, &fonts, &glyph_table)
                {
                    warn!("OSD stamp failed: {e}");
                }
                Arc::new(DynamicFrameOwned::from_static(displayed))
            }
        };

        let recorded_arc = Arc::new(DynamicFrameOwned::from_static(recorded));

        let is_recording = active.is_some();
        if let Some(rec) = active.as_mut() {
            // Log the canvas flo is currently showing, whether or not it was
            // stamped onto the displayed frame. The recording is the webcam, so
            // its OSD record stays complete and replayable across a switch to a
            // tracking camera — the operator's telemetry history must not have
            // holes just because the live view was pointed elsewhere.
            let canvas_for_recording = active_canvas
                .cloned()
                .unwrap_or_else(|| osd_utils::OsdCache::new(30, 16));
            if let Err(e) = rec.write(recorded_arc, timestamp, canvas_for_recording) {
                error!("error writing recording frame, stopping recording: {e:?}");
                if let Some(rec) = active.take() {
                    rec.finish();
                }
            }
        }

        // Always the webcam's capture time, even when the pixels came from a
        // tracking camera. The two cameras stamp frames on different clocks
        // (strand-cam may be using a trigger timestamp with its own epoch) and
        // the RTP encoder derives presentation times from this, so taking the
        // relayed frame's own timestamp would make the stream's timeline jump
        // on every switch.
        match sinks.on_frame(displayed_arc, timestamp, is_recording) {
            ControlFlow::Continue(()) => {}
            ControlFlow::Break(()) => {
                debug!("an output asked to stop; capture loop exiting");
                break;
            }
        }
    }

    if let Some(rec) = active.take() {
        rec.finish();
    }
    sinks.finish();
    webcam.close();
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
