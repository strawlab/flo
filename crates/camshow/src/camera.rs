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
use flo_core::{DisplaySource, RtpTarget};
use osd_overlay::OsdFonts;
use srt_writer::BufferingSrtFrameWriter;
use strand_dynamic_frame::DynamicFrameOwned;
use tokio::sync::{mpsc, watch};
use tracing::{debug, error, info, warn};

use crate::{
    APP_NAME,
    preview::{PreviewFrame, PreviewSink, downscale_rgb8},
    sink::{FrameSink, SinkConfig, Sinks},
    source::{FrameSource, WebcamSource},
    state::{Frame, OsdSnapshot, RecordingCommand, Timestamp},
    video_link::{LatestRelayedFrame, RELAYED_FRAME_STALE_AFTER},
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
    /// The complete set of RTP stream configurations, updated by FLO at runtime.
    pub(crate) rtp_targets_rx: watch::Receiver<Vec<RtpTarget>>,
    /// What the display and stream should show: the `--display-source` value to
    /// begin with, then whatever flo selects at runtime. Affects neither the
    /// recording nor the frame watchdog.
    pub(crate) display_source_rx: watch::Receiver<DisplaySource>,
    /// Frames relayed from a tracking camera, for the non-webcam sources.
    pub(crate) relayed: LatestRelayedFrame,
    /// Where webcam frames for flo's browser preview are handed over.
    pub(crate) preview: PreviewSink,
    /// Fires (or closes) when the process should shut down.
    pub(crate) shutdown_rx: mpsc::UnboundedReceiver<()>,
}

/// How often a frame is offered to the preview link. Slow on purpose: this is
/// a monitoring view in a browser, often at the far end of a radio link, and
/// every frame sent costs a stamp, a downscale, and a JPEG encode on flo.
const PREVIEW_INTERVAL: Duration = Duration::from_millis(200);

#[derive(serde::Serialize)]
struct SrtMsg {
    timestamp: Timestamp,
    osd: osd_utils::OsdCache,
}

struct ActiveRecording {
    mp4: bg_movie_writer::BgMovieWriter,
    srt: BufferingSrtFrameWriter,
    /// Capture time of the first frame written, which the SRT cue times are
    /// relative to. Set on that first write rather than from the requested
    /// creation time: a pre-capture recording begins with a buffered frame
    /// older than the trigger, and a cue time cannot be negative.
    srt_t0: Option<Timestamp>,
}

/// A frame held for pre-capture, with everything needed to write it later.
///
/// The OSD canvas travels with the frame because the `.osd.srt` sidecar
/// records what flo was showing *at capture time*; reusing the canvas current
/// at trigger time for the whole buffered window would backdate telemetry that
/// had not happened yet.
struct BufferedFrame {
    frame: Frame,
    timestamp: Timestamp,
    osd: osd_utils::OsdCache,
}

/// Ring buffer of recent frames, so a recording can begin before the operator
/// triggered it ("pre-capture", flo's name for a post-trigger recording).
///
/// The window is the only bound, matching the tracking cameras' post-trigger
/// buffers: what the operator asks for is what is held. These are decoded
/// frames, so a long window at a high frame rate is a lot of memory — the
/// window is the knob for that.
struct PreCaptureBuffer {
    window_secs: f64,
    inner: std::collections::VecDeque<BufferedFrame>,
}

impl PreCaptureBuffer {
    fn new() -> Self {
        Self {
            window_secs: 0.0,
            inner: std::collections::VecDeque::new(),
        }
    }

    /// Update the window. Zero (or negative) disables buffering and frees
    /// whatever is held.
    fn set_window_secs(&mut self, secs: f64) {
        if secs > 0.0 {
            self.window_secs = secs;
            self.trim();
        } else {
            self.window_secs = 0.0;
            self.clear();
        }
    }

    fn is_enabled(&self) -> bool {
        self.window_secs > 0.0
    }

    fn push(&mut self, frame: Frame, timestamp: Timestamp, osd: osd_utils::OsdCache) {
        if !self.is_enabled() {
            return;
        }
        self.inner.push_back(BufferedFrame {
            frame,
            timestamp,
            osd,
        });
        self.trim();
    }

    /// Drop frames that have fallen outside the window, measured back from the
    /// newest frame, so the buffer always holds the most recent video.
    fn trim(&mut self) {
        let Some(newest) = self.inner.back().map(|b| b.timestamp) else {
            return;
        };
        while let Some(oldest) = self.inner.front() {
            if secs_between(oldest.timestamp, newest) > self.window_secs {
                self.inner.pop_front();
            } else {
                break;
            }
        }
    }

    /// Take everything held, leaving the buffer empty and ready to refill.
    fn drain(&mut self) -> std::collections::VecDeque<BufferedFrame> {
        std::mem::take(&mut self.inner)
    }

    fn clear(&mut self) {
        self.inner.clear();
    }
}

/// Stamp our identity and the recording's creation time into the config's
/// H.264 metadata SEI.
///
/// flo sends a bare codec config, whose metadata is `None`. Without this the
/// file names no writing application and, because the metadata is the only
/// place a UTC offset is recorded, readers such as `show-timestamps` display
/// every capture time in UTC.
fn with_h264_metadata(
    cfg: &strand_cam_remote_control::RecordingConfig,
    creation_time: Timestamp,
) -> strand_cam_remote_control::RecordingConfig {
    use strand_cam_remote_control::{H264Metadata, RecordingConfig};

    let metadata = H264Metadata::new(APP_NAME, creation_time.into());
    let mut cfg = cfg.clone();
    match &mut cfg {
        RecordingConfig::Mp4(c) => c.h264_metadata = Some(metadata),
        RecordingConfig::Ffmpeg(c) => c.h264_metadata = Some(metadata),
    }
    cfg
}

fn secs_between(earlier: Timestamp, later: Timestamp) -> f64 {
    (later - earlier).num_milliseconds() as f64 / 1000.0
}

/// Chooses relayed frames while hiding the transport delay of an IR-to-IR
/// source switch. The old IR frame is held only for the normal stale-frame
/// window; if the new camera does not arrive, webcam fallback still wins.
struct RelayFrameSelector {
    selected: DisplaySource,
    last_relayed: Option<Frame>,
    handover_started: Option<Instant>,
}

impl RelayFrameSelector {
    fn new(selected: DisplaySource) -> Self {
        Self {
            selected,
            last_relayed: None,
            handover_started: None,
        }
    }

    fn select(&mut self, selected: DisplaySource, fresh: Option<Frame>) -> Option<Frame> {
        self.select_at(selected, fresh, Instant::now())
    }

    fn select_at(
        &mut self,
        selected: DisplaySource,
        fresh: Option<Frame>,
        now: Instant,
    ) -> Option<Frame> {
        if selected != self.selected {
            let ir_to_ir =
                self.selected.strand_cam_role().is_some() && selected.strand_cam_role().is_some();
            self.handover_started = ir_to_ir.then_some(now);
            if !ir_to_ir {
                self.last_relayed = None;
            }
            self.selected = selected;
        }

        if selected == DisplaySource::Webcam {
            self.last_relayed = None;
            self.handover_started = None;
            return None;
        }

        if let Some(frame) = fresh {
            self.last_relayed = Some(Arc::clone(&frame));
            self.handover_started = None;
            return Some(frame);
        }

        if self
            .handover_started
            .is_some_and(|started| now.duration_since(started) < RELAYED_FRAME_STALE_AFTER)
        {
            return self.last_relayed.as_ref().map(Arc::clone);
        }

        self.last_relayed = None;
        self.handover_started = None;
        None
    }
}

impl ActiveRecording {
    fn new(
        creation_time: Timestamp,
        cfg: &strand_cam_remote_control::RecordingConfig,
        data_dir: &camino::Utf8Path,
        queue_extra: usize,
    ) -> Result<Self> {
        let fname = creation_time.format(WEBCAM_MP4_TEMPLATE).to_string();
        let mp4_path = data_dir.join(&fname);
        let mut srt_path = mp4_path.clone();
        srt_path.set_extension("osd.srt");

        info!("starting webcam recording to {mp4_path}");
        // Room for the pre-capture burst on top of the steady-state headroom:
        // the buffered frames are handed over as fast as the writer will take
        // them, and `BgMovieWriter::write` drops rather than blocks when full.
        let mp4 = bg_movie_writer::BgMovieWriter::new(
            with_h264_metadata(cfg, creation_time),
            queue_extra + 100,
            mp4_path.clone().into(),
        );
        let srt_fd = std::fs::File::create(&srt_path)
            .with_context(|| format!("creating SRT file {srt_path}"))?;
        let srt = BufferingSrtFrameWriter::new(Box::new(srt_fd));

        Ok(Self {
            mp4,
            srt,
            srt_t0: None,
        })
    }

    fn write(
        &mut self,
        frame: Frame,
        timestamp: Timestamp,
        canvas: osd_utils::OsdCache,
    ) -> Result<()> {
        self.mp4.write(frame, timestamp)?;
        let srt_t0 = *self.srt_t0.get_or_insert(timestamp);
        let pts = timestamp.signed_duration_since(srt_t0).to_std()?;
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
        mut rtp_targets_rx,
        mut display_source_rx,
        relayed,
        preview,
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
            info!("shutdown requested; stopping the capture loop");
            shutting_down_for_shutdown.store(true, Ordering::Release);
            if let Some(on_shutdown) = on_shutdown {
                // Bracketed because this is where a GUI that is not being
                // painted strands the process: the close is only queued here,
                // and only a paint can act on it. "asking" with no "asked"
                // after it means this call itself blocked; "asked" with no
                // GUI exit after it means the close was queued and ignored.
                info!("asking the GUI to close");
                on_shutdown();
                info!("asked the GUI to close");
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
    let mut precapture = PreCaptureBuffer::new();
    let mut last_preview_at: Option<Instant> = None;
    let mut last_fallback_warn: Option<Instant> = None;
    let mut display_source = *display_source_rx.borrow_and_update();
    let mut relay_selector = RelayFrameSelector::new(display_source);
    let disconnected_canvas = osd_utils::flo_disconnected_pattern();

    loop {
        if shutting_down.load(Ordering::Acquire) {
            break;
        }

        // Drain pending recording commands without blocking. Honor only the
        // last one to avoid getting stuck if commands queue up.
        loop {
            match recording_rx.try_recv() {
                Ok(cmd) => apply_command(cmd, &mut active, &mut precapture),
                Err(mpsc::error::TryRecvError::Empty) => break,
                Err(mpsc::error::TryRecvError::Disconnected) => {
                    debug!("recording command channel closed; capture loop shutting down");
                    tear_down(active.take(), &mut sinks, &mut webcam);
                    return Ok(());
                }
            }
        }

        if matches!(rtp_targets_rx.has_changed(), Ok(true)) {
            let targets = rtp_targets_rx.borrow_and_update().clone();
            sinks.set_rtp_targets(targets);
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
        let live_canvas = osd_snapshot
            .as_ref()
            .filter(|s| s.received_at.elapsed() < OSD_STALE_AFTER)
            .map(|s| &s.canvas);
        // Live OSD takes precedence; the test pattern is the opt-in fallback
        // for verifying the render pipeline without flo running; otherwise
        // the disconnected notice makes a dead or dropped link visible
        // instead of leaving the corner blank. Note that this is the canvas
        // flo currently *has*, not necessarily the one that gets drawn: it is
        // also what the recording's SRT sidecar logs, and that must stay true
        // even while a tracking camera is displayed and nothing is stamped at
        // all.
        let active_canvas = live_canvas
            .or(test_pattern.as_ref())
            .or(Some(&disconnected_canvas));

        // A relayed tracking-camera frame is shown as it came in: the OSD
        // canvas is calibrated for the FPV camera, so stamping it onto an IR
        // frame would put the tracking marks in the wrong place. When the
        // selected relay has nothing fresh — it is starting up, flo died, the
        // camera stalled — fall back to the webcam rather than freezing on the
        // last frame. The sole exception is a bounded IR-to-IR handoff, which
        // holds the old IR frame until the new camera's first frame arrives so
        // the webcam does not flash between them. This is a display event, not
        // a reason to stop: the frame watchdog is fed by the webcam pull above
        // and only by that.
        let fresh_relayed = match display_source {
            DisplaySource::Webcam => None,
            source => relayed.peek_fresh(source),
        };
        let relayed_frame = relay_selector.select(display_source, fresh_relayed);
        match display_source {
            DisplaySource::Webcam => {}
            source => {
                match &relayed_frame {
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
            }
        }

        // Offer a frame to flo's browser preview. Always the webcam with the
        // OSD stamped on, whatever the live view is currently showing: the
        // point of the panel is to see the FPV camera from the browser.
        //
        // Skipped entirely while nobody is connected — this is the one output
        // that exists purely for someone looking at a web page — and stamped
        // before it is downscaled, because the OSD font is sized for a full
        // frame and would swamp a 640-wide one.
        if preview.wanted() && last_preview_at.is_none_or(|at| at.elapsed() >= PREVIEW_INTERVAL) {
            let mut stamped = recorded.clone();
            if let Some(canvas) = active_canvas
                && let Err(e) =
                    osd_overlay::stamp_canvas(&mut stamped, canvas, &fonts, &glyph_table)
            {
                warn!("OSD stamp failed for preview: {e}");
            }
            preview.send(PreviewFrame {
                image: downscale_rgb8(&stamped),
                timestamp,
            });
            last_preview_at = Some(Instant::now());
        }

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
        // Log the canvas flo is currently showing, whether or not it was
        // stamped onto the displayed frame. The recording is the webcam, so
        // its OSD record stays complete and replayable across a switch to a
        // tracking camera — the operator's telemetry history must not have
        // holes just because the live view was pointed elsewhere.
        //
        // Computed whether or not a recording is running: the pre-capture
        // buffer needs the same canvas for frames that may be written later.
        let canvas_for_recording = || {
            active_canvas
                .cloned()
                .unwrap_or_else(|| osd_utils::OsdCache::new(30, 16))
        };
        match active.as_mut() {
            Some(rec) => {
                if let Err(e) = rec.write(recorded_arc, timestamp, canvas_for_recording()) {
                    error!("error writing recording frame, stopping recording: {e:?}");
                    if let Some(rec) = active.take() {
                        rec.finish();
                    }
                }
            }
            // Not recording: hold the frame in case a pre-capture recording
            // starts and wants the window leading up to it.
            None => precapture.push(recorded_arc, timestamp, canvas_for_recording()),
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

    tear_down(active.take(), &mut sinks, &mut webcam);
    Ok(())
}

/// Flushes and releases everything the capture loop owns, announcing each step.
///
/// Every wait in here is unbounded — an MP4 `moov` write, and a V4L2 stream
/// teardown that `poll()`s a camera that may have stopped answering — and the
/// process cannot exit until all of them return. When it does not, the log has
/// to be able to say which one it stopped on.
fn tear_down(active: Option<ActiveRecording>, sinks: &mut Sinks, webcam: &mut WebcamSource) {
    if let Some(rec) = active {
        info!("finishing the active recording");
        rec.finish();
    }
    info!("finishing outputs");
    sinks.finish();
    info!("closing the webcam");
    webcam.close();
    info!("capture loop finished");
}

fn apply_command(
    cmd: RecordingCommand,
    active: &mut Option<ActiveRecording>,
    precapture: &mut PreCaptureBuffer,
) {
    match cmd {
        RecordingCommand::Start(start) => {
            if active.is_some() {
                warn!("StartRecording received while already recording; ignoring");
                return;
            }
            // Taken before the writer exists so the buffer is released either
            // way: on a normal start these frames are stale the moment
            // recording begins, and on a failed start holding them would pin
            // the memory until the next trigger.
            let buffered = if start.include_precapture {
                precapture.drain()
            } else {
                precapture.clear();
                Default::default()
            };
            let mut rec = match ActiveRecording::new(
                start.creation_time,
                &start.mp4_cfg,
                &start.data_dir,
                buffered.len(),
            ) {
                Ok(rec) => rec,
                Err(e) => {
                    error!("failed to start recording: {e:?}");
                    return;
                }
            };
            if !buffered.is_empty() {
                let secs = match (buffered.front(), buffered.back()) {
                    (Some(first), Some(last)) => secs_between(first.timestamp, last.timestamp),
                    _ => 0.0,
                };
                info!(
                    "writing {} pre-captured frames ({secs:.1}s) to recording",
                    buffered.len()
                );
                for b in buffered {
                    if let Err(e) = rec.write(b.frame, b.timestamp, b.osd) {
                        // Abandon the recording rather than keep a file whose
                        // pre-capture is half there: the operator asked for the
                        // window before the trigger, and a partial one is worse
                        // than an obvious failure.
                        error!("error writing pre-captured frame, stopping recording: {e:?}");
                        rec.finish();
                        return;
                    }
                }
            }
            *active = Some(rec);
        }
        RecordingCommand::Stop => {
            if let Some(rec) = active.take() {
                rec.finish();
            }
        }
        RecordingCommand::SetPreCaptureSeconds(secs) => {
            info!("setting pre-capture buffer to {secs} seconds");
            precapture.set_window_secs(secs);
        }
    }
}

#[cfg(test)]
mod tests {
    use machine_vision_formats::{owned::OImage, pixel_format::RGB8};

    use super::*;

    fn test_frame(value: u8) -> Frame {
        let image = OImage::<RGB8>::new(1, 1, 3, vec![value; 3]).unwrap();
        Arc::new(DynamicFrameOwned::from_static(image))
    }

    #[test]
    fn ir_to_ir_handoff_holds_the_old_frame_until_the_new_one_arrives() {
        let started = Instant::now();
        let main = test_frame(1);
        let secondary = test_frame(2);
        let mut selector = RelayFrameSelector::new(DisplaySource::StrandCamMain);

        let selected = selector
            .select_at(
                DisplaySource::StrandCamMain,
                Some(Arc::clone(&main)),
                started,
            )
            .unwrap();
        assert!(Arc::ptr_eq(&selected, &main));

        let held = selector
            .select_at(
                DisplaySource::StrandCamSecondary,
                None,
                started + Duration::from_millis(1),
            )
            .expect("the old IR frame bridges the handoff");
        assert!(Arc::ptr_eq(&held, &main));

        let selected = selector
            .select_at(
                DisplaySource::StrandCamSecondary,
                Some(Arc::clone(&secondary)),
                started + Duration::from_millis(2),
            )
            .unwrap();
        assert!(Arc::ptr_eq(&selected, &secondary));
    }

    #[test]
    fn an_ir_handoff_still_falls_back_after_the_stale_window() {
        let started = Instant::now();
        let main = test_frame(1);
        let mut selector = RelayFrameSelector::new(DisplaySource::StrandCamMain);
        selector.select_at(DisplaySource::StrandCamMain, Some(main), started);

        selector.select_at(
            DisplaySource::StrandCamSecondary,
            None,
            started + Duration::from_millis(1),
        );
        assert!(
            selector
                .select_at(
                    DisplaySource::StrandCamSecondary,
                    None,
                    started + RELAYED_FRAME_STALE_AFTER + Duration::from_millis(1),
                )
                .is_none(),
            "a missing secondary camera must eventually use webcam fallback"
        );
    }

    #[test]
    fn webcam_transitions_never_hold_an_ir_frame() {
        let started = Instant::now();
        let mut selector = RelayFrameSelector::new(DisplaySource::StrandCamMain);
        selector.select_at(DisplaySource::StrandCamMain, Some(test_frame(1)), started);

        assert!(
            selector
                .select_at(DisplaySource::Webcam, None, started)
                .is_none()
        );
        assert!(
            selector
                .select_at(DisplaySource::StrandCamSecondary, None, started)
                .is_none(),
            "switching from the webcam to IR must not resurrect an old IR frame"
        );
    }

    /// A fixed instant plus `secs`. Built from a UTC timestamp rather than a
    /// local wall-clock time so it is unambiguous in every timezone the tests
    /// might run in.
    fn at(secs: i64) -> Timestamp {
        chrono::DateTime::from_timestamp(1_786_000_000, 0)
            .unwrap()
            .with_timezone(&chrono::Local)
            + chrono::TimeDelta::seconds(secs)
    }

    fn empty_osd() -> osd_utils::OsdCache {
        osd_utils::OsdCache::new(30, 16)
    }

    #[test]
    fn a_disabled_buffer_holds_nothing() {
        let mut buf = PreCaptureBuffer::new();
        buf.push(test_frame(1), at(0), empty_osd());
        assert!(buf.drain().is_empty(), "the default window is zero");
    }

    #[test]
    fn frames_older_than_the_window_are_dropped() {
        let mut buf = PreCaptureBuffer::new();
        buf.set_window_secs(10.0);
        for secs in [0, 5, 9, 20] {
            buf.push(test_frame(secs as u8), at(secs), empty_osd());
        }

        // Measured back from the newest frame (t=20), so only t=20 is inside a
        // 10s window; the earlier three have aged out.
        let held = buf.drain();
        assert_eq!(held.len(), 1);
        assert_eq!(held.front().unwrap().timestamp, at(20));
    }

    #[test]
    fn draining_leaves_the_buffer_ready_to_refill() {
        let mut buf = PreCaptureBuffer::new();
        buf.set_window_secs(60.0);
        buf.push(test_frame(1), at(0), empty_osd());
        assert_eq!(buf.drain().len(), 1);

        assert!(buf.drain().is_empty(), "drain takes everything");
        buf.push(test_frame(2), at(1), empty_osd());
        assert_eq!(
            buf.drain().len(),
            1,
            "the window survives a drain, so buffering resumes"
        );
    }

    #[test]
    fn zeroing_the_window_frees_what_is_held() {
        let mut buf = PreCaptureBuffer::new();
        buf.set_window_secs(60.0);
        buf.push(test_frame(1), at(0), empty_osd());
        buf.set_window_secs(0.0);
        assert!(buf.drain().is_empty());
    }
}
