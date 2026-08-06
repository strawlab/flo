//! Where each displayed frame goes, and the fan-out that lets more than one
//! output run at once.

use std::ops::ControlFlow;

use eyre::Result;

use crate::{
    gui::{GuiSink, GuiSinkConfig},
    rtp::{RtpSinkConfig, RtpSinks},
    state::{Frame, Timestamp},
};

/// One output for OSD-stamped frames: the local display, the RTP stream, or
/// (via [`Sinks`]) any combination of them.
pub(crate) trait FrameSink: Send {
    /// Called once per captured frame, after the OSD canvas (if any) has been
    /// stamped onto it. `recording` reports whether a clean recording is
    /// currently being written to disk (e.g. to drive an on-screen "REC"
    /// indicator). Returning [`ControlFlow::Break`] stops the capture loop
    /// entirely.
    fn on_frame(&mut self, frame: Frame, timestamp: Timestamp, recording: bool) -> ControlFlow<()>;

    /// Replace the set of H.264/RTP stream configurations. No-op for non-RTP
    /// outputs.
    fn set_rtp_targets(&mut self, _targets: Vec<flo_core::RtpTarget>) {}

    /// Called once, after the capture loop exits, to flush/shut down.
    fn finish(&mut self);
}

/// Which outputs to build, from the CLI. `None` means that output is disabled
/// for this run.
pub(crate) struct SinkConfig {
    pub(crate) gui: Option<GuiSinkConfig>,
    pub(crate) rtp: Option<RtpSinkConfig>,
}

/// The enabled outputs. Every frame goes to all of them; an empty set is a
/// valid, if quiet, configuration (recording to disk still works).
pub(crate) struct Sinks {
    sinks: Vec<Box<dyn FrameSink>>,
    on_shutdown: Option<Box<dyn FnOnce() + Send>>,
}

impl Sinks {
    /// Builds the enabled outputs. Runs on the capture thread: with the GUI
    /// enabled this blocks until eframe hands over its context.
    pub(crate) fn build(cfg: SinkConfig) -> Result<Self> {
        let mut sinks: Vec<Box<dyn FrameSink>> = Vec::new();
        let mut on_shutdown = None;
        if let Some(gui_cfg) = cfg.gui {
            let gui = GuiSink::build(gui_cfg)?;
            on_shutdown = Some(gui.close_viewport_hook());
            sinks.push(Box::new(gui));
        }
        if let Some(rtp_cfg) = cfg.rtp {
            sinks.push(Box::new(RtpSinks::build(rtp_cfg)));
        }
        Ok(Self { sinks, on_shutdown })
    }

    /// Takes the hook to run from the shutdown watcher as soon as shutdown is
    /// requested. The GUI needs it to close its viewport (and so let eframe's
    /// main loop return); no other output has anything to do.
    pub(crate) fn take_on_shutdown(&mut self) -> Option<Box<dyn FnOnce() + Send>> {
        self.on_shutdown.take()
    }
}

impl FrameSink for Sinks {
    fn on_frame(&mut self, frame: Frame, timestamp: Timestamp, recording: bool) -> ControlFlow<()> {
        // Every output sees the frame, even if an earlier one asked to stop:
        // one output going away is not a reason to skip the others on the way
        // out. Cloning `frame` is an `Arc` bump, not a copy of the pixels.
        let mut stop = false;
        for sink in self.sinks.iter_mut() {
            stop |= sink
                .on_frame(frame.clone(), timestamp, recording)
                .is_break();
        }
        if stop {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    }

    fn set_rtp_targets(&mut self, targets: Vec<flo_core::RtpTarget>) {
        for sink in self.sinks.iter_mut() {
            sink.set_rtp_targets(targets.clone());
        }
    }

    fn finish(&mut self) {
        for sink in self.sinks.iter_mut() {
            sink.finish();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use machine_vision_formats::{owned::OImage, pixel_format::RGB8};
    use strand_dynamic_frame::DynamicFrameOwned;

    use super::*;

    #[derive(Default)]
    struct Observed {
        frames: usize,
        targets: Vec<Vec<flo_core::RtpTarget>>,
        finished: bool,
    }

    /// Stands in for a real output: records what it was handed, and can be
    /// told to ask for a stop after a given number of frames.
    struct TestSink {
        observed: Arc<Mutex<Observed>>,
        stop_after: Option<usize>,
    }

    impl TestSink {
        fn new(stop_after: Option<usize>) -> (Self, Arc<Mutex<Observed>>) {
            let observed = Arc::new(Mutex::new(Observed::default()));
            (
                Self {
                    observed: Arc::clone(&observed),
                    stop_after,
                },
                observed,
            )
        }
    }

    impl FrameSink for TestSink {
        fn on_frame(&mut self, _f: Frame, _t: Timestamp, _rec: bool) -> ControlFlow<()> {
            let mut observed = self.observed.lock().unwrap();
            observed.frames += 1;
            match self.stop_after {
                Some(n) if observed.frames > n => ControlFlow::Break(()),
                _ => ControlFlow::Continue(()),
            }
        }

        fn set_rtp_targets(&mut self, targets: Vec<flo_core::RtpTarget>) {
            self.observed.lock().unwrap().targets.push(targets);
        }

        fn finish(&mut self) {
            self.observed.lock().unwrap().finished = true;
        }
    }

    fn test_frame() -> Frame {
        let image = OImage::<RGB8>::new(2, 2, 6, vec![0u8; 12]).unwrap();
        Arc::new(DynamicFrameOwned::from_static(image))
    }

    fn sinks_of(sinks: Vec<Box<dyn FrameSink>>) -> Sinks {
        Sinks {
            sinks,
            on_shutdown: None,
        }
    }

    #[test]
    fn frames_targets_and_finish_reach_every_sink() {
        let (a, a_seen) = TestSink::new(None);
        let (b, b_seen) = TestSink::new(None);
        let mut sinks = sinks_of(vec![Box::new(a), Box::new(b)]);

        assert!(
            sinks
                .on_frame(test_frame(), chrono::Local::now(), false)
                .is_continue()
        );
        let targets = vec![flo_core::RtpTarget {
            addr: "127.0.0.1:5600".parse().unwrap(),
            bitrate_kbps: 1500,
        }];
        sinks.set_rtp_targets(targets.clone());
        sinks.finish();

        for seen in [a_seen, b_seen] {
            let seen = seen.lock().unwrap();
            assert_eq!(seen.frames, 1);
            assert_eq!(seen.targets, vec![targets.clone()]);
            assert!(seen.finished);
        }
    }

    #[test]
    fn one_sink_asking_to_stop_stops_the_loop_without_skipping_the_others() {
        let (a, a_seen) = TestSink::new(Some(0));
        let (b, b_seen) = TestSink::new(None);
        let mut sinks = sinks_of(vec![Box::new(a), Box::new(b)]);

        assert!(
            sinks
                .on_frame(test_frame(), chrono::Local::now(), false)
                .is_break(),
            "a Break from any sink stops the capture loop"
        );
        assert_eq!(a_seen.lock().unwrap().frames, 1);
        assert_eq!(
            b_seen.lock().unwrap().frames,
            1,
            "the sink after the stopping one still got the frame"
        );
    }

    #[test]
    fn no_sinks_is_a_valid_configuration() {
        let mut sinks = sinks_of(Vec::new());
        assert!(
            sinks
                .on_frame(test_frame(), chrono::Local::now(), true)
                .is_continue(),
            "with no outputs the capture loop keeps running (for recording)"
        );
        sinks.finish();
    }
}
