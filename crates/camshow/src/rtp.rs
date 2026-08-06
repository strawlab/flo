//! The `--rtp-dest` output: an H.264/RTP/UDP stream of whatever the capture
//! loop selected for display, for an OpenIPC-style groundstation link.

use std::{collections::BTreeMap, net::SocketAddr, ops::ControlFlow, path::PathBuf};

use flo_core::RtpTarget;
use rtp_h264_streamer::{
    EncoderKind, FfmpegEncoderConfig, OpenH264EncoderConfig, RtpH264Streamer, RtpSessionConfig,
    StreamConfig,
};
use tracing::{error, info};

use crate::{
    sink::FrameSink,
    state::{Frame, Timestamp},
};

#[derive(clap::ValueEnum, Clone, Copy, Debug)]
pub(crate) enum EncoderChoice {
    Ffmpeg,
    Openh264,
}

/// The CLI-derived pieces needed to build a fresh [`StreamConfig`] at a given
/// bitrate. `StreamConfig` itself can't be stored and reused: its
/// `EncoderKind` owns backend-specific, non-`Clone` state. Only needed to
/// (re)start the stream from scratch — [`RtpSink::set_rtp_bitrate_kbps`]
/// changes the bitrate of an already-running stream via
/// [`RtpH264Streamer::set_bitrate_kbps`] instead, which respawns just the
/// encoder and leaves the RTP session (and thus the receiver's synced state)
/// alone.
#[derive(Clone)]
pub(crate) struct RtpStreamParams {
    pub(crate) dest: SocketAddr,
    pub(crate) encoder: EncoderChoice,
    pub(crate) fps: f32,
    pub(crate) idr_interval_frames: u32,
    pub(crate) mtu: usize,
    pub(crate) queue_size: usize,
    pub(crate) dump_annexb: Option<PathBuf>,
}

impl RtpStreamParams {
    fn build(&self, bitrate_kbps: u32) -> StreamConfig {
        let encoder = match self.encoder {
            EncoderChoice::Ffmpeg => EncoderKind::Ffmpeg(FfmpegEncoderConfig::default()),
            EncoderChoice::Openh264 => EncoderKind::OpenH264(OpenH264EncoderConfig::default()),
        };
        StreamConfig {
            dest: self.dest,
            bitrate_kbps,
            fps: self.fps,
            idr_interval_frames: self.idr_interval_frames,
            encoder,
            // A new `RtpSessionConfig` (rather than one saved from an
            // earlier build) so a rebuilt stream gets a fresh, randomized
            // SSRC and initial sequence number, per RFC 3550 section 5.1 -
            // it's a distinct session, not a continuation of the torn-down
            // one.
            rtp: RtpSessionConfig {
                mtu: self.mtu,
                ..Default::default()
            },
            queue_size: self.queue_size,
            dump_annexb: self.dump_annexb.clone(),
        }
    }
}

/// Everything needed to start the RTP output.
pub(crate) struct RtpSinkConfig {
    pub(crate) params: RtpStreamParams,
    pub(crate) initial_targets: Vec<RtpTarget>,
}

pub(crate) struct RtpSink {
    params: RtpStreamParams,
    streamer: Option<RtpH264Streamer>,
    /// The bitrate the stream should be running at. Kept so the encoder can be
    /// rebuilt after a frame-size change.
    bitrate_kbps: u32,
    /// Geometry of the last frame handed to the encoder, so a change can be
    /// noticed.
    frame_size: Option<(u32, u32)>,
}

/// A set of independently encoded RTP streams, all receiving the displayed
/// frame. Each target needs its own RTP session and encoder, so a target is
/// added or removed as a whole stream rather than as another socket on one
/// session.
pub(crate) struct RtpSinks {
    params: RtpStreamParams,
    sinks: BTreeMap<SocketAddr, RtpSink>,
}

impl RtpSinks {
    pub(crate) fn build(cfg: RtpSinkConfig) -> Self {
        let RtpSinkConfig {
            params,
            initial_targets,
        } = cfg;
        let mut sinks = BTreeMap::new();
        for target in initial_targets {
            let mut target_params = params.clone();
            target_params.dest = target.addr;
            sinks.insert(
                target.addr,
                RtpSink::build(target_params, target.bitrate_kbps),
            );
        }
        Self { params, sinks }
    }

    fn set_targets(&mut self, targets: Vec<RtpTarget>) {
        let targets: BTreeMap<_, _> = targets
            .into_iter()
            .map(|target| (target.addr, target.bitrate_kbps))
            .collect();
        self.sinks.retain(|target, sink| {
            if targets.contains_key(target) {
                true
            } else {
                sink.finish();
                false
            }
        });
        for (target, bitrate_kbps) in targets {
            let sink = self.sinks.entry(target).or_insert_with(|| {
                let mut params = self.params.clone();
                params.dest = target;
                // One Annex-B dump cannot safely be shared by multiple
                // encoders. Preserve it for the initial CLI target only.
                params.dump_annexb = None;
                RtpSink::build(params, bitrate_kbps)
            });
            if sink.bitrate_kbps != bitrate_kbps {
                sink.set_rtp_bitrate_kbps(bitrate_kbps);
            }
        }
    }
}

impl RtpSink {
    /// Starts the stream at the CLI-requested bitrate. A failure here is
    /// logged and leaves the stream off rather than taking the process down:
    /// recording and the local display must survive a bad network or a
    /// missing encoder, and flo can update the target configuration later.
    fn build(params: RtpStreamParams, bitrate_kbps: u32) -> Self {
        let mut sink = Self {
            params,
            streamer: None,
            bitrate_kbps,
            frame_size: None,
        };
        sink.start(bitrate_kbps);
        sink
    }

    fn start(&mut self, kbps: u32) {
        self.streamer = match RtpH264Streamer::new(self.params.build(kbps)) {
            Ok(s) => {
                info!("RTP stream started at {kbps} kbps to {}", self.params.dest);
                Some(s)
            }
            Err(e) => {
                error!("failed to start RTP streamer at {kbps} kbps: {e:?}");
                None
            }
        };
    }

    /// Whether a frame of `size` needs the encoder rebuilt. The first frame
    /// never does: the encoder learns its geometry from whatever it is handed
    /// first, so there is nothing to rebuild until that changes.
    fn needs_restart(previous: Option<(u32, u32)>, size: (u32, u32)) -> bool {
        matches!(previous, Some(previous) if previous != size)
    }

    /// Tears the stream down and brings a fresh one up at the same bitrate.
    fn restart(&mut self) {
        if let Some(mut s) = self.streamer.take()
            && let Err(e) = s.finish()
        {
            error!("error finishing RTP streamer before restart: {e:?}");
        }
        self.start(self.bitrate_kbps);
    }

    fn set_rtp_bitrate_kbps(&mut self, kbps: u32) {
        self.bitrate_kbps = kbps;
        if let Some(s) = self.streamer.as_mut() {
            match s.set_bitrate_kbps(kbps) {
                Ok(()) => {
                    info!("RTP bitrate set to {kbps} kbps");
                    return;
                }
                Err(e) => error!("failed to change RTP bitrate, restarting stream: {e:?}"),
            }
            // The streamer's worker thread is gone; fall through and start a
            // fresh one rather than leaving the stream stuck at the last
            // bitrate it was able to apply.
            self.streamer = None;
        }

        self.start(kbps);
    }
}

impl FrameSink for RtpSink {
    fn on_frame(
        &mut self,
        frame: Frame,
        timestamp: Timestamp,
        _recording: bool,
    ) -> ControlFlow<()> {
        // The H.264 encoder is built for one frame size and rejects a change,
        // so switching the display between cameras of different resolutions
        // means rebuilding it. The receiver sees a new SSRC and has to resync;
        // that is accepted for now, and the alternative (scaling every relayed
        // frame to the webcam's geometry) is the fix if a groundstation turns
        // out not to cope.
        let size = {
            let borrowed = frame.borrow();
            (borrowed.width(), borrowed.height())
        };
        if Self::needs_restart(self.frame_size, size) {
            let previous = self
                .frame_size
                .expect("needs_restart implies a previous size");
            info!(
                "displayed frame size changed from {}x{} to {}x{}; restarting the RTP stream \
                 (the receiver will resync)",
                previous.0, previous.1, size.0, size.1
            );
            self.restart();
        }
        self.frame_size = Some(size);

        // The RTP stream carries the frame the operator is watching. Keep
        // recording to disk and displaying OSD state even if the stream
        // itself has died (e.g. its worker thread panicked) rather than
        // tearing down the whole process over it.
        if let Some(s) = self.streamer.as_mut()
            && let Err(e) = s.send(frame, timestamp)
        {
            error!("RTP streamer failed, dropping it: {e:?}");
            self.streamer = None;
        }
        ControlFlow::Continue(())
    }

    fn finish(&mut self) {
        if let Some(mut s) = self.streamer.take()
            && let Err(e) = s.finish()
        {
            error!("error finishing RTP streamer: {e:?}");
        }
    }
}

impl FrameSink for RtpSinks {
    fn on_frame(&mut self, frame: Frame, timestamp: Timestamp, recording: bool) -> ControlFlow<()> {
        let mut stop = false;
        for sink in self.sinks.values_mut() {
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

    fn set_rtp_targets(&mut self, targets: Vec<RtpTarget>) {
        self.set_targets(targets);
    }

    fn finish(&mut self) {
        for sink in self.sinks.values_mut() {
            sink.finish();
        }
    }
}

#[cfg(test)]
mod tests {
    use machine_vision_formats::{owned::OImage, pixel_format::RGB8};

    use super::*;

    /// An address nothing listens on, but valid for a UDP socket to send
    /// to — enough to exercise stream setup/teardown without a real peer.
    fn unused_dest() -> SocketAddr {
        let socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        socket.local_addr().unwrap()
    }

    fn params() -> RtpStreamParams {
        RtpStreamParams {
            dest: unused_dest(),
            encoder: EncoderChoice::Openh264,
            fps: 30.0,
            idr_interval_frames: 30,
            mtu: 1400,
            queue_size: 4,
            dump_annexb: None,
        }
    }

    fn target(addr: SocketAddr, bitrate_kbps: u32) -> RtpTarget {
        RtpTarget { addr, bitrate_kbps }
    }

    #[test]
    fn set_rtp_bitrate_kbps_changes_the_selected_stream() {
        let mut sink = RtpSink::build(params(), 4000);
        assert!(sink.streamer.is_some(), "starts with a streamer running");

        // An already-running stream goes through
        // `RtpH264Streamer::set_bitrate_kbps`, not a rebuild.
        sink.set_rtp_bitrate_kbps(2000);
        assert!(sink.streamer.is_some(), "changing bitrate keeps it running");
        assert_eq!(sink.bitrate_kbps, 2000);

        sink.finish();
        assert!(sink.streamer.is_none(), "finish tears the stream down");
    }

    fn frame(width: u32, height: u32) -> Frame {
        let stride = width as usize * 3;
        let image = OImage::<RGB8>::new(width, height, stride, vec![0u8; stride * height as usize])
            .unwrap();
        std::sync::Arc::new(strand_dynamic_frame::DynamicFrameOwned::from_static(image))
    }

    /// The encoder rejects a format or size change outright, so a switch
    /// between cameras of different resolutions must be noticed and the
    /// encoder rebuilt — otherwise the stream dies on the first frame from the
    /// newly-selected camera.
    #[test]
    fn only_a_changed_frame_size_asks_for_a_rebuild() {
        assert!(
            !RtpSink::needs_restart(None, (64, 48)),
            "the first frame is what the encoder is built from; nothing to rebuild"
        );
        assert!(
            !RtpSink::needs_restart(Some((64, 48)), (64, 48)),
            "an unchanged size must not disturb a running stream"
        );
        assert!(RtpSink::needs_restart(Some((64, 48)), (1440, 1080)));
        assert!(
            RtpSink::needs_restart(Some((1440, 1080)), (1080, 1440)),
            "a transposed size is still a different size"
        );
    }

    #[test]
    fn a_frame_size_change_leaves_the_stream_running() {
        let mut sink = RtpSink::build(params(), 4000);

        let _ = sink.on_frame(frame(64, 48), chrono::Local::now(), false);
        assert_eq!(sink.frame_size, Some((64, 48)));

        let _ = sink.on_frame(frame(1440, 1080), chrono::Local::now(), false);
        assert!(
            sink.streamer.is_some(),
            "the rebuild leaves a stream running, not a dead one"
        );
        assert_eq!(sink.frame_size, Some((1440, 1080)));

        sink.finish();
    }

    #[test]
    fn target_set_adds_and_removes_independent_streams() {
        let first = unused_dest();
        let second = unused_dest();
        let mut sinks = RtpSinks::build(RtpSinkConfig {
            params: params(),
            initial_targets: Vec::new(),
        });
        assert!(sinks.sinks.is_empty());

        sinks.set_targets(vec![target(first, 4000), target(second, 2500)]);
        assert_eq!(sinks.sinks.len(), 2);
        assert!(sinks.sinks.contains_key(&second));
        assert_eq!(sinks.sinks[&first].bitrate_kbps, 4000);
        assert_eq!(sinks.sinks[&second].bitrate_kbps, 2500);

        sinks.set_targets(vec![target(first, 1200), target(second, 2500)]);
        assert_eq!(sinks.sinks[&first].bitrate_kbps, 1200);
        assert_eq!(sinks.sinks[&second].bitrate_kbps, 2500);

        sinks.set_targets(vec![target(second, 2500)]);
        assert_eq!(sinks.sinks.len(), 1);
        assert!(sinks.sinks.contains_key(&second));
        sinks.finish();
    }
}
