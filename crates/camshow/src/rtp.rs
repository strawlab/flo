//! The `--rtp-dest` output: an H.264/RTP/UDP stream of whatever the capture
//! loop selected for display, for an OpenIPC-style groundstation link.

use std::{net::SocketAddr, ops::ControlFlow, path::PathBuf};

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
    pub(crate) initial_bitrate_kbps: u32,
}

pub(crate) struct RtpSink {
    params: RtpStreamParams,
    streamer: Option<RtpH264Streamer>,
    /// The bitrate the stream should be running at, or `None` once flo disabled
    /// it. Kept so the stream can be rebuilt — see [`RtpSink::on_frame`] — with
    /// the right bitrate and without forgetting that it is meant to be off.
    bitrate_kbps: Option<u32>,
    /// Geometry of the last frame handed to the encoder, so a change can be
    /// noticed.
    frame_size: Option<(u32, u32)>,
}

impl RtpSink {
    /// Starts the stream at the CLI-requested bitrate. A failure here is
    /// logged and leaves the stream off rather than taking the process down:
    /// recording and the local display must survive a bad network or a
    /// missing encoder, and flo can turn the stream on later with
    /// `SetRtpBitrateKbps`.
    pub(crate) fn build(cfg: RtpSinkConfig) -> Self {
        let mut sink = Self {
            params: cfg.params,
            streamer: None,
            bitrate_kbps: Some(cfg.initial_bitrate_kbps),
            frame_size: None,
        };
        sink.start(cfg.initial_bitrate_kbps);
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

    /// Tears the stream down and, unless flo has disabled it, brings a fresh
    /// one up at the same bitrate.
    fn restart(&mut self) {
        if let Some(mut s) = self.streamer.take()
            && let Err(e) = s.finish()
        {
            error!("error finishing RTP streamer before restart: {e:?}");
        }
        if let Some(kbps) = self.bitrate_kbps {
            self.start(kbps);
        }
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
            let previous = self.frame_size.expect("needs_restart implies a previous size");
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

    fn set_rtp_bitrate_kbps(&mut self, kbps: Option<u32>) {
        self.bitrate_kbps = kbps;
        let Some(kbps) = kbps else {
            if let Some(mut s) = self.streamer.take()
                && let Err(e) = s.finish()
            {
                error!("error finishing RTP streamer: {e:?}");
            }
            info!("RTP stream disabled");
            return;
        };

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

    fn finish(&mut self) {
        if let Some(mut s) = self.streamer.take()
            && let Err(e) = s.finish()
        {
            error!("error finishing RTP streamer: {e:?}");
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

    #[test]
    fn set_rtp_bitrate_kbps_changes_disables_and_restarts_the_stream() {
        let mut sink = RtpSink::build(RtpSinkConfig {
            params: params(),
            initial_bitrate_kbps: 4000,
        });
        assert!(sink.streamer.is_some(), "starts with a streamer running");

        // Some(kbps) on an already-running stream goes through
        // `RtpH264Streamer::set_bitrate_kbps`, not a rebuild; it should still
        // be the same running stream afterward.
        sink.set_rtp_bitrate_kbps(Some(2000));
        assert!(sink.streamer.is_some(), "changing bitrate keeps it running");

        sink.set_rtp_bitrate_kbps(None);
        assert!(sink.streamer.is_none(), "None disables the stream");

        sink.set_rtp_bitrate_kbps(Some(2000));
        assert!(sink.streamer.is_some(), "Some restarts a disabled stream");

        sink.finish();
        assert!(sink.streamer.is_none(), "finish tears the stream down");
    }

    fn frame(width: u32, height: u32) -> Frame {
        let stride = width as usize * 3;
        let image =
            OImage::<RGB8>::new(width, height, stride, vec![0u8; stride * height as usize]).unwrap();
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
        let mut sink = RtpSink::build(RtpSinkConfig {
            params: params(),
            initial_bitrate_kbps: 4000,
        });

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
    fn a_size_change_does_not_resurrect_a_disabled_stream() {
        let mut sink = RtpSink::build(RtpSinkConfig {
            params: params(),
            initial_bitrate_kbps: 4000,
        });
        let _ = sink.on_frame(frame(64, 48), chrono::Local::now(), false);
        sink.set_rtp_bitrate_kbps(None);

        let _ = sink.on_frame(frame(1440, 1080), chrono::Local::now(), false);
        assert!(
            sink.streamer.is_none(),
            "flo asked for the stream to be off; a resolution switch must not turn it back on"
        );
    }
}
