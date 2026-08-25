// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

use std::{
    collections::VecDeque,
    sync::{Arc, Mutex, mpsc::SyncSender},
    time::Duration,
};

use ffmpeg_writer::{FfmpegCodecArgs, FfmpegFrameSink, FfmpegOutput};
use h264_reader::{annexb::AnnexBReader, push::NalFragmentHandler};
use h264_rtp::AccessUnit;
use strand_dynamic_frame::DynamicFrame;

use crate::{Result, encoder::H264StreamEncoder};

/// H.264 NAL unit type for an Access Unit Delimiter (RFC 6184 / ITU-T H.264
/// §7.4.1.2.3). Used only to detect AU boundaries in ffmpeg's output; never
/// forwarded onto the wire (see [`Handler::nal_fragment`]).
const NAL_TYPE_AUD: u8 = 9;
/// Supplemental Enhancement Information. x264 emits a ~642-byte
/// user-data-unregistered SEI before every IDR that carries nothing an
/// SDP-less receiver needs; dropped to save real bytes on a constrained link.
const NAL_TYPE_SEI: u8 = 6;
/// IDR slice: presence of one marks an access unit as a keyframe.
const NAL_TYPE_IDR: u8 = 5;

/// Configuration specific to the ffmpeg sidecar encoder.
///
/// Currently carries no fields of its own: bitrate, frame rate and the
/// intra-frame period all come from the shared [`crate::StreamConfig`] fields.
#[derive(Debug, Clone, Default)]
pub struct FfmpegEncoderConfig {}

/// Encoder backed by an `ffmpeg` child process piping raw video in and reading
/// an H.264 elementary stream back out.
///
/// [`Self::set_bitrate_kbps`] and [`Self::request_keyframe`] both respawn the
/// child (there is no cheaper way to change bitrate or force an IDR on an
/// already-running ffmpeg process): the RTP session lives in a different
/// thread entirely (see `sender.rs`), so a respawn loses a few frames but
/// leaves the session (and thus the receiver's decoder state) untouched — the
/// receiver just sees a fresh IDR once the new child's first frame arrives.
pub(crate) struct FfmpegStreamEncoder {
    bitrate_kbps: u32,
    fps: f32,
    idr_interval_frames: u32,
    au_tx: SyncSender<AccessUnit>,
    /// Real per-frame presentation times, in submission order, popped by the
    /// stdout thread as each access unit completes. `-bf 0` (no B-frames)
    /// guarantees the encoder neither reorders nor drops frames, so this
    /// preserves the caller's real capture-time jitter instead of smoothing it
    /// into a fixed `frame_index / fps` clock.
    pts_queue: Arc<Mutex<VecDeque<Duration>>>,
    running: Option<Running>,
}

struct Running {
    /// Owns the ffmpeg child, the frame piping, the geometry invariant and the
    /// stderr draining -- see [`FfmpegFrameSink`], which `FfmpegWriter` in
    /// strand-braid is built on too.
    sink: FfmpegFrameSink,
    /// Parses the encoded elementary stream off the sink's stdout. Ours rather
    /// than the sink's, because what arrives on that pipe is this crate's
    /// business.
    stdout_thread: std::thread::JoinHandle<()>,
}

impl FfmpegStreamEncoder {
    pub(crate) fn new(
        _cfg: FfmpegEncoderConfig,
        bitrate_kbps: u32,
        fps: f32,
        idr_interval_frames: u32,
        au_tx: SyncSender<AccessUnit>,
    ) -> Result<Self> {
        Ok(Self {
            bitrate_kbps,
            fps,
            idr_interval_frames,
            au_tx,
            pts_queue: Arc::new(Mutex::new(VecDeque::new())),
            running: None,
        })
    }

    /// Spawn ffmpeg configured to read raw video of this frame's format and
    /// emit a zero-latency, SDP-less-receiver-friendly H.264 elementary stream.
    fn start(&mut self, frame: &DynamicFrame) -> Result<()> {
        // bufsize = 2 seconds' worth of frames at the target bitrate, a
        // conventional VBV buffer size for low-latency live encoding.
        let bufsize_bits =
            ((self.bitrate_kbps as f64 * 1000.0) / self.fps as f64 * 2.0).round() as u64;
        let bitrate_bps = self.bitrate_kbps.saturating_mul(1000);
        let codec_args = FfmpegCodecArgs {
            codec: Some("libx264".to_string()),
            max_bframes: Some(0),
            post_codec_args: Some(vec![
                ("-preset".into(), "ultrafast".into()),
                ("-tune".into(), "zerolatency".into()),
                ("-g".into(), self.idr_interval_frames.to_string()),
                ("-b:v".into(), bitrate_bps.to_string()),
                ("-maxrate".into(), bitrate_bps.to_string()),
                ("-bufsize".into(), bufsize_bits.to_string()),
                // repeat-headers is the Annex-B default (not disabled here),
                // so SPS/PPS precede every IDR with no extra flag; aud=1 gives
                // the stdout thread explicit AU boundaries to delimit on.
                ("-x264-params".into(), "slice-max-size=1200:aud=1".into()),
                ("-flush_packets".into(), "1".into()),
                ("-f".into(), "h264".into()),
            ]),
            ..Default::default()
        };

        // The sink decides how to hand a mono frame over. That matters for the
        // IR tracking cameras: some ffmpeg releases write 0 rather than 128
        // into the chroma planes when converting `gray` to a full-range
        // semi-planar format, which streams a solid green cast, and the sink
        // pipes NV12 with real chroma where it measures that happening. Getting
        // this for free is the point of sharing the sink -- the hand-rolled
        // spawn this replaced was safe only by accident, because `codec_args`
        // above leaves `pixfmt` at its default `yuv420p`.
        let mut sink = FfmpegFrameSink::new(
            frame,
            &codec_args,
            (self.fps.round().max(1.0) as usize, 1),
            FfmpegOutput::Stdout,
        )?;
        let stdout = sink
            .take_stdout()
            .expect("a sink writing to Stdout has a stdout");

        let stdout_thread = {
            let au_tx = self.au_tx.clone();
            let pts_queue = self.pts_queue.clone();
            std::thread::spawn(move || run_stdout_reader(stdout, au_tx, pts_queue))
        };

        self.running = Some(Running {
            sink,
            stdout_thread,
        });
        Ok(())
    }

    /// Kill the running child (if any), drain its threads, and clear any
    /// presentation times left over from frames it never got to encode
    /// before dying, so they cannot be misattributed to the next child's
    /// access units.
    fn stop(&mut self) -> Result<()> {
        if let Some(running) = self.running.take() {
            let Running {
                sink,
                stdout_thread,
            } = running;
            // Closing the sink signals EOF on ffmpeg's stdin and waits for it.
            // Not a failure worth propagating -- every caller here is
            // deliberately discarding this child -- but worth saying out loud,
            // since the sink can now report what ffmpeg actually complained
            // about.
            if let Err(e) = sink.close() {
                tracing::warn!("ffmpeg exited badly while being replaced: {e}");
            }
            // Its stdout hit EOF when the child exited, so this returns
            // promptly and we lose no already-encoded access unit.
            let _ = stdout_thread.join();
        }
        self.pts_queue
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
        Ok(())
    }
}

impl H264StreamEncoder for FfmpegStreamEncoder {
    fn submit(&mut self, frame: &DynamicFrame, pts: Duration) -> Result<()> {
        let running = match &mut self.running {
            Some(running) => running,
            None => {
                self.start(frame)?;
                self.running.as_mut().expect("just started")
            }
        };

        // Pushed before the frame, so an access unit cannot come back off the
        // stdout thread before its presentation time is queued. The sink
        // rejects a frame whose format or size does not match the running
        // child, so a mismatch cannot desynchronize this queue.
        self.pts_queue
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push_back(pts);
        // A failure here propagates out of the worker loop, which drops the
        // encoder and its queue, so there is nothing to unwind.
        running.sink.send(frame)?;
        Ok(())
    }

    fn set_bitrate_kbps(&mut self, kbps: u32) -> Result<()> {
        self.bitrate_kbps = kbps;
        self.stop()
    }

    fn request_keyframe(&mut self) -> Result<()> {
        // No cheap in-process equivalent to openh264's force_intra_frame for
        // an already-spawned ffmpeg child; respawning also forces a fresh IDR.
        self.stop()
    }

    fn finish(mut self: Box<Self>) -> Result<()> {
        self.stop()
    }
}

/// Groups the parsed NAL stream into [`AccessUnit`]s and pushes them to
/// `au_tx`, using AUD (type 9) NALs purely as boundary markers: an
/// [`AccessUnit`] is only known to be complete once the *next* one starts, so
/// this never emits early. SEI (type 6) is dropped; every other NAL type is
/// forwarded verbatim.
struct Handler {
    au_tx: SyncSender<AccessUnit>,
    pts_queue: Arc<Mutex<VecDeque<Duration>>>,
    /// Accumulates fragments of the NAL currently being parsed, across
    /// however many `nal_fragment` calls (and thus however many stdout reads)
    /// it takes to complete.
    current_nal: Vec<u8>,
    /// Completed NALs of the access unit not yet flushed.
    pending_nals: Vec<Vec<u8>>,
    pending_is_keyframe: bool,
}

impl Handler {
    fn flush(&mut self) {
        if self.pending_nals.is_empty() {
            return;
        }
        let nals = std::mem::take(&mut self.pending_nals);
        let is_keyframe = std::mem::take(&mut self.pending_is_keyframe);
        let pts = self
            .pts_queue
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .pop_front()
            .unwrap_or_default();
        let au = AccessUnit {
            nals,
            is_keyframe,
            pts,
        };
        // If the sender side is gone, the whole encoder is being torn down;
        // there is nothing more useful to do than stop forwarding.
        let _ = self.au_tx.send(au);
    }
}

impl NalFragmentHandler for Handler {
    fn nal_fragment(&mut self, bufs: &[&[u8]], is_end: bool) {
        for buf in bufs {
            self.current_nal.extend_from_slice(buf);
        }
        if !is_end {
            return;
        }
        let nal = std::mem::take(&mut self.current_nal);
        let Some(&header) = nal.first() else {
            return;
        };
        match header & 0x1F {
            NAL_TYPE_AUD => self.flush(),
            NAL_TYPE_SEI => {}
            nal_type => {
                if nal_type == NAL_TYPE_IDR {
                    self.pending_is_keyframe = true;
                }
                self.pending_nals.push(nal);
            }
        }
    }
}

fn run_stdout_reader(
    mut stdout: impl std::io::Read,
    au_tx: SyncSender<AccessUnit>,
    pts_queue: Arc<Mutex<VecDeque<Duration>>>,
) {
    let mut reader = AnnexBReader::for_fragment_handler(Handler {
        au_tx,
        pts_queue,
        current_nal: Vec::new(),
        pending_nals: Vec::new(),
        pending_is_keyframe: false,
    });
    let mut buf = [0u8; 64 * 1024];
    loop {
        match std::io::Read::read(&mut stdout, &mut buf) {
            Ok(0) => break,
            Ok(n) => reader.push(&buf[..n]),
            Err(_) => break,
        }
    }
    // Flush whatever access unit was still pending when the child exited.
    reader.into_fragment_handler().flush();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::sync_channel;

    /// Reassemble the access units this encoder produced into the Annex-B byte
    /// stream a decoder expects, putting back the start codes that
    /// [`AccessUnit`] carries implicitly.
    fn annex_b(au_rx: &std::sync::mpsc::Receiver<AccessUnit>) -> Vec<u8> {
        let mut stream = Vec::new();
        for au in au_rx.iter() {
            for nal in au.nals {
                stream.extend_from_slice(&[0, 0, 0, 1]);
                stream.extend_from_slice(&nal);
            }
        }
        stream
    }

    /// The IR tracking cameras are grayscale, and a `gray` frame handed to the
    /// wrong ffmpeg records chroma 0 -- a solid green cast -- instead of 128.
    /// The framing decision that avoids that lives in [`FfmpegFrameSink`], so
    /// this asserts we really do get it by streaming mono frames through the
    /// encoder as configured for flight and inspecting the chroma that comes
    /// back out. (Whether the *bad* framing is correctly detected is
    /// ffmpeg-writer's own test; this is the end of that pipe.)
    #[test]
    fn mono_frames_stream_with_neutral_chroma() {
        let (width, height) = (192u32, 144u32);
        let (au_tx, au_rx) = sync_channel(256);
        let mut encoder =
            FfmpegStreamEncoder::new(FfmpegEncoderConfig {}, 2_000, 25.0, 10, au_tx).unwrap();

        let mut buf = vec![0u8; (width * height) as usize];
        for y in 0..height as usize {
            for x in 0..width as usize {
                buf[y * width as usize + x] = (x * 255 / (width as usize - 1)) as u8;
            }
        }
        let frame = strand_dynamic_frame::DynamicFrameOwned::from_buf(
            width,
            height,
            width as usize,
            buf,
            machine_vision_formats::pixel_format::PixFmt::Mono8,
        )
        .unwrap();

        for i in 0..10u64 {
            encoder
                .submit(&frame.borrow(), Duration::from_millis(i * 40))
                .unwrap();
        }
        // Closes ffmpeg's stdin and joins the stdout thread, so every access
        // unit has been pushed by the time this returns; dropping the last
        // sender is what ends `annex_b`'s iteration.
        Box::new(encoder).finish().unwrap();

        let stream = annex_b(&au_rx);
        assert!(
            !stream.is_empty(),
            "the encoder produced no access units at all"
        );

        let tmp = tempfile::tempdir().unwrap();
        let stream_path = tmp.path().join("streamed.h264");
        std::fs::write(&stream_path, &stream).unwrap();

        // Decode to H.264's own planar 4:2:0 layout, so the decode adds no
        // conversion that could hide -- or invent -- a chroma error.
        let decoded = std::process::Command::new("ffmpeg")
            .args(["-v", "error", "-nostdin", "-i"])
            .arg(&stream_path)
            .args([
                "-frames:v",
                "1",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "yuv420p",
                "-",
            ])
            .output()
            .expect("running ffmpeg to decode the streamed access units");
        assert!(
            decoded.status.success(),
            "decode failed: {}",
            String::from_utf8_lossy(&decoded.stderr)
        );

        let luma_len = width as usize * height as usize;
        let chroma = &decoded.stdout[luma_len..];
        assert!(!chroma.is_empty(), "decoded frame carried no chroma");
        let mean = chroma.iter().map(|&c| c as u64).sum::<u64>() / chroma.len() as u64;
        assert!(
            mean.abs_diff(128) <= 4,
            "streamed mono video is not neutral (mean chroma {mean}, 0 would be green)"
        );
    }

    fn new_handler(
        au_tx: SyncSender<AccessUnit>,
        pts: impl IntoIterator<Item = Duration>,
    ) -> Handler {
        Handler {
            au_tx,
            pts_queue: Arc::new(Mutex::new(pts.into_iter().collect())),
            current_nal: Vec::new(),
            pending_nals: Vec::new(),
            pending_is_keyframe: false,
        }
    }

    fn feed(handler: &mut Handler, nal: &[u8]) {
        handler.nal_fragment(&[nal], true);
    }

    /// AUD (type 9) NALs delimit access units but are never themselves
    /// forwarded; SEI (type 6) is dropped; an IDR slice (type 5) anywhere in
    /// the AU marks it as a keyframe; every other NAL type is forwarded
    /// verbatim in arrival order.
    #[test]
    fn aud_boundaries_group_nals_and_drop_sei_and_aud() {
        let (au_tx, au_rx) = sync_channel(8);
        let mut handler = new_handler(au_tx, [Duration::from_millis(0), Duration::from_millis(33)]);

        // AU 1: (no preceding AUD -- this is the very first access unit) SPS,
        // PPS, a dropped SEI, then an IDR slice.
        feed(&mut handler, &[7, 1, 2, 3]); // SPS
        feed(&mut handler, &[8, 4, 5]); // PPS
        feed(&mut handler, &[6, 9, 9, 9]); // SEI, dropped
        feed(&mut handler, &[5, 0xAA, 0xBB]); // IDR slice

        // The AUD starting AU 2 flushes AU 1.
        feed(&mut handler, &[9, 0xF0]);
        feed(&mut handler, &[1, 0xCC]); // P slice

        // "Stream end": flush whatever was still pending.
        handler.flush();

        let au1 = au_rx.try_recv().unwrap();
        assert_eq!(
            au1.nals,
            vec![vec![7, 1, 2, 3], vec![8, 4, 5], vec![5, 0xAA, 0xBB]]
        );
        assert!(au1.is_keyframe);
        assert_eq!(au1.pts, Duration::from_millis(0));

        let au2 = au_rx.try_recv().unwrap();
        assert_eq!(au2.nals, vec![vec![1, 0xCC]]);
        assert!(!au2.is_keyframe);
        assert_eq!(au2.pts, Duration::from_millis(33));

        assert!(
            au_rx.try_recv().is_err(),
            "no third access unit was ever pushed"
        );
    }

    /// A NAL delivered across several `nal_fragment` calls (as happens when it
    /// straddles two stdout reads) must be reassembled whole before its type
    /// is inspected, not treated as multiple separate NALs.
    #[test]
    fn nal_fragments_spanning_multiple_calls_are_reassembled() {
        let (au_tx, au_rx) = sync_channel(8);
        let mut handler = new_handler(au_tx, [Duration::ZERO]);

        handler.nal_fragment(&[&[1]], false);
        handler.nal_fragment(&[&[0xAA, 0xBB]], false);
        handler.nal_fragment(&[&[0xCC]], true);
        handler.flush();

        let au = au_rx.try_recv().unwrap();
        assert_eq!(au.nals, vec![vec![1, 0xAA, 0xBB, 0xCC]]);
    }

    /// Flushing an access unit with no NALs accumulated (e.g. two AUDs in a
    /// row) must be a no-op: in particular it must not consume a PTS that a
    /// later, real access unit needs.
    #[test]
    fn flush_with_no_pending_nals_is_a_no_op() {
        let (au_tx, au_rx) = sync_channel(8);
        let mut handler = new_handler(au_tx, [Duration::from_millis(5)]);

        feed(&mut handler, &[9, 0xF0]); // AUD with nothing pending yet
        feed(&mut handler, &[1, 0x11]);
        handler.flush();

        let au = au_rx.try_recv().unwrap();
        assert_eq!(
            au.pts,
            Duration::from_millis(5),
            "the sole PTS must still be consumed by the real AU"
        );
        assert!(au_rx.try_recv().is_err());
    }
}
