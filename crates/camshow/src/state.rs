//! Types passed between the TCP server, the capture loop, and the outputs.

use std::sync::Arc;

use camshow_protocol::RecordingStart;
use osd_utils::OsdCache;
use strand_dynamic_frame::DynamicFrameOwned;

/// A captured frame handed to the outputs (OSD-stamped) or to the recorder
/// (clean).
pub(crate) type Frame = Arc<DynamicFrameOwned>;

/// Capture timestamp, in the local timezone (matches recording/SRT timestamps).
pub(crate) type Timestamp = chrono::DateTime<chrono::Local>;

/// Latest OSD canvas plus its arrival time. The timestamp lets the capture
/// loop blank the overlay if flo has gone silent.
#[derive(Clone)]
pub(crate) struct OsdSnapshot {
    pub(crate) canvas: OsdCache,
    pub(crate) received_at: std::time::Instant,
}

/// Recording command from the TCP server to the capture loop. Carries the
/// codec config and target directory chosen by flo.
#[derive(Debug)]
pub(crate) enum RecordingCommand {
    Start(Box<RecordingStart>),
    Stop,
}

/// One frame the GUI draws on the next repaint. Produced by the capture loop.
#[derive(Clone)]
pub(crate) struct DisplayFrame {
    pub(crate) frame: Frame,
    /// Whether the capture loop is currently writing this frame to disk.
    /// Drives the on-screen "REC" indicator.
    pub(crate) recording: bool,
}
