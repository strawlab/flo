//! Shared types passed between the TCP server, camera, and GUI tasks.

use std::sync::Arc;

use camshow_protocol::RecordingStart;
use osd_utils::OsdCache;
use strand_dynamic_frame::DynamicFrameOwned;

/// One frame the GUI draws on the next repaint. Produced by the camera thread.
#[derive(Clone)]
pub(crate) struct DisplayFrame {
    pub(crate) frame: Arc<DynamicFrameOwned>,
    /// Whether the camera thread is currently writing this frame to disk.
    /// Drives the on-screen "REC" indicator.
    pub(crate) recording: bool,
}

/// Latest OSD canvas plus its arrival time. The timestamp lets the camera
/// thread blank the overlay if flo has gone silent.
#[derive(Clone)]
pub(crate) struct OsdSnapshot {
    pub(crate) canvas: OsdCache,
    pub(crate) received_at: std::time::Instant,
}

/// Recording command from the TCP server to the camera thread. Carries the
/// codec config and target directory chosen by flo.
#[derive(Debug)]
pub(crate) enum RecordingCommand {
    Start(Box<RecordingStart>),
    Stop,
}
