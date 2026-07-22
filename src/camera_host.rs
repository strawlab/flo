//! First-class in-process camera hosting for FLO compositions.

use color_eyre::eyre::Result;
use flo_core::StrandCamRole;
use tokio::{sync::mpsc, task::JoinHandle};

use crate::{CentroidInputSender, ProcessingFeedback};

/// An in-process camera supplied by a [`CameraHost`].
///
/// FLO uses this identity to assign main/secondary centroid roles. Unlike
/// legacy Strand Camera configuration, this does not create an HTTP
/// reverse-proxy session or a BUI camera-proxy entry.
#[derive(Clone, Debug)]
pub struct CameraRegistration {
    pub role: StrandCamRole,
    pub name: String,
    /// Optional in-process replacement for Strand Cam's HTTP `callback`
    /// endpoint. FLO sends camera commands such as MP4 recording and
    /// post-trigger buffer control through this channel.
    pub control_tx: Option<mpsc::Sender<strand_cam_remote_control::CamArg>>,
    /// Frame rate used to convert FLO's pre-capture duration into this
    /// camera's post-trigger buffer frame count.
    pub expected_fps: Option<f64>,
}

/// Runtime resources made available to an in-process [`CameraHost`].
pub struct CameraHostContext<'a> {
    /// Bounded ingress for observations from all hosted cameras.
    pub centroid_tx: CentroidInputSender,
    /// Current controller state relevant to camera processing.
    pub processing_feedback: tokio::sync::watch::Receiver<ProcessingFeedback>,
    /// Set when FLO is stopping; camera hosts should shut down promptly.
    pub shutdown_rx: tokio::sync::watch::Receiver<bool>,
    /// Directory where the host may write recordings or other camera outputs.
    pub data_dir: &'a camino::Utf8Path,
}

/// An in-process owner of one or more cameras.
///
/// `registrations` runs during startup, before legacy Strand Cam sessions are
/// initialized. `spawn` runs on FLO's caller-owned `LocalSet`, so a host may
/// launch thread-affine camera work with `tokio::task::spawn_local`.
pub trait CameraHost: 'static {
    /// Describe every camera this host will provide.
    fn registrations(&self) -> Vec<CameraRegistration>;

    /// Start the host's supervised task.
    fn spawn(self: Box<Self>, ctx: CameraHostContext<'_>) -> Result<JoinHandle<Result<()>>>;
}
