//! Plug-in interfaces for composing extra subsystems into the flo binary.
//!
//! An [`Extension`] is a long-running tokio task that joins the supervisor
//! select loop. An [`OsdOverlay`] contributes draw operations to the OSD canvas
//! every render tick. Both are intentionally minimal: they exist so that
//! extensions can be linked in without `flo`'s lib needing to know about it.

use color_eyre::eyre::{self, Result};
use flo_core::{
    Broadway, FloControllerConfig, SaveToDiskMsg, osd_structs::LoadedFpvCameraOSDCalibration,
};
use osd_utils::OsdCache;
use tokio::sync::mpsc::UnboundedSender;
use tokio::task::JoinHandle;

use crate::{CentroidInputSender, ProcessingFeedback, osd::DroneStatus};

/// Context handed to [`Extension::spawn`] so the extension has access to
/// the shared runtime, the broadway, and the place to write recordings.
pub struct ExtensionContext<'a> {
    pub handle: &'a tokio::runtime::Handle,
    pub broadway: Broadway,
    pub saver_tx: UnboundedSender<SaveToDiskMsg>,
    pub data_dir: &'a camino::Utf8Path,
    pub config: &'a FloControllerConfig,
    /// Bounded, in-process ingress for camera observations. Extensions should
    /// send centroids here instead of serializing them onto FLO's legacy UDP
    /// listener.
    pub centroid_tx: CentroidInputSender,
    /// Latest FLO state relevant to camera processing. The value is updated
    /// after motor readouts and control-loop output changes.
    pub processing_feedback: tokio::sync::watch::Receiver<ProcessingFeedback>,
}

/// A long-running subsystem composed into the flo binary at link time.
pub trait Extension: Send + 'static {
    /// Stable identifier used for logs and per-extension recording files
    /// (e.g. "extension" → `extension.jsonl`).
    fn name(&self) -> &'static str;

    /// Optional companion OSD overlay. The default returns `None`. An
    /// extension that wants to render on the OSD constructs its overlay
    /// here using the supplied broadway and tokio handle; flo::run
    /// inserts the result into the overlay list passed to the OSD task.
    fn make_osd_overlay(
        &self,
        _broadway: &Broadway,
        _handle: &tokio::runtime::Handle,
    ) -> Option<Box<dyn OsdOverlay + Send + Sync>> {
        None
    }

    /// Spawn the extension's tokio task. The supervisor selects on the
    /// returned `JoinHandle`; if the task ends, the program exits the
    /// same way it would for any other subsystem failure.
    fn spawn(self: Box<Self>, ctx: ExtensionContext<'_>) -> Result<JoinHandle<Result<()>>>;
}

/// Validate the config with respect to the extensions.
///
/// Returns an error if any extension config section is present for which no
/// extension is registered. This would happen if a config file has a top-level
/// field for which there is no registered extension. We cannot use serde's
/// automatically derived unknown field detection (`deny_unknown_fields`)
/// because the config file is deserialized before we know which extensions are
/// registered, so we have to do it in this separate post-deserialization step.
pub fn validate_config(
    cfg: FloControllerConfig,
    extensions: &[Box<dyn Extension>],
) -> Result<FloControllerConfig, eyre::Error> {
    // Validation logic here

    // Fail on any extension config section that has no registered extension
    // claims, so a typo or a stale config aimed at a binary without the
    // extension fails immediately rather than silently doing nothing.
    let registered_names: std::collections::BTreeSet<&'static str> =
        extensions.iter().map(|e| e.name()).collect();
    for key in cfg.extensions.keys() {
        let name = key
            .as_str()
            .ok_or_else(|| eyre::eyre!("non-string key in extension config: {key:?}"))?;
        if !registered_names.contains(name) {
            eyre::bail!(
                "config file has `{name}:` section but no extension is registered for it; \
                    this binary was built without that extension"
            );
        }
    }
    Ok(cfg)
}

/// An OSD overlay contributes draw operations to the canvas each render
/// tick. The overlay is responsible for any background subscriptions it
/// needs to maintain its internal state (typically via a watch channel
/// updated by a tokio task spawned at construction time).
pub trait OsdOverlay: Send + Sync {
    fn draw(
        &self,
        canvas: &mut OsdCache,
        cal: &LoadedFpvCameraOSDCalibration,
        drone_status: &DroneStatus,
    );
}
