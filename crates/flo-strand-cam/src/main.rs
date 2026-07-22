//! The composed FLO and Strand Camera application.
//!
//! This binary is the composition root for camera acquisition and FLO. It
//! deliberately disables FLO's legacy UDP centroid listener and carries ImOps
//! detections through bounded Tokio channels instead.

use color_eyre::eyre::{Context, Result, eyre};
use serde::Deserialize;
use strand_cam::imops_processor::{
    ImOpsDetection, ImOpsHostConfiguration, ImOpsHostOptions, ImOpsProcessorConfig,
};
use strand_cam::{StandaloneArgs, StandaloneOrBraid, StrandCamArgs, TimestampSource};
use tokio::sync::{mpsc, oneshot};

const EXTENSION_NAME: &str = "flo-strand-cam";
const CAMERA_DETECTION_QUEUE_CAPACITY: usize = 64;

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExtensionConfig {
    backend: CameraBackend,
    #[serde(default)]
    camera_name: Option<String>,
    #[serde(default = "default_http_address")]
    http_address: String,
    imops: ImOpsConfig,
}

fn default_http_address() -> String {
    "127.0.0.1:3440".to_owned()
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum CameraBackend {
    Pylon,
    Vimba,
    Webcam,
    Sim,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
struct ImOpsConfig {
    #[serde(default = "default_imops_enabled")]
    enabled: bool,
    #[serde(default = "default_threshold")]
    threshold: u8,
    center_x: u32,
    center_y: u32,
}

fn default_imops_enabled() -> bool {
    true
}

fn default_threshold() -> u8 {
    200
}

struct StrandCamExtension;

impl flo::Extension for StrandCamExtension {
    fn name(&self) -> &'static str {
        EXTENSION_NAME
    }

    fn spawn(
        self: Box<Self>,
        ctx: flo::ExtensionContext<'_>,
    ) -> Result<tokio::task::JoinHandle<Result<()>>> {
        let config = extension_config(ctx.config)?;
        let centroid_tx = ctx.centroid_tx.clone();
        // Keep this receiver with the camera task. The first integration does
        // not alter detection from FLO state yet, but this reserves the
        // feedback path for time-aware processing without a later transport
        // redesign.
        let processing_feedback = ctx.processing_feedback.clone();
        let mut extension_shutdown_rx = ctx.shutdown_rx.clone();
        let data_dir = ctx.data_dir.as_std_path().to_owned();

        // Strand Camera's acquisition future is intentionally `!Send`: it
        // holds vendor camera objects and other thread-affine state. FLO runs
        // extensions inside its caller-owned LocalSet, so keep this task on
        // that same runtime rather than creating another runtime or process.
        Ok(tokio::task::spawn_local(async move {
            let _processing_feedback = processing_feedback;
            let (detection_tx, mut detection_rx) = mpsc::channel(CAMERA_DETECTION_QUEUE_CAPACITY);
            // Retain the sender for the lifetime of the extension. Future FLO
            // policy can update this live control path without involving a
            // network endpoint or restarting camera acquisition.
            let (_imops_configuration_tx, imops_configuration_rx) =
                tokio::sync::watch::channel(ImOpsHostConfiguration {
                    enabled: config.imops.enabled,
                    processor: ImOpsProcessorConfig {
                        threshold: config.imops.threshold,
                        center_x: config.imops.center_x,
                        center_y: config.imops.center_y,
                    },
                });
            let imops = ImOpsHostOptions {
                configuration_rx: imops_configuration_rx,
                detection_tx,
            };
            let (shutdown_tx, shutdown_rx) = oneshot::channel();
            let mut camera_task = Box::pin(run_camera(config, data_dir, shutdown_rx, imops));

            loop {
                tokio::select! {
                    shutdown = extension_shutdown_rx.changed() => {
                        if shutdown.is_err() || *extension_shutdown_rx.borrow() {
                            let _ = shutdown_tx.send(());
                            camera_task.await?;
                            return Ok(());
                        }
                    }
                    camera = &mut camera_task => {
                        camera?;
                        return Err(eyre!("Strand Camera stopped unexpectedly"));
                    }
                    detection = detection_rx.recv() => {
                        let Some(detection) = detection else {
                            return Err(eyre!("Strand Camera ImOps detection channel closed"));
                        };
                        let Some(centroid) = detection_to_centroid(detection) else {
                            continue;
                        };
                        match centroid_tx.try_send(centroid) {
                            Ok(()) => {}
                            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                                tracing::warn!("dropping ImOps detection because FLO's centroid queue is full");
                            }
                            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                                return Err(eyre!("FLO centroid input channel closed"));
                            }
                        }
                    }
                }
            }
        }))
    }
}

fn extension_config(config: &flo_core::FloControllerConfig) -> Result<ExtensionConfig> {
    let value = config
        .extensions
        .get(serde_yaml::Value::String(EXTENSION_NAME.to_owned()))
        .ok_or_else(|| eyre!("missing required `{EXTENSION_NAME}:` config section"))?
        .clone();
    serde_yaml::from_value(value).wrap_err("parsing flo-strand-cam configuration")
}

fn detection_to_centroid(detection: ImOpsDetection) -> Option<flo_core::MomentCentroid> {
    let frame_number = match u32::try_from(detection.metadata.frame_number) {
        Ok(frame_number) => frame_number,
        Err(_) => {
            tracing::warn!(
                frame_number = detection.metadata.frame_number,
                "dropping ImOps detection because its frame number does not fit FLO's u32 input"
            );
            return None;
        }
    };
    let timestamp_source = match detection.metadata.timestamp_source {
        TimestampSource::BraidTrigger => flo_core::TimestampSource::BraidTrigger,
        TimestampSource::HostAcquiredTimestamp => flo_core::TimestampSource::HostAcquiredTimestamp,
    };
    Some(flo_core::MomentCentroid {
        schema_version: 2,
        framenumber: frame_number,
        timestamp_source,
        timestamp: detection.metadata.timestamp,
        mu00: f64::from(detection.mu00),
        mu01: f64::from(detection.mu01),
        mu10: f64::from(detection.mu10),
        center_x: detection.center_x,
        center_y: detection.center_y,
        cam_name: detection.metadata.camera_name,
    })
}

async fn run_camera(
    config: ExtensionConfig,
    data_dir: std::path::PathBuf,
    shutdown_rx: oneshot::Receiver<()>,
    imops: ImOpsHostOptions,
) -> Result<()> {
    let args = strand_args(&config, data_dir);
    match config.backend {
        CameraBackend::Pylon => {
            let module: &'static ci2_pylon::WrappedModule =
                Box::leak(Box::new(ci2_pylon::new_module()?));
            let guard = ci2_pylon::make_singleton_guard(&module)?;
            strand_cam::run_strand_cam_app_async_with_host_options(
                ci2_async::into_threaded_async(module, &guard),
                args,
                EXTENSION_NAME,
                shutdown_rx,
                Some(imops),
            )
            .await?;
        }
        CameraBackend::Vimba => {
            let module: &'static ci2_vimba::WrappedModule =
                Box::leak(Box::new(ci2_vimba::new_module()?));
            let guard = ci2_vimba::make_singleton_guard(&module)?;
            strand_cam::run_strand_cam_app_async_with_host_options(
                ci2_async::into_threaded_async(module, &guard),
                args,
                EXTENSION_NAME,
                shutdown_rx,
                Some(imops),
            )
            .await?;
        }
        CameraBackend::Webcam => {
            let module: &'static ci2_webcam::WrappedModule =
                Box::leak(Box::new(ci2_webcam::new_module()?));
            let guard = ci2_webcam::make_singleton_guard(&module)?;
            strand_cam::run_strand_cam_app_async_with_host_options(
                ci2_async::into_threaded_async(module, &guard),
                args,
                EXTENSION_NAME,
                shutdown_rx,
                Some(imops),
            )
            .await?;
        }
        CameraBackend::Sim => {
            let module: &'static ci2_sim::WrappedModule =
                Box::leak(Box::new(ci2_sim::new_module()?));
            let guard = ci2_sim::make_singleton_guard(&module)?;
            strand_cam::run_strand_cam_app_async_with_host_options(
                ci2_async::into_threaded_async(module, &guard),
                args,
                EXTENSION_NAME,
                shutdown_rx,
                Some(imops),
            )
            .await?;
        }
    }
    Ok(())
}

fn strand_args(config: &ExtensionConfig, data_dir: std::path::PathBuf) -> StrandCamArgs {
    let mut args = StrandCamArgs::default();
    args.standalone_or_braid = StandaloneOrBraid::Standalone(StandaloneArgs {
        camera_name: config.camera_name.clone(),
        http_server_addr: Some(config.http_address.clone()),
        ..Default::default()
    });
    args.data_dir = Some(data_dir);
    args
}

fn main() -> Result<()> {
    flo::run(flo::AppOptions {
        extensions: vec![Box::new(StrandCamExtension)],
        enable_udp_listener: false,
        ..Default::default()
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::DateTime;
    use strand_cam::imops_processor::ImOpsFrameMetadata;

    fn detection(frame_number: u64) -> ImOpsDetection {
        ImOpsDetection {
            metadata: ImOpsFrameMetadata {
                frame_number,
                timestamp: DateTime::UNIX_EPOCH,
                timestamp_source: TimestampSource::HostAcquiredTimestamp,
                camera_name: "sim-camera".to_owned(),
            },
            mu00: 1.0,
            mu01: 2.0,
            mu10: 3.0,
            center_x: 4,
            center_y: 5,
            centroid: None,
        }
    }

    #[test]
    fn converts_detection_without_serialization() {
        let centroid = detection_to_centroid(detection(42)).unwrap();
        assert_eq!(centroid.framenumber, 42);
        assert_eq!(centroid.cam_name, "sim-camera");
        assert_eq!(centroid.mu10, 3.0);
    }

    #[test]
    fn drops_frame_numbers_flo_cannot_represent() {
        assert!(detection_to_centroid(detection(u64::from(u32::MAX) + 1)).is_none());
    }

    #[test]
    fn config_requires_image_center() {
        let parsed: std::result::Result<ImOpsConfig, _> = serde_yaml::from_str("threshold: 200");
        assert!(parsed.is_err());
    }
}
