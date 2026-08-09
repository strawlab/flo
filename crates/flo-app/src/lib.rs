//! The composed FLO and Strand Camera application.
//!
//! This crate is the composition root for camera acquisition and FLO. It
//! carries ImOps detections through bounded Tokio channels.

use color_eyre::eyre::{Context, Result, eyre};
use flo_imops::{ImOpsDetection, ImOpsFrameMetadata, ImOpsProcessor, ImOpsProcessorConfig};
use machine_vision_formats::{owned::OImage, pixel_format::Mono8};
use serde::Deserialize;
use std::{
    ffi::OsString,
    future::Future,
    path::{Path, PathBuf},
    sync::Mutex,
};
use strand_cam::{
    HostAnnotation, HostFrame, StandaloneArgs, StandaloneOrBraid, StrandCamArgs,
    StrandCamHostOptions,
};
use strand_cam_remote_control::{CamArg, CodecSelection, FfmpegCodecArgs, RecordingFrameRate};
use strand_http_video_streaming_types::Point;
use tokio::sync::{mpsc, oneshot, watch};

mod video_relay;
use video_relay::{VideoRelay, VideoRelayConfig};

const APP_NAME: &str = "flo";
/// The YAML section holding this application's camera-host settings. This is
/// deliberately *not* `APP_NAME`: the executable was renamed back to `flo` for
/// continuity with earlier releases, but renaming the config key would break
/// every deployed configuration file for no gain. Keep in sync with the
/// `#[serde(rename = ...)]` attributes that deserialize the section.
const CONFIG_SECTION: &str = "flo-strand-cam";
/// Cargo package version and the source revision that produced this binary.
pub const VERSION: &str = concat!(env!("CARGO_PKG_VERSION"), " (git ", env!("GIT_HASH"), ")");
/// The lossless Strand Camera-to-FLO queue only absorbs scheduling jitter. A
/// deeper queue would retain several full frames and add tracking latency.
const CAMERA_FRAME_SINK_QUEUE_CAPACITY: usize = 2;

/// Frames already received by FLO may be discarded for display. This queue is
/// deliberately shallow so camshow can never backpressure detection.
const CAMERA_VIDEO_RELAY_QUEUE_CAPACITY: usize = 2;

#[derive(Debug, Clone)]
struct CameraHostConfig {
    main: CameraConfig,
    secondary: Option<CameraConfig>,
    video_relay: VideoRelayConfig,
}

impl CameraHostConfig {
    fn resolve(raw: RawCameraHostConfig) -> Result<Self> {
        let main = raw.main.resolve(CameraRole::Main);
        let secondary = raw
            .secondary
            .map(|camera| camera.resolve(CameraRole::Secondary));

        if let Some(secondary) = &secondary {
            if main.camera_name == secondary.camera_name {
                return Err(eyre!(
                    "flo-strand-cam main and secondary camera_name values must differ"
                ));
            }
            if main.http_address == secondary.http_address {
                return Err(eyre!(
                    "flo-strand-cam main and secondary http_address values must differ"
                ));
            }
        }

        Ok(Self {
            main,
            secondary,
            video_relay: raw.video_relay,
        })
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawCameraHostConfig {
    main: RawCameraConfig,
    #[serde(default)]
    secondary: Option<RawCameraConfig>,
    /// Relaying camera frames to camshow for the operator's live view. Enabled
    /// by default; the section only needs to be present to change something.
    #[serde(default)]
    video_relay: VideoRelayConfig,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawCameraConfig {
    backend: CameraBackend,
    camera_name: String,
    #[serde(default)]
    http_address: Option<String>,
    #[serde(default)]
    expected_fps: Option<f64>,
    #[serde(default)]
    mp4_max_framerate: Option<RecordingFrameRate>,
    #[serde(default)]
    mp4_codec: Option<CodecSelectionConfig>,
    imops: ImOpsConfig,
    #[serde(default)]
    force_camera_sync_mode: Option<bool>,
}

impl RawCameraConfig {
    fn resolve(self, role: CameraRole) -> CameraConfig {
        CameraConfig {
            backend: self.backend,
            camera_name: self.camera_name,
            http_address: self
                .http_address
                .unwrap_or_else(|| role.default_http_address()),
            expected_fps: self.expected_fps,
            mp4_max_framerate: self.mp4_max_framerate,
            mp4_codec: self.mp4_codec.map(Into::into),
            imops: self.imops,
            force_camera_sync_mode: self.force_camera_sync_mode,
        }
    }
}

/// YAML-friendly form of Strand Camera's externally tagged codec enum.
///
/// Accept an ordinary mapping matching the former JSON `ToCamera` command,
/// without requiring YAML-specific enum-tag syntax.
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
enum CodecSelectionConfig {
    Named(NamedCodecSelection),
    Ffmpeg(FfmpegCodecSelection),
}

#[derive(Debug, Clone, Copy, Deserialize)]
enum NamedCodecSelection {
    H264Nvenc,
    H264OpenH264,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct FfmpegCodecSelection {
    #[serde(rename = "Ffmpeg")]
    ffmpeg: FfmpegCodecArgs,
}

impl From<CodecSelectionConfig> for CodecSelection {
    fn from(value: CodecSelectionConfig) -> Self {
        let codec = match value {
            CodecSelectionConfig::Named(NamedCodecSelection::H264Nvenc) => Self::H264Nvenc,
            CodecSelectionConfig::Named(NamedCodecSelection::H264OpenH264) => Self::H264OpenH264,
            CodecSelectionConfig::Ffmpeg(config) => Self::Ffmpeg(config.ffmpeg),
        };
        canonicalize_legacy_vaapi_codec(codec)
    }
}

/// Migrate older spellings of FLO's VAAPI preset.
///
/// `ffmpeg-writer` now supplies the full-range color metadata itself, and
/// VAAPI's hardware surface path must not force an output pixel format. Keep
/// this narrow so custom FFmpeg configurations remain untouched.
fn canonicalize_legacy_vaapi_codec(codec: CodecSelection) -> CodecSelection {
    let CodecSelection::Ffmpeg(args) = codec else {
        return codec;
    };

    let is_legacy_vaapi = args.device_args.as_deref()
        == Some(&[("-vaapi_device".to_owned(), "/dev/dri/renderD128".to_owned())])
        && args.pre_codec_args.as_deref()
            == Some(&[("-vf".to_owned(), "format=nv12,hwupload".to_owned())])
        && args.codec.as_deref() == Some("h264_vaapi")
        && args.post_codec_args.is_none()
        && matches!(args.pixfmt.as_deref(), None | Some("yuv420p"))
        && args.max_bframes.is_none();

    if !is_legacy_vaapi {
        return CodecSelection::Ffmpeg(args);
    }

    CodecSelection::Ffmpeg(strand_cam_remote_control::FfmpegCodecArgs {
        device_args: args.device_args,
        pre_codec_args: args.pre_codec_args,
        codec: args.codec,
        post_codec_args: Some(vec![("-color_range".to_owned(), "pc".to_owned())]),
        pixfmt: None,
        max_bframes: None,
    })
}

#[derive(Debug, Clone)]
struct CameraConfig {
    backend: CameraBackend,
    camera_name: String,
    http_address: String,
    expected_fps: Option<f64>,
    mp4_max_framerate: Option<RecordingFrameRate>,
    mp4_codec: Option<CodecSelection>,
    imops: ImOpsConfig,
    force_camera_sync_mode: Option<bool>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CameraRole {
    Main,
    Secondary,
}

impl CameraRole {
    fn default_http_address(self) -> String {
        match self {
            Self::Main => "127.0.0.1:3440",
            Self::Secondary => "127.0.0.1:3441",
        }
        .to_owned()
    }
}

#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
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

impl ImOpsConfig {
    /// The detector state FLO owns for one embedded camera. Strand Camera only
    /// receives frames; it no longer needs a configuration channel for ImOps.
    fn processor_config(self) -> ImOpsProcessorConfig {
        ImOpsProcessorConfig {
            threshold: self.threshold,
            center_x: self.center_x,
            center_y: self.center_y,
        }
    }
}

const CAMERA_CONTROL_QUEUE_CAPACITY: usize = 16;

struct CameraControlChannels {
    main_tx: mpsc::Sender<CamArg>,
    main_rx: Mutex<Option<mpsc::Receiver<CamArg>>>,
    secondary_tx: mpsc::Sender<CamArg>,
    secondary_rx: Mutex<Option<mpsc::Receiver<CamArg>>>,
    main_router_tx: Mutex<Option<oneshot::Sender<axum::Router>>>,
    main_router_rx: Mutex<Option<oneshot::Receiver<axum::Router>>>,
    secondary_router_tx: Mutex<Option<oneshot::Sender<axum::Router>>>,
    secondary_router_rx: Mutex<Option<oneshot::Receiver<axum::Router>>>,
}

impl CameraControlChannels {
    fn new() -> Self {
        let (main_tx, main_rx) = mpsc::channel(CAMERA_CONTROL_QUEUE_CAPACITY);
        let (secondary_tx, secondary_rx) = mpsc::channel(CAMERA_CONTROL_QUEUE_CAPACITY);
        let (main_router_tx, main_router_rx) = oneshot::channel();
        let (secondary_router_tx, secondary_router_rx) = oneshot::channel();
        Self {
            main_tx,
            main_rx: Mutex::new(Some(main_rx)),
            secondary_tx,
            secondary_rx: Mutex::new(Some(secondary_rx)),
            main_router_tx: Mutex::new(Some(main_router_tx)),
            main_router_rx: Mutex::new(Some(main_router_rx)),
            secondary_router_tx: Mutex::new(Some(secondary_router_tx)),
            secondary_router_rx: Mutex::new(Some(secondary_router_rx)),
        }
    }

    fn take_main_receiver(&self) -> mpsc::Receiver<CamArg> {
        self.main_rx
            .lock()
            .expect("main camera control receiver mutex is not poisoned")
            .take()
            .expect("main camera control receiver was already taken")
    }

    fn take_secondary_receiver(&self) -> mpsc::Receiver<CamArg> {
        self.secondary_rx
            .lock()
            .expect("secondary camera control receiver mutex is not poisoned")
            .take()
            .expect("secondary camera control receiver was already taken")
    }

    fn take_main_router_sender(&self) -> oneshot::Sender<axum::Router> {
        self.main_router_tx
            .lock()
            .expect("main camera router sender mutex is not poisoned")
            .take()
            .expect("main camera router sender was already taken")
    }

    fn take_secondary_router_sender(&self) -> oneshot::Sender<axum::Router> {
        self.secondary_router_tx
            .lock()
            .expect("secondary camera router sender mutex is not poisoned")
            .take()
            .expect("secondary camera router sender was already taken")
    }

    fn take_main_router_receiver(&self) -> oneshot::Receiver<axum::Router> {
        self.main_router_rx
            .lock()
            .expect("main camera router receiver mutex is not poisoned")
            .take()
            .expect("main camera router receiver was already taken")
    }

    fn take_secondary_router_receiver(&self) -> oneshot::Receiver<axum::Router> {
        self.secondary_router_rx
            .lock()
            .expect("secondary camera router receiver mutex is not poisoned")
            .take()
            .expect("secondary camera router receiver was already taken")
    }
}

struct StrandCamHost {
    config: CameraHostConfig,
    controls: CameraControlChannels,
}

impl StrandCamHost {
    fn camera_registrations(&self) -> Vec<flo::CameraRegistration> {
        let mut registrations = vec![flo::CameraRegistration {
            role: flo_core::StrandCamRole::Main,
            name: self.config.main.camera_name.clone(),
            router_rx: Some(self.controls.take_main_router_receiver()),
            control_tx: Some(self.controls.main_tx.clone()),
            expected_fps: self.config.main.expected_fps,
        }];
        if let Some(secondary) = &self.config.secondary {
            registrations.push(flo::CameraRegistration {
                role: flo_core::StrandCamRole::Secondary,
                name: secondary.camera_name.clone(),
                router_rx: Some(self.controls.take_secondary_router_receiver()),
                control_tx: Some(self.controls.secondary_tx.clone()),
                expected_fps: secondary.expected_fps,
            });
        }
        registrations
    }

    fn spawn(
        self: Box<Self>,
        ctx: flo::CameraHostContext<'_>,
    ) -> Result<tokio::task::JoinHandle<Result<()>>> {
        let centroid_tx = ctx.centroid_tx.clone();
        let host_shutdown_rx = ctx.shutdown_rx.clone();
        let display_source = ctx.display_source.clone();
        let data_dir = ctx.data_dir.as_std_path().to_owned();

        // Strand Camera's acquisition future is intentionally `!Send`: it
        // holds vendor camera objects and other thread-affine state. FLO runs
        // camera hosts inside its caller-owned LocalSet, so keep this task on
        // that same runtime rather than creating another runtime or process.
        Ok(tokio::task::spawn_local(async move {
            // Every embedded camera gets a lossless sink because tracking
            // detection must see every processed frame. The optional relay is
            // downstream of FLO and is allowed to discard display frames.
            let video_relay_config = self.config.video_relay.clone();
            let video_relay_enabled = video_relay_config.enabled;
            let relay_channel = || {
                video_relay_enabled
                    .then(|| mpsc::channel(CAMERA_VIDEO_RELAY_QUEUE_CAPACITY))
                    .unzip()
            };
            let (main_relay_tx, main_relay_rx) = relay_channel();
            let (secondary_relay_tx, secondary_relay_rx) =
                if video_relay_enabled && self.config.secondary.is_some() {
                    relay_channel()
                } else {
                    (None, None)
                };
            let (main_frame_sink, main_frames) = mpsc::channel(CAMERA_FRAME_SINK_QUEUE_CAPACITY);
            let (secondary_frame_sink, secondary_frames) = if self.config.secondary.is_some() {
                let (tx, rx) = mpsc::channel(CAMERA_FRAME_SINK_QUEUE_CAPACITY);
                (Some(tx), Some(rx))
            } else {
                (None, None)
            };

            let main = CameraRuntime::new(
                self.config.main,
                centroid_tx.clone(),
                self.controls.main_tx.clone(),
                self.controls.take_main_receiver(),
                self.controls.take_main_router_sender(),
                main_frame_sink,
                main_frames,
                main_relay_tx,
            );
            let secondary = self.config.secondary.map(|config| {
                CameraRuntime::new(
                    config,
                    centroid_tx,
                    self.controls.secondary_tx.clone(),
                    self.controls.take_secondary_receiver(),
                    self.controls.take_secondary_router_sender(),
                    secondary_frame_sink
                        .expect("secondary frame sink exists with secondary config"),
                    secondary_frames
                        .expect("secondary frame receiver exists with secondary config"),
                    secondary_relay_tx,
                )
            });

            // Best-effort and separate from camera supervision: camshow not
            // running, or the video link failing, must never take a camera down.
            tokio::task::spawn_local(async move {
                let result = video_relay::run(VideoRelay {
                    config: video_relay_config,
                    main_tap: main_relay_rx,
                    secondary_tap: secondary_relay_rx,
                    display_source,
                })
                .await;
                if let Err(e) = result {
                    tracing::error!("video relay stopped: {e:?}");
                }
            });

            run_cameras(main, secondary, data_dir, host_shutdown_rx).await
        }))
    }
}

impl flo::CameraHost for StrandCamHost {
    fn registrations(&self) -> Vec<flo::CameraRegistration> {
        self.camera_registrations()
    }

    fn spawn(
        self: Box<Self>,
        ctx: flo::CameraHostContext<'_>,
    ) -> Result<tokio::task::JoinHandle<Result<()>>> {
        StrandCamHost::spawn(self, ctx)
    }
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
    Some(flo_core::MomentCentroid {
        schema_version: 2,
        framenumber: frame_number,
        timestamp_source: detection.metadata.timestamp_source,
        timestamp: detection.metadata.timestamp,
        mu00: f64::from(detection.mu00),
        mu01: f64::from(detection.mu01),
        mu10: f64::from(detection.mu10),
        center_x: detection.center_x,
        center_y: detection.center_y,
        cam_name: detection.metadata.camera_name,
    })
}

struct CameraRuntime {
    config: CameraConfig,
    host_options: StrandCamHostOptions,
    embedded_http: strand_cam::EmbeddedHttpOptions,
    /// Dropping a join handle detaches the worker. It exits after Strand Camera
    /// drops its frame sink, so joining it from the LocalSet would only add a
    /// shutdown wait with no additional cleanup.
    _imops_worker: std::thread::JoinHandle<()>,
}

impl CameraRuntime {
    #[expect(
        clippy::too_many_arguments,
        reason = "construction keeps each camera channel and frame path explicit"
    )]
    fn new(
        config: CameraConfig,
        centroid_tx: flo::CentroidInputSender,
        cam_args_tx: mpsc::Sender<CamArg>,
        cam_args_rx: mpsc::Receiver<CamArg>,
        router_tx: oneshot::Sender<axum::Router>,
        frame_sink: mpsc::Sender<HostFrame>,
        frames: mpsc::Receiver<HostFrame>,
        relay_tx: Option<mpsc::Sender<HostFrame>>,
    ) -> Self {
        send_initial_camera_args(&config, &cam_args_tx);
        let (annotation_tx, annotation_rx) = watch::channel(HostAnnotation::default());
        let imops_worker = spawn_imops_worker(
            config.camera_name.clone(),
            config.imops,
            frames,
            centroid_tx,
            relay_tx,
            annotation_tx,
        );
        Self {
            config,
            host_options: StrandCamHostOptions {
                cam_args_rx: Some(cam_args_rx),
                frame_sink: Some(frame_sink),
                annotation_rx: Some(annotation_rx),
            },
            embedded_http: strand_cam::EmbeddedHttpOptions { router_tx },
            _imops_worker: imops_worker,
        }
    }
}

fn send_initial_camera_args(config: &CameraConfig, cam_args_tx: &mpsc::Sender<CamArg>) {
    let initial_cam_args = [
        config
            .mp4_max_framerate
            .clone()
            .map(CamArg::SetMp4MaxFramerate),
        config.mp4_codec.clone().map(CamArg::SetMp4Codec),
    ];
    for arg in initial_cam_args.into_iter().flatten() {
        cam_args_tx
            .try_send(arg)
            .expect("initial camera commands fit the bounded control queue");
    }
}

fn spawn_imops_worker(
    camera_name: String,
    config: ImOpsConfig,
    mut frames: mpsc::Receiver<HostFrame>,
    centroid_tx: flo::CentroidInputSender,
    relay_tx: Option<mpsc::Sender<HostFrame>>,
    annotation_tx: watch::Sender<HostAnnotation>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name(format!("flo-imops-{camera_name}"))
        .spawn(move || {
            let processor = ImOpsProcessor::new(config.processor_config());
            while let Some(frame) = frames.blocking_recv() {
                if let Some(relay_tx) = &relay_tx {
                    // Display is best-effort. Detection has already received
                    // this frame, so a full relay queue is the intended place
                    // to drop it.
                    let _ = relay_tx.try_send(frame.clone());
                }

                let detection = if config.enabled {
                    process_host_frame(&processor, &camera_name, &frame)
                } else {
                    None
                };
                let _ = annotation_tx.send(HostAnnotation {
                    points: detection
                        .as_ref()
                        .and_then(|detection| detection.centroid)
                        .map(|centroid| Point {
                            x: centroid.x,
                            y: centroid.y,
                            theta: None,
                            area: None,
                        })
                        .into_iter()
                        .collect(),
                    frame_number: frame.frame_number,
                    timestamp: Some(frame.timestamp),
                });

                let Some(detection) = detection else {
                    continue;
                };
                let Some(centroid) = detection_to_centroid(detection) else {
                    continue;
                };
                match centroid_tx.try_send(centroid) {
                    Ok(()) => {}
                    Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                        tracing::warn!(
                            "dropping ImOps detection because FLO's centroid queue is full"
                        );
                    }
                    Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                        tracing::warn!(
                            "ending ImOps worker because FLO's centroid queue is closed"
                        );
                        return;
                    }
                }
            }
        })
        .expect("spawning FLO ImOps worker thread")
}

fn process_host_frame(
    processor: &ImOpsProcessor,
    camera_name: &str,
    frame: &HostFrame,
) -> Option<ImOpsDetection> {
    let image = frame.image.borrow();
    let Some(mono8) = image.as_static::<Mono8>() else {
        tracing::warn!(
            camera_name,
            pixel_format = ?image.pixel_format(),
            "ImOps only supports Mono8 frames"
        );
        return None;
    };
    let timestamp_source = match frame.timestamp_source {
        strand_cam::TimestampSource::BraidTrigger => flo_core::TimestampSource::BraidTrigger,
        strand_cam::TimestampSource::HostAcquiredTimestamp => {
            flo_core::TimestampSource::HostAcquiredTimestamp
        }
    };
    Some(processor.process(
        OImage::copy_from(&mono8),
        ImOpsFrameMetadata {
            frame_number: frame.frame_number,
            timestamp: frame.timestamp,
            timestamp_source,
            camera_name: camera_name.to_owned(),
        },
    ))
}

async fn run_cameras(
    main: CameraRuntime,
    secondary: Option<CameraRuntime>,
    data_dir: std::path::PathBuf,
    shutdown_rx: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    match secondary {
        None => run_one_camera(main, data_dir, shutdown_rx).await,
        Some(secondary) if main.config.backend == secondary.config.backend => {
            run_same_backend_pair(main, secondary, data_dir, shutdown_rx).await
        }
        Some(secondary) => run_independent_pair(main, secondary, data_dir, shutdown_rx).await,
    }
}

async fn run_one_camera(
    camera: CameraRuntime,
    data_dir: std::path::PathBuf,
    mut shutdown_rx: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    let (shutdown_tx, camera_shutdown_rx) = oneshot::channel();
    let mut camera_task = Box::pin(run_camera(camera, data_dir, camera_shutdown_rx));
    tokio::select! {
        shutdown = shutdown_rx.changed() => {
            if shutdown.is_err() || *shutdown_rx.borrow() {
                let _ = shutdown_tx.send(());
                camera_task.await?;
                Ok(())
            } else {
                Err(eyre!("FLO shutdown channel changed to an unsupported value"))
            }
        }
        result = &mut camera_task => {
            result?;
            Err(eyre!("main Strand Camera stopped unexpectedly"))
        }
    }
}

async fn run_independent_pair(
    main: CameraRuntime,
    secondary: CameraRuntime,
    data_dir: std::path::PathBuf,
    shutdown_rx: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    let (main_shutdown_tx, main_shutdown_rx) = oneshot::channel();
    let (secondary_shutdown_tx, secondary_shutdown_rx) = oneshot::channel();
    supervise_camera_pair(
        run_camera(main, data_dir.clone(), main_shutdown_rx),
        run_camera(secondary, data_dir, secondary_shutdown_rx),
        shutdown_rx,
        main_shutdown_tx,
        secondary_shutdown_tx,
    )
    .await
}

async fn run_same_backend_pair(
    main: CameraRuntime,
    secondary: CameraRuntime,
    data_dir: std::path::PathBuf,
    shutdown_rx: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    match main.config.backend {
        CameraBackend::Pylon => {
            let module: &'static ci2_pylon::WrappedModule =
                Box::leak(Box::new(ci2_pylon::new_module()?));
            let guard = ci2_pylon::make_singleton_guard(&module)?;
            run_shared_module_pair(module, guard, main, secondary, data_dir, shutdown_rx).await
        }
        CameraBackend::Vimba => {
            let module: &'static ci2_vimba::WrappedModule =
                Box::leak(Box::new(ci2_vimba::new_module()?));
            let guard = GuardUntilCleanShutdown::new(ci2_vimba::make_singleton_guard(&module)?);
            let result = run_shared_module_pair_with_guard(
                module,
                guard.get(),
                main,
                secondary,
                data_dir,
                shutdown_rx,
            )
            .await;
            guard.clean_shutdown();
            result
        }
        CameraBackend::Webcam => {
            let module: &'static ci2_webcam::WrappedModule =
                Box::leak(Box::new(ci2_webcam::new_module()?));
            let guard = ci2_webcam::make_singleton_guard(&module)?;
            run_shared_module_pair(module, guard, main, secondary, data_dir, shutdown_rx).await
        }
        CameraBackend::Sim => {
            let module: &'static ci2_sim::WrappedModule =
                Box::leak(Box::new(ci2_sim::new_module()?));
            let guard = ci2_sim::make_singleton_guard(&module)?;
            run_shared_module_pair(module, guard, main, secondary, data_dir, shutdown_rx).await
        }
    }
}

async fn run_shared_module_pair<M, C, G>(
    module: M,
    guard: G,
    main: CameraRuntime,
    secondary: CameraRuntime,
    data_dir: std::path::PathBuf,
    shutdown_rx: tokio::sync::watch::Receiver<bool>,
) -> Result<()>
where
    M: ci2::CameraModule<CameraType = C, Guard = G> + Copy,
    C: 'static + ci2::Camera + Send,
    G: Send,
{
    run_shared_module_pair_with_guard(module, &guard, main, secondary, data_dir, shutdown_rx).await
}

async fn run_shared_module_pair_with_guard<M, C, G>(
    module: M,
    guard: &G,
    main: CameraRuntime,
    secondary: CameraRuntime,
    data_dir: std::path::PathBuf,
    shutdown_rx: tokio::sync::watch::Receiver<bool>,
) -> Result<()>
where
    M: ci2::CameraModule<CameraType = C, Guard = G> + Copy,
    C: 'static + ci2::Camera + Send,
    G: Send,
{
    let (main_shutdown_tx, main_shutdown_rx) = oneshot::channel();
    let (secondary_shutdown_tx, secondary_shutdown_rx) = oneshot::channel();
    supervise_camera_pair(
        run_camera_with_module(module, guard, main, data_dir.clone(), main_shutdown_rx),
        run_camera_with_module(module, guard, secondary, data_dir, secondary_shutdown_rx),
        shutdown_rx,
        main_shutdown_tx,
        secondary_shutdown_tx,
    )
    .await
}

/// Vimba's termination guard calls `VmbShutdown` when dropped. On an ordinary
/// camera-host shutdown we drop it after both Strand Camera futures have
/// joined. If FLO forcibly aborts the host after its shutdown deadline, this
/// wrapper is dropped while those futures may still own acquisition threads;
/// in that case retain the SDK guard until process exit instead of shutting the
/// SDK down underneath them.
struct GuardUntilCleanShutdown<G>(Option<G>);

impl<G> GuardUntilCleanShutdown<G> {
    fn new(guard: G) -> Self {
        Self(Some(guard))
    }

    fn get(&self) -> &G {
        self.0
            .as_ref()
            .expect("Vimba guard is present while cameras run")
    }

    fn clean_shutdown(mut self) {
        drop(self.0.take());
    }
}

impl<G> Drop for GuardUntilCleanShutdown<G> {
    fn drop(&mut self) {
        if let Some(guard) = self.0.take() {
            tracing::warn!(
                "retaining Vimba SDK guard until process exit after forced camera-host cancellation"
            );
            std::mem::forget(guard);
        }
    }
}

async fn supervise_camera_pair<Main, Secondary>(
    main: Main,
    secondary: Secondary,
    mut shutdown_rx: tokio::sync::watch::Receiver<bool>,
    main_shutdown_tx: oneshot::Sender<()>,
    secondary_shutdown_tx: oneshot::Sender<()>,
) -> Result<()>
where
    Main: Future<Output = Result<()>>,
    Secondary: Future<Output = Result<()>>,
{
    let mut main = Box::pin(main);
    let mut secondary = Box::pin(secondary);
    let mut main_shutdown_tx = Some(main_shutdown_tx);
    let mut secondary_shutdown_tx = Some(secondary_shutdown_tx);
    tokio::select! {
        shutdown = shutdown_rx.changed() => {
            if shutdown.is_err() || *shutdown_rx.borrow() {
                let _ = main_shutdown_tx.take().expect("main shutdown sender is available").send(());
                let _ = secondary_shutdown_tx.take().expect("secondary shutdown sender is available").send(());
                main.await?;
                secondary.await?;
                Ok(())
            } else {
                Err(eyre!("FLO shutdown channel changed to an unsupported value"))
            }
        }
        result = &mut main => {
            let _ = secondary_shutdown_tx.take().expect("secondary shutdown sender is available").send(());
            let secondary_result = secondary.await;
            result?;
            secondary_result?;
            Err(eyre!("main Strand Camera stopped unexpectedly"))
        }
        result = &mut secondary => {
            let _ = main_shutdown_tx.take().expect("main shutdown sender is available").send(());
            let main_result = main.await;
            result?;
            main_result?;
            Err(eyre!("secondary Strand Camera stopped unexpectedly"))
        }
    }
}

async fn run_camera(
    camera: CameraRuntime,
    data_dir: std::path::PathBuf,
    shutdown_rx: oneshot::Receiver<()>,
) -> Result<()> {
    let CameraRuntime {
        config,
        host_options,
        embedded_http,
        ..
    } = camera;
    let args = strand_args(&config, data_dir);
    match config.backend {
        CameraBackend::Pylon => {
            let module: &'static ci2_pylon::WrappedModule =
                Box::leak(Box::new(ci2_pylon::new_module()?));
            let guard = ci2_pylon::make_singleton_guard(&module)?;
            strand_cam::run_strand_cam_app_async_with_host_options(
                ci2_async::into_threaded_async(module, &guard),
                args,
                APP_NAME,
                shutdown_rx,
                Some(host_options),
                Some(embedded_http),
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
                APP_NAME,
                shutdown_rx,
                Some(host_options),
                Some(embedded_http),
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
                APP_NAME,
                shutdown_rx,
                Some(host_options),
                Some(embedded_http),
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
                APP_NAME,
                shutdown_rx,
                Some(host_options),
                Some(embedded_http),
            )
            .await?;
        }
    }
    Ok(())
}

async fn run_camera_with_module<M, C, G>(
    module: M,
    guard: &G,
    camera: CameraRuntime,
    data_dir: std::path::PathBuf,
    shutdown_rx: oneshot::Receiver<()>,
) -> Result<()>
where
    M: ci2::CameraModule<CameraType = C, Guard = G>,
    C: 'static + ci2::Camera + Send,
    G: Send,
{
    let CameraRuntime {
        config,
        host_options,
        embedded_http,
        ..
    } = camera;
    strand_cam::run_strand_cam_app_async_with_host_options(
        ci2_async::into_threaded_async(module, guard),
        strand_args(&config, data_dir),
        APP_NAME,
        shutdown_rx,
        Some(host_options),
        Some(embedded_http),
    )
    .await?;
    Ok(())
}

fn strand_args(config: &CameraConfig, data_dir: std::path::PathBuf) -> StrandCamArgs {
    let mut args = StrandCamArgs::default();
    args.standalone_or_braid = StandaloneOrBraid::Standalone(StandaloneArgs {
        camera_name: Some(config.camera_name.clone()),
        http_server_addr: Some(config.http_address.clone()),
        force_camera_sync_mode: config.force_camera_sync_mode.unwrap_or(false),
        ..Default::default()
    });
    args.data_dir = Some(data_dir);
    // FLO detects for itself, in `spawn_imops_worker`, from the frames Strand
    // Camera hands to the host sink. Leaving Strand Camera's own detector
    // available would cost a second threshold-and-moments pass over every
    // frame, and would offer the operator a panel of ImOps controls in the
    // camera BUI that cannot affect the detection FLO is actually using.
    args.disable_imops = true;
    args
}

fn config_path_from_args(args: &[OsString]) -> Result<PathBuf> {
    let mut args = args.iter().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--config" {
            let path = args
                .next()
                .ok_or_else(|| eyre!("`flo` requires a path after `--config`"))?;
            return Ok(PathBuf::from(path));
        }
        if let Some(path) = arg.to_str().and_then(|arg| arg.strip_prefix("--config=")) {
            return Ok(PathBuf::from(path));
        }
    }
    Err(eyre!(
        "`flo` requires `--config <path>` so it can configure its embedded camera host"
    ))
}

/// The `camera_name` values a config's `flo-strand-cam` section registers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraNames {
    pub main: String,
    pub secondary: Option<String>,
}

/// Read the camera names out of a composed configuration file.
///
/// A centroid source composed into this binary has to attribute its
/// observations to the cameras FLO actually registered, or the controller sees
/// an unknown camera and never pairs a stereo observation. This is what lets
/// such a source take the names from the same config file rather than
/// duplicating them on the command line.
pub fn configured_camera_names(path: &Path) -> Result<CameraNames> {
    #[derive(Deserialize)]
    struct CameraSection {
        // Must match `CONFIG_SECTION`; `serde(rename)` requires a literal.
        #[serde(rename = "flo-strand-cam")]
        camera_host: RawCameraHostConfig,
    }

    let yaml = std::fs::read_to_string(path)
        .with_context(|| format!("opening configuration file {}", path.display()))?;
    let section: CameraSection = serde_yaml::from_str(&yaml)
        .with_context(|| format!("parsing flo-strand-cam configuration in {}", path.display()))?;
    let camera_host = CameraHostConfig::resolve(section.camera_host)?;
    Ok(CameraNames {
        main: camera_host.main.camera_name,
        secondary: camera_host.secondary.map(|camera| camera.camera_name),
    })
}

fn load_composed_config(
    path: &Path,
) -> Result<(CameraHostConfig, flo_core::FloControllerConfig, String)> {
    let yaml = std::fs::read_to_string(path)
        .with_context(|| format!("opening configuration file {}", path.display()))?;
    let (camera_host, flo_config) = parse_composed_config(&yaml)
        .with_context(|| format!("parsing YAML in configuration file {}", path.display()))?;
    Ok((camera_host, flo_config, yaml))
}

fn parse_composed_config(yaml: &str) -> Result<(CameraHostConfig, flo_core::FloControllerConfig)> {
    #[derive(Deserialize)]
    struct CameraSection {
        // Must match `CONFIG_SECTION`; `serde(rename)` requires a literal.
        #[serde(rename = "flo-strand-cam")]
        camera_host: RawCameraHostConfig,
    }

    // Parse the camera-host section directly from the original YAML. This
    // avoids routing the primary application's configuration through FLO's
    // open-ended extension-value representation.
    let camera_section: CameraSection =
        serde_yaml::from_str(yaml).wrap_err("parsing flo-strand-cam configuration")?;
    let camera_host = CameraHostConfig::resolve(camera_section.camera_host)?;

    let mut flo_config: flo_core::FloControllerConfig = serde_yaml::from_str(yaml)?;
    flo_config
        .extensions
        .remove(serde_yaml::Value::String(CONFIG_SECTION.to_owned()));
    Ok((camera_host, flo_config))
}

/// Run the composed application while retaining caller-supplied FLO options.
///
/// Downstream binaries can use this to add ordinary [`flo::Extension`] values.
/// The camera host remains owned by this crate.
pub fn run(options: flo::AppOptions) -> Result<()> {
    run_with_args(options, std::env::args_os())
}

/// Run the composed application with explicit command-line arguments.
pub fn run_with_args<I, T>(options: flo::AppOptions, args: I) -> Result<()>
where
    I: IntoIterator<Item = T>,
    T: Into<OsString> + Clone,
{
    let args: Vec<OsString> = args.into_iter().map(Into::into).collect();
    // Both of these are answered before a configuration file is required. Help
    // especially: reading it is how someone finds out that `--config` exists
    // and what it wants, so demanding one first would be circular.
    if is_version_request(&args) {
        println!("{APP_NAME} {VERSION}");
        return Ok(());
    }
    if let Some(help) = flo::cli_help(&args) {
        print!("{help}");
        return Ok(());
    }
    let config_path = config_path_from_args(&args)?;
    let (config, flo_config, raw_config_source) = load_composed_config(&config_path)?;
    flo::run_with_args_and_config(
        compose_options(options, config)?,
        args,
        flo_config,
        Some(raw_config_source),
    )
}

fn is_version_request(args: &[OsString]) -> bool {
    args.iter()
        .any(|arg| matches!(arg.to_str(), Some("--version" | "-V")))
}

fn compose_options(
    mut options: flo::AppOptions,
    config: CameraHostConfig,
) -> Result<flo::AppOptions> {
    if options.camera_host.is_some() {
        return Err(eyre!(
            "flo owns the first-class camera host; caller AppOptions.camera_host must be None"
        ));
    }
    options.camera_host = Some(Box::new(StrandCamHost {
        config,
        controls: CameraControlChannels::new(),
    }));
    Ok(options)
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::DateTime;

    fn detection(frame_number: u64) -> ImOpsDetection {
        ImOpsDetection {
            metadata: ImOpsFrameMetadata {
                frame_number,
                timestamp: DateTime::UNIX_EPOCH,
                timestamp_source: flo_core::TimestampSource::HostAcquiredTimestamp,
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
    fn version_arguments_do_not_require_a_configuration_file() {
        assert!(is_version_request(&[
            OsString::from("flo"),
            OsString::from("--version"),
        ]));
        assert!(is_version_request(&[
            OsString::from("flo"),
            OsString::from("-V"),
        ]));
        assert!(!is_version_request(&[OsString::from("flo")]));
    }

    /// Reading the help is how someone learns that `--config` exists, so it
    /// has to be answerable without one. Driven through `run_with_args` rather
    /// than the helper alone, because the bug this covers was the ordering of
    /// the two, not the rendering.
    #[test]
    fn help_arguments_do_not_require_a_configuration_file() {
        for flag in ["--help", "-h"] {
            run_with_args(
                flo::AppOptions::default(),
                [OsString::from("flo"), OsString::from(flag)],
            )
            .unwrap_or_else(|e| panic!("`flo {flag}` should print help and succeed, got: {e}"));
        }
    }

    /// A command line that is merely wrong must not be mistaken for a request
    /// for help; it still has to reach the error that explains what is missing.
    #[test]
    fn a_bare_invocation_still_demands_a_configuration_file() {
        assert!(run_with_args(flo::AppOptions::default(), [OsString::from("flo")]).is_err());
    }

    #[test]
    fn drops_frame_numbers_flo_cannot_represent() {
        assert!(detection_to_centroid(detection(u64::from(u32::MAX) + 1)).is_none());
    }

    #[test]
    fn imops_configuration_is_owned_by_flo() {
        let config = ImOpsConfig {
            enabled: false,
            threshold: 123,
            center_x: 456,
            center_y: 789,
        };

        assert!(!config.enabled);
        assert_eq!(
            config.processor_config(),
            ImOpsProcessorConfig {
                threshold: 123,
                center_x: 456,
                center_y: 789,
            }
        );
    }

    #[test]
    fn sim_direct_sink_reaches_bounded_flo_ingress_without_udp() {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        runtime.block_on(async {
            // This is the same bounded direct sink supplied to Strand Camera
            // in `StrandCamHost::spawn`; no socket is involved.
            let (direct_sink_tx, mut direct_sink_rx) = mpsc::channel(1);
            direct_sink_tx.try_send(detection(42)).unwrap();

            let received_detection = direct_sink_rx.recv().await.unwrap();
            let centroid = detection_to_centroid(received_detection).unwrap();

            // Model FLO's bounded in-process centroid ingress. The camera host
            // uses the equivalent `CentroidInputSender::try_send` API, so the
            // camera task never waits for FLO to drain observations.
            let (flo_ingress_tx, mut flo_ingress_rx) = mpsc::channel(1);
            flo_ingress_tx.try_send(centroid).unwrap();
            assert_eq!(flo_ingress_rx.recv().await.unwrap().framenumber, 42);

            flo_ingress_tx
                .try_send(detection_to_centroid(detection(43)).unwrap())
                .unwrap();
            assert!(matches!(
                flo_ingress_tx.try_send(detection_to_centroid(detection(44)).unwrap()),
                Err(tokio::sync::mpsc::error::TrySendError::Full(_))
            ));
        });
    }

    #[test]
    fn integrated_app_composes_a_camera_host() {
        let (config, _) =
            parse_composed_config(include_str!("../../../config-flo-strand-cam-sim.yaml")).unwrap();
        assert!(
            compose_options(flo::AppOptions::default(), config)
                .unwrap()
                .camera_host
                .is_some()
        );
    }

    struct DummyExtension;

    impl flo::Extension for DummyExtension {
        fn name(&self) -> &'static str {
            "dummy"
        }

        fn spawn(
            self: Box<Self>,
            _ctx: flo::ExtensionContext<'_>,
        ) -> Result<tokio::task::JoinHandle<Result<()>>> {
            unreachable!("composition test does not start FLO")
        }
    }

    #[test]
    fn composition_preserves_caller_extensions_and_rejects_another_camera_host() {
        let (config, _) =
            parse_composed_config(include_str!("../../../config-flo-strand-cam-sim.yaml")).unwrap();
        let options = flo::AppOptions {
            extensions: vec![Box::new(DummyExtension)],
            ..Default::default()
        };
        let composed = compose_options(options, config).unwrap();
        assert_eq!(composed.extensions.len(), 1);
        assert_eq!(composed.extensions[0].name(), "dummy");

        let (config, _) =
            parse_composed_config(include_str!("../../../config-flo-strand-cam-sim.yaml")).unwrap();
        assert!(compose_options(composed, config).is_err());
    }

    /// Every shipped `sim` configuration has to name its cameras the way the
    /// `ci2-sim` backend does — it enumerates `simcam0`, `simcam1`, ... and
    /// nothing else — or the camera never opens and the documented command
    /// fails at startup. This is exactly how `config-sim-stereo.yaml` went
    /// stale.
    #[test]
    fn every_shipped_sim_config_uses_the_backend_s_camera_names() {
        for (name, yaml) in [
            (
                "config-flo-strand-cam-sim.yaml",
                include_str!("../../../config-flo-strand-cam-sim.yaml"),
            ),
            (
                "config-sim-stereo.yaml",
                include_str!("../../../config-sim-stereo.yaml"),
            ),
        ] {
            let (camera_host, _) = parse_composed_config(yaml).unwrap();
            for camera in std::iter::once(&camera_host.main).chain(camera_host.secondary.as_ref()) {
                if !matches!(camera.backend, CameraBackend::Sim) {
                    continue;
                }
                assert!(
                    camera
                        .camera_name
                        .strip_prefix("simcam")
                        .is_some_and(|index| index.parse::<usize>().is_ok()),
                    "{name} names a sim camera {:?}, which the ci2-sim backend never offers",
                    camera.camera_name,
                );
            }
        }
    }

    /// The stereo simulation drives its own centroids, so both cameras have to
    /// be present (there is a `stereopsis_calib` to exercise) and both need an
    /// `expected_fps` for the post-trigger buffer to be sizeable.
    #[test]
    fn the_stereo_sim_config_is_complete_enough_to_drive() {
        let (camera_host, flo_config) =
            parse_composed_config(include_str!("../../../config-sim-stereo.yaml")).unwrap();
        assert!(flo_config.stereopsis_calib.is_some());
        let secondary = camera_host
            .secondary
            .expect("a stereo config needs a secondary camera");
        assert!(camera_host.main.expected_fps.is_some());
        assert!(secondary.expected_fps.is_some());
    }

    #[test]
    fn sim_example_config_is_extracted_before_flo_parses_its_config() {
        let (camera_host, flo_config) =
            parse_composed_config(include_str!("../../../config-flo-strand-cam-sim.yaml")).unwrap();
        assert!(matches!(camera_host.main.backend, CameraBackend::Sim));
        let secondary = camera_host.secondary.unwrap();
        assert_eq!(camera_host.main.camera_name, "simcam0");
        assert_eq!(secondary.camera_name, "simcam1");
        assert_eq!(secondary.http_address, "127.0.0.1:3441");
        assert!(matches!(
            camera_host.main.mp4_max_framerate,
            Some(RecordingFrameRate::Fps60)
        ));
        let Some(CodecSelection::Ffmpeg(ffmpeg)) = camera_host.main.mp4_codec else {
            panic!("example must preserve the VAAPI Ffmpeg codec configuration");
        };
        assert_eq!(ffmpeg.codec.as_deref(), Some("h264_vaapi"));
        assert_eq!(
            ffmpeg.device_args,
            Some(vec![(
                "-vaapi_device".to_owned(),
                "/dev/dri/renderD128".to_owned(),
            )])
        );
        assert_eq!(
            ffmpeg.pre_codec_args,
            Some(vec![("-vf".to_owned(), "format=nv12,hwupload".to_owned(),)])
        );
        assert_eq!(
            ffmpeg.post_codec_args,
            Some(vec![("-color_range".to_owned(), "pc".to_owned())])
        );
        assert!(
            !flo_config
                .extensions
                .contains_key(serde_yaml::Value::String(CONFIG_SECTION.to_owned()))
        );
    }

    #[test]
    fn legacy_vaapi_codec_config_is_migrated_to_the_current_preset() {
        let raw: RawCameraHostConfig = serde_yaml::from_str(
            "main:\n  backend: sim\n  camera_name: main\n  mp4_codec:\n    Ffmpeg:\n      device_args:\n        - [\"-vaapi_device\", \"/dev/dri/renderD128\"]\n      pre_codec_args:\n        - [\"-vf\", \"format=nv12,hwupload\"]\n      codec: h264_vaapi\n      post_codec_args: null\n      pixfmt: yuv420p\n  imops:\n    center_x: 1\n    center_y: 1\n",
        )
        .unwrap();

        let CodecSelection::Ffmpeg(codec) = raw.main.resolve(CameraRole::Main).mp4_codec.unwrap()
        else {
            panic!("expected an FFmpeg codec");
        };
        assert_eq!(codec.pixfmt, None);
        assert_eq!(
            codec.post_codec_args,
            Some(vec![("-color_range".to_owned(), "pc".to_owned())])
        );
    }

    #[test]
    fn config_path_is_required_and_accepts_both_clap_forms() {
        assert!(config_path_from_args(&[OsString::from("flo")]).is_err());
        assert_eq!(
            config_path_from_args(&[
                OsString::from("flo"),
                OsString::from("--config"),
                OsString::from("config.yaml"),
            ])
            .unwrap(),
            PathBuf::from("config.yaml")
        );
        assert_eq!(
            config_path_from_args(&[
                OsString::from("flo"),
                OsString::from("--config=config.yaml"),
            ])
            .unwrap(),
            PathBuf::from("config.yaml")
        );
    }

    #[test]
    fn config_requires_image_center() {
        let parsed: std::result::Result<ImOpsConfig, _> = serde_yaml::from_str("threshold: 200");
        assert!(parsed.is_err());
    }

    #[test]
    fn config_requires_a_main_camera_and_camera_names() {
        let no_main: std::result::Result<RawCameraHostConfig, _> = serde_yaml::from_str("{}");
        assert!(no_main.is_err());

        let no_name: std::result::Result<RawCameraHostConfig, _> = serde_yaml::from_str(
            "main:\n  backend: sim\n  imops:\n    center_x: 1\n    center_y: 1\n",
        );
        assert!(no_name.is_err());
    }

    #[test]
    fn secondary_camera_is_optional_and_gets_a_distinct_default_http_address() {
        let main_only: RawCameraHostConfig = serde_yaml::from_str(
            "main:\n  backend: sim\n  camera_name: main\n  imops:\n    center_x: 1\n    center_y: 1\n",
        )
        .unwrap();
        assert!(
            CameraHostConfig::resolve(main_only)
                .unwrap()
                .secondary
                .is_none()
        );

        let raw: RawCameraHostConfig = serde_yaml::from_str(
            "main:\n  backend: sim\n  camera_name: main\n  imops:\n    center_x: 1\n    center_y: 1\nsecondary:\n  backend: sim\n  camera_name: secondary\n  imops:\n    center_x: 1\n    center_y: 1\n",
        )
        .unwrap();
        let config = CameraHostConfig::resolve(raw).unwrap();
        assert_eq!(config.main.http_address, "127.0.0.1:3440");
        assert_eq!(config.secondary.unwrap().http_address, "127.0.0.1:3441");
    }

    #[test]
    fn dual_camera_config_rejects_duplicate_names_and_http_addresses() {
        let duplicate_name: RawCameraHostConfig = serde_yaml::from_str(
            "main:\n  backend: sim\n  camera_name: same\n  imops:\n    center_x: 1\n    center_y: 1\nsecondary:\n  backend: sim\n  camera_name: same\n  imops:\n    center_x: 1\n    center_y: 1\n",
        )
        .unwrap();
        assert!(CameraHostConfig::resolve(duplicate_name).is_err());

        let duplicate_address: RawCameraHostConfig = serde_yaml::from_str(
            "main:\n  backend: sim\n  camera_name: main\n  http_address: 127.0.0.1:4000\n  imops:\n    center_x: 1\n    center_y: 1\nsecondary:\n  backend: sim\n  camera_name: secondary\n  http_address: 127.0.0.1:4000\n  imops:\n    center_x: 1\n    center_y: 1\n",
        )
        .unwrap();
        assert!(CameraHostConfig::resolve(duplicate_address).is_err());
    }

    #[test]
    fn registers_each_configured_camera_for_its_flo_role() {
        let (config, _) =
            parse_composed_config(include_str!("../../../config-flo-strand-cam-sim.yaml")).unwrap();
        let camera_host = StrandCamHost {
            config,
            controls: CameraControlChannels::new(),
        };
        let registrations = camera_host.camera_registrations();

        assert_eq!(registrations.len(), 2);
        assert_eq!(registrations[0].role, flo_core::StrandCamRole::Main);
        assert_eq!(registrations[0].name, "simcam0");
        assert!(registrations[0].router_rx.is_some());
        assert_eq!(registrations[0].expected_fps, Some(60.0));
        assert!(registrations[0].control_tx.is_some());
        assert_eq!(registrations[1].role, flo_core::StrandCamRole::Secondary);
        assert_eq!(registrations[1].name, "simcam1");
        assert!(registrations[1].router_rx.is_some());
    }

    #[test]
    fn startup_recording_controls_share_the_in_process_control_channel() {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        runtime.block_on(async {
            let (cam_args_tx, cam_args_rx) = mpsc::channel(CAMERA_CONTROL_QUEUE_CAPACITY);
            let config = CameraConfig {
                backend: CameraBackend::Sim,
                camera_name: "main".to_owned(),
                http_address: "127.0.0.1:3440".to_owned(),
                expected_fps: Some(60.0),
                mp4_max_framerate: Some(RecordingFrameRate::Fps60),
                mp4_codec: Some(CodecSelection::H264OpenH264),
                imops: ImOpsConfig {
                    enabled: true,
                    threshold: 200,
                    center_x: 1,
                    center_y: 1,
                },
                force_camera_sync_mode: None,
            };
            send_initial_camera_args(&config, &cam_args_tx);
            let mut cam_args_rx = cam_args_rx;
            assert!(matches!(
                cam_args_rx.recv().await,
                Some(CamArg::SetMp4MaxFramerate(RecordingFrameRate::Fps60))
            ));
            assert!(matches!(
                cam_args_rx.recv().await,
                Some(CamArg::SetMp4Codec(CodecSelection::H264OpenH264))
            ));
        });
    }

    #[test]
    fn failed_camera_stops_its_peer() {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        runtime.block_on(async {
            let (_shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
            let (main_shutdown_tx, _main_shutdown_rx) = oneshot::channel();
            let (secondary_shutdown_tx, secondary_shutdown_rx) = oneshot::channel();
            let peer_stopped = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
            let peer_stopped_task = peer_stopped.clone();

            let result = supervise_camera_pair(
                async { Ok(()) },
                async move {
                    let _ = secondary_shutdown_rx.await;
                    peer_stopped_task.store(true, std::sync::atomic::Ordering::SeqCst);
                    Ok(())
                },
                shutdown_rx,
                main_shutdown_tx,
                secondary_shutdown_tx,
            )
            .await;

            assert!(
                result
                    .unwrap_err()
                    .to_string()
                    .contains("main Strand Camera")
            );
            assert!(peer_stopped.load(std::sync::atomic::Ordering::SeqCst));
        });
    }

    #[test]
    fn cancellation_retains_vendor_guard_but_clean_shutdown_drops_it() {
        struct DropCounter(std::sync::Arc<std::sync::atomic::AtomicUsize>);
        impl Drop for DropCounter {
            fn drop(&mut self) {
                self.0.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            }
        }

        let drops = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        drop(GuardUntilCleanShutdown::new(DropCounter(drops.clone())));
        assert_eq!(drops.load(std::sync::atomic::Ordering::SeqCst), 0);

        GuardUntilCleanShutdown::new(DropCounter(drops.clone())).clean_shutdown();
        assert_eq!(drops.load(std::sync::atomic::Ordering::SeqCst), 1);
    }
}
