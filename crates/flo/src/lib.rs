//! # Fast Lock On (FLO)
//!
//! Camera-based subject tracking and videography across several motor
//! controller backends (PWM servos, Trinamic steppers, SimpleBGC gimbal).
//!
//! Two deployment contexts are supported by the public flo binary:
//!
//! - **Ground-based tracking**: Running on a desktop or embedded
//!   computer, controlling pan/tilt/focus motors to keep the camera
//!   pointed at the subject.
//!
//! - **Drone telemetry**: Running as a process on a companion computer
//!   alongside a flight controller. flo-mavlink streams telemetry,
//!   parses RC channel data, and surfaces battery / flight-mode / GNSS
//!   state on the OSD. The pilot still drives the drone — flo only
//!   reads.
//!
//! **Error handling behavior.**
//!
//! In the drone telemetry deployment, error handling is safety-critical:
//! the webcam video path must keep running for the pilot's safety, even
//! if motors, tracking, or other subsystems fail. flo never issues
//! steering commands in this binary, so a fault never causes loss of
//! manual control.
//!
//! Note that "in this binary" is doing real work in that sentence.
//! [`ExtensionContext::autopilot_link`] lets a binary that composes flo with an
//! [`Extension`] send to the flight controller, and nothing here constrains
//! what. Such a binary is no longer covered by the argument above and has to
//! make its own.

use clap::Parser;
use color_eyre::eyre::{self, Result, WrapErr};
use futures::StreamExt;
use std::{
    future::{Future, pending},
    pin::Pin,
};
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};
use tokio_serial::SerialPortBuilderExt;
use tracing::{self as log};

use flo_core::{
    Angle, Broadway, CamRole, CamStaleBitmask, CommandSource, DeviceId, DeviceMode, DeviceState,
    DisplaySource, FloCommand, FloControllerConfig, FloEvent, FloatType, FocusMotorType,
    GimbalConfig, LocalFloState, ModeChangeReason, MomentCentroid, MotorPositionResult, MotorType,
    MotorValueCache, OsdState, PwmSerial, RadialDistance, SaveToDiskMsg, StampedBMsg,
    StereoSyncParams, StereoSynchronizer, StrandCamRole, drone_structs::DroneEvent,
};
use tracking::{centroid_to_sensor_angles, compute_motor_output, kalman_step};

mod camera_host;
mod camshow_client;
mod codec;
mod extension;
mod json_lines_writer;
pub mod osd;
mod pwm_serial_io;
mod tilta_io;
mod tracking;
mod trinamic_io;
mod webcam_preview_client;
mod writing_state;
mod zip_dir;

pub use camera_host::{CameraHost, CameraHostContext, CameraRegistration};
pub use extension::{Extension, ExtensionContext, OsdOverlay};
pub use osd::DroneStatus;

/// The MAVLink layer, re-exported because [`ExtensionContext::autopilot_link`]
/// hands out one of its types.
///
/// An out-of-tree extension can reach [`flo_mavlink::AutopilotLink`], the
/// autopilot address constants, and (through [`flo_mavlink::mavlink`]) the
/// message types themselves without naming `flo-mavlink` as a second dependency
/// that could come to point at a different revision of this repository than
/// `flo` does.
pub use flo_mavlink;

/// Default number of camera observations retained while the coordinator is
/// busy. The bounded queue provides backpressure rather than allowing a slow
/// controller to grow memory without limit.
pub const DEFAULT_CENTROID_INPUT_CAPACITY: usize = 64;

/// Timestamped FLO state that image-processing integrations can use when
/// interpreting a frame. This is intentionally a compact application-facing
/// snapshot rather than a view of the coordinator internals.
#[derive(Clone, Debug)]
pub struct ProcessingFeedback {
    /// Time at which FLO last assembled this snapshot.
    pub updated_at: chrono::DateTime<chrono::Utc>,
    /// Most recently reported motor positions, if any motor backend has
    /// provided them.
    pub motor_readout: Option<MotorPositionResult>,
    /// Most recent command sent to the motor-control watch channel.
    pub motor_command: MotorValueCache,
    /// Current tracking mode associated with the command.
    pub tracking_mode: DeviceMode,
}

/// Sender for the transport-independent camera-observation input of a FLO
/// application.
///
/// This is handed to extensions in [`ExtensionContext`]. A full queue means
/// the coordinator cannot keep up; camera integrations should choose and log
/// their own drop/backpressure policy via [`Self::try_send`].
#[derive(Clone, Debug)]
pub struct CentroidInputSender {
    tx: mpsc::Sender<MomentCentroid>,
}

impl CentroidInputSender {
    /// Queue an observation, waiting for coordinator capacity if necessary.
    pub async fn send(
        &self,
        centroid: MomentCentroid,
    ) -> std::result::Result<(), mpsc::error::SendError<MomentCentroid>> {
        self.tx.send(centroid).await
    }

    /// Queue an observation from a blocking producer thread.
    ///
    /// This is intended for compatibility sources which reproduce recorded
    /// wall-clock cadence. New async camera integrations should use
    /// [`Self::send`] instead.
    pub fn blocking_send(
        &self,
        centroid: MomentCentroid,
    ) -> std::result::Result<(), mpsc::error::SendError<MomentCentroid>> {
        self.tx.blocking_send(centroid)
    }

    /// Attempt to queue an observation without waiting for coordinator
    /// capacity.
    pub fn try_send(
        &self,
        centroid: MomentCentroid,
    ) -> std::result::Result<(), mpsc::error::TrySendError<MomentCentroid>> {
        self.tx.try_send(centroid)
    }

    /// Returns whether the coordinator receiver has been dropped.
    pub fn is_closed(&self) -> bool {
        self.tx.is_closed()
    }
}

fn centroid_input_channel(
    capacity: usize,
) -> (CentroidInputSender, mpsc::Receiver<MomentCentroid>) {
    let (tx, rx) = mpsc::channel(capacity);
    (CentroidInputSender { tx }, rx)
}

const HIGHMAG_VISIBLE_MP4_PATH_TEMPLATE: &str = "highmag%Y%m%d_%H%M%S.%f.mp4";
const WEBCAM_MP4_PATH_TEMPLATE: &str = "webcam%Y%m%d_%H%M%S.%f.mp4";
const FLO_DIRNAME_TEMPLATE: &str = "flo%Y%m%d_%H%M%S.%f";

/// Pattern that valid floz output directory names match. Compiled once
/// the first time recording starts; reused on every subsequent start.
static FLO_DIRNAME_RE: std::sync::LazyLock<regex::Regex> = std::sync::LazyLock::new(|| {
    regex::Regex::new(r"^flo[0-9]{8}_[0-9]{6}\.[0-9]+$").expect("FLO_DIRNAME_RE is a valid regex")
});

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[clap(subcommand)]
    command: Option<Commands>,

    /// Path of RPi Pico serial device flashed with `rpipico-pantilt`
    #[arg(long)]
    pwm_serial: Option<String>,

    /// Path of pan Trinamic motor
    #[arg(long)]
    trinamic_pan: Option<String>,

    /// Path of tilt Trinamic motor
    #[arg(long)]
    trinamic_tilt: Option<String>,

    /// Path to focus Trinamic motor
    #[arg(long)]
    trinamic_focus: Option<String>,

    /// Path to serial port of SBGC gimbal
    #[arg(long)]
    gimbal: Option<String>,

    /// serial port of OSD
    #[arg(long)]
    osd: Option<String>,

    /// TCP address of a `camshow` instance to push OSD canvas updates and
    /// recording control to. Overrides `osd_config.camshow_addr` if set.
    #[arg(long)]
    camshow: Option<String>,

    /// The address to bind for the HTTP server
    #[arg(long, default_value = "0.0.0.0:2222")]
    http_addr: String,

    /// Client network(s) (CIDR, e.g. 100.64.0.0/10) trusted to have already
    /// authenticated the peer (e.g. Tailscale/WireGuard). Clients from these
    /// networks need no access token. May be repeated or comma-separated.
    #[arg(long = "trusted-network", value_delimiter = ',')]
    trusted_networks: Vec<String>,

    /// Filename of initial device configuration in YAML format
    #[arg(long)]
    config: Option<String>,

    /// If set, .floz files and logs are saved to this directory.
    #[arg(long)]
    data_dir: Option<camino::Utf8PathBuf>,
}

#[derive(Debug, clap::Subcommand, Default)]
enum Commands {
    /// Run the program (default command)
    #[default]
    Run,
    /// Show the configuration and then quit
    ShowConfig,
}

fn get_device_id() -> Result<flo_core::DeviceId> {
    use sha2::Digest;

    let device_id = platform_uuid::get_uuid()?;
    let digest = sha2::Sha256::digest(device_id);
    assert!(digest.len() >= flo_core::DEVICE_ID_LEN);
    let device_id = flo_core::DeviceId::new(digest[..flo_core::DEVICE_ID_LEN].try_into()?);
    log::info!("This is device {device_id:?}");
    Ok(device_id)
}

/// The streams camshow should be sending right now.
///
/// Camshow is given exactly what it should be sending, so a destination that is
/// switched off — individually, or by the master switch — is simply absent.
/// Both flags leave `state.rtp_targets` alone, which is what lets a stream be
/// stopped and restarted without its address being retyped.
fn rtp_targets_to_send(state: &flo_core::DeviceState) -> Vec<flo_core::RtpTarget> {
    if !state.rtp_send_enabled {
        return Vec::new();
    }
    canonical_rtp_targets(state.rtp_targets.clone())
        .into_iter()
        .filter(|config| config.enabled)
        .map(|config| config.target)
        .collect()
}

fn canonical_rtp_targets(
    targets: Vec<flo_core::RtpTargetConfig>,
) -> Vec<flo_core::RtpTargetConfig> {
    let mut unique = Vec::with_capacity(targets.len());
    for config in targets {
        if config.target.bitrate_kbps > 0
            && !unique.iter().any(|existing: &flo_core::RtpTargetConfig| {
                existing.target.addr == config.target.addr
            })
        {
            unique.push(config);
        }
    }
    unique
}

struct FloCoordinator<'a> {
    data_dir: &'a camino::Utf8Path,
    device_config: &'a mut FloControllerConfig,
    secondary_cam_name: Option<String>,
    tracking_state: flo_core::TrackingState,
    /// The most recent centroid data from the primary tracking camera.
    latest_centroid: Option<MomentCentroid>,
    /// Pairs the two cameras' detections by trigger, when there are two
    /// cameras. See [`flo_core::stereo_sync`].
    ///
    /// `None` in a monocular deployment. Distance is the only thing the second
    /// camera contributes, so its absence — configured away, not detecting, or
    /// not delivering — costs the distance estimate and nothing else: pan and
    /// tilt are tracked from the main camera, which never waits on any of this.
    stereo_sync: Option<StereoSynchronizer>,
    /// Distance from the newest stereo pair, with the acquisition time it was
    /// measured at, waiting for the next control step to consume it.
    pending_stereopsis: Option<(chrono::DateTime<chrono::Utc>, flo_core::StereopsisState)>,
    /// How old a stereo distance may be when the control step picks it up.
    /// Older measurements are dropped rather than fed to the filter as if they
    /// described the present.
    stereopsis_max_age: chrono::TimeDelta,
    /// A timestamp when last primary camera data arrived.
    stamp_cam_primary: Option<std::time::Instant>,
    /// A timestamp when last secondary camera data arrived.
    stamp_cam_secondary: Option<std::time::Instant>,
    last_motor_data: Option<MotorPositionResult>,
    all_cam_names_ever_seen: std::collections::BTreeSet<String>,
    /// Pairing counts as of the last slow tick, so only changes are reported.
    reported_stereo_stats: flo_core::StereoSyncStats,
    /// Detections from each camera since the last report, which is what says
    /// whether a second without stereo pairs means anything is wrong.
    detections_since_report: (u64, u64),
    /// Consecutive reports in which both cameras saw the subject and nothing
    /// paired. Startup spends a second or two here legitimately, learning the
    /// framenumber offset.
    seconds_unpaired: u32,
    flo_saver_tx: mpsc::UnboundedSender<SaveToDiskMsg>,
    osd_tx: Option<watch::Sender<OsdState>>,
    my_state: flo_core::DeviceState,
    local_flo_state: flo_core::LocalFloState,
    motors_tx: watch::Sender<MotorValueCache>,
    processing_feedback_tx: watch::Sender<ProcessingFeedback>,
    audio_stream: Option<&'a rodio::OutputStreamHandle>,
    from_device_http_tx: watch::Sender<DeviceState>,
    broadway: flo_core::Broadway,
    cam_control_main: Option<mpsc::Sender<strand_cam_remote_control::CamArg>>,
    cam_control_secondary: Option<mpsc::Sender<strand_cam_remote_control::CamArg>>,
    cam_control_main_expected_fps: Option<f64>,
    cam_control_secondary_expected_fps: Option<f64>,
    camshow_recording_tx: Option<mpsc::UnboundedSender<camshow_client::Command>>,
    /// The operator's live-view selection. Read by the camshow link and by the
    /// camera host's frame relay; this is the only writer.
    display_source_tx: watch::Sender<DisplaySource>,
    /// H.264/RTP destinations selected in the BUI, forwarded to camshow.
    rtp_targets_tx: watch::Sender<Vec<flo_core::RtpTarget>>,
    /// Pre-capture window mirrored to camshow, so its own frame buffer holds
    /// the same span of video the `.floz` writer holds of data.
    camshow_precapture_secs_tx: watch::Sender<f64>,
    highmag_visible_recorder: Option<h264_recorder::H264Recorder>,
    /// Seconds of data currently held in the writer's pre-capture buffer.
    precapture_buffered_rx: watch::Receiver<f64>,
}

impl<'a> FloCoordinator<'a> {
    #[expect(clippy::too_many_arguments)]
    fn new(
        device_config: &'a mut FloControllerConfig,
        flo_saver_tx: mpsc::UnboundedSender<SaveToDiskMsg>,
        osd_tx: Option<watch::Sender<OsdState>>,
        my_state: flo_core::DeviceState,
        local_flo_state: LocalFloState,
        motors_tx: watch::Sender<MotorValueCache>,
        processing_feedback_tx: watch::Sender<ProcessingFeedback>,
        audio_stream: Option<&'a rodio::OutputStreamHandle>,
        from_device_http_tx: watch::Sender<DeviceState>,
        broadway: Broadway,
        camshow_recording_tx: Option<mpsc::UnboundedSender<camshow_client::Command>>,
        display_source_tx: watch::Sender<DisplaySource>,
        rtp_targets_tx: watch::Sender<Vec<flo_core::RtpTarget>>,
        camshow_precapture_secs_tx: watch::Sender<f64>,
        data_dir: &'a camino::Utf8Path,
        precapture_buffered_rx: watch::Receiver<f64>,
        strand_cams: InitializedStrandCams,
    ) -> Self {
        let InitializedStrandCams {
            secondary_cam_name,
            embedded,
            ..
        } = strand_cams;
        let main_embedded = embedded
            .iter()
            .find(|camera| camera.role == StrandCamRole::Main);
        let secondary_embedded = embedded
            .iter()
            .find(|camera| camera.role == StrandCamRole::Secondary);
        // Both cameras are on the same trigger, so either one's rate describes
        // the pairing; take whichever the configuration states.
        let tracking_fps = main_embedded
            .and_then(|camera| camera.expected_fps)
            .or_else(|| secondary_embedded.and_then(|camera| camera.expected_fps));
        // A monocular deployment has nothing to pair, and says so by having no
        // synchronizer at all rather than one that never sees a second camera.
        let stereo_sync = secondary_cam_name
            .is_some()
            .then(|| StereoSynchronizer::new(StereoSyncParams::for_fps(tracking_fps)));
        Self {
            data_dir,
            device_config,
            secondary_cam_name,
            tracking_state: Default::default(),
            latest_centroid: Default::default(),
            stereo_sync,
            pending_stereopsis: None,
            stereopsis_max_age: stereopsis_max_age(tracking_fps),
            stamp_cam_primary: Default::default(),
            stamp_cam_secondary: Default::default(),
            last_motor_data: Default::default(),
            all_cam_names_ever_seen: Default::default(),
            reported_stereo_stats: Default::default(),
            detections_since_report: (0, 0),
            seconds_unpaired: 0,
            flo_saver_tx,
            osd_tx,
            my_state,
            local_flo_state,
            motors_tx,
            processing_feedback_tx,
            audio_stream,
            from_device_http_tx,
            broadway,
            cam_control_main: main_embedded.and_then(|camera| camera.control_tx.clone()),
            cam_control_secondary: secondary_embedded.and_then(|camera| camera.control_tx.clone()),
            cam_control_main_expected_fps: main_embedded.and_then(|camera| camera.expected_fps),
            cam_control_secondary_expected_fps: secondary_embedded
                .and_then(|camera| camera.expected_fps),
            camshow_recording_tx,
            display_source_tx,
            rtp_targets_tx,
            camshow_precapture_secs_tx,
            highmag_visible_recorder: None,
            precapture_buffered_rx,
        }
    }

    fn on_new_motor_position(&mut self, motor_position: MotorPositionResult) -> Result<()> {
        log::trace!(
            "got motor_position. pan = {:?}, tilt = {:?} ",
            motor_position.pan_enc,
            motor_position.tilt_enc
        );
        self.last_motor_data = Some(motor_position.clone());
        self.publish_processing_feedback();
        //rec.pan_pos = motor_position.pan_pos;
        //rec.tilt_pos = motor_position.tilt_pos;
        {
            if let Some(osd_tx) = self.osd_tx.as_mut() {
                let osd_state = flo_core::OsdState {
                    motor_state: motor_position.clone(),
                    bee_dist: self.tracking_state.dist_obs.into(),
                    tracking_mode: self.tracking_state.mode,
                    cam_stale: self.my_state.cam_stale,
                };
                osd_tx.send(osd_state)?;
            }
        }
        self.flo_saver_tx
            .send(SaveToDiskMsg::MotorPosition(Box::new(motor_position)))?;
        Ok(())
    }

    /// Ingest one observation from the embedded camera host.
    fn on_image_centroid(&mut self, centroid: MomentCentroid) -> Result<()> {
        if centroid.schema_version != 2 {
            tracing::error!(
                "expected centroid schema version 2, found {}",
                centroid.schema_version
            );
            return Ok(());
        }

        if centroid.mu00 != 0.0 {
            // Save data if something was detected.
            let msg = SaveToDiskMsg::CentroidData((chrono::Local::now(), centroid.clone()));
            self.flo_saver_tx.send(msg)?;
        }

        // Check if we have a second camera but no secondary camera in configuration.
        self.all_cam_names_ever_seen
            .insert(centroid.cam_name.clone());

        if self.all_cam_names_ever_seen.len() > 1 && self.secondary_cam_name.is_none() {
            eyre::bail!("More than one camera sending data, but no secondary camera assigned");
        }

        let cam_name = centroid.cam_name.as_str();
        let mut is_secondary = false;
        if let Some(scn) = self.secondary_cam_name.as_ref()
            && cam_name == scn
        {
            is_secondary = true;
        }

        let role = if is_secondary {
            self.stamp_cam_secondary = Some(std::time::Instant::now());
            CamRole::Secondary
        } else {
            self.stamp_cam_primary = Some(std::time::Instant::now());
            if centroid.mu00 != 0.0 {
                self.latest_centroid = Some(centroid.clone());
            }
            CamRole::Primary
        };

        if centroid.mu00 == 0.0 {
            // Nothing was detected, so there is nothing to pair or to report.
            return Ok(());
        }

        match role {
            CamRole::Primary => self.detections_since_report.0 += 1,
            CamRole::Secondary => self.detections_since_report.1 += 1,
        }

        // Pair here rather than on the control tick: an observation completes
        // its pair the moment it arrives, and waiting for a tick would only
        // add latency to the distance estimate. Nothing above this point
        // depends on it, so a deployment with one camera, or one whose second
        // camera cannot see the subject, tracks exactly as it otherwise would.
        if let Some(sync) = self.stereo_sync.as_mut()
            && let Some(pair) = sync.push(role, centroid.clone())
        {
            self.on_stereo_pair(pair)?;
        }

        self.broadway
            .flo_detections
            .send(flo_core::FloDetectionEvent::Centroid(
                flo_core::CentroidEvent {
                    centroid,
                    is_primary: !is_secondary,
                },
            ))?;

        Ok(())
    }

    /// Turn one completed stereo pair into a distance for the next control
    /// step.
    fn on_stereo_pair(&mut self, pair: flo_core::StereoPair) -> Result<()> {
        self.broadway
            .flo_detections
            .send(flo_core::FloDetectionEvent::StereoCentroid(
                pair.primary.clone(),
                pair.secondary.clone(),
            ))?;

        let Some(calib) = self.device_config.stereopsis_calib.as_ref() else {
            // Two cameras but no calibration to turn disparity into distance.
            return Ok(());
        };
        let acquired = pair.primary.timestamp;
        let stereopsis = calib.centroids_to_distance((pair.primary, pair.secondary));
        self.pending_stereopsis = Some((acquired, stereopsis));
        Ok(())
    }

    /// The distance from the newest stereo pair, if it is recent enough to
    /// describe where the subject is now.
    fn take_fresh_stereopsis(&mut self) -> Option<flo_core::StereopsisState> {
        let (acquired, stereopsis) = self.pending_stereopsis.take()?;
        let age = chrono::Utc::now() - acquired;
        if age > self.stereopsis_max_age {
            tracing::debug!(
                "stereo pairing: discarding a distance measured {} ms ago, older than the {} ms \
                 this control loop accepts",
                age.num_milliseconds(),
                self.stereopsis_max_age.num_milliseconds(),
            );
            return None;
        }
        Some(stereopsis)
    }

    fn on_fast_tick(&mut self, dt_secs: f64) -> Result<()> {
        let next_mode = {
            let (msg, next_mode) = {
                let stereopsis_state = self.take_fresh_stereopsis();
                let tracking_state = &mut self.tracking_state;
                let latest_centroid = self.latest_centroid.take();

                let centroid_timestamp = latest_centroid.as_ref().map(|x| x.timestamp);
                let angles = centroid_to_sensor_angles(self.device_config, latest_centroid.clone());

                if stereopsis_state.is_some() || self.my_state.stereopsis_state.is_none() {
                    //a hack to skip failed stereopsis detections because of centroid packets not arriving promptly, as there are many
                    self.my_state.stereopsis_state.clone_from(&stereopsis_state);
                }
                self.my_state
                    .motor_readout
                    .clone_from(&self.last_motor_data);

                let next_mode = kalman_step(
                    tracking_state,
                    self.device_config,
                    angles,
                    dt_secs,
                    centroid_timestamp,
                    chrono::Utc::now(),
                    self.last_motor_data.clone(),
                    stereopsis_state,
                    &mut self.broadway,
                );
                compute_motor_output(tracking_state, &self.my_state, self.device_config, dt_secs);
                let msg = flo_core::StampedTrackingState {
                    centroid_timestamp,
                    tracking_state: (*tracking_state).clone(),
                    processed_timestamp: chrono::Local::now(),
                };
                (msg, next_mode)
            };
            self.flo_saver_tx
                .send(SaveToDiskMsg::StampedTrackingState(Box::new(msg)))?;
            next_mode
        };

        let current_motors = self.tracking_state.compute_motor_cache();
        self.motors_tx.send(current_motors.clone())?;
        self.my_state.cached_motors = current_motors;
        if let Some(next_mode) = next_mode {
            // Equivalent to handling FloCommand::SwitchMode from
            // CommandSource::Automation, but called directly so this hot,
            // synchronous path need not go through the (now async) command
            // handler.
            self.switch_to_mode(next_mode.0, next_mode.1)?;
        }
        self.publish_processing_feedback();
        Ok(())
    }

    fn publish_processing_feedback(&self) {
        self.processing_feedback_tx
            .send_replace(ProcessingFeedback {
                updated_at: chrono::Utc::now(),
                motor_readout: self.last_motor_data.clone(),
                motor_command: self.tracking_state.compute_motor_cache(),
                tracking_mode: self.tracking_state.mode,
            });
    }

    fn on_slow_tick(&mut self) -> Result<()> {
        // Every second, echo state to all listeners.

        let is_stale_secondary = secondary_camera_is_stale(
            self.secondary_cam_name.as_deref(),
            self.stamp_cam_secondary.as_ref(),
        );
        self.my_state.cam_stale = CamStaleBitmask::new(
            is_stale(self.stamp_cam_primary.as_ref()),
            is_stale_secondary,
        );

        self.report_stereo_pairing();

        self.my_state.mode = self.tracking_state.mode; //my_state.mode is just a mirror of tracking_state.mode, make sure it's up to date
        // Reflect the writer's current pre-capture buffer fill.
        self.my_state.precapture_buffered_secs = *self.precapture_buffered_rx.borrow();
        // Mirror what the flight controller has reported so the BUI can show
        // it. flo-mavlink is the only writer, and leaves this `None` when there
        // is no flight controller.
        self.my_state.mavlink = self.local_flo_state.read().unwrap().mavlink.clone();
        // relay to HTTP server
        self.from_device_http_tx.send(self.my_state.clone())?;
        self.my_state.stereopsis_state = None; //a hack to skip failed stereopsis detections because of centroid packets not arriving promptly, as there are many
        Ok(())
    }

    /// Report once a second on how stereo pairing is doing.
    ///
    /// Reporting here rather than per frame is deliberate: the condition worth
    /// knowing about is "the cameras stopped pairing", which is a property of a
    /// second's worth of frames, and per-frame logging of it buried the signal
    /// in thousands of lines.
    fn report_stereo_pairing(&mut self) {
        let (primary_detections, secondary_detections) =
            std::mem::take(&mut self.detections_since_report);
        let Some(sync) = self.stereo_sync.as_ref() else {
            // Monocular deployment: nothing to pair, and nothing missing.
            return;
        };
        let stats = sync.stats().clone();
        let offset = sync.offset();
        let previous = std::mem::replace(&mut self.reported_stereo_stats, stats.clone());
        let paired = stats.paired - previous.paired;
        let unpaired = stats.unpaired - previous.unpaired;
        let late = stats.late - previous.late;
        let rejected = stats.rejected - previous.rejected;

        if rejected > 0 {
            // Cameras on one trigger, each counting every one of it, cannot
            // produce this: their framenumbers keep the difference they
            // started with. Something the pairing assumes is not true.
            tracing::warn!(
                "stereo pairing: {rejected} framenumber matches in the last second were not of \
                 the same trigger, by their timestamps. The learned offset {offset:?} does not \
                 describe these cameras; has a camera restarted?"
            );
        }
        if paired > 0 {
            self.seconds_unpaired = 0;
            tracing::debug!(
                "stereo pairing: {paired} pairs in the last second ({unpaired} unpaired, \
                 {late} too late to use), framenumber offset {offset:?}"
            );
        } else if primary_detections > 0 && secondary_detections > 0 {
            // Both cameras saw the subject and still nothing paired. Distance
            // estimation is down for a reason the operator cannot see anywhere
            // else, and this is the only case here that is a fault.
            self.seconds_unpaired += 1;
            // Except at the very start, where a second or so of both cameras
            // detecting is what the offset is learned from in the first place.
            if offset.is_none() && self.seconds_unpaired <= 2 {
                tracing::debug!(
                    "stereo pairing: synchronizing the cameras ({primary_detections} and \
                     {secondary_detections} detections in the last second)"
                );
            } else {
                tracing::warn!(
                    "stereo pairing: both cameras detected the subject in the last second \
                     ({primary_detections} and {secondary_detections} detections) but none of it \
                     paired ({unpaired} unpaired, {late} too late to use); no distance is being \
                     estimated"
                );
            }
        } else if primary_detections > 0 {
            // Ordinary: the subject is out of the second camera's view, behind
            // something, or too dim for it. Pan and tilt carry on regardless,
            // and distance resumes by itself when the subject comes back.
            tracing::debug!(
                "stereo pairing: only the main camera saw the subject in the last second \
                 ({primary_detections} detections); tracking continues without a distance estimate"
            );
        }
    }

    async fn handle_event(&mut self, evt: FloEvent) -> Result<()> {
        use FloEvent as E;
        if let E::Command(cmd, src) = evt {
            self.handle_command(cmd, src).await?;
        }
        Ok(())
    }

    /// Start saving a recording.
    ///
    /// When `include_precapture` is true, the writer flushes its pre-capture
    /// buffer into the new recording so it begins with the buffered window
    /// (the "post-trigger" behavior). A normal recording passes false.
    ///
    /// When [`DeviceState::record_tracking_cam_mp4`] is set, the tracking
    /// cameras' MP4 recordings start here too, so one operator action saves
    /// every source — the same thing arming over MAVLink does.
    async fn start_recording(&mut self, include_precapture: bool) -> Result<()> {
        self.local_flo_state.write().unwrap().is_recording = true;
        // A pre-capture recording begins in the past, so date it from where its
        // data actually starts rather than from the trigger. Every output takes
        // its name from this one value, which keeps the `.floz` and the webcam
        // MP4 named for the same instant.
        //
        // The buffered span, not the configured window: shortly after the
        // window is set or enlarged the buffer holds less than was asked for.
        // It is still an estimate — each writer keeps its own buffer, so their
        // first record lands near this time rather than exactly on it.
        let now = chrono::Local::now();
        let creation_time = if include_precapture {
            let buffered_secs = (*self.precapture_buffered_rx.borrow()).max(0.0);
            now - chrono::TimeDelta::microseconds((buffered_secs * 1e6) as i64)
        } else {
            now
        };
        let floz_dirname = creation_time.format(FLO_DIRNAME_TEMPLATE).to_string();
        if !FLO_DIRNAME_RE.is_match(&floz_dirname) {
            tracing::error!("new dirname does not match expected pattern");
        }
        let full_floz_dirname = self.data_dir.join(floz_dirname);
        self.my_state.floz_recording_path = Some(flo_core::RecordingPath::from_path_and_time(
            full_floz_dirname.to_string(),
            creation_time,
        ));

        if let Some(prettycam_cfg) = self.device_config.highmag_visible_recorder.as_ref() {
            let highmag_visible_recording_path = creation_time
                .format(HIGHMAG_VISIBLE_MP4_PATH_TEMPLATE)
                .to_string();
            let fullpath = self.data_dir.join(highmag_visible_recording_path);
            tracing::info!("Starting highmag recording \"{fullpath}\"");

            self.highmag_visible_recorder = Some(h264_recorder::H264Recorder::new(
                &fullpath,
                &prettycam_cfg.ffmpeg_cli,
            )?);
        }

        // The webcam file lives wherever camshow puts it. We record an
        // indicative path (using flo's data dir) so operators can see the
        // basename in the BUI; camshow generates the actual file name from
        // `creation_time`.
        if self.camshow_recording_tx.is_some() {
            let webcam_fname = creation_time.format(WEBCAM_MP4_PATH_TEMPLATE).to_string();
            let display_path = self.data_dir.join(webcam_fname);
            self.my_state.webcam_recording_path =
                Some(flo_core::RecordingPath::from_path_and_time(
                    display_path.to_string(),
                    creation_time,
                ));
        }

        let mp4_cfg = self
            .device_config
            .osd_config
            .as_ref()
            .and_then(|c| c.camshow_mp4_cfg.clone())
            .unwrap_or_default();
        let camshow_cmd =
            camshow_client::Command::Start(Box::new(camshow_protocol::RecordingStart {
                creation_time,
                data_dir: self.data_dir.to_path_buf(),
                mp4_cfg,
                include_precapture,
            }));

        self.flo_saver_tx
            .send(SaveToDiskMsg::ToggleSavingFloz(Some((
                creation_time,
                full_floz_dirname,
                include_precapture,
            ))))
            .unwrap();
        if let Some(tx) = self.camshow_recording_tx.as_ref() {
            // Best-effort: camshow may not be connected. The link task buffers
            // commands and forwards on reconnect.
            let _ = tx.send(camshow_cmd);
        }

        if self.my_state.record_tracking_cam_mp4 {
            // `PostTrigger` starts the MP4 with the camera's own buffered
            // frames prepended, so the video begins in the past like the
            // `.floz` does; a normal start just begins from now.
            let arg = if include_precapture {
                strand_cam_remote_control::CamArg::PostTrigger
            } else {
                strand_cam_remote_control::CamArg::SetIsRecordingMp4(true)
            };
            self.send_cam_arg_to_all(arg).await;
        }
        Ok(())
    }

    /// Stop saving the current recording, finalizing all outputs.
    ///
    /// The tracking cameras are told to stop unconditionally, not only when
    /// [`DeviceState::record_tracking_cam_mp4`] is set: whatever started those
    /// MP4s — this coordinator, a post-trigger, or arming — leaving them
    /// running writes an unbounded file that is never finalized.
    async fn stop_recording(&mut self) -> Result<()> {
        self.my_state.floz_recording_path = None;
        self.my_state.webcam_recording_path = None;

        if let Some(recorder) = self.highmag_visible_recorder.take() {
            match recorder.close() {
                Ok(()) => {
                    tracing::info!("Highmag recording stopped");
                }
                Err(e) => {
                    tracing::error!("Error stopping highmag recording: {e}");
                }
            }
        }

        self.flo_saver_tx
            .send(SaveToDiskMsg::ToggleSavingFloz(None))
            .unwrap();
        if let Some(tx) = self.camshow_recording_tx.as_ref() {
            let _ = tx.send(camshow_client::Command::Stop);
        }
        self.send_cam_arg_to_all(strand_cam_remote_control::CamArg::SetIsRecordingMp4(false))
            .await;
        self.local_flo_state.write().unwrap().is_recording = false;
        Ok(())
    }

    /// Send a `CamArg` to every embedded tracking camera.
    async fn send_cam_arg_to_all(&mut self, arg: strand_cam_remote_control::CamArg) {
        for tx in [&self.cam_control_main, &self.cam_control_secondary]
            .into_iter()
            .flatten()
        {
            send_in_process_cam_arg(tx, arg.clone()).await;
        }
    }

    /// Size each tracking camera's post-trigger buffer to match `secs` of
    /// pre-capture, using that camera's configured `expected_fps`. Cameras
    /// without a configured frame rate are skipped with a warning, since the
    /// seconds→frames conversion is impossible without it.
    async fn set_cam_precapture_buffer(&mut self, secs: f64) {
        for (role, direct_tx, fps) in [
            (
                StrandCamRole::Main,
                &self.cam_control_main,
                self.cam_control_main_expected_fps,
            ),
            (
                StrandCamRole::Secondary,
                &self.cam_control_secondary,
                self.cam_control_secondary_expected_fps,
            ),
        ] {
            let Some(direct_tx) = direct_tx else {
                continue;
            };
            match fps {
                Some(fps) if fps > 0.0 => {
                    let frames = (secs * fps).ceil().max(0.0) as usize;
                    let arg = strand_cam_remote_control::CamArg::SetPostTriggerBufferSize(frames);
                    send_in_process_cam_arg(direct_tx, arg).await;
                }
                _ => {
                    // Without a frame rate there is no seconds→frames
                    // conversion, so this camera keeps a zero-length buffer and
                    // its post-trigger video silently starts from now while the
                    // `.floz` starts in the past.
                    tracing::warn!(
                        "The {role:?} tracking camera has no `expected_fps` in its \
                         `flo-strand-cam` config, so its pre-capture buffer cannot be \
                         sized: post-trigger will record no video from before the trigger."
                    );
                }
            }
        }
    }

    async fn handle_command(&mut self, msg: FloCommand, src: CommandSource) -> Result<()> {
        let was_closed_loop = self.tracking_state.mode == DeviceMode::ClosedLoop;

        match msg {
            FloCommand::SwitchMode(mode, reason) => {
                if src != CommandSource::Automation {
                    //only allow external commands to switch to some modes
                    match mode {
                        DeviceMode::AcquiringLock | DeviceMode::ManualOpenLoop => {}
                        _ => {
                            tracing::error!(
                                "{:?} request to switch to mode {:?} denied",
                                src,
                                mode
                            );
                            return Ok(());
                        }
                    }
                }
                self.switch_to_mode(mode, reason)?;
            }
            FloCommand::SwitchToOpenLoop => {
                self.switch_to_mode(DeviceMode::ManualOpenLoop, ModeChangeReason::Operator)?;
            }
            FloCommand::SetHomePositionFromCurrent => {
                if was_closed_loop {
                    let tracking_state = &mut self.tracking_state;

                    if let Some(lmr) = &tracking_state.last_motor_readout {
                        let pan = lmr.pan_enc;
                        let tilt = lmr.tilt_enc;

                        if let Some(ked) = &tracking_state.kalman_estimates_distance {
                            let distance = ked.0.state()[0];
                            self.my_state.home_position =
                                (pan, tilt, flo_core::RadialDistance(distance));
                        } else {
                            tracing::error!(
                                "Kalman estimates not present. Cannot set new home position."
                            );
                        }
                    } else {
                        tracing::error!(
                            "Last motor readout not present. Cannot set new home position."
                        );
                    }
                } else {
                    tracing::error!("Not in closed loop mode. Will not set new home position.");
                }
                self.switch_to_mode(DeviceMode::ManualOpenLoop, ModeChangeReason::SetNewHome)?;
            }
            FloCommand::SetHomePosition((pan, tilt, distance)) => {
                if let Some(pan) = pan {
                    self.my_state.home_position.0 = pan;
                }
                if let Some(tilt) = tilt {
                    self.my_state.home_position.1 = tilt;
                }
                if let Some(distance) = distance {
                    self.my_state.home_position.2 = distance;
                }
            }
            FloCommand::SetDistCorr(dist_corr) => {
                if let Some(focus_motor_config) = self.device_config.focus_motor_config.as_mut() {
                    focus_motor_config.cal.distance_offset = RadialDistance(dist_corr);
                } else {
                    log::error!("can't adjust distance correction - focus config missing");
                };
            }
            FloCommand::AdjustFocus(change) => {
                if let Some(focus_motor_config) = self.device_config.focus_motor_config.as_mut() {
                    let old_offset = focus_motor_config.pos_offset;
                    focus_motor_config.pos_offset = match change {
                        0 => 0.0,
                        _ => {
                            old_offset
                                - FloatType::powi(10.0, i32::abs(change) - 1)
                                    * i32::signum(change) as FloatType
                                    * focus_motor_config.adjust_step
                        }
                    };
                    tracing::info!("new step offset {}", focus_motor_config.pos_offset);
                } else {
                    log::error!("can't adjust distance correction - focus config missing");
                };
            }
            FloCommand::SetDisplaySource(source) => {
                // Publishing to the watch is the whole action: the camshow link
                // tells camshow what to display, and the camera host's relay
                // starts or stops sending that camera's frames. The recording is
                // untouched by design — it is always the clean webcam.
                tracing::info!("{src:?} selected display source {source:?}");
                self.my_state.display_source = source;
                if self.display_source_tx.send(source).is_err() {
                    tracing::warn!(
                        "nothing is listening for the display source; is camshow configured?"
                    );
                }
                self.from_device_http_tx.send(self.my_state.clone())?;
            }
            FloCommand::AddRtpTarget {
                target,
                bitrate_kbps,
            } => {
                let addr = match target.parse() {
                    Ok(target) => target,
                    Err(e) => {
                        tracing::warn!(%target, "ignoring invalid RTP target: {e}");
                        return Ok(());
                    }
                };
                if bitrate_kbps == 0 {
                    tracing::warn!(%target, "ignoring zero RTP bitrate");
                    return Ok(());
                }
                if !self
                    .my_state
                    .rtp_targets
                    .iter()
                    .any(|config| config.target.addr == addr)
                {
                    self.my_state
                        .rtp_targets
                        .push(flo_core::RtpTargetConfig::new(flo_core::RtpTarget {
                            addr,
                            bitrate_kbps,
                        }));
                    // Adding a destination is a request to send to it, so it
                    // takes effect even if sending was switched off. Otherwise
                    // the new target would sit there doing nothing, looking like
                    // the add had failed.
                    self.my_state.rtp_send_enabled = true;
                    self.publish_rtp_targets()?;
                }
            }
            FloCommand::SetRtpTargetBitrate {
                target,
                bitrate_kbps,
            } => {
                let addr = match target.parse() {
                    Ok(target) => target,
                    Err(e) => {
                        tracing::warn!(%target, "ignoring invalid RTP target: {e}");
                        return Ok(());
                    }
                };
                if bitrate_kbps == 0 {
                    tracing::warn!(%target, "ignoring zero RTP bitrate");
                    return Ok(());
                }
                if let Some(config) = self
                    .my_state
                    .rtp_targets
                    .iter_mut()
                    .find(|config| config.target.addr == addr)
                    && config.target.bitrate_kbps != bitrate_kbps
                {
                    config.target.bitrate_kbps = bitrate_kbps;
                    self.publish_rtp_targets()?;
                }
            }
            FloCommand::RemoveRtpTarget(target) => {
                let target = match target.parse() {
                    Ok(target) => target,
                    Err(e) => {
                        tracing::warn!(%target, "ignoring invalid RTP target: {e}");
                        return Ok(());
                    }
                };
                let old_len = self.my_state.rtp_targets.len();
                self.my_state
                    .rtp_targets
                    .retain(|candidate| candidate.target.addr != target);
                if self.my_state.rtp_targets.len() != old_len {
                    self.publish_rtp_targets()?;
                }
            }
            FloCommand::SetRtpTargetEnabled { target, enabled } => {
                let addr = match target.parse() {
                    Ok(target) => target,
                    Err(e) => {
                        tracing::warn!(%target, "ignoring invalid RTP target: {e}");
                        return Ok(());
                    }
                };
                if let Some(config) = self
                    .my_state
                    .rtp_targets
                    .iter_mut()
                    .find(|config| config.target.addr == addr)
                    && config.enabled != enabled
                {
                    config.enabled = enabled;
                    self.publish_rtp_targets()?;
                }
            }
            FloCommand::SetRtpSendEnabled(enable) => {
                if self.my_state.rtp_send_enabled != enable {
                    self.my_state.rtp_send_enabled = enable;
                    self.publish_rtp_targets()?;
                }
            }
            FloCommand::SetRtpTargets(targets) => {
                // Camshow reports the streams it is sending, so they are all
                // enabled by definition.
                self.my_state.rtp_targets = canonical_rtp_targets(
                    targets
                        .into_iter()
                        .map(flo_core::RtpTargetConfig::new)
                        .collect(),
                );
                if self.my_state.rtp_send_enabled {
                    self.from_device_http_tx.send(self.my_state.clone())?;
                } else {
                    // This is camshow reporting the destinations it started
                    // with, which it does once per connection. With sending
                    // switched off they have to be taken away from it again, or
                    // a camshow restart would quietly resume streaming.
                    self.publish_rtp_targets()?;
                }
            }
            FloCommand::SetRecordingState(enable) => {
                if enable {
                    // Normal recording: start from now, ignoring the
                    // pre-capture buffer.
                    self.start_recording(false).await?;
                } else {
                    self.stop_recording().await?;
                }
            }
            FloCommand::SetRecordTrackingCamMp4(enable) => {
                self.my_state.record_tracking_cam_mp4 = enable;
                self.from_device_http_tx.send(self.my_state.clone())?;
            }
            FloCommand::StartPreCaptureRecording => {
                // Post-trigger: start a recording that also includes the
                // buffered pre-capture window, on the `.floz` and (if tied)
                // on the tracking cameras' MP4s. Stopping uses the normal
                // SetRecordingState(false) path.
                self.start_recording(true).await?;
            }
            FloCommand::SetPreCaptureSeconds(secs) => {
                let secs = secs.max(0.0);
                tracing::info!("Setting pre-capture buffer to {secs} seconds");
                self.my_state.precapture_window_secs = secs;
                if secs <= 0.0 {
                    // Buffer is being emptied; reflect that immediately rather
                    // than waiting for the next slow tick.
                    self.my_state.precapture_buffered_secs = 0.0;
                }
                self.flo_saver_tx
                    .send(SaveToDiskMsg::SetPreCaptureSeconds(secs))
                    .unwrap();
                self.from_device_http_tx.send(self.my_state.clone())?;
                // Mirror the window onto the tracking cameras' own post-trigger
                // buffers so their video can be pre-captured too.
                self.set_cam_precapture_buffer(secs).await;
                // And onto camshow, which buffers webcam frames for the same
                // reason. Ignore a send error: with no camshow configured
                // nothing holds the receiver.
                let _ = self.camshow_precapture_secs_tx.send(secs);
            }
        };
        Ok(())
    }

    /// Tell camshow which streams to send, and the BUI what FLO now holds.
    ///
    /// While sending is disabled camshow is given an empty list: it is the same
    /// message it already understands, so nothing about the protocol or camshow
    /// changes, and the destinations stay in `my_state` ready to be switched
    /// back on.
    fn publish_rtp_targets(&self) -> Result<()> {
        self.rtp_targets_tx
            .send(rtp_targets_to_send(&self.my_state))?;
        self.from_device_http_tx.send(self.my_state.clone())?;
        Ok(())
    }

    async fn handle_drone_event(&mut self, evt: DroneEvent) -> Result<()> {
        use DroneEvent as E;
        match evt {
            E::Armed | E::Disarmed => {
                let want_recording = evt == E::Armed;
                // start/stop saving .flo data and webcam .mp4 file
                if self.my_state.floz_recording_path.is_some() != want_recording {
                    self.broadway.flo_events.send(FloEvent::Command(
                        FloCommand::SetRecordingState(want_recording),
                        CommandSource::DroneRC,
                    ))?;
                }

                // Start/stop saving the .mp4 files on the tracking cameras.
                // When they are tied to the `.floz`, the recording command
                // above already owns them — and does it better, since a
                // pre-capture window then starts the video in the past too.
                if !self.my_state.record_tracking_cam_mp4 {
                    self.send_cam_arg_to_all(strand_cam_remote_control::CamArg::SetIsRecordingMp4(
                        want_recording,
                    ))
                    .await;
                }
            }
            _ => {}
        };
        Ok(())
    }
    /// handle mode switching
    fn switch_to_mode(&mut self, new_mode: DeviceMode, reason: ModeChangeReason) -> Result<()> {
        let tracking_state = &mut self.tracking_state;
        let old_mode = tracking_state.mode;

        log::info!("Switching to {new_mode:?}.");

        use flo_core::DeviceMode::*;

        //handle entering modes
        match new_mode {
            ManualOpenLoop => {}
            ClosedLoop => {
                // Reset the error
                // integrals. Set them such that they will keep us at the
                // present motor position in the absense of error from the
                // sensor.
                // ?? is it necessary? --Victor
                let cfg = &self.device_config;
                tracking_state.pan_err_integral = tracking_state.pan.0 / cfg.ki_pan_angle;
                tracking_state.tilt_err_integral = tracking_state.tilt.0 / cfg.ki_tilt_angle;
            }
            SuspendedClosedLoop => {
                // reset kalman filter, as we have been trying to follow it
                // for some time and got no observations, that's why we are
                // switching to SuspendedClosedLoop
                tracking_state.kalman_estimates = None;
                tracking_state.kalman_estimates_distance = None;
            }
            AcquiringLock => {
                // reset kalman filter, so that we don't end up trying to
                // track an object that is long gone
                tracking_state.kalman_estimates = None;
                tracking_state.kalman_estimates_distance = None;
                tracking_state.last_observation = None;
            }
        }

        tracking_state.mode = new_mode;
        self.my_state.mode = new_mode;

        self.broadway
            .flo_events
            .send(FloEvent::ModeChanged((old_mode, new_mode, reason)))?;

        play_sound(new_mode, self.device_config, self.audio_stream);

        self.from_device_http_tx.send(self.my_state.clone())?;
        Ok(())
    }
}

fn is_stale(stamp: Option<&std::time::Instant>) -> bool {
    if let Some(stamp) = stamp {
        stamp.elapsed() > std::time::Duration::from_secs(1)
    } else {
        true
    }
}

/// How old a stereo distance may be when a control step picks it up.
///
/// Pairing tolerates a secondary camera whose frames are stuck a few triggers
/// behind, because a late measurement is worth more than none. There is a
/// limit to that: past it, the subject has moved far enough that feeding the
/// measurement to the filter as if it were current does more harm than leaving
/// the distance to coast on the filter's own prediction. Ten frame intervals
/// is well beyond any lag seen in practice while still ruling out data from a
/// camera that has effectively stopped.
fn stereopsis_max_age(fps: Option<f64>) -> chrono::TimeDelta {
    const FALLBACK: chrono::TimeDelta = chrono::TimeDelta::milliseconds(100);
    match fps {
        Some(fps) if fps > 0.0 => chrono::TimeDelta::nanoseconds((10.0 / fps * 1e9) as i64),
        _ => FALLBACK,
    }
}

/// A secondary camera is expected whenever the embedded host registered one.
fn secondary_camera_is_stale(
    secondary_cam_name: Option<&str>,
    stamp: Option<&std::time::Instant>,
) -> bool {
    secondary_cam_name.is_some() && is_stale(stamp)
}

/// Send a camera command over an in-process extension channel. A closed
/// receiver means the extension stopped unexpectedly; log it and let the
/// controller continue shutting down.
async fn send_in_process_cam_arg(
    tx: &mpsc::Sender<strand_cam_remote_control::CamArg>,
    arg: strand_cam_remote_control::CamArg,
) {
    if let Err(error) = tx.send(arg).await {
        tracing::warn!(%error, "Ignoring error sending command to in-process camera");
    }
}

struct InitializedStrandCams {
    secondary_cam_name: Option<String>,
    embedded: Vec<CameraRegistration>,
    embedded_routers: flo_webserver::EmbeddedStrandCamRouters,
}

impl InitializedStrandCams {
    fn proxy_info(&self) -> Vec<flo_core::StrandCamProxyInfo> {
        self.embedded
            .iter()
            .map(|camera| flo_core::StrandCamProxyInfo::new(camera.role, camera.name.clone()))
            .collect()
    }

    fn embedded_routers(&self) -> flo_webserver::EmbeddedStrandCamRouters {
        self.embedded_routers.clone()
    }
}

/// Validate embedded camera registrations and return the secondary identity.
fn embedded_secondary_camera_name(embedded: &[CameraRegistration]) -> Result<Option<String>> {
    for (index, camera) in embedded.iter().enumerate() {
        if camera.name.is_empty() {
            eyre::bail!("embedded {:?} camera has an empty name", camera.role);
        }

        for other in &embedded[..index] {
            if camera.role == other.role {
                eyre::bail!(
                    "multiple embedded cameras registered for the {:?} role: {:?} and {:?}",
                    camera.role,
                    other.name,
                    camera.name,
                );
            }
            if camera.name == other.name {
                eyre::bail!(
                    "embedded camera name {:?} was registered for both {:?} and {:?}",
                    camera.name,
                    other.role,
                    camera.role,
                );
            }
        }
    }

    Ok(embedded
        .iter()
        .find(|camera| camera.role == StrandCamRole::Secondary)
        .map(|camera| camera.name.clone()))
}

async fn init_strand_cams(
    mut embedded: Vec<CameraRegistration>,
    mut camera_host_task: Option<&mut JoinHandle<Result<()>>>,
) -> Result<InitializedStrandCams> {
    let secondary_cam_name = embedded_secondary_camera_name(&embedded)?;

    let mut embedded_routers = flo_webserver::EmbeddedStrandCamRouters::new();
    for camera in &mut embedded {
        let Some(router_rx) = camera.router_rx.take() else {
            continue;
        };
        let router = match router_rx.await {
            Ok(router) => router,
            Err(_) => {
                let context = format!(
                    "embedded Strand Camera {:?} stopped before providing its browser router",
                    camera.name
                );
                let Some(camera_host_task) = camera_host_task.as_deref_mut() else {
                    return Err(eyre::eyre!(context));
                };
                return match camera_host_task.await {
                    Ok(Ok(())) => Err(eyre::eyre!(context)),
                    Ok(Err(error)) => Err(error).wrap_err(context),
                    Err(error) => Err(error).wrap_err(context),
                };
            }
        };
        if embedded_routers
            .insert(camera.name.clone(), router)
            .is_some()
        {
            eyre::bail!(
                "multiple embedded Strand Cameras reported the name {:?}",
                camera.name
            );
        }
    }

    Ok(InitializedStrandCams {
        secondary_cam_name,
        embedded,
        embedded_routers,
    })
}

trait BroadwaySend {
    fn bsend(&self, msg: flo_core::BMsg) -> eyre::Result<()>;
}

impl BroadwaySend for mpsc::UnboundedSender<SaveToDiskMsg> {
    fn bsend(&self, msg: flo_core::BMsg) -> eyre::Result<()> {
        let stamp = chrono::Local::now();
        let wrapped = SaveToDiskMsg::BroadwaySaveToDiskMsg(StampedBMsg { stamp, msg });
        self.send(wrapped).map_err(Into::into)
    }
}

/// Options used to compose extra subsystems into the FLO application.
pub struct AppOptions {
    /// First-class in-process owner of camera acquisition. Unlike an
    /// [`Extension`], a camera host registers camera identities and receives
    /// camera-specific lifecycle resources.
    pub camera_host: Option<Box<dyn CameraHost>>,
    /// Long-running subsystems that join the supervisor select loop.
    pub extensions: Vec<Box<dyn Extension>>,
    /// OSD overlays drawn each render tick after the base layer.
    pub osd_overlays: Vec<Box<dyn OsdOverlay + Send + Sync>>,
    /// Capacity of the bounded in-process centroid input queue.
    pub centroid_input_capacity: usize,
    /// What the binary composing FLO was built from, if it can say.
    ///
    /// A binary that composes FLO lives in its own repository, and the revision
    /// baked into these crates was fixed when *they* were compiled: it
    /// describes FLO's tree, not the one the operator built and is about to
    /// fly. Only the composing crate can know its own, because a build script's
    /// `cargo:rustc-env` reaches just the crate it belongs to, so this is
    /// passed in rather than discovered here.
    ///
    /// It joins FLO's own entry and any the extensions report, and the list is
    /// printed by `--version`, written into every `.floz` and shown in the web
    /// UI's footer. `None` means only FLO and the extensions are described --
    /// which for a recording is the difference between knowing what produced
    /// it and guessing.
    ///
    /// ```no_run
    /// // In the composing crate, whose build script sets GIT_HASH and GIT_DIRTY:
    /// let options = flo::AppOptions {
    ///     version: Some(flo_core::ComponentVersion {
    ///         name: env!("CARGO_PKG_NAME").to_owned(),
    ///         version: env!("CARGO_PKG_VERSION").to_owned(),
    ///         git_revision: env!("GIT_HASH").to_owned(),
    ///         dirty: match env!("GIT_DIRTY") {
    ///             "true" => Some(true),
    ///             "false" => Some(false),
    ///             _ => None,
    ///         },
    ///     }),
    ///     ..Default::default()
    /// };
    /// # let _ = options;
    /// ```
    pub version: Option<flo_core::ComponentVersion>,
}

/// FLO's own entry, from this crate's build script.
pub fn flo_component_version() -> flo_core::ComponentVersion {
    flo_core::ComponentVersion {
        name: "flo".to_owned(),
        // Not `CARGO_PKG_VERSION`: the build script appends the revision to
        // that, and this type keeps the two apart.
        version: env!("CARGO_PKG_VERSION")
            .split('+')
            .next()
            .unwrap_or("unknown")
            .to_owned(),
        git_revision: env!("GIT_HASH").to_owned(),
        dirty: match env!("GIT_DIRTY") {
            "true" => Some(true),
            "false" => Some(false),
            _ => None,
        },
    }
}

/// Every software component in this process that can say what it was built
/// from: FLO, the binary composing it, and any extension that reports one.
///
/// Order is deliberate and stable across runs, so the `.floz` record and the
/// footer read the same way every time: the composing binary first, because
/// that is the program the operator actually ran, then FLO, then extensions in
/// registration order. With no composing binary, FLO leads.
pub fn component_versions(options: &AppOptions) -> Vec<flo_core::ComponentVersion> {
    let mut versions: Vec<_> = options.version.clone().into_iter().collect();
    versions.push(flo_component_version());
    versions.extend(options.extensions.iter().filter_map(|ext| ext.version()));
    versions
}

impl Default for AppOptions {
    fn default() -> Self {
        Self {
            camera_host: None,
            extensions: Vec::new(),
            osd_overlays: Vec::new(),
            centroid_input_capacity: DEFAULT_CENTROID_INPUT_CAPACITY,
            version: None,
        }
    }
}

/// Run the flo application.
///
/// Parses CLI args, loads configuration, sets up logging, and runs the
/// tokio main loop. Returns when the program exits.
pub fn run(options: AppOptions) -> Result<()> {
    run_with_args(options, std::env::args_os())
}

/// Run FLO using explicit command-line arguments.
///
/// Composition binaries use this to reserve their own arguments while still
/// forwarding FLO's normal CLI unchanged. `args` must include a program name.
pub fn run_with_args<I, T>(options: AppOptions, args: I) -> Result<()>
where
    I: IntoIterator<Item = T>,
    T: Into<std::ffi::OsString> + Clone,
{
    run_with_args_inner(options, args, None, None)
}

/// Run FLO using explicit command-line arguments and an already parsed
/// controller configuration.
///
/// This is intended for composition binaries that own configuration parsing
/// (for example, a camera host with its own settings). If `args` includes
/// `--config`, the supplied configuration takes precedence.
///
/// `raw_config_source`, if given, is the verbatim text of the configuration
/// file the caller parsed `config` from (including any sections, such as
/// `flo-strand-cam:`, that are not part of `FloControllerConfig` itself). It
/// is recorded alongside the effective configuration in saved `.floz` files
/// so no part of the original configuration is lost.
pub fn run_with_args_and_config<I, T>(
    options: AppOptions,
    args: I,
    config: FloControllerConfig,
    raw_config_source: Option<String>,
) -> Result<()>
where
    I: IntoIterator<Item = T>,
    T: Into<std::ffi::OsString> + Clone,
{
    run_with_args_inner(options, args, Some(config), raw_config_source)
}

fn run_with_args_inner<I, T>(
    options: AppOptions,
    args: I,
    supplied_config: Option<FloControllerConfig>,
    mut raw_config_source: Option<String>,
) -> Result<()>
where
    I: IntoIterator<Item = T>,
    T: Into<std::ffi::OsString> + Clone,
{
    let cli = parse_cli(args);
    // Computed before `options` is handed on and its extensions consumed.
    let component_versions = component_versions(&options);
    let (log_dir, data_dir) = if let Some(dd) = cli.data_dir.as_ref() {
        (dd.clone(), dd.clone())
    } else {
        let home_dir = home::home_dir().unwrap();
        let home_dir = camino::Utf8PathBuf::from_path_buf(home_dir).unwrap();
        (home_dir.clone(), home_dir.join("flo-data"))
    };

    std::fs::create_dir_all(&log_dir)
        .with_context(|| format!("While creating directory {log_dir}"))?;
    std::fs::create_dir_all(&data_dir)
        .with_context(|| format!("While creating directory {data_dir}"))?;

    flo_tracing::init_tracing(&log_dir, "flo")?;

    // What produced this run, said before anything else happens. Each `.floz`
    // carries the same list, but a session that records nothing -- or that
    // fails before its first recording, which is exactly when the question
    // gets asked -- leaves the log as the only account of which build ran.
    // Same wording as `--version`, one component per line so it greps.
    for component in &component_versions {
        tracing::info!("component: {component}");
    }

    let device_id = match get_device_id() {
        Ok(device_id) => device_id,
        Err(e) => {
            log::warn!("No device id found ({e}), using random number.");
            DeviceId::new(rand::random())
        }
    };
    // Create our device state.
    let mut my_state = flo_core::DeviceState::new(device_id);

    let mut device_config = match supplied_config {
        Some(config) => {
            if let Some(device_config_fname) = &cli.config {
                tracing::debug!(
                    "using caller-supplied configuration parsed from {device_config_fname:?}"
                );
            }
            extension::validate_config(config, &options.extensions)?
        }
        None => {
            if let Some(device_config_fname) = &cli.config {
                tracing::debug!("Loading config file: \"{device_config_fname}\"");
                for name in options.extensions.iter().map(|e| e.name()) {
                    tracing::debug!("extension name: \"{name}\"");
                }

                log::info!("Reading initial device config from: {device_config_fname}");
                let cfg_buf = std::fs::read_to_string(device_config_fname)
                    .with_context(|| format!("opening file {device_config_fname}"))?;
                raw_config_source = Some(cfg_buf.clone());

                // Parse the YAML but raise on unknown fields, respecting extensions.
                let raw_cfg = serde_yaml::from_str(&cfg_buf)
                    .with_context(|| format!("while parsing YAML in file {device_config_fname}"))?;
                extension::validate_config(raw_cfg, &options.extensions)?
            } else {
                log::info!("Loading default device config.");
                // (TODO: why doesn't the default value suffice?)
                my_state.mode = DeviceMode::ManualOpenLoop;
                // each pixel is 1/20th of a degree
                let pixel_gain = Angle::from_degrees(1.0 / 20.0).0;
                FloControllerConfig {
                    pwm_output_enabled: true,
                    ki_pan_angle: -10.0,
                    ki_tilt_angle: 10.0,
                    centroid_to_sensor_x_angle_func: flo_core::CentroidToAngleCalibration {
                        dx_gain: pixel_gain,
                        dy_gain: 0.0,
                        offset: 0.0,
                    },
                    centroid_to_sensor_y_angle_func: flo_core::CentroidToAngleCalibration {
                        dx_gain: 0.0,
                        dy_gain: pixel_gain,
                        offset: 0.0,
                    },
                    ..Default::default()
                }
            }
        }
    };

    // Bring an older config file up to the current schema. Anything whose
    // meaning changed is converted here rather than silently reinterpreted, and
    // every conversion is warned about so the operator knows to update the file.
    for note in flo_core::migrate_config(&mut device_config)? {
        tracing::warn!("migrating config: {note}");
    }

    // The operator can retie or untie the tracking cameras' MP4 recordings in
    // the BUI; the config supplies only the value FLO starts with.
    my_state.record_tracking_cam_mp4 = device_config.record_tracking_cam_mp4_with_floz;

    // Set the initial home from the motor neutral positions. The home
    // position can be updated by the operator.
    my_state.home_position = (
        device_config.pan_motor_config.neutral_position,
        device_config.tilt_motor_config.neutral_position,
        device_config
            .focus_motor_config
            .as_ref()
            .map(|x| x.home_position)
            .unwrap_or_else(|| flo_core::RadialDistance(2.0)),
    );

    //make sure osd config is given.
    //(here, not later, so that it is printed to config dump)
    if let Some(port_path) = cli.osd.as_ref() {
        if let Some(osd_config) = device_config.osd_config.as_mut() {
            osd_config.port_path = Some(port_path.clone());
        } else {
            device_config.osd_config = Some(flo_core::OsdConfig {
                port_path: Some(port_path.clone()),
                cal: None,
                blob: Default::default(),
                camshow_addr: None,
                camshow_preview_addr: None,
                camshow_mp4_cfg: None,
            });
        }
    };
    if let Some(cfg) = device_config.osd_config.as_mut()
        && cfg.cal.is_none()
    {
        log::warn!(
            "osd device is provided but osd configuration is missing. Using built-in defaults."
        );
        cfg.cal = Some(flo_core::FpvCameraOSDCalibration::default());
    }

    // CLI `--camshow` overrides any address from the config file. If only
    // the CLI is provided and there is no `osd_config`, synthesize a minimal
    // one so the camshow link still works.
    if let Some(addr) = cli.camshow.as_ref() {
        let cfg = device_config
            .osd_config
            .get_or_insert_with(|| flo_core::OsdConfig {
                port_path: None,
                cal: Some(flo_core::FpvCameraOSDCalibration::default()),
                blob: Default::default(),
                camshow_addr: None,
                camshow_preview_addr: None,
                camshow_mp4_cfg: None,
            });
        cfg.camshow_addr = Some(addr.clone());
    }

    if let Some(gimbal_port) = &cli.gimbal {
        if device_config.gimbal_config.is_none() {
            log::warn!(
                "gimbal port provided but gimbal config is missing. Using built-in defaults."
            );
            device_config.gimbal_config = Some(GimbalConfig::default());
        }
        device_config.gimbal_config.as_mut().unwrap().port_path = gimbal_port.clone();
    }

    //make sure some focus config is present, so that we can unwrap() everywhere
    if device_config.focus_motor_config.is_none() {
        device_config.focus_motor_config = Some(Default::default());
    }
    if let Some(trinamic_motor) = &cli.trinamic_focus {
        if let FocusMotorType::Trinamic(tcfg) =
            &mut device_config.focus_motor_config.as_mut().unwrap().motor
        {
            tcfg.port = trinamic_motor.clone();
        } else {
            eyre::bail!("focus motor type conflict");
        };
    }

    let cfg_pretty = serde_yaml::to_string(&device_config)?;
    log::info!("device config:\n{cfg_pretty}");

    match cli.command {
        Some(Commands::ShowConfig) => {
            return Ok(());
        }
        Some(Commands::Run) | None => {} // continue
    };

    // We never need to ignore Ctrl-C or run a GUI on the main thread now
    // that the webcam viewer lives in the separate `camshow` process. Run
    // the tokio runtime directly.
    let (_shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel::<()>();

    let mavlink_port = if let Some(mavlink_cfg) = &device_config.mavlink_config {
        Some(flo_mavlink::MavlinkPort::open(mavlink_cfg)?)
    } else {
        None
    };

    run_tokio_main(
        cli,
        my_state,
        device_config,
        raw_config_source,
        component_versions,
        shutdown_rx,
        &data_dir,
        mavlink_port,
        options,
    )?;
    Ok(())
}

fn parse_cli<I, T>(args: I) -> Cli
where
    I: IntoIterator<Item = T>,
    T: Into<std::ffi::OsString> + Clone,
{
    Cli::parse_from(args)
}

/// The help text clap would print for `args`, if `args` asks for help.
///
/// `Some` only for a genuine `--help`/`-h`; anything else is `None`, including
/// a malformed command line, which the real parse further down reports in its
/// own words.
///
/// Exists because a composed binary has to answer help before it can load a
/// configuration file, while the flags being described live here. Rendering
/// through clap rather than writing the text out again keeps one description of
/// the command line, and passing the whole argument list rather than asking for
/// the top-level page means `flo <command> --help` lands on the right one.
pub fn cli_help<I, T>(args: I) -> Option<String>
where
    I: IntoIterator<Item = T>,
    T: Into<std::ffi::OsString> + Clone,
{
    match Cli::try_parse_from(args) {
        Err(e) if e.kind() == clap::error::ErrorKind::DisplayHelp => Some(e.render().to_string()),
        _ => None,
    }
}

/// Create the tokio Runtime and call the main app loop.
#[expect(clippy::too_many_arguments)]
fn run_tokio_main(
    cli: Cli,
    my_state: DeviceState,
    device_config: FloControllerConfig,
    raw_config_source: Option<String>,
    component_versions: Vec<flo_core::ComponentVersion>,
    shutdown_rx: tokio::sync::oneshot::Receiver<()>,
    data_dir: &camino::Utf8Path,
    mavlink_port: Option<flo_mavlink::MavlinkPort>,
    options: AppOptions,
) -> Result<()> {
    let tokio_rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;

    let handle = tokio_rt.handle();
    // Some camera backends are thread-affine and therefore expose `!Send`
    // futures. A LocalSet lets in-process integrations keep those tasks on
    // this caller-owned FLO runtime, while ordinary extensions continue to
    // use the multi-thread runtime normally.
    let local = tokio::task::LocalSet::new();
    local.block_on(
        &tokio_rt,
        app_main(
            handle,
            cli,
            my_state,
            device_config,
            raw_config_source,
            component_versions,
            shutdown_rx,
            data_dir,
            mavlink_port,
            options,
        ),
    )
}

/// Report a supervised subsystem that resolved, and turn its outcome into the
/// main loop's.
///
/// Every subsystem selected on in [`app_main`]'s loop is expected to outlive
/// that loop, so one that resolves — with an error, with a panic, or even with
/// `Ok(())` — is a shutdown of the whole application. Left unnamed, the only
/// thing an operator sees is the *downstream* effects: the embedded cameras
/// log a clean "ending nicely", the writer closes its `.floz`, and nothing
/// says which subsystem went first. Name it here, at the point where the
/// decision to stop is actually made.
fn subsystem_resolved(
    name: &str,
    result: std::result::Result<Result<()>, tokio::task::JoinError>,
) -> Result<()> {
    match result {
        Ok(Ok(())) => {
            // Not an error, but not expected either: nothing here is supposed
            // to finish on its own while FLO is running.
            tracing::warn!("{name} stopped without an error. FLO is shutting down.");
            Ok(())
        }
        Ok(Err(error)) => {
            // Logged as well as returned. The returned report reaches the
            // process exit status, but a deployment that reads only this log
            // should not have to infer the cause from a bare exit code.
            tracing::error!("{name} failed. FLO is shutting down. {error:?}");
            Err(error.wrap_err(format!("{name} failed")))
        }
        Err(error) => {
            tracing::error!("{name} did not shut down cleanly. FLO is shutting down. {error}");
            Err(eyre::Report::new(error).wrap_err(format!("{name} did not shut down cleanly")))
        }
    }
}

/// The highest-level main loop for the flo app.
#[expect(clippy::too_many_arguments)]
async fn app_main(
    handle: &tokio::runtime::Handle,
    cli: Cli,
    mut my_state: DeviceState,
    mut device_config: FloControllerConfig,
    raw_config_source: Option<String>,
    component_versions: Vec<flo_core::ComponentVersion>,
    mut shutdown_rx: tokio::sync::oneshot::Receiver<()>,
    data_dir: &camino::Utf8Path,
    mavlink_port: Option<flo_mavlink::MavlinkPort>,
    options: AppOptions,
) -> Result<()> {
    if let Some(h264_recording_cfg) = &device_config.highmag_visible_recorder {
        // Make a short test movie to check everything is working.
        let output_root = tempfile::tempdir().unwrap(); // will cleanup on drop
        let file1 = output_root.path().join("highmag_test.mp4");

        let recorder = h264_recorder::H264Recorder::new(
            camino::Utf8Path::from_path(&file1).unwrap(),
            &h264_recording_cfg.ffmpeg_cli,
        )?;
        std::thread::sleep(std::time::Duration::from_secs(10));
        recorder.close()?;
        if !std::fs::exists(&file1)? {
            eyre::bail!("Test video file was not created: {}", file1.display());
        }
        let metadata = std::fs::metadata(&file1)?;
        if metadata.len() == 0 {
            eyre::bail!("Test video file is empty: {}", file1.display());
        }
        tracing::debug!("Test video file created: {}", file1.display());
    }

    let (flo_saver_tx, flo_saver_rx) = mpsc::unbounded_channel();

    // Reports how many seconds of data the writer currently holds in its
    // pre-capture buffer, so the BUI can display it.
    let (precapture_buffered_tx, precapture_buffered_rx) = watch::channel(0.0f64);

    let _write_closer = writing_state::WriteCloser::new(flo_saver_tx.clone());

    //delivers drone status from mavlink thread to flo main loop
    let broadway = Broadway::new(100, 10_000);

    //dummy subscriptions to prevent send errors
    //FIXME: find a better way to do this than a stalled dummy receiver
    let _rx1 = broadway.flo_events.subscribe();
    let _rx2 = broadway.flo_detections.subscribe();
    let _rx5 = broadway.drone_realtime.subscribe();

    let mut drone_event_rx_stream =
        tokio_stream::wrappers::BroadcastStream::new(broadway.drone_events.subscribe());

    let mut flo_events_rx_stream =
        tokio_stream::wrappers::BroadcastStream::new(broadway.flo_events.subscribe());

    let local_flo_state = LocalFloState::default();
    let local_flo_state2 = local_flo_state.clone();

    // `None` when this deployment has no flight controller, which is what makes
    // an extension asking to talk to one fail at spawn rather than silently
    // sending into nothing.
    let mut autopilot_link: Option<flo_mavlink::AutopilotLink> = None;

    let mut mavlink_task: Pin<Box<dyn Future<Output = _>>> =
        if let Some(mavlink_port) = mavlink_port {
            let (mavlink_task_jh, link) = flo_mavlink::spawn_mavlink(
                handle,
                broadway.clone(),
                flo_saver_tx.clone(),
                device_config.rc_config.as_ref(),
                mavlink_port,
                local_flo_state2,
            )?;
            tracing::debug!("mavlink task spawned");
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            if mavlink_task_jh.is_finished() {
                mavlink_task_jh.await??;
                unreachable!(
                    "The mavlink task is already done, so we expected an error. \
                However, we never got an error. So we are confused and will now panic."
                );
            } else {
                autopilot_link = Some(link);
                Box::pin(mavlink_task_jh)
            }
        } else {
            tracing::info!("no mavlink");
            Box::pin(pending())
        };

    let trinamic_microsteps = device_config
        .focus_motor_config
        .as_ref()
        .map(|cfg| cfg.park_position)
        .unwrap_or(f64::NAN);

    // Create channel for communicating to motors. Note that this might
    // communicate to multiple motors such as pan, tilt, and focus motors. The
    // different motor types all listen to this channel and react appropriately.
    // On the sender side, it can be updated as often as wished. It is the motor
    // (receiver) side's job to react as quickly as possible but not faster.
    let (motors_tx, motors_rx) = watch::channel(MotorValueCache {
        pan: device_config.pan_motor_config.neutral_position,
        tilt: device_config.tilt_motor_config.neutral_position,
        vpan: 0.0,
        vtilt: 0.0,
        drivemode: flo_core::MotorDriveMode::Position,
        rel_frame: true,
        focus: trinamic_microsteps,
    });
    let initial_motor_command = motors_rx.borrow().clone();
    let (processing_feedback_tx, processing_feedback_rx) = watch::channel(ProcessingFeedback {
        updated_at: chrono::Utc::now(),
        motor_readout: None,
        motor_command: initial_motor_command,
        tracking_mode: DeviceMode::default(),
    });

    // Create channel for motor position feedback.
    let (motor_position_tx, mut motor_position_rx) = mpsc::channel::<MotorPositionResult>(10);
    let mut motor_position_tx = Some(motor_position_tx);

    let mut converter_handle: JoinHandle<eyre::Result<()>> = {
        let mut rx1 = broadway.flo_events.subscribe();
        let mut rx2 = broadway.flo_detections.subscribe();
        let mut rx4 = broadway.drone_events.subscribe();
        let mut rx5 = broadway.drone_realtime.subscribe();

        let flo_saver_tx = flo_saver_tx.clone();
        handle.spawn(async move {
            use flo_core::BMsg::*;
            loop {
                tokio::select! {
                    rx1_res = rx1.recv() => {
                        flo_saver_tx.bsend(FloEvent(rx1_res?))?;
                    }
                    rx2_res = rx2.recv() => {
                        flo_saver_tx.bsend(FloDetectionEvent(rx2_res?))?;
                    }
                    rx4_res = rx4.recv() => {
                        flo_saver_tx.bsend(DroneEvent(rx4_res?))?;
                    }
                    rx5_res = rx5.recv() => {
                        flo_saver_tx.bsend(DroneRealtimeEvent(rx5_res?))?;
                    }
                }
            }
        })
    };

    let mut saver_handle = {
        // Spawn task for saving data to disk
        let device_config = device_config.clone();
        let raw_config_source = raw_config_source.clone();
        let component_versions = component_versions.clone();
        handle.spawn_blocking(move || {
            writing_state::writer_task_main(
                flo_saver_rx,
                &device_config,
                raw_config_source.as_deref(),
                &component_versions,
                precapture_buffered_tx,
            )
        })
    };

    if device_config.pwm_output_enabled && cli.pwm_serial.is_none() {
        tracing::error!("PWM output enabled, but no PWM serial device specified");
    }

    // Launch task to run serial IO for rpi pico pantilt PWM motors. Gimbal
    // startup is deferred until the coordinator is ready to consume its
    // bounded motor-position channel.
    let motor_task_start: Box<dyn FnOnce() -> JoinHandle<eyre::Result<()>>>;
    let mut motor_position_tx_for_backend = motor_position_tx.clone();
    if let Some(pwm_serial) = cli.pwm_serial {
        my_state.motor_type = MotorType::PwmServo;

        let baud_rate = 115_200;

        let serial_device = tokio_serial::new(&pwm_serial, baud_rate)
            .open_native_async()
            .with_context(|| format!("Failed to open PWM serial device {}", pwm_serial))?;

        let pan_pwm_config = device_config.pan_pwm_config.clone();
        let pan_motor_config = device_config.pan_motor_config.clone();
        let tilt_pwm_config = device_config.tilt_pwm_config.clone();
        let tilt_motor_config = device_config.tilt_motor_config.clone();

        let motors_rx = motors_rx.clone();
        let handle2 = handle.clone();
        motor_task_start = Box::new(move || {
            let io_handle = handle2.clone();
            handle2.spawn(async move {
                pwm_serial_io::run_rpi_pico_pwm_serial_loop(
                    &io_handle,
                    motors_rx,
                    serial_device,
                    pan_pwm_config,
                    pan_motor_config,
                    tilt_pwm_config,
                    tilt_motor_config,
                )
                .await
            })
        });
    } else if let Some(trinamic_pan) = cli.trinamic_pan {
        my_state.motor_type = MotorType::Trinamic;
        let pan_trinamic_config = device_config.pan_trinamic_config.as_mut().unwrap();
        let tilt_trinamic_config = device_config.tilt_trinamic_config.as_mut().unwrap();

        let trinamic_tilt = cli
            .trinamic_tilt
            .ok_or_else(|| eyre::eyre!("trinamic_tilt not given but needed"))?;

        let baud_rate = 115_200;

        let mot_cfg = trinamic::MotorParameters::TMCM1240(trinamic::TMCM1240Parameters::default());
        tracing::info!("Connecting to trinamic motor for pan axis at {trinamic_pan}");
        let pan_device = trinamic::Motor::new(&trinamic_pan, baud_rate, mot_cfg.clone()).await?;
        tracing::info!("Connecting to trinamic motor for tilt axis at {trinamic_tilt}");
        let tilt_device = trinamic::Motor::new(&trinamic_tilt, baud_rate, mot_cfg.clone()).await?;
        tracing::info!("Successfully connected to trinamic motors for pan and tilt.");
        pan_trinamic_config.acceleration = Some(pan_device.config.acceleration());
        tilt_trinamic_config.acceleration = Some(tilt_device.config.acceleration());
        let mut focus_device = None;
        if let FocusMotorType::Trinamic(tcfg) =
            &mut device_config.focus_motor_config.as_mut().unwrap().motor
        {
            let mot_cfg =
                trinamic::MotorParameters::TMCM1141(trinamic::TMCM1141Parameters::default());
            focus_device = Some(trinamic::Motor::new(&tcfg.port, baud_rate, mot_cfg).await?);
            tcfg.acceleration = Some(focus_device.as_ref().unwrap().config.acceleration());
        }

        let pan_trinamic_config = pan_trinamic_config.clone();
        let tilt_trinamic_config = tilt_trinamic_config.clone();

        let motors_rx = motors_rx.clone();
        motor_task_start = Box::new(move || {
            handle.spawn(async move {
                trinamic_io::run_trinamic_loop(
                    motors_rx,
                    motor_position_tx_for_backend
                        .take()
                        .expect("motor-position sender is available for trinamic"),
                    pan_device,
                    tilt_device,
                    focus_device,
                    pan_trinamic_config,
                    tilt_trinamic_config,
                )
                .await
            })
        });
    } else if let Some(cfg) = device_config.gimbal_config.clone() {
        my_state.motor_type = MotorType::Gimbal;
        let flo_saver_tx = flo_saver_tx.clone();
        let motors_rx = motors_rx.clone();
        motor_task_start = Box::new(move || {
            let motor_position_tx = motor_position_tx
                .take()
                .expect("motor-position sender is available for gimbal");
            handle.spawn(async move {
                sbgc_gimbal::run_gimbal_loop(motors_rx, motor_position_tx, flo_saver_tx, cfg).await
            })
        });
    } else {
        tracing::warn!("No motor control method specified.");
        motor_task_start = Box::new(move || {
            handle.spawn(async {
                loop {
                    futures::future::pending().await
                }
            })
        });
    }

    let (delayed_tilta_focus_port, focusing) =
        match &device_config.focus_motor_config.as_ref().unwrap().motor {
            FocusMotorType::NoMotor => (None, false),
            FocusMotorType::Trinamic(_) => {
                if my_state.motor_type != MotorType::Trinamic {
                    eyre::bail!("trinamic focus with non-trinamic pan/tilt not yet implemented");
                }
                (None, true) // handled by the deferred Trinamic backend task
            }
            FocusMotorType::Tilta(tcfg) => (Some(tcfg.port.clone()), true),
        };
    my_state.focusing = focusing;

    if options.centroid_input_capacity == 0 {
        eyre::bail!("centroid_input_capacity must be greater than zero");
    }
    let (centroid_tx, mut centroid_rx) = centroid_input_channel(options.centroid_input_capacity);

    // Create a channel to send copies of our device state.
    let (from_device_http_tx, from_device_http_rx) = watch::channel(my_state.clone());

    // Register an in-process camera host before opening legacy Strand Camera
    // sessions, so the coordinator knows embedded camera roles from startup.
    let embedded_camera_registrations = options
        .camera_host
        .as_ref()
        .map(|host| host.registrations())
        .unwrap_or_default();

    let AppOptions {
        camera_host,
        extensions,
        osd_overlays,
        ..
    } = options;
    let (subsystem_shutdown_tx, subsystem_shutdown_rx) = watch::channel(false);

    // What the operator's live view shows. Two independent consumers need the
    // same decision: the camshow link, which tells camshow what to display, and
    // the camera host, which only relays frames for the selected camera. Created
    // here so it exists before the host is spawned below.
    let (display_source_tx, display_source_rx) = watch::channel(DisplaySource::default());
    // camshow reports its initial CLI destinations through the control link;
    // after that, this channel carries BUI edits back to camshow.
    let (rtp_targets_tx, rtp_targets_rx) = watch::channel(Vec::new());

    // The composed host owns the embedded Strand Camera servers and provides
    // their routers directly for FLO to mount under its authenticated UI.
    let mut camera_host_task = match camera_host {
        Some(host) => Some(
            host.spawn(CameraHostContext {
                centroid_tx: centroid_tx.clone(),
                processing_feedback: processing_feedback_rx.clone(),
                shutdown_rx: subsystem_shutdown_rx.clone(),
                display_source: display_source_rx.clone(),
                data_dir,
            })
            .with_context(|| "spawning camera host")?,
        ),
        None => None,
    };

    let strand_cams =
        init_strand_cams(embedded_camera_registrations, camera_host_task.as_mut()).await?;
    let embedded_strand_cam_routers = strand_cams.embedded_routers();
    let strand_cam_proxy_info = strand_cams.proxy_info();

    // Channel used to tell the web server it is shutting down. It is flipped to
    // `true` once the main loop below exits (but before this task returns and
    // the runtime drops the server task), so every connected browser is told to
    // show the "FLO has quit" screen and stop reconnecting.
    let (quit_tx, quit_rx) = watch::channel(false);

    // Create HTTP server for user interface. Load the persistent secret once:
    // it both mints the self-expiring access token in `start_listener` and
    // validates it in the auth layer inside `main_loop`.
    let persistent_secret = flo_webserver::load_persistent_secret()?;
    let trusted_networks = flo_webserver::parse_trusted_networks(&cli.trusted_networks)?;
    let (tcp_listener, token_config) =
        flo_webserver::start_listener(&cli.http_addr, &persistent_secret)
            .await
            .with_context(|| format!("Opening TCP listener at address \"{}\"", cli.http_addr))?;

    // Filled by the preview reader below and drained by the web server. The
    // preview page simply reports that there are no frames while camshow is
    // absent.
    let webcam_preview = flo_webserver::WebcamPreview::new();

    // Run web server main loop
    let _http_server_join_handle = {
        let device_config = device_config.clone();
        let webcam_preview_for_server = webcam_preview.clone();
        let component_versions_for_server = component_versions.clone();
        let event_tx = broadway.flo_events.clone();
        handle.spawn(async move {
            flo_webserver::main_loop(
                tcp_listener,
                token_config,
                persistent_secret,
                trusted_networks,
                embedded_strand_cam_routers,
                strand_cam_proxy_info,
                component_versions_for_server,
                from_device_http_rx,
                device_config,
                event_tx,
                quit_rx,
                webcam_preview_for_server,
            )
            .await
        })
    };

    let (_audio_stream, audio_stream_handle) = {
        let r = rodio::OutputStream::try_default();
        match r {
            Ok(r) => (Some(r.0), Some(r.1)),
            Err(e) => {
                // log error and then drop it.
                log::error!("error opening audio stream: {}", e);
                (None, None)
            }
        }
    };

    // Set up the camshow link if configured. If `camshow_addr` is set, the
    // OSD canvas is mirrored over TCP, webcam recording start/stop is
    // forwarded, and the display source is pushed; otherwise we feed neither.
    let camshow_addr = device_config
        .osd_config
        .as_ref()
        .and_then(|c| c.camshow_addr.clone());
    let (camshow_precapture_secs_tx, camshow_precapture_secs_rx) = watch::channel(0.0f64);
    // The preview link is independent of the control link. Always spawn its
    // reader so a camshow started later (or used only for preview) is picked up
    // without restarting FLO. It sits idle, not even connected, until the
    // preview page is opened.
    let preview_addr = device_config
        .osd_config
        .as_ref()
        .and_then(|c| c.camshow_preview_addr.clone())
        .unwrap_or_else(|| camshow_protocol::preview::DEFAULT_CAMSHOW_PREVIEW_ADDR.to_string());
    handle.spawn(webcam_preview_client::run(
        preview_addr,
        webcam_preview.clone(),
    ));

    let (canvas_tx, camshow_recording_tx, mut camshow_task) = if let Some(addr) = camshow_addr {
        let (canvas_tx, canvas_rx) = watch::channel(osd_utils::OsdCache::new(30, 16));
        let (rec_tx, rec_rx) = mpsc::unbounded_channel();
        let jh = handle.spawn(camshow_client::run(
            addr,
            canvas_rx,
            rec_rx,
            display_source_rx,
            rtp_targets_rx,
            camshow_precapture_secs_rx,
            broadway.flo_events.clone(),
        ));
        let task: Pin<Box<dyn Future<Output = _>>> = Box::pin(jh);
        (Some(canvas_tx), Some(rec_tx), task)
    } else {
        let task: Pin<Box<dyn Future<Output = _>>> = Box::pin(pending());
        (None, None, task)
    };

    // Build the OSD overlay list. Defaults from AppOptions::osd_overlays
    // first, then each registered extension contributes via
    // make_osd_overlay() if it has a companion overlay.
    let mut osd_overlays = osd_overlays;
    for ext in &extensions {
        if let Some(overlay) = ext.make_osd_overlay(&broadway, handle) {
            osd_overlays.push(overlay);
        }
    }

    //start up OSD
    let (osd_tx, mut osd_task): (_, Pin<Box<dyn Future<Output = _>>>) =
        if let Some(osd_config) = device_config.osd_config.clone() {
            let (osd_tx, osd_rx) = watch::channel(OsdState::default());
            let osd_join_handle = {
                let broadway = broadway.clone();
                let osd_config = osd_config.clone();
                let local_flo_state = local_flo_state.clone();
                let canvas_tx = canvas_tx.clone();
                tokio::spawn(async move {
                    osd::run_osd_loop(
                        osd_rx,
                        broadway,
                        osd_config,
                        local_flo_state,
                        canvas_tx,
                        osd_overlays,
                    )
                    .await
                })
            };

            (Some(osd_tx), Box::pin(osd_join_handle))
        } else {
            // OSD join handle never resolves.
            (None, Box::pin(pending()))
        };

    let mut extension_tasks: tokio::task::JoinSet<eyre::Result<()>> = tokio::task::JoinSet::new();
    for ext in extensions {
        let name = ext.name();
        let ctx = extension::ExtensionContext {
            handle,
            broadway: broadway.clone(),
            saver_tx: flo_saver_tx.clone(),
            data_dir,
            config: &device_config,
            centroid_tx: centroid_tx.clone(),
            processing_feedback: processing_feedback_rx.clone(),
            shutdown_rx: subsystem_shutdown_rx.clone(),
            autopilot_link: autopilot_link.clone(),
        };
        let jh = ext
            .spawn(ctx)
            .with_context(|| format!("spawning extension {name}"))?;
        extension_tasks.spawn(async move { jh.await? });
    }

    let fast_interval =
        std::time::Duration::from_secs_f64(device_config.control_loop_timestep_secs);
    let mut fast_tick = tokio::time::interval(fast_interval);
    fast_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut slow_tick = tokio::time::interval(std::time::Duration::from_millis(1000));
    slow_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    // Read before `device_config` is borrowed by the coordinator below.
    let configured_precapture_window_secs = device_config.precapture_window_secs;

    let mut coordinator = FloCoordinator::new(
        &mut device_config,
        flo_saver_tx.clone(),
        osd_tx,
        my_state,
        local_flo_state,
        motors_tx,
        processing_feedback_tx,
        audio_stream_handle.as_ref(),
        from_device_http_tx,
        broadway,
        camshow_recording_tx,
        display_source_tx,
        rtp_targets_tx,
        camshow_precapture_secs_tx,
        data_dir,
        precapture_buffered_rx,
        strand_cams,
    );

    // Arm the pre-capture buffer from the config before anything else runs, so
    // a post-trigger works on a FLO nobody has opened a browser on yet.
    if configured_precapture_window_secs > 0.0 {
        coordinator
            .handle_command(
                FloCommand::SetPreCaptureSeconds(configured_precapture_window_secs),
                CommandSource::Automation,
            )
            .await?;
    }

    // No motor backend is spawned until after the coordinator exists. The
    // coordinator select loop below is the consumer of motor_position_rx.
    let mut motor_task_join_handle = motor_task_start();
    let mut focus_motor_task: Pin<Box<dyn Future<Output = _>>> =
        if let Some(port) = delayed_tilta_focus_port {
            let handle2 = handle.clone();
            let motors_rx = motors_rx.clone();
            Box::pin(handle.spawn(async move {
                tilta_io::run_tilta_loop(&handle2, port.as_str(), motors_rx).await
            }))
        } else {
            Box::pin(pending())
        };

    // Main loop. Keep its result so extensions receive their shutdown signal
    // even if a subsystem returns an error rather than taking a normal break.
    let coordinator_result = async {
    loop {
        // Wait for any of a number of things to happen
        tokio::select! {
            _ = &mut shutdown_rx => {
                tracing::info!("FLO was asked to shut down.");
                break;
            }
            mavlink_result = &mut mavlink_task => {
                subsystem_resolved("The MAVLink link", mavlink_result)?;
                break;
            }
            osd_task_result = &mut osd_task => {
                subsystem_resolved("The OSD", osd_task_result)?;
                break;
            }
            Some(extension_result) = extension_tasks.join_next(), if !extension_tasks.is_empty() => {
                subsystem_resolved("An extension", extension_result)?;
                break;
            }
            camera_host_result = async {
                camera_host_task
                    .as_mut()
                    .expect("camera host task was selected only when present")
                    .await
            }, if camera_host_task.is_some() => {
                // This JoinHandle is complete; remove it before shutdown so
                // the graceful-stop path never polls it again.
                camera_host_task = None;
                subsystem_resolved("The camera host", camera_host_result)?;
                break;
            }
            motor_task_result = &mut motor_task_join_handle => {
                subsystem_resolved("The motor backend", motor_task_result)?;
                break;
            }
            focus_motor_task_result = &mut focus_motor_task => {
                subsystem_resolved("The focus motor backend", focus_motor_task_result)?;
                break;
            }
            saver_result = &mut saver_handle => {
                subsystem_resolved("The .floz writer", saver_result)?;
                break;
            }
            converter_result = &mut converter_handle => {
                subsystem_resolved("The event recorder", converter_result)?;
                break;
            }
            camshow_result = &mut camshow_task => {
                // The camshow client runs forever under normal operation;
                // if it returns it means the spawned task was cancelled.
                subsystem_resolved("The camshow link", camshow_result)?;
                break;
            }
            centroid = centroid_rx.recv() => {
                let centroid = centroid.ok_or_else(|| eyre::eyre!("centroid input channel closed"))?;
                coordinator.on_image_centroid(centroid)?;
                // TODO: we have fresh data. We should call
                // `self.on_fast_tick(dt_secs)?;` here?
            }
            flo_event = flo_events_rx_stream.next() => {
                let flo_event = flo_event.ok_or_else(|| eyre::eyre!("FLO events channel closed"))??;
                coordinator.handle_event(flo_event).await?;
            }
            motor_position = motor_position_rx.recv() => {
                let motor_position = motor_position.ok_or_else(|| eyre::eyre!("motor position channel closed"))?;
                coordinator.on_new_motor_position(motor_position)?;
            }
            _ = fast_tick.tick() => {
                let dt_secs = fast_tick.period().as_secs_f64() as FloatType;
                coordinator.on_fast_tick(dt_secs)?;
            }
            _ = slow_tick.tick() => {
                coordinator.on_slow_tick()?;
            }
            evt = drone_event_rx_stream.next() => {
                let evt = evt.ok_or_else(|| eyre::eyre!("drone events channel closed"))??;
                coordinator.handle_drone_event(evt).await?;
            }
        };
    }
    Ok::<(), eyre::Error>(())
    }
    .await;
    // At INFO, and with the error, because everything the operator sees after
    // this point is a *consequence* of it: cameras stopping, the recording
    // ending, browsers being told FLO has quit.
    match &coordinator_result {
        Ok(()) => tracing::info!("FLO is stopping its subsystems."),
        Err(error) => tracing::error!("FLO is stopping its subsystems after an error. {error:?}"),
    }

    // Give extensions a bounded opportunity to release external resources
    // before the runtime cancels their remaining tasks. This is especially
    // important for in-process camera acquisition, which needs to stop its
    // frame task before its vendor module is dropped.
    let _ = subsystem_shutdown_tx.send(true);
    let extension_shutdown_deadline = std::time::Duration::from_secs(2);
    if let Some(camera_host_task) = camera_host_task.as_mut()
        && tokio::time::timeout(extension_shutdown_deadline, &mut *camera_host_task)
            .await
            .is_err()
    {
        tracing::warn!(
            ?extension_shutdown_deadline,
            "camera host did not stop before shutdown deadline; aborting task"
        );
        camera_host_task.abort();
        let _ = camera_host_task.await;
    }
    let graceful_extensions = async {
        while let Some(extension_result) = extension_tasks.join_next().await {
            match extension_result {
                Ok(Ok(())) => {}
                Ok(Err(error)) => tracing::warn!(%error, "extension failed during shutdown"),
                Err(error) => tracing::warn!(%error, "extension task failed during shutdown"),
            }
        }
    };
    if tokio::time::timeout(extension_shutdown_deadline, graceful_extensions)
        .await
        .is_err()
    {
        tracing::warn!(
            ?extension_shutdown_deadline,
            "extensions did not stop before shutdown deadline; aborting remaining tasks"
        );
        extension_tasks.abort_all();
        while extension_tasks.join_next().await.is_some() {}
    }

    // Tell every connected browser we are shutting down, so all clients (not
    // only one) show the "FLO has quit" screen and stop reconnecting. This is
    // sent while the HTTP server task is still alive: it is only dropped when
    // this function returns and the tokio runtime is torn down. The brief sleep
    // gives the SSE message time to flush to clients first.
    let _ = quit_tx.send(true);
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    coordinator_result
}

fn play_sound(
    new_mode: DeviceMode,
    device_config: &FloControllerConfig,
    audio_stream: Option<&rodio::OutputStreamHandle>,
) {
    match play_sound_inner(new_mode, device_config, audio_stream) {
        Ok(()) => {}
        Err(e) => {
            // log error and then drop it.
            log::error!("{} {}:{}", e, file!(), line!());
        }
    }
}

fn play_sound_inner(
    new_mode: DeviceMode,
    device_config: &FloControllerConfig,
    audio_stream: Option<&rodio::OutputStreamHandle>,
) -> Result<()> {
    if let Some(audio_stream) = audio_stream
        && let Some(filename) = device_config.sounds_filenames.filename(new_mode)
    {
        use rodio::{Decoder, source::Source};
        use std::fs::File;
        use std::io::BufReader;

        let file = BufReader::new(File::open(filename)?);
        let source = Decoder::new(file)?;
        audio_stream.play_raw(source.convert_samples())?;
    };
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_cli_accepts_composition_arguments() {
        let cli = parse_cli(["flo", "--config", "sim.yaml"]);
        assert_eq!(cli.config.as_deref(), Some("sim.yaml"));
    }

    #[test]
    fn help_is_rendered_only_when_it_is_asked_for() {
        assert!(cli_help(["flo"]).is_none());
        assert!(cli_help(["flo", "--config", "sim.yaml"]).is_none());
        // Not help, and reported by the real parse rather than here.
        assert!(cli_help(["flo", "--no-such-flag"]).is_none());

        for flag in ["--help", "-h"] {
            let help = cli_help(["flo", flag]).expect("clap should render help");
            assert!(
                help.contains("--config"),
                "help must describe the flag it is usually read to find: {help}"
            );
        }
    }

    /// The whole argument list is handed to clap, so a subcommand's own help
    /// reaches that subcommand's page rather than the top-level one.
    #[test]
    fn a_subcommands_help_is_its_own() {
        let help = cli_help(["flo", "show-config"]);
        assert!(help.is_none(), "no help was asked for");
        let help = cli_help(["flo", "show-config", "--help"]).expect("clap should render help");
        assert!(
            help.contains("Show the configuration"),
            "expected the subcommand's page, got: {help}"
        );
    }

    fn target(addr: &str, bitrate_kbps: u32) -> flo_core::RtpTarget {
        flo_core::RtpTarget {
            addr: addr.parse().unwrap(),
            bitrate_kbps,
        }
    }

    fn state_with_targets(targets: Vec<flo_core::RtpTargetConfig>) -> flo_core::DeviceState {
        let mut state =
            flo_core::DeviceState::new(flo_core::DeviceId::new([0; flo_core::DEVICE_ID_LEN]));
        state.rtp_targets = targets;
        state
    }

    #[test]
    fn rtp_targets_keep_the_first_occurrence_of_each_address() {
        let first = flo_core::RtpTargetConfig::new(target("127.0.0.1:5600", 4000));
        let duplicate = flo_core::RtpTargetConfig::new(target("127.0.0.1:5600", 2000));
        let second = flo_core::RtpTargetConfig::new(target("127.0.0.1:5601", 2500));
        assert_eq!(
            canonical_rtp_targets(vec![first, second, duplicate]),
            vec![first, second]
        );
    }

    /// A destination is sent to as soon as it is added.
    #[test]
    fn a_new_destination_is_enabled() {
        let config = flo_core::RtpTargetConfig::new(target("127.0.0.1:5600", 4000));
        assert!(config.enabled);
        let state = state_with_targets(vec![config]);
        assert!(state.rtp_send_enabled, "sending is on by default");
        assert_eq!(rtp_targets_to_send(&state), vec![config.target]);
    }

    /// Each destination has its own switch, and switching one off leaves the
    /// others streaming.
    #[test]
    fn only_the_enabled_destinations_are_sent_to() {
        let mut first = flo_core::RtpTargetConfig::new(target("127.0.0.1:5600", 4000));
        let second = flo_core::RtpTargetConfig::new(target("127.0.0.1:5601", 2500));
        first.enabled = false;

        let state = state_with_targets(vec![first, second]);
        assert_eq!(rtp_targets_to_send(&state), vec![second.target]);
        // The disabled destination keeps its address and bitrate.
        assert_eq!(state.rtp_targets[0], first);
    }

    /// The master switch stops every stream, and restores each destination's own
    /// setting rather than turning them all back on.
    #[test]
    fn the_master_switch_covers_every_destination() {
        let first = flo_core::RtpTargetConfig::new(target("127.0.0.1:5600", 4000));
        let mut second = flo_core::RtpTargetConfig::new(target("127.0.0.1:5601", 2500));
        second.enabled = false;

        let mut state = state_with_targets(vec![first, second]);
        assert_eq!(rtp_targets_to_send(&state), vec![first.target]);

        state.rtp_send_enabled = false;
        assert_eq!(rtp_targets_to_send(&state), vec![]);

        state.rtp_send_enabled = true;
        assert_eq!(
            rtp_targets_to_send(&state),
            vec![first.target],
            "the destination that was switched off individually stays off"
        );
    }

    #[test]
    fn centroid_input_is_bounded() {
        let (tx, mut rx) = centroid_input_channel(1);
        let first = MomentCentroid::default();
        let second = MomentCentroid {
            framenumber: first.framenumber + 1,
            ..first.clone()
        };

        tx.try_send(first.clone()).unwrap();
        assert!(matches!(
            tx.try_send(second),
            Err(mpsc::error::TrySendError::Full(_))
        ));
        assert_eq!(rx.try_recv().unwrap(), first);
    }

    #[tokio::test]
    async fn centroid_input_send_waits_for_capacity() {
        let (tx, mut rx) = centroid_input_channel(1);
        let first = MomentCentroid::default();
        let second = MomentCentroid {
            framenumber: first.framenumber + 1,
            ..first.clone()
        };
        tx.try_send(first.clone()).unwrap();

        let send_tx = tx.clone();
        let send_task = tokio::spawn(async move { send_tx.send(second).await });
        tokio::task::yield_now().await;
        assert!(!send_task.is_finished());

        assert_eq!(rx.recv().await, Some(first));
        send_task.await.unwrap().unwrap();
        assert_eq!(
            rx.recv().await.map(|centroid| centroid.framenumber),
            Some(43)
        );
    }

    #[test]
    fn default_options_bound_centroid_queue() {
        let options = AppOptions::default();
        assert_eq!(
            options.centroid_input_capacity,
            DEFAULT_CENTROID_INPUT_CAPACITY
        );
    }

    #[test]
    fn localset_runs_thread_affine_extension_tasks_on_flo_runtime() {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let local = tokio::task::LocalSet::new();
        local.block_on(&runtime, async {
            let thread_affine = std::rc::Rc::new(());
            let task = tokio::task::spawn_local(async move {
                let _thread_affine = thread_affine;
                Ok::<_, eyre::Error>(())
            });
            task.await.unwrap().unwrap();
        });
    }

    fn embedded_camera(role: StrandCamRole, name: &str) -> CameraRegistration {
        CameraRegistration {
            role,
            name: name.to_owned(),
            router_rx: None,
            control_tx: None,
            expected_fps: None,
        }
    }

    #[tokio::test]
    async fn embedded_camera_startup_preserves_camera_host_error() {
        let (router_tx, router_rx) = tokio::sync::oneshot::channel();
        let mut camera = embedded_camera(StrandCamRole::Main, "broken-camera");
        camera.router_rx = Some(router_rx);
        let mut camera_host_task = tokio::spawn(async move {
            drop(router_tx);
            Err(eyre::eyre!("detailed camera backend error"))
        });

        let error = match init_strand_cams(vec![camera], Some(&mut camera_host_task)).await {
            Ok(_) => panic!("camera startup unexpectedly succeeded"),
            Err(error) => error,
        };
        let report = format!("{error:?}");
        assert!(report.contains("broken-camera"));
        assert!(report.contains("detailed camera backend error"));
    }

    #[test]
    fn embedded_main_only_needs_no_secondary_camera() {
        let secondary = embedded_secondary_camera_name(&[embedded_camera(
            StrandCamRole::Main,
            "embedded-main",
        )])
        .unwrap();

        assert_eq!(secondary, None);
        assert!(!secondary_camera_is_stale(None, None));
    }

    #[test]
    fn embedded_secondary_is_the_coordinator_secondary_identity() {
        let secondary = embedded_secondary_camera_name(&[
            embedded_camera(StrandCamRole::Main, "embedded-main"),
            embedded_camera(StrandCamRole::Secondary, "embedded-secondary"),
        ])
        .unwrap();

        assert_eq!(secondary.as_deref(), Some("embedded-secondary"));
        assert!(secondary_camera_is_stale(secondary.as_deref(), None));
    }

    #[test]
    fn embedded_camera_registrations_reject_duplicate_roles_and_names() {
        let role_conflict = embedded_secondary_camera_name(&[
            embedded_camera(StrandCamRole::Main, "main-a"),
            embedded_camera(StrandCamRole::Main, "main-b"),
        ])
        .unwrap_err();
        assert!(
            role_conflict
                .to_string()
                .contains("multiple embedded cameras")
        );

        let name_conflict = embedded_secondary_camera_name(&[
            embedded_camera(StrandCamRole::Main, "same"),
            embedded_camera(StrandCamRole::Secondary, "same"),
        ])
        .unwrap_err();
        assert!(name_conflict.to_string().contains("both"));
    }

    #[test]
    fn embedded_cameras_claim_direct_router_routes() {
        let cameras = InitializedStrandCams {
            secondary_cam_name: Some("embedded-secondary".to_owned()),
            embedded: vec![
                embedded_camera(StrandCamRole::Main, "embedded-main"),
                embedded_camera(StrandCamRole::Secondary, "embedded-secondary"),
            ],
            embedded_routers: flo_webserver::EmbeddedStrandCamRouters::new(),
        };

        assert_eq!(cameras.proxy_info().len(), 2);
    }

    #[tokio::test]
    async fn in_process_camera_control_uses_its_direct_channel() {
        let (tx, mut rx) = mpsc::channel(1);
        send_in_process_cam_arg(
            &tx,
            strand_cam_remote_control::CamArg::SetIsRecordingMp4(true),
        )
        .await;

        assert!(matches!(
            rx.recv().await,
            Some(strand_cam_remote_control::CamArg::SetIsRecordingMp4(true))
        ));
    }
}
