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

use clap::Parser;
use color_eyre::eyre::{self, Result, WrapErr};
use futures::StreamExt;
use preferences_serde1::Preferences;
use std::{
    future::{Future, pending},
    pin::Pin,
    sync::{Arc, RwLock},
};
use strand_bui_backend_session::HttpSession;
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};
use tokio_serial::SerialPortBuilderExt;
use tracing::{self as log};

use flo_core::{
    Angle, Broadway, CamStaleBitmask, CommandSource, DeviceId, DeviceMode, DeviceState, FloCommand,
    FloControllerConfig, FloEvent, FloatType, FocusMotorType, GimbalConfig, LocalFloState,
    ModeChangeReason, MomentCentroid, MotorPositionResult, MotorType, MotorValueCache, OsdState,
    PwmSerial, RadialDistance, SaveToDiskMsg, StampedBMsg, StrandCamConfig, UNICAST_UDP_DEFAULT,
    UdpMsg, drone_structs::DroneEvent,
};
use tracking::{centroid_to_sensor_angles, compute_motor_output, kalman_step};

mod camshow_client;
mod codec;
mod extension;
mod json_lines_writer;
pub mod osd;
mod pwm_serial_io;
mod tilta_io;
mod tracking;
mod trinamic_io;
mod udp_codec;
mod udp_handling;
mod writing_state;
mod zip_dir;

pub use extension::{Extension, ExtensionContext, OsdOverlay};
pub use osd::DroneStatus;

const HIGHMAG_VISIBLE_MP4_PATH_TEMPLATE: &str = "highmag%Y%m%d_%H%M%S.%f.mp4";
const WEBCAM_MP4_PATH_TEMPLATE: &str = "webcam%Y%m%d_%H%M%S.%f.mp4";
const FLO_DIRNAME_TEMPLATE: &str = "flo%Y%m%d_%H%M%S.%f";

/// Pattern that valid floz output directory names match. Compiled once
/// the first time recording starts; reused on every subsequent start.
static FLO_DIRNAME_RE: std::sync::LazyLock<regex::Regex> = std::sync::LazyLock::new(|| {
    regex::Regex::new(r"^flo[0-9]{8}_[0-9]{6}\.[0-9]+$").expect("FLO_DIRNAME_RE is a valid regex")
});

const STRAND_CAM_COOKIE_KEY: &str = "strand-cam-cookie";

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

    /// The address to bind for the UDP listener
    #[arg(long, default_value = UNICAST_UDP_DEFAULT)]
    udp_addr: String,

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

struct FloCoordinator<'a> {
    data_dir: &'a camino::Utf8Path,
    device_config: &'a mut FloControllerConfig,
    secondary_cam_name: Option<String>,
    tracking_state: flo_core::TrackingState,
    /// The most recent centroid data from the primary tracking camera.
    latest_centroid: Option<MomentCentroid>,
    /// The most recent centroid data from both cameras.
    latest_centroid_stereo: (Option<MomentCentroid>, Option<MomentCentroid>),
    /// A timestamp when last primary camera data arrived.
    stamp_cam_primary: Option<std::time::Instant>,
    /// A timestamp when last secondary camera data arrived.
    stamp_cam_secondary: Option<std::time::Instant>,
    last_motor_data: Option<MotorPositionResult>,
    all_cam_names_ever_seen: std::collections::BTreeSet<String>,
    stereo_cam_framenumber_offset: Option<i64>,
    flo_saver_tx: mpsc::UnboundedSender<SaveToDiskMsg>,
    osd_tx: Option<watch::Sender<OsdState>>,
    my_state: flo_core::DeviceState,
    _local_flo_state: flo_core::LocalFloState,
    motors_tx: watch::Sender<MotorValueCache>,
    audio_stream: Option<&'a rodio::OutputStreamHandle>,
    from_device_http_tx: watch::Sender<DeviceState>,
    broadway: flo_core::Broadway,
    cam_session_main: Option<HttpSession>,
    cam_session_secondary: Option<HttpSession>,
    camshow_recording_tx: Option<mpsc::UnboundedSender<camshow_client::Command>>,
    highmag_visible_recorder: Option<h264_recorder::H264Recorder>,
    /// Seconds of data currently held in the writer's pre-capture buffer.
    precapture_buffered_rx: watch::Receiver<f64>,
}

impl<'a> FloCoordinator<'a> {
    #[expect(clippy::too_many_arguments)]
    async fn new(
        device_config: &'a mut FloControllerConfig,
        flo_saver_tx: mpsc::UnboundedSender<SaveToDiskMsg>,
        osd_tx: Option<watch::Sender<OsdState>>,
        my_state: flo_core::DeviceState,
        _local_flo_state: LocalFloState,
        motors_tx: watch::Sender<MotorValueCache>,
        audio_stream: Option<&'a rodio::OutputStreamHandle>,
        from_device_http_tx: watch::Sender<DeviceState>,
        broadway: Broadway,
        camshow_recording_tx: Option<mpsc::UnboundedSender<camshow_client::Command>>,
        data_dir: &'a camino::Utf8Path,
        precapture_buffered_rx: watch::Receiver<f64>,
    ) -> Result<Self> {
        let (cam_session_main, cam_session_secondary, secondary_cam_name) =
            init_strand_cams(device_config).await?;
        Ok(Self {
            data_dir,
            device_config,
            secondary_cam_name,
            tracking_state: Default::default(),
            latest_centroid: Default::default(),
            latest_centroid_stereo: Default::default(),
            stamp_cam_primary: Default::default(),
            stamp_cam_secondary: Default::default(),
            last_motor_data: Default::default(),
            all_cam_names_ever_seen: Default::default(),
            stereo_cam_framenumber_offset: Default::default(),
            flo_saver_tx,
            osd_tx,
            my_state,
            _local_flo_state,
            motors_tx,
            audio_stream,
            from_device_http_tx,
            broadway,
            cam_session_main,
            cam_session_secondary,
            camshow_recording_tx,
            highmag_visible_recorder: None,
            precapture_buffered_rx,
        })
    }

    fn on_new_motor_position(&mut self, motor_position: MotorPositionResult) -> Result<()> {
        log::trace!(
            "got motor_position. pan = {:?}, tilt = {:?} ",
            motor_position.pan_enc,
            motor_position.tilt_enc
        );
        self.last_motor_data = Some(motor_position.clone());
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

    fn on_image_centroid(
        &mut self,
        udp_msg: std::result::Result<(UdpMsg, std::net::SocketAddr), udp_codec::Error>,
    ) -> Result<()> {
        log::trace!("got UDP message {:?}", udp_msg);
        // This is the path through which centroid data from Strand Camera
        // arrives.
        let centroid = match udp_msg {
            Ok((UdpMsg::Centroid(centroid), _addr)) => {
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
                    eyre::bail!(
                        "More than one camera sending data, but no secondary camera assigned"
                    );
                }
                centroid
            }
            Err(e) => {
                tracing::error!("Error deserializing UDP message: {e}");
                return Ok(());
            }
        };

        let cam_name = centroid.cam_name.as_str();
        let mut is_secondary = false;
        if let Some(scn) = self.secondary_cam_name.as_ref()
            && cam_name == scn
        {
            is_secondary = true;
        }

        if is_secondary {
            self.stamp_cam_secondary = Some(std::time::Instant::now());
            if centroid.mu00 != 0.0 {
                let cs = &mut self.latest_centroid_stereo;
                cs.1 = Some(centroid.clone());
            }
        } else {
            self.stamp_cam_primary = Some(std::time::Instant::now());
            if centroid.mu00 != 0.0 {
                let c = &mut self.latest_centroid;
                *c = Some(centroid.clone());

                let cs = &mut self.latest_centroid_stereo;
                cs.0 = Some(centroid.clone());
            }
        }

        if centroid.mu00 != 0.0 {
            self.broadway
                .flo_detections
                .send(flo_core::FloDetectionEvent::Centroid(
                    flo_core::CentroidEvent {
                        centroid,
                        is_primary: !is_secondary,
                    },
                ))?;
        }

        Ok(())
    }

    fn on_fast_tick(&mut self, dt_secs: f64) -> Result<()> {
        let next_mode = {
            let (msg, next_mode) = {
                let tracking_state = &mut self.tracking_state;
                let latest_centroid = self.latest_centroid.take();
                let centroids = {
                    let lg = &mut self.latest_centroid_stereo;
                    if let (Some(ctd0), Some(ctd1)) = (*lg).clone() {
                        // Full stereopsis data is available, erase self.latest_centroid_stereo.
                        *lg = (None, None);
                        Some((ctd0, ctd1))
                    } else {
                        // Do not erase self.latest_centroid_stereo, as the
                        // second centroid might still arrive a bit later.
                        None
                    }
                };

                let mut use_stereo = true;
                if let Some((c1, c2)) = &centroids {
                    // calculate offset in number of frames.
                    let this_offset: i64 = i64::from(c2.framenumber) - i64::from(c1.framenumber);
                    if let Some(expected_offset) = &self.stereo_cam_framenumber_offset {
                        if *expected_offset != this_offset {
                            tracing::warn!(
                                "stereo camera offset: off by {}",
                                this_offset - *expected_offset
                            );
                            use_stereo = false;
                        }
                    } else {
                        // We do not yet have a frame offset we will accept as
                        // valid. First, compute the time difference between the
                        // stereo pair.
                        let offset_millis = (c1.timestamp - c2.timestamp).num_milliseconds().abs();
                        // Next, if the time difference is small enough, accept
                        // this frame offset.
                        if offset_millis < 5 {
                            self.stereo_cam_framenumber_offset = Some(this_offset);
                        }
                    }
                }
                // Only use centroid pair for stereopsis if the frame offset is
                // expected.
                let stereo_centroids = if use_stereo { centroids } else { None };

                if use_stereo && stereo_centroids.is_some() {
                    self.broadway.flo_detections.send(
                        flo_core::FloDetectionEvent::StereoCentroid(
                            stereo_centroids.clone().unwrap().0,
                            stereo_centroids.clone().unwrap().1,
                        ),
                    )?;
                }

                let centroid_timestamp = latest_centroid.as_ref().map(|x| x.timestamp);
                let angles = centroid_to_sensor_angles(self.device_config, latest_centroid.clone());
                let stereopsis_state = if let Some(centroids) = stereo_centroids {
                    self.device_config
                        .stereopsis_calib
                        .as_ref()
                        .map(|cal| cal.centroids_to_distance(centroids))
                } else {
                    None
                };

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
        Ok(())
    }

    fn on_slow_tick(&mut self) -> Result<()> {
        // Every second, echo state to all listeners.

        let is_stale_secondary = if self.cam_session_secondary.is_some() {
            // If we have a secondary camera, is it stale?
            is_stale(self.stamp_cam_secondary.as_ref())
        } else {
            // We have no secondary camera, so it is not stale.
            false
        };
        self.my_state.cam_stale = CamStaleBitmask::new(
            is_stale(self.stamp_cam_primary.as_ref()),
            is_stale_secondary,
        );

        self.my_state.mode = self.tracking_state.mode; //my_state.mode is just a mirror of tracking_state.mode, make sure it's up to date
        // Reflect the writer's current pre-capture buffer fill.
        self.my_state.precapture_buffered_secs = *self.precapture_buffered_rx.borrow();
        // relay to HTTP server
        self.from_device_http_tx.send(self.my_state.clone())?;
        self.my_state.stereopsis_state = None; //a hack to skip failed stereopsis detections because of centroid packets not arriving promptly, as there are many
        Ok(())
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
    fn start_recording(&mut self, include_precapture: bool) -> Result<()> {
        let creation_time = chrono::Local::now();
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
        Ok(())
    }

    /// Stop saving the current recording, finalizing all outputs.
    fn stop_recording(&mut self) -> Result<()> {
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
        Ok(())
    }

    /// Send a `CamArg` to every connected tracking-camera control session.
    async fn send_cam_arg_to_all(&mut self, arg: strand_cam_remote_control::CamArg) {
        for sess in [&mut self.cam_session_main, &mut self.cam_session_secondary]
            .into_iter()
            .flatten()
        {
            post_cam_arg(sess, arg.clone()).await;
        }
    }

    /// Size each tracking camera's post-trigger buffer to match `secs` of
    /// pre-capture, using that camera's configured `expected_fps`. Cameras
    /// without a configured frame rate are skipped with a warning, since the
    /// seconds→frames conversion is impossible without it.
    async fn set_cam_precapture_buffer(&mut self, secs: f64) {
        // Read frame rates up front so the camera config (immutable borrow) and
        // the sessions (mutable borrow) are not borrowed simultaneously.
        let main_fps = self
            .device_config
            .strand_cam_main
            .as_ref()
            .and_then(|c| c.expected_fps);
        let secondary_fps = self
            .device_config
            .strand_cam_secondary
            .as_ref()
            .and_then(|c| c.expected_fps);

        for (sess, fps) in [
            (&mut self.cam_session_main, main_fps),
            (&mut self.cam_session_secondary, secondary_fps),
        ] {
            let Some(sess) = sess.as_mut() else { continue };
            match fps {
                Some(fps) if fps > 0.0 => {
                    let frames = (secs * fps).ceil().max(0.0) as usize;
                    post_cam_arg(
                        sess,
                        strand_cam_remote_control::CamArg::SetPostTriggerBufferSize(frames),
                    )
                    .await;
                }
                _ => {
                    tracing::warn!(
                        "Cannot size Strand Cam pre-capture buffer: no `expected_fps` \
                         configured for this camera"
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
            FloCommand::SetRecordingState(enable) => {
                if enable {
                    // Normal recording: start from now, ignoring the
                    // pre-capture buffer.
                    self.start_recording(false)?;
                } else {
                    self.stop_recording()?;
                }
            }
            FloCommand::StartPreCaptureRecording => {
                // Post-trigger: start a recording that also includes the
                // buffered pre-capture window. Stopping uses the normal
                // SetRecordingState(false) path.
                self.start_recording(true)?;
                // Have the tracking cameras flush their own post-trigger
                // buffers into their MP4s too, so the recorded video also
                // begins in the past.
                self.send_cam_arg_to_all(strand_cam_remote_control::CamArg::PostTrigger)
                    .await;
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
            }
        };
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

                // start/stop saving .mp4 file on tracking cameras
                self.send_cam_arg_to_all(strand_cam_remote_control::CamArg::SetIsRecordingMp4(
                    want_recording,
                ))
                .await;
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

/// POST a single `CamArg` to a Strand Cam control session. Errors are logged
/// and otherwise ignored, since the camera may be transiently unreachable.
async fn post_cam_arg(sess: &mut HttpSession, arg: strand_cam_remote_control::CamArg) {
    let body = axum::body::Body::new(http_body_util::Full::new(bytes::Bytes::from(
        serde_json::to_vec(&strand_cam_storetype::CallbackType::ToCamera(arg)).unwrap(),
    )));
    match sess.post("callback", body).await {
        Ok(_) => {}
        Err(e) => {
            tracing::warn!("Ignoring error sending command to Strand Cam: {e}");
        }
    }
}

/// Open the HTTP control session for a camera and return it together with the
/// name the camera reports.
///
/// Only called when the camera config has a `url`. The camera's own `/cam-name`
/// response is the authoritative name.
async fn initialize_strand_cam_session(
    url: &str,
    cfg: &StrandCamConfig,
    jar: Arc<RwLock<cookie_store::CookieStore>>,
) -> Result<(HttpSession, String)> {
    let info = strand_bui_backend_session_types::BuiServerAddrInfo::parse_url_with_token(url)?;
    let mut session = strand_bui_backend_session::create_session(&info, jar.clone())
        .await
        .with_context(|| format!("while opening HTTP connection to Strand Cam at {url}"))?;

    {
        // We have the cookie from Strand Cam now, so store it to disk.
        let jar = jar.read().unwrap();
        Preferences::save(&*jar, &flo_webserver::APP_INFO, STRAND_CAM_COOKIE_KEY)?;
        // The jar holds live session cookies; keep its file owner-only.
        flo_webserver::harden_prefs_file(&flo_webserver::APP_INFO, STRAND_CAM_COOKIE_KEY);
        tracing::debug!("saved cookie store {STRAND_CAM_COOKIE_KEY}");
    }

    let cam_name = {
        let name_response = session
            .get("cam-name")
            .await
            .with_context(|| "Making request for camera name")?;

        use http_body_util::BodyExt;
        let body_bytes = name_response.collect().await?.to_bytes();
        String::from_utf8(body_bytes.to_vec())?
    };

    tracing::info!("opened camera {cam_name} at {url}");

    for cmd in cfg.on_attach_json_commands.iter() {
        session
            .post("callback", cmd.clone().into())
            .await
            .with_context(|| format!("Making callback to {cam_name}: {cmd}"))?;
    }
    Ok((session, cam_name))
}

/// Resolve a configured camera into an optional live HTTP control session and
/// the camera's name.
///
/// * With a `url`: open the HTTP session, ask the camera its name (the
///   authoritative source), run the `on_attach` commands, and — if `cam_name`
///   is also set in config — check the two agree.
/// * Without a `url`: open no session (e.g. hardware-free / simulation use,
///   where centroids arrive over UDP). The name is taken from `cam_name`, which
///   is then required.
async fn resolve_strand_cam(
    role: &str,
    cfg: &StrandCamConfig,
    jar: Arc<RwLock<cookie_store::CookieStore>>,
) -> Result<(Option<HttpSession>, String)> {
    match cfg.url.as_ref() {
        Some(url) => {
            let (session, cam_name) = initialize_strand_cam_session(url, cfg, jar).await?;
            if let Some(configured) = cfg.cam_name.as_ref()
                && configured != &cam_name
            {
                eyre::bail!(
                    "{role} camera name in config (`cam_name: {configured}`) does not match \
                     \"{cam_name}\" reported by the camera at {url}",
                );
            }
            Ok((Some(session), cam_name))
        }
        None => {
            let cam_name = cfg.cam_name.clone().ok_or_else(|| {
                eyre::eyre!(
                    "{role} `strand_cam` config has neither `url` nor `cam_name`; one is \
                     required (set `cam_name` to identify the camera for hardware-free / \
                     simulation use, where centroids are injected over UDP)"
                )
            })?;
            tracing::info!(
                "{role} camera {cam_name:?} configured without `url`: no HTTP session opened"
            );
            Ok((None, cam_name))
        }
    }
}

async fn init_strand_cams(
    device_config: &FloControllerConfig,
) -> Result<(Option<HttpSession>, Option<HttpSession>, Option<String>)> {
    // Connect to strand-cam for main and secondary cameras.
    let jar: cookie_store::CookieStore =
        match Preferences::load(&flo_webserver::APP_INFO, STRAND_CAM_COOKIE_KEY) {
            Ok(jar) => {
                tracing::debug!("loaded cookie store {STRAND_CAM_COOKIE_KEY}");
                jar
            }
            Err(e) => {
                tracing::debug!("cookie store {STRAND_CAM_COOKIE_KEY} not loaded: {e} {e:?}");
                cookie_store::CookieStore::new(None)
            }
        };

    let jar = Arc::new(RwLock::new(jar.clone()));

    let main = futures::future::OptionFuture::from(
        device_config
            .strand_cam_main
            .as_ref()
            .map(|cfg| resolve_strand_cam("main", cfg, jar.clone())),
    )
    .await
    .transpose()?;

    let secondary = futures::future::OptionFuture::from(
        device_config
            .strand_cam_secondary
            .as_ref()
            .map(|cfg| resolve_strand_cam("secondary", cfg, jar)),
    )
    .await
    .transpose()?;

    // The primary camera's name is not needed: any centroid not from the
    // secondary camera is treated as primary.
    let cam_session_main = main.and_then(|(session, _name)| session);
    let (cam_session_secondary, mut secondary_cam_name) = match secondary {
        Some((session, name)) => (session, Some(name)),
        None => (None, None),
    };

    // Backward compatibility for the deprecated top-level `secondary_cam_name`.
    // Prefer `strand_cam_secondary.cam_name`; honor the old field with a warning
    // so configs in the wild keep working while users migrate.
    if let Some(deprecated) = device_config.secondary_cam_name.as_ref() {
        tracing::warn!(
            "config field `secondary_cam_name` is deprecated; \
             set `strand_cam_secondary.cam_name` instead"
        );
        match secondary_cam_name.as_ref() {
            Some(resolved) if resolved != deprecated => {
                eyre::bail!(
                    "deprecated `secondary_cam_name` ({deprecated:?}) does not match the \
                     secondary camera name ({resolved:?}) resolved from `strand_cam_secondary`",
                );
            }
            Some(_) => {} // agrees with the resolved name; nothing to do
            None => {
                // No `strand_cam_secondary` block: use the deprecated name on its own.
                secondary_cam_name = Some(deprecated.clone());
            }
        }
    }

    Ok((cam_session_main, cam_session_secondary, secondary_cam_name))
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

/// Options used to compose extra subsystems into the flo binary at link time.
/// The default flo binary passes `AppOptions::default()`; downstream crates
/// that need to add extensions construct their own `AppOptions`.
#[derive(Default)]
pub struct AppOptions {
    /// Long-running subsystems that join the supervisor select loop.
    pub extensions: Vec<Box<dyn Extension>>,
    /// OSD overlays drawn each render tick after the base layer.
    pub osd_overlays: Vec<Box<dyn OsdOverlay + Send + Sync>>,
}

/// Run the flo application.
///
/// Parses CLI args, loads configuration, sets up logging, and runs the
/// tokio main loop. Returns when the program exits.
pub fn run(options: AppOptions) -> Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        let envstr = format!("{}=info,info", env!("CARGO_PKG_NAME")).replace('-', "_");
        // SAFETY: We ensure that this only happens in single-threaded code
        // because this is immediately at the start of run() and no other
        // threads have started.
        unsafe { std::env::set_var("RUST_LOG", envstr) };
    }

    let cli = Cli::parse();
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

    let device_id = match get_device_id() {
        Ok(device_id) => device_id,
        Err(e) => {
            log::warn!("No device id found ({e}), using random number.");
            DeviceId::new(rand::random())
        }
    };
    // Create our device state.
    let mut my_state = flo_core::DeviceState::new(device_id);

    let mut device_config = if let Some(device_config_fname) = &cli.config {
        tracing::debug!("Loading config file: \"{device_config_fname}\"");
        for name in options.extensions.iter().map(|e| e.name()) {
            tracing::debug!("extension name: \"{name}\"");
        }

        log::info!("Reading initial device config from: {device_config_fname}");
        let cfg_buf = std::fs::read_to_string(device_config_fname)
            .with_context(|| format!("opening file {device_config_fname}"))?;

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
    };

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
        shutdown_rx,
        &data_dir,
        mavlink_port,
        options,
    )?;
    Ok(())
}

/// Create the tokio Runtime and call the main app loop.
fn run_tokio_main(
    cli: Cli,
    my_state: DeviceState,
    device_config: FloControllerConfig,
    shutdown_rx: tokio::sync::oneshot::Receiver<()>,
    data_dir: &camino::Utf8Path,
    mavlink_port: Option<flo_mavlink::MavlinkPort>,
    options: AppOptions,
) -> Result<()> {
    let tokio_rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;

    let handle = tokio_rt.handle();
    tokio_rt.block_on(app_main(
        handle,
        cli,
        my_state,
        device_config,
        shutdown_rx,
        data_dir,
        mavlink_port,
        options,
    ))
}

/// The highest-level main loop for the flo app.
#[expect(clippy::too_many_arguments)]
async fn app_main(
    handle: &tokio::runtime::Handle,
    cli: Cli,
    mut my_state: DeviceState,
    mut device_config: FloControllerConfig,
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

    let mut mavlink_task: Pin<Box<dyn Future<Output = _>>> =
        if let Some(mavlink_port) = mavlink_port {
            let mavlink_task_jh = flo_mavlink::spawn_mavlink(
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

    // Create channel for motor position feedback.
    let (motor_position_tx, mut motor_position_rx) = mpsc::channel::<MotorPositionResult>(10);

    let mut converter_handle: JoinHandle<eyre::Result<()>> = {
        let mut rx1 = broadway.flo_events.subscribe();
        let mut rx2 = broadway.flo_detections.subscribe();
        let mut rx4 = broadway.drone_events.subscribe();
        let mut rx5 = broadway.drone_realtime.subscribe();
        let mut rx8 = broadway.flight_setpoint.subscribe();
        let mut rx9 = broadway.flight_mode_request.subscribe();

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
                    rx8_res = rx8.recv() => {
                        flo_saver_tx.bsend(FlightSetpoint(rx8_res?))?;
                    }
                    rx9_res = rx9.recv() => {
                        flo_saver_tx.bsend(FlightModeRequest(rx9_res?))?;
                    }
                }
            }
        })
    };

    let mut saver_handle = {
        // Spawn task for saving data to disk
        let device_config = device_config.clone();
        handle.spawn_blocking(move || {
            writing_state::writer_task_main(flo_saver_rx, &device_config, precapture_buffered_tx)
        })
    };

    if device_config.pwm_output_enabled && cli.pwm_serial.is_none() {
        tracing::error!("PWM output enabled, but no PWM serial device specified");
    }

    // Launch task to run serial IO for rpi pico pantilt PWM motors
    let mut motor_task_join_handle = if let Some(pwm_serial) = cli.pwm_serial {
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
        {
            let handle2 = handle.clone();
            handle.spawn(async move {
                pwm_serial_io::run_rpi_pico_pwm_serial_loop(
                    &handle2,
                    motors_rx,
                    serial_device,
                    pan_pwm_config,
                    pan_motor_config,
                    tilt_pwm_config,
                    tilt_motor_config,
                )
                .await
            })
        }
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
        handle.spawn(async move {
            trinamic_io::run_trinamic_loop(
                motors_rx,
                motor_position_tx,
                pan_device,
                tilt_device,
                focus_device,
                pan_trinamic_config,
                tilt_trinamic_config,
            )
            .await
        })
    } else if let Some(cfg) = device_config.gimbal_config.clone() {
        my_state.motor_type = MotorType::Gimbal;
        let flo_saver_tx = flo_saver_tx.clone();
        let motors_rx = motors_rx.clone();
        handle.spawn(async move {
            sbgc_gimbal::run_gimbal_loop(motors_rx, motor_position_tx, flo_saver_tx, cfg).await
        })
    } else {
        tracing::warn!("No motor control method specified.");
        handle.spawn(async {
            loop {
                futures::future::pending().await
            }
        })
    };

    let (mut focus_motor_task, focusing): (Pin<Box<dyn Future<Output = _>>>, bool) =
        match &device_config.focus_motor_config.as_ref().unwrap().motor {
            FocusMotorType::NoMotor => (Box::pin(pending()), false),
            FocusMotorType::Trinamic(_) => {
                if my_state.motor_type != MotorType::Trinamic {
                    eyre::bail!("trinamic focus with non-trinamic pan/tilt not yet implemented");
                }
                //nothing to do, already handled when setting up pan/tilt
                (Box::pin(pending()), true)
            }
            FocusMotorType::Tilta(tcfg) => {
                //let task = tilta_io::run_tilta_loop();
                //(Some(task), true)
                let port = tcfg.port.clone();
                let jh = {
                    let handle2 = handle.clone();
                    handle.spawn(async move {
                        tilta_io::run_tilta_loop(&handle2, port.as_str(), motors_rx).await
                    })
                };
                (Box::pin(jh), true)
            }
        };
    my_state.focusing = focusing;

    // Setup FLO networking
    let mut udp_framed_recv = udp_handling::setup_udp(&cli.udp_addr).await?;

    // Create a channel to send copies of our device state.
    let (from_device_http_tx, from_device_http_rx) = watch::channel(my_state.clone());

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

    // Run web server main loop
    let _http_server_join_handle = {
        let device_config = device_config.clone();
        let event_tx = broadway.flo_events.clone();
        handle.spawn(async move {
            flo_webserver::main_loop(
                tcp_listener,
                token_config,
                persistent_secret,
                trusted_networks,
                from_device_http_rx,
                device_config,
                event_tx,
                quit_rx,
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
    // OSD canvas is mirrored over TCP and webcam recording start/stop is
    // forwarded; otherwise we feed neither.
    let camshow_addr = device_config
        .osd_config
        .as_ref()
        .and_then(|c| c.camshow_addr.clone());
    let (canvas_tx, camshow_recording_tx, mut camshow_task) = if let Some(addr) = camshow_addr {
        let (canvas_tx, canvas_rx) = watch::channel(osd_utils::OsdCache::new(30, 16));
        let (rec_tx, rec_rx) = mpsc::unbounded_channel();
        let jh = handle.spawn(camshow_client::run(addr, canvas_rx, rec_rx));
        let task: Pin<Box<dyn Future<Output = _>>> = Box::pin(jh);
        (Some(canvas_tx), Some(rec_tx), task)
    } else {
        let task: Pin<Box<dyn Future<Output = _>>> = Box::pin(pending());
        (None, None, task)
    };

    // Build the OSD overlay list. Defaults from AppOptions::osd_overlays
    // first, then each registered extension contributes via
    // make_osd_overlay() if it has a companion overlay.
    let mut osd_overlays = options.osd_overlays;
    let extensions: Vec<Box<dyn Extension>> = options.extensions;
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

    let mut coordinator = FloCoordinator::new(
        &mut device_config,
        flo_saver_tx.clone(),
        osd_tx,
        my_state,
        local_flo_state,
        motors_tx,
        audio_stream_handle.as_ref(),
        from_device_http_tx,
        broadway,
        camshow_recording_tx,
        data_dir,
        precapture_buffered_rx,
    )
    .await?;

    // Main loop.
    loop {
        // Wait for any of a number of things to happen
        tokio::select! {
            _ = &mut shutdown_rx => {
                break;
            }
            mavlink_result = &mut mavlink_task => {
                mavlink_result??;
                break;
            }
            osd_task_result = &mut osd_task => {
                osd_task_result??;
                break;
            }
            Some(extension_result) = extension_tasks.join_next(), if !extension_tasks.is_empty() => {
                extension_result??;
                break;
            }
            motor_task_result = &mut motor_task_join_handle => {
                motor_task_result??;
                break;
            }
            focus_motor_task_result = &mut focus_motor_task => {
                focus_motor_task_result??;
                break;
            }
            saver_result = &mut saver_handle => {
                saver_result??;
                break;
            }
            converter_result = &mut converter_handle => {
                converter_result??;
                break;
            }
            camshow_result = &mut camshow_task => {
                // The camshow client runs forever under normal operation;
                // if it returns it means the spawned task was cancelled.
                camshow_result??;
                break;
            }
            udp_msg = udp_framed_recv.next() => {
                let udp_msg = udp_msg.ok_or_else(|| eyre::eyre!("UDP channel closed"))?;
                coordinator.on_image_centroid(udp_msg)?;
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
    tracing::debug!("FloCoordinator ending.");

    // Tell every connected browser we are shutting down, so all clients (not
    // only one) show the "FLO has quit" screen and stop reconnecting. This is
    // sent while the HTTP server task is still alive: it is only dropped when
    // this function returns and the tokio runtime is torn down. The brief sleep
    // gives the SSE message time to flush to clients first.
    let _ = quit_tx.send(true);
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    Ok(())
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
