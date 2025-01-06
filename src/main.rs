use bui_backend_session::HttpSession;
use clap::Parser;
use color_eyre::eyre::{self, self as anyhow, Result, WrapErr};
use futures::StreamExt;
use preferences_serde1::Preferences;
use serde::{Deserialize, Serialize};
use std::{
    future::{pending, Future},
    pin::Pin,
    sync::{Arc, Mutex, RwLock},
};
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};
use tokio_serial::SerialPortBuilderExt;
use tracing::{self as log};

use flo_core::{
    drone_structs::DroneEvent, Angle, Broadway, CommandSource, DeviceId, DeviceMode, DeviceState,
    FloCommand, FloControllerConfig, FloEvent, FloatType, FocusMotorType, GimbalConfig,
    ModeChangeReason, MomentCentroid, MotorPositionResult, MotorType, MotorValueCache, OsdState,
    PwmSerial, RadialDistance, SaveToDiskMsg, StampedBMsg, StrandCamConfig, UNICAST_UDP_DEFAULT,
};
use tracking::{centroid_to_sensor_angles, compute_motor_output, kalman_step};

mod codec;
mod json_lines_writer;
mod osd;
mod pwm_serial_io;
mod tilta_io;
mod tracking;
mod trinamic_io;
mod udp_codec;
mod udp_handling;
mod writing_state;
mod zip_dir;

const FLO_DIRNAME_TEMPLATE: &str = "flo%Y%m%d_%H%M%S.%f";
const FLO_DIRNAME_GLOB: &str = "flo*";
const FLO_DIRNAME_RE: &str = r"^flo[0-9]{8}_[0-9]{6}\.[0-9]+$";

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

    /// The address to bind for the HTTP server
    #[arg(long, default_value = "0.0.0.0:2222")]
    http_addr: String,

    /// The address to bind for the UDP listener
    #[arg(long, default_value = UNICAST_UDP_DEFAULT)]
    udp_addr: String,

    /// Filename of initial device configuration in YAML format
    #[arg(long)]
    config: Option<String>,

    /// If set, .floz files and logs are saved to this directory.
    #[arg(long)]
    data_dir: Option<std::path::PathBuf>,
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

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub(crate) enum UdpMsg {
    Centroid(MomentCentroid),
}

struct FloCoordinator<'a> {
    data_dir: &'a std::path::Path,
    device_config: &'a mut FloControllerConfig,
    tracking_state: flo_core::TrackingState,
    latest_centroid: Option<MomentCentroid>,
    latest_centroid_stereo: (Option<MomentCentroid>, Option<MomentCentroid>),
    last_motor_data: Option<MotorPositionResult>,
    all_cam_names_ever_seen: std::collections::BTreeSet<String>,
    stereo_cam_framenumber_offset: Option<i64>,
    flo_saver_tx: mpsc::UnboundedSender<SaveToDiskMsg>,
    osd_tx: Option<watch::Sender<OsdState>>,
    my_state: flo_core::DeviceState,
    motors_tx: watch::Sender<MotorValueCache>,
    audio_stream: Option<&'a rodio::OutputStreamHandle>,
    from_device_http_tx: watch::Sender<DeviceState>,
    broadway: flo_core::Broadway,
    cam_session_main: Option<HttpSession>,
    cam_session_secondary: Option<HttpSession>,
}

impl<'a> FloCoordinator<'a> {
    #[allow(clippy::too_many_arguments)]
    async fn new(
        device_config: &'a mut FloControllerConfig,
        flo_saver_tx: mpsc::UnboundedSender<SaveToDiskMsg>,
        osd_tx: Option<watch::Sender<OsdState>>,
        my_state: flo_core::DeviceState,
        motors_tx: watch::Sender<MotorValueCache>,
        audio_stream: Option<&'a rodio::OutputStreamHandle>,
        from_device_http_tx: watch::Sender<DeviceState>,
        broadway: Broadway,
        data_dir: &'a std::path::Path,
    ) -> Result<Self> {
        let (cam_session_main, cam_session_secondary) = init_strand_cams(device_config).await?;
        Ok(Self {
            data_dir,
            device_config,
            tracking_state: Default::default(),
            latest_centroid: Default::default(),
            latest_centroid_stereo: Default::default(),
            last_motor_data: Default::default(),
            all_cam_names_ever_seen: Default::default(),
            stereo_cam_framenumber_offset: Default::default(),
            flo_saver_tx,
            osd_tx,
            my_state,
            motors_tx,
            audio_stream,
            from_device_http_tx,
            broadway,
            cam_session_main,
            cam_session_secondary,
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
                    bee_signal_strength: self.latest_centroid.as_ref().map(|mc| mc.mu00),
                    tracking_mode: self.tracking_state.mode,
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
        if let Ok((UdpMsg::Centroid(ref centroid), _addr)) = &udp_msg {
            // Save data
            let msg = SaveToDiskMsg::CentroidData((chrono::Local::now(), centroid.clone()));
            self.flo_saver_tx.send(msg)?;

            // Check if we have a second camera but no secondary camera in configuration.
            self.all_cam_names_ever_seen
                .insert(centroid.cam_name.clone());

            if self.all_cam_names_ever_seen.len() > 1
                && self.device_config.secondary_cam_name.is_none()
            {
                anyhow::bail!(
                    "More than one camera sending data, but no secondary camera assigned"
                );
            }
        }
        // Amongst others, this is the path through which centroid data
        // from Strand Camera arrives.
        let centroid = match udp_msg {
            Ok((UdpMsg::Centroid(c), _a)) => c,
            Err(e) => {
                tracing::error!("Error deserializing UDP message: {e}");
                return Ok(());
            }
        };

        if centroid.schema_version != 2 {
            tracing::error!(
                "expected centroid schema version 2, found {}",
                centroid.schema_version
            );
            return Ok(());
        }

        self.my_state.recent_centroid_packets += 1;

        let cam_name = centroid.cam_name.as_str();
        let mut is_secondary = false;
        if let Some(scn) = self.device_config.secondary_cam_name.as_ref() {
            if cam_name == scn {
                is_secondary = true;
            }
        }

        self.broadway
            .flo_detections
            .send(flo_core::FloDetectionEvent::Centroid(
                flo_core::CentroidEvent {
                    centroid: centroid.clone(),
                    is_primary: !is_secondary,
                },
            ))?;

        if is_secondary {
            {
                let cs = &mut self.latest_centroid_stereo;
                cs.1 = Some(centroid);
            }
        } else {
            {
                let c = &mut self.latest_centroid;
                *c = Some(centroid.clone());
            }
            {
                let cs = &mut self.latest_centroid_stereo;
                cs.0 = Some(centroid);
            }
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
                if let Some((ref c1, ref c2)) = &centroids {
                    // calculate offset in number of frames.
                    let this_offset: i64 = i64::from(c2.framenumber) - i64::from(c1.framenumber);
                    if let Some(ref expected_offset) = &self.stereo_cam_framenumber_offset {
                        if *expected_offset != this_offset {
                            tracing::error!(
                                "stereo camera offset error: off by {}",
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
                compute_motor_output(
                    tracking_state,
                    &self.my_state,
                    self.device_config,
                    &mut self.broadway,
                    dt_secs,
                );
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
        self.broadway
            .flo_motion
            .send(flo_core::FloMotionEvent::MotorCommand(
                current_motors.clone(),
            ))?;
        self.my_state.cached_motors = current_motors;
        if let Some(next_mode) = next_mode {
            self.handle_command(
                FloCommand::SwitchMode(next_mode.0, next_mode.1),
                CommandSource::Automation,
            )?;
        }
        Ok(())
    }

    fn on_slow_tick(&mut self) -> Result<()> {
        // Every second, echo state to all listeners.
        self.my_state.mode = self.tracking_state.mode; //my_state.mode is just a mirror of tracking_state.mode, make sure it's up to date
                                                       // relay to HTTP server
        self.from_device_http_tx.send(self.my_state.clone())?;
        self.my_state.recent_centroid_packets = 0;
        self.my_state.stereopsis_state = None; //a hack to skip failed stereopsis detections because of centroid packets not arriving promptly, as there are many
        Ok(())
    }

    fn handle_event(&mut self, evt: FloEvent) -> Result<()> {
        use FloEvent as E;
        if let E::Command(cmd, src) = evt {
            self.handle_command(cmd, src)?;
        }
        Ok(())
    }

    fn handle_command(&mut self, msg: FloCommand, src: CommandSource) -> Result<()> {
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
                let floz_msg = if enable {
                    let creation_time = chrono::Local::now();
                    let floz_dirname = creation_time.format(FLO_DIRNAME_TEMPLATE).to_string();
                    if !regex::Regex::new(FLO_DIRNAME_RE)?.is_match(&floz_dirname) {
                        tracing::error!("new dirname does not match expected pattern");
                    }
                    let full_floz_dirname = self.data_dir.join(floz_dirname);
                    self.my_state.floz_recording_path =
                        Some(flo_core::RecordingPath::from_path_and_time(
                            full_floz_dirname.display().to_string(),
                            creation_time,
                        ));
                    Some((creation_time, full_floz_dirname))
                } else {
                    // disable recording

                    self.my_state.floz_recording_path = None;
                    None
                };
                self.flo_saver_tx
                    .send(SaveToDiskMsg::ToggleSavingFloz(floz_msg))
                    .unwrap();
            }
        };
        Ok(())
    }

    async fn handle_drone_event(&mut self, evt: DroneEvent) -> Result<()> {
        use DroneEvent as E;
        match evt {
            E::Armed | E::Disarmed => {
                let want_recording = evt == E::Armed;
                // start/stop saving .flo data
                if self.my_state.floz_recording_path.is_some() != want_recording {
                    self.broadway.flo_events.send(FloEvent::Command(
                        FloCommand::SetRecordingState(want_recording),
                        CommandSource::DroneRC,
                    ))?;
                }

                // start/stop saving .mp4 file on tracking cameras
                for cam_session in
                    [&mut self.cam_session_main, &mut self.cam_session_secondary].iter_mut()
                {
                    if let Some(sess) = cam_session.as_mut() {
                        let body =
                            axum::body::Body::new(http_body_util::Full::new(bytes::Bytes::from(
                                serde_json::to_vec(&strand_cam_storetype::CallbackType::ToCamera(
                                    ci2_remote_control::CamArg::SetIsRecordingMp4(want_recording),
                                ))
                                .unwrap(),
                            )));
                        match sess.post("callback", body).await {
                            Ok(_) => {}
                            Err(e) => {
                                tracing::warn!(
                                    "Ignoring error requesting Strand Cam to record: {e}"
                                );
                            }
                        }
                    }
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

/// Clean up any existing FLO directories.
fn clean_up_old_flo_dirs(data_dir: &std::path::PathBuf) -> Result<()> {
    // First we match candidates with glob, then we get more specific with regex.
    let pattern = data_dir
        .join(FLO_DIRNAME_GLOB)
        .into_os_string()
        .into_string()
        .unwrap();
    let re_flo = regex::Regex::new(FLO_DIRNAME_RE).unwrap();
    for existing_flo_dir in glob::glob(&pattern)? {
        let existing_flo_dir = existing_flo_dir?;
        let existing_flo_dir_str = existing_flo_dir
            .file_name()
            .ok_or_else(|| eyre::eyre!("no filename"))?
            .to_str()
            .ok_or_else(|| eyre::eyre!("path not unicode"))?;
        if !re_flo.is_match(existing_flo_dir_str) {
            // Name does not match our more-specific regex, skip it.
            continue;
        }
        if existing_flo_dir.is_dir() {
            tracing::info!(
                "cleaning up existing FLO directory: {}",
                existing_flo_dir.display()
            );
            writing_state::repair_unfinished_flo(existing_flo_dir)?;
        }
    }

    tracing::info!("done cleaning up FLO directories with pattern {pattern}");
    Ok(())
}

async fn initialize_strand_cam_session(
    cfg: &StrandCamConfig,
    jar: Arc<RwLock<cookie_store::CookieStore>>,
) -> Result<HttpSession> {
    let info = flydra_types::BuiServerAddrInfo::parse_url_with_token(&cfg.url)?;
    let mut session = bui_backend_session::create_session(&info, jar.clone())
        .await
        .with_context(|| {
            format!(
                "while opening HTTP connection to main Strand Cam at {}",
                cfg.url
            )
        })?;

    {
        // We have the cookie from Strand Cam now, so store it to disk.
        let jar = jar.read().unwrap();
        Preferences::save(&*jar, &flo_webserver::APP_INFO, STRAND_CAM_COOKIE_KEY)?;
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

    tracing::info!("opened camera {cam_name} at {url}", url = &cfg.url);

    for cmd in cfg.on_attach_json_commands.iter() {
        session
            .post("callback", cmd.clone().into())
            .await
            .with_context(|| format!("Making callback to {cam_name}: {cmd}"))?;
    }
    Ok(session)
}

async fn init_strand_cams(
    device_config: &FloControllerConfig,
) -> Result<(Option<HttpSession>, Option<HttpSession>)> {
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

    let cam_session_main = futures::future::OptionFuture::from(
        device_config
            .strand_cam_main
            .as_ref()
            .map(|cfg| initialize_strand_cam_session(cfg, jar.clone())),
    )
    .await
    .transpose()?;

    let mut cam_session_secondary = futures::future::OptionFuture::from(
        device_config
            .strand_cam_secondary
            .as_ref()
            .map(|cfg| initialize_strand_cam_session(cfg, jar)),
    )
    .await
    .transpose()?;

    let cam_name_secondary2_resp = futures::future::OptionFuture::from(
        cam_session_secondary
            .as_mut()
            .map(|sess| sess.get("cam-name")),
    )
    .await
    .transpose()
    .with_context(|| "Making request for camera name")?;

    let cam_name_secondary2 = if let Some(cam_name_secondary2_resp) = cam_name_secondary2_resp {
        use http_body_util::BodyExt;
        let body_bytes = cam_name_secondary2_resp.collect().await?.to_bytes();
        Some(String::from_utf8(body_bytes.to_vec())?)
    } else {
        None
    };

    if let Some(secondary_cam_name) = device_config.secondary_cam_name.as_ref() {
        tracing::warn!("use of deprecated config `secondary_cam_name`");
        if let Some(cam_name_secondary2) = cam_name_secondary2.as_ref() {
            if cam_name_secondary2 != secondary_cam_name {
                anyhow::bail!(
                    "Camera name specified in config file \"{}\" (in field `secondary_cam_name`) \
                does not match \"{}\" as returned from {}/cam-name",
                    secondary_cam_name,
                    cam_name_secondary2,
                    device_config.strand_cam_secondary.as_ref().unwrap().url,
                );
            }
        }
    }
    Ok((cam_session_main, cam_session_secondary))
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

#[tokio::main]
async fn main() -> Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        let envstr = format!("{}=info,info", env!("CARGO_PKG_NAME")).replace('-', "_");
        std::env::set_var("RUST_LOG", envstr);
    }

    let cli = Cli::parse();
    let (log_dir, data_dir) = if let Some(dd) = cli.data_dir.as_ref() {
        (dd.clone(), dd.clone())
    } else {
        let home_dir = home::home_dir().unwrap();
        (home_dir.clone(), home_dir.join("flo-data"))
    };

    std::fs::create_dir_all(&log_dir)
        .with_context(|| format!("While creating directory {}", log_dir.display()))?;
    std::fs::create_dir_all(&data_dir)
        .with_context(|| format!("While creating directory {}", data_dir.display()))?;

    // Enable logging to console and to disk using tracing.
    {
        use time::{format_description::well_known::Iso8601, UtcOffset};
        use tracing_subscriber::{
            fmt::{self, time::OffsetTime},
            layer::SubscriberExt,
        };

        let log_file_name = chrono::Local::now()
            .format(".flo-%Y%m%d_%H%M%S.%f.log")
            .to_string();
        let full_log_file_name = log_dir
            .join(&log_file_name)
            .into_os_string()
            .into_string()
            .unwrap();

        // Create a fixed offset time formatter based on the timezone at the
        // time this line of code runs.
        let timer = OffsetTime::new(
            UtcOffset::from_whole_seconds(chrono::Local::now().offset().local_minus_utc())?,
            Iso8601::DEFAULT,
        );

        let file = std::fs::File::create(&full_log_file_name)
            .with_context(|| format!("While creating file {full_log_file_name}"))?;
        let file_writer = Mutex::new(file);
        let file_layer = fmt::layer()
            .with_timer(timer.clone())
            .with_writer(file_writer)
            .with_ansi(false)
            .with_file(true)
            .with_line_number(true);
        let console_layer = fmt::layer()
            .with_timer(timer)
            .with_file(true)
            .with_line_number(true);
        let collector = tracing_subscriber::registry()
            .with(file_layer)
            .with(console_layer)
            .with(tracing_subscriber::filter::EnvFilter::from_default_env());
        tracing::subscriber::set_global_default(collector)?;
        std::panic::set_hook(Box::new(tracing_panic::panic_hook));
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

    let mut device_config = if let Some(device_config_fname) = &cli.config {
        log::info!("Reading initial device config from: {device_config_fname}");
        let cfg_buf = std::fs::read_to_string(&device_config_fname)
            .with_context(|| format!("opening file {device_config_fname}"))?;

        serde_yaml::from_str(&cfg_buf)
            .with_context(|| format!("while parsing YAML in file {device_config_fname}"))?
    } else {
        log::info!("Loading default device config.");
        // Create initial device configuration when no file name given..
        // (TODO: why doesn't the default value suffice?).
        let mut device_config = FloControllerConfig::default();
        device_config.pwm_output_enabled = true;
        device_config.ki_pan_angle = -10.0;
        device_config.ki_tilt_angle = 10.0;
        device_config.centroid_to_sensor_x_angle_func.dx_gain = Angle::from_degrees(1.0 / 20.0).0; // each pixel is 1/20th of a degree
        device_config.centroid_to_sensor_x_angle_func.dy_gain = 0.0;
        device_config.centroid_to_sensor_x_angle_func.offset = 0.0;
        device_config.centroid_to_sensor_y_angle_func.dx_gain = 0.0;
        device_config.centroid_to_sensor_y_angle_func.dy_gain = Angle::from_degrees(1.0 / 20.0).0; // each pixel is 1/20th of a degree
        device_config.centroid_to_sensor_y_angle_func.offset = 0.0;
        my_state.mode = DeviceMode::ManualOpenLoop;
        device_config
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
        if device_config.osd_config.is_none() {
            device_config.osd_config = Some(flo_core::OsdConfig {
                port_path: Some(port_path.clone()),
                cal: None,
                blob: Default::default(),
            });
        };
        device_config.osd_config.as_mut().unwrap().port_path = Some(port_path.clone());
    };
    if let Some(cfg) = device_config.osd_config.as_mut() {
        if cfg.cal.is_none() {
            log::warn!(
                "osd device is provided but osd configuration is missing. Using built-in defaults."
            );
            cfg.cal = Some(flo_core::FpvCameraOSDCalibration::default());
        }
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
            anyhow::bail!("focus motor type conflict");
        };
    }

    let cfg_pretty = serde_yaml::to_string(&device_config)?;
    log::info!("device config:\n{cfg_pretty}");

    // Because the configuration is many lines long when displayed, we do this
    // after we display the config because it may show errors, which we want to
    // be after the config.
    device_config.fix_deprecations();

    match cli.command {
        Some(Commands::ShowConfig) => {
            return Ok(());
        }
        Some(Commands::Run) | None => {} // continue
    };

    clean_up_old_flo_dirs(&data_dir)?;

    // Find correct OSD canvas size.
    let (canv_w, canv_h) = match &device_config.osd_config {
        Some(osd_config) => {
            let cal: flo_core::osd_structs::FpvCameraOSDCalibration = osd_config
                .cal
                .as_ref()
                .ok_or_else(|| eyre::eyre!("OSD calibration required"))?
                .clone();
            (cal.osd_char_w, cal.osd_char_h)
        }
        None => (30, 16),
    };

    let canvas = Arc::new(Mutex::new(osd_utils::OsdCache::new(canv_w, canv_h)));

    let (flo_saver_tx, flo_saver_rx) = mpsc::unbounded_channel();

    let _write_closer = writing_state::WriteCloser::new(flo_saver_tx.clone());

    //delivers drone status from mavlink thread to flo main loop
    let broadway = Broadway::new(100, 10_000);

    //dummy subscriptions to prevent send errors
    //FIXME: find a better way to do this than a stalled dummy receiver
    let _rx1 = broadway.flo_events.subscribe();
    let _rx2 = broadway.flo_detections.subscribe();
    let _rx3 = broadway.flo_motion.subscribe();
    //let _rx4 = broadway.drone_events.subscribe();
    let _rx5 = broadway.drone_realtime.subscribe();

    let mut drone_event_rx_stream =
        tokio_stream::wrappers::BroadcastStream::new(broadway.drone_events.subscribe());

    let mut flo_events_rx_stream =
        tokio_stream::wrappers::BroadcastStream::new(broadway.flo_events.subscribe());

    let mut mavlink_task: Pin<Box<dyn Future<Output = _>>> =
        if let Some(ref mavlink_cfg) = &device_config.mavlink_config {
            tracing::info!("mavlink at {}", &mavlink_cfg.port_path);
            let tasks = flo_mavlink::spawn_mavlink(
                mavlink_cfg,
                broadway.clone(),
                flo_saver_tx.clone(),
                device_config.rc_config.as_ref(),
            )?;
            Box::pin(tasks.main_jh)
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

    // Spawn task for saving data to disk

    let mut converter_handle: JoinHandle<eyre::Result<()>> = {
        let mut rx1 = broadway.flo_events.subscribe();
        let mut rx2 = broadway.flo_detections.subscribe();
        // let mut rx3 = broadway.flo_motion.subscribe();
        let mut rx4 = broadway.drone_events.subscribe();
        let mut rx5 = broadway.drone_realtime.subscribe();

        let flo_saver_tx = flo_saver_tx.clone();
        tokio::spawn(async move {
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
        let device_config = device_config.clone();
        tokio::task::spawn_blocking(move || {
            writing_state::writer_task_main(flo_saver_rx, &device_config)
        })
    };

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
        tokio::spawn(async move {
            pwm_serial_io::run_rpi_pico_pwm_serial_loop(
                motors_rx,
                serial_device,
                pan_pwm_config,
                pan_motor_config,
                tilt_pwm_config,
                tilt_motor_config,
            )
            .await
        })
    } else if let Some(trinamic_pan) = cli.trinamic_pan {
        my_state.motor_type = MotorType::Trinamic;
        let pan_trinamic_config = device_config.pan_trinamic_config.as_mut().unwrap();
        let tilt_trinamic_config = device_config.tilt_trinamic_config.as_mut().unwrap();

        let trinamic_tilt = cli
            .trinamic_tilt
            .ok_or_else(|| anyhow::anyhow!("trinamic_tilt not given but needed"))?;

        let baud_rate = 115_200;

        let mot_cfg = trinamic::MotorParameters::TMCM1240(trinamic::TMCM1240Parameters::default());
        let pan_device = trinamic::Motor::new(&trinamic_pan, baud_rate, mot_cfg.clone()).await?;
        let tilt_device = trinamic::Motor::new(&trinamic_tilt, baud_rate, mot_cfg.clone()).await?;
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
        tokio::spawn(async move {
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
        tokio::spawn(async move {
            sbgc_gimbal::run_gimbal_loop(motors_rx, motor_position_tx, flo_saver_tx, cfg).await
        })
    } else {
        tracing::warn!("No motor control method specified.");
        tokio::spawn(async {
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
                    anyhow::bail!("trinamic focus with non-trinamic pan/tilt not yet implemented");
                }
                //nothing to do, already handled when setting up pan/tilt
                (Box::pin(pending()), true)
            }
            FocusMotorType::Tilta(tcfg) => {
                //let task = tilta_io::run_tilta_loop();
                //(Some(task), true)
                let port = tcfg.port.clone();
                let jh = tokio::spawn(async move {
                    tilta_io::run_tilta_loop(port.as_str(), motors_rx).await
                });
                (Box::pin(jh), true)
            }
        };
    my_state.focusing = focusing;

    // Setup FLO networking
    let mut udp_framed_recv = udp_handling::setup_udp(&cli.udp_addr).await?;

    // Create a channel to send copies of our device state.
    let (from_device_http_tx, from_device_http_rx) = watch::channel(my_state.clone());

    // Create HTTP server for user interface.
    let (tcp_listener, token_config) = flo_webserver::start_listener(&cli.http_addr)
        .await
        .with_context(|| format!("Opening TCP listener at address \"{}\"", cli.http_addr))?;

    // Run web server main loop
    let _http_server_join_handle = {
        let device_config = device_config.clone();
        let event_tx = broadway.flo_events.clone();
        tokio::spawn(async move {
            flo_webserver::main_loop(
                tcp_listener,
                token_config,
                from_device_http_rx,
                device_config,
                event_tx,
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

    //start up OSD
    let (osd_tx, mut osd_task): (_, Pin<Box<dyn Future<Output = _>>>) =
        if let Some(osd_config) = device_config.osd_config.clone() {
            let (osd_tx, osd_rx) = watch::channel(OsdState::default());
            let osd_join_handle = {
                let broadway = broadway.clone();
                tokio::spawn(async move {
                    osd::run_osd_loop(osd_rx, broadway, osd_config, canvas).await
                })
            };
            (Some(osd_tx), Box::pin(osd_join_handle))
        } else {
            let fut = pending(); // future never completes
            (None, Box::pin(fut))
        };

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
        motors_tx,
        audio_stream_handle.as_ref(),
        from_device_http_tx,
        broadway,
        &data_dir,
    )
    .await?;

    // Main loop.
    loop {
        // Wait for any of a number of things to happen
        tokio::select! {
            mavlink_result = &mut mavlink_task => {
                mavlink_result??;
                break;
            }
            osd_task_result = &mut osd_task => {
                osd_task_result??;
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
            udp_msg = udp_framed_recv.next() => {
                let udp_msg = udp_msg.ok_or_else(|| eyre::eyre!("UDP channel closed"))?;
                coordinator.on_image_centroid(udp_msg)?;
                // TODO: we have fresh data. We should call
                // `self.on_fast_tick(dt_secs)?;` here?
            }
            flo_event = flo_events_rx_stream.next() => {
                let flo_event = flo_event.ok_or_else(|| eyre::eyre!("FLO events channel closed"))??;
                coordinator.handle_event(flo_event)?;
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
    if let Some(audio_stream) = audio_stream {
        if let Some(filename) = device_config.sounds_filenames.filename(new_mode) {
            use rodio::{source::Source, Decoder};
            use std::fs::File;
            use std::io::BufReader;

            let file = BufReader::new(File::open(filename)?);
            let source = Decoder::new(file)?;
            audio_stream.play_raw(source.convert_samples())?;
        };
    };
    Ok(())
}
