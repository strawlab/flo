use eyre::{Ok, Result, WrapErr};
use mavlink::{
    MavHeader,
    ardupilotmega::{MavComponent, MavMessage, MavModeFlag},
};

use flo_core::{
    Angle, Broadway, CommandSource, DeviceMode, DisplaySource, DroneChannelData, FloCommand,
    FloEvent, FloatType, LocalFloState, ModeChangeReason, MyTimestamp, SaveToDiskMsg, StampedJson,
    drone_structs::{
        self, Attitude, BatteryState, ChannelCondition, DroneEvent, GlobalPosition, GnssDop,
        GnssRtkMode, GpsGlobalOrigin, GpsOriginCheck, LocalPositionNed,
    },
    elapsed, now,
};

mod ntrip;
mod param_snapshot;
mod px4_params;

/// The `mavlink` crate this was built against.
///
/// [`AutopilotLink::send`] takes an [`ardupilotmega::MavMessage`], so anything
/// composing a message has to name types from this crate. Reaching them through
/// here rather than declaring a second `mavlink` dependency is what makes it
/// impossible to end up holding a `MavMessage` from a different, silently
/// incompatible resolution of the same crate — a mistake that would otherwise
/// surface as a type error with two identically-spelled types in it.
///
/// [`ardupilotmega::MavMessage`]: mavlink::ardupilotmega::MavMessage
pub use mavlink;

/// MAVLink system ID of the flight controller FLO talks to.
///
/// Every message FLO addresses to the autopilot carries this as
/// `target_system`, and messages arriving from any other system are ignored.
/// MAVLink reserves no value for "the autopilot", so this is a convention: 1 is
/// what both ArduPilot and PX4 default to, and FLO has never been deployed
/// beside a flight controller configured otherwise. It is named rather than
/// repeated so an extension composing its own messages addresses the same
/// system FLO does, and so a future deployment that needs a different ID has one
/// place to look.
pub const AUTOPILOT_TARGET_SYSTEM: u8 = 1;

/// MAVLink component ID of the flight controller's autopilot component.
///
/// The companion to [`AUTOPILOT_TARGET_SYSTEM`] for `target_component`. Note
/// that `MAV_COMP_ID_AUTOPILOT1` is itself 1, so this and the system ID happen
/// to share a value; they are distinct fields and are kept as distinct
/// constants.
pub const AUTOPILOT_TARGET_COMPONENT: u8 = MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8;

/// Egress to the flight controller for code outside the MAVLink task.
///
/// [`spawn_mavlink`] returns one of these alongside the task's join handle. It
/// is cheap to clone and can be shared freely: sends go onto the same bounded
/// queue the MAVLink task writes to, so messages from every holder interleave on
/// the one serial link without any of them owning it.
///
/// The point of handing this out rather than a setpoint-shaped channel is that
/// the decisions a `SET_POSITION_TARGET_LOCAL_NED` encodes — which coordinate
/// frame, which fields the type mask ignores — belong to whoever computed the
/// trajectory, not to the transport. A holder composes the whole message and is
/// answerable for it.
///
/// One obligation comes with that, and it is not enforced here: address the
/// autopilot with [`AUTOPILOT_TARGET_SYSTEM`] and [`AUTOPILOT_TARGET_COMPONENT`]
/// for the `target_*` fields.
///
/// Recording, by contrast, is not the holder's job. [`Self::send`] writes every
/// message it queues into the `.floz`, so what actually reached the flight
/// controller is on record whether or not the sender thought about it.
#[derive(Clone)]
pub struct AutopilotLink {
    /// Cloned from the MAVLink task's sender, which means it does not carry that
    /// sender's write-error channel: a dead writer thread surfaces here as a
    /// closed queue rather than the underlying I/O error. That is tolerable
    /// because the MAVLink task still holds the real error, logs it, and exits —
    /// bringing the process down — so the specific cause is never lost, only
    /// unavailable at this particular call site.
    tx: tokio::sync::mpsc::Sender<(MavHeader, MavMessage)>,
    header: MavHeader,
    sys_start: MyTimestamp,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
}

impl AutopilotLink {
    /// Send one message to the flight controller, stamped with FLO's own
    /// system and component ID, and record it.
    ///
    /// Awaits a slot if the writer is behind. The queue is short, so this is
    /// backpressure on a slow serial link rather than an unbounded backlog:
    /// a caller publishing setpoints faster than the link drains them will be
    /// held here, which is the honest outcome.
    ///
    /// The message is recorded before it is queued, so a send that fails still
    /// leaves evidence of what was attempted.
    pub async fn send(&self, msg: MavMessage) -> Result<()> {
        save_tx(&self.floz_logger, &msg)?;
        self.tx
            .send((self.header, msg))
            .await
            .map_err(|_| eyre::eyre!("MAVLink writer is gone; cannot reach the flight controller"))
    }

    /// Milliseconds since the MAVLink connection was opened, for the
    /// `time_boot_ms` field several MAVLink messages carry.
    ///
    /// This shares FLO's time base rather than starting its own, so every
    /// message leaving under FLO's system ID agrees about when boot was.
    pub fn time_boot_ms(&self) -> u32 {
        (elapsed(self.sys_start) * 1000.0) as u32
    }
}

pub struct MavlinkPort {
    port: Box<dyn serialport::SerialPort>,
    cfg: flo_core::drone_structs::MavlinkConfig,
}

impl MavlinkPort {
    pub fn open(mavlink_cfg: &flo_core::drone_structs::MavlinkConfig) -> Result<Self> {
        let mavlink_cfg = mavlink_cfg.clone();

        let settings = &mavlink_cfg.port_path;

        let settings_toks: Vec<&str> = settings.split(':').collect();
        if settings_toks.len() < 2 {
            eyre::bail!(
                "Incomplete port settings. Expected format: serial://<device_path>:<baud_rate>"
            );
        }

        let settings_toks = &settings_toks[1..];

        let baud_rate = match settings_toks[1].parse() {
            core::result::Result::Ok(baud_rate) => baud_rate,
            Err(e) => {
                eyre::bail!("Error parsing baud rate: {e}");
            }
        };

        let path = settings_toks[0];
        tracing::info!("mavlink at {}", settings);
        let mut port = serialport::new(path, baud_rate)
            .open()
            .with_context(|| format!("Opening mavlink connection {}", settings))?;
        port.set_timeout(std::time::Duration::from_secs(60 * 60 * 24 * 365 * 100))?;
        let cfg = mavlink_cfg.clone();
        Ok(Self { port, cfg })
    }
}

fn convert_gnss_rtk_mode(fix_type: mavlink::ardupilotmega::GpsFixType) -> GnssRtkMode {
    use mavlink::ardupilotmega::GpsFixType::*;
    match fix_type {
        GPS_FIX_TYPE_NO_GPS => GnssRtkMode::NoGps,
        GPS_FIX_TYPE_NO_FIX => GnssRtkMode::NoFix,
        GPS_FIX_TYPE_2D_FIX => GnssRtkMode::TwoDFix,
        GPS_FIX_TYPE_3D_FIX => GnssRtkMode::ThreeDFix,
        GPS_FIX_TYPE_DGPS => GnssRtkMode::DGps,
        GPS_FIX_TYPE_RTK_FLOAT => GnssRtkMode::RtkFloat,
        GPS_FIX_TYPE_RTK_FIXED => GnssRtkMode::RtkFixed,
        GPS_FIX_TYPE_STATIC => GnssRtkMode::Static,
        GPS_FIX_TYPE_PPP => GnssRtkMode::Ppp,
    }
}

fn convert_satellites_visible(satellites_visible: u8) -> Option<u8> {
    (satellites_visible != u8::MAX).then_some(satellites_visible)
}

fn convert_dop(dop_times_100: u16) -> Option<FloatType> {
    (dop_times_100 != u16::MAX).then_some(dop_times_100 as FloatType / 100.0)
}

fn gnss_has_location(mode: &GnssRtkMode) -> bool {
    !matches!(mode, GnssRtkMode::NoGps | GnssRtkMode::NoFix)
}

/// A global origin changes the reference frame of local positions, so only do
/// it before takeoff and outside a recording.
fn origin_may_be_set(
    is_recording: bool,
    armed: Option<bool>,
    landed_state: Option<mavlink::ardupilotmega::MavLandedState>,
) -> bool {
    !is_recording
        && armed == Some(false)
        && landed_state == Some(mavlink::ardupilotmega::MavLandedState::MAV_LANDED_STATE_ON_GROUND)
}

fn is_local_position_out_of_bounds(v: &mavlink::ardupilotmega::LOCAL_POSITION_NED_DATA) -> bool {
    const MAX_LOCAL_POSITION_DIST_METERS: f32 = 10_000.0;
    (v.x * v.x + v.y * v.y + v.z * v.z)
        >= (MAX_LOCAL_POSITION_DIST_METERS * MAX_LOCAL_POSITION_DIST_METERS)
}

struct DroneCoordinator {
    mavlink_cfg: flo_core::drone_structs::MavlinkConfig,
    mavconn: tokio_mavlink::MavlinkConnection<MavMessage>,
    my_header: mavlink::MavHeader,
    rc_cfg: Option<flo_core::RcConfig>,
    broadway: flo_core::Broadway,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    rc_program_state: flo_core::drone_structs::RcProgramState,
    armed_cd: flo_core::utils::ChangeDetector<bool>,
    flight_mode_cd: flo_core::utils::ChangeDetector<u32>,
    local_position_out_of_bounds_cd: flo_core::utils::ChangeDetector<bool>,
    last_message_timestamp: Option<MyTimestamp>,
    prev_time_boot_ms: u32,
    local_flo_state: LocalFloState,
    /// The armed flag from the most recent heartbeat, `None` until the first one
    /// arrives.
    ///
    /// `armed_cd` cannot answer this: it starts out asserting `false`, which is
    /// what makes the first heartbeat register as a change, so it cannot tell a
    /// disarmed flight controller from one that has not been heard from at all.
    /// Anything that must not act on that guess — asking for a reboot, say —
    /// needs the distinction.
    last_reported_armed: Option<bool>,
    /// The most recent landed state, or `None` until the flight controller has
    /// reported one. A configured GPS origin is changed only on the ground.
    last_reported_landed_state: Option<mavlink::ardupilotmega::MavLandedState>,
    /// The local-position origin the config asks for, if any.
    requested_origin: Option<GpsGlobalOrigin>,
    /// Whether the configured origin has been sent at least once. Its first
    /// send waits for a usable GNSS fix; later sends are the bounded retries
    /// for a reported mismatch.
    origin_has_been_requested: bool,
    /// How many more times a mismatching origin will be re-sent before giving
    /// up and leaving the mismatch standing for the operator to act on.
    origin_retries_left: u8,
    /// When the origin was requested, so silence can be complained about. None
    /// once the request has been answered, or if none was made.
    origin_requested_at: Option<MyTimestamp>,
    /// The flight controller's parameter set, while it is still arriving. None
    /// before it has been asked for and once it has been stored or given up on,
    /// which is what keeps FLO from asking twice.
    param_snapshot: Option<param_snapshot::ParamSnapshot>,
}

/// How many times a `SET_GPS_GLOBAL_ORIGIN` that did not take hold is re-sent.
///
/// The flight controller streams `GPS_GLOBAL_ORIGIN` once a second, so this is
/// also roughly how many seconds FLO spends insisting before it stops.
const ORIGIN_SET_RETRIES: u8 = 5;

/// How long to wait for any `GPS_GLOBAL_ORIGIN` before saying so. The message
/// is requested at 1 Hz, so this is generous.
const ORIGIN_REPLY_TIMEOUT_SECS: FloatType = 10.0;

impl DroneCoordinator {
    async fn new(
        mavlink_cfg: &flo_core::drone_structs::MavlinkConfig,
        mavconn: tokio_mavlink::MavlinkConnection<MavMessage>,
        my_header: mavlink::MavHeader,
        rc_cfg: Option<flo_core::RcConfig>,
        broadway: flo_core::Broadway,
        floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
        local_flo_state: LocalFloState,
    ) -> Result<Self> {
        // fill initial struct
        let mut self_ = Self {
            mavlink_cfg: mavlink_cfg.clone(),
            mavconn,
            my_header,
            rc_cfg,
            broadway,
            floz_logger,
            rc_program_state: Default::default(),
            // Set a non-existing initial value so that true first value is detected as change.
            armed_cd: flo_core::utils::ChangeDetector::new_with_initial_state(&false),
            flight_mode_cd: flo_core::utils::ChangeDetector::new_with_initial_state(&0),
            local_position_out_of_bounds_cd:
                flo_core::utils::ChangeDetector::new_with_initial_state(&false),
            last_message_timestamp: Default::default(),
            prev_time_boot_ms: 0,
            local_flo_state,
            last_reported_armed: None,
            last_reported_landed_state: None,
            requested_origin: mavlink_cfg.requested_gps_global_origin(),
            origin_has_been_requested: false,
            origin_retries_left: ORIGIN_SET_RETRIES,
            origin_requested_at: None,
            param_snapshot: None,
        };
        {
            // This is also what makes `LocalFloStateInner::mavlink` `Some`,
            // and so what tells the BUI there is a flight controller at all.
            let mut state = self_.local_flo_state.write().unwrap();
            state.mavlink_mut().gps_origin.requested = self_.requested_origin;
        }

        // Below is the old Self::init() method, now moved into the constructor.

        if let Some(cfg) = &self_.rc_cfg {
            if let Some(knob_cfg) = &cfg.pan_knob {
                self_.rc_program_state.pan_ng.params = knob_cfg.noise_gate.clone();
            }
            if let Some(knob_cfg) = &cfg.tilt_knob {
                self_.rc_program_state.tilt_ng.params = knob_cfg.noise_gate.clone();
            }
        }

        // Bring the flight controller's own recording parameters in line with
        // the config. This goes first because it can end in a reboot, which
        // would discard the stream requests below and the global origin with
        // them.
        self_.apply_px4_param_config().await?;

        // Below is the old Self::request_streams() method, now moved into the constructor.

        use mavlink::MessageData as _;

        // Request the GPS_GLOBAL_ORIGIN message to be streamed at 1 second interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::GPS_GLOBAL_ORIGIN_DATA::ID as f32, // Message ID to be streamed
                param2: 1_000_000.0, // Interval in microseconds
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // The origin is only changed while the vehicle is reported on the
        // ground, so request that state rather than relying on a default stream.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::EXTENDED_SYS_STATE_DATA::ID as f32,
                param2: 1_000_000.0,
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // This is the estimator's global counterpart to LOCAL_POSITION_NED.
        // Request it explicitly so debugging their relationship does not
        // depend on the flight controller's default stream profile.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::GLOBAL_POSITION_INT_DATA::ID as f32,
                param2: 1_000_000.0,
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // GPS_RAW_INT says whether the receiver has a position fix. The
        // configured origin is deliberately withheld until then, so changing
        // the origin cannot make LOCAL_POSITION_NED refer to a distant stale
        // estimate.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::GPS_RAW_INT_DATA::ID as f32,
                param2: 1_000_000.0,
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // Keep the configured origin visible while waiting to see a usable GPS
        // position. It is sent from the GPS_RAW_INT handler once that arrives.
        if self_.requested_origin.is_some() {
            self_.set_origin_status_check(GpsOriginCheck::AwaitingGpsFix);
        }

        // Request the RC_CHANNELS message to be streamed at 4 msec interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::RC_CHANNELS_DATA::ID as f32, // Message ID to be streamed
                param2: 4000.0, // Interval in microseconds
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // Request the SYSTEM_TIME message to be streamed at 1 second interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::SYSTEM_TIME_DATA::ID as f32, // Message ID to be streamed
                param2: 1_000_000.0, // Interval in microseconds
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // Request the LOCAL_POSITION_NED message to be streamed at 10 millisecond interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::LOCAL_POSITION_NED_DATA::ID as f32, // Message ID to be streamed
                param2: 10_000.0, // Interval in microseconds
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // Request the ATTITUDE_QUATERNION message to be streamed at 10 millisecond interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::ATTITUDE_QUATERNION_DATA::ID as f32, // Message ID to be streamed
                param2: 10_000.0, // Interval in microseconds
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // Request the ODOMETRY message to be streamed at 100 millisecond interval.
        // This contains covariance of PX4 EKFs.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::ODOMETRY_DATA::ID as f32, // Message ID to be streamed
                param2: 100_000.0, // Interval in microseconds
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.send_to_autopilot(data).await?;

        // Read the flight controller's whole parameter set, for the recordings
        // this session is about to make. Last, and only started here rather than
        // waited for: the answers arrive over the following seconds through the
        // main loop, so this costs no startup time and leaves FLO responsive to
        // RC and position while the link is busy with them.
        self_.start_param_snapshot().await?;

        Ok(self_)
    }

    /// Send one message to the flight controller, and record it.
    ///
    /// The counterpart to [`AutopilotLink::send`], and it exists separately for
    /// one reason: this uses the task's own [`tokio_mavlink::MavlinkSender`],
    /// which carries the writer thread's error channel, so a failed write here
    /// reports the underlying I/O error rather than just a closed queue. Both
    /// paths record through [`save_tx`], so `mavlink.jsonl` holds everything FLO
    /// sent regardless of which one sent it.
    async fn send_to_autopilot(&mut self, data: MavMessage) -> Result<()> {
        save_tx(&self.floz_logger, &data)?;
        self.mavconn.tx.send((self.my_header, data)).await?;
        Ok(())
    }

    /// Ask the flight controller to use the configured local-position origin.
    async fn send_gps_global_origin(&mut self) -> Result<()> {
        let Some(origin) = self.requested_origin else {
            return Ok(());
        };
        let data = mavlink::ardupilotmega::MavMessage::SET_GPS_GLOBAL_ORIGIN(
            mavlink::ardupilotmega::SET_GPS_GLOBAL_ORIGIN_DATA {
                latitude: origin.latitude_e7,
                longitude: origin.longitude_e7,
                altitude: origin.altitude_mm,
                target_system: AUTOPILOT_TARGET_SYSTEM,
            },
        );
        self.send_to_autopilot(data).await?;
        tracing::info!("Sent SET_GPS_GLOBAL_ORIGIN: {origin}");
        Ok(())
    }

    /// Send the configured origin only when doing so cannot disturb flight or
    /// change the coordinate frame of data currently being recorded.
    async fn send_gps_global_origin_when_safe(&mut self) -> Result<()> {
        if self.requested_origin.is_none() || self.origin_has_been_requested {
            return Ok(());
        }
        let (has_gnss_location, is_recording) = {
            let state = self.local_flo_state.read().unwrap();
            (
                state
                    .mavlink
                    .as_ref()
                    .and_then(|mavlink| mavlink.gnss_location)
                    .is_some(),
                state.is_recording,
            )
        };
        if !has_gnss_location {
            self.set_origin_status_check(GpsOriginCheck::AwaitingGpsFix);
            return Ok(());
        }
        if !origin_may_be_set(
            is_recording,
            self.last_reported_armed,
            self.last_reported_landed_state,
        ) {
            self.set_origin_status_check(GpsOriginCheck::AwaitingSafeToSet);
            return Ok(());
        }

        self.origin_has_been_requested = true;
        self.set_origin_status_check(GpsOriginCheck::Awaiting);
        self.origin_requested_at = Some(now());
        self.send_gps_global_origin().await
    }

    fn set_origin_status_check(&mut self, check: GpsOriginCheck) {
        self.local_flo_state
            .write()
            .unwrap()
            .mavlink_mut()
            .gps_origin
            .check = check;
    }

    /// Compare what the flight controller reports against what FLO asked for.
    ///
    /// A `SET_GPS_GLOBAL_ORIGIN` can be silently ignored — the flight
    /// controller may refuse it while armed, or already have an origin from its
    /// own first fix. Everything FLO computes from `LOCAL_POSITION_NED` is then
    /// relative to the wrong point with nothing to show for it, so the request
    /// is re-sent a bounded number of times and the mismatch is both logged and
    /// published for the BUI.
    async fn check_gps_global_origin(
        &mut self,
        reported: mavlink::ardupilotmega::GPS_GLOBAL_ORIGIN_DATA,
    ) -> Result<()> {
        let reported = GpsGlobalOrigin {
            latitude_e7: reported.latitude,
            longitude_e7: reported.longitude,
            altitude_mm: reported.altitude,
        };
        if !self.origin_has_been_requested {
            // The controller may publish its own origin before it has a usable
            // fix. Record it, but do not call it a mismatch until FLO has sent
            // the configured origin after receiving a valid GPS position.
            self.local_flo_state
                .write()
                .unwrap()
                .mavlink_mut()
                .gps_origin
                .reported = Some(reported);
            return Ok(());
        }
        self.origin_requested_at = None;
        let (check, was) = {
            let mut state = self.local_flo_state.write().unwrap();
            let mavlink = state.mavlink_mut();
            let was = mavlink.gps_origin.check;
            (mavlink.gps_origin.on_reported(reported), was)
        };
        match check {
            GpsOriginCheck::Confirmed => {
                if was != GpsOriginCheck::Confirmed {
                    tracing::info!("flight controller confirmed GPS global origin {reported}");
                }
                self.origin_retries_left = ORIGIN_SET_RETRIES;
            }
            GpsOriginCheck::Mismatched => {
                // `requested` is Some here: that is what makes a mismatch
                // possible in the first place.
                let requested = self.requested_origin.unwrap();
                if self.origin_retries_left > 0 {
                    self.origin_retries_left -= 1;
                    tracing::warn!(
                        "flight controller is using GPS global origin {reported}, not the \
                         configured {requested}; re-sending SET_GPS_GLOBAL_ORIGIN \
                         ({} attempt(s) left)",
                        self.origin_retries_left
                    );
                    self.send_gps_global_origin().await?;
                } else {
                    tracing::error!(
                        "flight controller kept GPS global origin {reported} despite \
                         {ORIGIN_SET_RETRIES} attempts to set {requested}. Local positions \
                         are relative to the wrong point."
                    );
                }
            }
            GpsOriginCheck::NotRequested
            | GpsOriginCheck::AwaitingGpsFix
            | GpsOriginCheck::AwaitingSafeToSet
            | GpsOriginCheck::Awaiting => {}
        }
        Ok(())
    }

    /// Complain once if the flight controller never answers with a
    /// `GPS_GLOBAL_ORIGIN`, which is the other way a request can fail to stick:
    /// not contradicted, just ignored.
    fn warn_if_origin_unanswered(&mut self) {
        let Some(requested_at) = self.origin_requested_at else {
            return;
        };
        if elapsed(requested_at) < ORIGIN_REPLY_TIMEOUT_SECS {
            return;
        }
        self.origin_requested_at = None;
        tracing::error!(
            "no GPS_GLOBAL_ORIGIN from the flight controller {ORIGIN_REPLY_TIMEOUT_SECS} s after \
             SET_GPS_GLOBAL_ORIGIN. Whether the configured origin took hold is unknown."
        );
    }

    async fn send_heartbeat(&mut self) -> Result<()> {
        let data =
            mavlink::ardupilotmega::MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
                system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
                mavlink_version: 0x3,
            });

        self.send_to_autopilot(data).await?;
        Ok(())
    }

    async fn handle_message_from_drone(
        &mut self,
        header: MavHeader,
        msg: MavMessage,
    ) -> Result<()> {
        if !(header.system_id == AUTOPILOT_TARGET_SYSTEM
            && header.component_id == AUTOPILOT_TARGET_COMPONENT)
        {
            tracing::trace!(
                "ignoring message not from flight controller: {header:?}, msg: {msg:?}"
            );
            // not from flight controller - ignore this message
            return Ok(());
        }

        tracing::trace!("header: {header:?}, msg: {msg:?}");

        if self.last_message_timestamp.is_none() {
            tracing::debug!("mavlink established");
        }
        self.last_message_timestamp = Some(now());
        let logger = &self.floz_logger;

        match msg {
            MavMessage::HEARTBEAT(msg) => {
                let armed = msg
                    .base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                self.last_reported_armed = Some(armed);
                if self.armed_cd.update(&armed) {
                    self.broadway.drone_events.send(if armed {
                        DroneEvent::Armed
                    } else {
                        DroneEvent::Disarmed
                    })?;
                }
                let fm = msg.custom_mode;
                self.local_flo_state
                    .write()
                    .unwrap()
                    .mavlink_mut()
                    .custom_mode = Some(fm);
                if self.flight_mode_cd.update(&fm) {
                    self.broadway
                        .drone_events
                        .send(DroneEvent::FlightModeChanged(
                            self.flight_mode_cd
                                .latest_change
                                .unwrap()
                                .0
                                .map(|fm| fm.into()),
                            fm.into(),
                        ))?;
                }
                self.send_gps_global_origin_when_safe().await?;
            }
            MavMessage::BATTERY_STATUS(bs) => {
                self.broadway
                    .drone_events
                    .send(DroneEvent::BatteryState(BatteryState {
                        batt_voltage: bs.voltages[0] as FloatType
                            / self.mavlink_cfg.batt_s as FloatType
                            / 1000.0,
                        batt_percent: bs.battery_remaining as FloatType,
                        timestamp: now(),
                    }))?;
            }
            MavMessage::ALTITUDE(_)
            | MavMessage::ATTITUDE_TARGET(_)
            | MavMessage::CURRENT_EVENT_SEQUENCE(_)
            | MavMessage::ESTIMATOR_STATUS(_)
            | MavMessage::LINK_NODE_STATUS(_)
            | MavMessage::TIMESYNC(_)
            | MavMessage::RADIO_STATUS(_)
            | MavMessage::SYS_STATUS(_)
            | MavMessage::VIBRATION(_)
            | MavMessage::VFR_HUD(_) => {
                if header.system_id != 1 {
                    // ignore this message

                    // println!("header: {header:?}");
                    // println!("  received: {msg:?}");
                } else {
                    // ignore this message
                }
            }
            MavMessage::PARAM_VALUE(data) => {
                // Not recorded to `mavlink.jsonl`: enumerating the parameter set
                // produces some 1400 of these in a burst, and the snapshot they
                // are assembled into is a better record of the same thing than
                // 1400 lines of it would be.
                self.note_param_value(&data);
            }
            MavMessage::RC_CHANNELS(data) => {
                self.handle_rc(data).await?;
            }
            MavMessage::SYSTEM_TIME(v) => {
                save("SYSTEM_TIME", logger, &v)?;
                if v.time_boot_ms < self.prev_time_boot_ms {
                    // This will roll over after 49.7 days, but we ignore that.
                    eyre::bail!(
                        "Received SYSTEM_TIME with time_boot_ms {} < previous {}. Did the flight controller reset?",
                        v.time_boot_ms,
                        self.prev_time_boot_ms
                    );
                } else {
                    self.prev_time_boot_ms = v.time_boot_ms;
                }
            }
            MavMessage::UTM_GLOBAL_POSITION(v) => {
                save("UTM_GLOBAL_POSITION", logger, &v)?;
            }
            MavMessage::GPS_RAW_INT(v) => {
                let mode = convert_gnss_rtk_mode(v.fix_type);
                {
                    let mut state = self.local_flo_state.write().unwrap();
                    let mavlink = state.mavlink_mut();
                    mavlink.gnss_location = gnss_has_location(&mode).then_some(GlobalPosition {
                        latitude_e7: v.lat,
                        longitude_e7: v.lon,
                        altitude_msl_mm: v.alt,
                    });
                    mavlink.gnss_dop = Some(GnssDop {
                        horizontal: convert_dop(v.eph),
                        vertical: convert_dop(v.epv),
                    });
                    mavlink.gnss_rtk_mode = Some(mode);
                    mavlink.satellites_visible = convert_satellites_visible(v.satellites_visible);
                }
                save("GPS_RAW_INT", logger, &v)?;
                self.send_gps_global_origin_when_safe().await?;
            }
            MavMessage::EXTENDED_SYS_STATE(v) => {
                self.last_reported_landed_state = Some(v.landed_state);
                self.send_gps_global_origin_when_safe().await?;
            }
            MavMessage::GLOBAL_POSITION_INT(v) => {
                let position = GlobalPosition {
                    latitude_e7: v.lat,
                    longitude_e7: v.lon,
                    altitude_msl_mm: v.alt,
                };
                self.local_flo_state
                    .write()
                    .unwrap()
                    .mavlink_mut()
                    .fused_global_position = Some(position);
                save("GLOBAL_POSITION_INT", logger, &v)?;
            }
            MavMessage::LOCAL_POSITION_NED(v) => {
                let local_position_out_of_bounds = is_local_position_out_of_bounds(&v);
                {
                    let mut state = self.local_flo_state.write().unwrap();
                    state.local_position_out_of_bounds = local_position_out_of_bounds;
                    state.mavlink_mut().local_position = Some(LocalPositionNed {
                        north_m: v.x as FloatType,
                        east_m: v.y as FloatType,
                        down_m: v.z as FloatType,
                    });
                }
                if self
                    .local_position_out_of_bounds_cd
                    .update_and_has_changed_to(&local_position_out_of_bounds, &true)
                {
                    tracing::error!("local position {v:?} is more than 10km from global origin");
                }
                save("LOCAL_POSITION_NED", logger, &v)?;
            }
            // Both attitude messages carry the same attitude, and which one a
            // flight controller streams varies, so whichever arrives is used.
            // FLO asks for `ATTITUDE_QUATERNION` explicitly; `ATTITUDE` is in
            // the default stream set of the controllers seen so far.
            MavMessage::ATTITUDE(v) => {
                self.local_flo_state.write().unwrap().mavlink_mut().attitude = Some(Attitude {
                    roll_rad: v.roll as FloatType,
                    pitch_rad: v.pitch as FloatType,
                    yaw_rad: v.yaw as FloatType,
                });
            }
            MavMessage::ATTITUDE_QUATERNION(v) => {
                self.local_flo_state.write().unwrap().mavlink_mut().attitude =
                    Some(Attitude::from_quaternion(
                        v.q1 as FloatType,
                        v.q2 as FloatType,
                        v.q3 as FloatType,
                        v.q4 as FloatType,
                    ));
                save("ATTITUDE_QUATERNION", logger, &v)?;
            }
            MavMessage::ODOMETRY(v) => {
                save("ODOMETRY", logger, &v)?;
            }
            MavMessage::GPS_GLOBAL_ORIGIN(v) => {
                tracing::debug!("received GPS_GLOBAL_ORIGIN: {v:?}");
                save("GPS_GLOBAL_ORIGIN", logger, &v)?;
                self.check_gps_global_origin(v).await?;
            }
            msg => {
                tracing::trace!("unknown mavlink message: {msg:?}");
            }
        }

        Ok(())
    }

    async fn handle_rc(&mut self, data: mavlink::ardupilotmega::RC_CHANNELS_DATA) -> Result<()> {
        let vals = [
            data.chan1_raw,
            data.chan2_raw,
            data.chan3_raw,
            data.chan4_raw,
            data.chan5_raw,
            data.chan6_raw,
            data.chan7_raw,
            data.chan8_raw,
            data.chan9_raw,
            data.chan10_raw,
            data.chan11_raw,
            data.chan12_raw,
            data.chan13_raw,
            data.chan14_raw,
            data.chan15_raw,
            data.chan16_raw,
            data.chan17_raw,
            data.chan18_raw,
        ];

        let rc = if let Some(rc_cfg) = &self.rc_cfg {
            DroneChannelData {
                timestamp: now(),
                values: std::array::from_fn(|i| rc_cfg.us_mapping.convert(vals[i] as FloatType)),
            }
        } else {
            tracing::warn!("Received RC message, but ignoring because no rc_cfg set.");
            return Ok(());
        };

        self.broadway
            .drone_realtime
            .send(drone_structs::DroneRealtimeEvent::RcChannels(rc.clone()))?;

        self.handle_rc_flo_control(&rc)?;

        Ok(())
    }

    fn handle_rc_flo_control(&mut self, rc: &DroneChannelData) -> Result<()> {
        if self.rc_cfg.is_none() {
            return Ok(());
        };
        let cfg = self.rc_cfg.as_ref().unwrap();

        let track_start = ChannelCondition::test(&cfg.track_start, rc);
        let track_stop = ChannelCondition::test(&cfg.track_stop, rc);
        let set_home = ChannelCondition::test(&cfg.set_home, rc);
        let display_ir = ChannelCondition::test(&cfg.display_ir, rc);

        // start tracking
        if self
            .rc_program_state
            .track_start_cd
            .update_and_has_changed_to(&track_start, &true)
        {
            self.broadway.flo_events.send(FloEvent::Command(
                FloCommand::SwitchMode(DeviceMode::AcquiringLock, ModeChangeReason::Operator),
                CommandSource::DroneRC,
            ))?;
        }

        // set home
        if self
            .rc_program_state
            .set_home_cd
            .update_and_has_changed_to(&set_home, &true)
        {
            self.broadway.flo_events.send(FloEvent::Command(
                FloCommand::SetHomePositionFromCurrent,
                CommandSource::DroneRC,
            ))?;
        }

        // stop tracking
        if self
            .rc_program_state
            .track_stop_cd
            .update_and_has_changed_to(&track_stop, &true)
        {
            self.broadway.flo_events.send(FloEvent::Command(
                FloCommand::SwitchToOpenLoop,
                CommandSource::DroneRC,
            ))?;
        }

        // Choose what the operator's live view shows.
        if let Some(source) =
            rc_display_source(&mut self.rc_program_state.display_ir_cd, display_ir)
        {
            self.broadway.flo_events.send(FloEvent::Command(
                FloCommand::SetDisplaySource(source),
                CommandSource::DroneRC,
            ))?;
        }
        {
            use flo_core::utils::NoiseGateStatus as S;
            let mut new_home: (Option<Angle>, Option<Angle>) = (None, None);
            if let Some(knob_cfg) = &cfg.pan_knob {
                let (s, val) = self.rc_program_state.pan_ng.update(rc.get(knob_cfg.ch_no));
                if s == S::Varying {
                    new_home.0 = Some(Angle::from_degrees(val * knob_cfg.max_angle))
                }
            };
            if let Some(knob_cfg) = &cfg.tilt_knob {
                let (s, val) = self.rc_program_state.tilt_ng.update(rc.get(knob_cfg.ch_no));
                if s == S::Varying {
                    new_home.1 = Some(Angle::from_degrees(val * knob_cfg.max_angle))
                }
            }

            if new_home != (None, None) {
                self.broadway.flo_events.send(FloEvent::Command(
                    FloCommand::SetHomePosition((new_home.0, new_home.1, None)),
                    CommandSource::DroneRC,
                ))?;
            }
        }

        Ok(())
    }
}

fn save<T: serde::Serialize>(
    name: &str,
    logger: &tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    v: &T,
) -> Result<()> {
    logger.send(SaveToDiskMsg::MavlinkData(StampedJson::new(
        v,
        name.into(),
    )?))?;
    Ok(())
}

/// Record a message on its way out to the flight controller.
///
/// Lands in the same `mavlink.jsonl` as the receive side, and is named
/// `TX_<MESSAGE>` so direction is legible without having to know which message
/// types FLO only ever receives. The payload is the whole [`MavMessage`], which
/// serializes with a `type` tag of its own, so the record says what was actually
/// on the wire — every field, as encoded — rather than whatever the sender
/// computed it from.
///
/// One egress path does not come through here: NTRIP's `GPS_RTCM_DATA`, which
/// holds its own cloned sender. Recording it as MAVLink would be the wrong shape
/// — a stream of opaque correction bytes chopped into 180-byte fragments, when
/// what a reader wants is the RTCM3 stream itself. [`ntrip::ntrip_loop`] records
/// it instead, verbatim and unfragmented, to [`flo_core::NTRIP_RTCM_FNAME`].
fn save_tx(
    logger: &tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    msg: &MavMessage,
) -> Result<()> {
    use mavlink::Message as _;
    save(&format!("TX_{}", msg.message_name()), logger, msg)
}

/// How long to wait before the first attempt at restarting NTRIP, and the
/// ceiling that wait doubles up to.
///
/// Reconnecting is the NTRIP client's own job and it does that itself for as
/// long as the failure looks transient — indefinitely, if it cannot reach the
/// caster at all. Some failures still reach us: a 401, which it will not retry
/// because credentials the caster has rejected do not start working, and its
/// give-up limit after a hundred attempts the caster answered. Its docs are
/// explicit that stopping there is a handover to the caller rather than a
/// verdict, and this is the caller.
///
/// Corrections are an enhancement — the flight controller falls back to non-RTK
/// GNSS without them — so a caster having a bad afternoon must not end the
/// flight. Restart the task instead, backing off so a genuinely misconfigured
/// URL does not spin, and let `ntrip_kbps` (which decays to zero on its own once
/// bytes stop arriving) show the operator that no corrections are being
/// received.
const NTRIP_MIN_RESTART_BACKOFF: tokio::time::Duration = tokio::time::Duration::from_secs(1);
const NTRIP_MAX_RESTART_BACKOFF: tokio::time::Duration = tokio::time::Duration::from_secs(60);

/// Spawn [`ntrip::ntrip_loop`], optionally after waiting first.
///
/// Spawned rather than awaited inside `main_loop`'s `select!` so that relaying
/// corrections makes progress while the loop is busy handling a message.
fn spawn_ntrip_loop(
    handle: &tokio::runtime::Handle,
    ntrip_url: String,
    mavconn_tx: tokio::sync::mpsc::Sender<(MavHeader, MavMessage)>,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    rate_tx: tokio::sync::mpsc::UnboundedSender<usize>,
    header: MavHeader,
    wait_first: Option<tokio::time::Duration>,
) -> tokio::task::JoinHandle<eyre::Result<ntrip::NeverOk>> {
    handle.spawn(async move {
        if let Some(wait_first) = wait_first {
            tokio::time::sleep(wait_first).await;
        }
        ntrip::ntrip_loop(ntrip_url, mavconn_tx, floz_logger, rate_tx, header).await
    })
}

// Every argument is a distinct collaborator wired in from `spawn_mavlink`, and
// nearly all of them are forwarded straight to `DroneCoordinator::new`; bundling
// them into a struct would only move the same list one level out.
#[expect(clippy::too_many_arguments)]
async fn main_loop(
    handle: &tokio::runtime::Handle,
    mavlink_cfg: &flo_core::drone_structs::MavlinkConfig,
    mavconn: tokio_mavlink::MavlinkConnection<MavMessage>,
    rc_cfg: Option<flo_core::RcConfig>,
    broadway: flo_core::Broadway,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    local_flo_state: LocalFloState,
    header: MavHeader,
) -> eyre::Result<()> {
    // This is hacky, but we need to clone the mavconn sender for NTRIP.
    let mavconn_tx = mavconn.tx.hacky_clone_tx();
    // NTRIP records the corrections it relays, so it needs the recording sink
    // too. Cloned before the coordinator takes ownership of the original.
    let ntrip_floz_logger = floz_logger.clone();
    let ntrip_configured = mavlink_cfg.ntrip_url.is_some();
    let (ntrip_rate_tx, mut ntrip_rate_rx) = tokio::sync::mpsc::unbounded_channel();
    let mut ntrip_rate = ntrip::RollingByteRate::default();

    let mut coordinator = DroneCoordinator::new(
        mavlink_cfg,
        mavconn,
        header,
        rc_cfg,
        broadway,
        floz_logger,
        local_flo_state,
    )
    .await?;

    // For GNSS RTK, connect to NTRIP server and send RTCM data to the autopilot.
    // The senders are cloned into the task rather than moved so that it can be
    // restarted: see `NTRIP_MIN_RESTART_BACKOFF`.
    let mut ntrip_restart_backoff = NTRIP_MIN_RESTART_BACKOFF;
    let mut ntrip_task: std::pin::Pin<Box<dyn std::future::Future<Output = _> + Send>> = {
        if let Some(ntrip_url) = &mavlink_cfg.ntrip_url {
            Box::pin(spawn_ntrip_loop(
                handle,
                ntrip_url.clone(),
                mavconn_tx.clone(),
                ntrip_floz_logger.clone(),
                ntrip_rate_tx.clone(),
                header,
                None,
            ))
        } else {
            Box::pin(std::future::pending())
        }
    };

    let mut send_heartbeat_interval = tokio::time::interval(tokio::time::Duration::from_secs(1));
    send_heartbeat_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    if ntrip_configured {
        coordinator
            .local_flo_state
            .write()
            .unwrap()
            .mavlink_mut()
            .ntrip_kbps = Some(0.0);
    }

    loop {
        tokio::select! {
            _ = send_heartbeat_interval.tick() => {
                if ntrip_configured {
                    coordinator
                        .local_flo_state
                        .write()
                        .unwrap()
                        .mavlink_mut()
                        .ntrip_kbps = Some(ntrip_rate.kbps(tokio::time::Instant::now()));
                }
                coordinator.send_heartbeat().await?;
                coordinator.warn_if_origin_unanswered();
                coordinator.poll_param_snapshot().await?;
            },
            Some(received_bytes) = ntrip_rate_rx.recv() => {
                ntrip_rate.record(tokio::time::Instant::now(), received_bytes);
                // Corrections are flowing, so whatever we backed off from is
                // over: the next failure starts from the shortest wait again.
                ntrip_restart_backoff = NTRIP_MIN_RESTART_BACKOFF;
            }
            r = coordinator.mavconn.rx.recv() => {
                let (header, msg) = r?;
                coordinator.handle_message_from_drone(header, msg).await?;
            }
            ntrip_result = &mut ntrip_task => {
                // Deliberately not propagated: losing corrections degrades RTK
                // to plain GNSS, which is not a reason to stop FLO. See
                // `NTRIP_MIN_RESTART_BACKOFF`.
                // Spelled out because `Ok` in this crate is `eyre::Ok`, a
                // function, which cannot appear in a pattern.
                let error = match ntrip_result {
                    std::result::Result::Err(join_error) => eyre::Report::new(join_error)
                        .wrap_err("the NTRIP task did not shut down cleanly"),
                    std::result::Result::Ok(std::result::Result::Err(error)) => error,
                    std::result::Result::Ok(std::result::Result::Ok(never)) => match never {},
                };
                tracing::error!(
                    "NTRIP stopped; no RTK corrections are reaching the flight controller. \
                     Restarting it in {ntrip_restart_backoff:?}. {error:?}"
                );
                ntrip_task = Box::pin(spawn_ntrip_loop(
                    handle,
                    // Only reachable when NTRIP was configured: the `else`
                    // branch above never resolves.
                    mavlink_cfg.ntrip_url.clone().unwrap(),
                    mavconn_tx.clone(),
                    ntrip_floz_logger.clone(),
                    ntrip_rate_tx.clone(),
                    header,
                    Some(ntrip_restart_backoff),
                ));
                ntrip_restart_backoff =
                    (ntrip_restart_backoff * 2).min(NTRIP_MAX_RESTART_BACKOFF);
            }
        }
    }
}

/// spawns tokio task to handle mavlink.
///
/// This returns immediately with the join handle to the spawned task and an
/// [`AutopilotLink`] for sending to the flight controller from elsewhere. The
/// link is returned whether or not anything wants it: it is inert until someone
/// sends on it, and building it here is what lets it share the task's header and
/// `time_boot_ms` base.
pub fn spawn_mavlink(
    handle: &tokio::runtime::Handle,
    broadway: Broadway,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    rc_cfg: Option<&flo_core::RcConfig>,
    mavlink_port: MavlinkPort,
    local_flo_state: LocalFloState,
) -> eyre::Result<(tokio::task::JoinHandle<eyre::Result<()>>, AutopilotLink)> {
    let rc_cfg: Option<flo_core::RcConfig> = rc_cfg.cloned();

    let MavlinkPort {
        port: read_port,
        cfg: mavlink_cfg,
    } = mavlink_port;
    let write_port = read_port.try_clone()?;

    let mavconn = tokio_mavlink::spawn_mavlink_threads(
        read_port,
        write_port,
        10,
        10,
        mavlink::MavlinkVersion::V1,
    )?;

    let header = mavlink::MavHeader {
        system_id: mavlink_cfg.system_id,
        component_id: mavlink_cfg.component_id,
        sequence: 0,
    };

    // Established before the task starts, and shared with it, so that FLO's own
    // `time_boot_ms` and any sent through the returned link count from the same
    // instant.
    let sys_start = now();

    let autopilot_link = AutopilotLink {
        tx: mavconn.tx.hacky_clone_tx(),
        header,
        sys_start,
        floz_logger: floz_logger.clone(),
    };

    let handle2 = handle.clone();
    let main_jh = handle.spawn(async move {
        main_loop(
            &handle2,
            &mavlink_cfg,
            mavconn,
            rc_cfg,
            broadway,
            floz_logger,
            local_flo_state,
            header,
        )
        .await
    });

    Ok((main_jh, autopilot_link))
}

/// The display source the RC switch is asking for, or `None` if it has not
/// changed since the previous RC frame.
///
/// Unlike the other RC conditions this one is a *level*, not a trigger: both
/// directions have to be acted on, so this uses [`ChangeDetector::update`]
/// rather than `update_and_has_changed_to`. Note that
/// [`ChannelCondition::test`] reports false for stale RC data, so losing the
/// link falls the view back to the webcam, which is the safe default.
fn rc_display_source(
    display_ir_cd: &mut flo_core::ChangeDetector<bool>,
    display_ir: bool,
) -> Option<DisplaySource> {
    let requested = if display_ir {
        DisplaySource::StrandCamMain
    } else {
        DisplaySource::Webcam
    };
    display_ir_cd.update(&display_ir).then_some(requested)
}

#[cfg(test)]
mod tests {
    use flo_core::drone_structs::RcProgramState;

    use super::{
        DisplaySource, convert_dop, convert_satellites_visible, gnss_has_location,
        is_local_position_out_of_bounds, origin_may_be_set, rc_display_source,
    };

    #[test]
    fn mavlink_unknown_satellite_count_is_not_shown_as_255() {
        assert_eq!(convert_satellites_visible(18), Some(18));
        assert_eq!(convert_satellites_visible(u8::MAX), None);
    }

    #[test]
    fn mavlink_dop_scaling_and_unknown_sentinel_are_honored() {
        assert_eq!(convert_dop(73), Some(0.73));
        assert_eq!(convert_dop(u16::MAX), None);
    }

    #[test]
    fn gnss_location_is_only_valid_once_there_is_a_position_fix() {
        assert!(!gnss_has_location(&flo_core::GnssRtkMode::NoGps));
        assert!(!gnss_has_location(&flo_core::GnssRtkMode::NoFix));
        assert!(gnss_has_location(&flo_core::GnssRtkMode::TwoDFix));
        assert!(gnss_has_location(&flo_core::GnssRtkMode::RtkFixed));
    }

    #[test]
    fn gps_origin_waits_until_the_vehicle_is_disarmed_and_on_the_ground() {
        use super::mavlink::ardupilotmega::MavLandedState;

        assert!(origin_may_be_set(
            false,
            Some(false),
            Some(MavLandedState::MAV_LANDED_STATE_ON_GROUND)
        ));
        assert!(!origin_may_be_set(
            true,
            Some(false),
            Some(MavLandedState::MAV_LANDED_STATE_ON_GROUND)
        ));
        assert!(!origin_may_be_set(
            false,
            Some(true),
            Some(MavLandedState::MAV_LANDED_STATE_ON_GROUND)
        ));
        assert!(!origin_may_be_set(
            false,
            Some(false),
            Some(MavLandedState::MAV_LANDED_STATE_IN_AIR)
        ));
        assert!(!origin_may_be_set(false, None, None));
    }

    /// The switch is a level, so the operator flicking it either way has to
    /// change the view — and holding it still must not resend on every RC frame.
    #[test]
    fn the_rc_display_switch_acts_on_both_directions_and_only_on_change() {
        let mut state = RcProgramState::default();
        let mut step = |on| rc_display_source(&mut state.display_ir_cd, on);

        assert_eq!(
            step(false),
            None,
            "the webcam is already the display source at startup; nothing to say"
        );
        assert_eq!(step(false), None, "and holding the switch stays quiet");

        assert_eq!(step(true), Some(DisplaySource::StrandCamMain));
        assert_eq!(step(true), None, "held on, not resent every frame");

        assert_eq!(
            step(false),
            Some(DisplaySource::Webcam),
            "flicking back has to switch the view back"
        );
    }

    /// After a FLO restart the operator may still be holding the switch in the
    /// IR position, and no transition is coming. The seeded change detector is
    /// what makes the first RC frame re-assert it.
    #[test]
    fn an_ir_view_is_reasserted_on_the_first_rc_frame_after_a_restart() {
        let mut state = RcProgramState::default();
        assert_eq!(
            rc_display_source(&mut state.display_ir_cd, true),
            Some(DisplaySource::StrandCamMain)
        );
    }

    #[test]
    fn local_position_out_of_bounds_is_false_when_inside_limit() {
        let data = mavlink::ardupilotmega::LOCAL_POSITION_NED_DATA {
            x: 6_000.0,
            y: 7_000.0,
            z: 0.0,
            ..Default::default()
        };
        assert!(!is_local_position_out_of_bounds(&data));
    }

    #[test]
    fn local_position_out_of_bounds_is_true_at_limit() {
        let data = mavlink::ardupilotmega::LOCAL_POSITION_NED_DATA {
            x: 10_000.0,
            y: 0.0,
            z: 0.0,
            ..Default::default()
        };
        assert!(is_local_position_out_of_bounds(&data));
    }
}

#[cfg(test)]
mod tx_logging_tests {
    use super::*;

    fn a_setpoint() -> MavMessage {
        use mavlink::ardupilotmega::*;
        MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vx: 1.5,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.25,
            yaw_rate: 0.0,
            type_mask: PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE,
            target_system: AUTOPILOT_TARGET_SYSTEM,
            target_component: AUTOPILOT_TARGET_COMPONENT,
            coordinate_frame: MavFrame::MAV_FRAME_BODY_NED,
            time_boot_ms: 1234,
        })
    }

    /// The record an egress message produces is named by direction and carries
    /// the encoded message, so a reader can tell what actually went on the wire.
    #[test]
    fn tx_record_is_named_and_carries_encoded_fields() {
        let (logger, mut rx) = tokio::sync::mpsc::unbounded_channel();
        save_tx(&logger, &a_setpoint()).unwrap();

        let SaveToDiskMsg::MavlinkData(stamped) = rx.try_recv().unwrap() else {
            panic!("egress record should be MavlinkData");
        };
        assert_eq!(stamped.typ, "TX_SET_POSITION_TARGET_LOCAL_NED");
        // The payload is the whole MavMessage, internally tagged.
        assert_eq!(stamped.data["type"], "SET_POSITION_TARGET_LOCAL_NED");
        // Fields a reader would need to diagnose either of the historical bugs.
        assert_eq!(stamped.data["vx"], 1.5);
        // The two fields whose values were historically wrong are both in the
        // record, so a recording is enough to tell what frame was commanded and
        // which components were masked off. Both nest: mavlink tags its enums
        // and serializes bitflags as their raw bits.
        assert_eq!(
            stamped.data["coordinate_frame"]["type"],
            "MAV_FRAME_BODY_NED"
        );
        assert_eq!(stamped.data["type_mask"]["bits"], 448); // AX|AY|AZ_IGNORE
    }

    /// A receive-side record keeps its bare message name, so direction is
    /// distinguishable in `mavlink.jsonl`.
    #[test]
    fn rx_records_are_not_prefixed() {
        let (logger, mut rx) = tokio::sync::mpsc::unbounded_channel();
        save("LOCAL_POSITION_NED", &logger, &42u32).unwrap();
        let SaveToDiskMsg::MavlinkData(stamped) = rx.try_recv().unwrap() else {
            panic!("expected MavlinkData");
        };
        assert_eq!(stamped.typ, "LOCAL_POSITION_NED");
        assert!(!stamped.typ.starts_with("TX_"));
    }
}
