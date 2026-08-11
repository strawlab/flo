use crate::{
    FloatType, MyTimestamp, is_default,
    utils::{ChangeDetector, NoiseGate, NoiseGateParameters, elapsed},
};

use num_derive::FromPrimitive;
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum DroneEvent {
    Armed,
    Disarmed,
    BatteryState(BatteryState),
    FlightModeChanged(Option<FlightMode>, FlightMode),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub enum GnssRtkMode {
    #[default]
    NoGps,
    NoFix,
    TwoDFix,
    ThreeDFix,
    DGps,
    RtkFloat,
    RtkFixed,
    Static,
    Ppp,
}

impl GnssRtkMode {
    /// A label for the BUI. The OSD has only a few characters to spend and
    /// abbreviates separately.
    pub fn label(&self) -> &'static str {
        match self {
            Self::NoGps => "no GPS",
            Self::NoFix => "no fix",
            Self::TwoDFix => "2D fix",
            Self::ThreeDFix => "3D fix",
            Self::DGps => "DGPS",
            Self::RtkFloat => "RTK float",
            Self::RtkFixed => "RTK fixed",
            Self::Static => "static",
            Self::Ppp => "PPP",
        }
    }
}

/// A flight controller's local-position origin, in MAVLink's own units.
///
/// Both `SET_GPS_GLOBAL_ORIGIN` and `GPS_GLOBAL_ORIGIN` carry these integers,
/// so keeping them lets a request be compared against what the flight
/// controller reports without a round trip through floating point.
#[derive(Debug, PartialEq, Eq, Serialize, Deserialize, Clone, Copy)]
pub struct GpsGlobalOrigin {
    /// Latitude in units of 1e-7 degrees (WGS84).
    pub latitude_e7: i32,
    /// Longitude in units of 1e-7 degrees (WGS84).
    pub longitude_e7: i32,
    /// Altitude above mean sea level, in millimeters.
    pub altitude_mm: i32,
}

/// One unit of latitude is about 1.11 cm, so this is about a meter.
const ORIGIN_TOLERANCE_E7: i32 = 90;
/// A meter, in the units above. Altitude may legitimately be adjusted a little
/// by the flight controller's own geoid model.
const ORIGIN_TOLERANCE_MM: i32 = 1_000;

impl GpsGlobalOrigin {
    /// Build from the degrees/degrees/meters triple a config file carries.
    pub fn from_degrees_and_meters(
        lat_deg: FloatType,
        lon_deg: FloatType,
        alt_m: FloatType,
    ) -> Self {
        Self {
            latitude_e7: (lat_deg * 1e7) as i32,
            longitude_e7: (lon_deg * 1e7) as i32,
            altitude_mm: (alt_m * 1e3) as i32,
        }
    }

    pub fn latitude_deg(&self) -> FloatType {
        self.latitude_e7 as FloatType / 1e7
    }

    pub fn longitude_deg(&self) -> FloatType {
        self.longitude_e7 as FloatType / 1e7
    }

    pub fn altitude_m(&self) -> FloatType {
        self.altitude_mm as FloatType / 1e3
    }

    /// Whether `self` is the same place as `other`, within about a meter.
    ///
    /// An origin that did not take hold is wrong by kilometers, not
    /// centimeters, so this is loose enough that rounding in the flight
    /// controller never raises a false alarm.
    pub fn matches(&self, other: &Self) -> bool {
        (self.latitude_e7 - other.latitude_e7).abs() <= ORIGIN_TOLERANCE_E7
            && (self.longitude_e7 - other.longitude_e7).abs() <= ORIGIN_TOLERANCE_E7
            && (self.altitude_mm - other.altitude_mm).abs() <= ORIGIN_TOLERANCE_MM
    }
}

impl std::fmt::Display for GpsGlobalOrigin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:.7}, {:.7}, {:.3} m",
            self.latitude_deg(),
            self.longitude_deg(),
            self.altitude_m()
        )
    }
}

/// Whether the local-position origin FLO asked the flight controller to use is
/// the one the flight controller is actually using.
#[derive(Debug, PartialEq, Eq, Serialize, Deserialize, Clone, Copy, Default)]
pub enum GpsOriginCheck {
    /// The config has no `set_gps_global_origin`, so FLO never asked.
    #[default]
    NotRequested,
    /// FLO asked, but the flight controller has not reported an origin yet.
    Awaiting,
    /// The flight controller reports the origin FLO asked for.
    Confirmed,
    /// The flight controller reports a different origin. Local positions —
    /// and therefore everything FLO computes from them — refer to the wrong
    /// place.
    Mismatched,
}

/// The local-position origin as requested and as reported, plus the verdict.
#[derive(Debug, PartialEq, Eq, Serialize, Deserialize, Clone, Copy, Default)]
pub struct GpsOriginStatus {
    /// What FLO asked for, from `mavlink.set_gps_global_origin`.
    pub requested: Option<GpsGlobalOrigin>,
    /// The most recent `GPS_GLOBAL_ORIGIN` from the flight controller.
    pub reported: Option<GpsGlobalOrigin>,
    pub check: GpsOriginCheck,
}

impl GpsOriginStatus {
    /// Fold in a freshly received `GPS_GLOBAL_ORIGIN` and return the verdict.
    pub fn on_reported(&mut self, reported: GpsGlobalOrigin) -> GpsOriginCheck {
        self.reported = Some(reported);
        self.check = match &self.requested {
            None => GpsOriginCheck::NotRequested,
            Some(requested) if requested.matches(&reported) => GpsOriginCheck::Confirmed,
            Some(_) => GpsOriginCheck::Mismatched,
        };
        self.check
    }
}

/// The drone's position in the flight controller's local NED frame, in meters,
/// relative to the origin in [`GpsOriginStatus::reported`].
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Copy, Default)]
pub struct LocalPositionNed {
    pub north_m: FloatType,
    pub east_m: FloatType,
    /// Positive *downward*, as MAVLink defines it: a drone above its origin
    /// reports a negative value here.
    pub down_m: FloatType,
}

/// Below this horizontal offset no direction is reported. The compass point
/// would be estimator noise rather than a bearing, and would spin while the
/// drone sat still over its origin.
const MIN_DIRECTIONAL_DISTANCE_M: FloatType = 0.5;

/// The eight compass points, in bearing order from north, clockwise.
const COMPASS_POINTS: [&str; 8] = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];

impl LocalPositionNed {
    /// Distance from the origin in the horizontal plane, ignoring height.
    pub fn horizontal_distance_m(&self) -> FloatType {
        self.north_m.hypot(self.east_m)
    }

    /// Which way the drone lies from the origin, to eight compass points, or
    /// `None` when it is too close to the origin for a direction to mean
    /// anything.
    pub fn cardinal_8(&self) -> Option<&'static str> {
        if self.horizontal_distance_m() < MIN_DIRECTIONAL_DISTANCE_M {
            return None;
        }
        // `atan2(east, north)` is the compass convention — zero at north,
        // increasing clockwise — which swaps the arguments of the mathematical
        // one.
        let bearing_deg = self.east_m.atan2(self.north_m).to_degrees();
        let sector = (bearing_deg / 45.0).round() as i32;
        Some(COMPASS_POINTS[sector.rem_euclid(8) as usize])
    }

    /// The horizontal offset the way an operator says it out loud, such as
    /// `SW 200 m`. Directly over the origin there is no direction to give, so
    /// only the distance is returned.
    pub fn horizontal_offset_str(&self) -> String {
        let distance = self.horizontal_distance_m();
        match self.cardinal_8() {
            Some(direction) => format!("{direction} {distance:.0} m"),
            None => format!("{distance:.1} m"),
        }
    }
}

/// The drone's attitude, in radians.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Copy, Default)]
pub struct Attitude {
    pub roll_rad: FloatType,
    pub pitch_rad: FloatType,
    /// Rotation about the down axis, in MAVLink's -pi..pi.
    pub yaw_rad: FloatType,
}

impl Attitude {
    /// Recover roll, pitch and yaw from an `ATTITUDE_QUATERNION`.
    ///
    /// The arguments are that message's `q1..q4`, which is (w, x, y, z) of a
    /// unit quaternion taking the NED earth frame to the body frame. This is
    /// the standard yaw-pitch-roll extraction from it.
    pub fn from_quaternion(w: FloatType, x: FloatType, y: FloatType, z: FloatType) -> Self {
        Self {
            roll_rad: (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y)),
            // Clamped because a quaternion that is a hair off unit length —
            // which is what arrives over the wire — can push this just past 1
            // and make `asin` return NaN at straight up or straight down.
            pitch_rad: (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin(),
            yaw_rad: (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)),
        }
    }

    pub fn roll_deg(&self) -> FloatType {
        self.roll_rad.to_degrees()
    }

    pub fn pitch_deg(&self) -> FloatType {
        self.pitch_rad.to_degrees()
    }

    /// Yaw as a compass heading, 0..360 degrees, rather than MAVLink's signed
    /// range: that is how a heading is read off any other instrument.
    pub fn yaw_heading_deg(&self) -> FloatType {
        self.yaw_rad.to_degrees().rem_euclid(360.0)
    }
}

/// What the flight controller has told FLO, mirrored into
/// [`crate::DeviceState`] for the BUI.
///
/// The fields are separately optional because their messages arrive
/// independently, and a flight controller that streams one may not stream
/// another.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct MavlinkState {
    /// The local-position origin FLO asked for and the one the flight
    /// controller reports. Everything below is relative to the latter.
    pub gps_origin: GpsOriginStatus,
    pub local_position: Option<LocalPositionNed>,
    pub attitude: Option<Attitude>,
    /// `None` until the first `GPS_RAW_INT`, which is not the same as
    /// [`GnssRtkMode::NoGps`]: that is the flight controller saying it has no
    /// GPS, rather than FLO not yet knowing.
    pub gnss_rtk_mode: Option<GnssRtkMode>,
    /// `HEARTBEAT.custom_mode`, kept raw. See [`flight_mode_label`].
    pub custom_mode: Option<u32>,
}

/// A label for a `HEARTBEAT.custom_mode`.
///
/// [`FlightMode`] enumerates PX4's values. A flight controller that numbers its
/// modes differently — ArduPilot's are unrelated to PX4's, and ardupilotmega is
/// the dialect FLO speaks — gets the raw value shown rather than a wrong name
/// or a bare "other".
pub fn flight_mode_label(custom_mode: u32) -> String {
    match FlightMode::from(custom_mode) {
        FlightMode::Manual => "manual".to_string(),
        FlightMode::Altitude => "altitude".to_string(),
        FlightMode::Position => "position".to_string(),
        FlightMode::Hold => "hold".to_string(),
        FlightMode::Return => "return".to_string(),
        FlightMode::Offboard => "offboard".to_string(),
        FlightMode::Takeoff => "takeoff".to_string(),
        FlightMode::Land => "land".to_string(),
        FlightMode::Other => format!("unrecognized (0x{custom_mode:08x})"),
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum DroneRealtimeEvent {
    RcChannels(DroneChannelData),
    //Attitude(DroneAttitude),
    //GLocation(DroneLocation),
}

pub const NUM_RC_CHANNELS: usize = 18;

/// Drone RC channel values.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct DroneChannelData {
    /// Values in range -1.0..1.0.
    pub values: [FloatType; NUM_RC_CHANNELS],
    pub timestamp: MyTimestamp,
}
impl DroneChannelData {
    pub fn get(&self, chno: i32) -> FloatType {
        if chno == 0 {
            0.0
        } else {
            self.values[(chno - 1) as usize]
        }
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize, Default)]
pub struct BatteryState {
    pub timestamp: MyTimestamp,
    /// Battery voltage per cell.
    pub batt_voltage: FloatType,
    /// Battery percentage as reported by the flight controller.
    pub batt_percent: FloatType,
}

/// Which GNSS receiver traffic the flight controller records in its own flight
/// log: the value FLO writes to PX4's `GPS_DUMP_COMM` parameter.
///
/// PX4 logs no raw GNSS data by default, so a flight log holds only the
/// real-time fix. Turning this on puts the observations a post-processed
/// kinematic (PPK) solution is computed from into the `.ulg`, where they are
/// recovered with `pyulog`'s `ulog_extract_gps_dump` and converted to RINEX with
/// RTKLIB's `convbin`.
#[derive(Debug, PartialEq, Eq, Serialize, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum GpsDumpComm {
    /// `GPS_DUMP_COMM=0`: log nothing. PX4's default.
    #[default]
    Disabled,
    /// `GPS_DUMP_COMM=1`: log every byte in both directions, which includes the
    /// RTCM corrections FLO and the ground station inject.
    ///
    /// This does not by itself ask the receiver for raw observations, so it is
    /// not enough for PPK on its own: it captures what the base sent, not what
    /// the rover measured. PX4 ≥ 1.18 can add the rover's observations to this
    /// mode through a second parameter, `GPS_UBX_PPK`, which FLO does not set.
    FullCommunication,
    /// `GPS_DUMP_COMM=2`: configure the main receiver to emit RTCM3 MSM7
    /// observations at 1 Hz, and log those.
    ///
    /// This is the PPK mode. The log then holds the rover's own carrier-phase
    /// observations, and it holds them regardless of where the corrections came
    /// from — NTRIP through FLO, or a local base station relayed by the ground
    /// station — because what is recorded is what the receiver measured, not
    /// what it was told. Only the receive direction is logged, so base station
    /// observations are *not* in the flight log and have to be archived at
    /// whatever produced them.
    RtcmOutput,
}

impl GpsDumpComm {
    /// The value PX4 expects in `GPS_DUMP_COMM`.
    pub fn px4_value(self) -> i32 {
        match self {
            Self::Disabled => 0,
            Self::FullCommunication => 1,
            Self::RtcmOutput => 2,
        }
    }
}

/// What FLO should make of the flight controller's raw-GNSS logging at startup.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct PpkLoggingConfig {
    /// What the flight controller should log.
    pub gps_dump_comm: GpsDumpComm,

    /// Whether FLO may reboot the flight controller to make a changed value take
    /// effect.
    ///
    /// PX4 reads `GPS_DUMP_COMM` once, when its GPS driver starts, so storing a
    /// new value changes nothing about the flight in progress. With this on, FLO
    /// reboots — but only when the value actually had to change, and only while
    /// disarmed, so every startup after the first is a read and nothing more.
    /// With it off, a newly configured value first takes effect on whatever boot
    /// comes next.
    ///
    /// Turn this off if FLO reaches the flight controller over USB: the reboot
    /// takes the USB serial device away, and FLO cannot reopen it.
    #[serde(
        default = "default_reboot_to_apply",
        skip_serializing_if = "is_default_reboot_to_apply"
    )]
    pub reboot_to_apply: bool,
}

/// MAVLink configuration
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct MavlinkConfig {
    /// source of MAVLink data when running on drone
    /// (tcpout|tcpin|udpout|udpin|udpbcast|serial|file):(ip|dev|path):(port|baud)
    pub port_path: String,

    #[serde(
        default = "default_mavlink_system_id",
        skip_serializing_if = "is_default_mavlink_system_id"
    )]
    pub system_id: u8,

    #[serde(
        default = "default_mavlink_component_id",
        skip_serializing_if = "is_default_mavlink_component_id"
    )]
    pub component_id: u8,

    /// Number of cells in series in the drone battery.
    #[serde(
        default = "default_mavlink_batt_s",
        skip_serializing_if = "is_default_mavlink_batt_s"
    )]
    pub batt_s: u8,

    #[serde(default, skip_serializing_if = "is_default")]
    pub set_gps_global_origin: Option<[FloatType; 3]>,

    #[serde(default, skip_serializing_if = "is_default")]
    pub ntrip_url: Option<String>,

    /// Raw-GNSS logging on the flight controller, for post-processed kinematics.
    /// Left alone entirely when absent.
    #[serde(default, skip_serializing_if = "is_default")]
    pub ppk_logging: Option<PpkLoggingConfig>,
}

impl MavlinkConfig {
    /// The local-position origin this config asks the flight controller to
    /// use, if any.
    pub fn requested_gps_global_origin(&self) -> Option<GpsGlobalOrigin> {
        self.set_gps_global_origin
            .map(|[lat, lon, alt]| GpsGlobalOrigin::from_degrees_and_meters(lat, lon, alt))
    }
}

fn default_mavlink_system_id() -> u8 {
    1 // default autopilot
}

fn is_default_mavlink_system_id(val: &u8) -> bool {
    *val == default_mavlink_system_id()
}

fn default_mavlink_component_id() -> u8 {
    191 // mavlink::ardupilotmega::MavComponent::MAV_COMP_ID_ONBOARD_COMPUTER
}

fn is_default_mavlink_component_id(val: &u8) -> bool {
    *val == default_mavlink_component_id()
}

fn default_mavlink_batt_s() -> u8 {
    1
}

fn is_default_mavlink_batt_s(val: &u8) -> bool {
    *val == default_mavlink_batt_s()
}

fn default_reboot_to_apply() -> bool {
    // Without a reboot, configuring raw-GNSS logging does nothing until the
    // flight controller is next power-cycled, which is rarely what someone
    // switching it on meant.
    true
}

fn is_default_reboot_to_apply(val: &bool) -> bool {
    *val == default_reboot_to_apply()
}

impl Default for MavlinkConfig {
    fn default() -> Self {
        Self {
            port_path: "".to_string(),
            system_id: default_mavlink_system_id(),
            component_id: default_mavlink_component_id(),
            batt_s: default_mavlink_batt_s(),
            set_gps_global_origin: Default::default(),
            ntrip_url: Default::default(),
            ppk_logging: Default::default(),
        }
    }
}

/// Remote Control configuration
#[derive(Debug, PartialEq, Serialize, Deserialize, Default, Clone)]
#[serde(deny_unknown_fields)]
pub struct RcConfig {
    #[serde(default, skip_serializing_if = "is_default")]
    pub us_mapping: ChannelRange,

    /// State to wait for as a command to start tracking.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub track_start: Option<ChannelCondition>,

    /// State to wait for as a command to stop tracking and return to home position.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub track_stop: Option<ChannelCondition>,

    /// State to wait for as a command to set current position as home
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub set_home: Option<ChannelCondition>,

    /// State in which the operator's live view shows the main tracking camera
    /// rather than the FPV webcam. Unlike the conditions above this is a
    /// *level*, not a trigger: leaving it switches the view back.
    ///
    /// The secondary camera is deliberately not reachable from RC. It is used
    /// rarely enough that a config or BUI path is the better place for it, and
    /// one boolean keeps this to a single switch in the air.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub display_ir: Option<ChannelCondition>,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pan_knob: Option<AngleKnobConfig>,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tilt_knob: Option<AngleKnobConfig>,
}

/// Converts a PWM pulse duration (microseconds) to a value in -1..+1.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ChannelRange {
    pub neutral_value_us: FloatType,
    pub span_us: FloatType,
}

impl Default for ChannelRange {
    fn default() -> Self {
        Self {
            neutral_value_us: 1500.0,
            span_us: 500.0,
        }
    }
}

impl ChannelRange {
    pub fn convert(&self, val_us: FloatType) -> FloatType {
        (val_us - self.neutral_value_us) / self.span_us
    }
}

/// Checks whether an RC channel value is within a specified range.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ChannelCondition {
    /// Channel number, from 1 to 18.
    pub ch_no: i32,
    /// Channel value bounds; values are typically in -1.0..1.0.
    pub val_min: FloatType,
    pub val_max: FloatType,
}

impl ChannelCondition {
    pub fn test(condition: &Option<Self>, channels: &DroneChannelData) -> bool {
        if let Some(condition) = condition {
            (elapsed(channels.timestamp) < 1.5)
                && channels.get(condition.ch_no) >= condition.val_min
                && channels.get(condition.ch_no) <= condition.val_max
        } else {
            false
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct AngleKnobConfig {
    /// Channel number, from 1 to 18.
    pub ch_no: i32,
    pub noise_gate: NoiseGateParameters,
    //angle [degrees] from neutral, corresponding to extreme position of the knob
    pub max_angle: FloatType,
}

/// State of the RC change-detection program.
#[derive(Clone, Debug)]
pub struct RcProgramState {
    pub track_start_cd: ChangeDetector<bool>,
    pub track_stop_cd: ChangeDetector<bool>,
    pub set_home_cd: ChangeDetector<bool>,
    pub display_ir_cd: ChangeDetector<bool>,
    pub pan_ng: NoiseGate,
    pub tilt_ng: NoiseGate,
}

impl Default for RcProgramState {
    fn default() -> Self {
        Self {
            track_start_cd: Default::default(),
            track_stop_cd: Default::default(),
            set_home_cd: Default::default(),
            // Seeded with `false`, unlike the trigger conditions above, because
            // this one mirrors a state FLO already has: the display source
            // starts as the webcam. Seeding it means the first RC frame after a
            // FLO restart re-asserts an IR view the operator is still asking
            // for, instead of waiting for them to toggle the switch.
            display_ir_cd: ChangeDetector::new_with_initial_state(&false),
            pan_ng: NoiseGate::new(NoiseGateParameters {
                noise_gate: 0.02,
                hold_time: 2.0,
            }),
            tilt_ng: NoiseGate::new(NoiseGateParameters {
                noise_gate: 0.02,
                hold_time: 2.0,
            }),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Serialize, Deserialize, Clone, Copy, Default, FromPrimitive)]
#[rustfmt::skip]
pub enum FlightMode {
    Manual   = 0x0001_0000,
    Altitude = 0x0002_0000,
    Position = 0x0003_0000,
    Hold     = 0x0304_0000,
    Return   = 0x0504_0000,
    Offboard = 0x0006_0000,
    Takeoff  = 0x0204_0000,
    Land     = 0x0604_0000,
    #[default]
    Other,
}

impl From<u32> for FlightMode {
    fn from(fm: u32) -> Self {
        if let Some(fm_e) = num::FromPrimitive::from_u32(fm) {
            fm_e
        } else {
            Self::Other
        }
    }
}

impl FlightMode {
    /// Returns `(combined_flight_mode, main_mode, sub_mode)`, or `None` for [`FlightMode::Other`].
    ///
    /// This is the request-side split: `MAV_CMD_DO_SET_MODE` wants the main and
    /// sub mode as separate parameters, where the variants above pack both into
    /// one value. Nothing in this workspace commands a mode change, so this has
    /// no caller here; it is kept rather than deleted because unpacking PX4's
    /// layout is the kind of thing that goes subtly wrong when it is written out
    /// a second time somewhere else.
    pub fn mode_numbers(&self) -> Option<(u32, f32, f32)> {
        if *self == FlightMode::Other {
            None
        } else {
            let fm: u32 = *self as u32;
            let main_mode = (fm >> 16) & 0xFF;
            let sub_mode = (fm >> 24) & 0xFF;
            Some((fm, main_mode as f32, sub_mode as f32))
        }
    }
}

#[cfg(test)]
mod mavlink_state_tests {
    use super::{Attitude, LocalPositionNed, flight_mode_label};

    fn at(north_m: f64, east_m: f64) -> LocalPositionNed {
        LocalPositionNed {
            north_m,
            east_m,
            down_m: 0.0,
        }
    }

    #[test]
    fn horizontal_distance_ignores_height() {
        let position = LocalPositionNed {
            north_m: 3.0,
            east_m: 4.0,
            down_m: -100.0,
        };
        assert!((position.horizontal_distance_m() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn each_axis_and_diagonal_gets_its_compass_point() {
        assert_eq!(at(10.0, 0.0).cardinal_8(), Some("N"));
        assert_eq!(at(10.0, 10.0).cardinal_8(), Some("NE"));
        assert_eq!(at(0.0, 10.0).cardinal_8(), Some("E"));
        assert_eq!(at(-10.0, 10.0).cardinal_8(), Some("SE"));
        assert_eq!(at(-10.0, 0.0).cardinal_8(), Some("S"));
        assert_eq!(at(-10.0, -10.0).cardinal_8(), Some("SW"));
        assert_eq!(at(0.0, -10.0).cardinal_8(), Some("W"));
        assert_eq!(at(10.0, -10.0).cardinal_8(), Some("NW"));
    }

    /// Each point covers 45 degrees, so the switch is at 22.5.
    #[test]
    fn a_bearing_takes_the_nearest_of_the_eight_points() {
        let bearing = |deg: f64| {
            let (sin, cos) = deg.to_radians().sin_cos();
            at(100.0 * cos, 100.0 * sin).cardinal_8()
        };
        assert_eq!(bearing(22.4), Some("N"));
        assert_eq!(bearing(22.6), Some("NE"));
        // Wrapping the far side of north, where the sector index goes negative.
        assert_eq!(bearing(-22.4), Some("N"));
        assert_eq!(bearing(-22.6), Some("NW"));
        assert_eq!(bearing(-170.0), Some("S"));
    }

    /// Over the origin the bearing is estimator noise, so no direction is given.
    #[test]
    fn no_direction_is_reported_from_on_top_of_the_origin() {
        assert_eq!(at(0.0, 0.0).cardinal_8(), None);
        assert_eq!(at(0.2, -0.2).cardinal_8(), None);
        assert_eq!(at(0.0, 0.0).horizontal_offset_str(), "0.0 m");
    }

    #[test]
    fn the_horizontal_offset_reads_as_a_direction_and_a_distance() {
        let position = at(-141.421356, -141.421356);
        assert_eq!(position.horizontal_offset_str(), "SW 200 m");
    }

    #[test]
    fn a_level_northward_quaternion_is_all_zeros() {
        let attitude = Attitude::from_quaternion(1.0, 0.0, 0.0, 0.0);
        assert!(attitude.roll_deg().abs() < 1e-9);
        assert!(attitude.pitch_deg().abs() < 1e-9);
        assert!(attitude.yaw_heading_deg().abs() < 1e-9);
    }

    #[test]
    fn a_quaternion_round_trips_to_roll_pitch_and_yaw() {
        // A quarter turn about the down axis is a heading of due east.
        let half = std::f64::consts::FRAC_PI_4;
        let yawed = Attitude::from_quaternion(half.cos(), 0.0, 0.0, half.sin());
        assert!((yawed.yaw_heading_deg() - 90.0).abs() < 1e-6);

        // 30 degrees of roll, about the forward axis.
        let half = 15f64.to_radians();
        let rolled = Attitude::from_quaternion(half.cos(), half.sin(), 0.0, 0.0);
        assert!((rolled.roll_deg() - 30.0).abs() < 1e-6);
        assert!(rolled.pitch_deg().abs() < 1e-6);
    }

    /// A quaternion arrives as four floats and need not be exactly unit length.
    /// Straight up is where that pushes `asin` out of range.
    #[test]
    fn a_slightly_long_quaternion_pointing_up_is_not_nan() {
        let attitude = Attitude::from_quaternion(0.7072, 0.0, 0.7072, 0.0);
        assert!((attitude.pitch_deg() - 90.0).abs() < 1e-6);
    }

    #[test]
    fn yaw_is_reported_as_a_heading_rather_than_a_signed_angle() {
        let attitude = Attitude {
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: -std::f64::consts::FRAC_PI_2,
        };
        assert!((attitude.yaw_heading_deg() - 270.0).abs() < 1e-9);
    }

    #[test]
    fn a_known_flight_mode_gets_its_name() {
        assert_eq!(flight_mode_label(0x0003_0000), "position");
        assert_eq!(flight_mode_label(0x0001_0000), "manual");
    }

    /// A controller numbering its modes differently — every ArduPilot one —
    /// should show what it actually reported.
    #[test]
    fn an_unrecognized_flight_mode_shows_its_raw_value() {
        assert_eq!(flight_mode_label(0x1234), "unrecognized (0x00001234)");
    }
}

#[cfg(test)]
mod ppk_logging_config_tests {
    use super::{GpsDumpComm, MavlinkConfig, PpkLoggingConfig};

    /// A config that says nothing about PPK leaves the flight controller's
    /// logging alone.
    #[test]
    fn ppk_logging_is_absent_by_default() {
        let cfg: MavlinkConfig = serde_yaml::from_str("port_path: serial:/dev/ttyUSB0:57600")
            .expect("minimal MAVLink config should parse");
        assert_eq!(cfg.ppk_logging, None);
    }

    /// Asking for PPK logging should not also require deciding about reboots:
    /// storing a value that never takes effect is not what the ask means.
    #[test]
    fn rebooting_to_apply_is_the_default() {
        let cfg: MavlinkConfig = serde_yaml::from_str(
            "port_path: serial:/dev/ttyUSB0:57600\n\
             ppk_logging:\n  gps_dump_comm: rtcm_output\n",
        )
        .expect("PPK config should parse");
        assert_eq!(
            cfg.ppk_logging,
            Some(PpkLoggingConfig {
                gps_dump_comm: GpsDumpComm::RtcmOutput,
                reboot_to_apply: true,
            })
        );
    }

    #[test]
    fn rebooting_to_apply_can_be_declined() {
        let cfg: MavlinkConfig = serde_yaml::from_str(
            "port_path: serial:/dev/ttyUSB0:57600\n\
             ppk_logging:\n  gps_dump_comm: full_communication\n  reboot_to_apply: false\n",
        )
        .expect("PPK config should parse");
        let ppk = cfg.ppk_logging.expect("configured");
        assert_eq!(ppk.gps_dump_comm, GpsDumpComm::FullCommunication);
        assert!(!ppk.reboot_to_apply);
    }

    /// The numbers are PX4's, not ours: they are what lands in `GPS_DUMP_COMM`.
    #[test]
    fn modes_map_to_the_px4_parameter_values() {
        assert_eq!(GpsDumpComm::Disabled.px4_value(), 0);
        assert_eq!(GpsDumpComm::FullCommunication.px4_value(), 1);
        assert_eq!(GpsDumpComm::RtcmOutput.px4_value(), 2);
    }
}

#[cfg(test)]
mod gps_origin_tests {
    use super::{GpsGlobalOrigin, GpsOriginCheck, GpsOriginStatus};

    fn freiburg() -> GpsGlobalOrigin {
        GpsGlobalOrigin::from_degrees_and_meters(48.0038, 7.8449, 278.0)
    }

    #[test]
    fn a_config_triple_round_trips_through_mavlink_units() {
        let origin = freiburg();
        assert_eq!(origin.latitude_e7, 480_038_000);
        assert_eq!(origin.longitude_e7, 78_449_000);
        assert_eq!(origin.altitude_mm, 278_000);
        assert!((origin.latitude_deg() - 48.0038).abs() < 1e-9);
        assert!((origin.altitude_m() - 278.0).abs() < 1e-9);
    }

    /// The flight controller may round or apply its own geoid model, so a
    /// sub-meter difference is the same place.
    #[test]
    fn sub_meter_differences_still_match() {
        let requested = freiburg();
        let reported = GpsGlobalOrigin {
            latitude_e7: requested.latitude_e7 + 50,
            longitude_e7: requested.longitude_e7 - 50,
            altitude_mm: requested.altitude_mm + 400,
        };
        assert!(requested.matches(&reported));
    }

    #[test]
    fn an_origin_that_did_not_take_hold_does_not_match() {
        let requested = freiburg();
        // A whole degree away: what it looks like when the request is ignored
        // and the flight controller uses its own first fix.
        let reported = GpsGlobalOrigin {
            latitude_e7: requested.latitude_e7 + 10_000_000,
            ..requested
        };
        assert!(!requested.matches(&reported));
        // As does an altitude that is off by more than a meter.
        let reported = GpsGlobalOrigin {
            altitude_mm: requested.altitude_mm + 1_001,
            ..requested
        };
        assert!(!requested.matches(&reported));
    }

    #[test]
    fn the_verdict_follows_what_the_flight_controller_reports() {
        let mut status = GpsOriginStatus {
            requested: Some(freiburg()),
            reported: None,
            check: GpsOriginCheck::Awaiting,
        };
        assert_eq!(status.on_reported(freiburg()), GpsOriginCheck::Confirmed);

        let elsewhere = GpsGlobalOrigin::from_degrees_and_meters(0.0, 0.0, 0.0);
        assert_eq!(status.on_reported(elsewhere), GpsOriginCheck::Mismatched);
        assert_eq!(status.reported, Some(elsewhere));
    }

    /// Without a request there is nothing to check, so whatever the flight
    /// controller reports is simply recorded.
    #[test]
    fn nothing_is_checked_when_nothing_was_requested() {
        let mut status = GpsOriginStatus::default();
        assert_eq!(status.on_reported(freiburg()), GpsOriginCheck::NotRequested);
        assert_eq!(status.reported, Some(freiburg()));
    }
}
