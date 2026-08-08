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

impl Default for MavlinkConfig {
    fn default() -> Self {
        Self {
            port_path: "".to_string(),
            system_id: default_mavlink_system_id(),
            component_id: default_mavlink_component_id(),
            batt_s: default_mavlink_batt_s(),
            set_gps_global_origin: Default::default(),
            ntrip_url: Default::default(),
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

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct TrajectorySetpoint {
    pub pos: [Option<FloatType>; 3],
    /// movement speed in body frame (x = forward)
    pub vel: [Option<FloatType>; 3],
    pub yaw: Option<FloatType>,
    pub vyaw: Option<FloatType>,
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
