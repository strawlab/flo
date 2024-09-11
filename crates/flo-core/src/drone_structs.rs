use crate::utils::{elapsed, ChangeDetector, NoiseGate, NoiseGateParameters};
use crate::{FloatType, MyTimestamp};

use num_derive::FromPrimitive;
use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum DroneEvent {
    CommEstablished,
    CommLost,
    Armed,
    Disarmed,
    BatteryState(BatteryState),
    FlightModeChanged(Option<FlightMode>, FlightMode),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum DroneRealtimeEvent {
    RcChannels(DroneChannelData),
    //Attitude(DroneAttitude),
    //GLocation(DroneLocation),
}

pub const NUM_RC_CHANNELS: usize = 18;

///struct to send drone's remote-control channel values around in flo
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct DroneChannelData {
    ///values in range -1.0 .. 1.0
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

///struct to send other general drone status around in flo
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize, Default)]
pub struct DroneStatus {
    //FIXME: add timestamp
    pub armed: bool,
    pub flight_mode: FlightMode,
    ///battery voltage per cell
    pub batt_voltage: FloatType,
    ///battery percentage, as reported by flight controller
    pub batt_percent: FloatType,
    ///for now, this refers to battery status only; ar=med status is updated independently and is not timestamped
    pub timestamp: MyTimestamp,
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize, Default)]
pub struct BatteryState {
    pub timestamp: MyTimestamp,
    ///battery voltage per cell
    pub batt_voltage: FloatType,
    ///battery percentage, as reported by flight controller
    pub batt_percent: FloatType,
}

/// MAVLink configuration
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct MavlinkConfig {
    /// source of MAVLink data when running on drone
    /// (tcpout|tcpin|udpout|udpin|udpbcast|serial|file):(ip|dev|path):(port|baud)
    pub port_path: String,
    #[serde(default = "default_mavlink_system_id")]
    pub system_id: u8,
    #[serde(default = "default_mavlink_component_id")]
    pub component_id: u8,
    #[serde(default = "default_mavlink_loss_timeout")]
    pub loss_timeout: FloatType,
}

fn default_mavlink_system_id() -> u8 {
    1 // default autopilot
}

fn default_mavlink_component_id() -> u8 {
    191 // mavlink::ardupilotmega::MavComponent::MAV_COMP_ID_ONBOARD_COMPUTER
}

fn default_mavlink_loss_timeout() -> FloatType {
    1.0
}

impl Default for MavlinkConfig {
    fn default() -> Self {
        Self {
            port_path: "".to_string(),
            system_id: default_mavlink_system_id(),
            component_id: default_mavlink_component_id(),
            loss_timeout: default_mavlink_loss_timeout(),
        }
    }
}

/// Remote Control configuration
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct RcConfig {
    /// State to wait for as a command to start tracking.
    #[serde(default)]
    pub track_start: Option<ChannelCondition>,
    /// State to wait for as a command to stop tracking and return to home position.
    #[serde(default)]
    pub track_stop: Option<ChannelCondition>,
    /// State to wait for as a command to set current position as home
    #[serde(default)]
    pub set_home: Option<ChannelCondition>,

    #[serde(default)]
    pub pan_knob: Option<AngleKnobConfig>,

    #[serde(default)]
    pub tilt_knob: Option<AngleKnobConfig>,
}

///defines a check that a channel value is in a certain range
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ChannelCondition {
    ///number of the channel, from 1 to 18.
    pub ch_no: i32,
    ///channel values are typically value in range -1.0..1.0
    pub val_min: FloatType,
    pub val_max: FloatType,
}

impl ChannelCondition {
    pub fn test(condition: &Option<Self>, channels: &DroneChannelData) -> bool {
        if let Some(ref condition) = condition {
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
    ///number of the channel, from 1 to 18.
    pub ch_no: i32,
    pub noise_gate: NoiseGateParameters,
    //angle [degrees] from neutral, corresponding to extreme position of the knob
    pub max_angle: FloatType,
}

///state of rc change detection program
#[derive(Clone, Debug)]
pub struct RcProgramState {
    pub track_start_cd: ChangeDetector<bool>,
    pub track_stop_cd: ChangeDetector<bool>,
    pub set_home_cd: ChangeDetector<bool>,
    pub pan_ng: NoiseGate,
    pub tilt_ng: NoiseGate,
}

impl Default for RcProgramState {
    fn default() -> Self {
        Self {
            track_start_cd: ChangeDetector::new(),
            track_stop_cd: ChangeDetector::new(),
            set_home_cd: ChangeDetector::new(),
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
    ///returns (combined_flight_mude, main_mode, sub_mode), or None for Other
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
