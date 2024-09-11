use crate::{
    Angle, DeviceMode, FloatType, MomentCentroid, MotorPositionResult, MotorValueCache,
    RadialDistance,
};

#[allow(unused_imports)]
use crate::drone_structs::{DroneEvent, DroneRealtimeEvent};

use serde::{Deserialize, Serialize};

#[cfg(feature = "tokio")]
use tokio::sync::broadcast;

///global messaging queues.
#[cfg(feature = "tokio")]
#[derive(Clone)]
pub struct Broadway {
    pub flo_events: broadcast::Sender<FloEvent>,
    pub flo_detections: broadcast::Sender<FloDetectionEvent>,
    pub flo_motion: broadcast::Sender<FloMotionEvent>,
    ///stuff like arm/disarm, battery level
    pub drone_events: broadcast::Sender<DroneEvent>,
    ///stuff like pose and rc channels
    pub drone_realtime: broadcast::Sender<DroneRealtimeEvent>,
}

#[cfg(feature = "tokio")]
impl Broadway {
    pub fn new(capacity: usize, capacity_rt: usize) -> Self {
        Self {
            flo_events: broadcast::channel(capacity).0,
            flo_detections: broadcast::channel(capacity_rt).0,
            flo_motion: broadcast::channel(capacity_rt).0,
            drone_events: broadcast::channel(capacity).0,
            drone_realtime: broadcast::channel(capacity_rt).0,
        }
    }
}

///non-realtime flo events like bui command, mode change, file recording
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum FloEvent {
    Command(FloCommand, CommandSource),

    ///fires when the tracking mode is actually changed. Data: (old_mode, new_mode, reason).
    ModeChanged((DeviceMode, DeviceMode, ModeChangeReason)),
}

/// This is a message type which is sent to the FLO controller to change its
/// settings or operating mode.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum FloCommand {
    SwitchMode(DeviceMode, ModeChangeReason),
    /// changes the device mode to ManualOpenLoop
    SwitchToOpenLoop,
    /// does not change the device mode
    SetHomePosition((Option<Angle>, Option<Angle>, Option<RadialDistance>)),
    SetHomePositionFromCurrent,
    SetRecordingState(bool),
    SetDistCorr(FloatType),
    AdjustFocus(i32),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum CommandSource {
    Automation, //mode changes due to detected/lost
    Bui,
    DroneRC,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum ModeChangeReason {
    Operator,
    TargetAcqiured,
    TargetLost,
    Timeout, //suspendedclosedloop -> manualopenloop
    SetNewHome,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum FloDetectionEvent {
    Centroid(CentroidEvent),
    StereoCentroid(MomentCentroid, MomentCentroid),
    Observation(Observation),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct CentroidEvent {
    pub centroid: MomentCentroid,
    pub is_primary: bool, //true = main cam, false = secondary cam
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct Observation {
    pub timestamp: chrono::DateTime<chrono::Local>,
    ///detection angles, with periscope unrotation applied
    pub sensor_pan: Angle,
    pub sensor_tilt: Angle,
    ///estimate of the motor position at the instant the centroid was captured.
    /// (Missing for mini-pantilt)
    pub motor_estimate: Option<MotorPositionResult>,
    pub target_pan: Angle,  //in imu frame
    pub target_tilt: Angle, //in imu frame
    pub dist: Option<RadialDistance>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum FloMotionEvent {
    ///output of the kalman filter
    TargetEstimate(TargetEstimate),
    MotorCommand(MotorValueCache),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct TargetEstimate {
    pub timestamp: chrono::DateTime<chrono::Local>,
    pub target_pan: FloatType,  //in imu frame
    pub target_tilt: FloatType, //in imu frame
    pub target_vpan: FloatType,
    pub target_vtilt: FloatType,
    pub dist: Option<RadialDistance>,
    pub vdist: Option<FloatType>,
}
