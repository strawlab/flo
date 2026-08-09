use crate::{Angle, DeviceMode, FloatType, MomentCentroid, MotorPositionResult, RadialDistance};

#[cfg(feature = "tokio")]
use crate::drone_structs::{DroneEvent, DroneRealtimeEvent, FlightMode, TrajectorySetpoint};

use serde::{Deserialize, Serialize};

#[cfg(feature = "tokio")]
use tokio::sync::broadcast;

/// Global messaging queues.
#[cfg(feature = "tokio")]
#[derive(Clone)]
pub struct Broadway {
    pub flo_events: broadcast::Sender<FloEvent>,
    pub flo_detections: broadcast::Sender<FloDetectionEvent>,
    /// Non-realtime events such as arm/disarm and battery level.
    pub drone_events: broadcast::Sender<DroneEvent>,
    /// Realtime events such as pose and RC channel values.
    pub drone_realtime: broadcast::Sender<DroneRealtimeEvent>,
    /// Realtime channel: any subsystem that wants to drive the autopilot
    /// publishes a [`TrajectorySetpoint`] here. `flo-mavlink` consumes it
    /// and translates each setpoint into a MAVLink message.
    pub flight_setpoint: broadcast::Sender<TrajectorySetpoint>,
    /// Event channel: any subsystem that wants the autopilot in a
    /// particular flight mode publishes the desired mode here.
    /// `flo-mavlink` consumes it and issues MAVLink commands.
    pub flight_mode_request: broadcast::Sender<FlightMode>,
}

#[cfg(feature = "tokio")]
impl Broadway {
    pub fn new(capacity: usize, capacity_rt: usize) -> Self {
        Self {
            flo_events: broadcast::channel(capacity).0,
            flo_detections: broadcast::channel(capacity_rt).0,
            drone_events: broadcast::channel(capacity).0,
            drone_realtime: broadcast::channel(capacity_rt).0,
            flight_setpoint: broadcast::channel(capacity_rt).0,
            flight_mode_request: broadcast::channel(capacity).0,
        }
    }
}

/// Non-realtime FLO events such as BUI commands, mode changes, and recording state.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum FloEvent {
    Command(FloCommand, CommandSource),

    /// Fires when the tracking mode changes. Payload is `(old_mode, new_mode, reason)`.
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
    /// Start a recording that also includes the buffered pre-capture window
    /// (the "post-trigger" button). Stopping uses `SetRecordingState(false)`.
    StartPreCaptureRecording,
    /// Set the pre-capture ("post-trigger") buffer window, in seconds.
    ///
    /// When greater than zero, the most recent data is held in RAM so that
    /// starting a recording also writes the preceding window to disk — letting
    /// the operator "go back in time" and capture an event that already
    /// happened. Zero disables buffering and discards anything buffered.
    SetPreCaptureSeconds(FloatType),
    SetDistCorr(FloatType),
    AdjustFocus(i32),
    /// Choose what the operator's live view shows. This affects the camshow
    /// display and its RTP stream only — the recording stays the clean webcam.
    SetDisplaySource(crate::DisplaySource),
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
    /// Detection angles, with periscope de-rotation applied.
    pub sensor_pan: Angle,
    pub sensor_tilt: Angle,
    /// Estimated motor position at the instant the centroid was captured.
    /// Missing for mini-pantilt.
    pub motor_estimate: Option<MotorPositionResult>,
    pub target_pan: Angle,  //in imu frame
    pub target_tilt: Angle, //in imu frame
    pub dist: Option<RadialDistance>,
}
