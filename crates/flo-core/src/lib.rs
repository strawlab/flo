use nalgebra as na;
use serde::{Deserialize, Deserializer, Serialize};

pub mod math;
pub use math::*;

pub mod events;
pub use events::*;

pub mod drone_structs;
pub use drone_structs::{DroneChannelData, DroneStatus, RcConfig};

pub mod osd_structs;
pub use osd_structs::{FpvCameraOSDCalibration, OsdConfig, OsdState};

pub mod focus;
pub use focus::{FocusMotorConfig, FocusMotorType, TrinamicFocusConfig};

pub mod linear_observation_model;
pub mod motion_model;

pub mod utils;
pub use utils::{elapsed, elapsed_by, now, ChangeDetector, MyTimestamp, Timestamped};

mod backwards_compat;

pub const EVENTS_PATH: &str = "events";

/// The default unicast UDP send/receive port.
pub const UNICAST_UDP_DEFAULT_PORT: u16 = 8080;
pub const UNICAST_UDP_DEFAULT: &str = const_format::concatcp!("0.0.0.0:", UNICAST_UDP_DEFAULT_PORT);

pub type CamNameString = String;

pub use pwm_motor_types::{
    FloatType, PwmDuration, PwmSerial, PwmState, DATATYPES_VERSION, VERSION_RESPONSE_JSON_NEWLINE,
};

// --------------------------------------------------------------------------

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[allow(dead_code)]
#[derive(Default)]
pub enum SystemGeometry {
    #[default]
    MovingMirror,
    MovingCamera,
}

impl SystemGeometry {
    /// theta is tilt, phi is pan
    pub fn sensor_to_motor_transform(
        &self,
        sensor_x: FloatType,
        sensor_y: FloatType,
        theta: FloatType,
        phi: FloatType,
    ) -> (FloatType, FloatType) {
        match *self {
            SystemGeometry::MovingCamera => {
                let pan_angle = sensor_x / theta.cos();
                let tilt_angle = sensor_y;
                (pan_angle, tilt_angle)
            }
            SystemGeometry::MovingMirror => {
                let pan_angle =
                    sensor_x * phi.cos() / theta.cos() - sensor_y * phi.sin() / theta.cos();
                let tilt_angle = sensor_x * phi.sin() + sensor_y * phi.cos();
                (pan_angle, tilt_angle)
            }
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[allow(dead_code)]
#[derive(Default)]
pub enum MotorType {
    #[default]
    PwmServo,
    Trinamic,
    Gimbal,
}

#[derive(Debug, Clone)]
pub struct StampedTrackingState {
    /// The timestamp associated with the sensor data if an observation was used.
    pub centroid_timestamp: Option<chrono::DateTime<chrono::Utc>>,
    /// The tracking state, including the motor commands.
    pub tracking_state: TrackingState,
    /// The timestamp associated with processing the sensor data to update the
    /// motor commands.
    pub processed_timestamp: chrono::DateTime<chrono::Local>,
}

/// Tracking state in global coordinates with velocities
#[derive(Debug, Clone)]
pub struct TrackingState {
    pub mode: DeviceMode,
    /// (pan, tilt, pan_velocity, tilt_velocity) estimates for target. None if Kalman filter not yet started.
    pub kalman_estimates: KalmanEstimatesGlobalDynamic,
    /// Kalman estimates for target distance
    pub kalman_estimates_distance: KalmanEstimatesDistance,
    /// Pan observation
    pub pan_obs: FloatType,
    /// Tilt observation
    pub tilt_obs: FloatType,
    /// Distance observation
    pub dist_obs: FloatType,
    /// Timestamp of last observation
    pub last_observation: Option<chrono::DateTime<chrono::Utc>>,
    /// Pan integral error
    pub pan_err_integral: FloatType,
    /// Tilt integral error
    pub tilt_err_integral: FloatType,
    pub last_motor_readout: Option<MotorPositionResult>,
    /// Current motor command, pan angle
    pub pan_command: Angle,
    /// Current motor command, tilt angle
    pub tilt_command: Angle,
    /// Current motor command, focus distance
    pub focus_command: Option<RadialDistance>,
    pub focus_command_motpos: FloatType, //raw, in microsteps/motor coordinates
    pub focus_command_motpos_ng: FloatType, //noise-gate filtered focus_command_motpos
    pub focus_backlash_correction: FloatType,
    /// Actual motor position, prediction from position commands, pan angle
    pub pan: Angle,
    /// Actual motor position, prediction from position commands, tilt angle
    pub tilt: Angle,
    pub focus_motpos: FloatType, //microsteps
    /// Current motor speed, predicted from position command, pan angular velocity
    pub pan_velocity: Angle,
    /// Current motor speed, predicted from position command, tilt angular velocity
    pub tilt_velocity: Angle,
    /// Current motor speed, predicted from position command, focus velocity
    pub focus_mot_velocity: FloatType,
}

impl Default for TrackingState {
    fn default() -> Self {
        Self {
            mode: DeviceMode::ManualOpenLoop,
            kalman_estimates: None,
            kalman_estimates_distance: None,
            pan_obs: 0.0,
            tilt_obs: 0.0,
            dist_obs: 0.0,
            pan_err_integral: 0.0,
            tilt_err_integral: 0.0,
            focus_command: Some(RadialDistance(0.5)),
            focus_command_motpos: 0.0,
            focus_command_motpos_ng: 0.0,
            focus_backlash_correction: 0.0,
            last_motor_readout: None,
            pan_command: Angle(0.0),
            tilt_command: Angle(0.0),
            pan: Angle(0.0),
            tilt: Angle(0.0),
            pan_velocity: Angle(0.0),
            tilt_velocity: Angle(0.0),
            last_observation: None,
            focus_motpos: 0.0,
            focus_mot_velocity: 0.0,
        }
    }
}

impl TrackingState {
    pub fn compute_motor_cache(&self) -> MotorValueCache {
        let rel_frame = self.mode.is_rel_frame();
        MotorValueCache {
            // the motor value cache is what is sent to the motors. It should therefore contain the motor commands, not the motor positions
            pan: Angle(self.pan_command.0),
            tilt: Angle(self.tilt_command.0),
            vpan: self.pan_velocity.0,
            vtilt: self.tilt_velocity.0,
            drivemode: MotorDriveMode::Position,
            rel_frame,
            focus: self.focus_command_motpos + self.focus_backlash_correction,
        }
    }
}

// Kalman estimates for a dynamic motion model in global angular coordinates phi and theta
pub type KalmanEstimatesGlobalDynamic = Option<(adskalman::StateAndCovariance<FloatType, na::U4>,)>;

/// Kalman estimates for distance estimation
pub type KalmanEstimatesDistance = Option<(adskalman::StateAndCovariance<FloatType, na::U2>,)>;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct KalmanFilterParameters {
    ///rms of (unknown random) acceleration
    pub motion_noise: FloatType,
    ///rms of inaccuracy of measurement
    pub observation_noise: FloatType,
}

fn is_false(val: &bool) -> bool {
    !val
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct FloControllerConfig {
    pub geometry: SystemGeometry,
    #[serde(default, skip_serializing_if = "is_false")]
    pub pwm_output_enabled: bool,
    /// Kalman filter parameters for pan and tilt estimation
    pub kalman_filter_parameters: KalmanFilterParameters,
    /// Kalman filter parameters for distance estimation
    ///
    /// motion_noise: Motion noise [m/s2] for constant-speed dynamic model is
    /// root-mean-square of (unknown random) acceleration [m/s2]. flying animals
    /// have thrust to mass ratios anywhere between circa 11 and 50 m/s2 [doi:
    /// 10.1098/rsos.160746]. Honey bees are probably around 17 m/s2.
    ///
    /// observation_noise: rms measurement noise at 1 meter; this is internally
    /// scaled according to stereopsis law (i.e., multiplied by r^2)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kalman_filter_dist_parameters: Option<KalmanFilterParameters>,
    #[serde(default)]
    pub control_loop_timestep_secs: FloatType,
    pub kp_pan_angle: FloatType,
    pub kp_tilt_angle: FloatType,
    pub ki_pan_angle: FloatType,
    pub ki_tilt_angle: FloatType,
    ///d terms: add speed*kd to position commands. Speed comes from global-coordinate kalman filter
    #[serde(default)]
    pub kd_pan: FloatType,
    #[serde(default)]
    pub kd_tilt: FloatType,
    /// Coefficients to find "sensor x error angle" from centroid
    pub centroid_to_sensor_x_angle_func: CentroidToAngleCalibration,
    /// Coefficients to find "sensor y error angle" from centroid
    pub centroid_to_sensor_y_angle_func: CentroidToAngleCalibration,
    /// lag between flash firing and centroid data arriving
    #[serde(default)]
    pub centroid_lag: FloatType,
    /// Coefficients for stereopsis calibration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stereopsis_calib: Option<StereopsisCalibration>,
    /// Pan axis endpoints and "neutral position", which is used as the initial home position.
    pub pan_motor_config: MotorConfig,
    /// pan axis endpoints and "neutral position", which is used as the initial home position.
    pub tilt_motor_config: MotorConfig,
    /// focus axis endpoints and "neutral position", which is used as the initial home position.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub focus_motor_config: Option<FocusMotorConfig>,
    #[serde(default, skip_serializing_if = "PwmConfig::is_default")]
    /// tilt axis PWM pulse widths corresponding to motor endpoints. Used only in PWM mode.
    pub pan_pwm_config: PwmConfig,
    #[serde(default, skip_serializing_if = "PwmConfig::is_default")]
    /// Tilt axis PWM pulse widths corresponding to motor endpoints. Used only in PWM mode.
    pub tilt_pwm_config: PwmConfig,
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Pan axis Trinamic motor configuration. Used only with Trinamic motors.
    pub pan_trinamic_config: Option<TrinamicAxisConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Tilt axis Trinamic motor configuration. Used only with Trinamic motors.
    pub tilt_trinamic_config: Option<TrinamicAxisConfig>,
    /// Focus axis Trinamic motor configuration. Used only with Trinamic motors.
    ///  Also contains lens-specific parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub focus_trinamic_config: Option<TrinamicFocusConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pan_thorlabs_galvo_config: Option<ThorlabsGalvoConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tilt_thorlabs_galvo_config: Option<ThorlabsGalvoConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gimbal_config: Option<GimbalConfig>,
    /// Time thresholds for suspend/acquire lock modes.
    pub tracking_thresholds: TrackingThresholds,
    /// Time constant of the motor step response.
    pub motor_timeconstant_secs: FloatType,
    #[serde(default)]
    pub encoder_lag: FloatType,
    /// Name of second camera used for stereopsis.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub secondary_cam_name: Option<CamNameString>,
    /// URL for Strand Cam instance connected to primary camera
    ///
    /// If this is the first connection, the token will be required here.
    /// Otherwise, a saved cookie can be used and the token is not required.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub strand_cam_main: Option<StrandCamConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub strand_cam_secondary: Option<StrandCamConfig>,
    #[serde(default)]
    pub sounds_filenames: SoundsFilenames,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub osd_config: Option<OsdConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub rc_config: Option<RcConfig>,

    /// MAVLink configuration
    #[serde(rename = "mavlink", skip_serializing_if = "Option::is_none")]
    pub mavlink_config: Option<crate::drone_structs::MavlinkConfig>,

    // The following definitions are kept for backwards compatibility.
    #[serde(rename = "adc3_threshold", default, skip_serializing)]
    _unused_adc3_threshold: Option<u16>,
    #[serde(rename = "sensor_type", default, skip_serializing)]
    _unused_sensor_type: Option<backwards_compat::UnusedSensorType>,
    #[serde(rename = "adc_to_sensor_x_angle_func", default, skip_serializing)]
    _unused_adc_to_sensor_x_angle_func: Option<backwards_compat::UnusedAdcToAngleCalibration>,
    #[serde(rename = "adc_to_sensor_y_angle_func", default, skip_serializing)]
    _unused_adc_to_sensor_y_angle_func: Option<backwards_compat::UnusedAdcToAngleCalibration>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct TrackingThresholds {
    pub msecs_to_suspend: i64,
    pub msecs_to_acquire_lock: i64,
}

impl Default for TrackingThresholds {
    fn default() -> Self {
        Self {
            msecs_to_suspend: 5000,
            msecs_to_acquire_lock: 10000,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct MotorConfig {
    pub endpoint_low: Angle,
    pub endpoint_high: Angle,
    pub neutral_position: Angle,
    ///lag in seconds from computed motor command to motor starting to move (but not to reaching the commanded position)
    #[serde(default)]
    pub control_lag: FloatType,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            endpoint_low: Angle::from_degrees(0.0),
            endpoint_high: Angle::from_degrees(90.0),
            neutral_position: Angle::from_degrees(45.0),
            control_lag: 0.0,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct PwmConfig {
    /// Pulse width corresponding to [MotorConfig::endpoint_low], in microseconds
    pub low_value: PwmDuration,
    /// Pulse width corresponding to [MotorConfig::endpoint_high], in microseconds
    pub high_value: PwmDuration,
}

impl PwmConfig {
    pub fn is_default(&self) -> bool {
        self == &Self::default()
    }
    pub fn pwm(&self, target_angle: Angle, motor_config: &MotorConfig) -> PwmDuration {
        let motor_range = motor_config.endpoint_high.0 - motor_config.endpoint_low.0;
        let pwm_range: FloatType = self.high_value.duration_usec - self.low_value.duration_usec;
        let frac = (target_angle.0 - motor_config.endpoint_low.0) / motor_range;
        let duration_usec = frac * pwm_range + self.low_value.duration_usec;

        PwmDuration { duration_usec }
    }
}

impl Default for PwmConfig {
    fn default() -> Self {
        Self {
            low_value: PwmDuration::new(1000.0),
            high_value: PwmDuration::new(2000.0),
        }
    }
}

/// The raw motor position values.
///
/// For galvos controlled by DACs, this is the raw DAC value. For PWM based
/// motor commands, this is the pulse width in microseconds.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Default)]
pub struct RawMotorValue(pub i32);

impl From<i32> for RawMotorValue {
    fn from(orig: i32) -> Self {
        Self(orig)
    }
}

pub const DEVICE_ID_LEN: usize = 12;

#[derive(Serialize, Deserialize, PartialEq, Eq, Clone)]
pub struct DeviceId([u8; DEVICE_ID_LEN]);

impl core::fmt::Debug for DeviceId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        let mut unique_id_hex = [0; DEVICE_ID_LEN * 2];
        unique_id_to_hex(&self.0, &mut unique_id_hex);
        let hex_str = core::str::from_utf8(&unique_id_hex).unwrap();
        write!(f, "{}", hex_str)
    }
}

impl DeviceId {
    pub fn new(data: [u8; DEVICE_ID_LEN]) -> Self {
        DeviceId(data)
    }
    pub fn as_hex(&self, dest: &mut [u8; 24]) {
        unique_id_to_hex(&self.0, dest)
    }
    pub fn as_hex_string(&self) -> String {
        let mut unique_id_hex = [0; 24];
        self.as_hex(&mut unique_id_hex);
        let unique_id_str = core::str::from_utf8(&unique_id_hex).unwrap();
        unique_id_str.into()
    }
}

/// The state of the FLO controller
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct DeviceState {
    ///this is here for display in bui only. The real value is in TrackingStateGlobal
    pub mode: DeviceMode,
    pub motor_type: MotorType,
    pub focusing: bool,
    pub enqueue_drop: u16,
    pub cached_motors: MotorValueCache,
    pub motor_readout: Option<MotorPositionResult>,
    pub recent_centroid_packets: u16,
    pub device_id: DeviceId,
    pub home_position: (Angle, Angle, RadialDistance),
    pub floz_recording_path: Option<RecordingPath>,
    pub stereopsis_state: Option<StereopsisState>,
}

/// The data sent from FLO controller to the BUI
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum BuiEventData {
    DeviceState(DeviceState),
    Config(FloControllerConfig),
}

// /// Configuration to convert an angle into PWM units
// ///
// /// Currently, this implementation offers no actual configuration and sets an
// /// angle of 0 to have a PWM duration of 1.0 msec and an angle of 90 degrees to
// /// have a PWM duration of 2.0 msec.
// #[derive(Default)]
// pub struct PwmMotorCfg {}

// impl PwmMotorCfg {
//     pub fn pwm(
//         &self,
//         target_angle: &Angle,
//         pwm_config: &PwmConfig,
//         motor_config: &MotorConfig,
//     ) -> PwmDuration {
//         let motor_range = motor_config.endpoint_high.0 - motor_config.endpoint_low.0;
//         let pwm_range = pwm_config.high_value.duration_usec - pwm_config.low_value.duration_usec;
//         let frac = (target_angle.0 - motor_config.endpoint_low.0) / motor_range;
//         let duration_usec = frac * pwm_range + pwm_config.low_value.duration_usec;

//         PwmDuration { duration_usec }
//     }
// }

/// A 2D from sensor center to target. Always in radians.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct SensorAngle2D {
    pub x: Angle,
    pub y: Angle,
}

impl SensorAngle2D {
    pub fn new(x: Angle, y: Angle) -> Self {
        Self { x, y }
    }

    pub fn nan() -> Self {
        Self {
            x: Angle::nan(),
            y: Angle::nan(),
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct MotorPositionResult {
    pub local: chrono::DateTime<chrono::Local>,
    ///motor position, relative to frame
    pub pan_enc: Angle,
    pub tilt_enc: Angle,
    ///camera rotational position in field CS, imu-based (may drift?). For BYO setup, equal to _enc values.
    pub pan_imu: Angle,
    pub tilt_imu: Angle,
    ///speeds. Only supported by gimbal; these are imu speed, i.e. rotation speed around camera axes, not always equal to motor speeds even for stationary frame
    pub vpan_imu: Option<FloatType>,
    pub vtilt_imu: Option<FloatType>,
}

impl Default for MotorPositionResult {
    fn default() -> Self {
        Self {
            local: chrono::Local::now(),
            pan_enc: Angle(0.0),
            tilt_enc: Angle(0.0),
            pan_imu: Angle(0.0),
            tilt_imu: Angle(0.0),
            vpan_imu: None,
            vtilt_imu: None,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Copy)]
pub enum MotorDriveMode {
    ///move to specified position and stay there (for drone: relative to frame)
    Position,
    ///move with specified speed (for drone: relative to earth)
    Speed,
    //Track, //move so that position = (specified_position) + speed * (now() - timestamp) (not yet supported by anything)
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct MotorValueCache {
    pub pan: Angle,
    pub tilt: Angle,
    pub vpan: FloatType,
    pub vtilt: FloatType,
    ///drive mode only applies to pan and tilt, not focus
    pub drivemode: MotorDriveMode,
    ///if true, angles are relative to drone. If false, angles are imu angles
    pub rel_frame: bool,
    pub focus: FloatType,
}

impl Default for MotorValueCache {
    fn default() -> Self {
        Self {
            pan: Angle(0.0),
            tilt: Angle(0.0),
            vpan: 0.0,
            vtilt: 0.0,
            drivemode: MotorDriveMode::Position,
            rel_frame: true,
            focus: 0.0,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct TrinamicAxisConfig {
    pub microsteps_per_radian: FloatType,
    #[serde(default)]
    pub speed_limit: Option<FloatType>, // [microstep/s]
    #[serde(default)]
    pub acceleration: Option<FloatType>, // [microstep/s^2]
}

impl TrinamicAxisConfig {
    pub fn convert(&self, angle: Angle) -> i32 {
        ((angle.0) * self.microsteps_per_radian).round() as i32
    }
    pub fn convert_back(&self, pos: i32) -> Angle {
        Angle(pos as FloatType / self.microsteps_per_radian)
    }
}

impl Default for TrinamicAxisConfig {
    fn default() -> Self {
        Self {
            microsteps_per_radian: 10_000.0,
            speed_limit: None,
            acceleration: None,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct StrandCamConfig {
    pub url: String,
    #[serde(default)]
    pub on_attach_json_commands: Vec<String>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct GimbalConfig {
    pub port_path: String,
    pub reverse_pan: bool,
    pub reverse_tilt: bool,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct ThorlabsGalvoConfig {
    scale: FloatType,
    offset: FloatType,
}

impl ThorlabsGalvoConfig {
    pub fn convert(&self, angle: &Angle) -> u16 {
        ((angle.0) * self.scale + self.offset).round() as u16
    }
}

impl Default for ThorlabsGalvoConfig {
    fn default() -> Self {
        Self {
            scale: 100.0,
            offset: 0.0,
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
#[derive(Default)]
pub struct SoundsFilenames {
    pub enter_manual_open_loop: Option<String>,
    pub enter_closed_loop: Option<String>,
    pub enter_suspended_closed_loop: Option<String>,
    pub enter_acquiring_lock: Option<String>,
}

impl SoundsFilenames {
    pub fn filename(&self, mode: DeviceMode) -> Option<&String> {
        match mode {
            DeviceMode::ManualOpenLoop => self.enter_manual_open_loop.as_ref(),
            DeviceMode::ClosedLoop => self.enter_closed_loop.as_ref(),
            DeviceMode::SuspendedClosedLoop => self.enter_suspended_closed_loop.as_ref(),
            DeviceMode::AcquiringLock => self.enter_acquiring_lock.as_ref(),
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct ClosedLoopErrors {
    pub pan_error: FloatType,
    pub tilt_error: FloatType,
    pub pan_error_variance: FloatType,
    pub tilt_error_variance: FloatType,
    pub pan_error_integral: FloatType,
    pub tilt_error_integral: FloatType,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct StereopsisState {
    pub ctrd_x_0: FloatType,
    pub ctrd_x_1: FloatType,
    pub dx: FloatType,
    pub dist: FloatType,
}

impl StereopsisState {
    pub fn as_radial_distance(&self) -> RadialDistance {
        RadialDistance(self.dist)
    }
}

/// Result of calibration to calculate error angle from the centroid
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct CentroidToAngleCalibration {
    pub dx_gain: FloatType,
    pub dy_gain: FloatType,
    pub offset: FloatType,
}

/// Result of calibration to calculate distance from stereopsis
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct StereopsisCalibration {
    pub r1_m: FloatType,
    pub x_offset_1_px: FloatType,
    pub r2_m: FloatType,
    pub x_offset_2_px: FloatType,
    pub pixel_size_um: FloatType,
}

impl StereopsisCalibration {
    pub fn centroids_to_distance(
        &self,
        centroids: (MomentCentroid, MomentCentroid),
    ) -> StereopsisState {
        let (centroid_main, centroid_stereo) = centroids;

        let pixel_size = self.pixel_size_um * 1e6;
        let dx = (centroid_main.x() - centroid_stereo.x()) * pixel_size;
        let (r1, r2) = (self.r1_m, self.r2_m);
        let delta1 = (self.x_offset_1_px as FloatType) * pixel_size;
        let delta2 = (self.x_offset_2_px as FloatType) * pixel_size;

        let f = r1 * r2 * (delta1 - delta2) / (r2 - r1);
        let delta_infty = delta1 - f / r1;

        // let R = &self.scale * &self.focal_length_mm *&self.camera_distance_mm
        // /(&self.pixel_size_mm *(dx.abs()));
        let r = f / (dx - delta_infty).abs();

        StereopsisState {
            ctrd_x_0: centroid_main.x(),
            ctrd_x_1: centroid_stereo.x(),
            dx: centroid_main.x() - centroid_stereo.x(),
            dist: r,
        }
    }
}

#[derive(Debug, PartialEq, Eq, Serialize, Deserialize, Clone)]
pub enum TimestampSource {
    BraidTrigger,
    HostAcquiredTimestamp,
}

// This needs to be kept identical to the version in Strand Cam.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct MomentCentroid {
    pub schema_version: u8,
    pub framenumber: u32,
    pub timestamp_source: TimestampSource,
    pub timestamp: chrono::DateTime<chrono::Utc>,
    pub mu00: FloatType,
    pub mu01: FloatType,
    pub mu10: FloatType,
    pub center_x: u32,
    pub center_y: u32,
    #[serde(default)]
    pub cam_name: CamNameString,
}

impl Default for MomentCentroid {
    fn default() -> Self {
        Self {
            schema_version: Default::default(),
            framenumber: 42,
            timestamp_source: TimestampSource::HostAcquiredTimestamp,
            timestamp: Default::default(),
            mu00: Default::default(),
            mu01: Default::default(),
            mu10: Default::default(),
            center_x: Default::default(),
            center_y: Default::default(),
            cam_name: Default::default(),
        }
    }
}

impl MomentCentroid {
    pub fn x(&self) -> FloatType {
        self.mu10 / self.mu00
    }
    pub fn y(&self) -> FloatType {
        self.mu01 / self.mu00
    }
}

impl Default for KalmanFilterParameters {
    fn default() -> Self {
        KalmanFilterParameters {
            motion_noise: 1e2,
            observation_noise: 1e-3,
        }
    }
}

impl Default for FloControllerConfig {
    fn default() -> Self {
        Self {
            geometry: SystemGeometry::default(),
            pwm_output_enabled: false,
            strand_cam_main: Default::default(),
            strand_cam_secondary: Default::default(),
            kalman_filter_parameters: KalmanFilterParameters::default(),
            kalman_filter_dist_parameters: None,
            control_loop_timestep_secs: 0.001,
            kp_pan_angle: 0.0,
            kp_tilt_angle: 0.0,
            ki_pan_angle: 1e3,
            ki_tilt_angle: 1e3,
            kd_pan: 0.0,
            kd_tilt: 0.0,
            centroid_to_sensor_x_angle_func: CentroidToAngleCalibration::default(),
            centroid_to_sensor_y_angle_func: CentroidToAngleCalibration::default(),
            centroid_lag: 0.0,
            stereopsis_calib: None,
            pan_motor_config: Default::default(),
            tilt_motor_config: Default::default(),
            focus_motor_config: Default::default(),
            pan_pwm_config: Default::default(),
            tilt_pwm_config: Default::default(),
            pan_trinamic_config: Default::default(),
            tilt_trinamic_config: Default::default(),
            focus_trinamic_config: Default::default(),
            gimbal_config: None,
            pan_thorlabs_galvo_config: Default::default(),
            tilt_thorlabs_galvo_config: Default::default(),
            tracking_thresholds: Default::default(),
            motor_timeconstant_secs: 0.02,
            encoder_lag: 0.0,
            secondary_cam_name: None,
            sounds_filenames: Default::default(),
            osd_config: None,
            rc_config: None,
            mavlink_config: None,

            _unused_adc3_threshold: None,
            _unused_sensor_type: None,
            _unused_adc_to_sensor_x_angle_func: None,
            _unused_adc_to_sensor_y_angle_func: None,
        }
    }
}

impl DeviceState {
    pub fn new(device_id: DeviceId) -> Self {
        Self {
            mode: DeviceMode::ManualOpenLoop,
            motor_type: MotorType::PwmServo,
            focusing: false,
            enqueue_drop: 0,
            cached_motors: MotorValueCache::default(),
            motor_readout: None,
            recent_centroid_packets: 0,
            device_id,
            home_position: (Angle(0.0), Angle(0.0), RadialDistance(1.0)),
            floz_recording_path: None,
            stereopsis_state: Default::default(),
        }
    }
}

impl Default for CentroidToAngleCalibration {
    fn default() -> Self {
        Self {
            dx_gain: 0.1,
            dy_gain: 0.1,
            offset: 0.0,
        }
    }
}

impl Default for StereopsisCalibration {
    fn default() -> Self {
        Self {
            // completely random values for now
            r1_m: 2.0,
            x_offset_1_px: 20.0,
            r2_m: 5.0,
            x_offset_2_px: 10.0,
            pixel_size_um: 3.4,
        }
    }
}

#[derive(Debug, Default, PartialEq, Serialize, Deserialize, Clone, Copy)]
pub enum DeviceMode {
    #[default]
    ManualOpenLoop,
    /// tracking!
    ClosedLoop,
    /// lost target. Pointing at the last valid observation (no kalman filter extrapolation).
    SuspendedClosedLoop,
    /// at home position, waiting for observations to begin tracking
    AcquiringLock,
}

impl std::fmt::Display for DeviceMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ManualOpenLoop => write!(f, "Manual Open Loop"),
            Self::ClosedLoop => write!(f, "Closed Loop"),
            Self::SuspendedClosedLoop => write!(f, "Suspended Closed Loop"),
            Self::AcquiringLock => write!(f, "Acquiring Lock"),
        }
    }
}

impl DeviceMode {
    pub fn is_rel_frame(&self) -> bool {
        !(*self == Self::ClosedLoop || *self == Self::SuspendedClosedLoop)
    }
    pub fn is_armed(&self) -> bool {
        match self {
            Self::ManualOpenLoop => false,
            Self::ClosedLoop => true,
            Self::SuspendedClosedLoop => true,
            Self::AcquiringLock => true,
        }
    }
}

fn unique_id_to_hex(unique_id: &[u8; DEVICE_ID_LEN], hex_out: &mut [u8; DEVICE_ID_LEN * 2]) {
    // This is inspired by stm32_device_signature::device_id_hex()
    let hex = b"0123456789abcdef";
    for (i, b) in unique_id.iter().enumerate() {
        let lo = b & 0xf;
        let hi = (b >> 4) & 0xfu8;
        hex_out[i * 2] = hex[hi as usize];
        hex_out[i * 2 + 1] = hex[lo as usize];
    }
}

// -----------------------------------------------------------------------------

#[derive(Debug)]
pub enum SaveToDiskMsg {
    /// Start or stop saving data to disk.
    ToggleSavingFloz(Option<(chrono::DateTime<chrono::Local>, std::path::PathBuf)>),
    /// New centroid data
    CentroidData((chrono::DateTime<chrono::Local>, MomentCentroid)),
    /// New tracking state data
    StampedTrackingState(Box<StampedTrackingState>),
    /// Gimbal encoder offsets
    GimbalEncoderOffsets(GimbalEncoderOffsets),
    /// Gimbal encoder data
    GimbalEncoderData(GimbalEncoderData),
    /// Catch-all for (time)stamped JSON data from MAVLink
    MavlinkData(StampedJson),
    /// Motor positions
    MotorPosition(Box<MotorPositionResult>),
    /// Quit the writing thread.
    ///
    /// This is necessary because we block with no timeout in the writing thread
    /// task, so we have to send a quit message.
    Quit,
    BroadwaySaveToDiskMsg(StampedBMsg),
}

/// Timestamped broadway message
#[derive(Debug, Serialize, Deserialize)]
pub struct StampedBMsg {
    pub stamp: chrono::DateTime<chrono::Local>,
    pub msg: BMsg,
}

/// Broadway message
#[derive(Debug, Serialize, Deserialize)]
pub enum BMsg {
    FloEvent(FloEvent),
    FloDetectionEvent(FloDetectionEvent),
    DroneEvent(crate::drone_structs::DroneEvent),
    DroneRealtimeEvent(crate::drone_structs::DroneRealtimeEvent),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct GimbalEncoderOffsets {
    pub pitch: f64,
    pub yaw: f64,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct GimbalEncoderData {
    pub local: chrono::DateTime<chrono::Local>,
    pub timestamp_ms: u16,
    pub imu_angle_roll: i32,
    pub imu_angle_pitch: i32,
    pub imu_angle_yaw: i32,
    pub gyro_data_roll: i32,
    pub gyro_data_pitch: i32,
    pub gyro_data_yaw: i32,
    pub encoder_raw24_roll: i32,
    pub encoder_raw24_pitch: i32,
    pub encoder_raw24_yaw: i32,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct GimbalRPY {
    pub roll: i32,
    pub pitch: i32,
    pub yaw: i32,
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct StampedJson {
    pub local: chrono::DateTime<chrono::Local>,
    pub typ: String,
    pub data: serde_json::Value,
}

impl StampedJson {
    pub fn new<T>(val: &T, typ: String) -> eyre::Result<Self>
    where
        T: Serialize,
    {
        let local = chrono::Local::now();
        let data = serde_json::to_value(val)?;
        Ok(Self { local, typ, data })
    }
}

// -----------------------------------------------------------------------------

#[cfg(feature = "full")]
/// Get all socket addresses on all interfaces for which this socket address can
/// be reached.
///
/// If this cannot be determined because it is an IPv6 unspecified address,
/// `None` is returned. (This could be fixed in the future.)
pub fn all_addrs(orig_addr: std::net::SocketAddr) -> Option<Vec<std::net::SocketAddr>> {
    use std::net::SocketAddr;

    if orig_addr.ip().is_unspecified() {
        let mut ifaces: Vec<_> = if_addrs::get_if_addrs()
            .unwrap_or_else(|e| {
                tracing::error!("Failed to get local interface addresses: {}", e);
                Default::default()
            })
            .into_iter()
            .filter_map(|iface| {
                let r: Option<SocketAddr> = match (orig_addr, iface.ip()) {
                    (SocketAddr::V4(mut sock_v4), std::net::IpAddr::V4(ipv4)) => {
                        sock_v4.set_ip(ipv4);
                        Some(sock_v4.into())
                    }
                    (SocketAddr::V6(_), std::net::IpAddr::V6(_)) => {
                        // TODO: how to specify interface in socket? At least
                        // for link local addresses, need to specify interface.
                        // See https://superuser.com/a/99753.
                        None
                    }
                    _ => None,
                };
                r
            })
            .collect();
        ifaces.sort();
        if ifaces.is_empty() {
            None
        } else {
            Some(ifaces)
        }
    } else {
        Some(vec![orig_addr])
    }
}

// -----------------------------------------------------------------------------
// HTTP stuff
pub const EVENT_NAME: &str = "flo-evt";

#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
pub struct RecordingPath {
    path: String,
    start_time: chrono::DateTime<chrono::Local>,
    current_size_bytes: Option<usize>,
}

impl RecordingPath {
    pub fn new(path: String) -> Self {
        let start_time = chrono::Local::now();
        RecordingPath::from_path_and_time(path, start_time)
    }
    pub fn from_path_and_time(path: String, start_time: chrono::DateTime<chrono::Local>) -> Self {
        Self {
            path,
            start_time,
            current_size_bytes: None,
        }
    }
    pub fn path(&self) -> String {
        self.path.clone()
    }
    pub fn start_time(&self) -> chrono::DateTime<chrono::Local> {
        self.start_time
    }
}

// -----------------------------------------------------------------------------

/// A helper to deserialize `FloatType`, treating JSON null as f64::NAN. See
/// https://github.com/serde-rs/json/issues/202
///
/// Caution: inf and -inf are serialized to null, so they will be deserialized
/// as NAN with this. Another caution: this in installed for all serde
/// deserializers, not just JSON. In other words, even if format can
/// serialize/deserialize NAN with no trouble, this deserialized will
/// nevertheless still be used.
fn deserialize_float_null_as_nan<'de, D: Deserializer<'de>>(des: D) -> Result<FloatType, D::Error> {
    let optional = Option::<FloatType>::deserialize(des)?;
    Ok(optional.unwrap_or(FloatType::NAN))
}

// -----------------------------------------------------------------------------
// tests

/// Test [deserialize_float_null_as_nan] above in both CBOR (which natively
/// represents NaN and infinity) and JSON (which cannot).
#[test]
fn test_serde_nan() -> eyre::Result<()> {
    #[derive(Debug, Serialize, Deserialize)]
    struct MyStruct {
        angle: Angle,
        dist: RadialDistance,
    }

    // with finite floats
    let finite = MyStruct {
        angle: Angle(0.0),
        dist: RadialDistance(1.234),
    };

    let has_nans = MyStruct {
        angle: Angle(FloatType::NAN),
        dist: RadialDistance(FloatType::NAN),
    };

    // JSON with finite values
    let buf = serde_json::to_string(&finite)?;
    let normal_floats: MyStruct = serde_json::from_str(&buf)?;
    assert!((normal_floats.angle.0).abs() < 1e-10);
    assert!((normal_floats.dist.0 - 1.234).abs() < 1e-10);

    // JSON with NAN values
    let buf = serde_json::to_string(&has_nans)?;
    let with_nans: MyStruct = serde_json::from_str(&buf)?;
    assert!(with_nans.angle.0.is_nan());
    assert!(with_nans.dist.0.is_nan());

    Ok(())
}
