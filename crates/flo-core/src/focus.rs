use serde::{Deserialize, Serialize};

use crate::{FloatType, RadialDistance};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub enum FocusMotorType {
    #[default]
    NoMotor,
    Trinamic(TrinamicFocusConfig),
    Tilta(TiltaConfig),
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct FocusMotorConfig {
    pub motor: FocusMotorType,
    /// Minimum motor position (motor units / microsteps).
    pub min_pos: FloatType,
    /// Maximum motor position (motor units / microsteps).
    pub max_pos: FloatType,
    /// Focus offset in motor units; startup value, also adjustable interactively.
    pub pos_offset: FloatType,
    /// Change in focus offset per BUI button press (motor units).
    pub adjust_step: FloatType,
    /// Motor position at startup and shutdown (motor units).
    pub park_position: FloatType,
    /// Object distance at which to position focus when stopping tracking.
    pub home_position: RadialDistance,
    /// Backlash compensation amount (motor units).
    pub backlash: FloatType,
    /// Noise gate threshold below which position changes are ignored (motor units).
    pub noise_gate: FloatType,
    pub cal: FocusCalibration,
}

impl Default for FocusMotorConfig {
    fn default() -> Self {
        Self {
            motor: FocusMotorType::NoMotor,
            min_pos: 0.0,
            max_pos: 100.0,
            pos_offset: 0.0,
            adjust_step: 10.0,
            park_position: 0.0,
            home_position: RadialDistance(2.0),
            backlash: 0.0,
            noise_gate: 0.0,
            cal: Default::default(),
        }
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct FocusCalibration {
    /// Distance offset: difference between the stereopsis origin and the lens calibration function origin.
    pub distance_offset: RadialDistance,
    /// Polynomial coefficients: `mot_pos = a0 + a1/r + a2/r² + a3/r³`.
    pub a0: FloatType,
    pub a1: FloatType,
    pub a2: FloatType,
    pub a3: FloatType,
    /// Minimum distance the calibration is valid for (typically the closest focus distance of the lens).
    pub min_dist: RadialDistance,
    pub max_dist: RadialDistance,
}

impl Default for FocusCalibration {
    fn default() -> Self {
        Self {
            distance_offset: RadialDistance(0.0),
            a0: 0.0,
            a1: 0.0,
            a2: 0.0,
            a3: 0.0,
            min_dist: RadialDistance(0.3),
            max_dist: RadialDistance(1e100),
        }
    }
}

impl FocusCalibration {
    /// nonlinear conversion of focus radial distance to microsteps.
    pub fn convert(&self, r: RadialDistance, pos_offset: FloatType) -> FloatType {
        self.convert_with_deriv(r, pos_offset).0
    }
    /// Returns `(motor_position, d(motor_position)/dr)` — same value as [`Self::convert`] plus its derivative with respect to `r`.
    pub fn convert_with_deriv(
        &self,
        r: RadialDistance,
        pos_offset: FloatType,
    ) -> (FloatType, FloatType) {
        let r = (r.0 + self.distance_offset.0).clamp(self.min_dist.0, self.max_dist.0);
        let pos = self.a0 + self.a1 / r + self.a2 / r.powi(2) + self.a3 / r.powi(3);
        let d_r_r = -1.0 / (r * r); //derivative of 1/r
        let deriv = d_r_r * (self.a1 + self.a2 * 2.0 / r.powi(1) + 3.0 * self.a3 / r.powi(2));
        (pos + pos_offset, deriv)
    }
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct TrinamicFocusConfig {
    pub port: String,
    /// Override for speed limit (microstep/s).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_limit: Option<FloatType>,
    /// Override for acceleration (microstep/s²).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub acceleration: Option<FloatType>,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct TiltaConfig {
    pub port: String,
}
