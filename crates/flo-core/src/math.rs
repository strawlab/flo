/// general-purpose math functions
use crate::{deserialize_float_null_as_nan, FloatType};

use serde::{Deserialize, Serialize};

pub fn sq(a: FloatType) -> FloatType {
    a * a
}

/// An angle. Always in radians. Newtype wrapper of FloatType.
///
/// Note: can be NaN (not a number)
#[derive(Clone, Copy, PartialEq, Debug, Serialize, Deserialize)]
pub struct Angle(#[serde(deserialize_with = "deserialize_float_null_as_nan")] pub FloatType);

impl Angle {
    pub fn from_degrees(degrees: FloatType) -> Angle {
        let pi = core::f64::consts::PI as FloatType;
        Angle(degrees / 180.0 * pi)
    }
    pub fn nan() -> Self {
        Self(f64::NAN as FloatType)
    }
    pub fn degrees(&self) -> FloatType {
        let pi = core::f64::consts::PI as FloatType;
        self.0 * 180.0 / pi
    }

    pub fn as_float(&self) -> FloatType {
        self.0
    }

    //returns this angle in -pi..pi range
    pub fn constrained_signed(&self) -> Angle {
        let turn = core::f64::consts::TAU as FloatType;
        Angle(self.0 - (self.0 / turn + 0.5).floor() * turn)
    }

    //returns this angle in 0..2pi range
    pub fn constrained_unsigned(&self) -> Angle {
        let turn = core::f64::consts::TAU as FloatType;
        Angle(self.0 - (self.0 / turn).floor() * turn)
    }
}

#[derive(Clone, Copy, PartialEq, Debug, Serialize, Deserialize)]
pub struct RadialDistance(
    #[serde(deserialize_with = "deserialize_float_null_as_nan")] pub FloatType,
);

impl RadialDistance {
    pub fn new(r: FloatType) -> RadialDistance {
        RadialDistance(r)
    }
    pub fn nan() -> Self {
        Self(f32::NAN as FloatType)
    }
    pub fn as_float(&self) -> FloatType {
        self.0
    }
}

impl Default for RadialDistance {
    fn default() -> Self {
        Self(0.0)
    }
}

impl From<FloatType> for RadialDistance {
    fn from(value: FloatType) -> Self {
        Self(value)
    }
}
