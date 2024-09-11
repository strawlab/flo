#![no_std]

use serde::{Deserialize, Serialize};

#[cfg(feature = "use-defmt")]
use defmt::Format;

// According to my rough benchmarking, f32 is about 2x as fast as f64 on the STM32H7 chip.
pub type FloatType = f64;

// ----------------------------------------------------------------------
// Datatypes

/// Version number for datatypes
pub const DATATYPES_VERSION: u16 = 11; // Increment this if you change definitions here

/// A JSON + newline representation of the PwmSerial representation
pub const VERSION_RESPONSE_JSON_NEWLINE: &[u8] = b"{\"VersionResponse\":11}\n";

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PwmSerial {
    Set(PwmState),
    VersionRequest,
    VersionResponse(u16),
}

/// newtype wrapper of FloatType to specify duration of PWM pulse.
#[derive(Clone, Copy, PartialEq, Debug, Serialize, Deserialize)]
#[cfg_attr(feature = "use-defmt", derive(Format))]
#[serde(transparent)]
pub struct PwmDuration {
    pub duration_usec: FloatType,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PwmState {
    pub pan: PwmDuration,
    pub tilt: PwmDuration,
    pub enabled: bool,
}

// ----------------------------------------------------------------------

impl PwmDuration {
    pub fn new(duration_usec: FloatType) -> Self {
        Self { duration_usec }
    }
}

impl Default for PwmDuration {
    fn default() -> Self {
        Self {
            duration_usec: 1500.0,
        }
    }
}

#[test]
fn test_json_newline_version() -> eyre::Result<()> {
    // This ensures that `VERSION_RESPONSE_JSON_NEWLINE` stays up to date with
    // `DATATYPES_VERSION` and also that it ends with newline.
    assert!(VERSION_RESPONSE_JSON_NEWLINE.ends_with(b"\n"));
    let decoded: PwmSerial = serde_json::from_slice(VERSION_RESPONSE_JSON_NEWLINE)?;
    assert_eq!(PwmSerial::VersionResponse(DATATYPES_VERSION), decoded);
    Ok(())
}
