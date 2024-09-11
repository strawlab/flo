use crate::{Payload, PayloadParseError, RollPitchYaw};
use bytes::{BufMut, Bytes, BytesMut};

#[derive(BgcPayload, Copy, Clone, Debug, PartialEq)]
pub struct AngleInfo {
    /// Imu angles in 14-bit resolution per full turn
    /// Units: 0,02197265625 degree
    #[kind(raw)]
    #[name("IMU_ANGLE")]
    pub imu_angle: i16,

    /// Target angles in 14-bit resolution per full turn
    /// Units: 0,02197265625 degree
    #[kind(raw)]
    #[name("TARGET_ANGLE")]
    pub target_angle: i16,

    /// Target speed that gimbal should keep, over Euler axes
    /// Units: 0,1220740379 degree/sec
    #[kind(raw)]
    #[name("TARGET_SPEED")]
    pub target_speed: i16,
}

payload_rpy!(AngleInfo, 6);
