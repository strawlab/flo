use simplebgc::{Payload, PayloadParseError, I24};
use simplebgc_derive::BgcPayload;

#[derive(BgcPayload, Clone, Debug, PartialEq)]
pub struct RealTimeDataCustomFlo {
    #[kind(raw)]
    pub timestamp_ms: u16,

    ///unit: 0.02197265625 deg
    #[kind(payload)]
    #[size(6)]
    pub imu_angles: simplebgc::RollPitchYaw<i16>,

    ///unit: 0.06103701895 deg/s
    #[kind(payload)]
    #[size(6)]
    pub gyro_data: simplebgc::RollPitchYaw<i16>,

    #[kind(payload)]
    #[size(9)]
    pub encoder_raw24: simplebgc::RollPitchYaw<I24>,
}

#[derive(BgcPayload, Clone, Debug, PartialEq)]
pub struct RequestStreamIntervalCustom {
    #[kind(raw)]
    pub cmd_id: u8,

    ///milliseconds or sample ratedivisor, depending on sync_to_data
    #[kind(raw)]
    pub interval: u16,

    #[kind(raw)]
    pub realtime_data_custom_flags: u32,

    #[kind(raw)]
    pub padding0: u32,

    #[kind(raw)]
    #[format(u8)]
    pub sync_to_data: bool,

    #[kind(raw)]
    pub padding1: [u8; 9],
}

impl Default for RequestStreamIntervalCustom {
    fn default() -> Self {
        Self {
            cmd_id: simplebgc::constants::CMD_REALTIME_DATA_CUSTOM,
            interval: 1,
            realtime_data_custom_flags: 0,
            padding0: 0,
            sync_to_data: true,
            padding1: [0; 9],
        }
    }
}
