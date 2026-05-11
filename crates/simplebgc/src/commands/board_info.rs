use crate::{Payload, PayloadParseError};
use enumflags2::{BitFlags, bitflags};

#[bitflags]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum StateFlags1 {
    DebugMode = 1 << 0,
    IsFrameInverted = 1 << 1,
    InitStep1Done = 1 << 2,
    InitStep2Done = 1 << 3,
    StartupAutoRoutineDone = 1 << 4,
}

#[bitflags]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u16)]
pub enum BoardFeatures {
    ThreeAxis = 1 << 0,
    BatMonitoring = 1 << 1,
    Encoders = 1 << 2,
    BodeTest = 1 << 3,
    Scripting = 1 << 4,
    CurrentSensor = 1 << 5,
    MagSensor = 1 << 6,
    OrderOfAxesLetus = 1 << 7,
    ImuEeprom = 1 << 8,
    FrameImuEeprom = 1 << 9,
    CanPort = 1 << 10,
    Momentum = 1 << 11,
    CoggingCorrection = 1 << 12,
    Motor4Control = 1 << 13,
    AccAutoCalib = 1 << 14,
    BigFlash = 1 << 15,
}

#[bitflags]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum ConnectionFlag {
    USB = 1 << 0,
}

#[derive(BgcPayload, Copy, Clone, Debug, PartialEq)]
pub struct BoardInfo {
    #[kind(raw)]
    #[name("BOARD_VER")]
    pub board_version: u8,

    #[kind(raw)]
    #[name("FIRMWARE_VER")]
    pub firmware_version: u16,

    #[kind(flags)]
    #[name("STATE_FLAGS1")]
    #[format(u8)]
    pub state: BitFlags<StateFlags1>,

    #[kind(flags)]
    #[format(u16)]
    pub board_features: BitFlags<BoardFeatures>,

    #[kind(flags)]
    #[format(u8)]
    pub connection_flag: BitFlags<ConnectionFlag>,

    #[kind(raw)]
    pub frw_extra_id: u32,

    #[kind(raw)]
    pub reserved: [u8; 7],
}

#[derive(BgcPayload, Copy, Clone, Debug, PartialEq)]
pub struct BoardInfo3 {
    #[kind(raw)]
    pub device_id: [u8; 9],

    #[kind(raw)]
    pub mcu_id: [u8; 12],

    #[kind(raw)]
    pub eeprom_size: u32,

    #[kind(raw)]
    pub script_slot_size: u16,

    #[kind(raw)]
    pub profile_set_slots: u8,

    #[kind(raw)]
    pub profile_set_cur: u8,

    #[kind(raw)]
    pub reserved: [u8; 32],
}
