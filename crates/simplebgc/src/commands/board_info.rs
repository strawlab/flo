use crate::{Payload, PayloadParseError};
use enumflags2::{BitFlags, bitflags};

/// `STATE_FLAGS1` from `CMD_BOARD_INFO`.
///
/// Every bit of the byte has a variant, including the ones the SimpleBGC
/// documentation this crate was written from does not describe. That is not
/// tidiness: `BitFlags::from_bits` refuses a value carrying any bit it does not
/// know, and a `BitFlags` can only hold bits its enum defines. Leaving the top
/// bits out therefore threw away whole `CMD_BOARD_INFO` replies and lost the
/// raw byte with them -- boards running firmware 2.71b9 set bit 5, so FLO
/// recorded no gimbal firmware version at all.
///
/// The undocumented bits are named for their position because inventing a
/// meaning for them would be worse than admitting we do not know one. They
/// still reach `state_flags_raw` in the recorded provenance, which is where a
/// later reader can make sense of them.
#[bitflags]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum StateFlags1 {
    DebugMode = 1 << 0,
    IsFrameInverted = 1 << 1,
    InitStep1Done = 1 << 2,
    InitStep2Done = 1 << 3,
    StartupAutoRoutineDone = 1 << 4,
    Undocumented5 = 1 << 5,
    Undocumented6 = 1 << 6,
    Undocumented7 = 1 << 7,
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

/// `CONNECTION_FLAG` from `CMD_BOARD_INFO`. Total for the same reason as
/// [`StateFlags1`]: this field shares its packet, so a bit undefined here would
/// discard the firmware version and board features alongside it.
#[bitflags]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum ConnectionFlag {
    USB = 1 << 0,
    Undocumented1 = 1 << 1,
    Undocumented2 = 1 << 2,
    Undocumented3 = 1 << 3,
    Undocumented4 = 1 << 4,
    Undocumented5 = 1 << 5,
    Undocumented6 = 1 << 6,
    Undocumented7 = 1 << 7,
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
