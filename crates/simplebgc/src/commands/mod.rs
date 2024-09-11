#[macro_use]
pub(crate) mod macros;
pub mod constants;

mod board_info;
mod cmd_response;
mod control;
mod get_angles;
mod motors_off;
mod raw_message;
mod read_params;
mod realtime;

pub use self::board_info::*;
pub use self::cmd_response::*;
pub use self::control::*;
pub use self::get_angles::*;
pub use self::motors_off::*;
pub use self::raw_message::*;
pub use self::read_params::*;
pub use self::realtime::*;

use crate::{Payload, PayloadParseError, RollPitchYaw};
use bytes::{BufMut, Bytes, BytesMut};

payload_rpy!(u8, 1);
payload_rpy!(i8, 1);
payload_rpy!(u16, 2);
payload_rpy!(i16, 2);

#[derive(Clone, Debug, PartialEq)]
pub enum IncomingCommand {
    RawMessage(raw_message::RawMessage),
    CommandConfirm(ConfirmData),
    CommandError(ErrorData),
    BoardInfo(BoardInfo),
    BoardInfo3(BoardInfo3),
    GetAngles(RollPitchYaw<AngleInfo>),
    ReadParams(Params3Data),
    ReadParams3(Params3Data),
    ReadParamsExt(ParamsExtData),
    RealtimeData3(RealtimeData3),
}

#[derive(Clone, Debug, PartialEq)]
pub enum OutgoingCommand {
    BoardInfo,
    BoardInfo3,
    Reset,
    Control(ControlData),
    ControlConfig(ControlConfigData),
    MotorsOn,
    MotorsOff(MotorsOffQuery),
    ReadParams(ParamsQuery),
    ReadParams3(ParamsQuery),
    ReadParamsExt(ParamsQuery),
    ReadParamsExt2(ParamsQuery),
    ReadParamsExt3(ParamsQuery),
    WriteParams(Params3Data),
    WriteParams3(Params3Data),
    RealtimeData3,
    GetAngles,
    GetAnglesExt,
    Other { id: u8 }, //arbitrary message with empty payload?
    RawMessage(raw_message::RawMessage),
}
