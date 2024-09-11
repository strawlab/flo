use crate::commands::constants::*;
use crate::payload::*;
use crate::{IncomingCommand, OutgoingCommand};
use bytes::{Buf, BufMut, Bytes, BytesMut};
use thiserror::Error;
use tokio_util::codec::{Decoder, Encoder};

pub trait SbgcCodec {}

#[derive(Default)]
pub struct V1Codec;
#[derive(Default)]
pub struct V2Codec {
    in_sync: bool,
}

impl SbgcCodec for V1Codec {}
impl SbgcCodec for V2Codec {}

#[derive(Error, Debug)]
pub enum MessageParseError {
    #[error("bad version code")]
    BadVersionCode,
    #[error("bad command id: {id}")]
    BadCommandId { id: u8 },
    #[error("bad header checksum, expected {expected:#X}, got {actual:#X}")]
    BadHeaderChecksum { expected: u8, actual: u8 },
    #[error("bad payload checksum, expected {expected:#X}, got {actual:#X}")]
    BadPayloadChecksum { expected: u16, actual: u16 },
    #[error("there was not enough data in the buffer to read the whole message")]
    InsufficientData,
    #[error(transparent)]
    PayloadParse(#[from] PayloadParseError),
    #[error("there was an IO error")]
    IoError(std::io::Error),
}

impl From<std::io::Error> for MessageParseError {
    fn from(error: std::io::Error) -> Self {
        MessageParseError::IoError(error)
    }
}

pub trait Message {
    fn command_id(&self) -> u8;

    /// Returns a commands ID and a `Bytes` object representing
    /// the bytes of this payload.
    fn to_payload_bytes(&self) -> Bytes;

    fn from_payload_bytes(id: u8, bytes: Bytes) -> Result<Self, MessageParseError>
    where
        Self: Sized;

    fn to_v1_bytes(&self) -> Bytes {
        let cmd = self.command_id();
        let payload = self.to_payload_bytes();
        let mut buf = BytesMut::with_capacity(payload.len() + 8);

        buf.put_u8(0x3E);
        buf.put_u8(cmd);
        buf.put_u8(payload.len() as u8);

        let header_checksum = cmd.wrapping_add(payload.len() as u8);
        let payload_checksum = payload.iter().fold(0u8, |l, r| l.wrapping_add(*r));

        buf.put_u8(header_checksum);
        buf.put(payload);
        buf.put_u8(payload_checksum);

        buf.freeze()
    }

    fn to_v2_bytes(&self) -> Bytes {
        let cmd = self.command_id();
        let payload = self.to_payload_bytes();
        let mut buf = BytesMut::with_capacity(payload.len() + 8);

        buf.put_u8(0x24);
        buf.put_u8(cmd);
        buf.put_u8(payload.len() as u8);

        let header_checksum = cmd.wrapping_add(payload.len() as u8);

        buf.put_u8(header_checksum);
        buf.put(payload);

        let payload_checksum = checksum_bgc_v2(&buf[1..]);
        buf.put_u16_le(payload_checksum);

        buf.freeze()
    }

    /// On success, returns the number of bytes read from the buffer
    fn from_bytes(buf: &[u8]) -> Result<(Self, usize), MessageParseError>
    where
        Self: Sized,
    {
        // use indexing so as not to consume bytes if it's not valid
        match buf[0] {
            0x3E => Message::from_v1_bytes(buf),
            0x24 => Message::from_v2_bytes(buf),
            _ => Err(MessageParseError::BadVersionCode),
        }
    }

    fn from_v1_bytes(buf: &[u8]) -> Result<(Self, usize), MessageParseError>
    where
        Self: Sized,
    {
        // use indexing so as not to consume bytes if it's not valid

        // assume version byte was already checked
        let cmd = buf[1];

        if cmd == 0 {
            return Err(MessageParseError::BadCommandId { id: cmd });
        }

        let payload_len = buf[2] as usize;
        let expected_header_checksum = buf[3];
        let header_checksum = cmd.wrapping_add(payload_len as u8);

        // wrapping_add is the same as modulo 256
        if expected_header_checksum != header_checksum {
            return Err(MessageParseError::BadHeaderChecksum {
                expected: expected_header_checksum,
                actual: header_checksum,
            });
        }

        if buf.len() < 5 + payload_len {
            return Err(MessageParseError::InsufficientData);
        }

        let payload = Bytes::copy_from_slice(&buf[4..4 + payload_len]);
        let expected_payload_checksum = buf[4 + payload_len];
        let payload_checksum = checksum_bgc_v1(&payload[..]);

        if expected_payload_checksum != payload_checksum {
            return Err(MessageParseError::BadPayloadChecksum {
                expected: expected_payload_checksum as u16,
                actual: payload_checksum as u16,
            });
        }

        Self::from_payload_bytes(cmd, payload).map(|m| (m, payload_len + 5))
    }

    fn from_v2_bytes(buf: &[u8]) -> Result<(Self, usize), MessageParseError>
    where
        Self: Sized,
    {
        // use indexing so as not to consume bytes if it's not valid

        // assume version byte was already checked
        let cmd = buf[1];

        if cmd == 0 {
            return Err(MessageParseError::BadCommandId { id: cmd });
        }

        let payload_len = buf[2] as usize;
        let expected_header_checksum = buf[3];
        let header_checksum = cmd.wrapping_add(payload_len as u8);

        // wrapping_add is the same as modulo 256
        if expected_header_checksum != header_checksum {
            return Err(MessageParseError::BadHeaderChecksum {
                expected: expected_header_checksum,
                actual: header_checksum,
            });
        }

        if buf.len() < 6 + payload_len {
            return Err(MessageParseError::InsufficientData);
        }

        let payload = Bytes::copy_from_slice(&buf[4..4 + payload_len]);

        let expected_checksum = u16::from_le_bytes([buf[4 + payload_len], buf[5 + payload_len]]);
        let checksum = checksum_bgc_v2(&buf[1..4 + payload_len]);

        if expected_checksum != checksum {
            return Err(MessageParseError::BadPayloadChecksum {
                expected: expected_checksum,
                actual: checksum,
            });
        }

        Self::from_payload_bytes(cmd, payload).map(|m| (m, payload_len + 6))
    }
}

fn checksum_bgc_v1(buf: &[u8]) -> u8 {
    buf.iter().fold(0u8, |l, r| l.wrapping_add(*r))
}

fn checksum_bgc_v2(buf: &[u8]) -> u16 {
    const POLYNOM: u16 = 0x8005;
    let mut crc = 0;

    for &byte in buf.iter() {
        let mut shift_register = 1;
        while shift_register > 0 {
            let data_bit = byte & shift_register != 0;
            let crc_bit = (crc >> 15) != 0;
            crc <<= 1;

            if data_bit != crc_bit {
                crc ^= POLYNOM;
            }

            shift_register <<= 1;
        }
    }

    crc
}

impl Message for OutgoingCommand {
    fn command_id(&self) -> u8 {
        use OutgoingCommand::*;
        match self {
            BoardInfo => CMD_BOARD_INFO,
            BoardInfo3 => CMD_BOARD_INFO_3,
            Reset => CMD_RESET,
            Control { .. } => CMD_CONTROL,
            ControlConfig { .. } => CMD_CONTROL_CONFIG,
            MotorsOn => CMD_MOTORS_ON,
            MotorsOff { .. } => CMD_MOTORS_OFF,
            ReadParams { .. } => CMD_READ_PARAMS,
            ReadParams3 { .. } => CMD_READ_PARAMS_3,
            ReadParamsExt { .. } => CMD_READ_PARAMS_EXT,
            ReadParamsExt2 { .. } => CMD_READ_PARAMS_EXT2,
            ReadParamsExt3 { .. } => CMD_READ_PARAMS_EXT3,
            WriteParams(_) => CMD_WRITE_PARAMS,
            WriteParams3(_) => CMD_WRITE_PARAMS_3,
            RealtimeData3 => CMD_REALTIME_DATA_3,
            GetAngles => CMD_GET_ANGLES,
            GetAnglesExt => CMD_GET_ANGLES,
            RawMessage(msg) => msg.typ,
            _ => unimplemented!(),
        }
    }

    fn to_payload_bytes(&self) -> Bytes {
        use OutgoingCommand::*;
        match self {
            BoardInfo => Bytes::default(),
            BoardInfo3 => Bytes::default(),
            Reset => Bytes::default(),
            Control(data) => Payload::to_bytes(data),
            ControlConfig(data) => Payload::to_bytes(data),
            MotorsOn => Bytes::default(),
            MotorsOff(data) => Payload::to_bytes(data),
            ReadParams(data) => Payload::to_bytes(data),
            ReadParams3(data) => Payload::to_bytes(data),
            ReadParamsExt(data) => Payload::to_bytes(data),
            ReadParamsExt2(data) => Payload::to_bytes(data),
            ReadParamsExt3(data) => Payload::to_bytes(data),
            WriteParams(data) => Payload::to_bytes(data),
            WriteParams3(data) => Payload::to_bytes(data),
            RealtimeData3 => Bytes::default(),
            GetAngles => Bytes::default(),
            GetAnglesExt => Bytes::default(),
            Other { id: _ } => Bytes::default(),
            RawMessage(data) => Payload::to_bytes(data),
        }
    }

    fn from_payload_bytes(id: u8, bytes: Bytes) -> Result<Self, MessageParseError>
    where
        Self: Sized,
    {
        use OutgoingCommand::*;

        Ok(match id {
            CMD_READ_PARAMS => ReadParams(Payload::from_bytes(bytes)?),
            CMD_READ_PARAMS_3 => ReadParams3(Payload::from_bytes(bytes)?),
            CMD_READ_PARAMS_EXT => ReadParamsExt(Payload::from_bytes(bytes)?),
            CMD_READ_PARAMS_EXT2 => ReadParamsExt2(Payload::from_bytes(bytes)?),
            CMD_READ_PARAMS_EXT3 => ReadParamsExt3(Payload::from_bytes(bytes)?),
            CMD_WRITE_PARAMS => WriteParams(Payload::from_bytes(bytes)?),
            CMD_WRITE_PARAMS_3 => WriteParams3(Payload::from_bytes(bytes)?),
            CMD_GET_ANGLES => GetAngles,
            CMD_GET_ANGLES_EXT => GetAnglesExt,
            CMD_CONTROL => Control(Payload::from_bytes(bytes)?),
            CMD_MOTORS_ON => MotorsOn,
            CMD_MOTORS_OFF => MotorsOff(Payload::from_bytes(bytes)?),
            _ => return Err(MessageParseError::BadCommandId { id }),
        })
    }
}

impl Message for IncomingCommand {
    fn command_id(&self) -> u8 {
        match self {
            IncomingCommand::CommandConfirm(_) => CMD_CONFIRM,
            IncomingCommand::CommandError(_) => CMD_ERROR,
            IncomingCommand::BoardInfo(_) => CMD_BOARD_INFO,
            IncomingCommand::BoardInfo3(_) => CMD_BOARD_INFO_3,
            IncomingCommand::GetAngles(_) => CMD_GET_ANGLES,
            IncomingCommand::ReadParams(_) => CMD_READ_PARAMS,
            IncomingCommand::ReadParams3(_) => CMD_READ_PARAMS_3,
            IncomingCommand::ReadParamsExt(_) => CMD_READ_PARAMS_EXT,
            IncomingCommand::RealtimeData3(_) => CMD_REALTIME_DATA_3,
            IncomingCommand::RawMessage(msg) => msg.typ,
        }
    }

    fn to_payload_bytes(&self) -> Bytes {
        use IncomingCommand::*;
        match self {
            CommandConfirm(data) => Payload::to_bytes(data),
            CommandError(data) => Payload::to_bytes(data),
            BoardInfo(info) => Payload::to_bytes(info),
            BoardInfo3(info) => Payload::to_bytes(info),
            GetAngles(angles) => Payload::to_bytes(angles),
            ReadParams(params) => Payload::to_bytes(params),
            ReadParams3(params) => Payload::to_bytes(params),
            ReadParamsExt(params) => Payload::to_bytes(params),
            RealtimeData3(data) => Payload::to_bytes(data),
            RawMessage(msg) => Payload::to_bytes(msg),
        }
    }

    fn from_payload_bytes(id: u8, bytes: Bytes) -> Result<Self, MessageParseError>
    where
        Self: Sized,
    {
        use IncomingCommand::*;

        Ok(match id {
            CMD_CONFIRM => CommandConfirm(Payload::from_bytes(bytes)?),
            CMD_ERROR => CommandError(Payload::from_bytes(bytes)?),
            CMD_BOARD_INFO => BoardInfo(Payload::from_bytes(bytes)?),
            CMD_BOARD_INFO_3 => BoardInfo3(Payload::from_bytes(bytes)?),
            CMD_GET_ANGLES => GetAngles(Payload::from_bytes(bytes)?),
            CMD_READ_PARAMS => ReadParams(Payload::from_bytes(bytes)?),
            CMD_READ_PARAMS_3 => ReadParams3(Payload::from_bytes(bytes)?),
            CMD_READ_PARAMS_EXT => ReadParamsExt(Payload::from_bytes(bytes)?),
            CMD_REALTIME_DATA_3 => RealtimeData3(Payload::from_bytes(bytes)?),
            _ => IncomingCommand::RawMessage(crate::RawMessage {
                typ: id,
                payload: bytes,
            }),
        })
    }
}

impl Decoder for V1Codec {
    type Item = IncomingCommand;
    type Error = MessageParseError;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if src.len() < 5 {
            // not enough data to read length marker
            return Ok(None);
        }
        match IncomingCommand::from_bytes(&src[..]) {
            Ok((m, num_bytes)) => {
                src.advance(num_bytes);
                Ok(Some(m))
            }
            Err(MessageParseError::InsufficientData) => Ok(None),
            Err(e) => Err(e),
        }
    }
}

impl Decoder for V2Codec {
    type Item = IncomingCommand;
    type Error = MessageParseError;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        //tracing::debug!("codec {}", src.remaining());
        if !self.in_sync {
            let mut found = src.len();
            for i in 0..src.len() {
                if src[i] == 0x24 {
                    found = i;
                    tracing::debug!("got start");
                    break;
                }
            }
            src.advance(found);
        }
        if src.remaining() < 6 {
            // not enough data to read length marker
            //tracing::debug!("not enough data to read length marker");
            return Ok(None);
        }
        match IncomingCommand::from_bytes(src.chunk()) {
            Ok((m, num_bytes)) => {
                //tracing::debug!("message!");
                self.in_sync = true;
                src.advance(num_bytes);
                Ok(Some(m))
            }
            Err(MessageParseError::InsufficientData) => {
                //tracing::debug!("MessageParseError::InsufficientData");
                Ok(None)
            }
            Err(e) => {
                src.advance(1); //to not get stuck
                if self.in_sync {
                    //lost sync, error
                    tracing::error!("lost sync: {e:?}");
                    self.in_sync = false;
                    Ok(None)
                } else {
                    //just keep on looking for sync
                    tracing::error!("failed to sync {e:?}, keep looking... ");
                    Ok(None)
                }
            }
        }
    }
}

impl Encoder<OutgoingCommand> for V1Codec {
    type Error = MessageParseError;

    fn encode(&mut self, item: OutgoingCommand, dst: &mut BytesMut) -> Result<(), Self::Error> {
        let bytes = item.to_v1_bytes();
        dst.put_slice(&bytes[..]);
        Ok(())
    }
}

impl Encoder<OutgoingCommand> for V2Codec {
    type Error = MessageParseError;

    fn encode(&mut self, item: OutgoingCommand, dst: &mut BytesMut) -> Result<(), Self::Error> {
        let bytes = item.to_v2_bytes();
        dst.put_slice(&bytes[..]);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::{Message, OutgoingCommand, ParamsQuery};
    use std::error::Error;

    #[test]
    fn sanity() -> Result<(), Box<dyn Error>> {
        let packet = [0x3E, 0x52, 0x01, 0x53, 0x01, 0x01];
        let (msg, read) = OutgoingCommand::from_bytes(&packet[..])?;

        assert_eq!(read, 6, "should have read 6 bytes");
        assert_eq!(
            msg,
            OutgoingCommand::ReadParams(ParamsQuery { profile_id: 1 })
        );

        Ok(())
    }
}
