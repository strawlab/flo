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

        /// Keep a payload we cannot interpret rather than dropping it.
        ///
        /// By the time this runs both the header checksum and the CRC have
        /// been verified, so the bytes are exactly what the controller sent
        /// and the only thing at fault is this crate's model of them --
        /// typically a flags field carrying a bit no variant defines, which
        /// `BitFlags::from_bits` rejects outright. Degrading to `RawMessage`
        /// is what an unrecognised command id already does, and it keeps the
        /// bytes for whoever wanted them.
        ///
        /// Returning an error instead costs far more than the packet: the
        /// decoder treats it as lost framing and byte-walks to resynchronise,
        /// so one undefined bit takes out the surrounding traffic too.
        fn or_raw<T>(
            id: u8,
            bytes: &Bytes,
            parsed: Result<T, PayloadParseError>,
            understood: impl FnOnce(T) -> IncomingCommand,
        ) -> IncomingCommand {
            match parsed {
                Ok(payload) => understood(payload),
                Err(error) => {
                    tracing::warn!(
                        command_id = id,
                        ?error,
                        payload = ?&bytes[..],
                        "SimpleBGC payload passed both checksums but was not understood; \
                         keeping the raw bytes"
                    );
                    RawMessage(crate::RawMessage {
                        typ: id,
                        payload: bytes.clone(),
                    })
                }
            }
        }

        // `bytes` is cloned per arm rather than moved: `Bytes` is refcounted,
        // so this is a pointer bump, and it leaves the original for `or_raw`
        // to keep when the typed parse fails.
        Ok(match id {
            CMD_CONFIRM => or_raw(
                id,
                &bytes,
                Payload::from_bytes(bytes.clone()),
                CommandConfirm,
            ),
            CMD_ERROR => or_raw(id, &bytes, Payload::from_bytes(bytes.clone()), CommandError),
            CMD_BOARD_INFO => or_raw(id, &bytes, Payload::from_bytes(bytes.clone()), BoardInfo),
            CMD_BOARD_INFO_3 => or_raw(id, &bytes, Payload::from_bytes(bytes.clone()), BoardInfo3),
            CMD_GET_ANGLES => or_raw(id, &bytes, Payload::from_bytes(bytes.clone()), GetAngles),
            CMD_READ_PARAMS => or_raw(id, &bytes, Payload::from_bytes(bytes.clone()), ReadParams),
            CMD_READ_PARAMS_3 => {
                or_raw(id, &bytes, Payload::from_bytes(bytes.clone()), ReadParams3)
            }
            CMD_READ_PARAMS_EXT => or_raw(
                id,
                &bytes,
                Payload::from_bytes(bytes.clone()),
                ReadParamsExt,
            ),
            CMD_REALTIME_DATA_3 => or_raw(
                id,
                &bytes,
                Payload::from_bytes(bytes.clone()),
                RealtimeData3,
            ),
            _ => RawMessage(crate::RawMessage {
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
                    break;
                }
            }
            if found < src.len() {
                if found == 0 {
                    tracing::debug!("SimpleBGC v2 decoder acquired packet start");
                } else {
                    tracing::debug!(
                        discarded_bytes = found,
                        discarded_prefix = ?&src[..found.min(32)],
                        "SimpleBGC v2 decoder resynchronized at packet start"
                    );
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
                // Keep this bounded: this is the first bytes at which the decoder lost
                // framing, and is sufficient to distinguish a stray serial byte from a
                // corrupted SimpleBGC packet without dumping an unbounded receive buffer.
                let preview_len = src.len().min(32);
                tracing::debug!(
                    error = ?e,
                    was_in_sync = self.in_sync,
                    buffered_bytes = src.len(),
                    buffer_prefix = ?&src[..preview_len],
                    "SimpleBGC v2 decoder rejected packet bytes"
                );
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
    use super::*;
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

    /// A real `CMD_BOARD_INFO` reply, captured off a board running firmware
    /// 2.71b9, which this crate used to reject outright: its `STATE_FLAGS1` is
    /// `44`, and bit 5 had no variant. Both checksums verify, so nothing about
    /// these bytes is in doubt -- only our reading of them was.
    const BOARD_INFO_2_71B9: [u8; 24] = [
        0x24, 86, 18, 104, // '$', CMD_BOARD_INFO, payload len, cmd+len
        36,  // BOARD_VER 3.6
        159, 10, // FIRMWARE_VER 2719 -> 2.71b9
        44, // STATE_FLAGS1: init steps 1 and 2, plus undocumented bit 5
        191, 252, // BOARD_FEATURES
        0,   // CONNECTION_FLAG
        0, 0, 0, 0, // FRW_EXTRA_ID
        149, 7, 4, 0, 15, 149, 10, // RESERVED
        59, 130, // CRC16
    ];

    #[test]
    fn a_board_info_reply_with_an_undocumented_state_bit_is_understood() {
        let (msg, read) = IncomingCommand::from_bytes(&BOARD_INFO_2_71B9[..])
            .expect("the packet verifies, so it must parse");
        assert_eq!(read, BOARD_INFO_2_71B9.len());

        let IncomingCommand::BoardInfo(info) = msg else {
            panic!("expected BoardInfo, got {msg:?}");
        };
        assert_eq!(info.board_version, 36);
        assert_eq!(info.firmware_version, 2719);
        assert_eq!(
            info.state.bits(),
            44,
            "the raw byte must survive intact, undocumented bit and all"
        );
        assert_eq!(info.board_features.bits(), 0xFCBF);
    }

    /// The general guarantee: a payload that verifies but cannot be modelled
    /// keeps its bytes and costs nothing else. Before this, such a packet
    /// raised an error, which the decoder read as lost framing.
    #[test]
    fn a_verified_but_unmodellable_payload_is_kept_raw() {
        // A correctly sized CMD_REALTIME_DATA_3 whose RT_DATA_FLAGS byte (at
        // offset 57 of 63) carries bit 1, which `RTDataFlags` does not define.
        // The same shape as the CMD_BOARD_INFO failure above, in a command
        // that arrives continuously rather than once.
        let mut raw = vec![0u8; 63];
        raw[57] = 0b10;
        let payload = Bytes::from(raw);
        let msg = IncomingCommand::from_payload_bytes(CMD_REALTIME_DATA_3, payload.clone())
            .expect("an unmodellable payload must not become an error");
        assert_eq!(
            msg,
            IncomingCommand::RawMessage(crate::RawMessage {
                typ: CMD_REALTIME_DATA_3,
                payload,
            }),
            "the bytes must be handed on rather than dropped"
        );
    }

    /// A payload shorter than the struct we model used to panic inside the
    /// derived reader -- `bytes` panics rather than erroring on a short buffer
    /// -- which took down whatever task was decoding. It is reachable from the
    /// wire, since the CRC covers the length byte: a controller sending a
    /// shorter form of a command than we know produces exactly this.
    #[test]
    fn a_payload_that_runs_out_is_an_error_not_a_panic() {
        // CMD_BOARD_INFO's payload is 18 bytes; hand over 6.
        let short = Bytes::from_static(&[36, 159, 10, 44, 191, 252]);
        let msg = IncomingCommand::from_payload_bytes(CMD_BOARD_INFO, short.clone())
            .expect("a short payload must not become an error, let alone a panic");
        assert_eq!(
            msg,
            IncomingCommand::RawMessage(crate::RawMessage {
                typ: CMD_BOARD_INFO,
                payload: short,
            }),
            "the bytes that did arrive must still be handed on"
        );
    }

    /// The guard reports what was missing rather than merely refusing.
    #[test]
    fn running_out_says_which_field_and_by_how_much() {
        let err = crate::BoardInfo::from_bytes(Bytes::from_static(&[36, 159]))
            .expect_err("2 bytes cannot satisfy BoardInfo");
        assert!(
            matches!(
                &err,
                PayloadParseError::InsufficientData { needed, available, .. }
                    if *needed > *available
            ),
            "expected a shortfall report, got {err:?}"
        );
    }

    /// The consequence that actually hurt: one unreadable packet used to take
    /// the decoder's framing with it, so the *next* packets were lost too.
    #[test]
    fn an_unreadable_packet_does_not_cost_the_next_one() {
        let mut codec = super::V2Codec::default();
        let mut buf = BytesMut::new();
        buf.extend_from_slice(&BOARD_INFO_2_71B9);
        buf.extend_from_slice(&BOARD_INFO_2_71B9);

        for expected in 0..2 {
            match codec.decode(&mut buf) {
                Ok(Some(IncomingCommand::BoardInfo(info))) => {
                    assert_eq!(info.firmware_version, 2719)
                }
                other => panic!("packet {expected} did not decode: {other:?}"),
            }
        }
        assert!(buf.is_empty(), "both packets should be consumed");
    }
}
