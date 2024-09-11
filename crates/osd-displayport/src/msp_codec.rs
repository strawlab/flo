use bytes::{Buf, BufMut, BytesMut};
use multiwii_serial_protocol_v2::{MspPacket, MspPacketParseError, MspParser};
use thiserror;
use tokio_util::codec::{Decoder, Encoder};

use tracing as log;

#[derive(Debug)]
pub enum MspVersion {
    V1,
    V2,
}

pub struct MspCodec {
    inner_parser: MspParser,
    protocol_version: MspVersion,
}

impl MspCodec {
    pub fn new_v1() -> Self {
        Self {
            inner_parser: MspParser::new(),
            protocol_version: MspVersion::V1,
        }
    }
    pub fn new_v2() -> Self {
        Self {
            inner_parser: MspParser::new(),
            protocol_version: MspVersion::V2,
        }
    }
}

#[derive(thiserror::Error, Debug)]
pub enum MessageParseError {
    #[error("bad version code")]
    OutputBufferSizeMismatch,
    #[error("invalid data")]
    InvalidData,
    #[error("invalid header1")]
    InvalidHeader1,
    #[error("invalid header2")]
    InvalidHeader2,
    #[error("invalid direction value")]
    InvalidDirection,
    #[error("invalid data length")]
    InvalidDataLength,
    #[error("checksum mismatch, expected {expected:#X}, calculated {calculated:#X}")]
    CrcMismatch { expected: u8, calculated: u8 },
    #[error("there was an IO error")]
    IoError(std::io::Error),
}

impl From<std::io::Error> for MessageParseError {
    fn from(error: std::io::Error) -> Self {
        MessageParseError::IoError(error)
    }
}

impl From<MspPacketParseError> for MessageParseError {
    fn from(value: MspPacketParseError) -> Self {
        type I = MspPacketParseError;
        type O = MessageParseError;
        match value {
            I::OutputBufferSizeMismatch => O::OutputBufferSizeMismatch,
            I::CrcMismatch {
                expected,
                calculated,
            } => O::CrcMismatch {
                expected,
                calculated,
            },
            I::InvalidData => O::InvalidData,
            I::InvalidHeader1 => O::InvalidHeader1,
            I::InvalidHeader2 => O::InvalidHeader2,
            I::InvalidDirection => O::InvalidDirection,
            I::InvalidDataLength => O::InvalidDataLength,
        }
    }
}

impl Decoder for MspCodec {
    type Item = MspPacket;
    type Error = MessageParseError;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        //println!("codec {}", src.remaining());
        for i in 0..src.len() {
            let p_r = self.inner_parser.parse(src[i]);
            match p_r {
                Err(e) => {
                    log::error!("osd msp parse error: {e:?}")
                } //and keep on parsing
                Ok(p_r) => {
                    if let Some(msg) = p_r {
                        src.advance(i + 1);
                        return Ok(Some(msg));
                    }
                }
            }
        }
        src.advance(src.len());
        Ok(None)
    }
}

impl Encoder<MspPacket> for MspCodec {
    type Error = MessageParseError;
    fn encode(&mut self, item: MspPacket, dst: &mut BytesMut) -> Result<(), Self::Error> {
        let mut buf: [u8; 280] = [0; 280];
        match self.protocol_version {
            MspVersion::V1 => {
                let bufslice = &mut buf[..item.packet_size_bytes()];
                item.serialize(bufslice)?;
                dst.put_slice(bufslice);
                Ok(())
            }
            MspVersion::V2 => {
                let bufslice = &mut buf[..item.packet_size_bytes_v2()];
                item.serialize_v2(bufslice)?;
                dst.put_slice(bufslice);
                Ok(())
            }
        }
    }
}
