use crate::{Payload, PayloadParseError};
use bytes::{Buf, BufMut, Bytes, BytesMut};

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConfirmData {
    pub cmd_id: u8,
    pub data: Option<u16>,
}

impl Payload for ConfirmData {
    fn from_bytes(mut b: Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized,
    {
        Ok(ConfirmData {
            cmd_id: read_enum!(b, "CMD_ID", u8)?,
            // For some reason I was observing behavior inconsistent with the docs where
            // I was getting 0 bytes here when it says there should be 1 or 2 bytes.
            data: if b.remaining() == 0 {
                None
            } else if b.remaining() == 1 {
                Some(read_enum!(b, "DATA", u8)?)
            } else {
                Some(read_enum!(b, "DATA", u16)?)
            },
        })
    }

    fn to_bytes(&self) -> Bytes
    where
        Self: Sized,
    {
        let b = match self.data {
            None => {
                let mut b = BytesMut::with_capacity(1);
                b.put_u8(self.cmd_id);
                b
            }
            Some(data_raw) => {
                let mut b = BytesMut::with_capacity(3);
                b.put_u8(self.cmd_id);
                b.put_u16(data_raw);
                b
            }
        };

        b.freeze()
    }
}

#[derive(BgcPayload, Copy, Clone, Debug, PartialEq)]
pub struct ErrorData {
    #[kind(raw)]
    pub error_code: u8,

    #[kind(raw)]
    pub error_data: [u8; 4],
}
