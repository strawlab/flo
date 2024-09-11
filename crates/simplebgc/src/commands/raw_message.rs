use crate::{Payload, PayloadParseError};

use bytes::Bytes;

#[derive(Clone, Debug, PartialEq)]
pub struct RawMessage {
    pub typ: u8,
    pub payload: Bytes,
}

impl Payload for RawMessage {
    ///leaves typ unfilled!
    fn from_bytes(b: Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized,
    {
        Ok(RawMessage { typ: 0, payload: b })
    }

    fn to_bytes(&self) -> Bytes
    where
        Self: Sized,
    {
        self.payload.clone()
    }
}
