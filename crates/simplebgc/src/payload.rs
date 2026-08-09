use bytes::{Buf, Bytes};
use thiserror::Error;

#[derive(Error, Clone, Debug, PartialEq)]
pub enum PayloadParseError {
    #[error("invalid flags value for {name}")]
    InvalidFlags { name: String },
    #[error("invalid enum value for {name}")]
    InvalidEnum { name: String },
    /// A payload ended before a field this crate expects to find in it.
    ///
    /// Reachable from the wire, not just from a bug: the CRC covers the length
    /// byte, so a controller whose firmware sends a shorter version of a
    /// command than we model produces a packet that verifies and then runs out
    /// mid-parse. Reported rather than panicked, which is what the generated
    /// readers used to do, so the caller can keep the bytes and carry on.
    #[error("{name} needs {needed} bytes but only {available} remain in the payload")]
    InsufficientData {
        name: String,
        needed: usize,
        available: usize,
    },
}

pub trait Payload {
    /// Parses this payload from bytes according to the SimpleBGC spec.
    fn from_bytes(b: Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized;

    /// Converts this payload to bytes according to the SimpleBGC spec.
    fn to_bytes(&self) -> Bytes
    where
        Self: Sized;
}

impl Payload for u8 {
    fn from_bytes(mut b: Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized,
    {
        Ok(b.get_u8())
    }

    fn to_bytes(&self) -> Bytes
    where
        Self: Sized,
    {
        Bytes::copy_from_slice(&[*self])
    }
}

impl Payload for i8 {
    fn from_bytes(mut b: Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized,
    {
        Ok(b.get_i8())
    }

    fn to_bytes(&self) -> Bytes
    where
        Self: Sized,
    {
        Bytes::copy_from_slice(&[*self as u8])
    }
}

impl Payload for u16 {
    fn from_bytes(mut b: Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized,
    {
        Ok(b.get_u16_le())
    }

    fn to_bytes(&self) -> Bytes
    where
        Self: Sized,
    {
        Bytes::copy_from_slice(&[*self as u8, (*self >> 8) as u8])
    }
}

impl Payload for i16 {
    fn from_bytes(mut b: Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized,
    {
        Ok(b.get_i16_le())
    }

    fn to_bytes(&self) -> Bytes
    where
        Self: Sized,
    {
        Bytes::copy_from_slice(&[*self as u8, (*self >> 8) as u8])
    }
}
