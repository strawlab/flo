use bytes::Bytes;
use simplebgc_derive::BgcPayload;

#[allow(dead_code)]
#[derive(thiserror::Error, Debug)]
enum PayloadParseError {
    #[error("invalid flags value for {name}")]
    InvalidFlags { name: String },
    #[error("invalid enum value for {name}")]
    InvalidEnum { name: String },
}

#[allow(dead_code)]
trait Payload {
    /// Parses this payload from bytes according to the SimpleBGC spec.
    fn from_bytes(b: Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized;

    /// Converts this payload to bytes according to the SimpleBGC spec.
    fn to_bytes(&self) -> Bytes
    where
        Self: Sized;
}

// This checks that `derive(BgcPayload)` compiles.
#[derive(BgcPayload)]
#[allow(dead_code)]
struct BoardInfo {
    #[kind(raw)]
    #[name("BOARD_VER")]
    pub board_version: u8,
}
