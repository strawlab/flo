use bytes::Bytes;
use simplebgc_derive::BgcPayload;

#[expect(dead_code)]
#[derive(thiserror::Error, Debug)]
enum PayloadParseError {
    #[error("invalid flags value for {name}")]
    InvalidFlags { name: String },
    #[error("invalid enum value for {name}")]
    InvalidEnum { name: String },
    // Mirrors the real error in `simplebgc`: the generated parser checks the
    // buffer before each read and reports a shortfall through this.
    #[error("{name} needs {needed} bytes but only {available} remain in the payload")]
    InsufficientData {
        name: String,
        needed: usize,
        available: usize,
    },
}

#[expect(dead_code)]
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
struct BoardInfo {
    #[kind(raw)]
    #[name("BOARD_VER")]
    pub board_version: u8,
}
