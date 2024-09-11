#![no_std]

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum TitlaFocusDongleMessage {
    VersionRequest,
    ///check this against TitlaFocusDongleMessage::VERSION
    VersionResponse(u16),
    // Ping([u8; 8]),
    // Pong([u8; 8]),
    ///position is 0 to 4095 for full range of focus ring motion
    SetPos(i32),
}

impl TitlaFocusDongleMessage {
    pub const VERSION: u16 = 1;
    pub const VERSION_RESPONSE_JSON_NEWLINE: &'static [u8] = b"{\"VersionResponse\":1}\n";
}
