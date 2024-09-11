use multiwii_serial_protocol_v2::{MspPacket, MspPacketDirection};

///msp command code
const MSP_DISPLAYPORT: u16 = 182;

// see https://betaflight.com/docs/development/api/displayport
pub enum DisplayportMessage {
    Heartbeat,
    /// Clears the display and allows local rendering on the display device based on telemetry information etc.
    Release,
    /// Clear the display
    ClearScreen,
    /// Write a string at given coordinates
    WriteString(WriteStringPayload),
    /// Trigger a screen draw
    DrawScreen,
    /// Not used by Betaflight. Reserved by Ardupilot and INAV
    Options(u8),
    /// Display system element displayportSystemElement_e at given coordinates
    Sys((u8, u8, u8)),
}

impl DisplayportMessage {
    pub fn msg_code(&self) -> u8 {
        match *self {
            Self::Heartbeat => 0,
            Self::Release => 1,
            Self::ClearScreen => 2,
            Self::WriteString(_) => 3,
            Self::DrawScreen => 4,
            Self::Options(_) => 5,
            Self::Sys(_) => 6,
        }
    }

    pub fn make_msp_packet(&self) -> MspPacket {
        let mut data = Vec::<u8>::new();
        data.push(self.msg_code());
        match self {
            Self::WriteString(sss) => {
                data.extend_from_slice(&[sss.row, sss.col, sss.attribute]);
                data.extend_from_slice(&sss.string[0..sss.string_len]);
            }
            Self::Options(_) => unimplemented!(),
            Self::Sys(_) => unimplemented!(),
            _ => {}
        };
        MspPacket {
            cmd: MSP_DISPLAYPORT,
            direction: MspPacketDirection::FromFlightController,
            data,
        }
    }
}

pub struct WriteStringPayload {
    pub row: u8,
    pub col: u8,
    pub attribute: u8,
    ///null-terminated, up to 30 chars in length
    pub string: [u8; 31],
    pub string_len: usize,
}

impl WriteStringPayload {
    pub fn from_string(s: &str, row: u8, col: u8) -> Self {
        assert_eq!(s.len(), s.as_bytes().len()); //no utf chars allowed
        Self::from_bytestring(s.as_bytes(), row, col)
    }

    pub fn from_bytestring(s: &[u8], row: u8, col: u8) -> Self {
        let l = s.len();
        if l > 30 {
            panic!("displayport string too long")
        };

        let mut r = Self {
            row,
            col,
            attribute: 0,
            string: [0u8; 31],
            string_len: l,
        };
        r.string[0..l].clone_from_slice(s);
        r
    }
}
