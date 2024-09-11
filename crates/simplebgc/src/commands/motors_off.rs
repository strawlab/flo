use crate::*;
use num_traits::*;

#[derive(FromPrimitive, ToPrimitive, Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum MotorsOffMode {
    /// turn motors off leaving driver in a high impedance
    Normal = 0,
    /// turn motors off leaving driver in a low impedance
    Break,
    /// reduce power and wait while all motors stop rotating, then power
    /// off completely
    SafeStop,
}

#[derive(BgcPayload, Copy, Clone, Debug, PartialEq)]
pub struct MotorsOffQuery(
    #[kind(enumeration)]
    #[name("")]
    #[format(u8)]
    pub MotorsOffMode,
);

impl FromPrimitive for MotorsOffQuery {
    fn from_u64(n: u64) -> Option<Self> {
        FromPrimitive::from_u8(n as u8)
    }

    fn from_i64(n: i64) -> Option<Self> {
        FromPrimitive::from_u8(n as u8)
    }

    fn from_u8(n: u8) -> Option<Self> {
        use MotorsOffMode::*;
        Some(match n {
            0 => Self(Normal),
            1 => Self(Break),
            2 => Self(SafeStop),
            _ => return None,
        })
    }
}
