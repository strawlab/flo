use crate::{Payload, PayloadParseError};
use bytes::{BufMut, Bytes, BytesMut};

#[derive(Clone, Debug, PartialEq)]
pub struct RollPitchYaw<T> {
    pub roll: T,
    pub pitch: T,
    pub yaw: T,
}

impl<T> RollPitchYaw<T> {
    pub fn as_ref(&self) -> RollPitchYaw<&T> {
        RollPitchYaw {
            roll: &self.roll,
            pitch: &self.pitch,
            yaw: &self.yaw,
        }
    }

    pub fn as_mut(&mut self) -> RollPitchYaw<&mut T> {
        RollPitchYaw {
            roll: &mut self.roll,
            pitch: &mut self.pitch,
            yaw: &mut self.yaw,
        }
    }

    pub fn combine<U>(self, other: RollPitchYaw<U>) -> RollPitchYaw<(T, U)> {
        RollPitchYaw {
            roll: (self.roll, other.roll),
            pitch: (self.pitch, other.pitch),
            yaw: (self.yaw, other.yaw),
        }
    }

    pub fn map<U, F: Fn(T) -> U>(self, op: F) -> RollPitchYaw<U> {
        RollPitchYaw {
            roll: op(self.roll),
            pitch: op(self.pitch),
            yaw: op(self.yaw),
        }
    }

    pub fn update<U, F: Fn(&mut T) -> U>(&mut self, op: F) -> RollPitchYaw<U> {
        RollPitchYaw {
            roll: op(&mut self.roll),
            pitch: op(&mut self.pitch),
            yaw: op(&mut self.yaw),
        }
    }

    pub fn exec<U, F: Fn(&T) -> U>(&self, op: F) {
        op(&self.roll);
        op(&self.pitch);
        op(&self.yaw);
    }
}

impl<T> From<RollPitchYaw<T>> for (T, T, T) {
    fn from(val: RollPitchYaw<T>) -> Self {
        (val.roll, val.pitch, val.yaw)
    }
}

impl<T> From<(T, T, T)> for RollPitchYaw<T> {
    fn from(t: (T, T, T)) -> Self {
        RollPitchYaw {
            roll: t.0,
            pitch: t.1,
            yaw: t.2,
        }
    }
}

impl<T: Copy> Copy for RollPitchYaw<T> {}

#[macro_export]
macro_rules! payload_rpy {
    ($type: ty, $size: literal) => {
        impl Payload for RollPitchYaw<$type> {
            fn from_bytes(mut b: Bytes) -> Result<Self, PayloadParseError>
            where
                Self: Sized,
            {
                Ok(RollPitchYaw {
                    roll: Payload::from_bytes(b.split_to($size))?,
                    pitch: Payload::from_bytes(b.split_to($size))?,
                    yaw: Payload::from_bytes(b.split_to($size))?,
                })
            }

            fn to_bytes(&self) -> Bytes
            where
                Self: Sized,
            {
                let mut b = BytesMut::with_capacity($size * 3);
                b.put(Payload::to_bytes(&self.roll));
                b.put(Payload::to_bytes(&self.pitch));
                b.put(Payload::to_bytes(&self.yaw));
                b.freeze()
            }
        }
    };
}

///representation of 24-bit signed integer, for encoder data sbgc messages
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct I24(i32);

payload_rpy!(I24, 3);

impl From<I24> for i32 {
    fn from(orig: I24) -> i32 {
        orig.0
    }
}

impl Payload for I24 {
    fn from_bytes(b: bytes::Bytes) -> Result<Self, PayloadParseError>
    where
        Self: Sized,
    {
        assert_eq!(b.len(), 3);
        let (b0, b1, b2) = (b[0], b[1], b[2]);
        let mut value = b0 as i32 + ((b1 as i32) << 8) + ((b2 as i32) << 16);
        if value >= 1 << 23 {
            value -= 1 << 24;
        }
        Ok(I24(value))
    }

    fn to_bytes(&self) -> bytes::Bytes
    where
        Self: Sized,
    {
        let mut val = self.0;
        //reinterpret_cast to unsigned24:
        if val < 0 {
            val += 1 << 24;
        }
        assert!(val >= 0);
        assert!(val < 1 << 24);
        //make the bytes from unsigned
        Bytes::copy_from_slice(&[
            (val & 0xFF) as u8,
            (val >> 8 & 0xFF) as u8,
            (val >> 16 & 0xFF) as u8,
        ])
    }
}

#[test]
fn test_i24_roundtrip() {
    let values_i32: &[i32] = &[
        -0x7F_FFFF, -0x7F_FFFE, 0x7F_FFFF, 0x7F_FFFE, 0x00_0000, 0x00_0001,
    ];
    for value_i32 in values_i32 {
        let value_i24 = I24(*value_i32);
        let bytes = value_i24.to_bytes();
        let roundtrip: I24 = Payload::from_bytes(bytes).unwrap();
        assert_eq!(roundtrip.0, *value_i32);
    }
}
