#[macro_export]
macro_rules! read_enum {
    ($buf: ident, $name: literal, $repr: ident) => {
        read_enum!($buf, $name, $repr, $repr)
    };
    ($buf: ident, $name: literal, $repr: ident, $kind: ident) => {{
        use num_traits::{FromPrimitive};

        paste! {
            FromPrimitive::[< from_ $kind >]($buf.[< get_ $repr >]()).ok_or(PayloadParseError::InvalidEnum { name: $name.into() })
        }
    }};
}

#[macro_export]
macro_rules! read_flags {
    ($buf: ident, $name: literal, $repr: ident) => {{
        paste! {
            BitFlags::from_bits($buf.[< get_ $repr >]()).or(Err(PayloadParseError::InvalidFlags { name: $name.into() }))
        }
    }}
}

#[macro_export]
macro_rules! read_flags_truncate {
    ($buf: ident, $name: literal, $repr: ident) => {{
        paste! {
            BitFlags::from_bits_truncate($buf.[< get_ $repr >]())
        }
    }};
}
