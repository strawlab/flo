use proc_macro2::Ident;
use quote::IdentFragment;
use std::fmt::Display;
use syn::{Path, Type, TypePath};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PrimitiveKind {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    I128,
    U128,
    Bool,
}

#[derive(Clone)]
pub struct InvalidPrimitiveError {
    _ty: Type,
}

impl Display for PrimitiveKind {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            PrimitiveKind::I8 => write!(f, "i8"),
            PrimitiveKind::U8 => write!(f, "u8"),
            PrimitiveKind::I16 => write!(f, "i16"),
            PrimitiveKind::U16 => write!(f, "u16"),
            PrimitiveKind::I32 => write!(f, "i32"),
            PrimitiveKind::U32 => write!(f, "u32"),
            PrimitiveKind::I64 => write!(f, "i64"),
            PrimitiveKind::U64 => write!(f, "u64"),
            PrimitiveKind::I128 => write!(f, "i128"),
            PrimitiveKind::U128 => write!(f, "u128"),
            PrimitiveKind::Bool => write!(f, "bool"),
        }
    }
}

impl IdentFragment for PrimitiveKind {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        Display::fmt(&self, f)
    }
}

impl TryFrom<Ident> for PrimitiveKind {
    type Error = InvalidPrimitiveError;

    fn try_from(ident: Ident) -> Result<Self, Self::Error> {
        use PrimitiveKind::*;
        if ident == "u8" {
            Ok(U8)
        } else if ident == "i8" {
            Ok(I8)
        } else if ident == "u16" {
            Ok(U16)
        } else if ident == "i16" {
            Ok(I16)
        } else if ident == "u32" {
            Ok(U32)
        } else if ident == "i32" {
            Ok(I32)
        } else if ident == "u64" {
            Ok(U64)
        } else if ident == "i64" {
            Ok(I64)
        } else if ident == "u128" {
            Ok(U128)
        } else if ident == "i128" {
            Ok(I128)
        } else if ident == "bool" {
            Ok(Bool)
        } else {
            Err(InvalidPrimitiveError {
                _ty: Type::Path(TypePath {
                    qself: None,
                    path: Path::from(ident),
                }),
            })
        }
    }
}

impl TryFrom<Type> for PrimitiveKind {
    type Error = InvalidPrimitiveError;

    fn try_from(ty: Type) -> Result<Self, Self::Error> {
        match ty {
            Type::Path(ref path) => match path.path.get_ident() {
                Some(ident) => PrimitiveKind::try_from(ident.clone()),
                _ => Err(InvalidPrimitiveError { _ty: ty }),
            },
            _ => Err(InvalidPrimitiveError { _ty: ty }),
        }
    }
}
