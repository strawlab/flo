use proc_macro_error::emit_error;
use quote::format_ident;

use crate::primitive::*;
use proc_macro2::Span;
use syn::spanned::Spanned;
use syn::*;

pub enum FieldKind {
    Flags { repr: PrimitiveKind },
    Enum { repr: PrimitiveKind },
    Payload { ty: Type, size: usize },
    Raw { ty: Type },
}

pub struct FieldInfo {
    pub kind: FieldKind,
    pub ident: Option<Ident>,
    pub variable: Ident,
    pub name: String,
    pub span: Span,
    #[allow(dead_code)]
    pub idx: usize,
}

pub fn get_info_for_field(idx: usize, field: &Field) -> Option<FieldInfo> {
    match field.vis {
        Visibility::Public(_) => {}
        _ => emit_error!(&field, "bgc payload fields should be public"),
    }

    // get name of field in Rust
    // or generate one if it doesn't have a name
    let variable = field.ident.clone().unwrap_or(format_ident!("field{}", idx));

    let repr: Option<PrimitiveKind> = field
        .attrs
        .iter()
        // actual helper attribute is called "format" to avoid conflict
        .filter(|&attr| attr.path().is_ident("format"))
        .last()
        .and_then(|attr| match attr.parse_args::<Type>() {
            Ok(ty) => match ty.try_into() {
                Ok(ty) => Some(ty),
                _ => {
                    emit_error!(attr, "invalid repr attribute");
                    None
                }
            },
            _ => {
                emit_error!(attr, "invalid repr attribute");
                None
            }
        });

    let size: Option<usize> = field
        .attrs
        .iter()
        .filter(|&attr| attr.path().is_ident("size"))
        .last()
        .and_then(|attr| match attr.parse_args::<LitInt>() {
            Ok(s) => match s.base10_parse::<usize>() {
                Ok(s) => Some(s),
                Err(_) => {
                    emit_error!(attr, "invalid size attribute");
                    None
                }
            },
            Err(_) => {
                emit_error!(attr, "invalid size attribute");
                None
            }
        });

    let kind = field
        .attrs
        .iter()
        .filter(|&attr| attr.path().is_ident("kind"))
        .last()
        .and_then(|attr| match attr.parse_args::<Ident>() {
            Ok(ident) if ident == "raw" => Some(FieldKind::Raw {
                ty: field.ty.clone(),
            }),
            Ok(ident) if ident == "enumeration" => Some(FieldKind::Enum {
                repr: match repr {
                    Some(repr) => repr,
                    _ => {
                        emit_error!(attr, "repr attribute is required for enum fields");
                        return None;
                    }
                },
            }),
            Ok(ident) if ident == "flags" => Some(FieldKind::Flags {
                repr: match repr {
                    Some(repr) => repr,
                    _ => {
                        emit_error!(attr, "repr attribute is required for flags fields");
                        return None;
                    }
                },
            }),
            Ok(ident) if ident == "payload" => Some(FieldKind::Payload {
                size: match size {
                    Some(size) => size,
                    _ => {
                        emit_error!(attr, "size attribute is required for raw fields");
                        return None;
                    }
                },
                ty: field.ty.clone(),
            }),
            _ => {
                emit_error!(attr, "invalid kind attribute");
                None
            }
        });

    let kind = match kind {
        Some(kind) => kind,
        _ => {
            emit_error!(field, "field kind must be specified");
            return None;
        }
    };

    let name: Option<String> = field
        .attrs
        .iter()
        .filter(|&attr| attr.path().is_ident("name"))
        .last()
        .and_then(|attr| match attr.parse_args::<LitStr>() {
            Ok(name) => Some(name.value()),
            _ => {
                emit_error!(attr, "invalid name attribute");
                None
            }
        })
        .or(field.ident.clone().map(|i| i.to_string().to_uppercase()));

    let name = match name {
        Some(name) => name,
        _ => {
            emit_error!(
                field,
                "unnamed fields must include name of this field in the SimpleBGC spec"
            );
            return None;
        }
    };

    Some(FieldInfo {
        kind,
        idx,
        ident: field.ident.clone(),
        name,
        variable,
        span: field.span(),
    })
}
