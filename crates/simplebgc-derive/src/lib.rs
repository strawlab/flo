//! # `simplebgc-derive`
//! This crate contains the derive macro that is used to take a lot of the drudgery out of
//! serializing/deserializing these structures. See [derive.BgcPayload] for more info.

extern crate proc_macro;
extern crate proc_macro_error;
extern crate quote;

use crate::field::*;
use crate::primitive::*;
use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use proc_macro_error::*;
use quote::{format_ident, quote, quote_spanned};
use syn::spanned::Spanned;
use syn::*;

mod field;
mod primitive;

/// Add this to any structure that represents a payload
/// for a SimpleBGC command.
///
/// If a command contains repeated data, you can consider these as
/// "sub-payloads" and refactor them into their own structure.
///
/// ## `#[kind]`
/// This helper attribute must be present on every member of a `BgcPayload` struct. It will
/// tell the derive macro how to process this member.
///
/// ### `#[kind(enumeration)]`
/// Indicates that this struct member is an enum. The member's type must implement
/// [`num_traits::FromPrimitive`](https://docs.rs/num-traits/0.2.11/num_traits/cast/trait.FromPrimitive.html)
/// and [`num_traits::ToPrimitive`](https://docs.rs/num-traits/0.2.11/num_traits/cast/trait.ToPrimitive.html).
///
/// ### `#[kind(flags)]`
/// Indicates that this struct member is a bit field. The member's type must be an
/// [`enumflags2::BitFlags<T>`](https://docs.rs/enumflags2/0.6.4/enumflags2/struct.BitFlags.html).
///
/// ### `#[kind(payload)]`
/// Indicates that this struct member is a sub-payload. The member's type must implement
/// [`simplebgc::payload`].
///
/// ### `#[kind(raw)]`
/// Indicates that this struct member is a primitive value. The member's type must be a primitive
/// integer, boolean, array of `u8`, or tuple of primitives.
///
/// ## `#[size]`
/// This helper attribute specifies the number of bytes that a sub-payload takes up in the
/// serialized representation, so that it can be known how many bytes to split off and give
/// to `Payload::from_bytes`. It is required for members with `kind(payload)` and has
/// no effect for all others. It accepts one argument: a number representing the size of this
/// sub-payload.
///
/// ```ignore
/// # struct Example {
/// #[kind(payload)]
/// #[size(18)]
/// pub pid: RollPitchYaw<AxisPidParams>,
/// # }
/// ```
///
/// ## `#[format]`
/// This helper attribute specifies the underlying representation of enum and flags members.
/// It takes one argument: a type, which must be a primitive integer type (`u8`, `i8`, `u16`, etc.).
/// ```ignore
/// # struct Example {
///     #[kind(enumeration)]
///     #[format(u8)]
///     pub serial_speed: SerialSpeed,
/// # }
/// ```
///
/// ## `#[name]`
/// This helper attribute specifies the name of this item as specified in the SimpleBGC spec.
/// This is to be used in error messages in case deserialization fails. If it is not provided,
/// it is assumed that the spec name is the same as the member name. This attribute is required
/// for members of tuple structs.
///
/// ```ignore
/// # struct Example {
///     #[kind(enumeration)]
///     #[name("RC_MAP_FC_ROLL")]
///     #[format(u8)]
///     pub fc_roll: RcMap,
/// # }
/// ```
#[proc_macro_error]
#[proc_macro_derive(BgcPayload, attributes(kind, size, name, format))]
pub fn payload_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let ty = input.ident;

    dummy_const_trick(
        &ty,
        match input.data {
            Data::Struct(data) => match data.fields {
                Fields::Named(fields) => {
                    let fields_info = fields
                        .named
                        .iter()
                        .enumerate()
                        .filter_map(|(i, field)| get_info_for_field(i, field))
                        .collect::<Vec<_>>();

                    let parse_stmts = fields_info
                        .iter()
                        .filter_map(get_parser_for_field)
                        .collect::<Vec<_>>();

                    let ser_stmts = fields_info
                        .iter()
                        .filter_map(get_serializer_for_field)
                        .collect::<Vec<_>>();

                    let vars = fields_info
                        .iter()
                        .map(|info| &info.variable)
                        .collect::<Vec<_>>();

                    let fields = fields_info
                        .iter()
                        .map(|info| info.ident.as_ref().unwrap())
                        .collect::<Vec<_>>();

                    quote! {
                        impl Payload for #ty {
                            fn from_bytes(mut _b: Bytes) -> Result<Self, PayloadParseError>
                            where
                                Self: Sized,
                            {
                                #(#parse_stmts)*

                                Ok(#ty {
                                    #(#fields: #vars),*
                                })
                            }

                            fn to_bytes(&self) -> Bytes
                            where
                                Self: Sized,
                            {
                                let mut _b = BytesMut::new();
                                let &#ty { #(#vars),* } = self;

                                #(#ser_stmts)*

                                _b.freeze()
                            }
                        }
                    }
                }
                Fields::Unnamed(fields) => {
                    let fields_info: Vec<_> = fields
                        .unnamed
                        .iter()
                        .enumerate()
                        .filter_map(|(i, field)| get_info_for_field(i, field))
                        .collect();

                    let parse_stmts: Vec<_> = fields_info
                        .iter()
                        .filter_map(get_parser_for_field)
                        .collect();

                    let ser_stmts = fields_info
                        .iter()
                        .filter_map(get_serializer_for_field)
                        .collect::<Vec<_>>();

                    let vars = fields_info
                        .iter()
                        .map(|info| &info.variable)
                        .collect::<Vec<_>>();

                    quote! {
                        impl Payload for #ty {
                            fn from_bytes(mut _b: Bytes) -> Result<Self, PayloadParseError>
                            where
                                Self: Sized,
                            {
                                #(#parse_stmts)*

                                Ok(#ty (
                                    #(#vars),*
                                ))
                            }

                            fn to_bytes(&self) -> Bytes
                            where
                                Self: Sized,
                            {
                                let mut _b = BytesMut::new();
                                let &#ty ( #(#vars),* ) = self;

                                #(#ser_stmts)*

                                _b.freeze()
                            }
                        }
                    }
                }
                Fields::Unit => abort!(data.struct_token, "this does not work on unit structs"),
            },
            Data::Enum(_) => unimplemented!(),
            Data::Union(_) => unimplemented!(),
        },
    )
    .into()
}

// This trick is taken from num_traits:
// https://github.com/rust-num/num-derive/blob/bafa54c551a9c89d005eb9a41d015a6cca6b614f/src/lib.rs#L49
fn dummy_const_trick<T: quote::ToTokens>(name: &Ident, exp: T) -> TokenStream2 {
    let dummy_const = format_ident!("__IMPL_PAYLOAD_FOR_{}", name);
    quote! {
        #[allow(non_upper_case_globals, unused_qualifications)]
        const #dummy_const: () = {
            use bytes::{Bytes, BytesMut, Buf, BufMut};
            #[allow(unused_imports)]
            use enumflags2::{BitFlags};
            #[allow(unused_imports)]
            use num_traits::{FromPrimitive, ToPrimitive};
            #exp
        };
    }
}

const ERR_RAW_PRIMITIVE: &str =
    "field must be primitive type, tuple of primitive types, or array of u8 for raw values";

fn get_parser_for_field(info: &FieldInfo) -> Option<TokenStream2> {
    let var = &info.variable;
    let span = info.span;
    let name = &info.name;

    match &info.kind {
        FieldKind::Payload { ty, size } => Some(quote_spanned! {span=>
            let #var: #ty = Payload::from_bytes(_b.split_to(#size))?;
        }),
        FieldKind::Flags { repr } => {
            let get_value = match repr {
                PrimitiveKind::U8 | PrimitiveKind::I8 => format_ident!("get_{}", repr),
                _ => format_ident!("get_{}_le", repr),
            };

            Some(quote_spanned! {span=>
                let #var = BitFlags::from_bits(_b.#get_value())
                    .or(Err(PayloadParseError::InvalidFlags { name: #name.into() }))?;
            })
        }
        FieldKind::Enum { repr } => {
            let get_value = match repr {
                PrimitiveKind::U8 | PrimitiveKind::I8 => format_ident!("get_{}", repr),
                _ => format_ident!("get_{}_le", repr),
            };

            let from_value = format_ident!("from_{}", repr);

            Some(quote_spanned! {span=>
                let #var = FromPrimitive::#from_value(_b.#get_value())
                    .ok_or(PayloadParseError::InvalidEnum { name: #name.into() })?;
            })
        }
        FieldKind::Raw { ty } => {
            // if it is a primitive, this is simple
            if let Ok(repr) = PrimitiveKind::try_from(ty.clone()) {
                return Some(match repr {
                    PrimitiveKind::Bool => {
                        quote_spanned! {span=>
                            let #var = _b.get_u8() != 0;
                        }
                    }
                    PrimitiveKind::U8 | PrimitiveKind::I8 => {
                        let get_value = format_ident!("get_{}", repr);
                        quote_spanned! {span=>
                            let #var = _b.#get_value();
                        }
                    }
                    _ => {
                        let get_value = format_ident!("get_{}_le", repr);
                        quote_spanned! {span=>
                            let #var = _b.#get_value();
                        }
                    }
                });
            }

            match ty {
                Type::Array(ty) => {
                    if let Ok(PrimitiveKind::U8) = PrimitiveKind::try_from(ty.elem.as_ref().clone())
                    {
                        let len = &ty.len;

                        Some(quote_spanned! {span=>
                            let mut #var = [0u8; #len];
                            _b.copy_to_slice(&mut #var[..]);
                        })
                    } else {
                        emit_error!(ty, ERR_RAW_PRIMITIVE);
                        None
                    }
                }
                Type::Tuple(ty) => {
                    let item_parse_stmts = ty
                        .elems
                        .iter()
                        .enumerate()
                        .filter_map(|(elem_idx, elem_ty)| {
                            // recursion ftw
                            get_parser_for_field(&FieldInfo {
                                name: format!("{}[{}]", &info.name, elem_idx),
                                kind: FieldKind::Raw {
                                    ty: (*elem_ty).clone(),
                                },
                                idx: elem_idx,
                                span: info.span,
                                variable: format_ident!("{}_{}", &info.variable, elem_idx),
                                ident: None,
                            })
                        })
                        .collect::<Vec<_>>();

                    let item_vars = ty
                        .elems
                        .iter()
                        .enumerate()
                        .map(|(elem_idx, _)| format_ident!("{}_{}", &info.variable, elem_idx))
                        .collect::<Vec<_>>();

                    if item_vars.len() != item_parse_stmts.len() {
                        // some of the parse statement generations failed, abort
                        return None;
                    }

                    Some(quote_spanned! {span=>
                        let #var = {
                            #(#item_parse_stmts)*
                            (#(#item_vars),*)
                        };
                    })
                }
                _ => {
                    emit_error!(ty, ERR_RAW_PRIMITIVE);
                    None
                }
            }
        }
    }
}

fn get_serializer_for_field(info: &FieldInfo) -> Option<TokenStream2> {
    let var = &info.variable;
    let span = info.span;

    match &info.kind {
        FieldKind::Payload { .. } => Some(quote_spanned! {span=>
            _b.put(Payload::to_bytes(&#var));
        }),
        FieldKind::Flags { repr } => {
            let put_value = match repr {
                PrimitiveKind::U8 | PrimitiveKind::I8 => format_ident!("put_{}", repr),
                _ => format_ident!("put_{}_le", repr),
            };

            Some(quote_spanned! {span=>
                _b.#put_value(#var.bits());
            })
        }
        FieldKind::Enum { repr } => {
            let put_value = match repr {
                PrimitiveKind::U8 | PrimitiveKind::I8 => format_ident!("put_{}", repr),
                _ => format_ident!("put_{}_le", repr),
            };

            let to_value = format_ident!("to_{}", repr);

            Some(quote_spanned! {span=>
                _b.#put_value(ToPrimitive::#to_value(&#var).unwrap());
            })
        }
        FieldKind::Raw { ty } => {
            // if it is a primitive, this is simple
            if let Ok(repr) = PrimitiveKind::try_from(ty.clone()) {
                return Some(match repr {
                    PrimitiveKind::Bool => {
                        quote_spanned! {span=>
                            _b.put_u8(#var as u8);
                        }
                    }
                    PrimitiveKind::U8 | PrimitiveKind::I8 => {
                        let put_value = format_ident!("put_{}", repr);
                        quote_spanned! {span=>
                            _b.#put_value(#var);
                        }
                    }
                    _ => {
                        let put_value = format_ident!("put_{}_le", repr);
                        let repr = format_ident!("{}", repr);
                        quote_spanned! {span=>
                            _b.#put_value(#var as #repr);
                        }
                    }
                });
            }
            match ty {
                Type::Array(ty) => {
                    if let Ok(PrimitiveKind::U8) = PrimitiveKind::try_from(ty.elem.as_ref().clone())
                    {
                        Some(quote_spanned! {span=>
                            _b.extend_from_slice(&#var[..]);
                        })
                    } else {
                        None
                    }
                }
                Type::Tuple(ty) => {
                    let item_ser_stmts = ty
                        .elems
                        .iter()
                        .enumerate()
                        .filter_map(|(elem_idx, elem_ty)| {
                            // recursion ftw
                            get_serializer_for_field(&FieldInfo {
                                name: format!("{}[{}]", &info.name, elem_idx),
                                kind: FieldKind::Raw {
                                    ty: (*elem_ty).clone(),
                                },
                                idx: elem_idx,
                                span: elem_ty.span(),
                                variable: format_ident!("{}_{}", &info.variable, elem_idx),
                                ident: None,
                            })
                        })
                        .collect::<Vec<_>>();

                    let item_vars = ty
                        .elems
                        .iter()
                        .enumerate()
                        .map(|(elem_idx, _)| format_ident!("{}_{}", &info.variable, elem_idx))
                        .collect::<Vec<_>>();

                    if item_vars.len() != item_ser_stmts.len() {
                        // some of the parse statement generations failed, abort
                        return None;
                    }

                    Some(quote_spanned! {span=>
                        {
                            let (#(#item_vars),*) = #var;
                            #(#item_ser_stmts)*
                        };
                    })
                }
                _ => {
                    emit_error!(ty, ERR_RAW_PRIMITIVE);
                    None
                }
            }
        }
    }
}
