//! RegMap derive macro implementation.

use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{
    parse::{Parse, ParseStream},
    Attribute, Data, DeriveInput, Expr, Fields, Ident, Lit, Meta, Token, Type,
};

/// Access permission parsed from `#[reg(...)]` attribute.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Access {
    RO,
    WO,
    RW,
    RWE,
    Reserved,
}

/// Parsed field information.
struct FieldInfo {
    name: Ident,
    ty: Type,
    access: Access,
    size: usize,
}

/// Parsed `#[regmap(...)]` attribute.
struct RegMapAttr {
    base: u16,
    fields_name: Option<String>,
}

impl Parse for RegMapAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut base = None;
        let mut fields_name = None;

        while !input.is_empty() {
            let ident: Ident = input.parse()?;
            input.parse::<Token![=]>()?;

            match ident.to_string().as_str() {
                "base" => {
                    let lit: Lit = input.parse()?;
                    if let Lit::Int(lit_int) = lit {
                        base = Some(lit_int.base10_parse()?);
                    } else {
                        return Err(syn::Error::new_spanned(lit, "expected integer literal"));
                    }
                }
                "fields" => {
                    let lit: Lit = input.parse()?;
                    if let Lit::Str(lit_str) = lit {
                        fields_name = Some(lit_str.value());
                    } else {
                        return Err(syn::Error::new_spanned(lit, "expected string literal"));
                    }
                }
                _ => return Err(syn::Error::new_spanned(ident, "unknown attribute")),
            }

            if input.peek(Token![,]) {
                input.parse::<Token![,]>()?;
            }
        }

        Ok(RegMapAttr {
            base: base.unwrap_or(0),
            fields_name,
        })
    }
}

/// Parse `#[reg(RO|WO|RW|RWE)]` attribute from a field.
fn parse_reg_attr(attrs: &[Attribute]) -> syn::Result<Option<Access>> {
    for attr in attrs {
        if attr.path().is_ident("reg") {
            let meta = &attr.meta;
            if let Meta::List(list) = meta {
                let tokens = list.tokens.clone();
                let ident: Ident = syn::parse2(tokens)?;
                let access = match ident.to_string().as_str() {
                    "RO" => Access::RO,
                    "WO" => Access::WO,
                    "RW" => Access::RW,
                    "RWE" => Access::RWE,
                    other => {
                        return Err(syn::Error::new_spanned(
                            ident,
                            format!("unknown access type: {}", other),
                        ))
                    }
                };
                return Ok(Some(access));
            }
        }
    }
    Ok(None) // No #[reg(...)] = Reserved
}

/// Get size of a type. Supports primitives and `[u8; N]` arrays.
fn type_size(ty: &Type) -> syn::Result<usize> {
    match ty {
        Type::Path(tp) => {
            let ident = tp.path.get_ident().map(|i| i.to_string());
            match ident.as_deref() {
                Some("u8") | Some("i8") | Some("bool") => Ok(1),
                Some("u16") | Some("i16") => Ok(2),
                Some("u32") | Some("i32") => Ok(4),
                Some("u64") | Some("i64") => Ok(8),
                // Unit types from open-servo-units
                Some("CentiC") | Some("CentiDeg") | Some("MilliVolt") | Some("MilliAmp") => Ok(2),
                Some("CentiDeg32") => Ok(4),
                _ => Err(syn::Error::new_spanned(
                    ty,
                    format!("unsupported type: {:?}", ident),
                )),
            }
        }
        Type::Array(arr) => {
            // [u8; N] - get N
            let elem_size = type_size(&arr.elem)?;
            if let Expr::Lit(lit) = &arr.len {
                if let Lit::Int(lit_int) = &lit.lit {
                    let len: usize = lit_int.base10_parse()?;
                    return Ok(elem_size * len);
                }
            }
            Err(syn::Error::new_spanned(
                &arr.len,
                "expected integer literal for array length",
            ))
        }
        _ => Err(syn::Error::new_spanned(ty, "unsupported type")),
    }
}

/// Get the encoding token for a type.
fn type_encoding(ty: &Type) -> TokenStream {
    match ty {
        Type::Path(tp) => {
            let ident = tp.path.get_ident().map(|i| i.to_string());
            match ident.as_deref() {
                Some("u8") => quote!(crate::spec::Encoding::U8),
                Some("i8") => quote!(crate::spec::Encoding::U8), // No I8 variant, use U8
                Some("bool") => quote!(crate::spec::Encoding::Bool),
                Some("u16") => quote!(crate::spec::Encoding::U16Le),
                Some("i16") => quote!(crate::spec::Encoding::I16Le),
                Some("u32") => quote!(crate::spec::Encoding::U32Le),
                Some("i32") => quote!(crate::spec::Encoding::I32Le),
                Some("u64") => quote!(crate::spec::Encoding::U32Le), // No U64, fallback
                Some("i64") => quote!(crate::spec::Encoding::I32Le), // No I64, fallback
                // Unit types
                Some("CentiC") | Some("CentiDeg") | Some("MilliVolt") | Some("MilliAmp") => {
                    quote!(crate::spec::Encoding::I16Le)
                }
                Some("CentiDeg32") => quote!(crate::spec::Encoding::I32Le),
                _ => quote!(crate::spec::Encoding::U8), // Fallback
            }
        }
        Type::Array(_) => quote!(crate::spec::Encoding::U8), // Reserved arrays
        _ => quote!(crate::spec::Encoding::U8),
    }
}

/// Get the Rust primitive type token for use in Reg<A, T>.
fn type_primitive(ty: &Type) -> Option<TokenStream> {
    match ty {
        Type::Path(tp) => {
            let ident = tp.path.get_ident().map(|i| i.to_string());
            match ident.as_deref() {
                Some("u8") => Some(quote!(u8)),
                Some("i8") => Some(quote!(i8)),
                Some("bool") => Some(quote!(bool)),
                Some("u16") => Some(quote!(u16)),
                Some("i16") => Some(quote!(i16)),
                Some("u32") => Some(quote!(u32)),
                Some("i32") => Some(quote!(i32)),
                // Unit types map to their backing primitives
                Some("CentiC") | Some("CentiDeg") | Some("MilliVolt") | Some("MilliAmp") => {
                    Some(quote!(i16))
                }
                Some("CentiDeg32") => Some(quote!(i32)),
                _ => None,
            }
        }
        Type::Array(_) => None, // Reserved arrays don't get Reg constants
        _ => None,
    }
}

pub fn impl_regmap(input: &DeriveInput) -> syn::Result<TokenStream> {
    // Parse #[regmap(...)] attribute
    let regmap_attr = input
        .attrs
        .iter()
        .find(|a| a.path().is_ident("regmap"))
        .map(|a| a.parse_args::<RegMapAttr>())
        .transpose()?
        .unwrap_or(RegMapAttr {
            base: 0,
            fields_name: None,
        });

    let base_addr = regmap_attr.base;
    let struct_name = &input.ident;

    // Default fields name: STRUCTNAME_FIELDS (e.g., EEPROM_REGS_FIELDS)
    let fields_name = regmap_attr
        .fields_name
        .map(|s| format_ident!("{}", s))
        .unwrap_or_else(|| {
            let name = struct_name.to_string().to_uppercase();
            format_ident!("{}_FIELDS", name)
        });

    // Get struct fields
    let fields = match &input.data {
        Data::Struct(s) => match &s.fields {
            Fields::Named(named) => &named.named,
            _ => return Err(syn::Error::new_spanned(input, "expected named fields")),
        },
        _ => return Err(syn::Error::new_spanned(input, "expected struct")),
    };

    // Parse all fields
    let mut field_infos = Vec::new();
    for field in fields {
        let name = field.ident.clone().unwrap();
        let ty = field.ty.clone();
        let access = parse_reg_attr(&field.attrs)?.unwrap_or(Access::Reserved);
        let size = type_size(&ty)?;

        field_infos.push(FieldInfo {
            name,
            ty,
            access,
            size,
        });
    }

    // Compute addresses
    let mut addr_consts = Vec::new();
    let mut reg_consts = Vec::new();
    let mut field_specs = Vec::new();
    let mut current_addr = base_addr;

    for info in &field_infos {
        let addr = current_addr;
        let name_upper = format_ident!("{}", info.name.to_string().to_uppercase());
        let name_str = info.name.to_string();
        let encoding = type_encoding(&info.ty);
        let size = info.size as u8;

        // Generate address constant (skip reserved fields)
        if info.access != Access::Reserved {
            addr_consts.push(quote! {
                pub const #name_upper: u16 = #addr;
            });

            // Generate typed Reg constant
            if let Some(prim_ty) = type_primitive(&info.ty) {
                let access_marker = match info.access {
                    Access::RO => quote!(crate::reg::RO),
                    Access::WO => quote!(crate::reg::WO),
                    Access::RW => quote!(crate::reg::RW),
                    Access::RWE => quote!(crate::reg::RWE),
                    Access::Reserved => unreachable!(),
                };
                reg_consts.push(quote! {
                    pub const #name_upper: crate::reg::Reg<#access_marker, #prim_ty> =
                        crate::reg::Reg::new(#addr);
                });
            }
        }

        // Generate field spec
        let access_token = match info.access {
            Access::RO => quote!(crate::spec::Access::RO),
            Access::WO => quote!(crate::spec::Access::WO),
            Access::RW => quote!(crate::spec::Access::RW),
            Access::RWE => quote!(crate::spec::Access::RWE),
            Access::Reserved => quote!(crate::spec::Access::Reserved),
        };

        if info.access == Access::Reserved {
            field_specs.push(quote! {
                crate::spec::RegSpec::reserved(#name_str, #addr, #size)
            });
        } else {
            field_specs.push(quote! {
                crate::spec::RegSpec::new(#name_str, #addr, #encoding, #access_token)
            });
        }

        current_addr += info.size as u16;
    }

    // Generate output
    Ok(quote! {
        /// Generated address constants.
        pub mod addr {
            #(#addr_consts)*
        }

        /// Generated typed register handles.
        pub mod reg {
            #(#reg_consts)*
        }

        /// Generated field specifications.
        pub const #fields_name: &[RegSpec] = &[
            #(#field_specs),*
        ];
    })
}
