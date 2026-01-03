//! FacadeMap derive macro implementation.
//!
//! Generates mapping tables for facade↔vendor register translation.

use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{
    parse::{Parse, ParseStream},
    Attribute, Data, DeriveInput, Expr, Fields, Ident, Lit, Meta, Token,
};

/// Parsed `#[facademap(...)]` container attribute.
struct FacadeMapAttr {
    table_name: String,
}

impl Parse for FacadeMapAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut table_name = None;

        while !input.is_empty() {
            let ident: Ident = input.parse()?;
            input.parse::<Token![=]>()?;

            match ident.to_string().as_str() {
                "table" => {
                    let lit: Lit = input.parse()?;
                    if let Lit::Str(lit_str) = lit {
                        table_name = Some(lit_str.value());
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

        Ok(FacadeMapAttr {
            table_name: table_name.unwrap_or_else(|| "FACADE_MAPPINGS".to_string()),
        })
    }
}

/// Parsed `#[map(...)]` field attribute.
struct MapAttr {
    src: Expr,
    src_len: u8,
    dst: Expr,
    dst_len: u8,
    dir: Ident,
    codec: Ident,
}

impl Parse for MapAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut src: Option<Expr> = None;
        let mut src_len: Option<u8> = None;
        let mut dst: Option<Expr> = None;
        let mut dst_len: Option<u8> = None;
        let mut dir: Option<Ident> = None;
        let mut codec: Option<Ident> = None;

        while !input.is_empty() {
            let ident: Ident = input.parse()?;
            input.parse::<Token![=]>()?;

            match ident.to_string().as_str() {
                "src" => {
                    src = Some(input.parse()?);
                }
                "src_len" => {
                    let lit: Lit = input.parse()?;
                    if let Lit::Int(lit_int) = lit {
                        src_len = Some(lit_int.base10_parse()?);
                    } else {
                        return Err(syn::Error::new_spanned(lit, "expected integer literal"));
                    }
                }
                "dst" => {
                    dst = Some(input.parse()?);
                }
                "dst_len" => {
                    let lit: Lit = input.parse()?;
                    if let Lit::Int(lit_int) = lit {
                        dst_len = Some(lit_int.base10_parse()?);
                    } else {
                        return Err(syn::Error::new_spanned(lit, "expected integer literal"));
                    }
                }
                "dir" => {
                    dir = Some(input.parse()?);
                }
                "codec" => {
                    codec = Some(input.parse()?);
                }
                _ => return Err(syn::Error::new_spanned(ident, "unknown map attribute")),
            }

            if input.peek(Token![,]) {
                input.parse::<Token![,]>()?;
            }
        }

        Ok(MapAttr {
            src: src.ok_or_else(|| input.error("missing `src` attribute"))?,
            src_len: src_len.ok_or_else(|| input.error("missing `src_len` attribute"))?,
            dst: dst.ok_or_else(|| input.error("missing `dst` attribute"))?,
            dst_len: dst_len.ok_or_else(|| input.error("missing `dst_len` attribute"))?,
            dir: dir.ok_or_else(|| input.error("missing `dir` attribute"))?,
            codec: codec.ok_or_else(|| input.error("missing `codec` attribute"))?,
        })
    }
}

/// Parse `#[map(...)]` attribute from a field.
fn parse_map_attr(attrs: &[Attribute]) -> syn::Result<Option<MapAttr>> {
    for attr in attrs {
        if attr.path().is_ident("map") {
            let meta = &attr.meta;
            if let Meta::List(list) = meta {
                let parsed: MapAttr = syn::parse2(list.tokens.clone())?;
                return Ok(Some(parsed));
            }
        }
    }
    Ok(None)
}

/// Parsed field with its mapping.
struct FieldMapping {
    name: Ident,
    map: MapAttr,
}

pub fn impl_facademap(input: &DeriveInput) -> syn::Result<TokenStream> {
    // Parse #[facademap(...)] attribute
    let facademap_attr = input
        .attrs
        .iter()
        .find(|a| a.path().is_ident("facademap"))
        .map(|a| a.parse_args::<FacadeMapAttr>())
        .transpose()?
        .unwrap_or(FacadeMapAttr {
            table_name: "FACADE_MAPPINGS".to_string(),
        });

    let table_name = format_ident!("{}", facademap_attr.table_name);
    let count_name = format_ident!("{}_COUNT", facademap_attr.table_name);

    // Get struct fields
    let fields = match &input.data {
        Data::Struct(s) => match &s.fields {
            Fields::Named(named) => &named.named,
            _ => return Err(syn::Error::new_spanned(input, "expected named fields")),
        },
        _ => return Err(syn::Error::new_spanned(input, "expected struct")),
    };

    // Parse all field mappings
    let mut mappings = Vec::new();
    for field in fields {
        let name = field.ident.clone().unwrap();
        if let Some(map) = parse_map_attr(&field.attrs)? {
            mappings.push(FieldMapping { name, map });
        }
    }

    let count = mappings.len();

    // Generate MapEntry initializers
    let entries: Vec<TokenStream> = mappings
        .iter()
        .map(|m| {
            let name_str = m.name.to_string();
            let src = &m.map.src;
            let src_len = m.map.src_len;
            let dst = &m.map.dst;
            let dst_len = m.map.dst_len;
            let dir = &m.map.dir;
            let codec = &m.map.codec;

            quote! {
                MapEntry {
                    name: #name_str,
                    src_addr: #src,
                    src_len: #src_len,
                    dst_addr: #dst,
                    dst_len: #dst_len,
                    direction: MapDirection::#dir,
                    codec: CodecKind::#codec,
                }
            }
        })
        .collect();

    // Generate output
    Ok(quote! {
        /// Generated mapping table.
        pub const #table_name: &[MapEntry] = &[
            #(#entries),*
        ];

        /// Number of entries in the mapping table.
        pub const #count_name: usize = #count;
    })
}
