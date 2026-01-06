//! RegMap derive macro implementation.
//!
//! Supports:
//! - `#[reg(RO|RW)]` — simple access (RAM)
//! - `#[reg(RO, eeprom)]` or `#[reg(RW, eeprom)]` — EEPROM-backed field
//! - `#[reg(facade)]` — DXL facade alias (emits Access::Facade)
//! - `#[reg(access=RW, eeprom, facade=EXPR, codec=IDENT)]` — vendor with DXL alias
//! - `#[indirect_address(n)]` — indirect address bank
//! - `#[indirect_data(n)]` — indirect data bank
//!
//! Access is orthogonal to EEPROM persistence:
//! - `RO` = read-only (telemetry, immutable EEPROM like model_number)
//! - `RW` = read-write (writable when enabled; EEPROM requires torque=0)
//!
//! # EEPROM Address Convention
//!
//! EEPROM fields must be placed in one of two regions for bitmap optimization:
//! - **DXL EEPROM**: addresses 0-63
//! - **Vendor EEPROM**: addresses 512-575
//!
//! This allows O(1) `touches_eeprom()` checks using two 64-bit bitmaps.

use const_fnv1a_hash::fnv1a_hash_str_32 as fnv1a_hash;
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
    Facade,
    Reserved,
}

/// Facade info for vendor registers.
#[derive(Clone)]
struct FacadeInfo {
    dxl_addr: Expr,
    codec: Ident,
}

/// Indirect bank info.
#[derive(Clone)]
struct IndirectInfo {
    bank: u8,
    kind: IndirectKind,
    slots: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IndirectKind {
    Address,
    Data,
}

/// Parsed field information.
struct FieldInfo {
    name: Ident,
    ty: Type,
    access: Access,
    eeprom: bool,
    size: usize,
    facade: Option<FacadeInfo>,
    indirect: Option<IndirectInfo>,
    is_codec_ctx: bool,
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

/// Parsed reg attribute content.
struct RegAttrContent {
    access: Option<Access>,
    eeprom: bool,
    facade: Option<Expr>,
    codec: Option<Ident>,
    codec_ctx: bool,
}

impl Parse for RegAttrContent {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut access = None;
        let mut eeprom = false;
        let mut facade = None;
        let mut codec = None;
        let mut codec_ctx = false;

        // Try simple form first: #[reg(RO)] or #[reg(RO, eeprom)] or #[reg(facade)]
        if input.peek(Ident) && !input.peek2(Token![=]) {
            let ident: Ident = input.parse()?;
            let ident_str = ident.to_string();
            match ident_str.as_str() {
                "RO" => access = Some(Access::RO),
                "WO" => access = Some(Access::WO),
                "RW" => access = Some(Access::RW),
                "facade" => access = Some(Access::Facade),
                _ => {
                    return Err(syn::Error::new_spanned(
                        ident,
                        format!("unknown access type: {}", ident_str),
                    ))
                }
            }

            // Check for additional flags after access specifier
            while input.peek(Token![,]) {
                input.parse::<Token![,]>()?;
                if input.is_empty() {
                    break;
                }
                let flag: Ident = input.parse()?;
                let flag_str = flag.to_string();
                match flag_str.as_str() {
                    "eeprom" => eeprom = true,
                    "codec_ctx" => codec_ctx = true,
                    _ => {
                        return Err(syn::Error::new_spanned(
                            flag,
                            format!("unknown flag: {}", flag_str),
                        ))
                    }
                }
            }

            return Ok(RegAttrContent {
                access,
                eeprom,
                facade,
                codec,
                codec_ctx,
            });
        }

        // Key-value form: #[reg(access=RW, eeprom, facade=EXPR, codec=IDENT, codec_ctx)]
        while !input.is_empty() {
            let key: Ident = input.parse()?;

            // Check for bare flag (no =)
            if !input.peek(Token![=]) {
                match key.to_string().as_str() {
                    "eeprom" => eeprom = true,
                    "codec_ctx" => codec_ctx = true,
                    _ => return Err(syn::Error::new_spanned(key, "unknown flag")),
                }
                if input.peek(Token![,]) {
                    input.parse::<Token![,]>()?;
                }
                continue;
            }

            input.parse::<Token![=]>()?;

            match key.to_string().as_str() {
                "access" => {
                    let val: Ident = input.parse()?;
                    access = Some(match val.to_string().as_str() {
                        "RO" => Access::RO,
                        "WO" => Access::WO,
                        "RW" => Access::RW,
                        _ => return Err(syn::Error::new_spanned(val, "expected RO, WO, or RW")),
                    });
                }
                "facade" => {
                    facade = Some(input.parse()?);
                }
                "codec" => {
                    codec = Some(input.parse()?);
                }
                _ => return Err(syn::Error::new_spanned(key, "unknown reg attribute key")),
            }

            if input.peek(Token![,]) {
                input.parse::<Token![,]>()?;
            }
        }

        Ok(RegAttrContent {
            access,
            eeprom,
            facade,
            codec,
            codec_ctx,
        })
    }
}

/// Parsed result from #[reg(...)] attribute.
struct RegAttrResult {
    access: Access,
    eeprom: bool,
    facade: Option<FacadeInfo>,
    codec_ctx: bool,
}

/// Parse `#[reg(...)]` attribute from a field.
fn parse_reg_attr(attrs: &[Attribute]) -> syn::Result<Option<RegAttrResult>> {
    for attr in attrs {
        if attr.path().is_ident("reg") {
            let meta = &attr.meta;
            if let Meta::List(list) = meta {
                let content: RegAttrContent = syn::parse2(list.tokens.clone())?;

                let access = content.access.ok_or_else(|| {
                    syn::Error::new_spanned(&list.tokens, "missing access specifier")
                })?;

                // Validate facade ⇔ codec
                match (&content.facade, &content.codec) {
                    (Some(_), None) => {
                        return Err(syn::Error::new_spanned(
                            &list.tokens,
                            "facade requires codec",
                        ))
                    }
                    (None, Some(_)) => {
                        return Err(syn::Error::new_spanned(
                            &list.tokens,
                            "codec requires facade",
                        ))
                    }
                    (Some(dxl_addr), Some(codec)) => {
                        let facade_info = FacadeInfo {
                            dxl_addr: dxl_addr.clone(),
                            codec: codec.clone(),
                        };
                        return Ok(Some(RegAttrResult {
                            access,
                            eeprom: content.eeprom,
                            facade: Some(facade_info),
                            codec_ctx: content.codec_ctx,
                        }));
                    }
                    (None, None) => {
                        return Ok(Some(RegAttrResult {
                            access,
                            eeprom: content.eeprom,
                            facade: None,
                            codec_ctx: content.codec_ctx,
                        }));
                    }
                }
            }
        }
    }
    Ok(None)
}

/// Parse `#[indirect_address(n)]` or `#[indirect_data(n)]` attribute.
fn parse_indirect_attr(attrs: &[Attribute]) -> syn::Result<Option<(u8, IndirectKind)>> {
    for attr in attrs {
        let kind = if attr.path().is_ident("indirect_address") {
            IndirectKind::Address
        } else if attr.path().is_ident("indirect_data") {
            IndirectKind::Data
        } else {
            continue;
        };

        let meta = &attr.meta;
        if let Meta::List(list) = meta {
            let lit: Lit = syn::parse2(list.tokens.clone())?;
            if let Lit::Int(lit_int) = lit {
                let bank: u8 = lit_int.base10_parse()?;
                return Ok(Some((bank, kind)));
            } else {
                return Err(syn::Error::new_spanned(lit, "expected integer literal"));
            }
        }
    }
    Ok(None)
}

/// Get size of a type. Supports primitives and `[u8; N]` / `[u16; N]` arrays.
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

/// Get array element count for indirect banks.
fn array_len(ty: &Type) -> syn::Result<usize> {
    match ty {
        Type::Array(arr) => {
            if let Expr::Lit(lit) = &arr.len {
                if let Lit::Int(lit_int) = &lit.lit {
                    return lit_int.base10_parse();
                }
            }
            Err(syn::Error::new_spanned(
                &arr.len,
                "expected integer literal for array length",
            ))
        }
        _ => Err(syn::Error::new_spanned(ty, "expected array type")),
    }
}

/// Validate indirect array element type.
fn validate_indirect_type(ty: &Type, kind: IndirectKind) -> syn::Result<()> {
    if let Type::Array(arr) = ty {
        if let Type::Path(tp) = arr.elem.as_ref() {
            let ident = tp.path.get_ident().map(|i| i.to_string());
            let expected = match kind {
                IndirectKind::Address => "u16",
                IndirectKind::Data => "u8",
            };
            if ident.as_deref() == Some(expected) {
                return Ok(());
            }
            return Err(syn::Error::new_spanned(
                &arr.elem,
                format!(
                    "indirect_{} requires [{}; N], got {:?}",
                    match kind {
                        IndirectKind::Address => "address",
                        IndirectKind::Data => "data",
                    },
                    expected,
                    ident
                ),
            ));
        }
    }
    Err(syn::Error::new_spanned(
        ty,
        "indirect attribute requires array type",
    ))
}

/// Get the encoding token for a type.
fn type_encoding(ty: &Type) -> TokenStream {
    match ty {
        Type::Path(tp) => {
            let ident = tp.path.get_ident().map(|i| i.to_string());
            match ident.as_deref() {
                Some("u8") => quote!(crate::spec::Encoding::U8),
                Some("i8") => quote!(crate::spec::Encoding::U8),
                Some("bool") => quote!(crate::spec::Encoding::Bool),
                Some("u16") => quote!(crate::spec::Encoding::U16Le),
                Some("i16") => quote!(crate::spec::Encoding::I16Le),
                Some("u32") => quote!(crate::spec::Encoding::U32Le),
                Some("i32") => quote!(crate::spec::Encoding::I32Le),
                Some("u64") => quote!(crate::spec::Encoding::U32Le),
                Some("i64") => quote!(crate::spec::Encoding::I32Le),
                Some("CentiC") | Some("CentiDeg") | Some("MilliVolt") | Some("MilliAmp") => {
                    quote!(crate::spec::Encoding::I16Le)
                }
                Some("CentiDeg32") => quote!(crate::spec::Encoding::I32Le),
                _ => quote!(crate::spec::Encoding::U8),
            }
        }
        Type::Array(_) => quote!(crate::spec::Encoding::U8),
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
                Some("CentiC") | Some("CentiDeg") | Some("MilliVolt") | Some("MilliAmp") => {
                    Some(quote!(i16))
                }
                Some("CentiDeg32") => Some(quote!(i32)),
                _ => None,
            }
        }
        Type::Array(_) => None,
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
    let is_vendor = base_addr >= 512;
    let is_dxl = base_addr < 252;

    // Default fields name
    let fields_name = regmap_attr
        .fields_name
        .as_ref()
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

        // Check for indirect attribute first
        if let Some((bank, kind)) = parse_indirect_attr(&field.attrs)? {
            validate_indirect_type(&ty, kind)?;
            let slots = array_len(&ty)?;
            let size = type_size(&ty)?;
            field_infos.push(FieldInfo {
                name,
                ty,
                access: Access::RW, // Indirect registers are RW
                eeprom: false,      // Indirect registers are not EEPROM
                size,
                facade: None,
                indirect: Some(IndirectInfo { bank, kind, slots }),
                is_codec_ctx: false,
            });
            continue;
        }

        // Check for reg attribute
        let reg_result = parse_reg_attr(&field.attrs)?;
        let (access, eeprom, facade, codec_ctx) = match reg_result {
            Some(r) => (r.access, r.eeprom, r.facade, r.codec_ctx),
            None => (Access::Reserved, false, None, false),
        };
        let size = type_size(&ty)?;

        // Validate facade placement
        if facade.is_some() && !is_vendor {
            return Err(syn::Error::new_spanned(
                &field.ident,
                "facade attribute only allowed on vendor registers (base >= 512)",
            ));
        }
        if access == Access::Facade && !is_dxl {
            return Err(syn::Error::new_spanned(
                &field.ident,
                "#[reg(facade)] only allowed on DXL registers (base < 252)",
            ));
        }
        // Validate codec_ctx placement
        if codec_ctx && !is_vendor {
            return Err(syn::Error::new_spanned(
                &field.ident,
                "codec_ctx only allowed on vendor registers (base >= 512)",
            ));
        }

        field_infos.push(FieldInfo {
            name,
            ty,
            access,
            eeprom,
            size,
            facade,
            indirect: None,
            is_codec_ctx: codec_ctx,
        });
    }

    // Validate indirect bank pairing
    let mut indirect_banks: std::collections::HashMap<
        u8,
        (Option<(u16, usize)>, Option<(u16, usize)>),
    > = std::collections::HashMap::new();
    let mut current_addr = base_addr;
    for info in &field_infos {
        let addr = current_addr;
        if let Some(indirect) = &info.indirect {
            let entry = indirect_banks.entry(indirect.bank).or_insert((None, None));
            match indirect.kind {
                IndirectKind::Address => entry.0 = Some((addr, indirect.slots)),
                IndirectKind::Data => entry.1 = Some((addr, indirect.slots)),
            }
        }
        current_addr += info.size as u16;
    }
    for (bank, (addr_info, data_info)) in &indirect_banks {
        match (addr_info, data_info) {
            (Some((_, addr_slots)), Some((_, data_slots))) => {
                if addr_slots != data_slots {
                    return Err(syn::Error::new(
                        proc_macro2::Span::call_site(),
                        format!(
                            "indirect bank {} has mismatched slots: address={}, data={}",
                            bank, addr_slots, data_slots
                        ),
                    ));
                }
            }
            (Some(_), None) => {
                return Err(syn::Error::new(
                    proc_macro2::Span::call_site(),
                    format!("indirect bank {} has address but no data", bank),
                ));
            }
            (None, Some(_)) => {
                return Err(syn::Error::new(
                    proc_macro2::Span::call_site(),
                    format!("indirect bank {} has data but no address", bank),
                ));
            }
            (None, None) => {}
        }
    }

    // Compute addresses and generate output
    let mut addr_consts = Vec::new();
    let mut reg_consts = Vec::new();
    let mut field_specs = Vec::new();
    let mut eeprom_field_specs = Vec::new(); // EEPROM fields only
    let mut eeprom_info: Vec<(u32, u16, u8)> = Vec::new(); // (name_hash, addr, size) for map builder
    let mut facade_entries = Vec::new();
    let mut indirect_bank_specs = Vec::new();
    // Collect codec_ctx fields for CodecCtx struct generation
    let mut codec_ctx_fields: Vec<(Ident, Type, u16, usize)> = Vec::new();
    current_addr = base_addr;

    for info in &field_infos {
        let addr = current_addr;
        let name_upper = format_ident!("{}", info.name.to_string().to_uppercase());
        let name_str = info.name.to_string();
        let encoding = type_encoding(&info.ty);
        let size = info.size as u8;

        // Handle indirect fields
        if let Some(indirect) = &info.indirect {
            // Generate address constant for indirect arrays
            addr_consts.push(quote! {
                pub const #name_upper: u16 = #addr;
            });

            // Generate field spec (indirect arrays are RW, not EEPROM)
            field_specs.push(quote! {
                crate::spec::RegSpec::new(#name_str, #addr, #encoding, crate::spec::Access::RW, false)
            });

            // Generate indirect bank spec (only once per bank, from address side)
            if indirect.kind == IndirectKind::Address {
                if let Some((_, Some((data_base, _)))) = indirect_banks.get(&indirect.bank) {
                    let bank = indirect.bank;
                    let slots = indirect.slots as u8;
                    let data_base = *data_base;
                    indirect_bank_specs.push(quote! {
                        crate::spec::IndirectBankSpec {
                            bank: #bank,
                            addr_base: #addr,
                            data_base: #data_base,
                            slots: #slots,
                        }
                    });
                }
            }

            current_addr += info.size as u16;
            continue;
        }

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
                    Access::Facade => quote!(crate::reg::Facade),
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
            Access::Facade => quote!(crate::spec::Access::Facade),
            Access::Reserved => quote!(crate::spec::Access::Reserved),
        };

        if info.access == Access::Reserved {
            field_specs.push(quote! {
                crate::spec::RegSpec::reserved(#name_str, #addr, #size)
            });
        } else {
            let eeprom = info.eeprom;
            let spec = quote! {
                crate::spec::RegSpec::new(#name_str, #addr, #encoding, #access_token, #eeprom)
            };
            field_specs.push(spec.clone());

            // Collect EEPROM fields for persistence (RW + eeprom only).
            // RO + eeprom fields (like model_number) come from board config
            // and should NOT be persisted to flash.
            if info.eeprom && matches!(info.access, Access::RW) {
                eeprom_field_specs.push(spec);
                // Track for bitmap and hash map generation
                // Compute FNV-1a hash at compile time
                let hash = fnv1a_hash(&name_str);
                eeprom_info.push((hash, addr, size));
            }
        }

        // Generate facade mapping entry (vendor registers only)
        if let Some(facade) = &info.facade {
            let dxl_addr = &facade.dxl_addr;
            let codec = &facade.codec;
            let vendor_len = size;
            // Direction inferred from access
            let direction = match info.access {
                Access::RO => quote!(crate::spec::MapDirection::ToFacade),
                _ => quote!(crate::spec::MapDirection::Both),
            };

            facade_entries.push(quote! {
                crate::spec::MapEntry {
                    name: #name_str,
                    dxl_addr: #dxl_addr,
                    dxl_len: #vendor_len, // Assume same length for now
                    vendor_addr: #addr,
                    vendor_len: #vendor_len,
                    direction: #direction,
                    codec: crate::spec::CodecKind::#codec,
                }
            });
        }

        // Collect codec_ctx fields
        if info.is_codec_ctx {
            codec_ctx_fields.push((info.name.clone(), info.ty.clone(), addr, info.size));
        }

        current_addr += info.size as u16;
    }

    // Generate EEPROM fields name
    let eeprom_fields_name = format_ident!(
        "{}_EEPROM",
        regmap_attr
            .fields_name
            .as_ref()
            .map(|s| s.as_str())
            .unwrap_or_else(|| {
                // Can't call to_uppercase on &str in format_ident context
                "FIELDS"
            })
    );

    // Generate output
    let mut output = quote! {
        /// Generated address constants.
        pub mod addr {
            #(#addr_consts)*
        }

        /// Generated typed register handles.
        pub mod reg {
            #(#reg_consts)*
        }

        /// Generated field specifications.
        pub const #fields_name: &[crate::spec::RegSpec] = &[
            #(#field_specs),*
        ];

        /// Generated EEPROM field specifications for persistence.
        pub const #eeprom_fields_name: &[crate::spec::RegSpec] = &[
            #(#eeprom_field_specs),*
        ];
    };

    // Generate FACADE_MAPPINGS for vendor regmaps
    if is_vendor && !facade_entries.is_empty() {
        let count = facade_entries.len();
        output.extend(quote! {
            /// Generated facade↔vendor mapping table.
            pub const FACADE_MAPPINGS: &[crate::spec::MapEntry] = &[
                #(#facade_entries),*
            ];

            /// Number of facade mappings.
            pub const FACADE_MAPPINGS_COUNT: usize = #count;
        });
    }

    // Generate INDIRECT_BANKS for DXL regmaps
    if is_dxl && !indirect_bank_specs.is_empty() {
        output.extend(quote! {
            /// Generated indirect bank specifications.
            pub const INDIRECT_BANKS: &[crate::spec::IndirectBankSpec] = &[
                #(#indirect_bank_specs),*
            ];
        });
    }

    // Generate CodecCtx for vendor regmaps with codec_ctx fields
    if is_vendor && !codec_ctx_fields.is_empty() {
        // Generate struct fields
        let struct_fields: Vec<_> = codec_ctx_fields
            .iter()
            .filter_map(|(name, ty, _, _)| {
                let prim = type_primitive(ty)?;
                Some(quote! { pub #name: #prim })
            })
            .collect();

        // Generate from_view reads
        let read_stmts: Vec<_> = codec_ctx_fields
            .iter()
            .filter_map(|(name, ty, _, size)| {
                let prim = type_primitive(ty)?;
                let addr_const = format_ident!("{}", name.to_string().to_uppercase());
                let read_stmt = match *size {
                    1 => quote! {
                        let mut buf = [0u8; 1];
                        view.read(addr::#addr_const, &mut buf).expect("codec_ctx read");
                        let #name = buf[0] as #prim;
                    },
                    2 => quote! {
                        let mut buf = [0u8; 2];
                        view.read(addr::#addr_const, &mut buf).expect("codec_ctx read");
                        let #name = #prim::from_le_bytes(buf);
                    },
                    4 => quote! {
                        let mut buf = [0u8; 4];
                        view.read(addr::#addr_const, &mut buf).expect("codec_ctx read");
                        let #name = #prim::from_le_bytes(buf);
                    },
                    _ => return None,
                };
                Some(read_stmt)
            })
            .collect();

        // Generate field initializers for Self
        let field_inits: Vec<_> = codec_ctx_fields
            .iter()
            .map(|(name, _, _, _)| quote! { #name })
            .collect();

        output.extend(quote! {
            /// Context required by codecs for position translation.
            #[derive(Debug, Clone, Copy)]
            pub struct CodecCtx {
                #(#struct_fields),*
            }

            impl CodecCtx {
                /// Read codec context fields from shadow view.
                pub fn from_view(view: &open_servo_shadow::KernelView<'_>) -> Self {
                    #(#read_stmts)*
                    Self { #(#field_inits),* }
                }
            }
        });
    }

    // Generate EEPROM bitmaps and hash map builder for DXL regmaps
    if is_dxl && !eeprom_info.is_empty() {
        // Compute bitmaps: DXL (0-63) and Vendor (512-575)
        let mut bitmap_dxl: u64 = 0;
        let mut bitmap_vendor: u64 = 0;

        for &(_, addr, size) in &eeprom_info {
            for offset in 0..size {
                let a = addr + offset as u16;
                if a < 64 {
                    bitmap_dxl |= 1u64 << a;
                } else if a >= 512 && a < 576 {
                    bitmap_vendor |= 1u64 << (a - 512);
                } else {
                    // Validate: EEPROM must be in valid range
                    return Err(syn::Error::new(
                        proc_macro2::Span::call_site(),
                        format!(
                            "EEPROM field at address {} is outside valid ranges (0-63 or 512-575)",
                            addr
                        ),
                    ));
                }
            }
        }

        let eeprom_count = eeprom_info.len();
        // heapless::FnvIndexMap requires power-of-2 capacity
        let eeprom_capacity = eeprom_count.next_power_of_two();

        // Generate hash map insert statements
        let map_inserts: Vec<_> = eeprom_info
            .iter()
            .enumerate()
            .map(|(idx, &(hash, _, _))| {
                quote! {
                    let _ = map.insert(#hash, #idx);
                }
            })
            .collect();

        output.extend(quote! {
            /// Bitmap of DXL EEPROM addresses (0-63).
            ///
            /// Bit N is set if address N is part of an EEPROM field.
            /// Used by `touches_eeprom()` for O(1) range checks.
            pub const EEPROM_BITMAP_DXL: u64 = #bitmap_dxl;

            /// Bitmap of Vendor EEPROM addresses (512-575).
            ///
            /// Bit N is set if address (512 + N) is part of an EEPROM field.
            /// Used by `touches_eeprom()` for O(1) range checks.
            pub const EEPROM_BITMAP_VENDOR: u64 = #bitmap_vendor;

            /// Number of EEPROM fields.
            pub const EEPROM_COUNT: usize = #eeprom_count;

            /// Capacity for EEPROM hash map (power of 2, >= EEPROM_COUNT).
            pub const EEPROM_CAPACITY: usize = #eeprom_capacity;

            /// Build a hash map from field name_hash to index in EEPROM_FIELDS.
            ///
            /// Call once during system initialization. Returns a map for O(1) lookup.
            pub fn build_eeprom_hash_map() -> heapless::FnvIndexMap<u32, usize, #eeprom_capacity> {
                let mut map = heapless::FnvIndexMap::new();
                #(#map_inserts)*
                map
            }
        });
    }

    Ok(output)
}
