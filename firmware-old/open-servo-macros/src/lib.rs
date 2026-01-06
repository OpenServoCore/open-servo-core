//! Proc-macros for open-servo firmware.
//!
//! # RegMap
//!
//! Derive macro for type-safe register map definitions.
//!
//! ```ignore
//! use open_servo_macros::RegMap;
//!
//! #[derive(RegMap)]
//! #[regmap(base = 0, fields = "DXL_FIELDS")]
//! struct DxlRegs {
//!     #[reg(RO, eeprom)]
//!     model_number: u16,
//!     #[reg(RW, eeprom)]
//!     id: u8,
//!     _reserved1: [u8; 6],  // No attr = Reserved
//! }
//! ```
//!
//! Generates:
//! - `pub mod addr { pub const MODEL_NUMBER: u16 = 0; ... }`
//! - `pub mod reg { pub const MODEL_NUMBER: Reg<RO, u16> = Reg::new(0); ... }`
//! - `pub const EEPROM_FIELDS: &[RegSpec] = &[...];`
//!
//! # AdcChannels
//!
//! Derive macro for feature-gated ADC channel configuration.
//!
//! ```ignore
//! use open_servo_macros::AdcChannels;
//!
//! #[derive(AdcChannels)]
//! #[adc_channels(buffer = "AdcBuffer", count = "ADC_CHANNEL_COUNT")]
//! pub struct Channels<'a> {
//!     buf: &'a [u16],
//!     #[channel(name = "VREFINT")]
//!     _vrefint: (),
//!     #[cfg(feature = "current-sense-bus")]
//!     #[channel(name = "CURRENT")]
//!     _current: (),
//! }
//! ```
//!
//! Generates:
//! - `pub const ADC_CHANNEL_COUNT: usize = ...;`
//! - `pub type AdcBuffer = [u16; ADC_CHANNEL_COUNT];`
//! - `pub mod idx { pub const VREFINT: usize = 0; ... }`
//! - `impl Channels { pub fn new(...) -> Self; pub fn vrefint(&self) -> u16; ... }`

mod adc_channels;
mod regmap;

use proc_macro::TokenStream;
use syn::{parse_macro_input, DeriveInput};

/// Derive macro for register map definitions.
///
/// Supports the following field attributes:
/// - `#[reg(RO|RW)]` - Standalone register with specified access
/// - `#[reg(RO, eeprom)]` or `#[reg(RW, eeprom)]` - EEPROM-backed field
/// - `#[reg(facade)]` - DXL facade alias (emits `Access::Facade`)
/// - `#[reg(access=X, eeprom, facade=EXPR, codec=IDENT)]` - Vendor register with DXL alias
/// - `#[indirect_address(n)]` - Indirect address bank (requires `[u16; N]`)
/// - `#[indirect_data(n)]` - Indirect data bank (requires `[u8; N]`)
///
/// For vendor regmaps (base >= 512) with facade attrs, generates:
/// - `pub const FACADE_MAPPINGS: &[MapEntry] = &[...];`
/// - `pub const FACADE_MAPPINGS_COUNT: usize = N;`
///
/// For DXL regmaps (base < 252) with indirect attrs, generates:
/// - `pub const INDIRECT_BANKS: &[IndirectBankSpec] = &[...];`
#[proc_macro_derive(RegMap, attributes(regmap, reg, indirect_address, indirect_data))]
pub fn derive_regmap(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);

    match regmap::impl_regmap(&input) {
        Ok(tokens) => tokens.into(),
        Err(e) => e.to_compile_error().into(),
    }
}

/// Derive macro for ADC channel configuration.
///
/// Generates feature-gated channel index constants and typed accessors.
#[proc_macro_derive(AdcChannels, attributes(adc_channels, channel))]
pub fn derive_adc_channels(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);

    match adc_channels::impl_adc_channels(&input) {
        Ok(tokens) => tokens.into(),
        Err(e) => e.to_compile_error().into(),
    }
}
