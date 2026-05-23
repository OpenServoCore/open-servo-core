use proc_macro::TokenStream;
use syn::{DeriveInput, parse_macro_input};

mod block;
mod enums;

#[proc_macro_derive(Block, attributes(ct_field, ct_block))]
pub fn derive_block(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    block::expand(&input)
        .unwrap_or_else(syn::Error::into_compile_error)
        .into()
}

#[proc_macro_derive(Enum, attributes(ct_enum))]
pub fn derive_enum(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    enums::expand(&input)
        .unwrap_or_else(syn::Error::into_compile_error)
        .into()
}

#[proc_macro_derive(Region, attributes(ct_region, ct_block))]
pub fn derive_region(_input: TokenStream) -> TokenStream {
    TokenStream::new()
}

#[proc_macro_derive(Table, attributes(ct_table))]
pub fn derive_table(_input: TokenStream) -> TokenStream {
    TokenStream::new()
}
