use proc_macro::TokenStream;

#[proc_macro_derive(Block, attributes(ct_field, ct_block))]
pub fn derive_block(_input: TokenStream) -> TokenStream {
    TokenStream::new()
}

#[proc_macro_derive(Region, attributes(ct_region, ct_block))]
pub fn derive_region(_input: TokenStream) -> TokenStream {
    TokenStream::new()
}

#[proc_macro_derive(Table, attributes(ct_table))]
pub fn derive_table(_input: TokenStream) -> TokenStream {
    TokenStream::new()
}

#[proc_macro_derive(Enum, attributes(ct_enum))]
pub fn derive_enum(_input: TokenStream) -> TokenStream {
    TokenStream::new()
}
