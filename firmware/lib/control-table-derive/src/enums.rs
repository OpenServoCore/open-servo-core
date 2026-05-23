use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::spanned::Spanned;
use syn::{Attribute, Data, DeriveInput, Fields, Ident};

pub fn expand(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let enum_ty = &input.ident;

    check_repr_u8(&input.attrs, enum_ty.span())?;

    let Data::Enum(e) = &input.data else {
        return Err(syn::Error::new(
            input.span(),
            "Enum can only be derived for enums",
        ));
    };

    let mut variant_idents: Vec<&Ident> = Vec::new();
    for v in &e.variants {
        if !matches!(v.fields, Fields::Unit) {
            return Err(syn::Error::new(
                v.span(),
                "Enum derive requires unit variants only",
            ));
        }
        variant_idents.push(&v.ident);
    }

    Ok(quote! {
        impl #enum_ty {
            pub const ALLOWED: &'static [u8] = &[
                #(Self::#variant_idents as u8),*
            ];
        }

        impl ::control_table::HasAllowed for #enum_ty {
            const ALLOWED: &'static [u8] = <Self>::ALLOWED;
        }
    })
}

fn check_repr_u8(attrs: &[Attribute], enum_span: proc_macro2::Span) -> syn::Result<()> {
    let mut has_u8 = false;
    for a in attrs {
        if !a.path().is_ident("repr") {
            continue;
        }
        a.parse_nested_meta(|m| {
            if m.path.is_ident("u8") {
                has_u8 = true;
                Ok(())
            } else {
                Err(m.error("Enum derive requires #[repr(u8)]"))
            }
        })?;
    }
    if !has_u8 {
        return Err(syn::Error::new(
            enum_span,
            "Enum derive requires #[repr(u8)]",
        ));
    }
    Ok(())
}
