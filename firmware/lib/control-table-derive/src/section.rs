use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::spanned::Spanned;
use syn::{Attribute, Data, DeriveInput, Expr, Fields, Ident, Path, Type, TypePath};

#[derive(Default)]
struct SectionAttrs {
    base: Option<Expr>,
    size: Option<Expr>,
    write_locked_by: Option<Expr>,
    hooks: Option<Path>,
}

pub fn expand(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let struct_ty = &input.ident;

    check_repr(&input.attrs, struct_ty.span())?;

    let Data::Struct(s) = &input.data else {
        return Err(syn::Error::new(
            input.span(),
            "Section can only be derived for structs",
        ));
    };
    let Fields::Named(fields) = &s.fields else {
        return Err(syn::Error::new(
            input.span(),
            "Section requires a struct with named fields",
        ));
    };

    let attrs = parse_section_attrs(&input.attrs)?;
    let base = attrs.base.ok_or_else(|| {
        syn::Error::new(
            struct_ty.span(),
            "Section requires `#[ct_section(base = ...)]`",
        )
    })?;
    let size = attrs.size.ok_or_else(|| {
        syn::Error::new(
            struct_ty.span(),
            "Section requires `#[ct_section(size = ...)]`",
        )
    })?;
    let hooks_bound = attrs.hooks;

    let mut size_terms: Vec<TokenStream2> = Vec::new();
    let mut new_inits: Vec<TokenStream2> = Vec::new();

    let mut block_tys: Vec<&Type> = Vec::new();
    let mut block_idents: Vec<&Ident> = Vec::new();
    let mut block_ty_idents: Vec<&Ident> = Vec::new();

    for field in &fields.named {
        let name = field.ident.as_ref().unwrap();
        let ty = &field.ty;

        let init = crate::block::default_init_for_type(ty);
        new_inits.push(quote!(#name: #init));
        size_terms.push(quote!(::core::mem::size_of::<#ty>()));

        if parse_field_skip(&field.attrs)? {
            continue;
        }
        block_ty_idents.push(block_type_ident(ty)?);
        block_tys.push(ty);
        block_idents.push(name);
    }

    let base_exprs: Vec<TokenStream2> = block_idents
        .iter()
        .map(|name| quote!(#struct_ty::BASE + ::core::mem::offset_of!(#struct_ty, #name) as u16))
        .collect();

    let n_writable = quote!(0 #(+ <#block_tys>::CT_WRITABLE.len())*);

    // Blocks in declaration order (= address order): preserves the old
    // sorted-by-offset first-failure precedence. Each call is gated on the
    // block's byte range so a write that misses a block skips its whole
    // field-guard walk (the compiled checks have no interpreter-style
    // sorted-scan early exit; this is its replacement).
    let check_calls: Vec<TokenStream2> = block_tys
        .iter()
        .zip(&base_exprs)
        .map(|(ty, base_expr)| {
            quote! {
                {
                    let b_lo = (#base_expr) as usize;
                    let b_hi = b_lo + ::core::mem::size_of::<#ty>();
                    if b_lo < hi && b_hi > lo {
                        <#ty>::ct_check(view, lo, hi, #base_expr)?;
                    }
                }
            }
        })
        .collect();

    let check_args = if check_calls.is_empty() {
        quote!(_view: &::control_table::View, _lo: usize, _hi: usize)
    } else {
        quote!(view: &::control_table::View, lo: usize, hi: usize)
    };

    let writable_copy: Vec<TokenStream2> = block_tys
        .iter()
        .zip(&base_exprs)
        .map(|(ty, base_expr)| {
            quote! {
                {
                    let src = <#ty>::CT_WRITABLE;
                    let base = #base_expr;
                    let mut i = 0;
                    while i < src.len() {
                        out[__n] = (src[i].0 + base, src[i].1);
                        __n += 1;
                        i += 1;
                    }
                }
            }
        })
        .collect();

    let write_lock = match &attrs.write_locked_by {
        Some(expr) => quote!(::core::option::Option::Some(#expr)),
        None => quote!(::core::option::Option::None),
    };

    let addr_mods: Vec<TokenStream2> = block_idents
        .iter()
        .zip(&block_ty_idents)
        .map(|(name, block_ty_ident)| {
            let meta_macro =
                Ident::new(&flat_meta_macro_name(block_ty_ident), block_ty_ident.span());
            quote! {
                #[allow(dead_code)]
                pub mod #name {
                    use super::super::{#struct_ty, #block_ty_ident};
                    #meta_macro!(@addr_consts
                        base = (#struct_ty::BASE
                            + ::core::mem::offset_of!(#struct_ty, #name) as u16),
                        block_ty = #block_ty_ident);
                }
            }
        })
        .collect();

    let dispatch_calls: Vec<TokenStream2> = block_idents
        .iter()
        .zip(&base_exprs)
        .map(|(name, base_expr)| {
            quote! {
                self.#name.dispatch_events(abs_addr, len, #base_expr, hooks);
            }
        })
        .collect();

    let where_clause = match &hooks_bound {
        Some(path) => quote!(where H: #path),
        None => quote!(),
    };
    let dispatch_args = if dispatch_calls.is_empty() {
        quote!(_abs_addr: u16, _len: u16, _hooks: &mut H)
    } else {
        quote!(abs_addr: u16, len: u16, hooks: &mut H)
    };

    Ok(quote! {
        impl #struct_ty {
            pub const BASE: u16 = #base;
            pub const SECTION_SIZE: u16 = #size;

            #[doc(hidden)]
            pub fn ct_check(
                #check_args,
            ) -> ::core::result::Result<(), ::control_table::Error> {
                #(#check_calls)*
                ::core::result::Result::Ok(())
            }

            pub const CT_WRITABLE_ABS: [(u16, u16); #n_writable] = {
                let mut out = [(0u16, 0u16); #n_writable];
                let mut __n = 0;
                #(#writable_copy)*
                out
            };

            pub const WRITE_LOCK: ::core::option::Option<u16> = #write_lock;
        }

        #[allow(clippy::new_without_default)]
        impl #struct_ty {
            pub const fn new() -> Self {
                Self { #(#new_inits),* }
            }
        }

        impl #struct_ty {
            pub fn dispatch_events<H>(
                &self,
                #dispatch_args,
            ) #where_clause {
                #(#dispatch_calls)*
            }
        }

        pub mod addr {
            #(#addr_mods)*
        }

        const _: () = {
            assert!(
                ::core::mem::size_of::<#struct_ty>() == (#size as usize),
                "Section struct size does not equal its declared section budget; add an explicit `_rsvd` tail array to fill the section budget",
            );
            assert!(
                0 #(+ #size_terms)* == ::core::mem::size_of::<#struct_ty>(),
                "Section struct has repr(C) padding; add explicit `_rsvd` fields so the flat read path never exposes uninitialized bytes",
            );
        };
    })
}

fn check_repr(attrs: &[Attribute], struct_span: proc_macro2::Span) -> syn::Result<()> {
    let mut has_c = false;
    for a in attrs {
        if !a.path().is_ident("repr") {
            continue;
        }
        a.parse_nested_meta(|m| {
            if m.path.is_ident("C") {
                has_c = true;
                Ok(())
            } else if m.path.is_ident("packed") {
                if m.input.peek(syn::token::Paren) {
                    let _: proc_macro2::Group = m.input.parse()?;
                }
                Err(m.error("Section derive forbids #[repr(packed)] (unaligned reads + offset_of are unsound)"))
            } else if m.path.is_ident("align") {
                if m.input.peek(syn::token::Paren) {
                    let _: proc_macro2::Group = m.input.parse()?;
                }
                Ok(())
            } else {
                Err(m.error("Section derive requires #[repr(C)] and forbids alternate reprs"))
            }
        })?;
    }
    if !has_c {
        return Err(syn::Error::new(
            struct_span,
            "Section derive requires #[repr(C)]",
        ));
    }
    Ok(())
}

fn parse_section_attrs(attrs: &[Attribute]) -> syn::Result<SectionAttrs> {
    let mut out = SectionAttrs::default();
    for attr in attrs {
        if !attr.path().is_ident("ct_section") {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("base") {
                out.base = Some(m.value()?.parse()?);
                Ok(())
            } else if m.path.is_ident("size") {
                out.size = Some(m.value()?.parse()?);
                Ok(())
            } else if m.path.is_ident("write_locked_by") {
                out.write_locked_by = Some(m.value()?.parse()?);
                Ok(())
            } else if m.path.is_ident("hooks") {
                out.hooks = Some(m.value()?.parse()?);
                Ok(())
            } else if m.path.is_ident("validators") {
                Err(m.error(
                    "region validators are gone in Section; gate the whole section with `write_locked_by = ...`",
                ))
            } else {
                Err(m.error(
                    "unknown ct_section key (expected base|size|write_locked_by|hooks)",
                ))
            }
        })?;
    }
    Ok(out)
}

fn parse_field_skip(attrs: &[Attribute]) -> syn::Result<bool> {
    let mut skip = false;
    for attr in attrs {
        if !attr.path().is_ident("ct_section") {
            continue;
        }
        if matches!(attr.meta, syn::Meta::Path(_)) {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("skip") {
                skip = true;
                Ok(())
            } else {
                Err(m.error("unknown ct_section field key (expected `skip`)"))
            }
        })?;
    }
    Ok(skip)
}

fn block_type_ident(ty: &Type) -> syn::Result<&Ident> {
    let Type::Path(TypePath { qself: None, path }) = ty else {
        return Err(syn::Error::new(
            ty.span(),
            "Section field type must be a simple path to a Block-derived struct",
        ));
    };
    let last = path
        .segments
        .last()
        .ok_or_else(|| syn::Error::new(ty.span(), "Section field type has empty path"))?;
    Ok(&last.ident)
}

/// Recomputes `block.rs`'s `__flat_meta_<crate>_<Block>` macro name so the
/// Section can invoke each block's addr-const macro. Both derives run in the
/// consumer's compilation, so `CARGO_CRATE_NAME` matches.
fn flat_meta_macro_name(struct_ty: &Ident) -> String {
    let crate_name = std::env::var("CARGO_CRATE_NAME").unwrap_or_default();
    let crate_part = crate_name.replace('-', "_");
    format!("__flat_meta_{crate_part}_{struct_ty}")
}
