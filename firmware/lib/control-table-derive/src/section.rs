use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::spanned::Spanned;
use syn::{Attribute, Data, DeriveInput, Expr, Fields, Ident, Path, Type, TypePath};

#[derive(Default)]
struct SectionAttrs {
    base: Option<Expr>,
    size: Option<Expr>,
    hooks: Option<Path>,
}

pub fn expand(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let struct_ty = &input.ident;

    crate::common::check_repr(&input.attrs, struct_ty.span(), "Section")?;

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

        if crate::common::parse_field_skip(&field.attrs, "ct_section")? {
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
    let n_cmp = quote!(0 #(+ <#block_tys>::CT_CMP_RULES.len())*);
    let n_allowed = quote!(0 #(+ <#block_tys>::CT_ALLOWED_RULES.len())*);

    // Blocks in declaration order (= address order): preserves the old
    // sorted-by-offset first-failure precedence. Only `addr` is rebased into
    // table-absolute form; register-RHS `val` addrs are already absolute.
    let rebase_rule = |src: TokenStream2, base_expr: &TokenStream2| {
        crate::common::fill_loop(
            src,
            quote!(let base = #base_expr;),
            quote!(let mut r = src[i]; r.addr += base; out[__n] = r;),
        )
    };
    let cmp_copy: Vec<TokenStream2> = block_tys
        .iter()
        .zip(&base_exprs)
        .map(|(ty, base_expr)| rebase_rule(quote!(<#ty>::CT_CMP_RULES), base_expr))
        .collect();

    let allowed_copy: Vec<TokenStream2> = block_tys
        .iter()
        .zip(&base_exprs)
        .map(|(ty, base_expr)| rebase_rule(quote!(<#ty>::CT_ALLOWED_RULES), base_expr))
        .collect();

    let writable_copy: Vec<TokenStream2> = block_tys
        .iter()
        .zip(&base_exprs)
        .map(|(ty, base_expr)| {
            crate::common::fill_loop(
                quote!(<#ty>::CT_WRITABLE),
                quote!(let base = #base_expr;),
                quote!(out[__n] = (src[i].0 + base, src[i].1);),
            )
        })
        .collect();

    let addr_mods: Vec<TokenStream2> = block_idents
        .iter()
        .zip(&block_ty_idents)
        .map(|(name, block_ty_ident)| {
            let meta_macro = Ident::new(
                &crate::common::flat_meta_macro_name(block_ty_ident),
                block_ty_ident.span(),
            );
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

    let where_clause = crate::common::hooks_where_clause(&hooks_bound);
    let dispatch_args = if dispatch_calls.is_empty() {
        quote!(_abs_addr: u16, _len: u16, _hooks: &mut H)
    } else {
        quote!(abs_addr: u16, len: u16, hooks: &mut H)
    };

    Ok(quote! {
        impl #struct_ty {
            pub const BASE: u16 = #base;
            pub const SECTION_SIZE: u16 = #size;

            pub const CT_WRITABLE_ABS: [(u16, u16); #n_writable] = {
                let mut out = [(0u16, 0u16); #n_writable];
                let mut __n = 0;
                #(#writable_copy)*
                out
            };

            pub const CT_CMP_RULES_ABS: [::control_table::rules::CmpRule; #n_cmp] = {
                let mut out = [::control_table::rules::CmpRule { addr: 0, spec: 0, val: 0 }; #n_cmp];
                let mut __n = 0;
                #(#cmp_copy)*
                out
            };

            pub const CT_ALLOWED_RULES_ABS: [::control_table::rules::AllowedRule; #n_allowed] = {
                let mut out =
                    [::control_table::rules::AllowedRule { addr: 0, allowed: &[] }; #n_allowed];
                let mut __n = 0;
                #(#allowed_copy)*
                out
            };

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
            } else if m.path.is_ident("hooks") {
                out.hooks = Some(m.value()?.parse()?);
                Ok(())
            } else {
                Err(m.error("unknown ct_section key (expected base|size|hooks)"))
            }
        })?;
    }
    Ok(out)
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
