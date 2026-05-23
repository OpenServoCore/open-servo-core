use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::spanned::Spanned;
use syn::{Attribute, Data, DeriveInput, Expr, ExprArray, Fields, Ident, Type, TypePath};

#[derive(Default)]
struct RegionAttrs {
    addr: Option<Expr>,
    size: Option<Expr>,
    validators: Vec<Expr>,
}

pub fn expand(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let region_ty = &input.ident;

    check_repr_c(&input.attrs, region_ty.span())?;

    let Data::Struct(s) = &input.data else {
        return Err(syn::Error::new(
            input.span(),
            "Region can only be derived for structs",
        ));
    };
    let Fields::Named(fields) = &s.fields else {
        return Err(syn::Error::new(
            input.span(),
            "Region requires a struct with named fields",
        ));
    };

    let region_attrs = parse_region_attrs(&input.attrs)?;
    let addr = region_attrs.addr.ok_or_else(|| {
        syn::Error::new(
            region_ty.span(),
            "Region requires `#[ct_region(addr = ...)]`",
        )
    })?;
    let size = region_attrs.size.ok_or_else(|| {
        syn::Error::new(
            region_ty.span(),
            "Region requires `#[ct_region(size = ...)]`",
        )
    })?;
    let validators = region_attrs.validators;

    let impl_mod = format_ident!("__region_impl_{region_ty}");

    let mut rebased_consts: Vec<TokenStream2> = Vec::new();
    let mut block_descs: Vec<TokenStream2> = Vec::new();
    let mut addr_mods: Vec<TokenStream2> = Vec::new();

    for field in &fields.named {
        let field_name = field.ident.as_ref().unwrap();
        let block_ty_ident = block_type_ident(&field.ty)?;
        let block_ty = &field.ty;

        let rebased_const = format_ident!("REBASED_{}", field_name.to_string().to_uppercase());
        let base_expr =
            quote!((#addr as u16 + ::core::mem::offset_of!(#region_ty, #field_name) as u16));

        rebased_consts.push(quote! {
            pub(super) const #rebased_const:
                [::control_table::FieldDesc; <#block_ty>::FIELD_COUNT] = {
                let mut out = <#block_ty>::FIELDS_AT_ZERO_ARR;
                let base = #base_expr;
                let mut i = 0;
                while i < out.len() {
                    out[i].addr += base;
                    i += 1;
                }
                out
            };
        });

        block_descs.push(quote! {
            ::control_table::BlockDesc {
                addr: #base_expr,
                size: <#block_ty>::SIZE,
                struct_offset: ::core::mem::offset_of!(#region_ty, #field_name) as u16,
                fields: &#impl_mod::#rebased_const,
                validators: <#block_ty>::VALIDATORS,
            }
        });

        let meta_macro = format_ident!("__ct_meta_{block_ty_ident}");
        addr_mods.push(quote! {
            #[allow(dead_code)]
            pub mod #field_name {
                use super::super::{#region_ty, #block_ty_ident};
                #meta_macro!(@addr_consts
                    base = (#addr as u16
                        + ::core::mem::offset_of!(#region_ty, #field_name) as u16),
                    block_ty = #block_ty_ident);
            }
        });
    }

    Ok(quote! {
        #[doc(hidden)]
        mod #impl_mod {
            use super::*;
            #(#rebased_consts)*
        }

        impl #region_ty {
            pub const REGION_DESC: &'static ::control_table::RegionDesc =
                &::control_table::RegionDesc {
                    addr: #addr as u16,
                    size: #size as u16,
                    blocks: &[#(#block_descs),*],
                    validators: &[#(#validators),*],
                };
        }

        pub mod addr {
            #(#addr_mods)*
        }

        const _: () = {
            assert!(::core::mem::size_of::<#region_ty>() <= (#size as usize));
        };
    })
}

fn check_repr_c(attrs: &[Attribute], struct_span: proc_macro2::Span) -> syn::Result<()> {
    let mut has_c = false;
    for a in attrs {
        if !a.path().is_ident("repr") {
            continue;
        }
        a.parse_nested_meta(|m| {
            if m.path.is_ident("C") {
                has_c = true;
                Ok(())
            } else if m.path.is_ident("align") {
                if m.input.peek(syn::token::Paren) {
                    let _: proc_macro2::Group = m.input.parse()?;
                }
                Ok(())
            } else {
                Err(m.error("Region derive requires #[repr(C)] and forbids alternate reprs"))
            }
        })?;
    }
    if !has_c {
        return Err(syn::Error::new(
            struct_span,
            "Region derive requires #[repr(C)]",
        ));
    }
    Ok(())
}

fn parse_region_attrs(attrs: &[Attribute]) -> syn::Result<RegionAttrs> {
    let mut out = RegionAttrs::default();
    for attr in attrs {
        if !attr.path().is_ident("ct_region") {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("addr") {
                out.addr = Some(m.value()?.parse()?);
                Ok(())
            } else if m.path.is_ident("size") {
                out.size = Some(m.value()?.parse()?);
                Ok(())
            } else if m.path.is_ident("validators") {
                let array: ExprArray = m.value()?.parse()?;
                for elem in array.elems {
                    out.validators.push(elem);
                }
                Ok(())
            } else {
                Err(m.error("unknown ct_region key (expected addr|size|validators)"))
            }
        })?;
    }
    Ok(out)
}

fn block_type_ident(ty: &Type) -> syn::Result<&Ident> {
    let Type::Path(TypePath { qself: None, path }) = ty else {
        return Err(syn::Error::new(
            ty.span(),
            "Region field type must be a simple path to a Block-derived struct",
        ));
    };
    let last = path
        .segments
        .last()
        .ok_or_else(|| syn::Error::new(ty.span(), "Region field type has empty path"))?;
    Ok(&last.ident)
}
