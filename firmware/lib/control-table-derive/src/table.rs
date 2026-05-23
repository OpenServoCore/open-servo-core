use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::spanned::Spanned;
use syn::{
    Attribute, Data, DeriveInput, Expr, Field, Fields, GenericArgument, Ident, Path, PathArguments,
    Type, TypePath,
};

#[derive(Default)]
struct TableAttrs {
    max_sram: Option<Expr>,
}

struct RegionField<'a> {
    ident: &'a Ident,
    inner_ty: &'a Type,
    addr_mod: Option<Path>,
}

pub fn expand(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let table_ty = &input.ident;

    check_repr_c(&input.attrs, table_ty.span())?;

    let Data::Struct(s) = &input.data else {
        return Err(syn::Error::new(
            input.span(),
            "Table can only be derived for structs",
        ));
    };
    let Fields::Named(fields) = &s.fields else {
        return Err(syn::Error::new(
            input.span(),
            "Table requires a struct with named fields",
        ));
    };

    let table_attrs = parse_table_attrs(&input.attrs)?;
    let max_sram = table_attrs.max_sram.ok_or_else(|| {
        syn::Error::new(
            table_ty.span(),
            "Table requires `#[ct_table(max_sram = ...)]`",
        )
    })?;

    let mut region_fields: Vec<RegionField> = Vec::new();
    for f in &fields.named {
        let Some(field_attrs) = parse_ct_region_attrs(&f.attrs)? else {
            return Err(syn::Error::new(
                f.span(),
                "Table field must carry `#[ct_region]`",
            ));
        };
        region_fields.push(RegionField {
            ident: f.ident.as_ref().unwrap(),
            inner_ty: extract_sync_unsafe_cell_inner(f)?,
            addr_mod: field_attrs.addr_mod,
        });
    }

    if region_fields.is_empty() {
        return Err(syn::Error::new(
            input.span(),
            "Table requires at least one `#[ct_region]` field",
        ));
    }

    let region_refs = region_fields.iter().map(|rf| {
        let ty = rf.inner_ty;
        quote!(&<#ty>::REGION_DESC)
    });

    let const_new_inits = region_fields.iter().map(|rf| {
        let ident = rf.ident;
        let ty = rf.inner_ty;
        quote!(#ident: ::core::cell::SyncUnsafeCell::new(<#ty>::const_new()))
    });

    let region_base_arms = region_fields.iter().map(|rf| {
        let ident = rf.ident;
        let ty = rf.inner_ty;
        quote! {
            if addr == <#ty>::REGION_DESC.addr {
                return self.#ident.get() as *mut u8;
            }
        }
    });

    let addr_reexports = region_fields.iter().map(|rf| {
        let ident = rf.ident;
        match &rf.addr_mod {
            Some(path) => quote!(pub use #path::addr as #ident;),
            None => quote!(pub use super::#ident::addr as #ident;),
        }
    });

    Ok(quote! {
        impl #table_ty {
            pub const REGIONS: &'static [&'static ::control_table::RegionDesc] =
                &[#(#region_refs),*];

            pub const fn const_new() -> Self {
                Self { #(#const_new_inits),* }
            }
        }

        impl ::control_table::Router for #table_ty {
            fn regions(&self) -> &'static [&'static ::control_table::RegionDesc] {
                Self::REGIONS
            }

            fn region_base(&self, desc: &::control_table::RegionDesc) -> *mut u8 {
                let addr = desc.addr;
                #(#region_base_arms)*
                ::core::ptr::null_mut()
            }
        }

        pub mod addr {
            #(#addr_reexports)*
        }

        const _: () = {
            assert!(::core::mem::size_of::<#table_ty>() <= (#max_sram as usize));
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
                Err(m.error("Table derive requires #[repr(C)] and forbids alternate reprs"))
            }
        })?;
    }
    if !has_c {
        return Err(syn::Error::new(
            struct_span,
            "Table derive requires #[repr(C)]",
        ));
    }
    Ok(())
}

fn parse_table_attrs(attrs: &[Attribute]) -> syn::Result<TableAttrs> {
    let mut out = TableAttrs::default();
    for attr in attrs {
        if !attr.path().is_ident("ct_table") {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("max_sram") {
                out.max_sram = Some(m.value()?.parse()?);
                Ok(())
            } else {
                Err(m.error("unknown ct_table key (expected max_sram)"))
            }
        })?;
    }
    Ok(out)
}

#[derive(Default)]
struct CtRegionAttrs {
    addr_mod: Option<Path>,
}

fn parse_ct_region_attrs(attrs: &[Attribute]) -> syn::Result<Option<CtRegionAttrs>> {
    let mut out: Option<CtRegionAttrs> = None;
    for attr in attrs {
        if !attr.path().is_ident("ct_region") {
            continue;
        }
        let entry = out.get_or_insert_with(CtRegionAttrs::default);
        if matches!(attr.meta, syn::Meta::Path(_)) {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("addr_mod") {
                entry.addr_mod = Some(m.value()?.parse()?);
                Ok(())
            } else {
                Err(m.error("unknown ct_region key (expected addr_mod)"))
            }
        })?;
    }
    Ok(out)
}

fn extract_sync_unsafe_cell_inner(field: &Field) -> syn::Result<&Type> {
    let Type::Path(TypePath { qself: None, path }) = &field.ty else {
        return Err(syn::Error::new(
            field.ty.span(),
            "Table field must be `SyncUnsafeCell<T>`",
        ));
    };
    let last = path
        .segments
        .last()
        .ok_or_else(|| syn::Error::new(field.ty.span(), "Table field has empty type path"))?;
    if last.ident != "SyncUnsafeCell" {
        return Err(syn::Error::new(
            field.ty.span(),
            "Table field must be `SyncUnsafeCell<T>`",
        ));
    }
    let PathArguments::AngleBracketed(args) = &last.arguments else {
        return Err(syn::Error::new(
            field.ty.span(),
            "Table field `SyncUnsafeCell` requires a type argument",
        ));
    };
    let mut tys = args.args.iter().filter_map(|a| match a {
        GenericArgument::Type(t) => Some(t),
        _ => None,
    });
    let inner = tys.next().ok_or_else(|| {
        syn::Error::new(
            field.ty.span(),
            "Table field `SyncUnsafeCell` requires a type argument",
        )
    })?;
    if tys.next().is_some() {
        return Err(syn::Error::new(
            field.ty.span(),
            "Table field `SyncUnsafeCell` takes exactly one type argument",
        ));
    }
    Ok(inner)
}
