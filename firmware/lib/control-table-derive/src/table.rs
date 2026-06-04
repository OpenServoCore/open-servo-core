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
    hooks: Option<Path>,
}

struct RegionField<'a> {
    ident: &'a Ident,
    storage_ty: &'a Type,
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
            storage_ty: &f.ty,
            inner_ty: extract_storage_inner_ty(f)?,
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
        quote!(&<#ty>::DESC)
    });

    let region_extents = region_fields.iter().map(|rf| {
        let ty = rf.inner_ty;
        quote!((<#ty>::DESC.addr, <#ty>::DESC.size))
    });

    let new_inits = region_fields.iter().map(|rf| {
        let ident = rf.ident;
        let storage_ty = rf.storage_ty;
        let inner_ty = rf.inner_ty;
        quote!(#ident: <#storage_ty>::new(<#inner_ty>::new()))
    });

    let region_base_arms = region_fields.iter().map(|rf| {
        let ident = rf.ident;
        let inner_ty = rf.inner_ty;
        quote! {
            if ::core::ptr::eq(desc, <#inner_ty>::DESC) {
                return ::core::option::Option::Some(
                    ::control_table::RegionStorageRaw::region_ptr(&self.#ident) as *mut u8,
                );
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

    let region_dispatch_calls: Vec<TokenStream2> = region_fields
        .iter()
        .map(|rf| {
            let ident = rf.ident;
            let inner_ty = rf.inner_ty;
            quote! {
                {
                    let __r_lo = <#inner_ty>::DESC.addr as u32;
                    let __r_hi = __r_lo + <#inner_ty>::DESC.size as u32;
                    let __w_lo = abs_addr as u32;
                    let __w_hi = __w_lo + len as u32;
                    if __w_lo >= __r_lo && __w_hi <= __r_hi {
                        ::control_table::RegionStorage::with(&self.#ident, |__r| {
                            __r.dispatch_events(abs_addr, len, hooks);
                        });
                        return;
                    }
                }
            }
        })
        .collect();

    let hooks_where = match &table_attrs.hooks {
        Some(path) => quote!(where H: #path),
        None => quote!(),
    };
    let dispatch_args = if region_dispatch_calls.is_empty() {
        quote!(_abs_addr: u16, _len: u16, _hooks: &mut H)
    } else {
        quote!(abs_addr: u16, len: u16, hooks: &mut H)
    };

    Ok(quote! {
        impl #table_ty {
            pub const REGIONS: &'static [&'static ::control_table::RegionDesc] =
                &[#(#region_refs),*];
        }

        #[allow(clippy::new_without_default)]
        impl #table_ty {
            pub const fn new() -> Self {
                Self { #(#new_inits),* }
            }
        }

        impl ::control_table::Router for #table_ty {
            fn regions(&self) -> &'static [&'static ::control_table::RegionDesc] {
                Self::REGIONS
            }

            fn region_base(
                &self,
                desc: &::control_table::RegionDesc,
            ) -> ::core::option::Option<*mut u8> {
                #(#region_base_arms)*
                ::core::option::Option::None
            }
        }

        impl #table_ty {
            pub fn dispatch_events<H>(
                &self,
                #dispatch_args,
            ) #hooks_where {
                #(#region_dispatch_calls)*
            }
        }

        pub mod addr {
            #(#addr_reexports)*
        }

        const _: () = {
            assert!(::core::mem::size_of::<#table_ty>() <= (#max_sram as usize));
        };

        const _: () = {
            const EXTENTS: &[(u16, u16)] = &[#(#region_extents),*];
            let mut i = 0;
            while i < EXTENTS.len() {
                let (ai, si) = EXTENTS[i];
                if (ai as u32) + (si as u32) > 0x10000 {
                    panic!("Table region exceeds u16 address space");
                }
                let mut j = i + 1;
                while j < EXTENTS.len() {
                    let (aj, sj) = EXTENTS[j];
                    let i_end = (ai as u32) + (si as u32);
                    let j_end = (aj as u32) + (sj as u32);
                    if (ai as u32) < j_end && (aj as u32) < i_end {
                        panic!("Table regions overlap");
                    }
                    j += 1;
                }
                i += 1;
            }
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
            } else if m.path.is_ident("packed") {
                if m.input.peek(syn::token::Paren) {
                    let _: proc_macro2::Group = m.input.parse()?;
                }
                Err(m.error("Table derive forbids #[repr(packed)] (unaligned reads + offset_of are unsound)"))
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
            } else if m.path.is_ident("hooks") {
                out.hooks = Some(m.value()?.parse()?);
                Ok(())
            } else {
                Err(m.error("unknown ct_table key (expected `max_sram` or `hooks`)"))
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

fn extract_storage_inner_ty(field: &Field) -> syn::Result<&Type> {
    const MSG: &str = "Table field must be a single-type-parameter storage type \
        (e.g. `SyncUnsafeCell<R>`) impl'ing `RegionStorageRaw<R>`";
    let Type::Path(TypePath { qself: None, path }) = &field.ty else {
        return Err(syn::Error::new(field.ty.span(), MSG));
    };
    let last = path
        .segments
        .last()
        .ok_or_else(|| syn::Error::new(field.ty.span(), MSG))?;
    let PathArguments::AngleBracketed(args) = &last.arguments else {
        return Err(syn::Error::new(field.ty.span(), MSG));
    };
    let mut tys = args.args.iter().filter_map(|a| match a {
        GenericArgument::Type(t) => Some(t),
        _ => None,
    });
    let inner = tys
        .next()
        .ok_or_else(|| syn::Error::new(field.ty.span(), MSG))?;
    if tys.next().is_some() {
        return Err(syn::Error::new(field.ty.span(), MSG));
    }
    Ok(inner)
}
