use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::spanned::Spanned;
use syn::{Attribute, Data, DeriveInput, Expr, Fields, Ident, Path, Type};

#[derive(Default)]
struct TableAttrs {
    size: Option<Expr>,
    hooks: Option<Path>,
}

pub fn expand(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let table_ty = &input.ident;

    check_repr(&input.attrs, table_ty.span())?;

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

    let attrs = parse_table_attrs(&input.attrs)?;
    let size = attrs.size.ok_or_else(|| {
        syn::Error::new(table_ty.span(), "Table requires `#[ct_table(size = ...)]`")
    })?;
    let hooks_bound = attrs.hooks;

    let mut size_terms: Vec<TokenStream2> = Vec::new();
    let mut new_inits: Vec<TokenStream2> = Vec::new();

    let mut sec_tys: Vec<&Type> = Vec::new();
    let mut sec_idents: Vec<&Ident> = Vec::new();

    for field in &fields.named {
        let name = field.ident.as_ref().unwrap();
        let ty = &field.ty;

        let init = crate::block::default_init_for_type(ty);
        new_inits.push(quote!(#name: #init));
        size_terms.push(quote!(::core::mem::size_of::<#ty>()));

        if parse_field_skip(&field.attrs)? {
            continue;
        }
        sec_tys.push(ty);
        sec_idents.push(name);
    }

    let words = quote!((#size as usize).div_ceil(32));

    let writable_fill: Vec<TokenStream2> = sec_tys
        .iter()
        .map(|ty| {
            quote! {
                {
                    let spans = <#ty>::CT_WRITABLE_ABS;
                    let mut si = 0;
                    while si < spans.len() {
                        let (off, len) = spans[si];
                        let mut b = off as usize;
                        let end = off as usize + len as usize;
                        while b < end {
                            w[b / 32] |= 1u32 << (b % 32);
                            b += 1;
                        }
                        si += 1;
                    }
                }
            }
        })
        .collect();

    // Per-section index range into the table-level rule arrays: start is the
    // cumulative rule count of preceding sections (section declaration order).
    let section_metas: Vec<TokenStream2> = sec_tys
        .iter()
        .enumerate()
        .map(|(i, ty)| {
            let cmp_pre = &sec_tys[..i];
            let allowed_pre = &sec_tys[..i];
            quote! {
                ::control_table::map::SectionMeta {
                    base: <#ty>::BASE,
                    size: <#ty>::SECTION_SIZE,
                    cmp_rules: {
                        let start = 0u16 #(+ <#cmp_pre>::CT_CMP_RULES_ABS.len() as u16)*;
                        (start, start + <#ty>::CT_CMP_RULES_ABS.len() as u16)
                    },
                    allowed_rules: {
                        let start = 0u16 #(+ <#allowed_pre>::CT_ALLOWED_RULES_ABS.len() as u16)*;
                        (start, start + <#ty>::CT_ALLOWED_RULES_ABS.len() as u16)
                    },
                    write_lock: <#ty>::WRITE_LOCK,
                }
            }
        })
        .collect();

    let n_cmp = quote!(0 #(+ <#sec_tys>::CT_CMP_RULES_ABS.len())*);
    let n_allowed = quote!(0 #(+ <#sec_tys>::CT_ALLOWED_RULES_ABS.len())*);

    let cmp_fill: Vec<TokenStream2> = sec_tys
        .iter()
        .map(|ty| {
            quote! {
                {
                    let src = <#ty>::CT_CMP_RULES_ABS;
                    let mut i = 0;
                    while i < src.len() {
                        out[__n] = src[i];
                        __n += 1;
                        i += 1;
                    }
                }
            }
        })
        .collect();

    let allowed_fill: Vec<TokenStream2> = sec_tys
        .iter()
        .map(|ty| {
            quote! {
                {
                    let src = <#ty>::CT_ALLOWED_RULES_ABS;
                    let mut i = 0;
                    while i < src.len() {
                        out[__n] = src[i];
                        __n += 1;
                        i += 1;
                    }
                }
            }
        })
        .collect();

    let dispatch_calls: Vec<TokenStream2> = sec_idents
        .iter()
        .map(|name| quote!(self.#name.dispatch_events(abs_addr, len, hooks);))
        .collect();

    let addr_reexports: Vec<TokenStream2> = sec_idents
        .iter()
        .map(|name| quote!(pub use super::#name::addr as #name;))
        .collect();

    let base_offset_asserts: Vec<TokenStream2> = sec_idents
        .iter()
        .zip(&sec_tys)
        .map(|(name, ty)| {
            quote! {
                assert!(
                    ::core::mem::offset_of!(#table_ty, #name) as u16 == <#ty>::BASE,
                    "section base must equal its byte offset in the table (address IS offset)",
                );
            }
        })
        .collect();

    let extents: Vec<TokenStream2> = sec_tys
        .iter()
        .map(|ty| quote!((<#ty>::BASE, <#ty>::SECTION_SIZE)))
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

    // Orphan rules forbid `impl RegisterMap for SyncUnsafeCell<#table_ty>` in the
    // consumer crate (foreign trait, non-fundamental foreign wrapper). A local
    // #[repr(transparent)] newtype is orphan-legal, and its interior UnsafeCell
    // keeps `base()`'s pointer mutably-provenanced (writes through a `&#table_ty`
    // would be UB).
    let cell_ty = format_ident!("{table_ty}Cell");

    Ok(quote! {
        impl #table_ty {
            pub const WRITABLE_WORDS: [u32; #words] = {
                let mut w = [0u32; #words];
                #(#writable_fill)*
                w
            };

            pub const CMP_RULES: [::control_table::rules::CmpRule; #n_cmp] = {
                let mut out =
                    [::control_table::rules::CmpRule { addr: 0, spec: 0, val: 0 }; #n_cmp];
                let mut __n = 0;
                #(#cmp_fill)*
                out
            };

            pub const ALLOWED_RULES: [::control_table::rules::AllowedRule; #n_allowed] = {
                let mut out =
                    [::control_table::rules::AllowedRule { addr: 0, allowed: &[] }; #n_allowed];
                let mut __n = 0;
                #(#allowed_fill)*
                out
            };
        }

        #[allow(clippy::new_without_default)]
        impl #table_ty {
            pub const fn new() -> Self {
                Self { #(#new_inits),* }
            }
        }

        #[repr(transparent)]
        pub struct #cell_ty(::core::cell::SyncUnsafeCell<#table_ty>);

        #[allow(clippy::new_without_default)]
        impl #cell_ty {
            pub const fn new() -> Self {
                Self(::core::cell::SyncUnsafeCell::new(#table_ty::new()))
            }

            /// Shared view of the underlying table (e.g. for `dispatch_events`).
            ///
            /// # Safety
            /// Caller upholds the single-writer contract: no `&mut` to the table
            /// may be live for the borrow.
            pub unsafe fn table(&self) -> &#table_ty {
                unsafe { &*self.0.get() }
            }
        }

        impl ::control_table::Region for #table_ty {}

        impl ::control_table::RegionStorage<#table_ty> for #cell_ty {
            fn with<T>(&self, f: impl FnOnce(&#table_ty) -> T) -> T {
                // SAFETY: caller upholds RegionStorage's single-writer contract.
                unsafe { f(&*self.0.get()) }
            }
            fn with_mut<T>(&self, f: impl FnOnce(&mut #table_ty) -> T) -> T {
                // SAFETY: caller upholds RegionStorage's single-writer contract.
                unsafe { f(&mut *self.0.get()) }
            }
        }

        // SAFETY: the interior `SyncUnsafeCell::get` returns a valid aligned
        // pointer to the table, valid for the cell's lifetime.
        unsafe impl ::control_table::RegionStorageRaw<#table_ty> for #cell_ty {
            fn region_ptr(&self) -> *mut #table_ty {
                self.0.get()
            }
        }

        impl ::control_table::map::RegisterMap for #cell_ty {
            const SIZE: usize = #size as usize;
            const WRITABLE: &'static [u32] = &#table_ty::WRITABLE_WORDS;
            const SECTIONS: &'static [::control_table::map::SectionMeta] =
                &[#(#section_metas),*];
            const CMP_RULES: &'static [::control_table::rules::CmpRule] =
                &#table_ty::CMP_RULES;
            const ALLOWED_RULES: &'static [::control_table::rules::AllowedRule] =
                &#table_ty::ALLOWED_RULES;
            fn base(&self) -> *mut u8 {
                self.0.get() as *mut u8
            }
        }

        impl #table_ty {
            pub fn dispatch_events<H>(
                &self,
                #dispatch_args,
            ) #where_clause {
                #(#dispatch_calls)*
            }
        }

        pub mod addr {
            #(#addr_reexports)*
        }

        const _: () = {
            assert!(
                ::core::mem::size_of::<#table_ty>() == (#size as usize),
                "Table struct size does not equal its declared size; add an explicit `_rsvd` tail array to fill the table budget",
            );
            assert!(
                0 #(+ #size_terms)* == ::core::mem::size_of::<#table_ty>(),
                "Table struct has repr(C) padding; add explicit `_rsvd` fields so the flat read path never exposes uninitialized bytes",
            );

            #(#base_offset_asserts)*

            const EXTENTS: &[(u16, u16)] = &[#(#extents),*];
            let mut i = 0;
            while i < EXTENTS.len() {
                let (ai, si) = EXTENTS[i];
                assert!(
                    (ai as u32) + (si as u32) <= (#size as u32),
                    "section extends past the table end",
                );
                let mut j = i + 1;
                while j < EXTENTS.len() {
                    let (aj, sj) = EXTENTS[j];
                    let i_end = (ai as u32) + (si as u32);
                    let j_end = (aj as u32) + (sj as u32);
                    assert!(
                        (ai as u32) >= j_end || (aj as u32) >= i_end,
                        "sections overlap within the table",
                    );
                    j += 1;
                }
                i += 1;
            }
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
        if matches!(attr.meta, syn::Meta::Path(_)) {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("size") {
                out.size = Some(m.value()?.parse()?);
                Ok(())
            } else if m.path.is_ident("hooks") {
                out.hooks = Some(m.value()?.parse()?);
                Ok(())
            } else {
                Err(m.error("unknown ct_table key (expected `size` or `hooks`)"))
            }
        })?;
    }
    Ok(out)
}

fn parse_field_skip(attrs: &[Attribute]) -> syn::Result<bool> {
    let mut skip = false;
    for attr in attrs {
        if !attr.path().is_ident("ct_table") {
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
                Err(m.error("unknown ct_table field key (expected `skip`)"))
            }
        })?;
    }
    Ok(skip)
}
