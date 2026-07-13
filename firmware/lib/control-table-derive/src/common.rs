//! Helpers shared by the `Block`/`Section`/`Table` derives. Each derive used to
//! carry its own copy of these; the derive name (for error text) and the attr
//! ident are the only things that vary, so they pass as parameters.

use proc_macro2::{Group, Span, TokenStream as TokenStream2};
use quote::quote;
use syn::{Attribute, Ident, Meta, Path};

/// Enforce `#[repr(C)]` and reject `packed`/alternate reprs. `derive` names the
/// deriving macro for the error text (`"Block"`/`"Section"`/`"Table"`).
pub(crate) fn check_repr(attrs: &[Attribute], struct_span: Span, derive: &str) -> syn::Result<()> {
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
                    let _: Group = m.input.parse()?;
                }
                Err(m.error(format!(
                    "{derive} derive forbids #[repr(packed)] (unaligned reads + offset_of are unsound)"
                )))
            } else if m.path.is_ident("align") {
                if m.input.peek(syn::token::Paren) {
                    let _: Group = m.input.parse()?;
                }
                Ok(())
            } else {
                Err(m.error(format!(
                    "{derive} derive requires #[repr(C)] and forbids alternate reprs"
                )))
            }
        })?;
    }
    if !has_c {
        return Err(syn::Error::new(
            struct_span,
            format!("{derive} derive requires #[repr(C)]"),
        ));
    }
    Ok(())
}

/// `#[ct_*(skip)]` on a field. `attr_ident` is the deriving macro's attribute
/// (`"ct_table"`/`"ct_section"`).
pub(crate) fn parse_field_skip(attrs: &[Attribute], attr_ident: &str) -> syn::Result<bool> {
    let mut skip = false;
    for attr in attrs {
        if !attr.path().is_ident(attr_ident) {
            continue;
        }
        if matches!(attr.meta, Meta::Path(_)) {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("skip") {
                skip = true;
                Ok(())
            } else {
                Err(m.error(format!("unknown {attr_ident} field key (expected `skip`)")))
            }
        })?;
    }
    Ok(skip)
}

/// `__flat_meta_<crate>_<Block>` -- the `<crate>` prefix prevents name collisions
/// at the consumer's crate root when two crates each derive a block with the
/// same name. Section and Block must compute the same name for the macro the
/// Block defines and the Section invokes; both run in the consumer's
/// compilation, so `CARGO_CRATE_NAME` matches.
pub(crate) fn flat_meta_macro_name(struct_ty: &Ident) -> String {
    let crate_name = std::env::var("CARGO_CRATE_NAME").unwrap_or_default();
    let crate_part = crate_name.replace('-', "_");
    format!("__flat_meta_{crate_part}_{struct_ty}")
}

/// Optional `where H: <trait>` clause from a single hooks-bound path (the
/// `dispatch_events` generic bound on Table and Section).
pub(crate) fn hooks_where_clause(bound: &Option<Path>) -> TokenStream2 {
    match bound {
        Some(path) => quote!(where H: #path),
        None => quote!(),
    }
}

/// One block of the const-array concat: walk `src`, run `assign` per element,
/// and bump the shared `__n`/`i` counters. `prologue` runs once before the loop
/// (e.g. a `let base = ...;` binding the assignment reads). The enclosing
/// `const {}` owns `out` and `__n`; this only appends.
pub(crate) fn fill_loop(
    src: TokenStream2,
    prologue: TokenStream2,
    assign: TokenStream2,
) -> TokenStream2 {
    quote! {
        {
            let src = #src;
            #prologue
            let mut i = 0;
            while i < src.len() {
                #assign
                __n += 1;
                i += 1;
            }
        }
    }
}
