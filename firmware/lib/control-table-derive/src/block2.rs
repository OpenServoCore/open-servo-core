use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::punctuated::Punctuated;
use syn::spanned::Spanned;
use syn::token::PathSep;
use syn::{Attribute, Data, DeriveInput, Expr, Fields, Ident, Path, PathSegment, Type};

#[derive(Copy, Clone)]
enum AccessMode {
    Ro,
    Rw,
}

#[derive(Copy, Clone)]
enum CmpOp {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
}

impl CmpOp {
    fn token(self) -> TokenStream2 {
        match self {
            CmpOp::Lt => quote!(::control_table::rules::CmpOp::Lt),
            CmpOp::Le => quote!(::control_table::rules::CmpOp::Le),
            CmpOp::Gt => quote!(::control_table::rules::CmpOp::Gt),
            CmpOp::Ge => quote!(::control_table::rules::CmpOp::Ge),
            CmpOp::Eq => quote!(::control_table::rules::CmpOp::Eq),
            CmpOp::Ne => quote!(::control_table::rules::CmpOp::Ne),
        }
    }
}

struct FieldAttrs {
    skip: bool,
    access: AccessMode,
    compares: Vec<(CmpOp, Expr)>,
    abs: bool,
    hook: Option<Path>,
}

impl Default for FieldAttrs {
    fn default() -> Self {
        Self {
            skip: false,
            access: AccessMode::Rw,
            compares: Vec::new(),
            abs: false,
            hook: None,
        }
    }
}

struct HookBinding<'a> {
    field_ident: &'a Ident,
    field_ty: &'a Type,
    trait_path: Path,
    method_ident: Ident,
}

pub fn expand(input: &DeriveInput) -> syn::Result<TokenStream2> {
    let struct_ty = &input.ident;

    check_repr(&input.attrs, struct_ty.span())?;

    let Data::Struct(s) = &input.data else {
        return Err(syn::Error::new(
            input.span(),
            "FlatBlock can only be derived for structs",
        ));
    };

    let Fields::Named(fields) = &s.fields else {
        return Err(syn::Error::new(
            input.span(),
            "FlatBlock requires a struct with named fields",
        ));
    };

    let hooks_trait = parse_block_attrs(&input.attrs)?;

    let mut rules: Vec<TokenStream2> = Vec::new();
    let mut writable: Vec<TokenStream2> = Vec::new();
    let mut size_terms: Vec<TokenStream2> = Vec::new();
    let mut new_inits: Vec<TokenStream2> = Vec::new();
    let mut kept_idents: Vec<&Ident> = Vec::new();
    let mut kept_upper: Vec<Ident> = Vec::new();
    let mut hook_bindings: Vec<HookBinding> = Vec::new();

    for field in &fields.named {
        let name = field.ident.as_ref().unwrap();
        let ty = &field.ty;
        let attrs = parse_field_attrs(&field.attrs)?;

        if let Some(hook_path) = &attrs.hook {
            let resolved = resolve_hook_path(hook_path, hooks_trait.as_ref())?;
            hook_bindings.push(HookBinding {
                field_ident: name,
                field_ty: ty,
                trait_path: resolved.0,
                method_ident: resolved.1,
            });
        }

        let init = crate::block::default_init_for_type(ty);
        new_inits.push(quote!(#name: #init));
        size_terms.push(quote!(::core::mem::size_of::<#ty>()));

        let is_ro = matches!(attrs.access, AccessMode::Ro);
        if !attrs.compares.is_empty() && (attrs.skip || is_ro) {
            return Err(syn::Error::new(
                name.span(),
                "compare clause on a skipped or read-only field never runs (drop the compare or make the field rw)",
            ));
        }

        if attrs.skip {
            continue;
        }

        let rel_addr = quote!(::core::mem::offset_of!(#struct_ty, #name) as u16);

        if !is_ro {
            if let Some(allowed) = auto_enum_allowed(ty) {
                rules.push(quote! {
                    ::control_table::rules::Rule {
                        offset: #rel_addr,
                        width: 1,
                        kind: ::control_table::rules::RuleKind::Enum { allowed: #allowed },
                    }
                });
            }

            if !attrs.compares.is_empty() {
                let (width, signed) = cmp_width_signed(ty)?;
                let abs = attrs.abs;
                for (op, expr) in &attrs.compares {
                    let op_tok = op.token();
                    let rhs = build_rhs(expr);
                    rules.push(quote! {
                        ::control_table::rules::Rule {
                            offset: #rel_addr,
                            width: #width,
                            kind: ::control_table::rules::RuleKind::Cmp {
                                op: #op_tok,
                                rhs: #rhs,
                                signed: #signed,
                                abs: #abs,
                            },
                        }
                    });
                }
            }

            writable.push(quote!((#rel_addr, ::core::mem::size_of::<#ty>() as u16)));
        }

        kept_upper.push(Ident::new(&name.to_string().to_uppercase(), name.span()));
        kept_idents.push(name);
    }

    let n_rules = rules.len();
    let n_writable = writable.len();
    let meta_macro = Ident::new(&flat_meta_macro_name(struct_ty), struct_ty.span());
    let hooks_emit = build_hooks_emit(struct_ty, &hook_bindings);

    Ok(quote! {
        impl #struct_ty {
            pub const CT_SIZE: u16 = ::core::mem::size_of::<Self>() as u16;
            pub const CT_RULES: [::control_table::rules::Rule; #n_rules] = [#(#rules),*];
            pub const CT_WRITABLE: [(u16, u16); #n_writable] = [#(#writable),*];
        }

        #[allow(clippy::new_without_default)]
        impl #struct_ty {
            pub const fn new() -> Self {
                Self { #(#new_inits),* }
            }
        }

        #hooks_emit

        const _: () = {
            assert!(
                0 #(+ #size_terms)* == ::core::mem::size_of::<#struct_ty>(),
                "FlatBlock struct has repr(C) padding; add explicit `_rsvd` fields so the flat read path never exposes uninitialized bytes",
            );
        };

        #[doc(hidden)]
        #[macro_export]
        macro_rules! #meta_macro {
            (@addr_consts base = $base:expr, block_ty = $bty:path) => {
                #(
                    pub const #kept_upper: u16 =
                        $base + ::core::mem::offset_of!($bty, #kept_idents) as u16;
                )*
            };
        }
    })
}

fn build_hooks_emit(struct_ty: &Ident, bindings: &[HookBinding<'_>]) -> TokenStream2 {
    let unique_traits = unique_trait_paths(bindings);
    let body: Vec<TokenStream2> = bindings
        .iter()
        .map(|b| {
            let field_ident = b.field_ident;
            let field_ty = b.field_ty;
            let trait_path = &b.trait_path;
            let method_ident = &b.method_ident;
            quote! {
                {
                    let __f_lo = block_base as u32
                        + ::core::mem::offset_of!(#struct_ty, #field_ident) as u32;
                    let __f_hi = __f_lo + ::core::mem::size_of::<#field_ty>() as u32;
                    let __w_lo = abs_addr as u32;
                    let __w_hi = __w_lo + len as u32;
                    if __w_lo < __f_hi && __f_lo < __w_hi {
                        <H as #trait_path>::#method_ident(hooks, self.#field_ident);
                    }
                }
            }
        })
        .collect();

    let where_clause = if unique_traits.is_empty() {
        quote!()
    } else {
        quote!(where H: #(#unique_traits)+*)
    };

    let dispatch_args = if bindings.is_empty() {
        quote!(_abs_addr: u16, _len: u16, _block_base: u16, _hooks: &mut H)
    } else {
        quote!(abs_addr: u16, len: u16, block_base: u16, hooks: &mut H)
    };

    quote! {
        impl #struct_ty {
            pub fn dispatch_events<H>(
                &self,
                #dispatch_args,
            ) #where_clause {
                #(#body)*
            }
        }
    }
}

fn unique_trait_paths<'a>(bindings: &'a [HookBinding<'_>]) -> Vec<&'a Path> {
    let mut out: Vec<&Path> = Vec::new();
    for b in bindings {
        if !out.iter().any(|p| path_eq(p, &b.trait_path)) {
            out.push(&b.trait_path);
        }
    }
    out
}

fn path_eq(a: &Path, b: &Path) -> bool {
    use quote::ToTokens;
    a.to_token_stream().to_string() == b.to_token_stream().to_string()
}

/// Splits `Trait::method` into (Trait, method). If `hook` is a single ident
/// and the block declared `#[ct_block(hooks = ...)]`, that trait is used as
/// the prefix.
fn resolve_hook_path(hook: &Path, block_trait: Option<&Path>) -> syn::Result<(Path, Ident)> {
    let total = hook.segments.len();
    if total >= 2 {
        let mut trait_path = Path {
            leading_colon: hook.leading_colon,
            segments: Punctuated::<PathSegment, PathSep>::new(),
        };
        for (i, seg) in hook.segments.iter().enumerate() {
            if i == total - 1 {
                break;
            }
            trait_path.segments.push(seg.clone());
        }
        let method_ident = hook.segments.last().unwrap().ident.clone();
        Ok((trait_path, method_ident))
    } else {
        let method_ident = hook
            .segments
            .last()
            .map(|s| s.ident.clone())
            .ok_or_else(|| syn::Error::new(hook.span(), "empty hook path"))?;
        let trait_path = block_trait.ok_or_else(|| {
            syn::Error::new(
                hook.span(),
                "single-ident `hook = name` requires `#[ct_block(hooks = TraitPath)]` on the block",
            )
        })?;
        Ok((trait_path.clone(), method_ident))
    }
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
                Err(m.error("FlatBlock derive forbids #[repr(packed)] (unaligned reads + offset_of are unsound)"))
            } else if m.path.is_ident("align") {
                if m.input.peek(syn::token::Paren) {
                    let _: proc_macro2::Group = m.input.parse()?;
                }
                Ok(())
            } else {
                Err(m.error("FlatBlock derive requires #[repr(C)] and forbids alternate reprs"))
            }
        })?;
    }
    if !has_c {
        return Err(syn::Error::new(
            struct_span,
            "FlatBlock derive requires #[repr(C)]",
        ));
    }
    Ok(())
}

fn parse_block_attrs(attrs: &[Attribute]) -> syn::Result<Option<Path>> {
    let mut hooks_trait = None;
    for attr in attrs {
        if !attr.path().is_ident("ct_block") {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("hooks") {
                hooks_trait = Some(m.value()?.parse()?);
                Ok(())
            } else if m.path.is_ident("validators") {
                Err(m.error(
                    "block validators are gone in FlatBlock; move the check to a field rule",
                ))
            } else {
                Err(m.error("unknown ct_block key (expected `hooks = TraitPath`)"))
            }
        })?;
    }
    Ok(hooks_trait)
}

fn parse_field_attrs(attrs: &[Attribute]) -> syn::Result<FieldAttrs> {
    let mut out = FieldAttrs::default();
    for attr in attrs {
        if !attr.path().is_ident("ct_field") {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("skip") {
                out.skip = true;
                Ok(())
            } else if m.path.is_ident("abs") {
                out.abs = true;
                Ok(())
            } else if m.path.is_ident("access") {
                let val = m.value()?;
                let ident: Ident = val.parse()?;
                out.access = match ident.to_string().as_str() {
                    "ro" => AccessMode::Ro,
                    "rw" => AccessMode::Rw,
                    "reserved" => {
                        return Err(syn::Error::new(
                            ident.span(),
                            "`access = reserved` is gone; use `skip` for padding fields",
                        ));
                    }
                    other => {
                        return Err(syn::Error::new(
                            ident.span(),
                            format!("unknown access `{other}` (expected ro|rw)"),
                        ));
                    }
                };
                Ok(())
            } else if m.path.is_ident("hook") {
                let val = m.value()?;
                out.hook = Some(val.parse()?);
                Ok(())
            } else if let Some(op) = cmp_op_for_path(&m.path) {
                let val = m.value()?;
                let expr: Expr = val.parse()?;
                out.compares.push((op, expr));
                Ok(())
            } else {
                Err(m.error("unknown ct_field key"))
            }
        })?;
    }
    Ok(out)
}

fn cmp_op_for_path(path: &Path) -> Option<CmpOp> {
    let ident = path.get_ident()?;
    Some(match ident.to_string().as_str() {
        "lt" => CmpOp::Lt,
        "le" => CmpOp::Le,
        "gt" => CmpOp::Gt,
        "ge" => CmpOp::Ge,
        "eq" => CmpOp::Eq,
        "ne" => CmpOp::Ne,
        _ => return None,
    })
}

/// Auto enum rule source: `bool` -> `BOOL_ALLOWED`; any non-primitive, non-array
/// path type -> `<Ty as HasAllowed>::ALLOWED`; primitives and arrays -> none.
fn auto_enum_allowed(ty: &Type) -> Option<TokenStream2> {
    let Type::Path(tp) = ty else { return None };
    let path = &tp.path;
    if let Some(ident) = path.get_ident() {
        let name = ident.to_string();
        if name == "bool" {
            return Some(quote!(::control_table::BOOL_ALLOWED));
        }
        if is_primitive(&name) {
            return None;
        }
    }
    Some(quote!(<#path as ::control_table::HasAllowed>::ALLOWED))
}

fn is_primitive(name: &str) -> bool {
    matches!(
        name,
        "u8" | "u16"
            | "u32"
            | "u64"
            | "u128"
            | "i8"
            | "i16"
            | "i32"
            | "i64"
            | "i128"
            | "f32"
            | "f64"
            | "usize"
            | "isize"
    )
}

/// `(width, signed)` for a Cmp rule; errors on u32/arrays/other (no wider Cmp).
fn cmp_width_signed(ty: &Type) -> syn::Result<(u8, bool)> {
    if let Type::Path(tp) = ty
        && let Some(ident) = tp.path.get_ident()
    {
        match ident.to_string().as_str() {
            "u8" => return Ok((1, false)),
            "i8" => return Ok((1, true)),
            "u16" => return Ok((2, false)),
            "i16" => return Ok((2, true)),
            "i32" => return Ok((4, true)),
            _ => {}
        }
    }
    Err(syn::Error::new(
        ty.span(),
        "compare clauses require a primitive integer (u8/u16/i8/i16/i32)",
    ))
}

fn build_rhs(expr: &Expr) -> TokenStream2 {
    if let Expr::Reference(r) = expr {
        let inner = &r.expr;
        quote!(::control_table::rules::Rhs::Reg(#inner))
    } else {
        quote!(::control_table::rules::Rhs::Imm((#expr) as i32))
    }
}

/// `__flat_meta_<crate>_<Block>` — the `<crate>` prefix prevents name collisions
/// at the consumer's crate root when two crates each derive a block with the
/// same name. The Section derive must compute the same name for its invocation.
fn flat_meta_macro_name(struct_ty: &Ident) -> String {
    let crate_name = std::env::var("CARGO_CRATE_NAME").unwrap_or_default();
    let crate_part = crate_name.replace('-', "_");
    format!("__flat_meta_{crate_part}_{struct_ty}")
}
