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
    /// The matching `control_table::rules::OP_*` const, as a path token.
    fn const_tokens(self) -> TokenStream2 {
        match self {
            CmpOp::Lt => quote!(::control_table::rules::OP_LT),
            CmpOp::Le => quote!(::control_table::rules::OP_LE),
            CmpOp::Gt => quote!(::control_table::rules::OP_GT),
            CmpOp::Ge => quote!(::control_table::rules::OP_GE),
            CmpOp::Eq => quote!(::control_table::rules::OP_EQ),
            CmpOp::Ne => quote!(::control_table::rules::OP_NE),
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

    crate::common::check_repr(&input.attrs, struct_ty.span(), "Block")?;

    let Data::Struct(s) = &input.data else {
        return Err(syn::Error::new(
            input.span(),
            "Block can only be derived for structs",
        ));
    };

    let Fields::Named(fields) = &s.fields else {
        return Err(syn::Error::new(
            input.span(),
            "Block requires a struct with named fields",
        ));
    };

    let hooks_trait = parse_block_attrs(&input.attrs)?;

    let mut cmp_rules: Vec<TokenStream2> = Vec::new();
    let mut allowed_rules: Vec<TokenStream2> = Vec::new();
    let mut writable: Vec<TokenStream2> = Vec::new();
    let mut field_descs: Vec<TokenStream2> = Vec::new();
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
                allowed_rules.push(quote! {
                    ::control_table::rules::AllowedRule { addr: #rel_addr, allowed: #allowed }
                });
            }

            if !attrs.compares.is_empty() {
                let (width, signed) = cmp_width_signed(ty)?;
                let abs = attrs.abs && signed;
                for (op, expr) in &attrs.compares {
                    let op_const = op.const_tokens();
                    let (is_reg, val) = build_rule_val(expr);
                    let spec = if is_reg {
                        quote!(::control_table::rules::spec(#width, #signed, #abs, #op_const)
                            | ::control_table::rules::SPEC_RHS_REG)
                    } else {
                        quote!(::control_table::rules::spec(#width, #signed, #abs, #op_const))
                    };
                    cmp_rules.push(quote! {
                        ::control_table::rules::CmpRule { addr: #rel_addr, spec: #spec, val: #val }
                    });
                }
            }

            writable.push(quote!((#rel_addr, ::core::mem::size_of::<#ty>() as u16)));
        }

        let name_str = name.to_string();
        let kind = field_kind(ty);
        let (min, max) = field_bounds(name, &attrs.compares, attrs.abs)?;
        let writable_flag = !is_ro;
        field_descs.push(quote! {
            ::control_table::descriptor::FieldDesc {
                name: #name_str,
                addr: #rel_addr,
                width: ::core::mem::size_of::<#ty>() as u16,
                writable: #writable_flag,
                kind: #kind,
                min: #min,
                max: #max,
            }
        });

        kept_upper.push(Ident::new(&name.to_string().to_uppercase(), name.span()));
        kept_idents.push(name);
    }

    let n_writable = writable.len();
    let n_fields = field_descs.len();
    let n_cmp = cmp_rules.len();
    let n_allowed = allowed_rules.len();
    let meta_macro = Ident::new(
        &crate::common::flat_meta_macro_name(struct_ty),
        struct_ty.span(),
    );
    let hooks_emit = build_hooks_emit(struct_ty, &hook_bindings);

    Ok(quote! {
        impl #struct_ty {
            pub const CT_SIZE: u16 = ::core::mem::size_of::<Self>() as u16;
            pub const CT_WRITABLE: [(u16, u16); #n_writable] = [#(#writable),*];

            // Block-relative rule descriptors; the Section derive rebases `addr`
            // into table-absolute form (register RHS addrs are already absolute).
            pub const CT_CMP_RULES: [::control_table::rules::CmpRule; #n_cmp] =
                [#(#cmp_rules),*];
            pub const CT_ALLOWED_RULES: [::control_table::rules::AllowedRule; #n_allowed] =
                [#(#allowed_rules),*];

            // Block-relative field descriptors; the Section derive rebases
            // `addr` into table-absolute form.
            pub const CT_FIELDS: [::control_table::descriptor::FieldDesc; #n_fields] =
                [#(#field_descs),*];
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
                "Block struct has repr(C) padding; add explicit `_rsvd` fields so the flat read path never exposes uninitialized bytes",
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
                Err(m.error("block validators are gone in Block; move the check to a field rule"))
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

/// The integer primitives the derive recognizes: zero-initialized in the
/// generated `new()` ([`default_init_for_type`]) and, with f32/f64, the set that
/// carries no auto-enum rule ([`is_primitive`]). One list so a new width can't be
/// added to one consumer and forgotten in the other.
const PRIMITIVE_INTS: &[&str] = &[
    "u8", "u16", "u32", "u64", "u128", "i8", "i16", "i32", "i64", "i128", "usize", "isize",
];

fn is_primitive(name: &str) -> bool {
    PRIMITIVE_INTS.contains(&name) || matches!(name, "f32" | "f64")
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

/// Descriptor `kind` token for a field type, mirroring the runtime helpers:
/// `bool` -> Bool; unsigned ints -> UInt; signed ints -> Int; floats and
/// arrays -> Bytes (no Float kind, none exist in tables); any other path type
/// -> `Enum(<Ty as HasAllowed>::VARIANTS)`.
fn field_kind(ty: &Type) -> TokenStream2 {
    if let Type::Array(_) = ty {
        return quote!(::control_table::descriptor::FieldKind::Bytes);
    }
    if let Type::Path(tp) = ty
        && let Some(ident) = tp.path.get_ident()
    {
        match ident.to_string().as_str() {
            "bool" => return quote!(::control_table::descriptor::FieldKind::Bool),
            "u8" | "u16" | "u32" | "u64" | "u128" | "usize" => {
                return quote!(::control_table::descriptor::FieldKind::UInt);
            }
            "i8" | "i16" | "i32" | "i64" | "i128" | "isize" => {
                return quote!(::control_table::descriptor::FieldKind::Int);
            }
            "f32" | "f64" => return quote!(::control_table::descriptor::FieldKind::Bytes),
            _ => {}
        }
    }
    let Type::Path(tp) = ty else {
        return quote!(::control_table::descriptor::FieldKind::Bytes);
    };
    let path = &tp.path;
    quote!(::control_table::descriptor::FieldKind::Enum(
        <#path as ::control_table::HasAllowed>::VARIANTS
    ))
}

/// Descriptor `(min, max)` tokens from a field's compare clauses. Only
/// immediate right-hand sides contribute: a register RHS names another field,
/// and abs bounds are `|x|` bounds, not plain scalar bounds - both export as
/// `None`. `gt`/`lt` fold to inclusive by +/-1; `eq`/`ne` contribute nothing.
/// Two immediates on the same side of one field is an error, not last-wins.
fn field_bounds(
    name: &Ident,
    compares: &[(CmpOp, Expr)],
    abs: bool,
) -> syn::Result<(TokenStream2, TokenStream2)> {
    let mut min: Option<TokenStream2> = None;
    let mut max: Option<TokenStream2> = None;
    if !abs {
        for (op, expr) in compares {
            if let Expr::Reference(_) = expr {
                continue;
            }
            let (slot, val) = match op {
                CmpOp::Ge => (&mut min, quote!(Some((#expr) as i32))),
                CmpOp::Gt => (&mut min, quote!(Some(((#expr) as i32) + 1))),
                CmpOp::Le => (&mut max, quote!(Some((#expr) as i32))),
                CmpOp::Lt => (&mut max, quote!(Some(((#expr) as i32) - 1))),
                CmpOp::Eq | CmpOp::Ne => continue,
            };
            if slot.is_some() {
                return Err(syn::Error::new(
                    name.span(),
                    "two immediate bounds on the same side of one field",
                ));
            }
            *slot = Some(val);
        }
    }
    Ok((
        min.unwrap_or_else(|| quote!(None)),
        max.unwrap_or_else(|| quote!(None)),
    ))
}

/// RHS of a compare as `(is_reg, val_bits)` for a `CmpRule`: a `&path`
/// reference names another register (its addr const is already table-absolute,
/// widened by the helper to the LHS width+signedness -- pinned); any other
/// expression is the immediate's `i32` bits.
fn build_rule_val(expr: &Expr) -> (bool, TokenStream2) {
    if let Expr::Reference(r) = expr {
        let inner = &r.expr;
        (true, quote!((#inner) as u32))
    } else {
        (false, quote!(((#expr) as i32) as u32))
    }
}

/// Type-driven default for the auto-emitted `new()`. Primitives get their
/// zero value; arrays recurse; anything else falls through to `<Ty>::new()`,
/// which requires the field type to expose a `const fn new`.
pub(crate) fn default_init_for_type(ty: &Type) -> TokenStream2 {
    if let Type::Array(arr) = ty {
        let inner = default_init_for_type(&arr.elem);
        let len = &arr.len;
        return quote!([#inner; #len]);
    }
    if let Type::Path(tp) = ty
        && let Some(ident) = tp.path.get_ident()
    {
        let name = ident.to_string();
        if name == "bool" {
            return quote!(false);
        }
        if PRIMITIVE_INTS.contains(&name.as_str()) {
            return quote!(0);
        }
        if matches!(name.as_str(), "f32" | "f64") {
            return quote!(0.0);
        }
    }
    quote!(<#ty>::new())
}
