use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::punctuated::Punctuated;
use syn::spanned::Spanned;
use syn::token::PathSep;
use syn::{Attribute, Data, DeriveInput, Expr, ExprArray, Fields, Ident, Path, PathSegment, Type};

#[derive(Copy, Clone)]
enum AccessMode {
    Ro,
    Rw,
    Reserved,
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
            CmpOp::Lt => quote!(::control_table::CompareOp::Lt),
            CmpOp::Le => quote!(::control_table::CompareOp::Le),
            CmpOp::Gt => quote!(::control_table::CompareOp::Gt),
            CmpOp::Ge => quote!(::control_table::CompareOp::Ge),
            CmpOp::Eq => quote!(::control_table::CompareOp::Eq),
            CmpOp::Ne => quote!(::control_table::CompareOp::Ne),
        }
    }
}

struct FieldAttrs {
    skip: bool,
    access: AccessMode,
    allowed: Option<Expr>,
    custom: Vec<Path>,
    compares: Vec<(CmpOp, Expr)>,
    abs: bool,
    hook: Option<Path>,
}

impl Default for FieldAttrs {
    fn default() -> Self {
        Self {
            skip: false,
            access: AccessMode::Rw,
            allowed: None,
            custom: Vec::new(),
            compares: Vec::new(),
            abs: false,
            hook: None,
        }
    }
}

struct BlockAttrs {
    validators: Vec<TokenStream2>,
    hooks_trait: Option<Path>,
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
            "Block can only be derived for structs",
        ));
    };

    let Fields::Named(fields) = &s.fields else {
        return Err(syn::Error::new(
            input.span(),
            "Block requires a struct with named fields",
        ));
    };

    let block_attrs = parse_block_attrs(&input.attrs)?;
    let block_validators = &block_attrs.validators;

    let mut field_inits: Vec<TokenStream2> = Vec::new();
    let mut kept_idents: Vec<&Ident> = Vec::new();
    let mut kept_upper: Vec<Ident> = Vec::new();
    let mut new_inits: Vec<TokenStream2> = Vec::new();
    let mut hook_bindings: Vec<HookBinding> = Vec::new();
    for field in &fields.named {
        let name = field.ident.as_ref().unwrap();
        let ty = &field.ty;
        let attrs = parse_field_attrs(&field.attrs)?;

        if let Some(hook_path) = &attrs.hook {
            let resolved = resolve_hook_path(hook_path, block_attrs.hooks_trait.as_ref())?;
            hook_bindings.push(HookBinding {
                field_ident: name,
                field_ty: ty,
                trait_path: resolved.0,
                method_ident: resolved.1,
            });
        }

        let init = default_init_for_type(ty);
        new_inits.push(quote!(#name: #init));

        if attrs.skip {
            continue;
        }
        let access = match attrs.access {
            AccessMode::Ro => quote!(::control_table::Access::Ro),
            AccessMode::Rw => quote!(::control_table::Access::Rw),
            AccessMode::Reserved => quote!(::control_table::Access::Reserved),
        };
        let validators = build_validators(ty, &attrs)?;

        let rel_addr = quote!(::core::mem::offset_of!(#struct_ty, #name) as u16);
        let size = quote!(::core::mem::size_of::<#ty>() as u16);

        field_inits.push(quote! {
            ::control_table::FieldDesc {
                addr: #rel_addr,
                size: #size,
                struct_offset: #rel_addr,
                access: #access,
                validators: &[#(#validators),*],
            }
        });
        kept_upper.push(Ident::new(&name.to_string().to_uppercase(), name.span()));
        kept_idents.push(name);
    }

    let count = field_inits.len();
    let meta_macro = Ident::new(&meta_macro_name(struct_ty), struct_ty.span());
    let hooks_emit = build_hooks_emit(struct_ty, &hook_bindings);

    Ok(quote! {
        impl #struct_ty {
            pub const FIELDS: [::control_table::FieldDesc; #count] = [#(#field_inits),*];
            pub const DESC: ::control_table::BlockDesc = ::control_table::BlockDesc {
                addr: 0,
                size: ::core::mem::size_of::<Self>() as u16,
                struct_offset: 0,
                fields: &Self::FIELDS,
                validators: &[#(#block_validators),*],
            };
        }

        #[allow(clippy::new_without_default)]
        impl #struct_ty {
            pub const fn new() -> Self {
                Self { #(#new_inits),* }
            }
        }

        #hooks_emit

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
                // accept `packed` and `packed(N)`; reject either
                if m.input.peek(syn::token::Paren) {
                    let _: proc_macro2::Group = m.input.parse()?;
                }
                Err(m.error("Block derive forbids #[repr(packed)] (unaligned reads + offset_of are unsound)"))
            } else if m.path.is_ident("align") {
                // align(N) is fine; consume the (N)
                if m.input.peek(syn::token::Paren) {
                    let _: proc_macro2::Group = m.input.parse()?;
                }
                Ok(())
            } else {
                Err(m.error("Block derive requires #[repr(C)] and forbids alternate reprs"))
            }
        })?;
    }
    if !has_c {
        return Err(syn::Error::new(
            struct_span,
            "Block derive requires #[repr(C)]",
        ));
    }
    Ok(())
}

fn parse_block_attrs(attrs: &[Attribute]) -> syn::Result<BlockAttrs> {
    let mut out = BlockAttrs {
        validators: Vec::new(),
        hooks_trait: None,
    };
    for attr in attrs {
        if !attr.path().is_ident("ct_block") {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("validators") {
                let value = m.value()?;
                let array: ExprArray = value.parse()?;
                for elem in array.elems {
                    out.validators.push(quote!(#elem));
                }
                Ok(())
            } else if m.path.is_ident("hooks") {
                out.hooks_trait = Some(m.value()?.parse()?);
                Ok(())
            } else {
                Err(m.error(
                    "unknown ct_block key (expected `validators = [...]` or `hooks = TraitPath`)",
                ))
            }
        })?;
    }
    Ok(out)
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
            } else if m.path.is_ident("reserved") {
                out.access = AccessMode::Reserved;
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
                    other => {
                        return Err(syn::Error::new(
                            ident.span(),
                            format!("unknown access `{other}` (expected ro|rw)"),
                        ));
                    }
                };
                Ok(())
            } else if m.path.is_ident("allowed") {
                let val = m.value()?;
                out.allowed = Some(val.parse()?);
                Ok(())
            } else if m.path.is_ident("custom") {
                let val = m.value()?;
                out.custom.push(val.parse()?);
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

fn build_validators(ty: &Type, attrs: &FieldAttrs) -> syn::Result<Vec<TokenStream2>> {
    let mut out = Vec::new();

    if let Some(expr) = &attrs.allowed {
        out.push(quote!(::control_table::FieldValidator::EnumU8 { allowed: #expr }));
    } else if let Some(default) = default_enum_for_type(ty) {
        out.push(default);
    }

    if !attrs.compares.is_empty() {
        let variant = compare_variant_for_type(ty).ok_or_else(|| {
            syn::Error::new(
                ty.span(),
                "compare clauses require a primitive integer (u8/u16/i8/i16/i32)",
            )
        })?;
        let abs_lit = attrs.abs;
        for (op, expr) in &attrs.compares {
            let op_tok = op.token();
            let rhs = build_rhs(expr);
            out.push(quote! {
                ::control_table::FieldValidator::#variant {
                    op: #op_tok,
                    abs: #abs_lit,
                    rhs: #rhs,
                }
            });
        }
    }

    for path in &attrs.custom {
        out.push(quote!(::control_table::FieldValidator::Custom(#path)));
    }

    Ok(out)
}

/// Array / tuple / reference types fall through `Type::Path` and get no default
/// validator; user must opt in via `allowed = …` or `custom = …`.
fn default_enum_for_type(ty: &Type) -> Option<TokenStream2> {
    let Type::Path(tp) = ty else { return None };
    let path = &tp.path;
    if let Some(ident) = path.get_ident() {
        let name = ident.to_string();
        if name == "bool" {
            return Some(quote!(::control_table::FieldValidator::EnumU8 {
                allowed: ::control_table::BOOL_ALLOWED
            }));
        }
        if is_primitive(&name) {
            return None;
        }
    }
    Some(quote!(::control_table::FieldValidator::EnumU8 {
        allowed: <#path as ::control_table::HasAllowed>::ALLOWED
    }))
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

fn compare_variant_for_type(ty: &Type) -> Option<Ident> {
    let Type::Path(tp) = ty else { return None };
    let ident = tp.path.get_ident()?;
    let name = match ident.to_string().as_str() {
        "u8" => "CompareU8",
        "u16" => "CompareU16",
        "i8" => "CompareI8",
        "i16" => "CompareI16",
        "i32" => "CompareI32",
        _ => return None,
    };
    Some(Ident::new(name, ident.span()))
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
        if matches!(
            name.as_str(),
            "u8" | "u16"
                | "u32"
                | "u64"
                | "u128"
                | "i8"
                | "i16"
                | "i32"
                | "i64"
                | "i128"
                | "usize"
                | "isize"
        ) {
            return quote!(0);
        }
        if matches!(name.as_str(), "f32" | "f64") {
            return quote!(0.0);
        }
    }
    quote!(<#ty>::new())
}

/// `__ct_meta_<crate>_<Block>` — the `<crate>` prefix prevents name collisions
/// at the consumer's crate root when two crates each derive a Block with the
/// same name. Region derive must compute the same name for its invocation.
pub(crate) fn meta_macro_name(struct_ty: &Ident) -> String {
    let crate_name = std::env::var("CARGO_CRATE_NAME").unwrap_or_default();
    let crate_part = crate_name.replace('-', "_");
    format!("__ct_meta_{crate_part}_{struct_ty}")
}

fn build_rhs(expr: &Expr) -> TokenStream2 {
    if let Expr::Reference(r) = expr {
        let inner = &r.expr;
        quote!(::control_table::Rhs::Addr(#inner))
    } else {
        quote!(::control_table::Rhs::Value(#expr))
    }
}
