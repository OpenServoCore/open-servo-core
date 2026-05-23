use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::spanned::Spanned;
use syn::{Attribute, Data, DeriveInput, Expr, ExprArray, Fields, Ident, Path, Type};

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
        }
    }
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

    let block_validators = parse_block_validators(&input.attrs)?;

    let mut field_inits: Vec<TokenStream2> = Vec::new();
    let mut kept_idents: Vec<&Ident> = Vec::new();
    let mut kept_upper: Vec<Ident> = Vec::new();
    for field in &fields.named {
        let name = field.ident.as_ref().unwrap();
        let ty = &field.ty;
        let attrs = parse_field_attrs(&field.attrs)?;
        if attrs.skip {
            continue;
        }
        let access = match attrs.access {
            AccessMode::Ro => quote!(::control_table::Access::Ro),
            AccessMode::Rw => quote!(::control_table::Access::Rw),
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
    let meta_macro = Ident::new(&format!("__ct_meta_{struct_ty}"), struct_ty.span());

    Ok(quote! {
        impl #struct_ty {
            pub const FIELDS_AT_ZERO_ARR: [::control_table::FieldDesc; #count] =
                [#(#field_inits),*];
            pub const FIELDS_AT_ZERO: &'static [::control_table::FieldDesc] =
                &Self::FIELDS_AT_ZERO_ARR;
            pub const FIELD_COUNT: usize = #count;
            pub const SIZE: u16 = ::core::mem::size_of::<Self>() as u16;
            pub const VALIDATORS: &'static [::control_table::BlockValidator] =
                &[#(#block_validators),*];
        }

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

fn parse_block_validators(attrs: &[Attribute]) -> syn::Result<Vec<TokenStream2>> {
    let mut out = Vec::new();
    for attr in attrs {
        if !attr.path().is_ident("ct_block") {
            continue;
        }
        attr.parse_nested_meta(|m| {
            if m.path.is_ident("validators") {
                let value = m.value()?;
                let array: ExprArray = value.parse()?;
                for elem in array.elems {
                    out.push(quote!(#elem));
                }
                Ok(())
            } else {
                Err(m.error("unknown ct_block key (expected `validators = [...]`)"))
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

fn build_rhs(expr: &Expr) -> TokenStream2 {
    if let Expr::Reference(r) = expr {
        let inner = &r.expr;
        quote!(::control_table::Rhs::Addr(#inner))
    } else {
        quote!(::control_table::Rhs::Value(#expr))
    }
}
