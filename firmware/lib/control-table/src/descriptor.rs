//! Machine-readable field descriptors emitted alongside the runtime consts by
//! the Block/Section/Table derives. A host-side exporter walks a table's
//! `FIELDS` to produce a device-description JSON.
//!
//! These consts cost the flash image nothing when firmware never references
//! them: a Rust const is lazy, materialized only where it is used, so the
//! exporter-only descriptor data never lands in the servo binary. No feature
//! gate is needed.

/// One variant of an `Enum`-derived field, name paired with its discriminant.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EnumVariant {
    pub name: &'static str,
    pub value: u8,
}

/// Field value shape. `Enum` carries the deriving type's variants so the
/// exporter can render symbolic names.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FieldKind {
    UInt,
    Int,
    Bool,
    Enum(&'static [EnumVariant]),
    Bytes,
}

/// One field of a control table: identity, table-absolute placement, access,
/// value shape, and inclusive scalar bounds from immediate compare rules.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct FieldDesc {
    pub name: &'static str,
    /// Table-absolute after the Section rebase.
    pub addr: u16,
    /// Field width in bytes.
    pub width: u16,
    pub writable: bool,
    pub kind: FieldKind,
    /// Inclusive, from immediate compare rules only (register-RHS and abs
    /// bounds do not export).
    pub min: Option<i32>,
    pub max: Option<i32>,
}

impl FieldDesc {
    /// Zero-fill element for the const concat arrays, mirroring the rules
    /// arrays' filler.
    pub const EMPTY: FieldDesc = FieldDesc {
        name: "",
        addr: 0,
        width: 0,
        writable: false,
        kind: FieldKind::UInt,
        min: None,
        max: None,
    };
}
