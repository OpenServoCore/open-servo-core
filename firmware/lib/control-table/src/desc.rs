use crate::stage::StagedView;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    OutOfRange,
    AccessError,
    StagingFull,
    ValidationError(ValidationKind),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ValidationKind {
    Enum,
    Range,
    Compare,
    Locked,
    Custom,
}

/// `bool` is `u8` 0/1; any other byte yields UB on later access.
pub const BOOL_ALLOWED: &[u8] = &[0, 1];

#[diagnostic::on_unimplemented(
    message = "`{Self}` does not expose an `ALLOWED` discriminant slice",
    label = "missing `#[derive(control_table::Enum)]`",
    note = "add `#[derive(Enum)]` and `#[repr(u8)]` to `{Self}` to generate `ALLOWED`"
)]
pub trait HasAllowed {
    const ALLOWED: &'static [u8];
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Access {
    Ro,
    Rw,
    /// Address slot is part of the table layout but has no backing storage:
    /// reads zero-fill, writes return `AccessError`. Lets a build-disabled
    /// field hold its byte offset without exposing live state.
    Reserved,
}

/// `struct_offset` is block-relative; the block's own offset is added during walks.
#[derive(Copy, Clone, Debug)]
pub struct FieldDesc {
    pub addr: u16,
    pub size: u16,
    pub struct_offset: u16,
    pub access: Access,
    pub validators: &'static [FieldValidator],
}

/// `size` is `size_of::<Self>()` and may exceed the sum of `fields` (skipped fields
/// stay in the layout for padding); walks return `AccessError` inside those gaps.
#[derive(Copy, Clone, Debug)]
pub struct BlockDesc {
    pub addr: u16,
    pub size: u16,
    pub struct_offset: u16,
    pub fields: &'static [FieldDesc],
    pub validators: &'static [BlockValidator],
}

#[derive(Copy, Clone, Debug)]
pub struct RegionDesc {
    pub addr: u16,
    pub size: u16,
    pub blocks: &'static [BlockDesc],
    pub validators: &'static [RegionValidator],
}

pub type BlockValidator = fn(&StagedView, u16, u16) -> Result<(), Error>;
pub type RegionValidator = fn(&StagedView) -> Result<(), Error>;

#[derive(Copy, Clone, Debug)]
pub enum FieldValidator {
    EnumU8 {
        allowed: &'static [u8],
    },
    CompareU8 {
        op: CompareOp,
        abs: bool,
        rhs: Rhs<u8>,
    },
    CompareU16 {
        op: CompareOp,
        abs: bool,
        rhs: Rhs<u16>,
    },
    CompareI8 {
        op: CompareOp,
        abs: bool,
        rhs: Rhs<i8>,
    },
    CompareI16 {
        op: CompareOp,
        abs: bool,
        rhs: Rhs<i16>,
    },
    CompareI32 {
        op: CompareOp,
        abs: bool,
        rhs: Rhs<i32>,
    },
    Custom(fn(&StagedView, u16, u16) -> Result<(), Error>),
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Rhs<T: Copy> {
    Value(T),
    Addr(u16),
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CompareOp {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
}

#[cfg(feature = "defmt")]
impl defmt::Format for FieldValidator {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::EnumU8 { allowed } => defmt::write!(f, "EnumU8 {{ allowed: {} }}", allowed),
            Self::CompareU8 { op, abs, rhs } => {
                defmt::write!(f, "CompareU8 {{ op: {}, abs: {}, rhs: {} }}", op, abs, rhs)
            }
            Self::CompareU16 { op, abs, rhs } => {
                defmt::write!(f, "CompareU16 {{ op: {}, abs: {}, rhs: {} }}", op, abs, rhs)
            }
            Self::CompareI8 { op, abs, rhs } => {
                defmt::write!(f, "CompareI8 {{ op: {}, abs: {}, rhs: {} }}", op, abs, rhs)
            }
            Self::CompareI16 { op, abs, rhs } => {
                defmt::write!(f, "CompareI16 {{ op: {}, abs: {}, rhs: {} }}", op, abs, rhs)
            }
            Self::CompareI32 { op, abs, rhs } => {
                defmt::write!(f, "CompareI32 {{ op: {}, abs: {}, rhs: {} }}", op, abs, rhs)
            }
            Self::Custom(_) => defmt::write!(f, "Custom(<fn>)"),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FieldDesc {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "FieldDesc {{ addr: {}, size: {}, struct_offset: {}, access: {}, validators: {} }}",
            self.addr,
            self.size,
            self.struct_offset,
            self.access,
            self.validators,
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for BlockDesc {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "BlockDesc {{ addr: {}, size: {}, struct_offset: {}, fields: {}, validators: <{} fn> }}",
            self.addr,
            self.size,
            self.struct_offset,
            self.fields,
            self.validators.len(),
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for RegionDesc {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "RegionDesc {{ addr: {}, size: {}, blocks: {}, validators: <{} fn> }}",
            self.addr,
            self.size,
            self.blocks,
            self.validators.len(),
        )
    }
}
