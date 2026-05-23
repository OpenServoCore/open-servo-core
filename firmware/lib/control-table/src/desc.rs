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
