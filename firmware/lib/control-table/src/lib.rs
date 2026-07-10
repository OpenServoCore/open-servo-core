#![no_std]
#![cfg_attr(feature = "sync-unsafe-cell", feature(sync_unsafe_cell))]

pub mod map;
mod region;
pub mod rules;
mod stage;

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
    Compare,
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

pub use region::{Region, RegionStorage, RegionStorageRaw};
pub use stage::{STAGE_DATA_CAP, STAGE_ENTRY_CAP, Snapshot, StagedWrites};

pub use control_table_derive::{Block, Enum, Section, Table};

pub use map::{RegisterFile, RegisterMap, SectionMeta, View};
