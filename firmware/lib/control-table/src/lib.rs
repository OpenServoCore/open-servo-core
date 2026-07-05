#![no_std]
#![cfg_attr(feature = "sync-unsafe-cell", feature(sync_unsafe_cell))]

mod desc;
pub mod map;
mod region;
mod route;
pub mod rules;
mod stage;
mod validate;

#[cfg(test)]
mod tests;

pub use desc::{
    Access, BOOL_ALLOWED, BlockDesc, BlockValidator, CompareOp, Error, FieldDesc, FieldValidator,
    HasAllowed, RegionDesc, RegionValidator, Rhs, ValidationKind,
};
pub use region::{Region, RegionStorage, RegionStorageRaw};
pub use route::{ReadChunk, ReadIter, Router};
pub use stage::{STAGE_DATA_CAP, STAGE_ENTRY_CAP, Snapshot, StagedView, StagedWrites};

pub use control_table_derive::{Block, Enum, Region, Table};
