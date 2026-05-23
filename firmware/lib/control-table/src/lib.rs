#![no_std]

mod desc;
mod route;
mod stage;
mod validate;

#[cfg(test)]
mod tests;

pub use desc::{
    Access, BOOL_ALLOWED, BlockDesc, BlockValidator, CompareOp, CrossField, FieldDesc,
    FieldValidator, RegionDesc, RegionValidator, RegmapError, ValidationKind,
};
pub use route::Router;
pub use stage::{STAGE_DATA_CAP, STAGE_ENTRY_CAP, StagedView, StagedWrites};

pub use control_table_derive::{Block, Enum, Region, Table};
