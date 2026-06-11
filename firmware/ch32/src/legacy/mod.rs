//! Modules awaiting refactor into the driver-pattern shape. Quarantined here
//! to make the boundary visible; pieces are extracted out (or deleted) as the
//! new drivers / providers / services land.

pub(crate) mod bench;
pub(crate) mod chip_flash;
pub(crate) mod dxl;
pub(crate) mod idle_anchor;
pub(crate) mod statics;
#[cfg(feature = "defmt")]
pub(crate) mod telemetry;
