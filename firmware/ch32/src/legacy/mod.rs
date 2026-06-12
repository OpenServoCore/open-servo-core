//! Modules awaiting refactor into the driver-pattern shape. Quarantined here
//! to make the boundary visible; pieces are extracted out (or deleted) as the
//! new drivers / providers / services land.

pub(crate) mod bench;
pub(crate) mod chip_flash;
// dxl + idle_anchor: callers from `services::bus` and `services::events`
// were removed in M2 commit C; the modules themselves still feed the
// chip-side TX scheduler stub and IRQ vector bodies. M2 commit D drains
// the dead interior. Suppress dead-code in the meantime so the inter-commit
// CI gate stays green.
#[allow(dead_code)]
pub(crate) mod dxl;
#[allow(dead_code)]
pub(crate) mod idle_anchor;
pub(crate) mod statics;
#[cfg(feature = "defmt")]
pub(crate) mod telemetry;
