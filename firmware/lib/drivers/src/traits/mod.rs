//! Driver-owned interfaces. Drivers declare what they need from their
//! environment; chip-side adapters implement these over real HAL peripherals
//! (production) or recording mocks (tests).
//!
//! Transport-shaped traits live under [`dxl`]; the root holds only
//! cross-driver primitives ([`DigitalOut`], [`Monotonic`]).

pub mod dxl;

use crate::types::Level;

/// Drive a single digital output. Owned by the adapter; the driver holds
/// one `P: DigitalOut` and calls `set` to change the wire state.
pub trait DigitalOut {
    fn set(&mut self, level: Level);
}

/// Free-running monotonic tick counter, used by drivers that schedule by
/// elapsed time. `TICKS_PER_US` describes the rate so the driver can
/// convert µs ↔ ticks without importing chip constants.
pub trait Monotonic {
    const TICKS_PER_US: u32;
    fn ticks(&self) -> u32;
}
