//! Driver-owned interfaces. Drivers declare what they need from their
//! environment; chip-side adapters implement these over real HAL peripherals
//! (production) or recording mocks (tests).
//!
//! Transport-shaped traits live under [`dxl`]; the root holds only
//! cross-driver primitives ([`DigitalOut`], [`Monotonic`]).

mod digital_out;
mod monotonic;

pub mod dxl;

pub use digital_out::{DigitalOut, Level};
pub use monotonic::Monotonic;
