//! Driver-owned interfaces. Drivers declare what they need from their
//! environment; chip-side adapters implement these over real HAL peripherals
//! (production) or recording mocks (tests).

mod digital_out;
mod monotonic;

pub use digital_out::{DigitalOut, Level};
pub use monotonic::Monotonic;
