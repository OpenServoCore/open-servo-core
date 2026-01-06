//! Sensor capability traits for servo control.
//!
//! These traits define the hardware abstraction for various sensors.
//! Boards implement these traits to provide sensor readings to the control core.
//!
//! ## Safety Capability Traits
//!
//! Each sensor type has a corresponding "Safety*Source" trait that returns `Option<T>`.
//! Boards that have the sensor implement the base trait (e.g., `BusCurrentSensor`),
//! and automatically get the safety trait via blanket impl.
//!
//! Boards without a sensor implement only the safety trait, returning `None`.
//! This allows SafetyManager to skip checks for unavailable sensors.

mod current;
mod position;
mod temperature;
mod velocity;
mod voltage;

pub use current::*;
pub use position::*;
pub use temperature::*;
pub use velocity::*;
pub use voltage::*;
