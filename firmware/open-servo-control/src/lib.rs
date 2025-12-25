#![no_std]
#![forbid(unsafe_code)]

pub mod traits;

#[cfg(feature = "pid")]
pub mod pid;

#[cfg(feature = "cascade")]
pub mod cascade;

// Re-export units from the dedicated math crate for backwards compatibility
pub use open_servo_math::*;

// Re-export commonly used items
pub use traits::{ControlInput, ControlLoop, ControlOutput, DutyLimits};

#[cfg(feature = "pid")]
pub use traits::PidTunable;

#[cfg(feature = "pid")]
pub use pid::{PidConfig, PidController};
