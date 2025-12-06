#![no_std]
#![forbid(unsafe_code)]

pub mod traits;
pub mod units;

#[cfg(feature = "pid")]
pub mod pid;

#[cfg(feature = "cascade")]
pub mod cascade;

// Re-export commonly used items
pub use traits::*;
pub use units::*;

#[cfg(feature = "pid")]
pub use pid::{PidController, PidConfig};