#![no_std]
#![forbid(unsafe_code)]

pub mod traits;

#[cfg(feature = "pid")]
pub mod pid;

#[cfg(feature = "cascade")]
pub mod cascade;

// Re-export commonly used items
pub use traits::*;

#[cfg(feature = "pid")]
pub use pid::{PidController, PidConfig};