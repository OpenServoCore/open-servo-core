//! Motor driver traits.
//!
//! These traits define the hardware abstraction for various motor types.

mod bdc;
#[cfg(feature = "motor-bldc")]
mod bldc;

pub use bdc::*;
#[cfg(feature = "motor-bldc")]
pub use bldc::*;
