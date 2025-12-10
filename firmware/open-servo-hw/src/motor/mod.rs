//! Motor driver traits.
//!
//! These traits define the hardware abstraction for various motor types.

mod bdc;
mod bldc;

pub use bdc::*;
pub use bldc::*;
