//! Mathematical primitives for servo control
//!
//! This crate provides:
//! - Integer-based units with saturating arithmetic
//! - Fixed-point PID controller with compile-time optimization
//! - Gain type for float-free PID tuning

#![no_std]

pub mod gain;
pub mod pid;
pub mod units;

pub use gain::*;
pub use pid::*;
pub use units::*;
