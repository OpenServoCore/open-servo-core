//! Mathematical primitives for servo control
//!
//! This crate provides:
//! - Integer-based units with saturating arithmetic
//! - Fixed-point PID controller with compile-time optimization
//! - Gain type for float-free PID tuning
//! - NTC thermistor temperature conversion
//! - Low-pass filters for sensor data
//! - Thermal model for resistive heating/cooling

#![no_std]

pub mod filter;
pub mod gain;
pub mod ntc;
pub mod ntc_gen;
pub mod pid;
pub mod thermal;
pub mod units;

pub use filter::{FilterU16, FilterI16, FilterI32};
pub use gain::*;
pub use ntc::*;
pub use ntc_gen::{generate_ntc_lut, presets};
pub use pid::*;
pub use thermal::ThermalModel;
pub use units::*;
