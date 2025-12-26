//! Simple integer-based units for servo control
//!
//! All units use integer scaling to avoid floating point and minimize binary size.
//! The scaling factors are chosen to provide sufficient precision for servo control
//! while fitting comfortably in i16/u16 types.

#![no_std]

mod macros;

mod adc12;
mod centideg;
mod centideg32;
mod centic;
mod deg_per_sec10;
mod duty;
mod encoder_count;
mod helpers;
mod milliamp;
mod millivolt;

pub use adc12::Adc12;
pub use centideg::CentiDeg;
pub use centideg32::CentiDeg32;
pub use centic::CentiC;
pub use deg_per_sec10::DegPerSec10;
pub use duty::Duty;
pub use encoder_count::EncoderCount;
pub use helpers::mul_div_i32;
pub use milliamp::MilliAmp;
pub use millivolt::MilliVolt;
