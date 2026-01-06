//! Simple integer-based units for servo control
//!
//! All units use integer scaling to avoid floating point and minimize binary size.
//! The scaling factors are chosen to provide sufficient precision for servo control
//! while fitting comfortably in i16/u16 types.

#![no_std]

mod macros;

mod adc12;
mod centic;
mod centideg;
mod centideg32;
mod deg_per_sec10;
mod effort;
mod encoder_count;
mod helpers;
mod hertz;
mod microsecond;
mod milliamp;
mod millivolt;
mod timestamp;

pub use adc12::Adc12;
pub use centic::CentiC;
pub use centideg::CentiDeg;
pub use centideg32::CentiDeg32;
pub use deg_per_sec10::DegPerSec10;
pub use effort::Effort;
pub use encoder_count::EncoderCount;
pub use helpers::{
    div_round_i32, div_round_u32, mul_div_round_i32, mul_div_round_i64, mul_div_round_u32,
    mul_div_round_u64,
};
pub use hertz::Hertz;
pub use microsecond::MicroSecond;
pub use milliamp::MilliAmp;
pub use millivolt::MilliVolt;
pub use timestamp::TimeStampUs;
