//! DXL 2.0 wire-format encoder.
//!
//! [`InstructionEncoder`], [`StatusEncoder`], and [`SlotEncoder`] emit the
//! three frame shapes. Wire decoding lives in [`crate::streaming`].

pub mod encoder;

pub use encoder::{InstructionEncoder, SlotEncoder, StatusEncoder};
