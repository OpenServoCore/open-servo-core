//! DXL 2.0 wire-format codec -- packet-level encode + decode.
//!
//! [`Decoder`] consumes wire bytes and yields typed [`Packet`](crate::packet::Packet)
//! overlays; [`InstructionEncoder`], [`StatusEncoder`], and [`SlotEncoder`]
//! emit the three frame shapes.

pub mod decoder;
pub mod encoder;

pub use decoder::{Decoder, ResyncKind, Step};
pub use encoder::{InstructionEncoder, SlotEncoder, StatusEncoder};
