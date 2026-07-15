//! osc-native wire protocol (`docs/osc-native-protocol.md`).
//!
//! Layout types over ring memory plus pure span math. The break anchors a
//! frame and the header makes its end computable, so there is no streaming
//! parser here: the chip owns timing (framer deadlines, DMA, CRC engine),
//! this crate owns layout -- `#[repr(C)]` views, offset arithmetic, and the
//! osc-CRC-16 definition.
#![no_std]

pub mod build;
pub mod bytes;
pub mod crc;
pub mod frame;
pub mod group;
pub mod models;
pub mod reply;
pub mod table;
pub mod wire;

pub use bytes::FrameBytes;
