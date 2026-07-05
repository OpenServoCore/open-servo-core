#![no_std]
//! DXL 2.0 wire-format codec.
//!
//! [`frame::parse`] classifies and validates a frame at the head of a byte
//! slice; [`types::packet`] decodes it into typed instruction / Status
//! shapes. The [`encode`] emitters produce the three frame shapes in a single
//! fused pass over a caller-owned buffer.

mod buf;
pub mod crc;
pub mod encode;
pub mod frame;
pub mod log;
pub mod types;
pub mod unstuff;
pub mod wire;

pub use buf::{Chunk, WriteError};
#[cfg(feature = "software-crc")]
pub use crc::SoftwareCrcUmts;
pub use crc::{CrcUmts, crc16_umts_continue};
pub use encode::{
    encode_instruction, encode_slot, encode_slot_chunked, encode_status, encode_status_chunked,
};
pub use frame::{FrameKind, ParseError, Probe, RawFrame, parse, probe};
pub use types::{
    BulkReadEntry, ErrorCode, Id, Instruction, PingStatus, Slot, SlotPosition, Status, StatusError,
};
pub use unstuff::Bytes;
