#![no_std]
//! DXL 2.0 wire-format codec.
//!
//! Decode: feed wire bytes to a [`Decoder`](decoder::Decoder); it emits typed
//! [`Packet`](packet::Packet) variants that overlay the decoder's internal
//! buffer (zero-copy, alignment-1 `#[repr(C)]` structs).
//!
//! Encode: [`InstructionEmitter`] for request frames, [`StatusEmitter`] for
//! Status replies, [`SlotEmitter`] for Fast Sync/Bulk Read coalesced reply
//! slots.

mod buf;
mod constants;
mod crc;
#[cfg(feature = "software-crc")]
mod crc_software;
pub mod decoder;
mod emitter;
pub mod instruction;
pub mod packet;

pub use buf::{WriteBuf, WriteError};
pub use constants::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, CRC_BYTES, FAST_RESPONSE_SLOT0_BYTES,
    FAST_RESPONSE_SLOT_BYTES, HEADER, PACKET_LEN_GUARD, REQUEST_HEADER_BYTES,
    RESPONSE_HEADER_BYTES, SYNC_REQUEST_SLOT_BYTES,
};
pub use crc::CrcUmts;
#[cfg(feature = "software-crc")]
pub use crc_software::SoftwareCrcUmts;
pub use emitter::{InstructionEmitter, SlotEmitter, StatusEmitter};
pub use instruction::InstructionByte;
pub use packet::SlotPosition;
