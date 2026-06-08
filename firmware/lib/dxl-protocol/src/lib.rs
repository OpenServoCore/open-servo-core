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

pub mod decoder;
mod emitter;
pub mod instruction;
pub mod packet;
mod wire;

pub use emitter::{InstructionEmitter, SlotEmitter, StatusEmitter};
pub use instruction::InstructionByte;
pub use packet::SlotPosition;
#[cfg(feature = "software-crc")]
pub use wire::SoftwareCrcUmts;
pub use wire::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, CRC_BYTES, CrcUmts, FAST_RESPONSE_SLOT_BYTES,
    FAST_RESPONSE_SLOT0_BYTES, HEADER, PACKET_LEN_GUARD, REQUEST_HEADER_BYTES,
    RESPONSE_HEADER_BYTES, SYNC_REQUEST_SLOT_BYTES, WriteBuf, WriteError,
};
