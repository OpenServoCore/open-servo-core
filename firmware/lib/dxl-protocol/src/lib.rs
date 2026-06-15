#![no_std]
//! DXL 2.0 wire-format codec.
//!
//! [`streaming::Parser`] walks wire bytes and emits typed
//! [`Event`](streaming::Event)s per protocol field. [`InstructionEncoder`],
//! [`StatusEncoder`], and [`SlotEncoder`] emit the three frame shapes.

mod buf;
pub mod codec;
mod constants;
mod crc;
#[cfg(feature = "software-crc")]
mod crc_software;
pub mod packet;
pub mod streaming;

pub use buf::{WriteBuf, WriteError};
pub use codec::{InstructionEncoder, SlotEncoder, StatusEncoder};
pub use constants::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, CRC_BYTES, FAST_RESPONSE_SLOT_BYTES,
    FAST_RESPONSE_SLOT0_BYTES, HEADER, PACKET_LEN_GUARD, REQUEST_HEADER_BYTES,
    RESPONSE_HEADER_BYTES, SYNC_REQUEST_SLOT_BYTES,
};
pub use crc::CrcUmts;
#[cfg(feature = "software-crc")]
pub use crc_software::SoftwareCrcUmts;
pub use packet::{Id, Instruction, InstructionByte, InstructionPacket, Packet, SlotPosition};
