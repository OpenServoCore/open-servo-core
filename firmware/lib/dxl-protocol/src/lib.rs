#![no_std]

mod buf;
mod bytes;
mod crc;
mod fast;
mod instruction;
mod packet;
mod parser;
mod status_error;
mod writer;

pub mod prelude;

pub use buf::WriteBuf;
pub use bytes::{ByteIter, Bytes, Overflow};
pub use crc::{crc16, crc16_continue};
pub use fast::{
    FAST_RESPONSE_SLOT0_BYTES, FAST_RESPONSE_SLOT_BYTES, FastBulkTupleIter, FastReadPacket,
    FastSlot, FastSlotBody, FastSlotInfo, FastSlotPosition, write_fast_slot,
};
pub use instruction::Instruction;
#[cfg(feature = "osc")]
pub use packet::CalibratePacket;
pub use packet::{
    ActionPacket, BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, BulkReadPacket, BulkWritePacket,
    CRC_BYTES, ClearPacket, ControlTableBackupPacket, FactoryResetPacket, FastBulkReadPacket,
    FastSyncReadPacket, HEADER, MAX_LENGTH, Packet, PingPacket, REQUEST_HEADER_BYTES,
    RESPONSE_HEADER_BYTES, ReadPacket, RebootPacket, RegWritePacket, SYNC_REQUEST_SLOT_BYTES,
    StatusPacket, SyncReadPacket, SyncWritePacket, WritePacket,
};
pub use parser::{ParseError, parse_one};
pub use status_error::StatusError;
pub use writer::{WriteError, write};
