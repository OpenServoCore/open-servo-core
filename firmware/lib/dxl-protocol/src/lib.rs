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
    FAST_SLOT_PREFIX, FAST_SLOT0_PREFIX, FastBulkTupleIter, FastReadPacket, FastSlot, FastSlotBody,
    FastSlotInfo, FastSlotPosition, write_fast_slot,
};
pub use instruction::Instruction;
pub use packet::{
    ActionPacket, BROADCAST_ID, BulkReadPacket, BulkWritePacket, ClearPacket,
    ControlTableBackupPacket, FactoryResetPacket, FastBulkReadPacket, FastSyncReadPacket, HEADER,
    MAX_LENGTH, Packet, PingPacket, ReadPacket, RebootPacket, RegWritePacket, StatusPacket,
    SyncReadPacket, SyncWritePacket, WritePacket,
};
pub use parser::{ParseError, parse_one};
pub use status_error::StatusError;
pub use writer::{WriteError, write};
