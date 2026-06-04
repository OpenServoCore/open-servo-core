#![no_std]

mod buf;
mod bulk;
mod bytes;
mod codec;
mod crc;
#[cfg(feature = "software-crc")]
mod crc_software;
mod fast;
mod frame;
mod instruction;
mod packet;
mod parser;
mod reply;
mod status_error;
mod writer;

pub mod prelude;

pub use buf::WriteBuf;
pub use bulk::{BulkReadSlotIter, BulkSlot, BulkSlotInfo, SyncSlotInfo};
pub use bytes::{ByteIter, Bytes, Overflow};
pub use codec::Codec;
pub use crc::CrcUmts;
#[cfg(feature = "software-crc")]
pub use crc_software::SoftwareCrcUmts;
pub use fast::{
    FAST_RESPONSE_SLOT_BYTES, FAST_RESPONSE_SLOT0_BYTES, FastBulkTupleIter, FastPosition,
    FastReadPacket, FastReadVariant, FastSlotInfo,
};
pub use frame::RawFrame;
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
pub use parser::ParseError;
pub use reply::StatusReply;
pub use status_error::StatusError;
pub use writer::WriteError;
