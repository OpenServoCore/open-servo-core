#![no_std]

mod buf;
mod bytes;
mod crc;
mod instruction;
mod packet;
mod parser;
mod status_error;
mod writer;

pub mod prelude;

pub use buf::WriteBuf;
pub use bytes::{ByteIter, Bytes, Overflow};
pub use crc::crc16;
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
