//! Typed layer: per-instruction `Packet` structs, `Reply` shapes, and
//! decode/encode functions. Builds on `crate::wire::RawFrame` for parsing and
//! on `crate::wire::WriteBuf` for emission.

mod decoder;
mod instruction_ext;
mod fast;
mod instruction;
mod packet;
mod reply;
mod reply_ext;
mod slot;
mod status_error;
mod writer;

pub use decoder::{DecodeError, decode};
pub use instruction_ext::{InstructionExt, NoInstructionExt};
pub use fast::{FastBulkTupleIter, FastPosition, FastReadPacket, FastReadVariant, FastSlotInfo};
pub use instruction::Instruction;
pub use packet::{
    ActionPacket, BulkReadPacket, BulkWritePacket, ClearPacket, ControlTableBackupPacket,
    FactoryResetPacket, FastBulkReadPacket, FastSyncReadPacket, Packet, PingPacket, RawStatus,
    ReadPacket, RebootPacket, RegWritePacket, SyncReadPacket, SyncWritePacket, WritePacket,
};
pub use reply::{
    ActionReply, BulkReadReply, ErrorReply, FastBulkReadReply, FastErrorReply, FastSyncReadReply,
    PingReply, ReadReply, RebootReply, RegWriteReply, Reply, SyncReadReply, WriteReply,
};
pub use reply_ext::{NoReplyExt, ReplyExt};
pub use slot::{BulkReadSlotIter, BulkSlot, BulkSlotInfo, SyncSlotInfo};
pub use status_error::StatusError;

pub(crate) use reply::write_reply;
pub(crate) use writer::write;
