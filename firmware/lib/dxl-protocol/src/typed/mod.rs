//! Typed layer: per-instruction `Packet` structs, `StatusReply` shapes, and
//! decode/encode functions. Builds on `crate::wire::RawFrame` for parsing and
//! on `crate::wire::WriteBuf` for emission.

mod bulk;
mod decoder;
mod extension;
mod fast;
mod instruction;
mod packet;
mod reply;
mod reply_ext;
mod status_error;
mod writer;

pub use bulk::{BulkReadSlotIter, BulkSlot, BulkSlotInfo, SyncSlotInfo};
pub use decoder::{DecodeError, decode};
pub use extension::{Extension, NoExt};
pub use fast::{FastBulkTupleIter, FastPosition, FastReadPacket, FastReadVariant, FastSlotInfo};
pub use instruction::Instruction;
pub use packet::{
    ActionPacket, BulkReadPacket, BulkWritePacket, ClearPacket, ControlTableBackupPacket,
    FactoryResetPacket, FastBulkReadPacket, FastSyncReadPacket, Packet, PingPacket, ReadPacket,
    RebootPacket, RegWritePacket, StatusPacket, SyncReadPacket, SyncWritePacket, WritePacket,
};
pub use reply::StatusReply;
pub use reply_ext::{NoReplyExt, ReplyExt};
pub use status_error::StatusError;

pub(crate) use reply::write_status_reply;
pub(crate) use writer::write;
