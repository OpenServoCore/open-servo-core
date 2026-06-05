//! Typed layer: per-instruction `Packet` structs, `Status` shapes, and
//! decode/encode functions. Builds on `crate::wire::RawFrame` for parsing and
//! on `crate::wire::WriteBuf` for emission.

mod decoder;
mod instruction_ext;
mod fast;
mod instruction;
mod packet;
mod status;
mod status_ext;
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
pub use status::{
    ActionStatus, BulkReadStatus, ErrorStatus, FastBulkReadStatus, FastErrorStatus, FastSyncReadStatus,
    PingStatus, ReadStatus, RebootStatus, RegWriteStatus, Status, SyncReadStatus, WriteStatus,
};
pub use status_ext::{NoStatusExt, StatusExt};
pub use slot::{BulkReadSlotIter, BulkSlot, BulkSlotInfo, SyncSlotInfo};
pub use status_error::StatusError;

pub(crate) use status::write_status;
pub(crate) use writer::write;
