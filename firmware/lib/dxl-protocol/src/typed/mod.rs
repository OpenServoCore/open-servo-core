//! Typed layer: per-instruction `Packet` structs, `Status` shapes, the
//! `Slot` wire-piece, and decode/encode functions. Builds on `crate::wire`
//! for parsing primitives and emission.

mod decoder;
mod fast;
mod fast_decoder;
mod instruction;
mod instruction_ext;
mod packet;
mod slot;
mod status;
mod status_decoder;
mod status_error;
mod status_ext;
mod writer;

pub use decoder::{DecodeError, decode};
pub use fast::{
    FastBulkTupleIter, FastReadPacket, FastReadVariant, FastSlotInfo, SlotPosition, write_slot,
};
pub use fast_decoder::{FastBulkReadStatus, FastBulkSlotIter, FastSyncReadStatus, FastSyncSlotIter};
pub use instruction::Instruction;
pub use instruction_ext::{InstructionExt, NoInstructionExt};
pub use packet::{
    ActionPacket, BulkReadPacket, BulkWritePacket, ClearPacket, ControlTableBackupPacket,
    FactoryResetPacket, FastBulkReadPacket, FastSyncReadPacket, Packet, PingPacket, RawStatus,
    ReadPacket, RebootPacket, RegWritePacket, SyncReadPacket, SyncWritePacket, WritePacket,
};
pub use slot::{BulkReadSlotIter, BulkSlot, BulkSlotInfo, Slot, SyncSlotInfo};
pub use status::{
    ActionStatus, BulkReadStatus, ErrorStatus, PingStatus, ReadStatus, RebootStatus,
    RegWriteStatus, Status, SyncReadStatus, WriteStatus, write_status,
};
pub use status_decoder::decode_status;
pub use status_error::StatusError;
pub use status_ext::{NoStatusExt, StatusExt};
pub use writer::write_packet;
