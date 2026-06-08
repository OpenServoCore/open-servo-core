//! Typed layer: per-instruction `Packet` structs, `Status` shapes, the
//! `Slot` wire-piece, and decode/encode functions. Builds on `crate::wire`
//! for parsing primitives and emission.

mod bulk;
mod ext;
mod fast;
mod packet;
mod packet_decoder;
mod packet_encoder;
mod status;
mod status_decoder;
mod status_encoder;
mod status_error;

pub use bulk::{
    BulkEntry, BulkReadSlotIter, BulkSlotInfo, BulkWriteEntry, BulkWriteSlotIter, SyncSlotInfo,
    SyncWriteEntry, SyncWriteSlotIter,
};
pub use crate::instruction::Instruction;
pub use ext::{InstructionExt, NoInstructionExt, NoStatusExt, StatusExt};
pub use fast::{FastReadPacket, FastReadVariant, FastSlotInfo, SlotPosition};
pub use packet::{
    ActionPacket, BulkReadPacket, BulkWritePacket, ClearPacket, ControlTableBackupPacket,
    FactoryResetPacket, FastBulkReadPacket, FastSyncReadPacket, Packet, PingPacket, RawStatus,
    ReadPacket, RebootPacket, RegWritePacket, SyncReadPacket, SyncWritePacket, WritePacket,
};
pub use packet_decoder::DecodeError;
pub(crate) use packet_decoder::decode;
pub use packet_encoder::{write_ext, write_packet};
pub use status::{
    ActionStatus, BulkReadStatus, ErrorStatus, FastBulkReadStatus, FastBulkSlotIter,
    FastSyncReadStatus, FastSyncSlotIter, PingStatus, ReadStatus, RebootStatus, RegWriteStatus,
    Slot, Status, SyncReadStatus, WriteStatus,
};
pub use status_decoder::decode_status;
pub use status_encoder::{write_slot, write_status};
pub use status_error::StatusError;
