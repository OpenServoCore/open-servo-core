//! Overlay types. Every `#[repr(C)]` struct mirrors the on-the-wire
//! (unstuffed) layout at alignment 1, so the decoder can cast a `&[u8]`
//! accumulator into any of them at any offset.

mod entries;
mod header;
pub mod instruction;
mod slot;
mod status;

pub use entries::{
    BulkReadEntry, BulkWriteEntries, BulkWriteEntry, SyncWriteEntries, SyncWriteEntry,
};
pub use header::{Header, U16Le};
pub use instruction::{
    ActionPacket, BulkReadHeader, BulkReadPacket, BulkWriteHeader, BulkWritePacket,
    FactoryResetPacket, FastBulkReadHeader, FastBulkReadPacket, FastSyncReadHeader,
    FastSyncReadPacket, Instruction, InstructionByte, PingPacket, RawHeader, RawPacket, ReadPacket,
    RebootPacket, SyncReadHeader, SyncReadPacket, SyncWriteHeader, SyncWritePacket, WriteHeader,
    WritePacket,
};
pub use slot::{BulkSlotInfo, FastSlotInfo, SlotPosition, SyncSlotInfo};
pub use status::{
    ErrorCode, FastBulkReadStatus, FastBulkSlotIter, FastSyncReadStatus, FastSyncSlotIter,
    PingStatus, RequestKind, Slot, Status, StatusError, StatusHeader, StatusPacket,
};

#[derive(Copy, Clone, Debug)]
pub enum Packet<'a> {
    Ping(&'a PingPacket),
    Read(&'a ReadPacket),
    Write(WritePacket<'a>),
    RegWrite(WritePacket<'a>),
    Action(&'a ActionPacket),
    Reboot(&'a RebootPacket),
    FactoryReset(&'a FactoryResetPacket),
    Status(StatusPacket<'a>),
    SyncRead(SyncReadPacket<'a>),
    SyncWrite(SyncWritePacket<'a>),
    BulkRead(BulkReadPacket<'a>),
    BulkWrite(BulkWritePacket<'a>),
    FastSyncRead(FastSyncReadPacket<'a>),
    FastBulkRead(FastBulkReadPacket<'a>),
    Raw(RawPacket<'a>),
}
