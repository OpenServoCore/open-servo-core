#![no_std]

mod codec;
pub mod typed;
pub mod wire;

pub mod prelude;

pub use codec::Codec;
pub use typed::{
    ActionPacket, ActionStatus, BulkReadPacket, BulkReadSlotIter, BulkReadStatus, BulkSlot,
    BulkSlotInfo, BulkWritePacket, ClearPacket, ControlTableBackupPacket, DecodeError, ErrorStatus,
    FactoryResetPacket, FastBulkReadPacket, FastBulkTupleIter, FastReadPacket, FastReadVariant,
    FastSlotInfo, FastSyncReadPacket, Instruction, InstructionExt, NoInstructionExt, NoStatusExt,
    Packet, PingPacket, PingStatus, RawStatus, ReadPacket, ReadStatus, RebootPacket, RebootStatus,
    RegWritePacket, RegWriteStatus, Slot, SlotPosition, Status, StatusError, StatusExt,
    SyncReadPacket, SyncReadStatus, SyncSlotInfo, SyncWritePacket, WritePacket, WriteStatus,
    decode, write_slot,
};
#[cfg(feature = "software-crc")]
pub use wire::SoftwareCrcUmts;
pub use wire::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, ByteIter, Bytes, CRC_BYTES, CrcUmts,
    FAST_RESPONSE_SLOT0_BYTES, FAST_RESPONSE_SLOT_BYTES, HEADER, MAX_LENGTH, Overflow, ParseError,
    REQUEST_HEADER_BYTES, RESPONSE_HEADER_BYTES, RawFrame, RxView, SYNC_REQUEST_SLOT_BYTES,
    WriteBuf, WriteError, parse_raw, write_raw,
};
