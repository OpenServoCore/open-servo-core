#![no_std]

mod codec;
pub mod typed;
pub mod wire;

pub mod prelude;

pub use codec::Codec;
#[cfg(feature = "osc")]
pub use typed::CalibratePacket;
pub use typed::{
    ActionPacket, BulkReadPacket, BulkReadSlotIter, BulkSlot, BulkSlotInfo, BulkWritePacket,
    ClearPacket, ControlTableBackupPacket, DecodeError, Extension, FactoryResetPacket,
    FastBulkReadPacket, FastBulkTupleIter, FastPosition, FastReadPacket, FastReadVariant,
    FastSlotInfo, FastSyncReadPacket, Instruction, NoExt, NoReplyExt, Packet, PingPacket,
    ReadPacket, RebootPacket, RegWritePacket, ReplyExt, StatusError, StatusPacket, StatusReply,
    SyncReadPacket, SyncSlotInfo, SyncWritePacket, WritePacket, decode,
};
#[cfg(feature = "software-crc")]
pub use wire::SoftwareCrcUmts;
pub use wire::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, ByteIter, Bytes, CRC_BYTES, CrcUmts,
    FAST_RESPONSE_SLOT_BYTES, FAST_RESPONSE_SLOT0_BYTES, HEADER, MAX_LENGTH, Overflow, ParseError,
    REQUEST_HEADER_BYTES, RESPONSE_HEADER_BYTES, RawFrame, RxView, SYNC_REQUEST_SLOT_BYTES,
    WriteBuf, WriteError, parse_raw, write_raw,
};
