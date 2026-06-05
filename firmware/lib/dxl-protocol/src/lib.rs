#![no_std]

mod codec;
pub mod typed;
pub mod wire;

pub mod prelude;

pub use codec::Codec;
pub use typed::{
    ActionPacket, ActionReply, BulkReadPacket, BulkReadReply, BulkReadSlotIter, BulkSlot,
    BulkSlotInfo, BulkWritePacket, ClearPacket, ControlTableBackupPacket, DecodeError, ErrorReply,
    Extension, FactoryResetPacket, FastBulkReadPacket, FastBulkReadReply, FastBulkTupleIter,
    FastErrorReply, FastPosition, FastReadPacket, FastReadVariant, FastSlotInfo,
    FastSyncReadPacket, FastSyncReadReply, Instruction, NoExt, NoReplyExt, Packet, PingPacket,
    PingReply, RawStatus, ReadPacket, ReadReply, RebootPacket, RebootReply, RegWritePacket,
    RegWriteReply, Reply, ReplyExt, StatusError, SyncReadPacket, SyncReadReply, SyncSlotInfo,
    SyncWritePacket, WritePacket, WriteReply, decode,
};
#[cfg(feature = "software-crc")]
pub use wire::SoftwareCrcUmts;
pub use wire::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, ByteIter, Bytes, CRC_BYTES, CrcUmts,
    FAST_RESPONSE_SLOT_BYTES, FAST_RESPONSE_SLOT0_BYTES, HEADER, MAX_LENGTH, Overflow, ParseError,
    REQUEST_HEADER_BYTES, RESPONSE_HEADER_BYTES, RawFrame, RxView, SYNC_REQUEST_SLOT_BYTES,
    WriteBuf, WriteError, parse_raw, write_raw,
};
