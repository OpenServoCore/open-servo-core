#![no_std]
//! DXL 2.0 wire-format codec — master and slave share the same primitives.
//!
//! Top-level entry points:
//! - [`parse_packet`] — bytes → typed [`Packet`]
//! - [`decode_status`] — request instruction + [`RawStatus`] → typed [`Status`]
//! - [`write_packet`] — [`Packet`] → wire bytes (full standalone frame)
//! - [`write_status`] — typed [`Status`] → wire bytes (slave convenience)
//! - [`write_slot`] — Fast Sync/Bulk Read slot + [`SlotPosition`] → wire bytes
//!
//! Extension hooks: implement [`InstructionExt`] to add custom request
//! instructions; implement [`StatusExt`] to add custom typed status flavors.

mod typed;
mod wire;

pub use typed::{
    ActionPacket, ActionStatus, BulkReadPacket, BulkReadSlotIter, BulkReadStatus, BulkSlot,
    BulkSlotInfo, BulkWritePacket, ClearPacket, ControlTableBackupPacket, DecodeError, ErrorStatus,
    FactoryResetPacket, FastBulkReadPacket, FastBulkReadStatus, FastBulkSlotIter, FastBulkTupleIter,
    FastReadPacket, FastReadVariant, FastSlotInfo, FastSyncReadPacket, FastSyncReadStatus,
    FastSyncSlotIter, Instruction, InstructionExt, NoInstructionExt, NoStatusExt, Packet,
    PingPacket, PingStatus, RawStatus, ReadPacket, ReadStatus, RebootPacket, RebootStatus,
    RegWritePacket, RegWriteStatus, Slot, SlotPosition, Status, StatusError, StatusExt,
    SyncReadPacket, SyncReadStatus, SyncSlotInfo, SyncWritePacket, WritePacket, WriteStatus,
    decode_status, write_packet, write_slot, write_status,
};
#[cfg(feature = "software-crc")]
pub use wire::SoftwareCrcUmts;
pub use wire::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, ByteIter, Bytes, CRC_BYTES, CrcUmts,
    FAST_RESPONSE_SLOT0_BYTES, FAST_RESPONSE_SLOT_BYTES, HEADER, MAX_LENGTH, Overflow, ParseError,
    REQUEST_HEADER_BYTES, RESPONSE_HEADER_BYTES, RawFrame, RxView, Stuffer, Unstuffer,
    SYNC_REQUEST_SLOT_BYTES, WriteBuf, WriteError, write_raw,
};

/// Parse the first DXL frame in the byte view (which may straddle a ring
/// buffer wrap via [`RxView::ring`]). Returns the typed [`Packet`] plus the
/// number of virtual bytes consumed from the view.
///
/// Instruction bytes outside the standard set are routed to
/// [`InstructionExt::decode`]: `None` from the extension surfaces as
/// [`ParseError::BadInstruction`]; `Some(Err)` as [`ParseError::BadLength`].
pub fn parse_packet<'a, CRC: CrcUmts, I: InstructionExt>(
    view: RxView<'a>,
) -> Result<(Packet<'a, I>, usize), ParseError> {
    let (raw, consumed) = wire::parse_raw::<CRC>(view)?;
    match typed::decode::<I>(raw) {
        Ok(packet) => Ok((packet, consumed)),
        Err(DecodeError::UnknownInstruction) => match I::decode(raw) {
            Some(Ok(v)) => Ok((Packet::Ext(v), consumed)),
            Some(Err(_)) => Err(ParseError::BadLength { skip: consumed }),
            None => Err(ParseError::BadInstruction { skip: consumed }),
        },
        Err(DecodeError::BadParams) => Err(ParseError::BadLength { skip: consumed }),
    }
}
