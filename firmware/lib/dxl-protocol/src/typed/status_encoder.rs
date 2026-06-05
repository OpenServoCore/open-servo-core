use crate::wire::{BROADCAST_ID, CrcUmts, HEADER, RawFrame, WriteBuf, WriteError, write_raw};

use super::ext::StatusExt;
use super::fast::SlotPosition;
use super::instruction::Instruction;
use super::status::{
    ActionStatus, BulkReadStatus, ErrorStatus, PingStatus, ReadStatus, RebootStatus,
    RegWriteStatus, Slot, Status, SyncReadStatus, WriteStatus,
};
use super::status_error::StatusError;

pub fn write_status<W: WriteBuf, CRC: CrcUmts, S: StatusExt>(
    out: &mut W,
    status: &Status<'_, S>,
) -> Result<(), WriteError> {
    match *status {
        Status::Ping(PingStatus {
            id,
            model,
            firmware,
        }) => {
            let m = model.to_le_bytes();
            let payload = [m[0], m[1], firmware];
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, payload.iter().copied())
        }
        Status::Read(ReadStatus { id, data })
        | Status::SyncRead(SyncReadStatus { id, data })
        | Status::BulkRead(BulkReadStatus { id, data }) => {
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, data)
        }
        Status::Write(WriteStatus { id })
        | Status::RegWrite(RegWriteStatus { id })
        | Status::Action(ActionStatus { id })
        | Status::Reboot(RebootStatus { id }) => {
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, core::iter::empty())
        }
        Status::Error(ErrorStatus { id, error }) => {
            write_status_frame::<W, _, CRC>(out, id, error, core::iter::empty())
        }
        // Master-side decode outputs — slaves emit Fast slots via `write_slot`.
        Status::FastSyncRead(_) | Status::FastBulkRead(_) => Err(WriteError::Invalid),
        Status::Ext(ref v) => S::write::<W, CRC>(v, out),
    }
}

fn write_status_frame<W: WriteBuf, I: IntoIterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    id: u8,
    error: StatusError,
    payload: I,
) -> Result<(), WriteError> {
    write_raw::<W, _, CRC>(
        out,
        RawFrame {
            id,
            instruction: Instruction::Status.as_u8(),
            params: core::iter::once(error.as_u8()).chain(payload),
        },
    )
}

/// CRC slot placeholder. Fire ISR overwrites in-flight during the DMA
/// pre-fetch race; on race loss these bytes appear on the wire and the
/// master sees a CRC mismatch (intentional — surfaces the failure).
const CRC_PLACEHOLDER: [u8; 2] = [0xAA, 0xBB];

/// Serialize one Fast Status slot — one slave's piece of a coalesced response.
/// The payload is NOT byte-stuffed: Fast Read decoding is positional (slot
/// indices), not trigger-driven, so stuffing would break deterministic layout.
pub fn write_slot<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    slot: &Slot<'_>,
    position: SlotPosition,
) -> Result<(), WriteError> {
    let start = out.len();
    match write_slot_inner::<W, CRC>(out, slot, position) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn write_slot_inner<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    slot: &Slot<'_>,
    position: SlotPosition,
) -> Result<(), WriteError> {
    match position {
        SlotPosition::Only { packet_length } => {
            let frame_start = out.len();
            write_slot_header(out, packet_length)?;
            write_slot_body(out, slot)?;
            // No predecessors → CRC is purely local; compute over the bytes
            // we just emitted and append.
            let crc = CRC::accumulate(0, &out.as_slice()[frame_start..]);
            out.push(crc as u8)?;
            out.push((crc >> 8) as u8)?;
        }
        SlotPosition::First { packet_length } => {
            write_slot_header(out, packet_length)?;
            write_slot_body(out, slot)?;
        }
        SlotPosition::Middle => {
            write_slot_body(out, slot)?;
        }
        SlotPosition::Last => {
            write_slot_body(out, slot)?;
            out.push(CRC_PLACEHOLDER[0])?;
            out.push(CRC_PLACEHOLDER[1])?;
        }
    }
    Ok(())
}

fn write_slot_header<W: WriteBuf>(out: &mut W, length: u16) -> Result<(), WriteError> {
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(BROADCAST_ID)?;
    let lb = length.to_le_bytes();
    out.push(lb[0])?;
    out.push(lb[1])?;
    out.push(Instruction::Status.as_u8())?;
    Ok(())
}

fn write_slot_body<W: WriteBuf>(out: &mut W, slot: &Slot<'_>) -> Result<(), WriteError> {
    out.push(slot.error)?;
    out.push(slot.id)?;
    for b in slot.data.iter() {
        out.push(b)?;
    }
    Ok(())
}
