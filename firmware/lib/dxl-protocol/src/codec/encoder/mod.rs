//! Frame encoders for the three DXL 2.0 frame shapes:
//!
//! - [`InstructionEncoder`] -- master -> slave request frames
//! - [`StatusEncoder`]      -- slave -> master Status reply frames
//! - [`SlotEncoder`]        -- one slave's slice of a coalesced Fast
//!   Sync/Bulk Read reply chain
//!
//! Each encoder borrows the caller's TX buffer and emits one frame per
//! method call. Each exposes a per-variant fluent API plus a unified
//! `.emit(...)` that round-trips the matching decoder enum
//! (`Packet<'_>` / `Status<'_>` / `Slot<'_>` + `SlotPosition`). The
//! unified path handles every variant -- no decode-only refusals.

#![allow(dead_code)]

mod instruction;
mod slot;
mod status;

pub use instruction::InstructionEncoder;
pub use slot::SlotEncoder;
pub use status::StatusEncoder;

use crate::buf::{WriteBuf, WriteError};
use crate::constants::{HEADER, STUFFING_BYTE, STUFFING_TRIGGER};
use crate::crc::CrcUmts;
use crate::packet::{BulkReadEntry, Id, Instruction, Slot};

/// Emit one DXL 2.0 frame: header + id + length + instruction + stuffed
/// params + CRC. `params` chunks form the logical parameter region; the
/// `0xFF 0xFF 0xFD` trigger is stuffed inline. CRC runs over the wire bytes
/// (header through stuffed params). On failure `out` is truncated back to
/// its entry length so prior frames stay intact.
pub(super) fn emit_frame<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    crc: &mut CRC,
    id: Id,
    instruction: u8,
    params: &[&[u8]],
) -> Result<(), WriteError> {
    // `id == 0xFF` would let the unstuffed header..instruction prefix itself
    // complete a stuffing trigger, breaking framing.
    if id.as_byte() == 0xFF {
        return Err(WriteError::Invalid);
    }
    let start = out.len();
    match emit_frame_inner(out, crc, start, id, instruction, params) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn emit_frame_inner<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    crc: &mut CRC,
    start: usize,
    id: Id,
    instruction: u8,
    params: &[&[u8]],
) -> Result<(), WriteError> {
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(id.as_byte())?;
    let len_pos = out.len();
    out.push(0)?;
    out.push(0)?;
    out.push(instruction)?;

    // Seed the sliding window with the instruction byte so a trigger that
    // straddles the instr->params boundary still emits the stuffing FD.
    let mut last2 = [0u8, instruction];
    for chunk in params {
        for &b in *chunk {
            out.push(b)?;
            if [last2[0], last2[1], b] == STUFFING_TRIGGER {
                out.push(STUFFING_BYTE)?;
                last2 = [last2[1], STUFFING_BYTE];
            } else {
                last2 = [last2[1], b];
            }
        }
    }

    let stuffed_params_len = out.len() - (len_pos + 2 + 1);
    let length_value = (1 + stuffed_params_len + 2) as u16;
    let len_bytes = length_value.to_le_bytes();
    out.set(len_pos, len_bytes[0]);
    out.set(len_pos + 1, len_bytes[1]);

    crc.reset();
    crc.update(&out.as_slice()[start..]);
    let crc_bytes = crc.finalize().to_le_bytes();
    out.push(crc_bytes[0])?;
    out.push(crc_bytes[1])?;

    Ok(())
}

pub(super) fn emit_slot_header<W: WriteBuf>(out: &mut W, length: u16) -> Result<(), WriteError> {
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(Id::BROADCAST.as_byte())?;
    let lb = length.to_le_bytes();
    out.push(lb[0])?;
    out.push(lb[1])?;
    out.push(Instruction::Status.as_u8())?;
    Ok(())
}

pub(super) fn emit_slot_body<W: WriteBuf>(out: &mut W, slot: &Slot<'_>) -> Result<(), WriteError> {
    out.push(slot.error.as_byte())?;
    out.push(slot.id.as_byte())?;
    for &b in slot.data.iter() {
        out.push(b)?;
    }
    Ok(())
}

pub(super) fn bulk_entries_as_bytes(entries: &[BulkReadEntry]) -> &[u8] {
    // SAFETY: BulkReadEntry is #[repr(C)] align 1 with three byte-sized
    // fields (u8, U16Le, U16Le) = exactly 5 bytes, no padding. The byte
    // view of the slice is sound.
    unsafe {
        core::slice::from_raw_parts(
            entries.as_ptr() as *const u8,
            core::mem::size_of_val(entries),
        )
    }
}
