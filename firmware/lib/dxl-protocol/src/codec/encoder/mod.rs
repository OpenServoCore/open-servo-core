//! Frame encoders for the three DXL 2.0 frame shapes:
//!
//! - [`InstructionEncoder`] -- master -> slave request frames
//! - [`StatusEncoder`]      -- slave -> master Status reply frames
//! - [`SlotEncoder`]        -- one slave's slice of a coalesced Fast
//!   Sync/Bulk Read reply chain
//!
//! Each encoder borrows the caller's TX buffer and emits one frame per
//! method call. Each exposes a per-variant fluent API plus a unified
//! `.emit(...)` over the matching enum (`Status<'_>` / `Slot<'_>` +
//! `SlotPosition`).

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
use crate::types::{Id, Instruction, Slot};

/// Sliding-window state for the byte-stuffing encoder: after each pushed
/// param byte, a `0xFF 0xFF 0xFD` window triggers an inline `0xFD`.
pub(super) struct Stuffer([u8; 2]);

impl Stuffer {
    /// Seed the window with the instruction byte so a trigger straddling the
    /// instr->params boundary still emits the stuffing FD.
    fn new(instruction: u8) -> Self {
        Self([0, instruction])
    }

    pub(super) fn push<W: WriteBuf>(&mut self, out: &mut W, b: u8) -> Result<(), WriteError> {
        out.push(b)?;
        if [self.0[0], self.0[1], b] == STUFFING_TRIGGER {
            out.push(STUFFING_BYTE)?;
            self.0 = [self.0[1], STUFFING_BYTE];
        } else {
            self.0 = [self.0[1], b];
        }
        Ok(())
    }
}

/// Emit one DXL 2.0 frame: header + id + length + instruction + stuffed
/// params + CRC. `write_params` is called once with the open output and a
/// [`Stuffer`] -- each `Stuffer::push` byte counts toward the wire `Length`
/// field after stuffing. On failure `out` is truncated back to its entry
/// length so prior frames stay intact.
pub(super) fn emit_frame_with<W, CRC, F>(
    out: &mut W,
    crc: &mut CRC,
    id: Id,
    instruction: u8,
    write_params: F,
) -> Result<(), WriteError>
where
    W: WriteBuf,
    CRC: CrcUmts,
    F: FnOnce(&mut W, &mut Stuffer) -> Result<(), WriteError>,
{
    // `id == 0xFF` would let the unstuffed header..instruction prefix itself
    // complete a stuffing trigger, breaking framing.
    if id.as_byte() == 0xFF {
        return Err(WriteError::Invalid);
    }
    let start = out.len();
    match emit_frame_inner(out, crc, start, id, instruction, write_params) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn emit_frame_inner<W, CRC, F>(
    out: &mut W,
    crc: &mut CRC,
    start: usize,
    id: Id,
    instruction: u8,
    write_params: F,
) -> Result<(), WriteError>
where
    W: WriteBuf,
    CRC: CrcUmts,
    F: FnOnce(&mut W, &mut Stuffer) -> Result<(), WriteError>,
{
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(id.as_byte())?;
    let len_pos = out.len();
    out.push(0)?;
    out.push(0)?;
    out.push(instruction)?;

    let mut stuffer = Stuffer::new(instruction);
    write_params(out, &mut stuffer)?;

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

/// Convenience over [`emit_frame_with`] for params that are already a slice
/// of byte chunks (stuffing window straddles chunk boundaries).
pub(super) fn emit_frame<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    crc: &mut CRC,
    id: Id,
    instruction: u8,
    params: &[&[u8]],
) -> Result<(), WriteError> {
    emit_frame_with(out, crc, id, instruction, |out, stuffer| {
        for chunk in params {
            for &b in *chunk {
                stuffer.push(out, b)?;
            }
        }
        Ok(())
    })
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
