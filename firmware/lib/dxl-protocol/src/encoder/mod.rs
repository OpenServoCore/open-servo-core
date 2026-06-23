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

mod instruction;
mod slot;
mod status;
mod stuffing;

pub use instruction::InstructionEncoder;
pub use slot::SlotEncoder;
pub use status::StatusEncoder;

use crate::buf::{Chunk, WriteBuf, WriteError};
use crate::crc::CrcUmts;
use crate::types::{Id, Instruction, Slot, StatusError};
use crate::wire::HEADER;

use stuffing::Stuffer;

/// Emit one DXL 2.0 frame: header + id + length + instruction + stuffed
/// params + CRC. `write_params` is called once with the open output and a
/// [`Stuffer`] -- each `Stuffer::push` byte counts toward the wire `Length`
/// field after stuffing. On failure `out` is truncated back to its entry
/// length so prior frames stay intact.
pub(in crate::encoder) fn emit_frame_with<W, CRC, F>(
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
    let prefix = [
        HEADER[0],
        HEADER[1],
        HEADER[2],
        HEADER[3],
        id.as_byte(),
        0,
        0,
        instruction,
    ];
    let len_pos = out.len() + 5;
    out.push_slice(&prefix)?;

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
    out.push_slice(&crc_bytes)?;

    Ok(())
}

/// Convenience over [`emit_frame_with`] for params that are already a slice
/// of byte chunks (stuffing window straddles chunk boundaries).
pub(in crate::encoder) fn emit_frame<W: WriteBuf, CRC: CrcUmts>(
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

pub(in crate::encoder) fn emit_slot_header<W: WriteBuf>(
    out: &mut W,
    length: u16,
) -> Result<(), WriteError> {
    let lb = length.to_le_bytes();
    let prefix = [
        HEADER[0],
        HEADER[1],
        HEADER[2],
        HEADER[3],
        Id::BROADCAST.as_byte(),
        lb[0],
        lb[1],
        Instruction::Status.as_u8(),
    ];
    out.push_slice(&prefix)
}

pub(in crate::encoder) fn emit_slot_body<W: WriteBuf>(
    out: &mut W,
    slot: &Slot<'_>,
) -> Result<(), WriteError> {
    out.push_slice(&[slot.error.as_byte(), slot.id.as_byte()])?;
    out.push_slice(slot.data)
}

/// Body emitter for a slot whose data comes from a chunk iterator. Same
/// wire shape as [`emit_slot_body`] (`error`, `id`, then `data...`) but
/// the data bytes are written straight from each `Chunk::Slice` /
/// `Chunk::Zero` without a scratch buffer in between. Slot bodies are
/// unstuffed, so this is a plain `push_slice` / `push_zero` per chunk.
pub(in crate::encoder) fn emit_slot_body_chunked<'a, W, I>(
    out: &mut W,
    id: Id,
    error: StatusError,
    chunks: I,
) -> Result<(), WriteError>
where
    W: WriteBuf,
    I: IntoIterator<Item = Chunk<'a>>,
{
    out.push_slice(&[error.as_byte(), id.as_byte()])?;
    for chunk in chunks {
        match chunk {
            Chunk::Slice(s) => out.push_slice(s)?,
            Chunk::Zero(n) => out.push_zero(n)?,
        }
    }
    Ok(())
}
