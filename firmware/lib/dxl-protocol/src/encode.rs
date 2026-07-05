//! Fused single-pass DXL 2.0 frame emitters over a caller-owned `&mut [u8]`.
//!
//! Each emitter writes into a caller-provided buffer (the driver's DMA TX
//! buffer; hosts pass arrays), checks capacity once up front, and folds the
//! CRC into the write pass rather than making a second pass over the frame.
//!
//! ## Why the CRC handling differs by source shape
//!
//! The wire CRC covers `HEADER, ID, LENGTH, INSTRUCTION, params` in that order.
//! The `LENGTH` field sits ahead of the params in the CRC stream, but its value
//! is the *stuffed* param count — known only after the params are walked. So a
//! fused header-then-params CRC needs the stuffed length before the loop.
//!
//! - **Re-iterable slice sources** (instruction frames, slice Status frames):
//!   the stuffed length is pre-measured with a cheap scan (slices with no
//!   `0xFD` cannot complete a trigger and skip to a window update), then one
//!   fused pass writes the frame and folds every byte into the running CRC.
//! - **FAST slot blocks**: slot data is never stuffed and the block length is
//!   caller-supplied, so the CRC fuses into the write with no pre-measure — even
//!   for a lazy chunk source.
//! - **Chunked Status frames** ([`encode_status_chunked`]): the control-table
//!   read iterator is single-pass and lazy, so the stuffed length cannot be
//!   known before the params are consumed. This path writes the stuffed params,
//!   backfills `LENGTH`, then folds the frame's CRC in one trailing pass.

use crate::buf::{Chunk, WriteError};
use crate::crc::CrcUmts;
use crate::types::{Id, Instruction, Slot, SlotPosition, StatusError};
use crate::wire::{CRC_BYTES, HEADER, REQUEST_HEADER_BYTES};

/// The inline byte a stuffing trigger inserts (`HEADER[2]`, `0xFD`).
const STUFF_BYTE: u8 = HEADER[2];

// -- byte-stuffing window ------------------------------------------------

/// Sliding 2-byte window over the pre-stuffing param stream. Seeded with the
/// instruction byte so a trigger straddling the instruction->params boundary
/// still inserts the stuffing `FD`.
struct Window([u8; 2]);

impl Window {
    fn new(instruction: u8) -> Self {
        Self([0, instruction])
    }

    /// Advance over one param byte; returns `true` when an inline `FD` must
    /// follow. After a trigger the window becomes `[prev, 0xFD]`, so the next
    /// trigger needs three fresh bytes.
    #[inline]
    fn step(&mut self, b: u8) -> bool {
        let trigger = self.0[0] == HEADER[0] && self.0[1] == HEADER[1] && b == HEADER[2];
        self.0 = if trigger {
            [self.0[1], STUFF_BYTE]
        } else {
            [self.0[1], b]
        };
        trigger
    }

    /// Window update for a run written without per-byte trigger checks (a
    /// slice with no `0xFD`, or a zero run — neither can complete a trigger).
    fn absorb_tail(&mut self, s: &[u8]) {
        match s {
            [.., a, b] => self.0 = [*a, *b],
            [b] => self.0 = [self.0[1], *b],
            [] => {}
        }
    }

    fn absorb_zeros(&mut self, n: usize) {
        if n >= 2 {
            self.0 = [0, 0];
        } else if n == 1 {
            self.0 = [self.0[1], 0];
        }
    }
}

/// Insertions a param slice contributes, advancing `win`. Slices with no
/// `0xFD` byte cannot complete a trigger regardless of window state, so they
/// take a bulk window update — identical to the per-byte result.
fn measure_slice(win: &mut Window, s: &[u8]) -> usize {
    if !s.contains(&STUFF_BYTE) {
        win.absorb_tail(s);
        return 0;
    }
    let mut n = 0;
    for &b in s {
        if win.step(b) {
            n += 1;
        }
    }
    n
}

// -- checked cursor (lazy / per-run capacity) ----------------------------

/// A write cursor over a caller buffer with per-write bounds checks. Used by
/// the FAST slot and chunked Status paths, whose lazy sources preclude a single
/// up-front capacity check; checks stay per-run (per chunk / field), never a
/// per-byte `Result` in a byte loop.
struct Cursor<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn put(&mut self, b: u8) -> Result<(), WriteError> {
        let cell = self.buf.get_mut(self.pos).ok_or(WriteError::Overflow)?;
        *cell = b;
        self.pos += 1;
        Ok(())
    }

    fn put_slice(&mut self, s: &[u8]) -> Result<(), WriteError> {
        let end = self.pos.checked_add(s.len()).ok_or(WriteError::Overflow)?;
        let dst = self
            .buf
            .get_mut(self.pos..end)
            .ok_or(WriteError::Overflow)?;
        dst.copy_from_slice(s);
        self.pos = end;
        Ok(())
    }

    fn fill_zero(&mut self, n: usize) -> Result<(), WriteError> {
        let end = self.pos.checked_add(n).ok_or(WriteError::Overflow)?;
        let dst = self
            .buf
            .get_mut(self.pos..end)
            .ok_or(WriteError::Overflow)?;
        dst.fill(0);
        self.pos = end;
        Ok(())
    }

    /// Overwrite an already-written cell (LENGTH backfill). No-op if out of
    /// range; callers only target header positions they have written.
    fn set(&mut self, idx: usize, b: u8) {
        if let Some(cell) = self.buf.get_mut(idx) {
            *cell = b;
        }
    }

    fn written(&self, end: usize) -> &[u8] {
        self.buf.get(..end).unwrap_or(&[])
    }

    fn stuff_byte(&mut self, b: u8, win: &mut Window) -> Result<(), WriteError> {
        self.put(b)?;
        if win.step(b) {
            self.put(STUFF_BYTE)?;
        }
        Ok(())
    }

    fn stuff_slice(&mut self, s: &[u8], win: &mut Window) -> Result<(), WriteError> {
        if !s.contains(&STUFF_BYTE) {
            self.put_slice(s)?;
            win.absorb_tail(s);
            return Ok(());
        }
        for &b in s {
            self.stuff_byte(b, win)?;
        }
        Ok(())
    }

    fn stuff_zeros(&mut self, n: usize, win: &mut Window) -> Result<(), WriteError> {
        self.fill_zero(n)?;
        win.absorb_zeros(n);
        Ok(())
    }
}

// -- instruction / slice-Status frames (fused, pre-measured) -------------

/// Emit `HEADER, ID, LENGTH, INSTRUCTION, stuffed params, CRC` into `out`,
/// returning the frame length. One pass: the header and every param byte fold
/// into the running CRC as they are written, the stuffed length having been
/// pre-measured so `LENGTH` is correct at fold time.
fn encode_framed<C: CrcUmts>(
    out: &mut [u8],
    id: Id,
    instruction: u8,
    params: &[&[u8]],
) -> Result<usize, WriteError> {
    // `id == 0xFF` would let the unstuffed header..instruction prefix complete
    // a stuffing trigger, breaking framing.
    if id.as_byte() == 0xFF {
        return Err(WriteError::Invalid);
    }

    let mut win = Window::new(instruction);
    let mut raw = 0usize;
    let mut inserts = 0usize;
    for s in params {
        raw += s.len();
        inserts += measure_slice(&mut win, s);
    }
    let stuffed = raw + inserts;
    let total = REQUEST_HEADER_BYTES + stuffed + CRC_BYTES;

    let region = out.get_mut(..total).ok_or(WriteError::Overflow)?;
    let (head, rest) = region
        .split_at_mut_checked(REQUEST_HEADER_BYTES)
        .ok_or(WriteError::Overflow)?;
    let (body, tail) = rest
        .split_at_mut_checked(stuffed)
        .ok_or(WriteError::Overflow)?;

    let length = (1 + stuffed + CRC_BYTES) as u16;
    let lb = length.to_le_bytes();
    let header = [
        HEADER[0],
        HEADER[1],
        HEADER[2],
        HEADER[3],
        id.as_byte(),
        lb[0],
        lb[1],
        instruction,
    ];
    head.copy_from_slice(&header);

    let mut crc = C::new();
    crc.update(&header);

    // `body` is sized to exactly `stuffed`, which equals the byte count emitted
    // below (`raw` params + `inserts` inline FDs), so every `next()` yields
    // `Some`; the `if let` keeps the loop panic-free with no per-byte `Result`.
    let mut cells = body.iter_mut();
    let mut win = Window::new(instruction);
    for s in params {
        for &b in *s {
            if let Some(cell) = cells.next() {
                *cell = b;
            }
            crc.update_byte(b);
            if win.step(b) {
                if let Some(cell) = cells.next() {
                    *cell = STUFF_BYTE;
                }
                crc.update_byte(STUFF_BYTE);
            }
        }
    }

    tail.copy_from_slice(&crc.finalize().to_le_bytes());
    Ok(total)
}

/// Encode a host->servo instruction frame. `params` is the instruction's
/// param region as an ordered list of byte runs (the stuffing window straddles
/// run boundaries). Returns the frame length in bytes.
pub fn encode_instruction<C: CrcUmts>(
    out: &mut [u8],
    id: Id,
    instruction: u8,
    params: &[&[u8]],
) -> Result<usize, WriteError> {
    encode_framed::<C>(out, id, instruction, params)
}

/// Encode a servo->host single-target Status reply (`id, error, payload`).
/// `payload` is the post-error param region (already in wire byte order);
/// stuffing spans the error byte and the payload. Returns the frame length.
pub fn encode_status<C: CrcUmts>(
    out: &mut [u8],
    id: Id,
    error: StatusError,
    payload: &[u8],
) -> Result<usize, WriteError> {
    let err = [error.as_byte()];
    encode_framed::<C>(out, id, Instruction::Status.as_u8(), &[&err, payload])
}

/// Chunked counterpart of [`encode_status`]: the payload comes from a lazy,
/// single-pass [`Chunk`] iterator (a control-table read straight into `out`).
///
/// Because the source is single-pass, the stuffed length is unknown until the
/// params are written, so `LENGTH` is backfilled and the CRC folded in one
/// trailing pass over the written frame rather than fused per byte.
pub fn encode_status_chunked<'c, C, I>(
    out: &mut [u8],
    id: Id,
    error: StatusError,
    chunks: I,
) -> Result<usize, WriteError>
where
    C: CrcUmts,
    I: IntoIterator<Item = Chunk<'c>>,
{
    if id.as_byte() == 0xFF {
        return Err(WriteError::Invalid);
    }

    let mut cur = Cursor::new(out);
    let header = [
        HEADER[0],
        HEADER[1],
        HEADER[2],
        HEADER[3],
        id.as_byte(),
        0,
        0,
        Instruction::Status.as_u8(),
    ];
    cur.put_slice(&header)?;
    let params_start = cur.pos;

    let mut win = Window::new(Instruction::Status.as_u8());
    cur.stuff_byte(error.as_byte(), &mut win)?;
    for chunk in chunks {
        match chunk {
            Chunk::Slice(s) => cur.stuff_slice(s, &mut win)?,
            Chunk::Zero(n) => cur.stuff_zeros(n as usize, &mut win)?,
        }
    }
    let params_end = cur.pos;

    let length = (1 + (params_end - params_start) + CRC_BYTES) as u16;
    let lb = length.to_le_bytes();
    cur.set(HEADER.len() + 1, lb[0]);
    cur.set(HEADER.len() + 2, lb[1]);

    let mut crc = C::new();
    crc.update(cur.written(params_end));
    let crc_bytes = crc.finalize().to_le_bytes();
    cur.put_slice(&crc_bytes)?;
    Ok(cur.pos)
}

// -- FAST slot blocks (fused, unstuffed) ---------------------------------

/// Emit one slot block of a coalesced FAST Sync/Bulk Read reply chain into
/// `out`. Slot data is never stuffed (official FAST format), so the block is a
/// verbatim copy plus a CRC and the layout is fixed — the driver patches
/// successor CRCs at offsets derived from it. Returns the block length.
pub fn encode_slot<C: CrcUmts>(
    out: &mut [u8],
    slot: &Slot<'_>,
    position: SlotPosition,
) -> Result<usize, WriteError> {
    encode_slot_chunked::<C, _>(
        out,
        slot.id,
        slot.error,
        position,
        [Chunk::Slice(slot.data)],
    )
}

/// Chunked counterpart of [`encode_slot`]: the slot data comes from a
/// [`Chunk`] iterator (control-table read into `out`) instead of a slice.
/// Unstuffed like [`encode_slot`], so the CRC fuses into the write.
pub fn encode_slot_chunked<'c, C, I>(
    out: &mut [u8],
    id: Id,
    error: StatusError,
    position: SlotPosition,
    chunks: I,
) -> Result<usize, WriteError>
where
    C: CrcUmts,
    I: IntoIterator<Item = Chunk<'c>>,
{
    let mut cur = Cursor::new(out);
    match position {
        SlotPosition::First { packet_length } => {
            let lb = packet_length.to_le_bytes();
            let header = [
                HEADER[0],
                HEADER[1],
                HEADER[2],
                HEADER[3],
                Id::BROADCAST.as_byte(),
                lb[0],
                lb[1],
                Instruction::Status.as_u8(),
            ];
            let mut crc = C::new();
            cur.put_slice(&header)?;
            crc.update(&header);
            for b in [error.as_byte(), id.as_byte()] {
                cur.put(b)?;
                crc.update_byte(b);
            }
            for chunk in chunks {
                match chunk {
                    Chunk::Slice(s) => {
                        cur.put_slice(s)?;
                        crc.update(s);
                    }
                    Chunk::Zero(n) => {
                        cur.fill_zero(n as usize)?;
                        for _ in 0..n {
                            crc.update_byte(0);
                        }
                    }
                }
            }
            cur.put_slice(&crc.finalize().to_le_bytes())?;
        }
        SlotPosition::Successor { crc } => {
            cur.put(error.as_byte())?;
            cur.put(id.as_byte())?;
            for chunk in chunks {
                match chunk {
                    Chunk::Slice(s) => cur.put_slice(s)?,
                    Chunk::Zero(n) => cur.fill_zero(n as usize)?,
                }
            }
            cur.put_slice(&crc.to_le_bytes())?;
        }
    }
    Ok(cur.pos)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::SoftwareCrcUmts as Crc;

    #[test]
    fn ping_matches_reference_bytes() {
        let mut buf = [0u8; 16];
        let n = encode_instruction::<Crc>(&mut buf, Id::new(0x01), 0x01, &[]).unwrap();
        assert_eq!(
            &buf[..n],
            &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E]
        );
    }

    #[test]
    fn id_0xff_is_rejected() {
        let mut buf = [0u8; 16];
        assert_eq!(
            encode_instruction::<Crc>(&mut buf, Id::new(0xFF), 0x01, &[]),
            Err(WriteError::Invalid)
        );
        assert_eq!(
            encode_status::<Crc>(&mut buf, Id::new(0xFF), StatusError::OK, &[]),
            Err(WriteError::Invalid)
        );
    }

    #[test]
    fn one_byte_short_buffer_overflows_without_panic() {
        let mut big = [0u8; 32];
        let n =
            encode_status::<Crc>(&mut big, Id::new(0x04), StatusError::OK, &[1, 2, 3, 4]).unwrap();
        let mut exact = [0u8; 32];
        assert!(
            encode_status::<Crc>(
                &mut exact[..n],
                Id::new(0x04),
                StatusError::OK,
                &[1, 2, 3, 4]
            )
            .is_ok()
        );
        assert_eq!(
            encode_status::<Crc>(
                &mut exact[..n - 1],
                Id::new(0x04),
                StatusError::OK,
                &[1, 2, 3, 4]
            ),
            Err(WriteError::Overflow)
        );
    }
}
