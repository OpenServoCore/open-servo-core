//! The reply snapshot's copy (protocol sec 4.2): each span moves into the
//! snapshot buffer so that every naturally aligned field it covers is one
//! access. The copy channel takes a span's aligned middle in beats as wide as
//! both its ends allow; the CPU copies the ends, one natural source unit each.
//!
//! The channel drops the address bits a beat's width does not use, on both
//! ends and with no error (RM sec 8.3.5), so a wrong plan ships shifted bytes
//! under a valid CRC: [`plan`] and [`units`] are pure and pinned.

use osc_protocol::frame::Header;
use osc_protocol::wire;

/// One frame at any of the four shifts [`frame_shift`] picks, plus the byte
/// below an odd base that the CRC feed starts on.
pub const LEN: usize = 3 + wire::covered_len(wire::len_for(wire::MAX_PAYLOAD));

#[repr(C, align(4))]
pub struct Buf(pub [u8; LEN]);

impl Buf {
    pub const fn new() -> Self {
        Buf([0; LEN])
    }
}

impl Default for Buf {
    fn default() -> Self {
        Self::new()
    }
}

/// One access's width, in bytes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Beat {
    Byte = 1,
    Half = 2,
    Word = 4,
}

/// One span's copy: the CPU takes the bytes before `head` and from `mid`
/// on, the channel `beats` beats of `beat` in between.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    pub head: usize,
    pub beat: Beat,
    pub beats: usize,
    pub mid: usize,
}

/// Spans up to this long are copied by the CPU alone, whatever their
/// alignment: below it one channel arm costs more instructions than the
/// copy it saves.
pub const CPU_SPAN_MAX: usize = 7;

const _: () = assert!(CPU_SPAN_MAX >= 4, "a single field must take the CPU row");

/// The copy of `len` bytes from address `src` to address `dst`:
/// - [`CPU_SPAN_MAX`] bytes or less: the CPU alone, every field whole;
/// - longer, the same address mod 4: word beats from the first 4-aligned
///   byte, CPU ends, every field whole;
/// - longer, same parity: halfword beats with a CPU byte at either end,
///   fields up to 2 bytes whole;
/// - longer, opposite parity: byte beats, nothing whole.
#[inline(always)]
pub const fn plan(src: usize, dst: usize, len: usize) -> Plan {
    let skew = (src ^ dst) & 3;
    if len <= CPU_SPAN_MAX {
        Plan {
            head: len,
            beat: Beat::Byte,
            beats: 0,
            mid: len,
        }
    } else if skew == 0 {
        let head = src.wrapping_neg() & 3;
        let body = (len - head) & !3;
        Plan {
            head,
            beat: Beat::Word,
            beats: body / 4,
            mid: head + body,
        }
    } else if skew & 1 == 0 {
        let head = src & 1;
        let body = (len - head) & !1;
        Plan {
            head,
            beat: Beat::Half,
            beats: body / 2,
            mid: head + body,
        }
    } else {
        Plan {
            head: 0,
            beat: Beat::Byte,
            beats: len,
            mid: len,
        }
    }
}

/// The CPU's units of `plan`'s copy of `len` bytes from address `src`, in
/// order, as `unit(offset, width)`: every byte outside the channel's
/// `head..mid`, at each address the widest naturally aligned unit that
/// fits before `len`. So a byte at an odd address, a halfword at 2 mod 4,
/// words while 4 are left, then a halfword and a byte. Each is one aligned
/// load, so a field the CPU's part covers whole is one unit. No unit
/// crosses `head`: every row ends the head where a wider unit could start.
#[inline(always)]
pub fn units(src: usize, plan: &Plan, len: usize, mut unit: impl FnMut(usize, Beat)) {
    let mut i = 0;
    loop {
        if i == plan.head {
            i = plan.mid;
        }
        if i >= len {
            return;
        }
        let (a, left) = (src + i, len - i);
        let w = if a & 1 != 0 || left < 2 {
            Beat::Byte
        } else if a & 2 != 0 || left < 4 {
            Beat::Half
        } else {
            Beat::Word
        };
        unit(i, w);
        i += w as usize;
    }
}

/// The frame base's offset in [`Buf`] for a payload whose first byte comes
/// from address `first`: the payload then sits at `first`'s address mod 4.
#[inline(always)]
pub const fn frame_shift(first: usize) -> usize {
    first.wrapping_sub(Header::SIZE) & 3
}

/// Copy `spans`, concatenated, into `buf` [`Header::SIZE`] past the frame
/// base, and return the base, [`frame_shift`] of the first span into `buf`.
/// `dma(src, dst, beat, beats)` starts one span's middle on the copy channel,
/// first waiting out its own copy in flight; the last may still run on
/// return. Each span's CPU ends are copied after its middle starts.
///
/// # Safety
/// `buf` writable; the spans total at most [`wire::MAX_PAYLOAD`] bytes.
#[inline(always)]
pub unsafe fn gather(
    buf: *mut Buf,
    spans: &[&[u8]],
    mut dma: impl FnMut(*const u8, *mut u8, Beat, u16),
) -> *mut u8 {
    let first = match spans {
        [s, ..] => s.as_ptr().addr(),
        [] => 0,
    };
    // SAFETY: the shift, the header and the payload fit `LEN` (caller's
    // total), and every span is readable for its length.
    unsafe {
        let base = buf.cast::<u8>().add(frame_shift(first));
        let mut dst = base.add(Header::SIZE);
        for s in spans {
            let src = s.as_ptr();
            let p = plan(src.addr(), dst.addr(), s.len());
            if p.beats != 0 {
                dma(src.add(p.head), dst.add(p.head), p.beat, p.beats as u16);
            }
            cpu_copy(dst, src, &p, s.len());
            dst = dst.add(s.len());
        }
        base
    }
}

/// [`units`] carried out: one volatile load each, so a field is read whole
/// even with the kernel preempting the copy; one store where the
/// destination is aligned for it, else byte stores.
///
/// # Safety
/// `src` readable and `dst` writable for `len` bytes.
#[inline(always)]
unsafe fn cpu_copy(dst: *mut u8, src: *const u8, plan: &Plan, len: usize) {
    units(src.addr(), plan, len, |i, w| {
        // SAFETY: `units` stays inside `len` and aligns each load to its width;
        // a typed store is taken only at an address aligned for it.
        unsafe {
            let (s, d) = (src.add(i), dst.add(i));
            let mut v = match w {
                Beat::Word => s.cast::<u32>().read_volatile(),
                Beat::Half => s.cast::<u16>().read_volatile() as u32,
                Beat::Byte => s.read_volatile() as u32,
            };
            if d.addr() & (w as usize - 1) == 0 {
                match w {
                    Beat::Word => d.cast::<u32>().write(v),
                    Beat::Half => d.cast::<u16>().write(v as u16),
                    Beat::Byte => d.write(v as u8),
                }
            } else {
                let mut k = 0;
                while k < w as usize {
                    d.add(k).write(v as u8);
                    v >>= 8;
                    k += 1;
                }
            }
        }
    });
}

#[cfg(test)]
mod tests;
