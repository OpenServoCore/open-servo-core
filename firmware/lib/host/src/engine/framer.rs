//! Host RX framer: a pure ring walker.
//!
//! The host has no break wake -- all RX is solicited -- so frames are found
//! entirely from ring data: a candidate starts at a `0x00` ring byte (the
//! break's ring image), the header makes its end computable (LEN at byte 2),
//! and the CRC verdict is the only accept authority. This is the fault
//! contract (protocol sec 3.4) with the wake deleted: positions from ring
//! data only; every clock stays in the engine.
//!
//! Resync after garble is the hunt: skip to the next `0x00` in the
//! available span. A data `0x00` can masquerade as an anchor; it dies by
//! geometry or CRC and costs bounded ring bytes, never the stream.

use osc_protocol::FrameBytes;
use osc_protocol::crc::{osc_crc, osc_crc_continue};
use osc_protocol::wire::{self, Id, Inst};

/// One CRC-clean frame view over the ring.
#[derive(Debug)]
pub struct Frame<'a> {
    pub id: Id,
    pub inst: Inst,
    pub payload: FrameBytes<'a>,
}

/// Outcome of one [`Framer::step`]; callers loop until `Idle` or `Partial`.
#[derive(Debug)]
pub enum Step<'a> {
    /// No unconsumed bytes.
    Idle,
    /// A candidate frame has started but not fully ringed; positions hold.
    /// Whether it ever completes is the engine's deadline to keep.
    Partial,
    /// One frame consumed.
    Frame(Frame<'a>),
    /// `n` ring bytes skipped that anchored no frame.
    Garble(u16),
}

/// The walker: one cursor (`anchor`) into the ring, always in `[0, len)`.
#[derive(Default)]
pub struct Framer {
    anchor: usize,
}

impl Framer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Jump to `cursor`, discarding anything unconsumed. Called only when
    /// the bus is provably quiet (before a TX window opens) -- the host-side
    /// analog of the servo's bootstrap-only cursor read.
    pub fn resync(&mut self, cursor: usize) {
        self.anchor = cursor;
    }

    /// Walk one step against the ring state. `cursor` is the next-write
    /// index in `[0, ring.len())`; the ring length must be a power of two.
    /// Lap hazard (same as the servo's): more than one ring of unconsumed
    /// bytes is indistinguishable from an empty ring -- the caller's poll
    /// cadence keeps consumption within a lap.
    pub fn step<'a>(&mut self, ring: &'a [u8], cursor: usize) -> Step<'a> {
        debug_assert!(ring.len().is_power_of_two());
        let mask = ring.len() - 1;
        let avail = cursor.wrapping_sub(self.anchor) & mask;
        if avail == 0 {
            return Step::Idle;
        }

        // A frame candidate starts at the break's 0x00 ring byte.
        if ring[self.anchor] != 0x00 {
            return Step::Garble(self.hunt(ring, mask, avail));
        }
        if avail < 4 {
            return Step::Partial;
        }

        let id = Id::new(ring[(self.anchor + 1) & mask]);
        let len = ring[(self.anchor + 2) & mask];
        let inst = Inst(ring[(self.anchor + 3) & mask]);
        let valid_shape = len >= 3
            && id.is_valid()
            && (inst.is_status() || inst.opcode().is_some())
            && wire::footprint(len) <= ring.len();
        if !valid_shape {
            return Step::Garble(self.hunt(ring, mask, avail));
        }

        let footprint = wire::footprint(len);
        if avail < footprint {
            return Step::Partial;
        }

        // CRC over the anchor-inclusive covered span: the leading 0x00 is an
        // init-0 no-op (protocol sec 3.2), so feeding from the anchor equals
        // the wire checksum over ID..payload.
        let clen = wire::covered_len(len);
        let crc = ring_crc(ring, self.anchor, clen);
        let lo = ring[(self.anchor + clen) & mask];
        let hi = ring[(self.anchor + clen + 1) & mask];
        if crc != u16::from_le_bytes([lo, hi]) {
            return Step::Garble(self.hunt(ring, mask, avail));
        }

        let payload = ring_bytes(
            ring,
            (self.anchor + 4) & mask,
            wire::payload_len(len) as usize,
        );
        self.anchor = (self.anchor + footprint) & mask;
        Step::Frame(Frame { id, inst, payload })
    }

    /// Advance past a dead candidate to the next `0x00` within `avail`
    /// (skipping the current anchor -- it already failed), or consume the
    /// whole span if none. Returns the bytes skipped.
    fn hunt(&mut self, ring: &[u8], mask: usize, avail: usize) -> u16 {
        let mut n = 1;
        while n < avail && ring[(self.anchor + n) & mask] != 0x00 {
            n += 1;
        }
        self.anchor = (self.anchor + n) & mask;
        n as u16
    }
}

/// osc-CRC over a possibly-wrapped ring span.
fn ring_crc(ring: &[u8], start: usize, n: usize) -> u16 {
    let head = n.min(ring.len() - start);
    let crc = osc_crc(&ring[start..start + head]);
    if head == n {
        crc
    } else {
        osc_crc_continue(crc, &ring[..n - head])
    }
}

/// Zero-copy view over a possibly-wrapped ring span.
fn ring_bytes(ring: &[u8], start: usize, n: usize) -> FrameBytes<'_> {
    let head = n.min(ring.len() - start);
    FrameBytes::new(&ring[start..start + head], &ring[..n - head])
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_protocol::reply::FrameBuf;
    use osc_protocol::wire::{Opcode, ResultCode};

    /// A 64-byte test ring with a write cursor.
    struct Ring {
        buf: [u8; 64],
        cursor: usize,
    }

    impl Ring {
        fn new() -> Self {
            Ring {
                buf: [0xFF; 64],
                cursor: 0,
            }
        }

        fn feed(&mut self, bytes: &[u8]) {
            for &b in bytes {
                self.buf[self.cursor] = b;
                self.cursor = (self.cursor + 1) & 63;
            }
        }
    }

    fn status_frame(id: u8, result: ResultCode, payload: &[u8]) -> heapless_vec::Vec {
        let mut b = FrameBuf::<64>::new();
        b.start(Id::new(id), Inst::status(result, false));
        b.payload_mut()[..payload.len()].copy_from_slice(payload);
        b.finish(payload.len() as u8);
        heapless_vec::Vec::from(b.seal())
    }

    /// Minimal fixed vec so tests stay no_std-friendly without heapless.
    mod heapless_vec {
        pub struct Vec {
            buf: [u8; 64],
            len: usize,
        }
        impl Vec {
            pub fn from(s: &[u8]) -> Self {
                let mut buf = [0u8; 64];
                buf[..s.len()].copy_from_slice(s);
                Vec { buf, len: s.len() }
            }
            pub fn as_slice(&self) -> &[u8] {
                &self.buf[..self.len]
            }
        }
    }

    fn expect_frame<'a>(step: Step<'a>) -> Frame<'a> {
        match step {
            Step::Frame(f) => f,
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    #[test]
    fn clean_status_frame_parses() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let frame = status_frame(5, ResultCode::Ok, &[0xAA, 0xBB]);
        ring.feed(frame.as_slice());

        let got = expect_frame(f.step(&ring.buf, ring.cursor));
        assert_eq!(got.id, Id::new(5));
        assert!(got.inst.is_status());
        assert_eq!(got.inst.result(), Some(ResultCode::Ok));
        assert!(got.payload.bytes().eq([0xAA, 0xBB]));
        assert!(matches!(f.step(&ring.buf, ring.cursor), Step::Idle));
    }

    #[test]
    fn back_to_back_frames_walk_in_order() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(status_frame(1, ResultCode::Ok, &[1]).as_slice());
        ring.feed(status_frame(2, ResultCode::Ok, &[2]).as_slice());

        assert_eq!(expect_frame(f.step(&ring.buf, ring.cursor)).id, Id::new(1));
        assert_eq!(expect_frame(f.step(&ring.buf, ring.cursor)).id, Id::new(2));
        assert!(matches!(f.step(&ring.buf, ring.cursor), Step::Idle));
    }

    #[test]
    fn frame_split_across_the_wrap_parses() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        // Park the cursor near the top so the frame wraps.
        ring.feed(&[0xEE; 60]);
        f.resync(60);
        ring.feed(status_frame(7, ResultCode::Ok, &[0x11, 0x22, 0x33]).as_slice());

        let got = expect_frame(f.step(&ring.buf, ring.cursor));
        assert_eq!(got.id, Id::new(7));
        assert!(got.payload.bytes().eq([0x11, 0x22, 0x33]));
    }

    #[test]
    fn junk_before_a_frame_is_skipped_then_parsed() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(&[0xAA, 0xBB, 0xCC]);
        ring.feed(status_frame(3, ResultCode::Ok, &[]).as_slice());

        assert!(matches!(f.step(&ring.buf, ring.cursor), Step::Garble(3)));
        assert_eq!(expect_frame(f.step(&ring.buf, ring.cursor)).id, Id::new(3));
    }

    #[test]
    fn corrupt_crc_dies_by_hunt() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let good = status_frame(4, ResultCode::Ok, &[9, 9]);
        let mut bad = [0u8; 64];
        let n = good.as_slice().len();
        bad[..n].copy_from_slice(good.as_slice());
        bad[n - 1] ^= 0xFF;
        ring.feed(&bad[..n]);

        // The corrupt frame anchors, fails CRC, and the hunt walks off it;
        // successive steps consume the remaining junk without ever yielding
        // a frame.
        let mut frames = 0;
        loop {
            match f.step(&ring.buf, ring.cursor) {
                Step::Frame(_) => frames += 1,
                Step::Garble(_) | Step::Partial => continue,
                Step::Idle => break,
            }
        }
        assert_eq!(frames, 0);
    }

    #[test]
    fn partial_frame_holds_then_completes() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let frame = status_frame(6, ResultCode::Ok, &[1, 2, 3, 4]);
        let bytes = frame.as_slice();
        ring.feed(&bytes[..3]);
        assert!(matches!(f.step(&ring.buf, ring.cursor), Step::Partial));
        ring.feed(&bytes[3..5]);
        assert!(matches!(f.step(&ring.buf, ring.cursor), Step::Partial));
        ring.feed(&bytes[5..]);
        assert_eq!(expect_frame(f.step(&ring.buf, ring.cursor)).id, Id::new(6));
    }

    #[test]
    fn payload_zeros_do_not_confuse_the_walk() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(status_frame(8, ResultCode::Ok, &[0, 0, 0, 0]).as_slice());
        ring.feed(status_frame(9, ResultCode::Ok, &[0]).as_slice());

        assert_eq!(expect_frame(f.step(&ring.buf, ring.cursor)).id, Id::new(8));
        assert_eq!(expect_frame(f.step(&ring.buf, ring.cursor)).id, Id::new(9));
    }

    #[test]
    fn masquerading_zero_dies_by_geometry_and_the_real_frame_survives() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        // 0x00 followed by an invalid id (0x00): a fake anchor that fails
        // shape validation; the hunt must find the real frame behind it.
        ring.feed(&[0x00, 0x00, 0x55]);
        ring.feed(status_frame(2, ResultCode::Ok, &[7]).as_slice());

        let mut got = None;
        for _ in 0..8 {
            match f.step(&ring.buf, ring.cursor) {
                Step::Frame(fr) => {
                    got = Some(fr.id);
                    break;
                }
                _ => continue,
            }
        }
        assert_eq!(got, Some(Id::new(2)));
    }

    #[test]
    fn instruction_frames_parse_too() {
        // The walker is direction-neutral: a snooped instruction (or a
        // rogue-talker frame) parses and classifies by INST bit 7.
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let mut b = FrameBuf::<64>::new();
        b.start(Id::new(1), Inst::instruction(Opcode::Ping, 0));
        b.finish(0);
        ring.feed(b.seal());

        let got = expect_frame(f.step(&ring.buf, ring.cursor));
        assert!(!got.inst.is_status());
        assert_eq!(got.inst.opcode(), Some(Opcode::Ping));
    }
}
