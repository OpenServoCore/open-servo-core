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

/// One CRC-clean frame view over the ring. `payload_pos` names the
/// payload's ring index so a caller can re-materialize the view after the
/// borrow ends (see [`payload_view`]).
#[derive(Debug)]
pub struct Frame<'a> {
    pub id: Id,
    pub inst: Inst,
    pub payload: FrameBytes<'a>,
    pub payload_pos: usize,
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
    /// `n` whole rings landed unread: the walk resumes at the same ring
    /// index, over newer bytes.
    Lapped(u16),
}

/// The walker: one cursor (`anchor`) into the ring, on the
/// [`RxRing::written`](crate::traits::RxRing::written) count.
#[derive(Default)]
pub struct Framer {
    anchor: u32,
}

impl Framer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Jump to `written`, discarding anything unconsumed. Called only when
    /// the bus is provably quiet (before a TX window opens) -- the host-side
    /// analog of the servo's bootstrap-only cursor read.
    pub fn resync(&mut self, written: u32) {
        self.anchor = written;
    }

    /// Ring content no verdict has consumed (a parked plausible prefix,
    /// junk a hunt has not walked off).
    pub fn unresolved(&self, written: u32) -> u16 {
        u16::try_from(written.wrapping_sub(self.anchor)).unwrap_or(u16::MAX)
    }

    /// Walk one step against the ring state. `written` counts every byte
    /// the ring ever received; the ring length must be a power of two.
    pub fn step<'a>(&mut self, ring: &'a [u8], written: u32) -> Step<'a> {
        debug_assert!(ring.len().is_power_of_two());
        let unread = written.wrapping_sub(self.anchor);
        let laps = unread / ring.len() as u32;
        if laps > 0 {
            self.anchor = self.anchor.wrapping_add(laps * ring.len() as u32);
            return Step::Lapped(u16::try_from(laps).unwrap_or(u16::MAX));
        }
        let avail = unread as usize;
        if avail == 0 {
            return Step::Idle;
        }
        let mask = ring.len() - 1;
        let at = self.anchor as usize & mask;

        // A frame candidate starts at the break's 0x00 ring byte.
        if ring[at] != 0x00 {
            return Step::Garble(self.hunt(ring, mask, avail));
        }
        if avail < 4 {
            return Step::Partial;
        }

        let id = Id::new(ring[(at + 1) & mask]);
        let len = ring[(at + 2) & mask];
        let inst = Inst(ring[(at + 3) & mask]);
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
        let crc = ring_crc(ring, at, clen);
        let lo = ring[(at + clen) & mask];
        let hi = ring[(at + clen + 1) & mask];
        if crc != u16::from_le_bytes([lo, hi]) {
            return Step::Garble(self.hunt(ring, mask, avail));
        }

        let payload_pos = (at + 4) & mask;
        let payload = payload_view(ring, payload_pos, wire::payload_len(len) as usize);
        self.anchor = self.anchor.wrapping_add(footprint as u32);
        Step::Frame(Frame {
            id,
            inst,
            payload,
            payload_pos,
        })
    }

    /// Advance past a dead candidate to the next `0x00` within `avail`
    /// (skipping the current anchor -- it already failed), or consume the
    /// whole span if none. Returns the bytes skipped.
    fn hunt(&mut self, ring: &[u8], mask: usize, avail: usize) -> u16 {
        let at = self.anchor as usize & mask;
        let mut n = 1;
        while n < avail && ring[(at + n) & mask] != 0x00 {
            n += 1;
        }
        self.anchor = self.anchor.wrapping_add(n as u32);
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

/// Zero-copy view over a possibly-wrapped ring span. Public within the
/// engine so a consumed frame's payload can be re-materialized from its
/// [`Frame::payload_pos`] once the walk's borrow has ended.
pub(crate) fn payload_view(ring: &[u8], start: usize, n: usize) -> FrameBytes<'_> {
    let head = n.min(ring.len() - start);
    FrameBytes::new(&ring[start..start + head], &ring[..n - head])
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_protocol::reply::FrameBuf;
    use osc_protocol::wire::{Opcode, ResultCode};

    /// A 64-byte test ring with a running write count.
    struct Ring {
        buf: [u8; 64],
        written: u32,
    }

    impl Ring {
        fn new() -> Self {
            Ring {
                buf: [0xFF; 64],
                written: 0,
            }
        }

        fn feed(&mut self, bytes: &[u8]) {
            for &b in bytes {
                self.buf[self.written as usize & 63] = b;
                self.written += 1;
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

        let got = expect_frame(f.step(&ring.buf, ring.written));
        assert_eq!(got.id, Id::new(5));
        assert!(got.inst.is_status());
        assert_eq!(got.inst.result(), Some(ResultCode::Ok));
        assert!(got.payload.bytes().eq([0xAA, 0xBB]));
        assert!(matches!(f.step(&ring.buf, ring.written), Step::Idle));
    }

    #[test]
    fn back_to_back_frames_walk_in_order() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(status_frame(1, ResultCode::Ok, &[1]).as_slice());
        ring.feed(status_frame(2, ResultCode::Ok, &[2]).as_slice());

        assert_eq!(expect_frame(f.step(&ring.buf, ring.written)).id, Id::new(1));
        assert_eq!(expect_frame(f.step(&ring.buf, ring.written)).id, Id::new(2));
        assert!(matches!(f.step(&ring.buf, ring.written), Step::Idle));
    }

    #[test]
    fn frame_split_across_the_wrap_parses() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        // Park the cursor near the top so the frame wraps.
        ring.feed(&[0xEE; 60]);
        f.resync(60);
        ring.feed(status_frame(7, ResultCode::Ok, &[0x11, 0x22, 0x33]).as_slice());

        let got = expect_frame(f.step(&ring.buf, ring.written));
        assert_eq!(got.id, Id::new(7));
        assert!(got.payload.bytes().eq([0x11, 0x22, 0x33]));
    }

    #[test]
    fn junk_before_a_frame_is_skipped_then_parsed() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(&[0xAA, 0xBB, 0xCC]);
        ring.feed(status_frame(3, ResultCode::Ok, &[]).as_slice());

        assert!(matches!(f.step(&ring.buf, ring.written), Step::Garble(3)));
        assert_eq!(expect_frame(f.step(&ring.buf, ring.written)).id, Id::new(3));
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
            match f.step(&ring.buf, ring.written) {
                Step::Frame(_) => frames += 1,
                Step::Garble(_) | Step::Partial | Step::Lapped(_) => continue,
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
        assert!(matches!(f.step(&ring.buf, ring.written), Step::Partial));
        ring.feed(&bytes[3..5]);
        assert!(matches!(f.step(&ring.buf, ring.written), Step::Partial));
        ring.feed(&bytes[5..]);
        assert_eq!(expect_frame(f.step(&ring.buf, ring.written)).id, Id::new(6));
    }

    #[test]
    fn payload_zeros_do_not_confuse_the_walk() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(status_frame(8, ResultCode::Ok, &[0, 0, 0, 0]).as_slice());
        ring.feed(status_frame(9, ResultCode::Ok, &[0]).as_slice());

        assert_eq!(expect_frame(f.step(&ring.buf, ring.written)).id, Id::new(8));
        assert_eq!(expect_frame(f.step(&ring.buf, ring.written)).id, Id::new(9));
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
            match f.step(&ring.buf, ring.written) {
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
    fn a_ring_landed_unread_is_a_lap_not_an_empty_ring() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(&[0xEE; 64]);
        assert!(matches!(f.step(&ring.buf, ring.written), Step::Lapped(1)));
        assert!(matches!(f.step(&ring.buf, ring.written), Step::Idle));

        ring.feed(&[0xEE; 3 * 64 + 10]);
        assert!(matches!(f.step(&ring.buf, ring.written), Step::Lapped(3)));
        assert!(matches!(f.step(&ring.buf, ring.written), Step::Garble(10)));
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

        let got = expect_frame(f.step(&ring.buf, ring.written));
        assert!(!got.inst.is_status());
        assert_eq!(got.inst.opcode(), Some(Opcode::Ping));
    }
}
