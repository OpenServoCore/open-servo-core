//! Host RX framer: a ring walker anchored by break marks.
//!
//! The chip marks every received break with the ring position of its `0x00`
//! byte (`RxRing::marks`). A candidate frame starts only at a mark, the
//! header makes its end computable (LEN at byte 2), and the CRC verdict is
//! the only accept authority. A frame never contains a break, so a claim
//! that spans a later mark is junk and the walk restarts exactly at that
//! mark; bytes no mark heads are junk too. Positions come from the marks and
//! the ring count only (protocol sec 3.4); every clock stays in the engine.
//!
//! A mark trails its byte by the detector's service time, so the walk never
//! passes the last mark: bytes behind it stay unresolved until the next
//! mark lands or the engine's window closes.

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
    /// Unconsumed bytes wait for a frame's remainder or the next mark;
    /// positions hold. Whether either ever lands is the engine's deadline.
    Partial,
    /// One frame consumed.
    Frame(Frame<'a>),
    /// `n` ring bytes skipped that anchored no frame.
    Garble(u16),
    /// `n` whole rings landed unread: the walk resumes at the same ring
    /// index, over newer bytes.
    Lapped(u16),
}

/// The walker: a cursor (`anchor`) on the
/// [`RxRing::written`](crate::traits::RxRing::written) count and one on the
/// marks.
#[derive(Default)]
pub struct Framer {
    anchor: u32,
    /// The next mark not yet spent.
    mark: u32,
}

impl Framer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Jump to `written`/`marked`, discarding anything unconsumed. Called
    /// only when the bus is provably quiet (before a TX window opens) -
    /// the host-side analog of the servo's bootstrap-only cursor read.
    pub fn resync(&mut self, written: u32, marked: u32) {
        self.anchor = written;
        self.mark = marked;
    }

    /// Ring content no verdict has consumed (a frame still ringing, junk no
    /// later mark has closed).
    pub fn unresolved(&self, written: u32) -> u16 {
        u16::try_from(written.wrapping_sub(self.anchor)).unwrap_or(u16::MAX)
    }

    /// Walk one step. `marked` must be loaded before `written` (trait
    /// contract); both ring lengths must be powers of two.
    pub fn step<'a>(
        &mut self,
        ring: &'a [u8],
        written: u32,
        marks: &[u32],
        marked: u32,
    ) -> Step<'a> {
        debug_assert!(ring.len().is_power_of_two() && marks.len().is_power_of_two());
        let unread = written.wrapping_sub(self.anchor);
        let laps = unread / ring.len() as u32;
        if laps > 0 {
            self.anchor = self.anchor.wrapping_add(laps * ring.len() as u32);
            return Step::Lapped(u16::try_from(laps).unwrap_or(u16::MAX));
        }
        let Some(at_mark) = self.next_mark(marks, marked, unread) else {
            return if unread == 0 {
                Step::Idle
            } else {
                Step::Partial
            };
        };
        if at_mark != self.anchor {
            return self.skip_to(at_mark);
        }
        let mask = ring.len() - 1;
        let at = self.anchor as usize & mask;
        let verdict = probe(ring, mask, at, unread as usize);
        let here = self.mark;
        self.mark = here.wrapping_add(1);
        if let Probe::Clean { id, inst, len } = verdict {
            let payload_pos = (at + 4) & mask;
            let payload = payload_view(ring, payload_pos, wire::payload_len(len) as usize);
            self.anchor = self.anchor.wrapping_add(wire::footprint(len) as u32);
            return Step::Frame(Frame {
                id,
                inst,
                payload,
                payload_pos,
            });
        }
        // Dead, or a claim that a later mark lands inside: the walk
        // restarts at that mark. A dead break stays spent; a plausible
        // claim with no later mark keeps waiting.
        match self.next_mark(marks, marked, unread) {
            Some(next) => self.skip_to(next),
            None => {
                if let Probe::Partial = verdict {
                    self.mark = here;
                }
                Step::Partial
            }
        }
    }

    /// The first unspent mark at or past the anchor whose byte has landed.
    /// Marks behind the anchor are spent; a lapped mark buffer resumes at
    /// its oldest entry.
    fn next_mark(&mut self, marks: &[u32], marked: u32, unread: u32) -> Option<u32> {
        if marked.wrapping_sub(self.mark) > marks.len() as u32 {
            self.mark = marked.wrapping_sub(marks.len() as u32);
        }
        while self.mark != marked {
            let at = *marks.get(self.mark as usize & (marks.len().wrapping_sub(1)))?;
            let ahead = at.wrapping_sub(self.anchor);
            if (ahead as i32) >= 0 {
                return (ahead < unread).then_some(at);
            }
            self.mark = self.mark.wrapping_add(1);
        }
        None
    }

    fn skip_to(&mut self, at: u32) -> Step<'static> {
        let n = at.wrapping_sub(self.anchor);
        self.anchor = at;
        Step::Garble(n as u16)
    }
}

/// Verdict on the candidate anchored at ring index `at` with `avail` ring
/// bytes from it.
enum Probe {
    /// Dead by shape or CRC.
    Junk,
    /// Plausible so far; its footprint has not fully ringed.
    Partial,
    Clean {
        id: Id,
        inst: Inst,
        len: u8,
    },
}

fn probe(ring: &[u8], mask: usize, at: usize, avail: usize) -> Probe {
    if ring[at] != 0x00 {
        return Probe::Junk;
    }
    if avail < 4 {
        return Probe::Partial;
    }
    let id = Id::new(ring[(at + 1) & mask]);
    let len = ring[(at + 2) & mask];
    let inst = Inst(ring[(at + 3) & mask]);
    let valid_shape = len >= 3
        && id.is_valid()
        && (inst.is_status() || inst.opcode().is_some())
        && wire::footprint(len) <= ring.len();
    if !valid_shape {
        return Probe::Junk;
    }
    if avail < wire::footprint(len) {
        return Probe::Partial;
    }

    // CRC over the anchor-inclusive covered span: the leading 0x00 is an
    // init-0 no-op (protocol sec 3.2), so feeding from the anchor equals
    // the wire checksum over ID..payload.
    let clen = wire::covered_len(len);
    let crc = ring_crc(ring, at, clen);
    let lo = ring[(at + clen) & mask];
    let hi = ring[(at + clen + 1) & mask];
    if crc != u16::from_le_bytes([lo, hi]) {
        return Probe::Junk;
    }
    Probe::Clean { id, inst, len }
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

    /// A 64-byte test ring with a running write count and 8 break marks.
    struct Ring {
        buf: [u8; 64],
        written: u32,
        marks: [u32; 8],
        marked: u32,
    }

    impl Ring {
        fn new() -> Self {
            Ring {
                buf: [0xFF; 64],
                written: 0,
                marks: [0; 8],
                marked: 0,
            }
        }

        /// Bytes no break heads.
        fn feed(&mut self, bytes: &[u8]) {
            for &b in bytes {
                self.buf[self.written as usize & 63] = b;
                self.written += 1;
            }
        }

        /// A break's byte plus what follows it: marked as the detector would.
        fn frame(&mut self, bytes: &[u8]) {
            let at = self.written;
            self.feed(bytes);
            self.mark(at);
        }

        fn mark(&mut self, at: u32) {
            self.marks[self.marked as usize & 7] = at;
            self.marked += 1;
        }

        fn step(&self, f: &mut Framer) -> Step<'_> {
            f.step(&self.buf, self.written, &self.marks, self.marked)
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

    /// Walk to `Idle` or `Partial`; the ids of the frames found.
    fn walk(f: &mut Framer, ring: &Ring) -> std::vec::Vec<Id> {
        let mut ids = std::vec::Vec::new();
        loop {
            match ring.step(f) {
                Step::Frame(fr) => ids.push(fr.id),
                Step::Garble(_) | Step::Lapped(_) => {}
                Step::Partial | Step::Idle => return ids,
            }
        }
    }

    #[test]
    fn clean_status_frame_parses() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.frame(status_frame(5, ResultCode::Ok, &[0xAA, 0xBB]).as_slice());

        let got = expect_frame(ring.step(&mut f));
        assert_eq!(got.id, Id::new(5));
        assert!(got.inst.is_status());
        assert_eq!(got.inst.result(), Some(ResultCode::Ok));
        assert!(got.payload.bytes().eq([0xAA, 0xBB]));
        assert!(matches!(ring.step(&mut f), Step::Idle));
    }

    #[test]
    fn back_to_back_frames_walk_in_order() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.frame(status_frame(1, ResultCode::Ok, &[1]).as_slice());
        ring.frame(status_frame(2, ResultCode::Ok, &[2]).as_slice());

        assert_eq!(expect_frame(ring.step(&mut f)).id, Id::new(1));
        assert_eq!(expect_frame(ring.step(&mut f)).id, Id::new(2));
        assert!(matches!(ring.step(&mut f), Step::Idle));
    }

    #[test]
    fn frame_split_across_the_wrap_parses() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        // Park the cursor near the top so the frame wraps.
        ring.feed(&[0xEE; 60]);
        f.resync(60, ring.marked);
        ring.frame(status_frame(7, ResultCode::Ok, &[0x11, 0x22, 0x33]).as_slice());

        let got = expect_frame(ring.step(&mut f));
        assert_eq!(got.id, Id::new(7));
        assert!(got.payload.bytes().eq([0x11, 0x22, 0x33]));
    }

    #[test]
    fn bytes_no_break_heads_are_skipped_up_to_the_next_break() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(&[0xAA, 0x00, 0xCC]);
        ring.frame(status_frame(3, ResultCode::Ok, &[]).as_slice());

        assert!(matches!(ring.step(&mut f), Step::Garble(3)));
        assert_eq!(expect_frame(ring.step(&mut f)).id, Id::new(3));
    }

    #[test]
    fn a_corrupt_frame_is_junk_and_never_a_frame() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let good = status_frame(4, ResultCode::Ok, &[9, 9]);
        let mut bad = [0u8; 64];
        let n = good.as_slice().len();
        bad[..n].copy_from_slice(good.as_slice());
        bad[n - 1] ^= 0xFF;
        ring.frame(&bad[..n]);

        assert_eq!(walk(&mut f, &ring), []);
        assert_eq!(f.unresolved(ring.written), n as u16);
        ring.frame(status_frame(2, ResultCode::Ok, &[7]).as_slice());
        assert_eq!(walk(&mut f, &ring), [Id::new(2)]);
    }

    #[test]
    fn partial_frame_holds_then_completes() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let frame = status_frame(6, ResultCode::Ok, &[1, 2, 3, 4]);
        let bytes = frame.as_slice();
        ring.frame(&bytes[..3]);
        assert!(matches!(ring.step(&mut f), Step::Partial));
        ring.feed(&bytes[3..5]);
        assert!(matches!(ring.step(&mut f), Step::Partial));
        ring.feed(&bytes[5..]);
        assert_eq!(expect_frame(ring.step(&mut f)).id, Id::new(6));
    }

    #[test]
    fn payload_zeros_do_not_confuse_the_walk() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.frame(status_frame(8, ResultCode::Ok, &[0, 0, 0, 0]).as_slice());
        ring.frame(status_frame(9, ResultCode::Ok, &[0]).as_slice());

        assert_eq!(expect_frame(ring.step(&mut f)).id, Id::new(8));
        assert_eq!(expect_frame(ring.step(&mut f)).id, Id::new(9));
    }

    #[test]
    fn a_false_length_never_holds_the_walk_over_the_next_break() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        // A break whose header claims 51 bytes; the next break lands 6
        // bytes later, inside the claim.
        ring.frame(&[0x00, 0x05, 0x30, 0xA4, 1, 2]);
        assert_eq!(walk(&mut f, &ring), []);
        ring.frame(status_frame(2, ResultCode::Ok, &[7]).as_slice());

        assert_eq!(walk(&mut f, &ring), [Id::new(2)]);
        assert_eq!(f.unresolved(ring.written), 0);
    }

    #[test]
    fn a_long_masquerade_in_a_corrupt_frame_never_parks_the_walk_over_the_next() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        // A data 0x00 whose next bytes read as id 5, LEN 0x30, a status INST:
        // a 51-byte claim that outruns everything left on the wire. A flipped
        // body byte behind it fails the real frame's CRC.
        let good = status_frame(4, ResultCode::Ok, &[0x00, 0x05, 0x30, 0xA4, 1, 2, 3]);
        let mut bad = [0u8; 64];
        let n = good.as_slice().len();
        bad[..n].copy_from_slice(good.as_slice());
        bad[9] ^= 0xFF;
        ring.frame(&bad[..n]);
        ring.frame(status_frame(2, ResultCode::Ok, &[7]).as_slice());

        assert_eq!(walk(&mut f, &ring), [Id::new(2)]);
    }

    #[test]
    fn a_frame_still_ringing_is_never_overtaken_by_an_image_inside_it() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let inner = status_frame(3, ResultCode::Ok, &[1]);
        let mut payload = [9u8; 12];
        payload[..inner.as_slice().len()].copy_from_slice(inner.as_slice());
        let outer = status_frame(6, ResultCode::Ok, &payload);
        let bytes = outer.as_slice();
        // Through the inner image's last byte: it is whole and CRC-clean.
        ring.frame(&bytes[..4 + inner.as_slice().len() + 1]);
        assert_eq!(walk(&mut f, &ring), []);
        ring.feed(&bytes[4 + inner.as_slice().len() + 1..]);
        assert_eq!(walk(&mut f, &ring), [Id::new(6)]);
    }

    #[test]
    fn a_frame_whose_mark_has_not_landed_waits_for_it() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.frame(status_frame(1, ResultCode::Ok, &[1]).as_slice());
        let at = ring.written;
        ring.feed(status_frame(2, ResultCode::Ok, &[2]).as_slice());

        assert_eq!(walk(&mut f, &ring), [Id::new(1)]);
        assert_eq!(f.unresolved(ring.written), 7);
        ring.mark(at);
        assert_eq!(walk(&mut f, &ring), [Id::new(2)]);
    }

    #[test]
    fn a_mark_past_the_written_count_is_not_yet_visible() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let frame = status_frame(1, ResultCode::Ok, &[1]);
        ring.frame(&frame.as_slice()[..3]);
        ring.mark(ring.written + 2);
        assert!(matches!(ring.step(&mut f), Step::Partial));
        ring.feed(&frame.as_slice()[3..]);
        assert_eq!(walk(&mut f, &ring), [Id::new(1)]);
    }

    #[test]
    fn a_mark_on_a_data_byte_is_junk() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        let at = ring.written + 1;
        ring.feed(status_frame(1, ResultCode::Ok, &[1]).as_slice());
        ring.mark(at);
        assert_eq!(walk(&mut f, &ring), []);
        ring.frame(status_frame(2, ResultCode::Ok, &[2]).as_slice());
        assert_eq!(walk(&mut f, &ring), [Id::new(2)]);
    }

    #[test]
    fn a_lapped_mark_buffer_resumes_at_its_oldest_mark() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        for _ in 0..9 {
            ring.frame(&[0x00]);
        }
        ring.frame(status_frame(2, ResultCode::Ok, &[2]).as_slice());
        assert_eq!(walk(&mut f, &ring), [Id::new(2)]);
    }

    #[test]
    fn a_ring_landed_unread_is_a_lap_not_an_empty_ring() {
        let mut ring = Ring::new();
        let mut f = Framer::new();
        ring.feed(&[0xEE; 64]);
        assert!(matches!(ring.step(&mut f), Step::Lapped(1)));
        assert!(matches!(ring.step(&mut f), Step::Idle));

        ring.feed(&[0xEE; 3 * 64 + 10]);
        assert!(matches!(ring.step(&mut f), Step::Lapped(3)));
        assert!(matches!(ring.step(&mut f), Step::Partial));
        ring.frame(status_frame(2, ResultCode::Ok, &[2]).as_slice());
        assert!(matches!(ring.step(&mut f), Step::Garble(10)));
        assert_eq!(expect_frame(ring.step(&mut f)).id, Id::new(2));
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
        ring.frame(b.seal());

        let got = expect_frame(ring.step(&mut f));
        assert!(!got.inst.is_status());
        assert_eq!(got.inst.opcode(), Some(Opcode::Ping));
    }
}
