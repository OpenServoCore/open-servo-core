//! Two-state break-framed RX framer (`docs/osc-native-protocol.md` §4.1).
//!
//! Pure state machine: the composite owns the ring, deadline, and CRC
//! providers and drives this with plain values. Every legal frame fits whole
//! in the ring by construction (258 B max vs 512 B ring, §3.1), so there is no
//! chunked consumption — HUNT for a break, then two computed deadlines verify
//! the header and the frame end against the ring cursor.

use osc_protocol::frame::Header;

/// A verified frame's location in the ring: anchor index + ring-byte count,
/// plus the parser-derived wire-end estimate (`packet_end`) the reply
/// trigger's T_turn is measured from (§7). Estimated from the break's FE
/// tick + footprint byte-times + the drift adder, so it is conservative
/// (never earlier than the true end for an in-spec transmitter).
pub struct FrameSpan {
    pub anchor: u16,
    pub footprint: u16,
    pub packet_end: u32,
}

/// What the framer wants from the composite after an event.
pub enum FramerOut {
    /// Idle; no framer deadline wanted.
    None,
    /// Arm/refresh the framer deadline at this absolute tick.
    Wait(u32),
    /// The CRC-covered span is fully ringed — only the 2 wire-CRC bytes are
    /// still inbound — so the composite may front-load the CRC feed + dispatch
    /// now. The frame-end deadline is already armed at `end_due`; a matching
    /// [`FramerOut::Frame`] follows there. Emitted at most once per locked
    /// frame, and never for doomed frames.
    Covered { span: FrameSpan, end_due: u32 },
    /// A full frame is verified in the ring (header pre-validated); back in HUNT.
    Frame(FrameSpan),
}

enum State {
    Hunt,
    AwaitHeader {
        anchor: u16,
        /// FE-entry tick of the anchoring break — the break byte's wire end
        /// (F2/F5: the ring byte lands and the line rises by ISR entry).
        anchor_tick: u32,
        rechecks: u8,
        due: u32,
    },
    AwaitEnd {
        anchor: u16,
        footprint: u16,
        /// Parser-derived wire-end estimate (see [`FrameSpan::packet_end`]).
        packet_end: u32,
        /// Frame-end deadline, computed once at lock (the covered checkpoint
        /// re-arms to exactly this — end timing is independent of the checkpoint).
        end_due: u32,
        rechecks: u8,
        /// The covered-complete checkpoint has already fired for this frame.
        covered_seen: bool,
        due: u32,
    },
}

// §3.2/F2: a real break DMAs an all-zero byte into the ring; a nonzero byte at
// an FE event is line garble, not a frame start.
const BREAK_RING_BYTE: u8 = 0x00;

// §4.1: header = BREAK, ID, LEN, INST. `HEADER_SPAN_BYTES` bytes from the
// anchor make the header readable; the break byte is already ringed at FE
// entry, so `HEADER_LEAD_BYTES` more byte-times are still inbound.
const HEADER_SPAN_BYTES: u16 = Header::SIZE as u16;
const HEADER_LEAD_BYTES: u32 = HEADER_SPAN_BYTES as u32 - 1;

// Half a byte-time of ISR-entry slack folded onto deadline A.
const ISR_SLACK_DIV: u32 = 2;

// Deadline B slack past the packet-end estimate: a fixed, baud-independent
// tick count (ISR-entry + settle latencies are silicon-time, not wire-time;
// the wire-proportional drift term lives in `packet_end` itself). The slack
// bounds recheck count only, not correctness — every wake is
// cursor-verified.
const DRIFT_SHIFT: u32 = 6;

// The wire CRC tail is 2 bytes: the covered span completes this many byte-times
// before the frame end, so the covered checkpoint leads deadline B by it.
const COVERED_TAIL_BYTES: u32 = 2;

// Plateau backstops (§4.1): bounded rechecks one byte-time apart past the
// computed deadline before abandoning a stalled header/frame (host died
// mid-frame; the next break re-anchors and recovers).
const HEADER_RECHECKS: u8 = 8;
const END_RECHECKS: u8 = 8;

pub struct Framer {
    state: State,
    drops: u32,
    /// Fixed tick slack past the packet-end estimate for deadline B.
    end_slack: u32,
}

impl Framer {
    pub const fn new(end_slack: u32) -> Self {
        Self {
            state: State::Hunt,
            drops: 0,
            end_slack,
        }
    }

    /// Frames dropped at transport layer 1 (§5.3), monotonic.
    pub fn drops(&self) -> u32 {
        self.drops
    }

    /// Back to HUNT (e.g. on rescue-baud entry).
    pub fn abort(&mut self) {
        self.state = State::Hunt;
    }

    /// FE event (break or mid-frame garble). `cursor` already counts the FE
    /// byte's ring slot (provider contract), so the FE byte sits one slot back.
    pub fn on_break(&mut self, ring: &[u8], cursor: u16, now: u32, tpb: u32) -> FramerOut {
        let Some(byte) = fe_byte(ring, cursor) else {
            return FramerOut::None; // defensive: an empty ring holds no frame
        };
        let is_break = byte == BREAK_RING_BYTE;
        let locked_due = match &self.state {
            State::Hunt => Option::None,
            State::AwaitHeader { due, .. } | State::AwaitEnd { due, .. } => Some(*due),
        };
        match locked_due {
            // HUNT: only a real break (0x00 ring byte) anchors a frame.
            Option::None => {
                if is_break {
                    self.anchor_here(ring, cursor, now, tpb)
                } else {
                    FramerOut::None
                }
            }
            Some(due) => {
                if is_break {
                    // A fresh break preempts the in-flight frame (host restart /
                    // back-to-back after garble): the old frame is lost.
                    self.drops = self.drops.wrapping_add(1);
                    self.anchor_here(ring, cursor, now, tpb)
                } else {
                    // §3.1/F4: mid-frame garble rang its byte and the stream
                    // continues; the frame fails CRC downstream. Nothing changes
                    // here — but the pending deadline must survive, so re-emit it.
                    FramerOut::Wait(due)
                }
            }
        }
    }

    /// The framer's own deadline fired (the composite muxes deadlines).
    pub fn on_deadline(&mut self, ring: &[u8], cursor: u16, now: u32, tpb: u32) -> FramerOut {
        let len = ring.len();
        if len == 0 {
            self.state = State::Hunt;
            return FramerOut::None; // defensive: no ring to measure against
        }
        match self.state {
            // No deadline is armed in HUNT; a stray wake is a no-op (defensive).
            State::Hunt => FramerOut::None,
            State::AwaitHeader {
                anchor,
                anchor_tick,
                rechecks,
                ..
            } => {
                let received = dist(cursor, anchor, len);
                if received < HEADER_SPAN_BYTES {
                    self.recheck_header(anchor, anchor_tick, rechecks, now, tpb)
                } else {
                    self.lock_end(ring, anchor, anchor_tick, received, tpb)
                }
            }
            State::AwaitEnd {
                anchor,
                footprint,
                packet_end,
                end_due,
                rechecks,
                covered_seen,
                ..
            } => {
                let received = dist(cursor, anchor, len);
                // Covered-complete checkpoint: fire once, before the end, so the
                // composite can front-load CRC + dispatch. Re-arm at the
                // pre-computed end so deadline B stays where lock put it.
                if !covered_seen {
                    if received >= footprint - COVERED_TAIL_BYTES as u16 {
                        self.state = State::AwaitEnd {
                            anchor,
                            footprint,
                            packet_end,
                            end_due,
                            rechecks: 0,
                            covered_seen: true,
                            due: end_due,
                        };
                        return FramerOut::Covered {
                            span: FrameSpan {
                                anchor,
                                footprint,
                                packet_end,
                            },
                            end_due,
                        };
                    }
                } else if received >= footprint {
                    self.state = State::Hunt;
                    return FramerOut::Frame(FrameSpan {
                        anchor,
                        footprint,
                        packet_end,
                    });
                }
                // Short of the current checkpoint (covered or end): bounded
                // recheck one byte-time out before abandoning a stalled frame.
                if rechecks >= END_RECHECKS {
                    // Host died mid-frame; the next break recovers.
                    self.drops = self.drops.wrapping_add(1);
                    self.state = State::Hunt;
                    FramerOut::None
                } else {
                    let due = now.wrapping_add(tpb);
                    self.state = State::AwaitEnd {
                        anchor,
                        footprint,
                        packet_end,
                        end_due,
                        rechecks: rechecks + 1,
                        covered_seen,
                        due,
                    };
                    FramerOut::Wait(due)
                }
            }
        }
    }

    fn anchor_here(&mut self, ring: &[u8], cursor: u16, now: u32, tpb: u32) -> FramerOut {
        let Some(anchor) = fe_index(ring, cursor) else {
            return FramerOut::None;
        };
        let due = now
            .wrapping_add(HEADER_LEAD_BYTES.wrapping_mul(tpb))
            .wrapping_add(tpb / ISR_SLACK_DIV);
        self.state = State::AwaitHeader {
            anchor,
            anchor_tick: now,
            rechecks: 0,
            due,
        };
        FramerOut::Wait(due)
    }

    fn recheck_header(
        &mut self,
        anchor: u16,
        anchor_tick: u32,
        rechecks: u8,
        now: u32,
        tpb: u32,
    ) -> FramerOut {
        if rechecks >= HEADER_RECHECKS {
            self.drops = self.drops.wrapping_add(1);
            self.state = State::Hunt;
            FramerOut::None
        } else {
            let due = now.wrapping_add(tpb);
            self.state = State::AwaitHeader {
                anchor,
                anchor_tick,
                rechecks: rechecks + 1,
                due,
            };
            FramerOut::Wait(due)
        }
    }

    fn lock_end(
        &mut self,
        ring: &[u8],
        anchor: u16,
        anchor_tick: u32,
        received: u16,
        tpb: u32,
    ) -> FramerOut {
        let len = ring.len();
        let mut hb = [0u8; Header::SIZE];
        for (i, cell) in hb.iter_mut().enumerate() {
            *cell = ring[(anchor as usize + i) % len];
        }
        let h = Header::from_bytes(&hb);
        // Unaddressable or malformed: drop now, let the bytes ring out
        // silently, and let the next break re-anchor (FE fires in LOCKED too,
        // §3.1). Anchor parity is irrelevant — the CRC feed self-aligns (§3.2).
        if h.validate().is_err() {
            self.drops = self.drops.wrapping_add(1);
            self.state = State::Hunt;
            return FramerOut::None;
        }
        let footprint = h.frame_end() as u16;
        // Packet-end estimate from the anchor tick: the FE entry marks the
        // break byte's wire end (F2/F5), so `footprint - 1` byte-times
        // remain, plus the drift adder (~1.6%, §9.3) so an in-spec-slow
        // transmitter can never make the estimate early — T_turn is
        // measured from this (§7), and early would shave the wire gap.
        let frame_ticks = (footprint as u32 - 1).wrapping_mul(tpb);
        let packet_end = anchor_tick
            .wrapping_add(frame_ticks)
            .wrapping_add((footprint as u32).wrapping_mul(tpb) >> DRIFT_SHIFT);
        let end_due = packet_end.wrapping_add(self.end_slack);
        // The covered-complete checkpoint leads the end by the CRC tail (same
        // margin, two byte-times earlier). Short frames (a ping's covered span
        // IS its header) may have the whole covered span ringed already at
        // header lock — emit Covered from this same wake instead of arming an
        // aim that is already in the past.
        if received >= footprint - COVERED_TAIL_BYTES as u16 {
            self.state = State::AwaitEnd {
                anchor,
                footprint,
                packet_end,
                end_due,
                rechecks: 0,
                covered_seen: true,
                due: end_due,
            };
            return FramerOut::Covered {
                span: FrameSpan {
                    anchor,
                    footprint,
                    packet_end,
                },
                end_due,
            };
        }
        let due = end_due.wrapping_sub(COVERED_TAIL_BYTES.wrapping_mul(tpb));
        self.state = State::AwaitEnd {
            anchor,
            footprint,
            packet_end,
            end_due,
            rechecks: 0,
            covered_seen: false,
            due,
        };
        FramerOut::Wait(due)
    }
}

/// Ring index of the FE byte, one slot behind the cursor.
#[inline]
fn fe_index(ring: &[u8], cursor: u16) -> Option<u16> {
    let len = ring.len();
    if len == 0 {
        return None;
    }
    Some(((cursor as usize + len - 1) % len) as u16)
}

#[inline]
fn fe_byte(ring: &[u8], cursor: u16) -> Option<u8> {
    fe_index(ring, cursor).map(|i| ring[i as usize])
}

/// Wrap-distance from `anchor` to `cursor` (bytes ringed since the anchor,
/// inclusive). The ring is far larger than one frame, so this cannot alias
/// within a single frame's wait window (§4.1).
#[inline]
fn dist(cursor: u16, anchor: u16, len: usize) -> u16 {
    let len = len as u32;
    ((cursor as u32 + len - anchor as u32) % len) as u16
}

#[cfg(test)]
mod tests {
    /// Fixed end-slack for tests (ticks).
    const TEST_SLACK: u32 = 240;

    use super::*;

    const TPB: u32 = 30;

    // PING id 1: brk, id, len=3 (odd/valid), inst=0x10 (opcode Ping). footprint 6.
    const PING: [u8; 6] = [0x00, 0x01, 0x03, 0x10, 0xAA, 0xBB];

    fn place(ring: &mut [u8], at: usize, bytes: &[u8]) {
        let len = ring.len();
        for (i, &b) in bytes.iter().enumerate() {
            ring[(at + i) % len] = b;
        }
    }

    fn wait_tick(out: FramerOut) -> u32 {
        match out {
            FramerOut::Wait(t) => t,
            _ => panic!("expected Wait"),
        }
    }

    fn frame_span(out: FramerOut) -> FrameSpan {
        match out {
            FramerOut::Frame(s) => s,
            _ => panic!("expected Frame"),
        }
    }

    fn covered(out: FramerOut) -> (FrameSpan, u32) {
        match out {
            FramerOut::Covered { span, end_due } => (span, end_due),
            _ => panic!("expected Covered"),
        }
    }

    /// Header lock arms the covered aim; drive to the covered checkpoint.
    fn wait_then_covered(
        f: &mut Framer,
        ring: &[u8],
        k: usize,
        footprint: u16,
        tpb: u32,
    ) -> (FrameSpan, u32) {
        let cov_due = wait_tick(f.on_deadline(ring, (k + 4) as u16, 200, tpb));
        covered(f.on_deadline(ring, (k + footprint as usize - 2) as u16, cov_due, tpb))
    }

    #[test]
    fn happy_ping() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize; // even anchor
        place(&mut ring, k, &PING);
        let mut f = Framer::new(TEST_SLACK);

        let out = f.on_break(&ring, (k + 1) as u16, 100, TPB);
        // deadline A = anchor + 3 byte-times + half-byte slack.
        assert_eq!(wait_tick(out), 100 + 3 * TPB + TPB / 2);

        // A ping's covered span IS its header: deadline A locks the header and
        // emits Covered from the same wake (no separate checkpoint wake).
        let (span, end_due) = covered(f.on_deadline(&ring, (k + 4) as u16, 200, TPB));
        assert_eq!(span.anchor, k as u16);
        assert_eq!(span.footprint, 6);
        assert_eq!(end_due, 100 + 5 * TPB + ((6 * TPB) >> 6) + TEST_SLACK);

        // Deadline B then verifies the whole frame.
        let span = frame_span(f.on_deadline(&ring, (k + 6) as u16, end_due, TPB));
        assert_eq!(span.anchor, k as u16);
        assert_eq!(span.footprint, 6);
        assert_eq!(f.drops(), 0);
    }

    #[test]
    fn frame_wraps_ring_boundary() {
        let mut ring = [0xFFu8; 32];
        let k = 30usize; // spans 30,31,0,1,2,3
        place(&mut ring, k, &PING);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, ((k + 1) % 32) as u16, 100, TPB);
        let (_, end_due) = covered(f.on_deadline(&ring, ((k + 4) % 32) as u16, 200, TPB));
        let span = frame_span(f.on_deadline(&ring, ((k + 6) % 32) as u16, end_due, TPB));
        assert_eq!(span.anchor, 30);
        assert_eq!(span.footprint, 6);
    }

    #[test]
    fn deadline_b_short_then_completes() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        place(&mut ring, k, &PING);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        // Covered fires at header lock (received = footprint - 2).
        let (_, end_due) = covered(f.on_deadline(&ring, (k + 4) as u16, 200, TPB));
        // End deadline fires one byte short (drift): recheck armed one byte out.
        let out = f.on_deadline(&ring, (k + 5) as u16, end_due, TPB);
        assert_eq!(wait_tick(out), end_due + TPB);
        // Now complete.
        let span = frame_span(f.on_deadline(&ring, (k + 6) as u16, end_due + TPB, TPB));
        assert_eq!(span.footprint, 6);
        assert_eq!(f.drops(), 0);
    }

    #[test]
    fn covered_short_then_completes() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        // LEN 5 → footprint 8, covered target 6: the covered span outruns the
        // header, so header lock arms the aim instead of falling through.
        let frame = [0x00, 0x01, 0x05, 0x10, 0xAA, 0xBB, 0xCC, 0xDD];
        place(&mut ring, k, &frame);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        let cov_due = wait_tick(f.on_deadline(&ring, (k + 4) as u16, 200, TPB));
        // Covered deadline fires early (drift): only 5 of the 6 covered bytes
        // in. Recheck one byte-time out.
        let out = f.on_deadline(&ring, (k + 5) as u16, cov_due, TPB);
        assert_eq!(wait_tick(out), cov_due + TPB);
        // Covered span now complete → Covered fires exactly once.
        let (span, _) = covered(f.on_deadline(&ring, (k + 6) as u16, cov_due + TPB, TPB));
        assert_eq!(span.footprint, 8);
        assert_eq!(f.drops(), 0);
    }

    #[test]
    fn covered_fires_at_most_once() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        place(&mut ring, k, &PING);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        let (_, end_due) = covered(f.on_deadline(&ring, (k + 4) as u16, 200, TPB));
        // A second wake at the end yields the Frame, never another Covered.
        let out = f.on_deadline(&ring, (k + 6) as u16, end_due, TPB);
        assert!(matches!(out, FramerOut::Frame(_)));
    }

    #[test]
    fn header_starvation_exhausts_rechecks() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        // Only the break byte ever arrives (nothing after the anchor).
        ring[k] = 0x00;
        let mut f = Framer::new(TEST_SLACK);
        f.on_break(&ring, (k + 1) as u16, 100, TPB);

        // received stays 1 (< 4): each deadline rechecks until the bound.
        for i in 0..HEADER_RECHECKS {
            let out = f.on_deadline(&ring, (k + 1) as u16, 200 + i as u32, TPB);
            assert!(matches!(out, FramerOut::Wait(_)));
        }
        let out = f.on_deadline(&ring, (k + 1) as u16, 999, TPB);
        assert!(matches!(out, FramerOut::None));
        assert_eq!(f.drops(), 1);
    }

    #[test]
    fn odd_anchor_frames_normally() {
        // Anchor parity is irrelevant (§3.2 self-aligning feed): an odd-anchored
        // frame locks, covers, and completes like any other.
        let mut ring = [0xFFu8; 32];
        let k = 7usize;
        place(&mut ring, k, &PING);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        let (span, end_due) = covered(f.on_deadline(&ring, (k + 4) as u16, 200, TPB));
        assert_eq!(span.anchor, k as u16);
        let span = frame_span(f.on_deadline(&ring, (k + 6) as u16, end_due, TPB));
        assert_eq!(span.footprint, 6);
        assert_eq!(f.drops(), 0);
    }

    #[test]
    fn even_len_frames_normally() {
        // Even LEN is legal (§3.1): LEN 6 → 3-byte payload, footprint 9.
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        let frame = [0x00, 0x01, 0x06, 0x10, 0xAA, 0xBB, 0xCC, 0, 0];
        place(&mut ring, k, &frame);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        let (span, end_due) = wait_then_covered(&mut f, &ring, k, 9, TPB);
        assert_eq!(span.footprint, 9);
        let span = frame_span(f.on_deadline(&ring, (k + 9) as u16, end_due, TPB));
        assert_eq!(span.footprint, 9);
        assert_eq!(f.drops(), 0);
    }

    #[test]
    fn bad_id_drops_immediately() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        // ID 0x00 is never addressable.
        let frame = [0x00, 0x00, 0x03, 0x10, 0xAA, 0xBB];
        place(&mut ring, k, &frame);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        let out = f.on_deadline(&ring, (k + 4) as u16, 200, TPB);
        assert!(matches!(out, FramerOut::None));
        assert_eq!(f.drops(), 1);
    }

    #[test]
    fn midframe_garble_preserves_deadline() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        place(&mut ring, k, &PING);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        let (_, end_due) = covered(f.on_deadline(&ring, (k + 4) as u16, 200, TPB));
        // FE fires mid-frame on a nonzero data byte (ring[k+4] = 0xAA).
        let out = f.on_break(&ring, (k + 5) as u16, 250, TPB);
        assert_eq!(wait_tick(out), end_due); // pending deadline untouched
        assert_eq!(f.drops(), 0);
        // Frame still completes at the end.
        let span = frame_span(f.on_deadline(&ring, (k + 6) as u16, end_due, TPB));
        assert_eq!(span.footprint, 6);
    }

    #[test]
    fn midframe_break_reanchors_and_drops() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        place(&mut ring, k, &PING);
        let mut f = Framer::new(TEST_SLACK);

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        f.on_deadline(&ring, (k + 4) as u16, 200, TPB);
        // A fresh break arrives (0x00) at a new anchor m before the old frame ends.
        let m = 20usize;
        ring[m] = 0x00;
        let out = f.on_break(&ring, (m + 1) as u16, 300, TPB);
        // Re-anchored: new deadline A at the new break.
        assert_eq!(wait_tick(out), 300 + 3 * TPB + TPB / 2);
        assert_eq!(f.drops(), 1); // old frame counted lost
    }

    #[test]
    fn back_to_back_frames() {
        let mut ring = [0xFFu8; 64];
        let mut f = Framer::new(TEST_SLACK);

        let k0 = 6usize;
        place(&mut ring, k0, &PING);
        f.on_break(&ring, (k0 + 1) as u16, 100, TPB);
        let (_, e0) = covered(f.on_deadline(&ring, (k0 + 4) as u16, 200, TPB));
        let s0 = frame_span(f.on_deadline(&ring, (k0 + 6) as u16, e0, TPB));
        assert_eq!(s0.anchor, k0 as u16);

        // Second break immediately after the first frame.
        let k1 = k0 + 6; // 12, even
        place(&mut ring, k1, &PING);
        f.on_break(&ring, (k1 + 1) as u16, 400, TPB);
        let (_, e1) = covered(f.on_deadline(&ring, (k1 + 4) as u16, 500, TPB));
        let s1 = frame_span(f.on_deadline(&ring, (k1 + 6) as u16, e1, TPB));
        assert_eq!(s1.anchor, k1 as u16);
        assert_eq!(f.drops(), 0);
    }
}
