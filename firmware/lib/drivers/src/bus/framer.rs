//! Two-state break-framed RX framer (`docs/osc-native-protocol.md` §4.1).
//!
//! Pure state machine: the composite owns the ring, deadline, and CRC
//! providers and drives this with plain values. Every legal frame fits whole
//! in the ring by construction (258 B max vs 512 B ring, §3.1), so there is no
//! chunked consumption — HUNT for a break, then two computed deadlines verify
//! the header and the frame end against the ring cursor.

use osc_protocol::frame::{FrameError, Header};

/// A verified frame's location in the ring: anchor index + ring-byte count.
pub struct FrameSpan {
    pub anchor: u16,
    pub footprint: u16,
}

/// What the framer wants from the composite after an event.
pub enum FramerOut {
    /// Idle; no framer deadline wanted.
    None,
    /// Arm/refresh the framer deadline at this absolute tick.
    Wait(u32),
    /// A full frame is verified in the ring (header pre-validated); back in HUNT.
    Frame(FrameSpan),
    /// §3.2 parity recovery: caller must `ring.rearm()` now; framer is back in
    /// HUNT expecting the next break's ring byte at index 0.
    Rearm,
}

/// Why a locked frame is doomed to drop at its computed end. Both faults leave
/// the ring at odd parity, so both need the boundary `Rearm` (§3.2).
#[derive(Copy, Clone)]
enum Doom {
    /// Header anchor landed on an odd ring index (§3.2 layer-1 fault).
    OddAnchor,
    /// `LEN` even — malformed; its odd footprint would flip ring parity (§3.1).
    EvenLen,
}

enum State {
    Hunt,
    AwaitHeader {
        anchor: u16,
        rechecks: u8,
        due: u32,
    },
    AwaitEnd {
        anchor: u16,
        footprint: u16,
        rechecks: u8,
        doom: Option<Doom>,
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

// Deadline B slack: two byte-times of ISR-entry margin plus
// `footprint >> DRIFT_SHIFT` (~1.6%) for worst-case untrimmed-HSI drift (§9.3).
// The margin bounds recheck count only, not correctness — every wake is
// verified against the ring cursor before the framer acts.
const END_SLACK_BYTES: u32 = 2;
const DRIFT_SHIFT: u32 = 6;

// Plateau backstops (§4.1): bounded rechecks one byte-time apart past the
// computed deadline before abandoning a stalled header/frame (host died
// mid-frame; the next break re-anchors and recovers).
const HEADER_RECHECKS: u8 = 8;
const END_RECHECKS: u8 = 8;

pub struct Framer {
    state: State,
    drops: u32,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    pub const fn new() -> Self {
        Self {
            state: State::Hunt,
            drops: 0,
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
                anchor, rechecks, ..
            } => {
                let received = dist(cursor, anchor, len);
                if received < HEADER_SPAN_BYTES {
                    self.recheck_header(anchor, rechecks, now, tpb)
                } else {
                    self.lock_end(ring, anchor, received, now, tpb)
                }
            }
            State::AwaitEnd {
                anchor,
                footprint,
                rechecks,
                doom,
                ..
            } => {
                let received = dist(cursor, anchor, len);
                if received >= footprint {
                    self.state = State::Hunt;
                    match doom {
                        // §4.1: the one sanctioned reload, at the computed boundary.
                        Some(_) => {
                            self.drops = self.drops.wrapping_add(1);
                            FramerOut::Rearm
                        }
                        Option::None => FramerOut::Frame(FrameSpan { anchor, footprint }),
                    }
                } else if rechecks >= END_RECHECKS {
                    // Host died mid-frame; the next break recovers.
                    self.drops = self.drops.wrapping_add(1);
                    self.state = State::Hunt;
                    FramerOut::None
                } else {
                    let due = now.wrapping_add(tpb);
                    self.state = State::AwaitEnd {
                        anchor,
                        footprint,
                        rechecks: rechecks + 1,
                        doom,
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
            rechecks: 0,
            due,
        };
        FramerOut::Wait(due)
    }

    fn recheck_header(&mut self, anchor: u16, rechecks: u8, now: u32, tpb: u32) -> FramerOut {
        if rechecks >= HEADER_RECHECKS {
            self.drops = self.drops.wrapping_add(1);
            self.state = State::Hunt;
            FramerOut::None
        } else {
            let due = now.wrapping_add(tpb);
            self.state = State::AwaitHeader {
                anchor,
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
        received: u16,
        now: u32,
        tpb: u32,
    ) -> FramerOut {
        let len = ring.len();
        let mut hb = [0u8; Header::SIZE];
        for (i, cell) in hb.iter_mut().enumerate() {
            *cell = ring[(anchor as usize + i) % len];
        }
        let h = Header::from_bytes(&hb);
        let doom = match h.validate() {
            // Even LEN flips ring parity at the frame end → rearm there (§3.2).
            Err(FrameError::EvenLen) => Some(Doom::EvenLen),
            // Well-formed length but unaddressable: drop now, let the bytes ring
            // out silently, and let the next break re-anchor (FE fires in LOCKED
            // too, §3.1).
            Err(FrameError::BadId | FrameError::BadOpcode) => {
                self.drops = self.drops.wrapping_add(1);
                self.state = State::Hunt;
                return FramerOut::None;
            }
            // §3.2: an odd anchor is a layer-1 fault; the end is still computable.
            Ok(()) => {
                if anchor & 1 == 1 {
                    Some(Doom::OddAnchor)
                } else {
                    Option::None
                }
            }
        };
        let footprint = h.frame_end() as u16;
        // Back-to-back traffic can already have pushed `received` past the end;
        // saturate so the deadline just fires promptly.
        let remaining = footprint.saturating_sub(received) as u32;
        let margin = END_SLACK_BYTES
            .wrapping_mul(tpb)
            .wrapping_add((footprint as u32).wrapping_mul(tpb) >> DRIFT_SHIFT);
        let due = now
            .wrapping_add(remaining.wrapping_mul(tpb))
            .wrapping_add(margin);
        self.state = State::AwaitEnd {
            anchor,
            footprint,
            rechecks: 0,
            doom,
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

    #[test]
    fn happy_ping() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize; // even anchor
        place(&mut ring, k, &PING);
        let mut f = Framer::new();

        let out = f.on_break(&ring, (k + 1) as u16, 100, TPB);
        // deadline A = anchor + 3 byte-times + half-byte slack.
        assert_eq!(wait_tick(out), 100 + 3 * TPB + TPB / 2);

        let out = f.on_deadline(&ring, (k + 4) as u16, 200, TPB);
        assert!(matches!(out, FramerOut::Wait(_)));

        let span = frame_span(f.on_deadline(&ring, (k + 6) as u16, 400, TPB));
        assert_eq!(span.anchor, k as u16);
        assert_eq!(span.footprint, 6);
        assert_eq!(f.drops(), 0);
    }

    #[test]
    fn frame_wraps_ring_boundary() {
        let mut ring = [0xFFu8; 32];
        let k = 30usize; // spans 30,31,0,1,2,3
        place(&mut ring, k, &PING);
        let mut f = Framer::new();

        f.on_break(&ring, ((k + 1) % 32) as u16, 100, TPB);
        f.on_deadline(&ring, ((k + 4) % 32) as u16, 200, TPB);
        let span = frame_span(f.on_deadline(&ring, ((k + 6) % 32) as u16, 400, TPB));
        assert_eq!(span.anchor, 30);
        assert_eq!(span.footprint, 6);
    }

    #[test]
    fn deadline_b_short_then_completes() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        place(&mut ring, k, &PING);
        let mut f = Framer::new();

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        f.on_deadline(&ring, (k + 4) as u16, 200, TPB);
        // One byte short (drift): recheck armed one byte-time out.
        let out = f.on_deadline(&ring, (k + 5) as u16, 400, TPB);
        assert_eq!(wait_tick(out), 400 + TPB);
        // Now complete.
        let span = frame_span(f.on_deadline(&ring, (k + 6) as u16, 430, TPB));
        assert_eq!(span.footprint, 6);
        assert_eq!(f.drops(), 0);
    }

    #[test]
    fn header_starvation_exhausts_rechecks() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        // Only the break byte ever arrives (nothing after the anchor).
        ring[k] = 0x00;
        let mut f = Framer::new();
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
    fn odd_anchor_rearms_at_end() {
        let mut ring = [0xFFu8; 32];
        let k = 7usize; // odd anchor, otherwise-valid PING
        place(&mut ring, k, &PING);
        let mut f = Framer::new();

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        f.on_deadline(&ring, (k + 4) as u16, 200, TPB);
        let out = f.on_deadline(&ring, (k + 6) as u16, 400, TPB);
        assert!(matches!(out, FramerOut::Rearm));
        assert_eq!(f.drops(), 1);
    }

    #[test]
    fn even_len_rearms_at_end() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        // LEN 4 is even → malformed; footprint = 3 + 4 = 7.
        let frame = [0x00, 0x01, 0x04, 0x10, 0, 0, 0];
        place(&mut ring, k, &frame);
        let mut f = Framer::new();

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        f.on_deadline(&ring, (k + 4) as u16, 200, TPB);
        let out = f.on_deadline(&ring, (k + 7) as u16, 400, TPB);
        assert!(matches!(out, FramerOut::Rearm));
        assert_eq!(f.drops(), 1);
    }

    #[test]
    fn bad_id_drops_immediately() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        // ID 0x00 is never addressable.
        let frame = [0x00, 0x00, 0x03, 0x10, 0xAA, 0xBB];
        place(&mut ring, k, &frame);
        let mut f = Framer::new();

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
        let mut f = Framer::new();

        f.on_break(&ring, (k + 1) as u16, 100, TPB);
        let due_b = wait_tick(f.on_deadline(&ring, (k + 4) as u16, 200, TPB));
        // FE fires mid-frame on a nonzero data byte (ring[k+4] = 0xAA).
        let out = f.on_break(&ring, (k + 5) as u16, 250, TPB);
        assert_eq!(wait_tick(out), due_b); // deadline B untouched
        assert_eq!(f.drops(), 0);
        // Frame still completes.
        let span = frame_span(f.on_deadline(&ring, (k + 6) as u16, due_b, TPB));
        assert_eq!(span.footprint, 6);
    }

    #[test]
    fn midframe_break_reanchors_and_drops() {
        let mut ring = [0xFFu8; 32];
        let k = 6usize;
        place(&mut ring, k, &PING);
        let mut f = Framer::new();

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
        let mut f = Framer::new();

        let k0 = 6usize;
        place(&mut ring, k0, &PING);
        f.on_break(&ring, (k0 + 1) as u16, 100, TPB);
        f.on_deadline(&ring, (k0 + 4) as u16, 200, TPB);
        let s0 = frame_span(f.on_deadline(&ring, (k0 + 6) as u16, 300, TPB));
        assert_eq!(s0.anchor, k0 as u16);

        // Second break immediately after the first frame.
        let k1 = k0 + 6; // 12, even
        place(&mut ring, k1, &PING);
        f.on_break(&ring, (k1 + 1) as u16, 400, TPB);
        f.on_deadline(&ring, (k1 + 4) as u16, 500, TPB);
        let s1 = frame_span(f.on_deadline(&ring, (k1 + 6) as u16, 600, TPB));
        assert_eq!(s1.anchor, k1 as u16);
        assert_eq!(f.drops(), 0);
    }
}
