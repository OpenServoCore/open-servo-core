//! Stream-continuity RX resolver (`docs/osc-servo-transport.md` §6 A2,
//! `docs/osc-native-protocol.md` §4.1).
//!
//! Pure state machine: the composite owns the ring, deadline, and CRC
//! providers and drives this with plain values. Anchors are DERIVED, never
//! sampled: frames are contiguous, so `next anchor = anchor + footprint`,
//! bootstrapped only at provably-quiet moments (boot = index 0, rescue = a
//! still cursor). Every legal frame fits whole in the ring (258 B max vs
//! 512 B, §3.1), so resolution is a pure function of ring content — the
//! clock only schedules *when to look again* for the one frame still
//! arriving (the frontier). A frame with bytes beyond its end is complete
//! by construction and resolves with zero clock involvement, however late
//! or coalesced the FE wakes were (A2: the ring is the queue).
//!
//! TIME is derived the way position is (A2 extended to the clock): every
//! aim and estimate is `now + missing·tpb` — the bytes still owed, at wire
//! pace. An FE carries neither position nor time; it is only a wake. ISR
//! delivery lag cancels out of every projection (`now` and the cursor
//! advance together), bounding the error to under one byte-time, always on
//! the late side: a byte counted as missing can only finish sooner than
//! assumed, so the reply grid measured from these estimates never fires
//! into a frame's own tail.

use osc_protocol::frame::Header;

use super::ring_wrap;
use crate::traits::bus::tick_reached;

/// A resolved frame's location in the ring: anchor index + ring-byte count,
/// plus the wire-end estimate (`packet_end`) the reply trigger's reply gap is
/// measured from (§7). Frontier frames project it from ring cadence at the
/// covered checkpoint (late by less than a byte-time, never early); backlog
/// frames ended in the past and carry `now` (elastic best-effort timing).
pub struct FrameSpan {
    pub anchor: u16,
    pub footprint: u16,
    pub packet_end: u32,
    /// Byte-cadence sample from this frame's arrival, when one was
    /// measurable (frontier frames only; the composite CRC-gates it).
    pub cadence: Option<CadenceSample>,
}

/// The local clock measured against one frame's byte cadence (§9.3): between
/// two progress observations of the same frontier — both strictly inside the
/// frame's arrival window by construction, so no inter-frame gap can pollute
/// the span — the wire delivered `n` bytes while the local clock counted
/// `measured` ticks. `err_ticks = measured − n·tpb`: positive means the local
/// clock ran fast against the transmitter's byte clock.
pub struct CadenceSample {
    pub err_ticks: i32,
    /// The nominal span (`n·tpb`) the error is relative to — the sample's
    /// weight when windows accumulate across frames and baud changes.
    pub span_ticks: u32,
}

/// What the resolver wants from the composite after a step. The composite
/// loops on [`Framer::resolve`] until it yields `None` or `Wait`.
pub enum FramerOut {
    /// Nothing resolvable; no framer deadline wanted.
    None,
    /// Arm/refresh the framer deadline at this absolute tick (frontier only).
    Wait(u32),
    /// The frontier frame's CRC-covered span is fully ringed — only the two
    /// wire-CRC bytes are still inbound — so the composite may front-load the
    /// CRC feed + dispatch now. The frame-end deadline is already armed at
    /// `end_due`; the matching [`FramerOut::Frame`] follows there. Emitted at
    /// most once per frame, never for backlog frames (they are complete —
    /// there is nothing left to overlap with).
    Covered { span: FrameSpan, end_due: u32 },
    /// A whole frame is in the ring (geometry pre-validated); the ladder has
    /// advanced past it. Resolve again for the next backlog frame.
    Frame(FrameSpan),
}

// §4.1: header = BREAK, ID, LEN, INST; the break byte is index 0 of the
// frame, so the header is readable `HEADER_SPAN_BYTES` in.
const HEADER_SPAN_BYTES: u16 = Header::SIZE as u16;

// §3.2/A2: a real break rings an all-zero byte; the ladder expects it at the
// derived anchor. Anything else there is desync (noise inserted/lost bytes).
const BREAK_RING_BYTE: u8 = 0x00;

// Header-aim epsilon (half a byte-time): the break rings at its FE point,
// ~4 bit-times before the line rises [F5], so the first data byte starts
// that far outside the byte cadence — header aims bridge the tail or they
// land one self-paced re-wake early. Data-anchored aims (covered, end)
// need no epsilon: `now` is at-or-past the last completion, so a cadence
// projection is never early, and the drift pad covers the rest.
const BREAK_TAIL_EPS_DIV: u32 = 2;

// Wire-proportional drift pad on every projection (~1.6%, §9.3): an
// in-spec-slow transmitter can never make a projection early.
const DRIFT_SHIFT: u32 = 6;

// The wire CRC tail: the covered span completes this many byte-times before
// the frame end, so the covered checkpoint leads the frame end by it.
const COVERED_TAIL_BYTES: u32 = 2;

// Dead-transmitter detector: a frontier whose ring made no progress for this
// many byte-times is a sacrificed partial (host died, or a junk header's
// footprint trapped the ladder) — the hunt resumes one byte in (§3.3).
// Generous by design: a live host's feeder can stall mid-frame (bench
// 2026-07-08: pirate walker bubbles pause the wire ~100 µs mid-write and an
// 8-byte horizon sacrificed live partials), waiting is free under break
// framing (position, not the clock, is the truth — A2), and the real bounds
// are the host's retry timeout (~ms) and ring pressure (512 byte-times).
const STARVE_GIVEUP_BYTES: u32 = 64;

// Hunt scan bound per resolve call, so a desynced wake stays bounded; the
// composite's drive loop re-enters via pend-on-past.
const HUNT_BYTES_PER_WAKE: u16 = 64;

// Cadence-sample floor: progress stamps are quantized by aim overshoot and
// ISR entry latency (~a byte-time or two of endpoint noise), so a short span
// is mostly noise — and would trip the reject gate below on ordinary jitter.
const CADENCE_MIN_BYTES: u16 = 32;

// Cadence reject gate, as a right-shift of the nominal span (1/16 ≈ 6%):
// legal clock offset is bounded by the HSITRIM throw (±3.4%), so anything
// past the gate is a host TX bubble, merged-transmitter garble, or a gross
// wake-latency outlier — not a clock reading.
const CADENCE_REJECT_SHIFT: u32 = 4;

/// Wire time for `bytes` more bytes at `tpb`, drift-padded — the one
/// projection formula behind every aim and estimate.
#[inline]
fn wire_lead(bytes: u32, tpb: u32) -> u32 {
    let t = bytes.wrapping_mul(tpb);
    t.wrapping_add(t >> DRIFT_SHIFT)
}

/// Frontier bookkeeping — schedule hints only; the ring stays the truth.
struct Frontier {
    /// Covered checkpoint emitted for the frame at the ladder anchor.
    covered: bool,
    /// Giveup baseline taken (first observation of this frontier).
    observed: bool,
    /// Ring progress watermark + its tick: the dead-transmitter detector.
    seen: u16,
    progress_tick: u32,
    /// First progress observation, frozen: the cadence span's near endpoint
    /// (`seen`/`progress_tick` keep advancing and become the far one).
    first_seen: u16,
    first_tick: u32,
}

impl Frontier {
    const fn idle() -> Self {
        Self {
            covered: false,
            observed: false,
            seen: 0,
            progress_tick: 0,
            first_seen: 0,
            first_tick: 0,
        }
    }

    /// The completed frontier's cadence sample, gated at the source: enough
    /// span for the endpoint noise to average out, and within the reject
    /// band a real clock offset can occupy.
    ///
    /// The endpoints are phase-asymmetric by construction: the near stamp is
    /// break-anchored (the FE fires at the break byte, sub-byte-exact) while
    /// the far stamp is a wake-quantized progress observation — its sub-byte
    /// phase lies anywhere in [0, tpb). The hop dither sweeps that phase
    /// uniformly across frames, so its window mean is tpb/2 — subtracted
    /// here, which is what keeps repetitive traffic (identical hot-loop
    /// frames, identical wake geometry) from freezing one phase into a
    /// permanent bias (sim-measured +3.1k ppm on a +5.2k signal, 2026-07-11).
    fn cadence(&self, tpb: u32, len: usize) -> Option<CadenceSample> {
        if !self.observed || self.seen == self.first_seen {
            return None;
        }
        let bytes = dist(self.seen, self.first_seen, len);
        let span_ticks = (bytes as u32).wrapping_mul(tpb);
        let err_ticks = self
            .progress_tick
            .wrapping_sub(self.first_tick)
            .wrapping_sub(span_ticks)
            .wrapping_sub(tpb / 2) as i32;
        (bytes >= CADENCE_MIN_BYTES
            && err_ticks.unsigned_abs() <= span_ticks >> CADENCE_REJECT_SHIFT)
            .then_some(CadenceSample {
                err_ticks,
                span_ticks,
            })
    }
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Framer {
    /// The ladder (A2): ring index where the current frame's break byte sits.
    anchor: u16,
    /// A recovery episode is in progress (desync/reject/giveup): drops count
    /// once per episode, and probe candidates that fail CRC are not wire
    /// faults. Cleared when a candidate resolves as a whole valid frame.
    hunting: bool,
    frontier: Frontier,
    /// Cursor at the last wire-fault service ([`Self::on_wire_fault`]).
    fault_seen: u16,
    drops: u32,
    /// Weyl sequence (odd step, visits all 8 phases whatever the per-frame
    /// hop count) behind the interior-hop dither — see [`Frontier::cadence`].
    dither: u8,
}

impl Framer {
    pub const fn new() -> Self {
        Self {
            anchor: 0,
            hunting: false,
            frontier: Frontier::idle(),
            fault_seen: 0,
            drops: 0,
            dither: 0,
        }
    }

    /// Frames dropped at transport layer 1 (§5.3), monotonic.
    pub fn drops(&self) -> u32 {
        self.drops
    }

    /// Re-sync the ladder to a known ring position — bootstrap only (boot is
    /// position 0 by construction; rescue reads a provably-still cursor).
    pub fn resync(&mut self, cursor: u16) {
        self.anchor = cursor;
        self.frontier = Frontier::idle();
        self.fault_seen = cursor;
    }

    /// USART FE/RX-error service — a pure wake, in the strong sense (the
    /// fault contract, osc-native §fault-handling): the event carries no
    /// position, no time, and no identity, and NOTHING may be derived from
    /// it beyond "look at the ring again". Latched flags re-fire and
    /// service lags behind the wire, so any cursor sampled here describes
    /// the service, not the event — a fence built on it killed live frames
    /// under burst load (bench 2026-07-11: hot chains 95%→80%). The one
    /// thing this service computes is freshness: `false` means no bytes
    /// ringed since the last service — the wake's evidence (if any) is
    /// still in flight, and the composite arms a one-byte-time recheck.
    pub fn on_wire_fault(&mut self, cursor: u16) -> bool {
        if cursor == self.fault_seen {
            return false;
        }
        self.fault_seen = cursor;
        true
    }

    /// The composite rejected a resolved frame (CRC fail): the header that
    /// sized the stride was itself unverified, so the stride cannot be
    /// trusted. Resume the hunt one byte past the rejected anchor — a real
    /// frame hiding inside the rejected span (e.g. behind noise) is still
    /// found and CRC-gated (§3.3).
    pub fn on_frame_rejected(&mut self, anchor: u16, ring_len: usize) {
        // The fault is already counted (`crc_fail`); the hunt is recovery.
        self.hunting = true;
        self.anchor = ring_wrap(anchor as usize + 1, ring_len) as u16;
        self.frontier = Frontier::idle();
    }

    /// The composite verified a resolved frame: the ladder is trusted again;
    /// any recovery episode ends here.
    pub fn on_frame_verified(&mut self) {
        self.hunting = false;
    }

    /// The ladder has consumed every ringed byte: nothing on the wire
    /// follows the last resolved frame. The composite's staged-reply kill
    /// keys off this (A2 positional truth): a reply is stale only if newer
    /// bytes follow its own frame.
    pub fn caught_up(&self, cursor: u16) -> bool {
        self.anchor == cursor
    }

    /// A rejected probe (a hunt candidate that failed CRC) is scan noise,
    /// not a received frame — the composite uses this to keep `crc_fail`
    /// meaning "a trusted frame failed", once per episode.
    pub fn probing(&self) -> bool {
        self.hunting
    }

    /// One resolution step against the CURRENT ring state (A2 data-first).
    /// The composite loops until `None`/`Wait`. `cursor` is the live DMA
    /// cursor read at call time — progress truth, not a stale event snapshot.
    pub fn resolve(&mut self, ring: &[u8], cursor: u16, now: u32, tpb: u32) -> FramerOut {
        let len = ring.len();
        if len == 0 {
            return FramerOut::None; // defensive: no ring to resolve against
        }
        let mut received = dist(cursor, self.anchor, len);
        if received == 0 {
            self.frontier = Frontier::idle();
            return FramerOut::None; // ladder caught up; wait for wire
        }
        // The ladder's first byte must be a break with plausible geometry
        // (LEN covers INST + CRC). Anything else means the stream lost or
        // gained bytes (noise, or a sacrificed partial): HUNT — scan forward
        // for the next plausible candidate; the CRC gates everything
        // downstream, so a wrong candidate costs nothing but the scan
        // (§3.3). Bounded per wake; counted once per episode.
        let mut budget = HUNT_BYTES_PER_WAKE;
        loop {
            let a = self.anchor as usize;
            let plausible = ring[a] == BREAK_RING_BYTE
                && (received < HEADER_SPAN_BYTES || ring[ring_wrap(a + 2, len)] >= 3);
            if plausible {
                break;
            }
            // Skipping noise sacrifices nothing: scan silently. Sacrificed
            // FRAMES are counted where they die (giveup / crc_fail).
            self.hunting = true;
            self.anchor = ring_wrap(a + 1, len) as u16;
            self.frontier = Frontier::idle();
            received -= 1;
            if received == 0 {
                return FramerOut::None; // scanned everything; next break lands here
            }
            budget -= 1;
            if budget == 0 {
                return FramerOut::Wait(now); // pend-on-past re-entry
            }
        }
        if received < HEADER_SPAN_BYTES {
            let aim = now
                .wrapping_add(wire_lead((HEADER_SPAN_BYTES - received) as u32, tpb))
                .wrapping_add(tpb / BREAK_TAIL_EPS_DIV);
            return self.wait(aim, now, tpb, cursor, len);
        }
        // Header readable: geometry pre-checked by the scan. The composite's
        // decoder owns addressing/opcode (a foreign frame still advances the
        // ladder; its bytes occupy the ring either way).
        let mut hb = [0u8; Header::SIZE];
        for (i, cell) in hb.iter_mut().enumerate() {
            *cell = ring[ring_wrap(self.anchor as usize + i, len)];
        }
        let h = Header::from_bytes(&hb);
        let footprint = h.frame_end() as u16;
        if received >= footprint {
            // Complete — the fast path: process now; the frame ended in the
            // past, so the estimate degenerates to elastic `now`. A tracked
            // frontier hands over its cadence sample here — its stamps, not
            // the current pair, so a gap behind the frame end can't leak in.
            let cadence = self.frontier.cadence(tpb, len);
            let anchor = self.anchor;
            self.anchor = ring_wrap(anchor as usize + footprint as usize, len) as u16;
            self.frontier = Frontier::idle();
            return FramerOut::Frame(FrameSpan {
                anchor,
                footprint,
                packet_end: now,
                cadence,
            });
        }
        let missing = (footprint - received) as u32;
        if !self.frontier.covered && missing <= COVERED_TAIL_BYTES {
            // Covered checkpoint: freeze the wire-end estimate here — the
            // tightest projection this frame gets (missing ≤ 2, so the
            // ring-cadence error is under a byte-time) — and hand the
            // composite the overlap window. This branch returns before
            // `wait`, so advance the progress watermark here too: it is the
            // cadence span's far endpoint, and `received < footprint` keeps
            // it strictly inside the frame's arrival window.
            let f = &mut self.frontier;
            if f.observed && cursor != f.seen {
                f.seen = cursor;
                f.progress_tick = now;
            }
            f.covered = true;
            let packet_end = now.wrapping_add(wire_lead(missing, tpb));
            let end_due = packet_end;
            return FramerOut::Covered {
                span: FrameSpan {
                    anchor: self.anchor,
                    footprint,
                    packet_end,
                    cadence: None,
                },
                end_due,
            };
        }
        // Aim at the next milestone: the covered point, or (once emitted)
        // the frame end. Data-anchored: pure cadence, no epsilon.
        let to_target = if self.frontier.covered {
            missing
        } else {
            missing - COVERED_TAIL_BYTES
        };
        let aim = now.wrapping_add(wire_lead(to_target, tpb));
        self.wait(aim, now, tpb, cursor, len)
    }

    /// Progress-track the frontier and wake at the sooner of the aim and the
    /// giveup horizon. Aims are fresh projections from live ring state, so a
    /// stalled wire self-paces at byte-time intervals with no recheck
    /// bookkeeping; the horizon is the sole death authority — a wire that
    /// makes NO progress for [`STARVE_GIVEUP_BYTES`] declares the partial
    /// dead regardless of the aim (a junk header's far-future footprint must
    /// not park the ladder).
    fn wait(&mut self, aim: u32, now: u32, tpb: u32, cursor: u16, len: usize) -> FramerOut {
        let giveup_span = STARVE_GIVEUP_BYTES.wrapping_mul(tpb);
        let f = &mut self.frontier;
        if !f.observed {
            f.observed = true;
            f.seen = cursor;
            f.progress_tick = now;
            f.first_seen = cursor;
            f.first_tick = now;
        } else if cursor != f.seen {
            f.seen = cursor;
            f.progress_tick = now;
        } else if tick_reached(now, f.progress_tick.wrapping_add(giveup_span)) {
            return self.give_up(len, now);
        }
        let horizon = self.frontier.progress_tick.wrapping_add(giveup_span);
        let sooner = if aim.wrapping_sub(now) <= horizon.wrapping_sub(now) {
            aim
        } else {
            // Interior hop: a long frame's wait is bounded by the horizon, so
            // re-checks land every STARVE_GIVEUP_BYTES — pure progress looks
            // with no timing contract (aims stay exact; giveup moves by under
            // a byte-time on a 64-byte horizon). Dither them across the byte
            // phase so the far cadence stamp sweeps [0, tpb) uniformly over
            // frames instead of freezing one phase ([`Frontier::cadence`]).
            self.dither = self.dither.wrapping_add(5);
            horizon.wrapping_add(((self.dither & 7) as u32 * tpb) >> 3)
        };
        FramerOut::Wait(sooner)
    }

    /// The candidate is dead — the starve horizon expired (transmitter died,
    /// or a junk header trapped the ladder): sacrifice one byte and resume
    /// the hunt — a real frame inside the span is still found, CRC-gated
    /// (§3.3). Data is the only death authority (the fault contract): a
    /// phantom born from garble dies by footprint-fill CRC or by starve,
    /// never by fault position.
    fn give_up(&mut self, len: usize, now: u32) -> FramerOut {
        // A partial that will never complete is sacrificed — count it, once
        // per recovery episode (junk candidates inside an episode also give
        // up, but they were never frames).
        if !self.hunting {
            self.drops = self.drops.wrapping_add(1);
            self.hunting = true;
        }
        self.anchor = ring_wrap(self.anchor as usize + 1, len) as u16;
        self.frontier = Frontier::idle();
        FramerOut::Wait(now) // pend-on-past re-entry: hunt continues
    }
}

/// Wrap-distance from `anchor` to `cursor` (bytes ringed since the anchor).
/// The ring is far larger than one frame, so this cannot alias within a
/// frame's window (§4.1).
#[inline]
fn dist(cursor: u16, anchor: u16, len: usize) -> u16 {
    ring_wrap((cursor as usize + len) - anchor as usize, len) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;

    /// Repetitive fat frames — the hostile case for the estimator: identical
    /// geometry freezes the far stamp's byte phase, and without the hop
    /// dither + tpb/2 centering the frozen phase reads as a permanent offset
    /// (+3.1k ppm on this exact scenario). The window mean must track the
    /// true skew to well under half a nominal trim step.
    #[test]
    fn repetitive_fat_frames_measure_the_true_skew() {
        let tpb: u32 = 480;
        let skew = 1.0052_f64; // +5200 ppm local clock
        let mut ring = vec![0u8; 512];
        let footprint = 206u16;
        let mut f = Framer::new();
        let mut err_sum = 0i64;
        let mut span_sum = 0u64;
        let mut samples = 0;
        for frame in 0..16usize {
            let base = (frame * footprint as usize) % 512;
            ring[base] = 0;
            ring[(base + 1) % 512] = 6;
            ring[(base + 2) % 512] = 203;
            ring[(base + 3) % 512] = 0x21;
            let t0 = frame as f64 * 300_000.0; // sim tick of this break FE
            let local = |sim_t: f64| (sim_t * skew) as u32;
            let cur = |sim_t: f64| {
                let k = (((sim_t - t0) / 480.0).floor() as u16).min(footprint - 1);
                ((base as u16 + 1 + k) as usize % 512) as u16
            };
            let mut sim_t = t0;
            for _ in 0..40 {
                let c = cur(sim_t);
                let now = local(sim_t);
                match f.resolve(&ring, c, now, tpb) {
                    FramerOut::None => break,
                    FramerOut::Wait(t) | FramerOut::Covered { end_due: t, .. } => {
                        sim_t += t.wrapping_sub(now) as f64 / skew;
                    }
                    FramerOut::Frame(span) => {
                        if let Some(sc) = span.cadence {
                            err_sum += sc.err_ticks as i64;
                            span_sum += sc.span_ticks as u64;
                            samples += 1;
                        }
                        break;
                    }
                }
            }
        }
        assert_eq!(samples, 16, "every frontier frame yields a sample");
        let mean = err_sum as f64 / span_sum as f64 * 1e6;
        assert!((mean - 5200.0).abs() < 800.0, "window mean {mean:.0} ppm");
    }

    /// A backlog frame (whole frame already ringed at first resolve) was
    /// never a tracked frontier: no stamps, no sample — hot-chain backlog
    /// walks contribute nothing, by design.
    #[test]
    fn backlog_frames_carry_no_sample() {
        let mut ring = vec![0u8; 512];
        ring[1] = 6;
        ring[2] = 43; // inst + 40 payload + crc2
        ring[3] = 0x21;
        let mut f = Framer::new();
        match f.resolve(&ring, 46, 1000, 480) {
            FramerOut::Frame(span) => assert!(span.cadence.is_none()),
            _ => panic!("whole ringed frame resolves on the fast path"),
        }
    }
}
