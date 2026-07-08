//! Stream-continuity RX resolver (`docs/osc-servo-transport.md` §6 A1/A2,
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

use osc_protocol::frame::Header;

use super::ring_wrap;
use crate::traits::bus::tick_reached;

/// A resolved frame's location in the ring: anchor index + ring-byte count,
/// plus the wire-end estimate (`packet_end`) the reply trigger's T_turn is
/// measured from (§7). Frontier frames estimate from their break tick + the
/// drift adder (conservative, never early); backlog frames ended in the past
/// and carry `now` (elastic best-effort timing, §7).
pub struct FrameSpan {
    pub anchor: u16,
    pub footprint: u16,
    pub packet_end: u32,
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
const HEADER_LEAD_BYTES: u32 = HEADER_SPAN_BYTES as u32 - 1;

// §3.2/A2: a real break rings an all-zero byte; the ladder expects it at the
// derived anchor. Anything else there is desync (noise inserted/lost bytes).
const BREAK_RING_BYTE: u8 = 0x00;

// Half a byte-time of wake slack folded onto deadline A.
const ISR_SLACK_DIV: u32 = 2;

// Wire-proportional drift adder on the frontier packet-end estimate (~1.6%,
// §9.3): an in-spec-slow transmitter can never make the estimate early.
const DRIFT_SHIFT: u32 = 6;

// The wire CRC tail: the covered span completes this many byte-times before
// the frame end, so the covered checkpoint leads deadline B by it.
const COVERED_TAIL_BYTES: u32 = 2;

// Plateau pacing (§4.1): bounded rechecks one byte-time apart past a
// computed frontier aim. A recheck is counted only when the aim actually
// FIRED with the data still short — scheduled first-aims and early wakes
// (garble FEs) are free — and any ring progress refunds the budget. An
// exhausted budget parks the frontier at the giveup horizon (wake pacing
// only); the progress-idle detector below is the sole death authority.
const STARVE_RECHECKS: u8 = 8;

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

/// The frontier frame's wait phase; each phase gets a fresh computed aim and
/// a fresh starvation budget.
#[derive(Copy, Clone, PartialEq, Eq)]
enum Phase {
    /// Header bytes still arriving (aim = deadline A).
    Header,
    /// Header locked; covered span still arriving (aim = covered checkpoint).
    PreCovered,
    /// Covered emitted; wire-CRC bytes still arriving (aim = deadline B).
    Tail,
}

/// Frontier bookkeeping — schedule hints only; the ring stays the truth.
struct Frontier {
    phase: Phase,
    /// The current absolute aim; re-emitted verbatim on early wakes.
    aim: u32,
    /// Starvation rechecks consumed in this phase.
    rechecks: u8,
    /// Ring progress watermark + its tick: the dead-transmitter detector.
    seen: u16,
    progress_tick: u32,
}

impl Frontier {
    const fn idle() -> Self {
        Self {
            phase: Phase::Header,
            aim: 0,
            rechecks: u8::MAX, // sentinel: no aim computed yet
            seen: 0,
            progress_tick: 0,
        }
    }
}

pub struct Framer {
    /// The ladder (A2): ring index where the current frame's break byte sits.
    anchor: u16,
    /// A recovery episode is in progress (desync/reject/giveup): drops count
    /// once per episode, and probe candidates that fail CRC are not wire
    /// faults. Cleared when a candidate resolves as a whole valid frame.
    hunting: bool,
    /// Tick of the newest FE (A1): the frontier frame's break-time estimate.
    /// A garble FE overwriting it makes estimates later = safer.
    frontier_tick: u32,
    /// An FE landed whose byte the ladder has not consumed yet. Every FE
    /// promises a ring byte, but a prompt entry can beat the byte's own DMA
    /// drain (F2 edge) — the resolver then sees nothing and must not go
    /// aimless, or a terminal frame sits unresolved until the next wire
    /// event (silicon 2026-07-08: the no-reply wedge). While set, a
    /// caught-up resolve waits one byte-time from the FE instead of
    /// returning `None`; re-derived on every walk, so no wake interleaving
    /// can destroy the aim. Surrendered once the promise window passes (a
    /// phantom FE delivers nothing).
    fe_pending: bool,
    frontier: Frontier,
    drops: u32,
    /// Fixed tick slack past the packet-end estimate for deadline B.
    end_slack: u32,
}

impl Framer {
    pub const fn new(end_slack: u32) -> Self {
        Self {
            anchor: 0,
            hunting: false,
            frontier_tick: 0,
            fe_pending: false,
            frontier: Frontier::idle(),
            drops: 0,
            end_slack,
        }
    }

    /// Frames dropped at transport layer 1 (§5.3), monotonic.
    pub fn drops(&self) -> u32 {
        self.drops
    }

    /// An FE event landed (break or garble — the resolver does not care
    /// which; classification is positional, A2). Record the tick: if the
    /// frontier frame is still headerless, this is (an upper bound on) its
    /// break time.
    pub fn on_fe(&mut self, now: u32) {
        self.frontier_tick = now;
        self.fe_pending = true;
    }

    /// Re-sync the ladder to a known ring position — bootstrap only (boot is
    /// position 0 by construction; rescue reads a provably-still cursor).
    pub fn resync(&mut self, cursor: u16) {
        self.anchor = cursor;
        self.frontier = Frontier::idle();
        self.fe_pending = false;
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
            // The FE promise (see `fe_pending`): its byte lands within a DMA
            // drain of the FE, so one byte-time from the FE tick bounds it
            // with margin. Past the window, a phantom FE delivered nothing —
            // surrender to idle.
            if self.fe_pending {
                let aim = self.frontier_tick.wrapping_add(tpb);
                if !tick_reached(now, aim) {
                    return FramerOut::Wait(aim);
                }
                self.fe_pending = false;
            }
            return FramerOut::None; // ladder caught up; wait for wire
        }
        self.fe_pending = false; // the promised byte (at least) is in
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
            let aim = self
                .frontier_tick
                .wrapping_add(HEADER_LEAD_BYTES.wrapping_mul(tpb))
                .wrapping_add(tpb / ISR_SLACK_DIV);
            return self.wait(Phase::Header, aim, now, tpb, cursor, len);
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
        let packet_end = self.packet_end(footprint, received, now, tpb);
        if received >= footprint {
            // Complete — the fast path: process now, timing per packet_end
            // (frontier estimate when fresh, elastic when the frame ended in
            // the past — backlog or stale tick).
            let anchor = self.anchor;
            self.anchor = ring_wrap(anchor as usize + footprint as usize, len) as u16;
            self.frontier = Frontier::idle();
            return FramerOut::Frame(FrameSpan {
                anchor,
                footprint,
                packet_end,
            });
        }
        // Frontier with a known end: covered checkpoint, then deadline B.
        let end_due = packet_end.wrapping_add(self.end_slack);
        if self.frontier.phase != Phase::Tail {
            if received >= footprint - COVERED_TAIL_BYTES as u16 {
                self.frontier.phase = Phase::Tail;
                self.frontier.aim = end_due;
                self.frontier.rechecks = 0;
                return FramerOut::Covered {
                    span: FrameSpan {
                        anchor: self.anchor,
                        footprint,
                        packet_end,
                    },
                    end_due,
                };
            }
            let covered_due = end_due.wrapping_sub(COVERED_TAIL_BYTES.wrapping_mul(tpb));
            return self.wait(Phase::PreCovered, covered_due, now, tpb, cursor, len);
        }
        self.wait(Phase::Tail, end_due, now, tpb, cursor, len)
    }

    /// Wire-end estimate for the frame at the ladder anchor. The break-tick
    /// estimate (tick + remaining byte-times + drift) is trusted while `now`
    /// is within the frame's own processing window (deadline B fires just
    /// past the estimate by construction — pend-on-past handles that grid
    /// cleanly). Far past it, the tick is stale (FE deliveries coalesce
    /// behind higher vectors — A1 records the tick at DELIVERY, which can
    /// lag the wire) or the frame ended long ago (backlog): re-estimate from
    /// live progress — the missing bytes arrive at wire pace from `now` at
    /// the earliest, conservative and degrading to elastic `now` for a
    /// complete frame.
    fn packet_end(&self, footprint: u16, received: u16, now: u32, tpb: u32) -> u32 {
        let frame_ticks = (footprint as u32 - 1).wrapping_mul(tpb);
        let est = self
            .frontier_tick
            .wrapping_add(frame_ticks)
            .wrapping_add((footprint as u32).wrapping_mul(tpb) >> DRIFT_SHIFT);
        // Trust window: end-slack plus a couple of byte-times of wake jitter.
        let window = self
            .end_slack
            .wrapping_add(COVERED_TAIL_BYTES.wrapping_mul(tpb));
        if tick_reached(now, est.wrapping_add(window)) {
            let missing = footprint.saturating_sub(received) as u32;
            now.wrapping_add(missing.wrapping_mul(tpb))
        } else {
            est
        }
    }

    /// Phase-aware wait: a fresh phase installs the computed aim with a full
    /// starvation budget; an early wake re-emits the standing aim untouched
    /// (garble FEs must not consume budget); an aim that fired with data
    /// still short converts into a one-byte-time recheck, bounded. A wire
    /// that makes NO progress for [`STARVE_GIVEUP_BYTES`] declares the
    /// partial dead regardless of the aim (a junk header's far-future
    /// footprint must not park the ladder) — the hunt resumes one byte in.
    fn wait(
        &mut self,
        phase: Phase,
        fresh_aim: u32,
        now: u32,
        tpb: u32,
        cursor: u16,
        len: usize,
    ) -> FramerOut {
        let giveup_span = STARVE_GIVEUP_BYTES.wrapping_mul(tpb);
        let f = &mut self.frontier;
        if f.rechecks == u8::MAX || cursor != f.seen {
            f.seen = cursor;
            f.progress_tick = now;
            // Progress refunds the starvation budget: rechecks bound a
            // transmitter that stalls, not one that is merely slower than a
            // lag-inflated wake storm (silicon 2026-07-08: coalesced-FE
            // wakes burned 8 rechecks mid-frame on a live zero-gap burst
            // and sacrificed the partial). A dead wire still trips the
            // progress-idle horizon below.
            if f.rechecks != u8::MAX {
                f.rechecks = 0;
            }
        } else if tick_reached(now, f.progress_tick.wrapping_add(giveup_span)) {
            return self.give_up(len, now);
        }
        if f.phase != phase || f.rechecks == u8::MAX {
            f.phase = phase;
            f.aim = fresh_aim;
            f.rechecks = 0;
        } else if tick_reached(now, f.aim) {
            if f.rechecks < STARVE_RECHECKS {
                f.aim = now.wrapping_add(tpb);
                f.rechecks += 1;
            } else {
                // Budget exhausted: park — hand the aim to the giveup
                // horizon so a stalled wire wakes exactly once more (a
                // reached aim left standing would pend-on-past forever).
                f.aim = f.progress_tick.wrapping_add(giveup_span);
            }
        }
        // Wake at the sooner of the phase aim and the giveup horizon, so a
        // dead wire is detected without waiting out a junk footprint.
        let giveup_at = self.frontier.progress_tick.wrapping_add(giveup_span);
        let aim = self.frontier.aim;
        let sooner = if aim.wrapping_sub(now) <= giveup_at.wrapping_sub(now) {
            aim
        } else {
            giveup_at
        };
        FramerOut::Wait(sooner)
    }

    /// Transmitter died mid-frame (or a junk header trapped the ladder):
    /// sacrifice one byte and resume the hunt — a real frame inside the
    /// span is still found, CRC-gated (§3.3).
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
