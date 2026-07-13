//! osc-native transport composite (`docs/osc-native-protocol.md`, driver-pattern
//! sec 4, sec 5.4). Routes break/deadline/TX events between the four sub-drivers
//! (`Framer`, `Chain`, `TxEngine`, `ClockTracker`), owns the one hardware CRC
//! engine both TX generation and RX validation share, and muxes their
//! deadlines onto the single tick-compare.

use osc_core::traits::Dispatch;
use osc_core::{BaudRate, BootMode};
use osc_protocol::wire::{Id, Opcode};

use super::chain::Chain;
use super::clock::ClockTracker;
use super::framer::Framer;
use super::ring_wrap;
use super::tx::{TxEngine, TxOut};
use crate::traits::bus::{Deadline, Providers, RxRing, UsartBaud, tick_reached};

mod crc;
mod reply;
mod route;

/// us-per-byte numerator: 10 bit-times/byte x 1e6 us/s. `tpb = TICKS_PER_US x
/// this / baud` stays within u32 for all four operational rates.
const BYTE_TIME_NUMERATOR: u32 = 10_000_000;

/// Slack on the reclaim-suspension frame allowance (sec 6): covers the snooper's
/// deadline-B margin on the predecessor's frame end.
const FRAME_ALLOWANCE_SLACK_BYTES: u32 = 8;

/// Bound on same-wake deadline draining. Generous: one frame consumes at most
/// a handful of due slots per wake (header lock, covered, end, chain, rescue);
/// anything past the bound falls out to `arm_deadline`, whose pend-on-past
/// contract re-enters the ISR rather than losing the slot.
const DEADLINE_DRAIN_MAX: u32 = 8;

/// Backlog frames resolved per wake before yielding via pend-on-past (position
/// from the stream): bounds one wake's dispatch work without ever dropping --
/// the ring holds the rest.
const FRAMES_PER_WAKE: u32 = 16;

/// Transport health counters the chip publishes into the telemetry region
/// (sec 5.3 layer 1: dropped frames are counted, never answered).
pub struct LinkDiag {
    pub crc_fail_count: u32,
    pub framing_drop_count: u32,
}

/// A frame dispatched ahead of its CRC verdict -- the spine (sec 4): dispatch
/// always runs under the CRC's own latency; the verdict gates EFFECTS, never
/// work. The reply (if any) sits staged in the TX engine and the write (if
/// any) in the staging buffer until the frame end verifies.
struct Pending {
    anchor: u16,
    footprint: u16,
    packet_end: u32,
    slot: u8,
    /// Wire effect staged: a reply awaits the send/don't-send verdict.
    staged: bool,
    /// Table effect staged in the dispatcher: a write awaits commit/revert.
    table: bool,
}

pub struct ServoBus<P: Providers> {
    framer: Framer,
    chain: Chain,
    tx: TxEngine<P::Tx>,
    crc: P::Crc,
    ring: P::Ring,
    deadline: P::Deadline,
    baud: P::Baud,
    id: u8,
    rate: BaudRate,
    tpb: u32,
    response_deadline_us: u16,
    pending_id: Option<u8>,
    pending_baud: Option<BaudRate>,
    pending_reboot: Option<BootMode>,
    crc_fails: u32,
    // The one frame dispatched ahead of its CRC verdict (sec 4: the spine).
    // Backpressure bounds it to one: a pending frame IS the frontier, so the
    // single staging slot and the single CRC accumulator are never contended.
    pending: Option<Pending>,
    // Deadline mux (sec 4.1/sec 6/sec 9.1): the soonest live slot arms the compare.
    framer_at: Option<u32>,
    chain_at: Option<u32>,
    // Clock discipline (sec 9.3): MGMT CAL + drift tracker + trim loop.
    clock: ClockTracker,
}

/// Ticks per byte-time at `rate` on the transport clock. Each arm folds to a
/// literal via `const {}` (TICKS_PER_US is an associated const) -- the board
/// build targets +zmmul (no hardware divide), so no runtime division is emitted
/// (mirrors the chip `brr_for` idiom).
fn tpb_for<P: Providers>(rate: BaudRate) -> u32 {
    const fn compute(ticks_per_us: u32, baud_hz: u32) -> u32 {
        ticks_per_us * BYTE_TIME_NUMERATOR / baud_hz
    }
    match rate {
        BaudRate::B500000 => const { compute(<P::Deadline as Deadline>::TICKS_PER_US, 500_000) },
        BaudRate::B1000000 => const { compute(<P::Deadline as Deadline>::TICKS_PER_US, 1_000_000) },
        BaudRate::B2000000 => const { compute(<P::Deadline as Deadline>::TICKS_PER_US, 2_000_000) },
        BaudRate::B3000000 => const { compute(<P::Deadline as Deadline>::TICKS_PER_US, 3_000_000) },
    }
}

/// Wrap-aware "slot `at` is due at `now`".
#[inline]
fn due(now: u32, at: Option<u32>) -> bool {
    matches!(at, Some(at) if tick_reached(now, at))
}

impl<P: Providers> ServoBus<P> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ring: P::Ring,
        deadline: P::Deadline,
        crc: P::Crc,
        tx: P::Tx,
        mut baud: P::Baud,
        id: u8,
        rate: BaudRate,
        response_deadline_us: u16,
    ) -> Self {
        baud.apply(rate);
        Self {
            framer: Framer::new(),
            chain: Chain::new(),
            tx: TxEngine::new(tx),
            crc,
            ring,
            deadline,
            baud,
            id,
            rate,
            tpb: tpb_for::<P>(rate),
            response_deadline_us,
            pending_id: None,
            pending_baud: None,
            pending_reboot: None,
            crc_fails: 0,
            pending: None,
            framer_at: None,
            chain_at: None,
            clock: ClockTracker::new(<P::Deadline as Deadline>::CLOCK_TRIM_STEP_PPM),
        }
    }

    /// Break-wake ISR (LBD: a genuine >=10-bit dominant span landed, sec 3.4
    /// -- garble never reaches this handler) -- a pure wake. It carries
    /// neither position nor time (both are derived from the stream): the
    /// resolver reads them from ring data, so a delayed or coalesced entry
    /// costs nothing (any frames completed meanwhile resolve on the fast path
    /// right here).
    pub fn on_break<D: Dispatch>(&mut self, d: &mut D) {
        let now = self.deadline.now();
        // MGMT CAL train (sec 9.3): while a train is live, breaks are ruler
        // marks, not frame traffic -- stamp against the announced gap and
        // return. The ring still collects the break bytes; the framer's
        // hunt scans them off silently once the train ends (0x00 runs are
        // implausible candidates, and any junk lock dies CRC-uncounted
        // under the hunt's probing flag).
        if self.clock.cal_active() {
            if let Some(at) = self.clock.on_cal_break(
                now,
                self.ring.cursor(),
                <P::Deadline as Deadline>::TICKS_PER_US,
            ) {
                self.framer_at = Some(at);
                self.arm_deadline();
            }
            return;
        }
        // Freshness first, resolve second: the fault service computes one
        // bit (did bytes ring since the last service?) and nothing else --
        // the fault contract. The resolve consumes whatever the ring holds.
        let cursor = self.ring.cursor();
        let fresh = self.framer.on_wire_fault(cursor);
        // Drift stamp only when the newest ringed byte IS a break's 0x00 --
        // classification from ring data, per the contract. A spurious or
        // lagged wake can land at frame end looking FRESH (the frame's own
        // bytes drained since the break's service), and stamping it
        // clobbers the pair in flight -- the tracker starved to zero on
        // silicon under the FE-era latched re-fires (DES pin:
        // `tracker_survives_latched_refires_between_frames`). A CRC tail
        // that happens to end 0x00 leaks one stamp and costs one pair --
        // the byte-exactness and span gates absorb it.
        if fresh && self.newest_is_break_byte(cursor) {
            self.clock
                .on_drift_break(now, self.ring.cursor(), self.ring.bytes().len(), self.tpb);
        }
        self.drive_framer(d);
        // A wake whose evidence isn't ringed yet leaves the resolver with
        // nothing (a coalesced or lagged service -- the contract's
        // spurious-wake case): the ring tells the truth one byte-time
        // later -- arm the recheck. A spurious re-fire costs one empty wake.
        if !fresh && self.framer_at.is_none() {
            self.framer_at = Some(now.wrapping_add(self.tpb));
        }
        // Wire safety: a staged, not-yet-streaming reply must never fire
        // into the host's NEXT frame. But an FE alone doesn't mean the host
        // moved on -- lagged deliveries routinely resolve the reply's own
        // frame (or reach its covered checkpoint) inside this very wake
        // (silicon event-trace: COVERED -> KILL 2 us apart, then verify
        // armed the chain over the emptied engine -- ghost trigger, silent
        // no-reply). Positional truth from the stream decides: kill only when
        // ringed bytes FOLLOW the reply's own frame. A pending frame is
        // mid-verdict by definition (its CRC gate reclaims the reply if this
        // break garbled the frame); a Waiting chain expects predecessor
        // breaks and only suspends its reclaim window (sec 6). Broadcast-ENUM
        // replies are exempt (sec 9.2): colliding with a peer matcher IS their
        // contract -- a yielded laggard turns the walk's collision signal
        // into one clean frame and prunes its subtree. A CRC-verified fresh
        // instruction still supersedes them (route_frame).
        if self.tx.staged()
            && !self.tx.collision_tolerant()
            && !self.chain.waiting()
            && self.pending.is_none()
            && !self.framer.caught_up(self.ring.cursor())
        {
            self.tx.abort();
            self.chain.reset();
            self.chain_at = None;
        }
        // sec 6: a break while we hold a staged chain slot means the predecessor
        // is alive -- suspend its reclaim window while the frame plays out.
        let out = self.chain.on_break_observed(now);
        self.route_chain(out);
        self.arm_deadline();
    }

    /// The newest ringed byte -- the wire-fault service's break/re-fire
    /// discriminator (a break rings its 0x00 last; a frame-end re-fire
    /// sees the CRC tail).
    fn newest_is_break_byte(&self, cursor: u16) -> bool {
        let ring = self.ring.bytes();
        let len = ring.len();
        len != 0 && ring[ring_wrap(cursor as usize + len - 1, len)] == 0x00
    }

    /// A verified frame between breaks: classify its shape from ring data
    /// (only silent shapes pair) and record it with the tracker.
    pub(super) fn drift_note_verified(&mut self, anchor: u16, footprint: u16) {
        // A second frame collapses the record to Many -- its shape is never
        // read, so skip the ring classification.
        if !self.clock.pair_open() {
            self.clock.note_verified(footprint, false);
            return;
        }
        let inst = self.ring_inst(anchor);
        let ring = self.ring.bytes();
        let len = ring.len();
        if len == 0 {
            return; // defensive: no ring, no record
        }
        let id = ring[ring_wrap(anchor as usize + 1, len)];
        let silent = match inst.opcode() {
            Some(Opcode::Gwrite) => true,
            Some(Opcode::Write | Opcode::Commit) => inst.noreply() || id == Id::BROADCAST.as_byte(),
            _ => false,
        };
        self.clock.note_verified(footprint, silent);
    }

    /// Tick-compare ISR: one or more muxed deadlines are due. Every slot a
    /// handler body overruns (front-loaded dispatch inside the covered window
    /// routinely overruns `end_due`) is drained in this same invocation -- a
    /// pend->exit->re-enter round trip costs ~10 us of turnaround.
    pub fn on_deadline<D: Dispatch>(&mut self, d: &mut D) {
        for _ in 0..DEADLINE_DRAIN_MAX {
            let now = self.deadline.now();
            let mut progressed = false;
            if due(now, self.framer_at) {
                progressed = true;
                self.framer_at = None;
                // CAL watchdog (sec 9.3): during a live train the framer slot
                // is the train's silence bound and nothing else -- expiry
                // means the train died. Abandon it (no decision) and let
                // the framer hunt whatever actually arrived. A dangling
                // announce (train never started) dies here too.
                self.clock.abandon_cal();
                self.drive_framer(d);
            }
            if due(now, self.chain_at) {
                progressed = true;
                self.chain_at = None;
                let out = self.chain.on_deadline(now);
                self.route_chain(out);
            }
            if !progressed {
                break;
            }
        }
        self.arm_deadline();
    }

    /// TX DMA arm-complete ISR: stream the next arm, or apply deferred config
    /// once the whole reply has drained (sec 4.2) -- the ack always leaves at
    /// the old id/baud, the change lands after.
    pub fn on_tx_complete(&mut self) {
        if self.tx.on_arm_complete(&mut self.crc) == TxOut::Released {
            if let Some(id) = self.pending_id.take() {
                self.id = id;
            }
            if let Some(baud) = self.pending_baud.take() {
                self.baud.apply(baud);
                self.rate = baud;
                self.tpb = tpb_for::<P>(self.rate);
                self.clock.restart();
            }
            // A pending reboot waits for the main loop's `take_reboot`.
        }
    }

    /// Main-loop poll for a deferred reboot -- withheld while a reply is
    /// draining (a reset mid-ack truncates the frame on the wire; bench-
    /// caught), honored on the first poll after the TX released. A silent
    /// (NOREPLY/broadcast) reboot stages no ack and takes immediately.
    pub fn take_reboot(&mut self) -> Option<BootMode> {
        if self.tx.busy() {
            return None;
        }
        self.pending_reboot.take()
    }

    pub fn diag(&self) -> LinkDiag {
        LinkDiag {
            crc_fail_count: self.crc_fails,
            framing_drop_count: self.framer.drops(),
        }
    }

    /// Main-loop poll, ISRs masked by the caller (the `diag` idiom): drain a
    /// completed measurement -- a CAL train (absolute) or a drift window
    /// (baseline-relative) -- through the trim loop. Returns the new trim
    /// total -- signed chip steps from the factory default, positive =
    /// slower -- for the caller to apply to the oscillator between frames.
    pub fn poll_clock_trim(&mut self) -> Option<i8> {
        self.clock.poll()
    }

    pub(super) fn reply_gap(&self) -> u32 {
        super::REPLY_GAP_US * <P::Deadline as Deadline>::TICKS_PER_US
    }

    pub(super) fn reclaim(&self) -> u32 {
        self.response_deadline_us as u32 * <P::Deadline as Deadline>::TICKS_PER_US
    }

    /// How long an observed predecessor break suspends its reclaim window:
    /// the largest legal frame plus the snooper's own end-detection slack.
    pub(super) fn frame_allowance(&self) -> u32 {
        (super::FRAME_MAX as u32 + FRAME_ALLOWANCE_SLACK_BYTES) * self.tpb
    }

    /// Arm the compare at the soonest live slot, or cancel if none. A slot
    /// already reached counts as due NOW, not as a full wrap away -- an ISR
    /// body that overruns a pending deadline (front-loaded dispatch inside
    /// the covered window does, routinely) must pend it, not push it behind
    /// every future slot (bench signature: deadline B riding the +100 us
    /// rescue wake).
    fn arm_deadline(&mut self) {
        let now = self.deadline.now();
        let remaining = |at: u32| {
            if tick_reached(now, at) {
                0
            } else {
                at.wrapping_sub(now)
            }
        };
        let mut best: Option<u32> = None;
        for at in [self.framer_at, self.chain_at].into_iter().flatten() {
            best = Some(match best {
                Some(b) if remaining(b) <= remaining(at) => b,
                _ => at,
            });
        }
        match best {
            Some(at) => self.deadline.set(at),
            None => self.deadline.cancel(),
        }
    }

    /// sec 9.1 rescue: the chip's line sampler measured a >=[`RESCUE_LOW_US`]
    /// dominant low with zero ring progress -- a signal no transport wake can
    /// observe (the break detector latches only at a span's END). Called from
    /// the main loop under a critical section; the pulse is still holding the
    /// line when the sampler declares, so the cursor is provably still.
    ///
    /// [`RESCUE_LOW_US`]: super::RESCUE_LOW_US
    pub fn on_rescue_break(&mut self) {
        // Own-TX guard: unreachable by physics on both wire configs (own
        // data always carries stop-bit highs; the buffered wire's sense pin
        // reads forced mark during TX), kept because an abort of a draining
        // ack is the one real damage a spurious call could do.
        if self.tx.streaming() {
            return;
        }
        // sec 9.1: volatile rate switch -- the config register is untouched.
        self.baud.apply(BaudRate::B500000);
        self.rate = BaudRate::B500000;
        self.tpb = tpb_for::<P>(self.rate);
        self.clock.restart();
        // Ladder bootstrap (position from the stream): a rescue pulse delivers
        // no start edges, so the cursor is provably still -- the one
        // sanctioned cursor read.
        let cursor = self.ring.cursor();
        self.framer.resync(cursor);
        self.chain.reset();
        self.tx.abort();
        // A dropped pending frame's staged table effect is reclaimed by the
        // dispatcher's auto-revert on the next dispatch.
        self.pending = None;
        self.framer_at = None;
        self.chain_at = None;
        self.arm_deadline();
    }

    /// Verdict-first ops: COMMIT (applies the whole staging buffer), MGMT
    /// (reboots/config), and anything undecodable -- their effects can't stage,
    /// so the CRC verdict must come first (the bus checks it, then dispatch
    /// applies directly on that contract). Everything else dispatches ahead of
    /// its verdict, effects gated by it. Read from the same INST byte
    /// [`decode`] reads, so routing and dispatch can never disagree.
    pub(super) fn verdict_first(&self, anchor: u16) -> bool {
        !matches!(
            self.ring_inst(anchor).opcode(),
            Some(Opcode::Ping | Opcode::Read | Opcode::Gread | Opcode::Write | Opcode::Gwrite)
        )
    }
}

#[cfg(test)]
mod tests;
