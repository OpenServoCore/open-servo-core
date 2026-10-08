//! osc-native transport composite (`docs/osc-native-protocol.md`, driver-pattern
//! sec 4, sec 5.4). Routes break/deadline/TX events between the four sub-drivers
//! (`Framer`, `Chain`, `TxEngine`, `ClockTracker`), owns the one hardware CRC
//! engine both TX generation and RX validation share, and muxes their
//! deadlines onto the single tick-compare.

use osc_protocol::wire::{Id, Opcode};
use osc_servo_core::traits::Dispatch;
use osc_servo_core::{BaudRate, BootMode};

use super::chain::Chain;
use super::clock::ClockTracker;
use super::decode::Slot;
use super::framer::Framer;
use super::ring_wrap;
use super::tx::{TxEngine, TxOut};
use crate::traits::bus::{BreakStamps, Deadline, Providers, RxRing, UsartBaud, tick_reached};

mod crc;
mod reply;
mod route;
mod tel;

use tel::TelBurst;

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

/// Stale break stamps one frame's verdict may skip before taking what is
/// there (sec 8): orphans arrive one at a time, and a lapped stamp ring
/// holds no more than its depth, so a longer run is a provider that never
/// runs dry - a bound, like every busy loop here.
const STALE_SKIPS_MAX: u32 = 8;

/// Half the break-stamp latch's range ([`BreakStamps`]: 16 bits of tick):
/// a stamp unwraps to the tick nearest its break's placement, so the
/// placement must land within this of the detector's instant.
const STAMP_HALF_RANGE: u32 = 1 << 15;

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
    slot: Slot,
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
    stamps: P::Stamps,
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
    // A break wake serviced before its 0x00 rang: the wake's stamp and the
    // cursor it saw, re-inspected one byte-time on (the third mux slot).
    unringed: Option<(u32, u16)>,
    // Clock discipline (sec 9.3): MGMT CAL + drift tracker + trim loop.
    clock: ClockTracker,
    // TEL burst stager (sec 5.3): armed by tel_count, fed by `poll_tel`.
    burst: TelBurst,
    // Rescue sampler window (sec 9.1): the first low sample's tick and the
    // cursor the ring must hold until the declaration.
    rescue_since: Option<u32>,
    rescue_cursor: u16,
    // A verified frame whose reply is staged: its tracker record waits
    // behind the reply's trigger (sec 8), so the record's body never sits
    // between the verdict and the status break.
    reply_record: Option<(u16, u16)>,
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

/// The tick nearest `near` whose low 16 bits are `raw`.
#[inline]
fn unwrap_near(near: u32, raw: u16) -> u32 {
    let off = raw.wrapping_sub(near as u16);
    if off < STAMP_HALF_RANGE as u16 {
        near.wrapping_add(off as u32)
    } else {
        near.wrapping_sub((STAMP_HALF_RANGE * 2) - off as u32)
    }
}

impl<P: Providers> ServoBus<P> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ring: P::Ring,
        deadline: P::Deadline,
        crc: P::Crc,
        tx: P::Tx,
        mut baud: P::Baud,
        stamps: P::Stamps,
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
            stamps,
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
            unringed: None,
            clock: ClockTracker::new(<P::Deadline as Deadline>::CLOCK_TRIM_STEP_PPM),
            burst: TelBurst::new(),
            rescue_since: None,
            rescue_cursor: 0,
            reply_record: None,
        }
    }

    /// Bind the TEL sample channel's consumer half. Bringup-only, before any
    /// arm can arrive; without it `tel_arm` stays a drop.
    pub fn attach_tel(&mut self, drain: crate::tel::TelDrain) {
        self.burst.attach(drain);
    }

    /// Break-wake ISR (the bus held low a break's length, sec 3.4 -- garble
    /// never reaches this handler) -- a pure wake. It carries neither
    /// position nor time (both are derived from the stream): the resolver
    /// reads them from ring data, so a delayed or coalesced entry costs
    /// nothing (any frames completed meanwhile resolve on the fast path
    /// right here).
    ///
    /// The detector may fire ahead of the stop-bit sample that rings the
    /// break's 0x00, so the wake can beat its own byte into the ring. A wake
    /// whose break byte has not rung gets its ring-dependent service (CAL
    /// mark, drift stamp, resolver, stale-reply kill) one byte-time later
    /// in [`Self::reinspect`], from ring data alone. The drift stamp itself
    /// is the detector's hardware latch ([`BreakStamps`]), not this entry's
    /// `now`: under load the entry lags the detector by up to two frames
    /// and merges breaks, and the lag's beat against the frame cadence
    /// gated every pair (DES pin `drift_stamps_ignore_isr_entry_lag`).
    pub fn on_break<D: Dispatch>(&mut self, d: &mut D) {
        let now = self.deadline.now();
        let cursor = self.ring.cursor();
        // MGMT CAL train (sec 9.3): while a train is live, breaks are ruler
        // marks, not frame traffic - stamp against the announced gap and
        // return. The ring still collects the break bytes; the framer's
        // hunt scans them off silently once the train ends (0x00 runs are
        // implausible candidates, and any junk lock dies CRC-uncounted
        // under the hunt's probing flag). A mark has no frame to settle the
        // ladder onto it, so it is classified by the newest byte: its 0x00,
        // rung fresh since the last service (a quiet wire between marks).
        // The mark's hardware stamp is dropped: the ruler measures entry
        // stamps, and a mark's stamp left latched would be taken by the
        // first frame after the train, costing that frame's pair and the
        // next (DES: `cal_anchors_then_tracker_follows`).
        if self.clock.cal_active() {
            let dropped = self.stamps.clear();
            crate::bench::trim_probe(|p| p.cleared += dropped as u32);
            let ringed = self.framer.on_wire_fault(cursor) && self.newest_is_break_byte(cursor);
            self.unringed = (!ringed).then_some((now, cursor));
            if ringed {
                self.cal_mark(now);
            }
            self.arm_deadline();
            return;
        }
        // A break during a live burst is the host reclaiming the line -- and
        // the host's abort lever. Kill the burst and any in-flight frame
        // before the resolver runs (the garbled frame is the host's chosen
        // cost); the arriving instruction then handles normally, including a
        // fresh tel_count re-arm. Inactive burst: this block is inert, so
        // non-burst exchanges are untouched.
        if self.burst.active() {
            self.burst.abort();
            self.tx.abort();
            self.record_behind_reply();
            self.chain.reset();
            self.chain_at = None;
        }
        self.unringed = (!self.serve_break(d)).then_some((now, cursor));
        // sec 6: a break while we hold a staged chain slot means the
        // predecessor is alive -- suspend its reclaim window while the
        // frame plays out.
        let out = self.chain.on_break_observed(now);
        self.route_chain(out);
        self.arm_deadline();
    }

    /// One byte-time after a wake whose break byte had not rung. A ruler
    /// mark is the first byte ringed since the wake, a 0x00, served at the
    /// wake's stamp; a frame break is wherever the ladder stands. Nothing
    /// ringed, or anything else, was no break of this wake's and only
    /// re-drives the resolver.
    fn reinspect<D: Dispatch>(&mut self, d: &mut D) {
        let Some((at, cursor)) = self.unringed.take() else {
            return;
        };
        if self.clock.cal_active() {
            let ring = self.ring.bytes();
            if self.ring.cursor() != cursor && ring.get(cursor as usize) == Some(&0x00) {
                self.framer
                    .on_wire_fault(ring_wrap(cursor as usize + 1, ring.len()) as u16);
                self.cal_mark(at);
            }
            return;
        }
        self.serve_break(d);
    }

    /// One CAL ruler mark at the wake's stamp `at`; the train's watchdog,
    /// then its end-of-train hunt, ride the framer slot.
    fn cal_mark(&mut self, at: u32) {
        if let Some(t) = self
            .clock
            .on_cal_break(at, <P::Deadline as Deadline>::TICKS_PER_US)
        {
            self.framer_at = Some(t);
        }
    }

    /// The ring-dependent half of a frame break's service. Position from
    /// the stream: the resolver walks the ladder through every frame whole
    /// in the ring and onto this break's byte once it has rung, however
    /// late the wake was served - a service lagging a byte-time or more
    /// finds data bytes newest, and a newest-byte test there starved the
    /// tracker under 1M bursts (DES pin `lagged_break_wakes_still_pair`).
    /// Returns false while this wake's byte has not rung: the caller
    /// re-inspects one byte-time on. Nothing here touches the stamps: a
    /// frame's break takes its stamp at the frame's verdict (sec 8), so the
    /// break body carries no tracker work and a backlog deeper than one
    /// drive's bound loses nothing (DES pin
    /// `saturated_backlog_stamps_every_break`).
    fn serve_break<D: Dispatch>(&mut self, d: &mut D) -> bool {
        let drained = self.drive_framer(d);
        crate::bench::trim_probe(|p| p.bound_hits += (!drained) as u32);
        let ringed = self
            .framer
            .break_ringed(self.ring.bytes(), self.ring.cursor())
            .is_some();
        self.kill_stale_reply();
        ringed
    }

    /// Wire safety: a staged, not-yet-streaming reply must never fire into
    /// the host's NEXT frame. But a wake alone doesn't mean the host moved
    /// on -- lagged deliveries routinely resolve the reply's own frame (or
    /// reach its covered checkpoint) inside the very wake (silicon
    /// event-trace: COVERED -> KILL 2 us apart, then verify armed the chain
    /// over the emptied engine -- ghost trigger, silent no-reply).
    /// Positional truth from the stream decides: kill only when ringed
    /// bytes FOLLOW the reply's own frame. A pending frame is mid-verdict by
    /// definition (its CRC gate reclaims the reply if this break garbled
    /// the frame); a Waiting chain expects predecessor breaks and only
    /// suspends its reclaim window (sec 6). Broadcast-ENUM replies are
    /// exempt (sec 9.2): colliding with a peer matcher IS their contract --
    /// a yielded laggard turns the walk's collision signal into one clean
    /// frame and prunes its subtree. A CRC-verified fresh instruction still
    /// supersedes them (route_frame).
    fn kill_stale_reply(&mut self) {
        if self.tx.staged()
            && !self.tx.collision_tolerant()
            && !self.chain.waiting()
            && self.pending.is_none()
            && !self.framer.caught_up(self.ring.cursor())
        {
            self.tx.abort();
            self.record_behind_reply();
            self.chain.reset();
            self.chain_at = None;
        }
    }

    /// The newest ringed byte -- the wake service's break discriminator (a
    /// break rings its 0x00 last; a frame-end re-fire sees the CRC tail).
    fn newest_is_break_byte(&self, cursor: u16) -> bool {
        let ring = self.ring.bytes();
        let len = ring.len();
        len != 0 && ring[ring_wrap(cursor as usize + len - 1, len)] == 0x00
    }

    /// A frame's verdict passed: its break takes its stamp (sec 8), then the
    /// frame is classified from ring data (only silent shapes pair) and
    /// recorded with the tracker, so the stamp lands between the frames it
    /// brackets however many frames one wake resolves. Stamps are latched
    /// in wire order and frames verify in wire order, so the oldest stamp
    /// not yet taken is this break's - unless it is older than the previous
    /// stamp plus the ring distance between the two breaks allows (the wire
    /// delivers no byte in under a byte-time): that one belongs to no frame
    /// (a parked low's re-fire, a CRC-failed frame's break) and is skipped.
    /// A stamp that reads too new is a later break's behind a detector
    /// miss, indistinguishable from an inter-burst gap: it is taken, the
    /// pairing runs one break behind until the stamps run dry at the next
    /// gap, and the pairs between measure the next frame's span.
    ///
    /// The latch holds 16 bits of tick, so a stamp is placed first: the
    /// break's byte rang one wire-time of everything ringed behind it
    /// before now, and the stamp unwraps to the tick nearest that - exact
    /// while the placement is within half the latch's range of the
    /// detector (a verdict any depth of backlog late, and any body lag
    /// under 680 us; the servo's own bytes never ring, so a frame verified
    /// behind a long reply of ours mis-places by the reply's span, its
    /// pair gates, and the next frame's placement stands on its own).
    /// A frame with a staged reply records behind the reply's trigger
    /// ([`Self::record_behind_reply`]): at 0.5M the verify body ran past
    /// the 12 us reply gap, so the record's length landed in the read
    /// turnaround (+2 us, bench) while a frame whose body ends inside the
    /// gap never showed it. Records run in frame order: one still waiting
    /// runs ahead of this frame's. Out of line: inlined it moved the
    /// resolver's hot path (3-4 us per exchange on the bench).
    #[cfg_attr(target_arch = "riscv32", inline(never))]
    pub(super) fn drift_record(&mut self, anchor: u16, footprint: u16) {
        self.record_behind_reply();
        self.record(anchor, footprint);
    }

    /// The tracker record a staged reply holds back, run once the reply has
    /// triggered or been dropped. Out of line: six call sites, inlined it
    /// cost 370 B of flash.
    #[cfg_attr(target_arch = "riscv32", inline(never))]
    pub(super) fn record_behind_reply(&mut self) {
        if let Some((anchor, footprint)) = self.reply_record.take() {
            self.record(anchor, footprint);
        }
    }

    fn record(&mut self, anchor: u16, footprint: u16) {
        let len = self.ring.bytes().len();
        let past = ring_wrap(anchor as usize + 1, len) as u16;
        crate::bench::trim_probe(|p| p.frames += 1);
        let ringed_since = ring_wrap(self.ring.cursor() as usize + len - past as usize, len) as u32;
        let placed = self
            .deadline
            .now()
            .wrapping_sub(ringed_since.wrapping_mul(self.tpb));
        let floor = self.clock.stamp_floor(past, len, self.tpb);
        let mut stamp = None;
        // Opaque bound: with the literal, LLVM unrolls the loop eightfold
        // (280 instructions of flash for a path that skips once in a run).
        for _ in 0..core::hint::black_box(STALE_SKIPS_MAX) {
            let Some(raw) = self.stamps.take() else {
                break;
            };
            let s = unwrap_near(placed, raw);
            match floor {
                Some(f) if (s.wrapping_sub(f) as i32) < 0 => {
                    crate::bench::trim_probe(|p| p.stale += 1);
                }
                _ => {
                    stamp = Some(s);
                    break;
                }
            }
        }
        match stamp {
            Some(s) => self.clock.on_drift_break(s, past, len, self.tpb),
            None => crate::bench::trim_probe(|p| p.unstamped += 1),
        }
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
            if due(now, self.reinspect_at()) {
                progressed = true;
                self.reinspect(d);
            }
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
                // The restart leaves no stamp floor: an orphan latched before
                // the change would pass as the next frame's stamp and put
                // every later frame one stamp behind (DES pin
                // `orphan_stamp_before_a_rate_change_never_shifts_pairs`).
                let dropped = self.stamps.clear();
                crate::bench::trim_probe(|p| p.cleared += dropped as u32);
            }
            // A pending reboot waits for the main loop's `take_reboot`.
            self.burst.on_tx_released();
            // The ladder waited on the CRC engine while the reply streamed.
            if !self.framer.caught_up(self.ring.cursor()) {
                self.framer_at = Some(self.deadline.now());
                self.arm_deadline();
            }
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

    fn reinspect_at(&self) -> Option<u32> {
        self.unringed.map(|(at, _)| at.wrapping_add(self.tpb))
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
        for at in [self.reinspect_at(), self.framer_at, self.chain_at]
            .into_iter()
            .flatten()
        {
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

    /// sec 9.1 rescue sampler: one call per main-loop wake under a critical
    /// section, with `low` read inside it. Declares after >= [`RESCUE_LOW_US`]
    /// of low samples with zero ring progress - a length no transport wake
    /// can measure (the break detector wakes once per span). A sample taken
    /// while the servo's own TX holds the wire restarts the window: HDSEL
    /// keeps own bytes out of the ring, so their low bits would otherwise
    /// pass for a host's pulse. The TX state is read in the pin's critical
    /// section, so no TX release can fall between sample and declaration.
    ///
    /// [`RESCUE_LOW_US`]: super::RESCUE_LOW_US
    pub fn sample_rescue(&mut self, low: bool) {
        let cursor = self.ring.cursor();
        let now = self.deadline.now();
        if !low || self.tx.streaming() || cursor != self.rescue_cursor {
            self.rescue_since = None;
            self.rescue_cursor = cursor;
        } else if let Some(t0) = self.rescue_since {
            if now.wrapping_sub(t0)
                >= super::RESCUE_LOW_US * <P::Deadline as Deadline>::TICKS_PER_US
            {
                self.rescue_since = None;
                self.on_rescue_break();
            }
        } else {
            self.rescue_since = Some(now);
        }
    }

    /// sec 9.1 rescue declaration, reached through [`Self::sample_rescue`]:
    /// the pulse is still holding the line, so the cursor is provably still.
    fn on_rescue_break(&mut self) {
        // sec 9.1: volatile rate switch -- the config register is untouched.
        self.baud.apply(BaudRate::B500000);
        self.rate = BaudRate::B500000;
        self.tpb = tpb_for::<P>(self.rate);
        self.clock.restart();
        let dropped = self.stamps.clear();
        crate::bench::trim_probe(|p| p.cleared += dropped as u32);
        // Ladder bootstrap (position from the stream): a rescue pulse delivers
        // no start edges, so the cursor is provably still -- the one
        // sanctioned cursor read.
        let cursor = self.ring.cursor();
        self.framer.resync(cursor);
        self.chain.reset();
        self.burst.abort();
        self.tx.abort();
        // A dropped pending frame's staged table effect is reclaimed by the
        // dispatcher's auto-revert on the next dispatch.
        self.pending = None;
        self.reply_record = None;
        self.unringed = None;
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
