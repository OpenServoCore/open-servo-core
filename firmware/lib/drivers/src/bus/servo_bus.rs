//! osc-native transport composite (`docs/osc-native-protocol.md`, driver-pattern
//! §4, §5.4). Routes break/deadline/TX events between the three sub-drivers
//! (`Framer`, `Chain`, `TxEngine`), owns the one hardware CRC engine both TX
//! generation and RX validation share, and muxes their deadlines onto the
//! single tick-compare.

use osc_core::traits::{Dispatch, Dispatched, Reply, Request, RequestCtx, SendError, Status};
use osc_core::{BaudRate, BootMode};
use osc_protocol::wire::{Id, Inst, Opcode, ResultCode};

use super::chain::{Chain, ChainOut};
use super::decode::{Decoded, decode};
use super::frame_view;
use super::framer::{FrameSpan, Framer, FramerOut};
use super::handoff::{DispatchJob, Handoff, ReplyRecord};
use super::tx::{TxEngine, TxOut};
use crate::traits::bus::{
    CrcEngine, Deadline, DispatchWake, Lane, LineSense, Providers, RxRing, TxWire, UsartBaud,
    tick_reached,
};

/// µs-per-byte numerator: 10 bit-times/byte × 1e6 µs/s. `tpb = TICKS_PER_US ×
/// this / baud` stays within u32 for all four operational rates.
const BYTE_TIME_NUMERATOR: u32 = 10_000_000;

/// §9.1: an ordinary break has risen by FE-ISR entry [F5]; a still-low line
/// this many µs later is a rescue pulse, not a frame delimiter.
const RESCUE_CONFIRM_US: u32 = 100;

/// Slack on the reclaim-suspension frame allowance (§6): covers the snooper's
/// deadline-B margin on the predecessor's frame end.
const FRAME_ALLOWANCE_SLACK_BYTES: u32 = 8;

/// Deadline-B slack past the packet-end estimate: fixed µs, not byte-times —
/// it covers ISR-entry + settle latencies (silicon-time, baud-independent);
/// the wire-proportional drift term lives in the estimate itself.
const END_SLACK_US: u32 = 5;

/// Bound on same-wake deadline draining. Generous: one frame consumes at most
/// a handful of due slots per wake (header lock, covered, end, chain, rescue);
/// anything past the bound falls out to `arm_deadline`, whose pend-on-past
/// contract re-enters the ISR rather than losing the slot.
const DEADLINE_DRAIN_MAX: u32 = 8;

/// Backlog frames resolved per wake before yielding via pend-on-past (A2):
/// bounds one wake's dispatch work without ever dropping — the ring holds
/// the rest.
const FRAMES_PER_WAKE: u32 = 16;

/// Transport health counters the chip publishes into the telemetry region
/// (§5.3 layer 1: dropped frames are counted, never answered).
pub struct LinkDiag {
    pub crc_fail_count: u32,
    pub framing_drop_count: u32,
}

/// Instruction class, readable from the INST byte at the covered checkpoint:
/// where dispatch runs, and when the CRC verdict is polled (§6 A3(b)).
enum Class {
    /// PING/READ/GREAD — side-effect-free, wire effect only: dispatch at
    /// HIGH; the frame-end verdict gates SEND/DON'T-SEND of the staged reply.
    Wire,
    /// WRITE/GWRITE — table effect (± wire effect): staged through the LOW
    /// consumer; the verdict at adoption gates COMMIT/REVERT of the staged
    /// write plus SEND/DON'T-SEND of the recorded reply.
    Table,
    /// COMMIT/MGMT (and anything undecodable) — effects cannot stage: the
    /// CRC verdict comes first, then dispatch (rare, short frames).
    VerdictFirst,
}

/// A wire-class frame dispatched at HIGH ahead of its CRC verdict — the
/// reply (if any) sits staged in the TX engine until the frame end verifies.
struct WirePending {
    anchor: u16,
    footprint: u16,
    packet_end: u32,
    slot: u8,
    /// Wire effect staged: a reply awaits the send/don't-send verdict.
    staged: bool,
    /// Table effect staged in the dispatcher (defensive — wire-class ops
    /// never stage one; the verdict debt is honored regardless).
    table: bool,
}

/// A table-class frame published to the LOW consumer ahead of its verdict.
/// The verdict fires once BOTH halves are in — the frame end and the
/// consumer's record — in either order (re-derived state, immune to wake
/// interleaving).
struct TablePending {
    anchor: u16,
    footprint: u16,
    ended: bool,
    record: Option<ReplyRecord>,
}

/// A frame dispatched before its CRC verdict — the spine (§4): dispatch
/// always runs under the CRC's own latency; the verdict gates EFFECTS,
/// never work.
enum Pending {
    Wire(WirePending),
    Table(TablePending),
}

pub struct ServoBus<P: Providers> {
    framer: Framer,
    chain: Chain,
    tx: TxEngine<P::Tx>,
    crc: P::Crc,
    ring: P::Ring,
    deadline: P::Deadline,
    baud: P::Baud,
    line: P::Line,
    wake: P::Wake,
    // The one-slot LOW-dispatch handoff (A3(b)); shared with the
    // DispatchConsumer that runs in the LOW vectors.
    handoff: &'static Handoff,
    // A rescue landed while a job/record was in flight: the pre-rescue
    // dispatch's reply (and config records) are stale — adoption discards
    // them, mirroring the abort+reset every other rescue path performs.
    discard_record: bool,
    id: u8,
    rate: BaudRate,
    tpb: u32,
    response_deadline_us: u16,
    pending_id: Option<u8>,
    pending_baud: Option<BaudRate>,
    pending_reboot: Option<BootMode>,
    crc_fails: u32,
    // The one frame dispatched ahead of its CRC verdict (§4: the spine).
    // Backpressure bounds it to one: a wire-class pending frame IS the
    // frontier, and a table-class pending frame holds the framer through the
    // handoff slot — so the single staging slot and the single CRC
    // accumulator are never contended.
    pending: Option<Pending>,
    // Deadline mux (§4.1/§6/§9.1): the soonest live slot arms the compare.
    framer_at: Option<u32>,
    chain_at: Option<u32>,
    rescue_at: Option<u32>,
    // Ring cursor at rescue arm: bytes ringed since mean in-flight traffic,
    // not a held-low pulse (a break of any length is one FE, no data — F3).
    rescue_cursor: u16,
}

/// Ticks per byte-time at `rate` on the transport clock.
fn tpb_for<P: Providers>(rate: BaudRate) -> u32 {
    <P::Deadline as Deadline>::TICKS_PER_US * BYTE_TIME_NUMERATOR / rate.as_hz()
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
        line: P::Line,
        wake: P::Wake,
        handoff: &'static Handoff,
        id: u8,
        rate: BaudRate,
        response_deadline_us: u16,
    ) -> Self {
        baud.apply(rate);
        Self {
            framer: Framer::new(END_SLACK_US * <P::Deadline as Deadline>::TICKS_PER_US),
            chain: Chain::new(),
            tx: TxEngine::new(tx),
            crc,
            ring,
            deadline,
            baud,
            line,
            wake,
            handoff,
            discard_record: false,
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
            rescue_at: None,
            rescue_cursor: 0,
        }
    }

    /// USART framing-error ISR: a break (or mid-frame garble) landed. A1:
    /// record the tick and resolve from ring DATA — the FE never carries
    /// position, so a delayed or coalesced entry costs nothing (any frames
    /// completed meanwhile resolve on the fast path right here).
    pub fn on_break<D: Dispatch>(&mut self, d: &mut D) {
        let now = self.deadline.now();
        self.framer.on_fe(now);
        self.drive_framer(d);
        // Wire safety: a staged, not-yet-streaming reply must never fire
        // into the host's NEXT frame. But an FE alone doesn't mean the host
        // moved on — lagged deliveries routinely resolve the reply's own
        // frame (or reach its covered checkpoint) inside this very wake
        // (silicon event-trace 2026-07-08: COVERED→KILL 2 µs apart, then
        // verify armed the chain over the emptied engine — ghost trigger,
        // silent no-reply). Positional truth (A2) decides: kill only when
        // ringed bytes FOLLOW the reply's own frame. A pending frame is
        // mid-verdict by definition (its CRC gate reclaims the reply if this
        // break garbled the frame); a Waiting chain expects predecessor
        // breaks and only suspends its reclaim window (§6).
        if self.tx.staged()
            && !self.chain.waiting()
            && self.pending.is_none()
            && !self.framer.caught_up(self.ring.cursor())
        {
            self.tx.abort();
            self.chain.reset();
            self.chain_at = None;
        }
        // §6: a break while we hold a staged chain slot means the predecessor
        // is alive — suspend its reclaim window while the frame plays out.
        let out = self.chain.on_break_observed(now);
        self.route_chain(out);
        // §9.1 rescue candidacy: confirm a held-low line ~100 µs on. The
        // line often still reads low at an ordinary break's FE entry (the
        // break's last bit), so this arms on most frames — the confirm's
        // cursor-progress gate is what keeps it from firing on traffic.
        if self.line.is_low() {
            let at = now.wrapping_add(RESCUE_CONFIRM_US * <P::Deadline as Deadline>::TICKS_PER_US);
            self.rescue_at = Some(at);
            self.rescue_cursor = self.ring.cursor();
        }
        self.arm_deadline();
    }

    /// Tick-compare ISR: one or more muxed deadlines are due. Every slot a
    /// handler body overruns (front-loaded dispatch inside the covered window
    /// routinely overruns `end_due`) is drained in this same invocation — a
    /// pend→exit→re-enter round trip costs ~10 µs of turnaround.
    pub fn on_deadline<D: Dispatch>(&mut self, d: &mut D) {
        for _ in 0..DEADLINE_DRAIN_MAX {
            let now = self.deadline.now();
            let mut progressed = false;
            // A completed LOW dispatch awaits adoption (the consumer's
            // reply-ready wake pends this vector). Adopting frees the slot,
            // so resume the held framer in the same wake.
            if let Some(rec) = self.handoff.take_reply() {
                progressed = true;
                self.on_record(rec, d);
                self.drive_framer(d);
            }
            if due(now, self.framer_at) {
                progressed = true;
                self.framer_at = None;
                self.drive_framer(d);
            }
            if due(now, self.chain_at) {
                progressed = true;
                self.chain_at = None;
                let out = self.chain.on_deadline(now);
                self.route_chain(out);
            }
            if due(now, self.rescue_at) {
                progressed = true;
                self.rescue_at = None;
                self.try_rescue();
            }
            if !progressed {
                break;
            }
        }
        self.arm_deadline();
    }

    /// TX DMA arm-complete ISR: stream the next arm, or apply deferred config
    /// once the whole reply has drained (§4.2) — the ack always leaves at the
    /// old id/baud, the change lands after.
    pub fn on_tx_complete(&mut self) {
        if self.tx.on_arm_complete(&mut self.crc) == TxOut::Released {
            if let Some(id) = self.pending_id.take() {
                self.id = id;
            }
            if let Some(baud) = self.pending_baud.take() {
                self.baud.apply(baud);
                self.rate = baud;
                self.tpb = tpb_for::<P>(self.rate);
            }
            // A pending reboot waits for the main loop's `take_reboot`.
        }
    }

    /// Main-loop poll for a deferred reboot honored after the ack drained.
    pub fn take_reboot(&mut self) -> Option<BootMode> {
        self.pending_reboot.take()
    }

    pub fn diag(&self) -> LinkDiag {
        LinkDiag {
            crc_fail_count: self.crc_fails,
            framing_drop_count: self.framer.drops(),
        }
    }

    fn t_turn(&self) -> u32 {
        super::T_TURN_BYTES * self.tpb
    }

    fn reclaim(&self) -> u32 {
        self.response_deadline_us as u32 * <P::Deadline as Deadline>::TICKS_PER_US
    }

    /// How long an observed predecessor break suspends its reclaim window:
    /// the largest legal frame plus the snooper's own end-detection slack.
    fn frame_allowance(&self) -> u32 {
        (super::FRAME_MAX as u32 + FRAME_ALLOWANCE_SLACK_BYTES) * self.tpb
    }

    /// Arm the compare at the soonest live slot, or cancel if none. A slot
    /// already reached counts as due NOW, not as a full wrap away — an ISR
    /// body that overruns a pending deadline (front-loaded dispatch inside
    /// the covered window does, routinely) must pend it, not push it behind
    /// every future slot (bench signature: deadline B riding the +100 µs
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
        for at in [self.framer_at, self.chain_at, self.rescue_at]
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

    /// Drive the resolver as far as ring DATA allows (A2): complete backlog
    /// frames process in-line with zero clock involvement; the frontier arms
    /// a deadline and returns. Bounded per wake — past the bound the framer
    /// slot re-arms at `now` (pend-on-past re-entry) so the other mux slots
    /// are never starved by a deep backlog.
    fn drive_framer<D: Dispatch>(&mut self, d: &mut D) {
        for _ in 0..FRAMES_PER_WAKE {
            // Backpressure (A3(b)): while a job or its record is in flight,
            // the framer holds position and the ring absorbs the backlog
            // (A2). The consumer's reply-ready wake resumes the walk;
            // framer_at is cleared so a stale estimate can't pend-on-past
            // spin against the held resolver.
            if !self.handoff.idle() {
                self.framer_at = None;
                return;
            }
            let now = self.deadline.now();
            let out = self
                .framer
                .resolve(self.ring.bytes(), self.ring.cursor(), now, self.tpb);
            match out {
                FramerOut::None => {
                    self.framer_at = None;
                    return;
                }
                FramerOut::Wait(t) => {
                    self.framer_at = Some(t);
                    return;
                }
                FramerOut::Covered { span, end_due } => {
                    self.framer_at = Some(end_due);
                    self.route_frame(span, false, d);
                    return;
                }
                FramerOut::Frame(span) => {
                    self.on_frame_end(span, d);
                }
            }
        }
        self.framer_at = Some(self.deadline.now());
    }

    fn route_chain(&mut self, out: ChainOut) {
        match out {
            ChainOut::None => {}
            ChainOut::Wait(t) => self.chain_at = Some(t),
            ChainOut::Trigger { predecessor_silent } => {
                let over = predecessor_silent.then_some(ResultCode::PredecessorSilent);
                self.tx.trigger(&mut self.crc, over);
            }
        }
    }

    fn try_rescue(&mut self) {
        // Our own reply is on the wire — a low sample here reads our own
        // data bits, not a host rescue pulse (bench-observed at 1M: the
        // recheck armed by the request's break lands mid-reply and the
        // phantom confirm aborts the reply + drops the rate mid-frame). A
        // real rescue pulse re-arms via its own FE once the wire is back.
        if self.tx.streaming() {
            return;
        }
        // Bytes ringed since the arm → an instruction is in flight and the
        // low sample is its data bits, not a pulse (bench-observed at 1M: a
        // 12-byte zero-payload WRITE keeps the line low at +100 µs and the
        // confirm dropped the rate mid-instruction). A held-low line
        // delivers no start edges, so a real pulse leaves the cursor put.
        if self.ring.cursor() != self.rescue_cursor {
            return;
        }
        // Line risen → it was an ordinary break, not a rescue pulse.
        if !self.line.is_low() {
            return;
        }
        // §9.1: volatile rate switch — the config register is untouched.
        self.baud.apply(BaudRate::B500000);
        self.rate = BaudRate::B500000;
        self.tpb = tpb_for::<P>(self.rate);
        // Ladder bootstrap (A2): a rescue pulse delivers no start edges, so
        // the cursor is provably still — the one sanctioned cursor read.
        let cursor = self.ring.cursor();
        self.framer.resync(cursor);
        self.chain.reset();
        self.tx.abort();
        // A dropped pending frame's staged table effect is reclaimed by the
        // dispatcher's auto-revert on the next dispatch.
        self.pending = None;
        self.framer_at = None;
        self.chain_at = None;
        // A job (or its record) from a pre-rescue frame is stale; the slot
        // can't be yanked from a mid-dispatch consumer, so adoption discards
        // it instead.
        if !self.handoff.idle() {
            self.discard_record = true;
        }
    }

    /// Instruction class from the ring INST byte — the routing decision, and
    /// the same byte [`decode`] reads at dispatch, so the two can never
    /// disagree (the soundness of verdict-first: a table-class publish can
    /// never decode into a COMMIT).
    fn classify(&self, anchor: u16) -> Class {
        match self.ring_inst(anchor).opcode() {
            Some(Opcode::Ping | Opcode::Read | Opcode::Gread) => Class::Wire,
            Some(Opcode::Write | Opcode::Gwrite) => Class::Table,
            _ => Class::VerdictFirst,
        }
    }

    /// Frame end (deadline B): a pending frame gets its verdict half; anything
    /// else is a complete frame entering the spine ([`Self::route_frame`]).
    fn on_frame_end<D: Dispatch>(&mut self, span: FrameSpan, d: &mut D) {
        match self.pending.take() {
            Some(Pending::Wire(w)) if w.anchor == span.anchor && w.footprint == span.footprint => {
                self.verify(w, d);
            }
            Some(Pending::Table(mut t))
                if t.anchor == span.anchor && t.footprint == span.footprint =>
            {
                t.ended = true;
                self.pending = Some(Pending::Table(t));
                self.try_resolve_table(d);
            }
            Some(_) => {
                // Defensive: a pending frame that isn't this one — drop it. Any
                // dangling staged write is reclaimed by the dispatcher
                // auto-revert on the next dispatch.
                self.drop_staged();
                self.route_frame(span, true, d);
            }
            None => self.route_frame(span, true, d),
        }
    }

    /// The spine: dispatch a frame ahead of its CRC verdict, routed by class.
    /// `complete` distinguishes a whole frame (backlog / fast path — every
    /// byte incl. the CRC is ringed, so the verdict fires now) from a frontier
    /// frame at its covered checkpoint (the two CRC bytes still inbound — the
    /// verdict is deferred to the frame end). Wire class dispatches inline at
    /// HIGH with the CRC feed chewing underneath; table class publishes to the
    /// LOW consumer so hop + decode + dispatch overlap the frame's own wire
    /// tail (decode needs only covered bytes); verdict-first (COMMIT/MGMT,
    /// effects unstageable) and any frame overlapping a live reply pipeline
    /// check the CRC first, then dispatch. A frontier frame that lands in the
    /// verdict-first path just defers — its frame end arrives here `complete`.
    fn route_frame<D: Dispatch>(&mut self, span: FrameSpan, complete: bool, d: &mut D) {
        let anchor = span.anchor;
        let footprint = span.footprint;
        let packet_end = span.packet_end;
        // Status frames only advance the snoop chain (§6) — framing-level
        // truth, NO validation: the chain consumes nothing from the body, and
        // skipping the CRC keeps the snapshot buffer free while our own reply
        // streams from it. A frontier status defers to its frame end.
        if self.ring_inst(anchor).is_status() {
            if complete {
                let out = self.chain.on_status_end(packet_end);
                self.route_chain(out);
            }
            return;
        }
        // The spine runs only from an idle reply pipeline: superseding a live
        // chain or staged reply belongs AFTER the CRC gate (a garbled frame
        // must touch nothing, §5.3 L1), so those overlaps fall to verdict-first
        // below. The guard also keeps the CRC engine free through a pending
        // window — chain/tx activate only at a verdict, so no trigger reset
        // can clobber a fed span.
        if !self.chain.active() && !self.tx.busy() {
            match self.classify(anchor) {
                Class::Wire => {
                    // Feed first so the engine chews the covered span under the
                    // dispatch body; the verdict then finds it settled.
                    self.crc_feed(anchor, footprint);
                    let (staged, slot, table) =
                        match self.dispatch_decoded(anchor, footprint, |req, ctx, h| {
                            d.dispatch(req, ctx, h)
                        }) {
                            Some((staged, slot, out)) => {
                                (staged, slot, matches!(out, Dispatched::Pending))
                            }
                            // Skip (a group op not listing us): a bare verdict
                            // moves the ladder (complete); the frame end does it
                            // for a frontier.
                            None if complete => (false, 0, false),
                            None => return,
                        };
                    let w = WirePending {
                        anchor,
                        footprint,
                        packet_end,
                        slot,
                        staged,
                        table,
                    };
                    if complete {
                        self.verify(w, d);
                    } else {
                        self.pending = Some(Pending::Wire(w));
                    }
                    return;
                }
                // Unicast to another servo never occupies the slot; group ops
                // ride broadcast and resolve their id-lists at LOW.
                Class::Table if self.ring_id(anchor).addresses(Id::new(self.id)) => {
                    self.crc_feed(anchor, footprint);
                    self.publish_job(anchor, footprint, packet_end, self.job_lane(complete));
                    self.pending = Some(Pending::Table(TablePending {
                        anchor,
                        footprint,
                        ended: complete,
                        record: None,
                    }));
                    return;
                }
                // VerdictFirst, or a table frame addressed elsewhere: fall
                // through to the CRC-first path.
                _ => {}
            }
        }
        // Verdict-first (only meaningful complete — a frontier defers): the CRC
        // is checked before any effect. COMMIT/MGMT (unstageable), foreign
        // frames, and frames overlapping a live reply pipeline. A fail drops
        // silently (§5.3 L1), the hunt resuming one byte in (§3.3).
        if !complete {
            return;
        }
        if !self.crc_ok(anchor, footprint) {
            if !self.framer.probing() {
                self.crc_fails = self.crc_fails.wrapping_add(1);
            }
            let len = self.ring.bytes().len();
            self.framer.on_frame_rejected(anchor, len);
            return;
        }
        self.framer.on_frame_verified();
        // A fresh instruction supersedes any stale, not-yet-streaming reply —
        // post-verdict, so a garbled frame touched nothing.
        self.chain.reset();
        self.chain_at = None;
        if self.tx.staged() {
            self.tx.abort();
        }
        // Publish for LOW dispatch (A3(b)); the record adopts directly — the
        // CRC already passed, so a staged effect commits at adoption.
        if !self.ring_id(anchor).addresses(Id::new(self.id)) {
            return;
        }
        self.publish_job(anchor, footprint, packet_end, self.job_lane(true));
    }

    /// Lane per the locked interleave policy (§6 A3): a frontier is the live
    /// edge (beats the kernel); a complete frame is live only if it is the
    /// last one ringed, else a backlog frame that lets a kernel tick slot in.
    fn job_lane(&self, complete: bool) -> Lane {
        if !complete || self.framer.caught_up(self.ring.cursor()) {
            Lane::Live
        } else {
            Lane::Queued
        }
    }

    /// Hand a decoded frame to the LOW consumer and wake it.
    fn publish_job(&mut self, anchor: u16, footprint: u16, packet_end: u32, lane: Lane) {
        self.handoff.publish(DispatchJob {
            anchor,
            footprint,
            packet_end,
            id: self.id,
        });
        self.wake.job_ready(lane);
    }

    /// Verify a wire-class pending frame's CRC and resolve the verdict.
    /// Pass → sequence the staged reply (SEND); fail (or spin miss) → drop
    /// it (DON'T-SEND), count, and rewind the ladder (§5.3 L1). The
    /// defensive `table` debt is honored either way.
    fn verify<D: Dispatch>(&mut self, w: WirePending, d: &mut D) {
        if !self.crc_verify(w.anchor, w.footprint) {
            if w.table {
                d.revert();
            }
            self.drop_staged();
            if !self.framer.probing() {
                self.crc_fails = self.crc_fails.wrapping_add(1);
            }
            let len = self.ring.bytes().len();
            self.framer.on_frame_rejected(w.anchor, len);
            return;
        }
        self.framer.on_frame_verified();
        if w.table {
            let mut handle = self.reply_handle();
            d.commit(&mut handle);
        }
        // Sequence from the ENGINE's state, not the recorded flag: any path
        // that reclaimed the staged reply between dispatch and here would
        // otherwise arm the chain over an empty engine (ghost trigger).
        if w.staged && self.tx.staged() {
            let t_turn = self.t_turn();
            let reclaim = self.reclaim();
            let allowance = self.frame_allowance();
            // T_turn is a wire gap measured from the packet end (§7), not
            // from this (later) verify wake — the estimate is the framer's,
            // conservative by the drift adder.
            let out = self
                .chain
                .on_reply_staged(w.slot, w.packet_end, t_turn, reclaim, allowance);
            self.route_chain(out);
        }
    }

    /// The table-class verdict fires when BOTH halves are in — the frame end
    /// and the consumer's record — in either order (re-derived on each
    /// trigger; no wake interleaving can lose it).
    fn try_resolve_table<D: Dispatch>(&mut self, d: &mut D) {
        let ready =
            matches!(&self.pending, Some(Pending::Table(t)) if t.ended && t.record.is_some());
        if !ready {
            return;
        }
        let Some(Pending::Table(t)) = self.pending.take() else {
            return; // unreachable: matched above
        };
        let Some(rec) = t.record else {
            return; // unreachable: matched above
        };
        self.verify_table(t.anchor, t.footprint, rec, d);
    }

    /// Resolve a table-class frame's verdict. The CRC was fed at publish and
    /// chewed under the hop, so the poll finds it settled. Pass → commit the
    /// staged table effect and adopt the record (stage + sequence the
    /// reply); fail → revert + don't-send: the live table is byte-identical,
    /// nothing reaches the wire, and the ladder rewinds exactly as an
    /// un-dispatched reject would — the hunt just starts one hop later, on
    /// corrupt frames only, inside the host's retry contract.
    fn verify_table<D: Dispatch>(
        &mut self,
        anchor: u16,
        footprint: u16,
        rec: ReplyRecord,
        d: &mut D,
    ) {
        if !self.crc_verify(anchor, footprint) {
            if rec.pending {
                d.revert();
            }
            if !self.framer.probing() {
                self.crc_fails = self.crc_fails.wrapping_add(1);
            }
            let len = self.ring.bytes().len();
            self.framer.on_frame_rejected(anchor, len);
            return;
        }
        self.framer.on_frame_verified();
        self.adopt(rec, d);
    }

    /// Drop a not-yet-streaming staged reply and any chain sequencing it began.
    fn drop_staged(&mut self) {
        if self.tx.staged() {
            self.tx.abort();
        }
        self.chain.reset();
        self.chain_at = None;
    }

    /// A completed LOW dispatch surfaced its record. A table-class pending
    /// frame gets its record half (the verdict resolves once the frame has
    /// also ended); a verdict-first job adopts directly — its CRC passed
    /// before publish.
    fn on_record<D: Dispatch>(&mut self, rec: ReplyRecord, d: &mut D) {
        if core::mem::take(&mut self.discard_record) {
            // Rescue landed while the job was in flight: the record is
            // stale, and `pending` was already cleared. A staged table
            // effect from it is reclaimed by the dispatcher's auto-revert
            // on the next dispatch.
            return;
        }
        match self.pending.take() {
            Some(Pending::Table(mut t)) => {
                t.record = Some(rec);
                self.pending = Some(Pending::Table(t));
                self.try_resolve_table(d);
            }
            // A wire-class pending frame cannot coexist with an outstanding
            // job (single slot, publish paths are framer-gated) — defensive:
            // keep it and adopt the record as verdict-first.
            other => {
                self.pending = other;
                self.adopt(rec, d);
            }
        }
    }

    /// Adopt a completed LOW dispatch whose verdict has passed: honor the
    /// record's table-effect debt (commit), apply its deferred-config
    /// records, stage the recorded reply into the TX engine, and sequence
    /// the chain from the frame's packet end — elastically, when this wake
    /// runs past the T_turn grid (§6 A3(b): misses degrade, they do not
    /// fail).
    fn adopt<D: Dispatch>(&mut self, rec: ReplyRecord, d: &mut D) {
        if rec.pending {
            // Apply the staged write into the table + fire its hooks
            // (baud/id stage through this handle) before sequencing the ack.
            let mut handle = self.reply_handle();
            d.commit(&mut handle);
        }
        if let Some(us) = rec.response_deadline_us {
            self.response_deadline_us = us;
        }
        if let Some(id) = rec.id_change {
            self.pending_id = Some(id);
        }
        if let Some(baud) = rec.baud_change {
            self.pending_baud = Some(baud);
        }
        if let Some(mode) = rec.reboot {
            self.pending_reboot = Some(mode);
        }
        if !rec.staged {
            return;
        }
        // Positional truth, as in the FE-kill gate: a unicast reply adopted
        // after newer bytes ringed would trigger into the host's next frame
        // (the zero-gap collision the break kill can no longer catch — the
        // next break may predate this adoption). A chain slot k>0 expects
        // wire traffic (predecessor statuses) and sequences through it.
        if rec.slot == 0 && !self.framer.caught_up(self.ring.cursor()) {
            return;
        }
        // SAFETY: §4.2 zero-copy contract — every non-empty core reply
        // payload references control-table storage, stable for the exchange
        // (the same contract the engine's external arms rely on); staging
        // snapshots it exactly as an inline dispatch would.
        let data = if rec.payload_len == 0 {
            &[][..]
        } else {
            unsafe { core::slice::from_raw_parts(rec.payload, rec.payload_len as usize) }
        };
        if self
            .tx
            .stage(&mut self.crc, self.id, rec.result, rec.alert, data)
            .is_err()
        {
            // Busy (a prior reply still draining — the host broke its
            // one-outstanding contract) or a sizing overflow: drop, the
            // host's timeout+retry closes the loop (§5.3).
            return;
        }
        let out = self.chain.on_reply_staged(
            rec.slot,
            rec.packet_end,
            self.t_turn(),
            self.reclaim(),
            self.frame_allowance(),
        );
        self.route_chain(out);
    }

    /// View the frame as up to two ring segments (one span unless it wraps the
    /// seam), decode, and run `f` over disjoint borrows (driver-pattern §4.3):
    /// the decoded request borrows the ring while `f` stages a reply through the
    /// [`ReplyHandle`]. Returns `(staged, slot, f-result)`, or `None` for a frame
    /// that isn't ours.
    fn dispatch_decoded<T>(
        &mut self,
        anchor: u16,
        footprint: u16,
        f: impl FnOnce(Request<'_>, RequestCtx, &mut ReplyHandle<'_, P::Tx, P::Crc>) -> T,
    ) -> Option<(bool, u8, T)> {
        let frame = frame_view(self.ring.bytes(), anchor, footprint);
        let (req, ctx, slot) = match decode(frame, self.id) {
            Decoded::Own(req, ctx, slot) => (req, ctx, slot),
            _ => return None,
        };
        // Disjoint field borrows: `frame`/`req` hold `&self.ring`; the handle
        // takes the reply-staging fields mutably.
        let mut handle = ReplyHandle {
            tx: &mut self.tx,
            crc: &mut self.crc,
            id: self.id,
            pending_id: &mut self.pending_id,
            pending_baud: &mut self.pending_baud,
            pending_reboot: &mut self.pending_reboot,
            response_deadline_us: &mut self.response_deadline_us,
            staged: false,
        };
        let out = f(req, ctx, &mut handle);
        Some((handle.staged, slot, out))
    }

    /// A reply surface over the deferred-config fields, with no request decode —
    /// used at verdict commit, where hooks stage baud/id but no frame is
    /// re-parsed (the ring isn't borrowed here).
    fn reply_handle(&mut self) -> ReplyHandle<'_, P::Tx, P::Crc> {
        ReplyHandle {
            tx: &mut self.tx,
            crc: &mut self.crc,
            id: self.id,
            pending_id: &mut self.pending_id,
            pending_baud: &mut self.pending_baud,
            pending_reboot: &mut self.pending_reboot,
            response_deadline_us: &mut self.response_deadline_us,
            staged: false,
        }
    }

    /// Snapshot the covered span (linearizing a ring-wrap split) and feed its
    /// even bulk into the CRC engine, no result poll. The feed starts at the
    /// anchor regardless of parity — the break's `0x00` leads as a CRC no-op
    /// and the snapshot buffer is even-based (§3.2). Linearization also makes
    /// segment-length parity irrelevant: the halfword feed sees one
    /// contiguous span. A trailing odd byte is left for the fold at verify.
    fn crc_feed(&mut self, anchor: u16, footprint: u16) {
        let ring = self.ring.bytes();
        let len = ring.len();
        let anchor = anchor as usize;
        let covered = footprint as usize - 2;
        let bulk = covered - (covered & 1);
        self.crc.reset();
        let base = if anchor + covered <= len {
            self.crc.snapshot(0, &ring[anchor..anchor + covered])
        } else {
            let first = len - anchor;
            let base = self.crc.snapshot(0, &ring[anchor..len]);
            self.crc.snapshot(first as u16, &ring[..covered - first]);
            base
        };
        // SAFETY: `base` targets the engine's snapshot buffer, stable and
        // sized for a max covered span (trait contract); the copy cannot be
        // overtaken by the feed (transfer ordering).
        self.crc
            .feed(unsafe { core::slice::from_raw_parts(base, bulk) });
    }

    /// The covered byte the even-bulk feed left behind, if the span was odd.
    fn crc_tail(&self, anchor: u16, footprint: u16) -> Option<u8> {
        let ring = self.ring.bytes();
        let len = ring.len();
        let anchor = anchor as usize;
        let end = anchor + footprint as usize - 2;
        if (end - anchor) & 1 == 1 {
            Some(ring[(end - 1) % len])
        } else {
            None
        }
    }

    /// Poll the CRC result, fold the trailing odd byte if the feed left one
    /// (§3.2), and compare against the little-endian wire CRC at the
    /// covered-span end. A spin miss counts as a fail — indistinguishable from
    /// a bad frame, and never wedges the wire. Requires a prior
    /// [`Self::crc_feed`] of the same span.
    fn crc_verify(&mut self, anchor: u16, footprint: u16) -> bool {
        let ring = self.ring.bytes();
        let len = ring.len();
        let covered = footprint as usize - 2;
        let end = anchor as usize + covered;
        let wire = u16::from_le_bytes([ring[end % len], ring[(end + 1) % len]]);
        let tail = self.crc_tail(anchor, footprint);
        let mut budget = super::SPIN_PER_BYTE * covered as u32;
        loop {
            if let Some(v) = self.crc.result() {
                let v = match tail {
                    Some(b) => osc_protocol::crc::osc_crc_continue(v, &[b]),
                    None => v,
                };
                return v == wire;
            }
            if budget == 0 {
                return false;
            }
            budget -= 1;
            core::hint::spin_loop();
        }
    }

    /// Feed then verify the covered span against the wire CRC (the full,
    /// non-front-loaded path).
    fn crc_ok(&mut self, anchor: u16, footprint: u16) -> bool {
        if self.ring.bytes().is_empty() || (footprint as usize) < 2 {
            return false;
        }
        self.crc_feed(anchor, footprint);
        self.crc_verify(anchor, footprint)
    }

    /// The INST byte, 3 slots past the anchor (`[0x00][ID][LEN][INST]`).
    fn ring_inst(&self, anchor: u16) -> Inst {
        let ring = self.ring.bytes();
        let len = ring.len();
        Inst(ring[(anchor as usize + 3) % len])
    }

    /// The frame ID byte, 1 slot past the anchor.
    fn ring_id(&self, anchor: u16) -> Id {
        let ring = self.ring.bytes();
        let len = ring.len();
        Id::new(ring[(anchor as usize + 1) % len])
    }
}

/// Reply surface over disjoint `ServoBus` fields (driver-pattern §4.3): the
/// decoded request still borrows the ring while the dispatcher stages the
/// reply into the TX engine and the deferred-config fields.
struct ReplyHandle<'a, W: TxWire, C: CrcEngine> {
    tx: &'a mut TxEngine<W>,
    crc: &'a mut C,
    id: u8,
    pending_id: &'a mut Option<u8>,
    pending_baud: &'a mut Option<BaudRate>,
    pending_reboot: &'a mut Option<BootMode>,
    response_deadline_us: &'a mut u16,
    staged: bool,
}

impl<W: TxWire, C: CrcEngine> Reply for ReplyHandle<'_, W, C> {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), SendError> {
        let r = self
            .tx
            .stage(self.crc, self.id, status.result, status.alert, status.data);
        if r.is_ok() {
            self.staged = true;
        }
        r
    }

    fn stage_id(&mut self, id: u8) {
        *self.pending_id = Some(id);
    }

    fn stage_baud(&mut self, baud: BaudRate) {
        *self.pending_baud = Some(baud);
    }

    fn set_response_deadline(&mut self, us: u16) {
        *self.response_deadline_us = us;
    }

    fn stage_reboot(&mut self, mode: BootMode) {
        *self.pending_reboot = Some(mode);
    }
}

#[cfg(test)]
mod tests;
