//! osc-native transport composite (`docs/osc-native-protocol.md`, driver-pattern
//! §4, §5.4). Routes break/deadline/TX events between the three sub-drivers
//! (`Framer`, `Chain`, `TxEngine`), owns the one hardware CRC engine both TX
//! generation and RX validation share, and muxes their deadlines onto the
//! single tick-compare.

use osc_core::traits::{Dispatch, Dispatched, Reply, Request, RequestCtx, SendError, Status};
use osc_core::{BaudRate, BootMode};
use osc_protocol::wire::{Inst, Opcode, ResultCode};

use super::chain::{Chain, ChainOut};
use super::decode::{Decoded, decode};
use super::frame_view;
use super::framer::{FrameSpan, Framer, FramerOut};
use super::tx::{TxEngine, TxOut};
use crate::traits::bus::{
    CrcEngine, Deadline, LineSense, Providers, RxRing, TxWire, UsartBaud, tick_reached,
};

/// µs-per-byte numerator: 10 bit-times/byte × 1e6 µs/s. `tpb = TICKS_PER_US ×
/// this / baud` stays within u32 for all four operational rates.
const BYTE_TIME_NUMERATOR: u32 = 10_000_000;

/// §9.1: an ordinary break has risen by FE-ISR entry [F5]; a still-low line
/// this many µs later is a rescue candidate, not a frame delimiter.
const RESCUE_CONFIRM_US: u32 = 100;

/// Second rescue sample, this many byte-times past a passing first one (a
/// hair over one, shift-friendly). One low sample can alias a data byte's
/// low bits while a host TX stall keeps the cursor frozen across the
/// confirm (bench-observed at 1M: a ~95 µs burst bubble right after a
/// break put the +100 µs sample inside the next byte's start bit —
/// phantom rescue, servo wedged at 0.5M under a 1M host). Data cannot
/// hold the line low a whole byte-time without completing a byte, and a
/// completed byte rings and moves the cursor — so a second low sample a
/// byte-time later with the cursor still frozen is a dominant low by
/// UART framing itself, at any host baud.
const RESCUE_RECONFIRM_NUM: u32 = 9;
const RESCUE_RECONFIRM_SHIFT: u32 = 3;

/// Slack on the reclaim-suspension frame allowance (§6): covers the snooper's
/// deadline-B margin on the predecessor's frame end.
const FRAME_ALLOWANCE_SLACK_BYTES: u32 = 8;

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

/// A frame dispatched ahead of its CRC verdict — the spine (§4): dispatch
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
    line: P::Line,
    id: u8,
    rate: BaudRate,
    tpb: u32,
    response_deadline_us: u16,
    pending_id: Option<u8>,
    pending_baud: Option<BaudRate>,
    pending_reboot: Option<BootMode>,
    crc_fails: u32,
    // The one frame dispatched ahead of its CRC verdict (§4: the spine).
    // Backpressure bounds it to one: a pending frame IS the frontier, so the
    // single staging slot and the single CRC accumulator are never contended.
    pending: Option<Pending>,
    // Deadline mux (§4.1/§6/§9.1): the soonest live slot arms the compare.
    framer_at: Option<u32>,
    chain_at: Option<u32>,
    rescue_at: Option<u32>,
    // Ring cursor at rescue arm: bytes ringed since mean in-flight traffic,
    // not a held-low pulse (a break of any length is one FE, no data — F3).
    rescue_cursor: u16,
    // A first confirm sample passed; the slot is re-armed one byte-time out
    // for the aliasing-proof second sample (see RESCUE_RECONFIRM_NUM).
    rescue_reconfirm: bool,
}

/// Ticks per byte-time at `rate` on the transport clock. Each arm folds to a
/// literal via `const {}` (TICKS_PER_US is an associated const) — the board
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
        line: P::Line,
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
            line,
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
            rescue_reconfirm: false,
        }
    }

    /// USART framing-error ISR: a break (or mid-frame garble) landed — a
    /// pure wake. The FE carries neither position nor time (A2): the
    /// resolver derives both from ring data, so a delayed or coalesced
    /// entry costs nothing (any frames completed meanwhile resolve on the
    /// fast path right here).
    pub fn on_break<D: Dispatch>(&mut self, d: &mut D) {
        let now = self.deadline.now();
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
            self.rescue_reconfirm = false;
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

    fn reply_gap(&self) -> u32 {
        super::REPLY_GAP_US * <P::Deadline as Deadline>::TICKS_PER_US
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

    fn route_chain(&mut self, mut out: ChainOut) {
        loop {
            match out {
                ChainOut::None => return,
                ChainOut::Wait(t) => {
                    // A wait tick already reached needs no scheduling round
                    // trip: consume the slot in place (high-baud grids and
                    // backlog replies are past-due by sequencing time, and
                    // an expired reclaim window is a reclaim by definition).
                    let now = self.deadline.now();
                    if !tick_reached(now, t) {
                        self.chain_at = Some(t);
                        return;
                    }
                    out = self.chain.on_deadline(now);
                }
                ChainOut::Trigger { predecessor_silent } => {
                    let over = predecessor_silent.then_some(ResultCode::PredecessorSilent);
                    self.tx.trigger(&mut self.crc, over);
                    return;
                }
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
        // One low sample can alias a data bit when a host TX stall froze the
        // cursor across the confirm: take a second sample a byte-time later
        // under the same frozen-cursor requirement — only a dominant low
        // survives both (see RESCUE_RECONFIRM_NUM).
        if !self.rescue_reconfirm {
            self.rescue_reconfirm = true;
            let lead = (self.tpb * RESCUE_RECONFIRM_NUM) >> RESCUE_RECONFIRM_SHIFT;
            self.rescue_at = Some(self.deadline.now().wrapping_add(lead));
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
    }

    /// Verdict-first ops: COMMIT (applies the whole staging buffer), MGMT
    /// (reboots/config), and anything undecodable — their effects can't stage,
    /// so the CRC verdict must come first (the bus checks it, then dispatch
    /// applies directly on that contract). Everything else dispatches ahead of
    /// its verdict, effects gated by it. Read from the same INST byte
    /// [`decode`] reads, so routing and dispatch can never disagree.
    fn verdict_first(&self, anchor: u16) -> bool {
        !matches!(
            self.ring_inst(anchor).opcode(),
            Some(Opcode::Ping | Opcode::Read | Opcode::Gread | Opcode::Write | Opcode::Gwrite)
        )
    }

    /// Frame end (deadline B): a pending frame gets its verdict; anything else
    /// is a complete frame entering the spine ([`Self::route_frame`]).
    fn on_frame_end<D: Dispatch>(&mut self, span: FrameSpan, d: &mut D) {
        match self.pending.take() {
            Some(p) if p.anchor == span.anchor && p.footprint == span.footprint => {
                self.verify(p, d);
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

    /// The spine: dispatch a frame ahead of its CRC verdict when its effects
    /// can stage (§4). `complete` distinguishes a whole frame (backlog / fast
    /// path — every byte incl. the CRC is ringed, so the verdict fires now)
    /// from a frontier frame at its covered checkpoint (the two CRC bytes still
    /// inbound — the verdict is deferred to the frame end). A stageable op
    /// (ping/read/gread/write/gwrite) dispatches inline with the CRC feed
    /// chewing underneath; the verdict gates its effects — SEND/DON'T-SEND of a
    /// staged reply, COMMIT/REVERT of a staged write. Verdict-first ops
    /// (COMMIT/MGMT, effects unstageable) and any frame overlapping a live
    /// reply pipeline check the CRC first, then dispatch. A frontier frame that
    /// lands in the verdict-first path just defers — its frame end arrives here
    /// `complete`.
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
        if !self.chain.active() && !self.tx.busy() && !self.verdict_first(anchor) {
            // Feed first so the engine chews the covered span under the
            // dispatch body; the verdict then finds it settled.
            self.crc_feed(anchor, footprint);
            let (staged, slot, table) =
                match self
                    .dispatch_decoded(anchor, footprint, |req, ctx, h| d.dispatch(req, ctx, h))
                {
                    Some((staged, slot, out)) => (staged, slot, matches!(out, Dispatched::Pending)),
                    // Skip (a group op not listing us): a bare verdict moves the
                    // ladder (complete); the frame end does it for a frontier.
                    None if complete => (false, 0, false),
                    None => return,
                };
            let p = Pending {
                anchor,
                footprint,
                packet_end,
                slot,
                staged,
                table,
            };
            if complete {
                self.verify(p, d);
            } else {
                self.pending = Some(p);
            }
            return;
        }
        // Verdict-first (only meaningful complete — a frontier defers): the CRC
        // is checked before any effect. COMMIT/MGMT (unstageable), and frames
        // overlapping a live reply pipeline. A fail drops silently (§5.3 L1),
        // the hunt resuming one byte in (§3.3).
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
        // Dispatch inline — the CRC already passed, so a staged table effect
        // commits directly and any reply sequences from the packet end. A frame
        // that decodes as another servo's touches nothing.
        let Some((staged, slot, out)) =
            self.dispatch_decoded(anchor, footprint, |req, ctx, h| d.dispatch(req, ctx, h))
        else {
            return;
        };
        if matches!(out, Dispatched::Pending) {
            let mut handle = self.reply_handle();
            d.commit(&mut handle);
        }
        if staged && self.tx.staged() {
            self.sequence_reply(slot, packet_end);
        }
    }

    /// Verify a pending frame's CRC and resolve the verdict. Pass → commit a
    /// staged table effect (COMMIT) and sequence a staged reply (SEND); fail
    /// (or spin miss) → revert the write (REVERT), drop the reply
    /// (DON'T-SEND), count, and rewind the ladder (§5.3 L1).
    #[cfg_attr(target_os = "none", unsafe(link_section = ".highcode"))]
    #[cfg_attr(target_os = "none", inline(never))]
    fn verify<D: Dispatch>(&mut self, p: Pending, d: &mut D) {
        if !self.crc_verify(p.anchor, p.footprint) {
            if p.table {
                d.revert();
            }
            self.drop_staged();
            if !self.framer.probing() {
                self.crc_fails = self.crc_fails.wrapping_add(1);
            }
            let len = self.ring.bytes().len();
            self.framer.on_frame_rejected(p.anchor, len);
            return;
        }
        self.framer.on_frame_verified();
        if p.table {
            let mut handle = self.reply_handle();
            d.commit(&mut handle);
        }
        // Sequence from the ENGINE's state, not the recorded flag: any path
        // that reclaimed the staged reply between dispatch and here would
        // otherwise arm the chain over an empty engine (ghost trigger).
        if p.staged && self.tx.staged() {
            self.sequence_reply(p.slot, p.packet_end);
        }
    }

    /// Sequence a staged reply onto the snoop chain. reply gap is a wire gap
    /// measured from the packet end (§7), not from this (later) verify wake —
    /// the estimate is the framer's, conservative by the drift adder.
    fn sequence_reply(&mut self, slot: u8, packet_end: u32) {
        let out = self.chain.on_reply_staged(
            slot,
            packet_end,
            self.reply_gap(),
            self.reclaim(),
            self.frame_allowance(),
        );
        self.route_chain(out);
    }

    /// Drop a not-yet-streaming staged reply and any chain sequencing it began.
    fn drop_staged(&mut self) {
        if self.tx.staged() {
            self.tx.abort();
        }
        self.chain.reset();
        self.chain_at = None;
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

    /// Feed the covered span into the CRC engine straight from the ring — no
    /// M2M staging. Even-align the halfword feed by dropping the break's
    /// leading `0x00` when the anchor is odd (an init-0 no-op, so the CRC is
    /// unchanged); the ring base is halfword-aligned (`ring` is
    /// `repr(align(2))`). A ring-wrap splits into two accumulating arms — the
    /// engine sums across them. A trailing odd byte is left for the fold at
    /// verify. Direct feed is safe against the live circular ring: the engine
    /// runs ~8× wire speed, so it drains the covered span long before the write
    /// cursor could lap the ring back onto it.
    #[cfg_attr(target_os = "none", unsafe(link_section = ".highcode"))]
    #[cfg_attr(target_os = "none", inline(never))]
    fn crc_feed(&mut self, anchor: u16, footprint: u16) {
        let ring = self.ring.bytes();
        let len = ring.len();
        let anchor = anchor as usize;
        let start = anchor + (anchor & 1);
        let end = anchor + footprint as usize - 2;
        let bulk_end = end - ((end - start) & 1);
        self.crc.reset();
        if bulk_end <= len {
            self.crc.feed(&ring[start..bulk_end]);
        } else {
            self.crc.feed(&ring[start..len]);
            self.crc.feed(&ring[..bulk_end - len]);
        }
    }

    /// The covered byte the even-bulk feed left behind, if the span was odd.
    fn crc_tail(&self, anchor: u16, footprint: u16) -> Option<u8> {
        let ring = self.ring.bytes();
        let len = ring.len();
        let anchor = anchor as usize;
        let start = anchor + (anchor & 1);
        let end = anchor + footprint as usize - 2;
        if (end - start) & 1 == 1 {
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
