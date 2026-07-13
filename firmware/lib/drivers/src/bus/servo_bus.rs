//! osc-native transport composite (`docs/osc-native-protocol.md`, driver-pattern
//! §4, §5.4). Routes break/deadline/TX events between the three sub-drivers
//! (`Framer`, `Chain`, `TxEngine`), owns the one hardware CRC engine both TX
//! generation and RX validation share, and muxes their deadlines onto the
//! single tick-compare.

use osc_core::traits::{Dispatch, Dispatched, Reply, Request, RequestCtx, SendError, Status};
use osc_core::{BaudRate, BootMode};
use osc_protocol::wire::{ENUM_REPLY_SLOTS, Id, Inst, Opcode, ResultCode};

use super::chain::{Chain, ChainOut};
use super::decode::{Decoded, decode};
use super::framer::{FrameSpan, Framer, FramerOut};
use super::trim::TrimLoop;
use super::tx::{TxEngine, TxOut};
use super::{frame_view, ring_wrap};
use crate::traits::bus::{CrcEngine, Deadline, Providers, RxRing, TxWire, UsartBaud, tick_reached};

/// µs-per-byte numerator: 10 bit-times/byte × 1e6 µs/s. `tpb = TICKS_PER_US ×
/// this / baud` stays within u32 for all four operational rates.
const BYTE_TIME_NUMERATOR: u32 = 10_000_000;

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

/// Trim measurement accept gate (§9.3), right-shift of the nominal span
/// (1/16 ≈ 6%): wider than any legal clock offset (the HSITRIM throw is
/// ±3.4%), far under a missed/spurious break or a real inter-frame pause.
/// Gates CAL gaps and drift chain-pairs alike.
const TRIM_GATE_SHIFT: u32 = 4;

/// CAL train watchdog, in announced gaps: a train silent this long is
/// abandoned — no decision — and the framer resumes (a suspended resolver
/// needs a deadline backstop like any busy-wait).
const CAL_WATCHDOG_GAPS: u32 = 2;

/// Seam-baseline capture length, in accepted chain-pairs (§9.3; a power of
/// two — the mean divides by shift on the divider-less chip build). Right
/// after a trim decision the clock is freshly measured, so the mean pair
/// error over these IS the host's queuing seam.
const DRIFT_BASELINE_PAIRS: u8 = 32;

/// Chain-pairs per drift window — a decision every second or two at
/// hot-loop rates, against thermal drift that moves over minutes.
const DRIFT_WINDOW_PAIRS: u8 = 128;

/// Drift-window verdicts past this are not thermal (HSI tempco cannot move
/// thousands of ppm between adjacent windows): a host seam shift or a
/// garbage window — discarded; the baseline stands, the next CAL re-anchors.
const DRIFT_SANITY_PPM: u32 = 8_000;

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

/// A live MGMT CAL break train (§9.3): the host's crystal spaces the breaks,
/// and break-FE entry stamps measure that ruler with the local clock. Both
/// stamps of every gap are the SAME ISR flavor, so entry latency cancels in
/// the difference; what survives is clock skew plus sub-µs jitter the
/// per-gap gate and the gap sum average out.
struct CalRun {
    gap_ticks: u32,
    gaps_left: u8,
    /// Announced gap count — the ≥-half validity bar at train end.
    total: u8,
    valid: u8,
    last_break: u32,
    /// Ring cursor at the last counted break: a real break rings its 0x00
    /// byte, a latched-flag re-fire does not — cursor progress is what
    /// distinguishes a ruler mark from a storm re-entry (the fault
    /// contract's freshness idiom, cal-local).
    cursor: u16,
    err: i32,
    span: u32,
}

/// What the wire delivered between two break-FE stamps (§9.3): the drift
/// tracker pairs the stamps only around exactly ONE CRC-verified SILENT
/// instruction — the one frame shape whose break-to-break span is
/// host-clocked end to end. Anything solicited puts a responder's
/// turnaround (its clock, not the host's) inside the span, and a reply gap
/// can slip under the 1/16 gate at 1M.
#[derive(Copy, Clone)]
enum VerifiedSpan {
    None,
    One { footprint: u16, silent: bool },
    Many,
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
    // The one frame dispatched ahead of its CRC verdict (§4: the spine).
    // Backpressure bounds it to one: a pending frame IS the frontier, so the
    // single staging slot and the single CRC accumulator are never contended.
    pending: Option<Pending>,
    // Deadline mux (§4.1/§6/§9.1): the soonest live slot arms the compare.
    framer_at: Option<u32>,
    chain_at: Option<u32>,
    // MGMT CAL (§9.3): a dispatched-but-not-started train (the announce),
    // the live train, and a completed measurement awaiting the main loop.
    pending_cal: Option<(u16, u8)>,
    cal: Option<CalRun>,
    cal_ready: bool,
    // Differential drift tracker (§9.3): the last break-FE stamp + ring
    // cursor, the one silent instruction verified since it, the seam
    // baseline, and the current drift window.
    drift_prev: Option<(u32, u16)>,
    drift_seen: VerifiedSpan,
    drift_base: Option<i32>,
    drift_base_err: i32,
    drift_base_n: u8,
    drift_win_err: i32,
    drift_win_span: u32,
    drift_win_n: u8,
    drift_ready: bool,
    // Completed-measurement handoff to the main loop's `poll_clock_trim`:
    // one (err, span) at a time, owned by whichever ready flag is set.
    cadence_err: i32,
    cadence_span: u32,
    trim: TrimLoop,
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
            pending_cal: None,
            cal: None,
            cal_ready: false,
            drift_prev: None,
            drift_seen: VerifiedSpan::None,
            drift_base: None,
            drift_base_err: 0,
            drift_base_n: 0,
            drift_win_err: 0,
            drift_win_span: 0,
            drift_win_n: 0,
            drift_ready: false,
            crc_fails: 0,
            pending: None,
            framer_at: None,
            chain_at: None,
            cadence_err: 0,
            cadence_span: 0,
            trim: TrimLoop::new(<P::Deadline as Deadline>::CLOCK_TRIM_STEP_PPM),
        }
    }

    /// Break-wake ISR (LBD: a genuine ≥10-bit dominant span landed, §3.4 —
    /// garble never reaches this handler) — a pure wake. It carries neither
    /// position nor time (A2): the resolver derives both from ring data, so
    /// a delayed or coalesced entry costs nothing (any frames completed
    /// meanwhile resolve on the fast path right here).
    pub fn on_break<D: Dispatch>(&mut self, d: &mut D) {
        let now = self.deadline.now();
        // MGMT CAL train (§9.3): while a train is live, breaks are ruler
        // marks, not frame traffic — stamp against the announced gap and
        // return. The ring still collects the break bytes; the framer's
        // hunt scans them off silently once the train ends (0x00 runs are
        // implausible candidates, and any junk lock dies CRC-uncounted
        // under the hunt's probing flag).
        if self.cal.is_some() || self.pending_cal.is_some() {
            self.on_cal_break(now);
            return;
        }
        // Freshness first, resolve second: the fault service computes one
        // bit (did bytes ring since the last service?) and nothing else —
        // the fault contract. The resolve consumes whatever the ring holds.
        let cursor = self.ring.cursor();
        let fresh = self.framer.on_wire_fault(cursor);
        // Drift stamp only when the newest ringed byte IS a break's 0x00 —
        // classification from ring data, per the contract. A spurious or
        // lagged wake can land at frame end looking FRESH (the frame's own
        // bytes drained since the break's service), and stamping it
        // clobbers the pair in flight — the tracker starved to zero on
        // silicon under the FE-era latched re-fires (2026-07-12; DES pin:
        // `tracker_survives_latched_refires_between_frames`). A CRC tail
        // that happens to end 0x00 leaks one stamp and costs one pair —
        // the byte-exactness and span gates absorb it.
        if fresh && self.newest_is_break_byte(cursor) {
            self.on_drift_break(now);
        }
        self.drive_framer(d);
        // A wake whose evidence isn't ringed yet leaves the resolver with
        // nothing (a coalesced or lagged service — the contract's
        // spurious-wake case): the ring tells the truth one byte-time
        // later — arm the recheck. A spurious re-fire costs one empty wake.
        if !fresh && self.framer_at.is_none() {
            self.framer_at = Some(now.wrapping_add(self.tpb));
        }
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
        // breaks and only suspends its reclaim window (§6). Broadcast-ENUM
        // replies are exempt (§9.2): colliding with a peer matcher IS their
        // contract — a yielded laggard turns the walk's collision signal
        // into one clean frame and prunes its subtree (task #30). A
        // CRC-verified fresh instruction still supersedes them (route_frame).
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
        // §6: a break while we hold a staged chain slot means the predecessor
        // is alive — suspend its reclaim window while the frame plays out.
        let out = self.chain.on_break_observed(now);
        self.route_chain(out);
        self.arm_deadline();
    }

    /// One CAL ruler mark (§9.3). The stamp is the CALLER's `now`, read at
    /// service entry before any other work — every gap's two ends then carry
    /// the same entry path, and its latency cancels in the difference.
    fn on_cal_break(&mut self, now: u32) {
        let cursor = self.ring.cursor();
        if let Some((gap_us, gaps)) = self.pending_cal.take() {
            self.drift_restart();
            // Train start: the first break after the announce opens gap 1.
            let gap_ticks = (gap_us as u32).wrapping_mul(<P::Deadline as Deadline>::TICKS_PER_US);
            self.cal = Some(CalRun {
                gap_ticks,
                gaps_left: gaps,
                total: gaps,
                valid: 0,
                last_break: now,
                cursor,
                err: 0,
                span: 0,
            });
            self.arm_cal_watchdog(now, gap_ticks);
            return;
        }
        let (finished, gap_ticks) = {
            let Some(cal) = &mut self.cal else {
                return; // SAFETY: caller guards; a bare entry changes nothing
            };
            if cursor == cal.cursor {
                return; // latched-flag re-fire: no byte ringed, not a mark
            }
            cal.cursor = cursor;
            let delta = now.wrapping_sub(cal.last_break);
            cal.last_break = now;
            let err = delta.wrapping_sub(cal.gap_ticks) as i32;
            if err.unsigned_abs() <= cal.gap_ticks >> TRIM_GATE_SHIFT {
                cal.err = cal.err.wrapping_add(err);
                cal.span = cal.span.wrapping_add(cal.gap_ticks);
                cal.valid += 1;
            }
            cal.gaps_left = cal.gaps_left.saturating_sub(1);
            (cal.gaps_left == 0, cal.gap_ticks)
        };
        if !finished {
            self.arm_cal_watchdog(now, gap_ticks);
            return;
        }
        if let Some(c) = self.cal.take() {
            // ≥ half the announced gaps measured clean, or no decision — a
            // mangled train yields nothing rather than something.
            if c.valid as u32 * 2 >= c.total as u32 {
                self.cadence_err = c.err;
                self.cadence_span = c.span;
                self.cal_ready = true;
            }
        }
        // Pend-on-past: hunt the train's break bytes off the ring now.
        self.framer_at = Some(now);
        self.arm_deadline();
    }

    /// The train's silence bound: `on_deadline` reads an expiring framer
    /// slot during a live train as "the train died" and abandons it.
    fn arm_cal_watchdog(&mut self, now: u32, gap_ticks: u32) {
        self.framer_at = Some(now.wrapping_add(gap_ticks.wrapping_mul(CAL_WATCHDOG_GAPS)));
        self.arm_deadline();
    }

    /// The newest ringed byte — the wire-fault service's break/re-fire
    /// discriminator (a break rings its 0x00 last; a frame-end re-fire
    /// sees the CRC tail).
    fn newest_is_break_byte(&self, cursor: u16) -> bool {
        let ring = self.ring.bytes();
        let len = ring.len();
        len != 0 && ring[ring_wrap(cursor as usize + len - 1, len)] == 0x00
    }

    /// Drift chain-pair (§9.3): adjacent break-FE stamps bracketing one
    /// silent verified instruction measure `seam + drift·span` — the host's
    /// queuing seam is unknown but stationary, so the mean pair error right
    /// after a trim decision IS the seam (baseline), and every later window
    /// reads drift as its shift from it. Anything constant — seam, FE latch
    /// offset, entry-path residue — dies in the subtraction; only changes
    /// survive, and the sanity band catches the non-thermal ones.
    fn on_drift_break(&mut self, now: u32) {
        let cursor = self.ring.cursor();
        let prev = self.drift_prev.replace((now, cursor));
        let seen = core::mem::replace(&mut self.drift_seen, VerifiedSpan::None);
        crate::bench::trim_probe(|p| p.stamps += 1);
        let (t1, c1) = match prev {
            Some(pair) => pair,
            None => {
                crate::bench::trim_probe(|p| p.no_prev += 1);
                return;
            }
        };
        let footprint = match seen {
            VerifiedSpan::One {
                footprint,
                silent: true,
            } => footprint,
            VerifiedSpan::One { silent: false, .. } => {
                crate::bench::trim_probe(|p| p.unsilent += 1);
                return;
            }
            VerifiedSpan::None => {
                crate::bench::trim_probe(|p| p.span_none += 1);
                return;
            }
            VerifiedSpan::Many => {
                crate::bench::trim_probe(|p| p.span_many += 1);
                return;
            }
        };
        let len = self.ring.bytes().len();
        // Byte-exactness: the pair's ring span must be exactly the verified
        // frame — anything else intervened (a status, garble, an echo).
        if len == 0 || ring_wrap(cursor as usize + len - c1 as usize, len) as u16 != footprint {
            crate::bench::trim_probe(|p| p.inexact += 1);
            return;
        }
        let span = (footprint as u32).wrapping_mul(self.tpb);
        let err = now.wrapping_sub(t1).wrapping_sub(span) as i32;
        if err.unsigned_abs() > span >> TRIM_GATE_SHIFT {
            crate::bench::trim_probe(|p| p.gated += 1);
            return; // a real pause (inter-chain gap), not a seam
        }
        match self.drift_base {
            None => {
                crate::bench::trim_probe(|p| p.base_pairs += 1);
                self.drift_base_err = self.drift_base_err.wrapping_add(err);
                self.drift_base_n += 1;
                if self.drift_base_n >= DRIFT_BASELINE_PAIRS {
                    self.drift_base = Some(self.drift_base_err / DRIFT_BASELINE_PAIRS as i32);
                }
            }
            Some(base) => {
                crate::bench::trim_probe(|p| p.win_pairs += 1);
                self.drift_win_err = self.drift_win_err.wrapping_add(err.wrapping_sub(base));
                self.drift_win_span = self.drift_win_span.saturating_add(span);
                self.drift_win_n += 1;
                if self.drift_win_n >= DRIFT_WINDOW_PAIRS {
                    if !self.cal_ready && !self.drift_ready {
                        crate::bench::trim_probe(|p| p.verdicts += 1);
                        self.cadence_err = self.drift_win_err;
                        self.cadence_span = self.drift_win_span;
                        self.drift_ready = true;
                    }
                    self.drift_win_err = 0;
                    self.drift_win_span = 0;
                    self.drift_win_n = 0;
                }
            }
        }
    }

    /// A verified frame between breaks — the drift tracker's qualification
    /// record. Only exactly-one counts, and only silent shapes pair.
    fn drift_note_verified(&mut self, anchor: u16, footprint: u16) {
        self.drift_seen = match self.drift_seen {
            VerifiedSpan::None => {
                let inst = self.ring_inst(anchor);
                let ring = self.ring.bytes();
                let len = ring.len();
                if len == 0 {
                    return; // defensive: no ring, no record
                }
                let id = ring[ring_wrap(anchor as usize + 1, len)];
                let silent = match inst.opcode() {
                    Some(Opcode::Gwrite) => true,
                    Some(Opcode::Write | Opcode::Commit) => {
                        inst.noreply() || id == Id::BROADCAST.as_byte()
                    }
                    _ => false,
                };
                VerifiedSpan::One { footprint, silent }
            }
            _ => VerifiedSpan::Many,
        };
    }

    /// Tracker restart: baseline, window, and pair continuity all drop —
    /// after a trim decision (the baseline's residual-skew term is stale),
    /// a CAL train (continuity broken), or a rate change.
    fn drift_restart(&mut self) {
        self.drift_prev = None;
        self.drift_seen = VerifiedSpan::None;
        self.drift_base = None;
        self.drift_base_err = 0;
        self.drift_base_n = 0;
        self.drift_win_err = 0;
        self.drift_win_span = 0;
        self.drift_win_n = 0;
        self.drift_ready = false;
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
                // CAL watchdog (§9.3): during a live train the framer slot
                // is the train's silence bound and nothing else — expiry
                // means the train died. Abandon it (no decision) and let
                // the framer hunt whatever actually arrived. A dangling
                // announce (train never started) dies here too.
                self.cal = None;
                self.pending_cal = None;
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
                self.drift_restart();
            }
            // A pending reboot waits for the main loop's `take_reboot`.
        }
    }

    /// Main-loop poll for a deferred reboot — withheld while a reply is
    /// draining (a reset mid-ack truncates the frame on the wire; bench
    /// 2026-07-10), honored on the first poll after the TX released. A
    /// silent (NOREPLY/broadcast) reboot stages no ack and takes
    /// immediately.
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
    /// completed measurement — a CAL train (absolute) or a drift window
    /// (baseline-relative) — through the trim loop. Returns the new trim
    /// total — signed chip steps from the factory default, positive =
    /// slower — for the caller to apply to the oscillator between frames.
    pub fn poll_clock_trim(&mut self) -> Option<i8> {
        if self.cal_ready {
            self.cal_ready = false;
            let (err, span) = (self.cadence_err, self.cadence_span);
            self.cadence_err = 0;
            self.cadence_span = 0;
            // The clock is freshly measured either way the decision goes:
            // the next pairs re-capture the seam baseline against it.
            self.drift_restart();
            return self.trim.on_window(err, span);
        }
        if !self.drift_ready {
            return None;
        }
        self.drift_ready = false;
        let (err, span) = (self.cadence_err, self.cadence_span);
        self.cadence_err = 0;
        self.cadence_span = 0;
        if span == 0 {
            return None; // defensive: an empty window decides nothing
        }
        let ppm = (err as i64 * 1_000_000 / span as i64) as i32;
        if ppm.unsigned_abs() > DRIFT_SANITY_PPM {
            return None; // not thermal — seam shift or garbage, discarded
        }
        let out = self.trim.on_window(err, span);
        if out.is_some() {
            // The clock moved: the baseline's residual-skew term is stale.
            self.drift_restart();
        }
        out
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

    /// §9.1 rescue: the chip's line sampler measured a ≥[`RESCUE_LOW_US`]
    /// dominant low with zero ring progress — a signal no transport wake can
    /// observe (the break detector latches only at a span's END, silicon
    /// 2026-07-12). Called from the main loop under a critical section; the
    /// pulse is still holding the line when the sampler declares, so the
    /// cursor is provably still.
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
        // §9.1: volatile rate switch — the config register is untouched.
        self.baud.apply(BaudRate::B500000);
        self.rate = BaudRate::B500000;
        self.tpb = tpb_for::<P>(self.rate);
        self.drift_restart();
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
        self.arm_deadline();
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
        self.drift_note_verified(anchor, footprint);
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
        self.drift_note_verified(p.anchor, p.footprint);
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
        // §9.2 slot delay: a collision-tolerant reply offsets its trigger by
        // (key ^ tick) & (SLOTS-1) byte-times. Twin matchers run
        // cycle-identical firmware and otherwise answer in unison — and a
        // sub-bit-aligned superposition of near-equal frames reads back as
        // ONE clean frame instead of collision garble (task #30). The tick
        // term re-draws every exchange, so equal keys never stick.
        let gap = match self.tx.slot_key() {
            Some(key) => {
                let draw = (key ^ self.deadline.now() as u8) & (ENUM_REPLY_SLOTS - 1);
                self.reply_gap().wrapping_add(draw as u32 * self.tpb)
            }
            None => self.reply_gap(),
        };
        let out = self.chain.on_reply_staged(
            slot,
            packet_end,
            gap,
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
        // §9.2: an ENUM reply's staged wire effect is collision-tolerant —
        // its job is to collide with peer matchers (the on_break kill site
        // skips it, and staging keys its slot delay off the UID payload).
        let tolerant = matches!(req, Request::Enumerate { .. });
        // Disjoint field borrows: `frame`/`req` hold `&self.ring`; the handle
        // takes the reply-staging fields mutably.
        let mut handle = ReplyHandle {
            tx: &mut self.tx,
            crc: &mut self.crc,
            id: &mut self.id,
            pending_id: &mut self.pending_id,
            pending_baud: &mut self.pending_baud,
            pending_reboot: &mut self.pending_reboot,
            pending_cal: &mut self.pending_cal,
            response_deadline_us: &mut self.response_deadline_us,
            staged: false,
            tolerant,
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
            id: &mut self.id,
            pending_id: &mut self.pending_id,
            pending_baud: &mut self.pending_baud,
            pending_reboot: &mut self.pending_reboot,
            pending_cal: &mut self.pending_cal,
            response_deadline_us: &mut self.response_deadline_us,
            staged: false,
            tolerant: false,
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
    /// Write-through to the bus id: `set_id` applies here so a status staged
    /// after it already carries the new id (the ASSIGN ack, §9.2).
    id: &'a mut u8,
    pending_id: &'a mut Option<u8>,
    pending_baud: &'a mut Option<BaudRate>,
    pending_reboot: &'a mut Option<BootMode>,
    pending_cal: &'a mut Option<(u16, u8)>,
    response_deadline_us: &'a mut u16,
    staged: bool,
    /// §9.2: the request is a broadcast ENUM — a reply staged through this
    /// handle is marked collision-tolerant, keyed off its own payload (the
    /// responder's UID) for the slot draw.
    tolerant: bool,
}

/// §9.2 slot key: the reply payload IS the responder's UID for ENUM — fold
/// its osc-CRC to a byte. The CRC mixing keeps same-reel sequential serials
/// apart; sequencing XORs in the live tick, so equal keys still re-roll.
fn slot_key(payload: &[u8]) -> u8 {
    let crc = osc_protocol::crc::osc_crc(payload);
    (crc ^ (crc >> 8)) as u8
}

impl<W: TxWire, C: CrcEngine> Reply for ReplyHandle<'_, W, C> {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), SendError> {
        let r = self
            .tx
            .stage(self.crc, *self.id, status.result, status.alert, status.data);
        if r.is_ok() {
            self.staged = true;
            if self.tolerant {
                self.tx.mark_collision_tolerant(slot_key(status.data));
            }
        }
        r
    }

    fn send_status_gather(
        &mut self,
        result: ResultCode,
        alert: bool,
        spans: &[&[u8]],
    ) -> Result<(), SendError> {
        let r = self
            .tx
            .stage_gather(self.crc, *self.id, result, alert, spans);
        if r.is_ok() {
            self.staged = true;
            if self.tolerant {
                let crc = spans
                    .iter()
                    .fold(0, |c, s| osc_protocol::crc::osc_crc_continue(c, s));
                self.tx.mark_collision_tolerant((crc ^ (crc >> 8)) as u8);
            }
        }
        r
    }

    fn stage_id(&mut self, id: u8) {
        *self.pending_id = Some(id);
    }

    fn set_id(&mut self, id: u8) {
        *self.id = id;
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

    fn begin_clock_cal(&mut self, gap_us: u16, gaps: u8) {
        *self.pending_cal = Some((gap_us, gaps));
    }
}

#[cfg(test)]
mod tests;
