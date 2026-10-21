//! `HostBus`: the host engine composite. One outstanding command; every
//! protocol deadline (RESPONSE_DEADLINE windows, fault pacing, the CAL
//! train grid, the rescue pulse) executes here, never client-side.
//!
//! Sans-io surface: `submit` accepts one [`Command`], the chip (or DES)
//! calls `poll` freely plus `on_tx_complete`/`on_deadline` from its ISRs,
//! and `poll` yields [`Event`]s -- streamed statuses, then exactly one
//! [`Terminal`]. Only the CAL train is time-critical inside `on_deadline`
//! (break placement rides ISR entry); every other transition times off
//! `now()` in `poll`, so the hardware compare is just a wake.

pub mod framer;
pub mod shape;
pub mod tb_framer;

#[cfg(test)]
mod tests;

use osc_protocol::FrameBytes;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, BaudRate, Id, Inst};

use crate::traits::{Deadline, Providers, RxRing, TxWire, UsartBaud, tick_reached};

#[cfg(feature = "bench")]
pub use wireop::WIRE_CAP;
/// The instrument side door; a same-shaped no-op without `bench`.
#[cfg_attr(not(feature = "bench"), path = "wireop_stub.rs")]
mod wireop;
use framer::{Framer, Step};
use shape::{InvalidReason, Replies, Shape};
use tb_framer::{TbFramer, TbStep};

pub use framer::Frame;

/// One logical command; exactly one [`Terminal`] answers it.
pub enum Command<'a> {
    /// Any wire instruction, MGMT included. The payload is raw instruction
    /// payload bytes; [`Shape::derive`] validates per-opcode before anything
    /// touches the wire. MGMT CAL additionally paces the break train after
    /// the announce; broadcast ENUM selects collect-until-quiet.
    Exchange {
        id: Id,
        inst: Inst,
        payload: &'a [u8],
    },
    /// An Exchange whose committed effect arms a servo-initiated TEL burst
    /// (the engine cannot know which writes are special; the client tags
    /// the exchange). After the arm's own ack -- none under NOREPLY or a
    /// silent broadcast -- the engine keeps collecting status frames until
    /// one satisfies [`wire::stream_last`]. `window_us` is the
    /// client-computed whole-burst allowance: one wide await window, never
    /// re-armed per frame; expiry before LAST is [`Outcome::Timeout`].
    ExchangeStream {
        id: Id,
        inst: Inst,
        payload: &'a [u8],
        window_us: u32,
    },
    /// Rescue pulse (protocol sec 9.1: ~1 ms dominant low), then the host
    /// UART drops to 0.5M in the same verb -- rescued servos are at the
    /// rescue rate by definition, so there is no way to half-do it.
    Rescue,
    /// Host-side UART rate only, any rate (an osc catalog rate takes the
    /// provider's exact divisor). Servo-first ordering is the client
    /// choreography's job (write the fleet's baud registers, then this);
    /// which rates are legal is the link server's.
    HostBaud { bps: u32 },
    /// Engine-held copy of the fleet's RESPONSE_DEADLINE register.
    SetResponseDeadline { us: u16 },
    /// One tinyboot frame, sent with no break (link sec 7): the engine adds
    /// the sync word, the request status, the length and the CRC. `cmd` is
    /// never read. `window_us` is the client's whole reply allowance, one
    /// fixed window that nothing re-arms; 0 exactly when `expect` is
    /// [`TbExpect::None`].
    TbExchange {
        cmd: u8,
        addr: u32,
        flags: u8,
        data: &'a [u8],
        expect: TbExpect,
        window_us: u32,
    },
}

/// The replies a [`Command::TbExchange`] draws (link sec 7).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TbExpect {
    /// [`Outcome::Sent`] once the frame has left the wire.
    None,
    /// [`Outcome::Complete`] at the first clean reply, else
    /// [`Outcome::Timeout`] at the window end.
    One,
    /// Every clean reply until the window ends, then [`Outcome::Complete`].
    Collect,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SubmitError {
    /// One outstanding command, ever -- including an unconsumed terminal.
    Busy,
    Invalid(InvalidReason),
}

/// Wire-level evidence gathered over one command, attached to its terminal.
/// The ENUM walk's collision verdicts (clean / garble / quiet) are client
/// policy over these counts -- the walk never lives in the engine.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WireEvidence {
    /// CRC-clean status frames delivered (a TEL burst can run to
    /// thousands).
    pub statuses: u16,
    /// Ring bytes that anchored no clean status (junk, CRC failures, and
    /// any rogue non-status frame).
    pub garble: u16,
    /// Garble arrived after the last clean frame -- the sec 9.2 trailing-
    /// energy signal.
    pub garble_after_last_frame: bool,
    /// Whole RX rings that landed before the walk read them (the consumer
    /// stalled, not the wire); the garble that follows is their wreckage.
    pub laps: u16,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// No reply owed: completed at wire-TX end (NOREPLY, silent broadcast,
    /// GWRITE, CAL train end, rescue pulse end).
    Sent,
    /// Every owed status delivered, or collect-until-quiet went quiet.
    Complete,
    /// Slot `slot` produced nothing within its window.
    Timeout { slot: u8 },
}

/// The one record that closes a command. `tick` is raw engine ticks
/// (`Deadline::TICKS_PER_US` converts); the link layer advertises the rate
/// once rather than converting per record.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Terminal {
    pub outcome: Outcome,
    pub tick: u32,
    pub evidence: WireEvidence,
}

/// Pulled by the caller until `None`; the link server turns these into pipe
/// records. Statuses stream before their terminal. Status payloads borrow
/// the ring zero-copy -- consumed before the next call by construction.
#[derive(Debug)]
pub enum Event<'a> {
    Status {
        /// GREAD list position; 0 for unicast; arrival order under Collect.
        /// For TEL burst frames a wrapping arrival index -- the payload's
        /// stream_seq is the real ordering.
        slot: u8,
        id: Id,
        inst: Inst,
        payload: FrameBytes<'a>,
    },
    /// A clean tinyboot reply (link sec 7.1); `slot` is its arrival index.
    TbFrame {
        slot: u8,
        cmd: u8,
        status: u8,
        addr: u32,
        flags: u8,
        data: FrameBytes<'a>,
    },
    Done(Terminal),
    /// An instrument wire op (raw send / burst / pulse) finished; `tick` is
    /// the wire-release moment. Never emitted without `bench`.
    WireDone {
        tick: u32,
    },
}

enum State {
    Idle,
    /// Enforced bus silence before the TX (sec 3.4): after garble, a
    /// rate change or a cancel.
    Pacing,
    /// Frame armed on the wire; waiting for the chip's `on_tx_complete`.
    Transmitting,
    /// CAL train: `left` breaks still to send on the `gap`-tick grid.
    Training {
        left: u8,
        gap: u32,
    },
    /// Rescue pulse held dominant until the deadline.
    RescueHold,
    Awaiting,
    /// An instrument wire op owns the engine; `wireop` holds its phase.
    #[cfg_attr(not(feature = "bench"), allow(dead_code))]
    Wire,
}

/// Rescue pulse hold: the sec 9.1 300 us sampler floor plus generous
/// sampler-jitter margin under load (protocol guidance ~1 ms).
const RESCUE_HOLD_US: u32 = 1_000;
/// Await-window slack beyond deadline + reply frame time, in byte-times --
/// covers the reply gap and scheduling fuzz at any baud.
const WINDOW_MARGIN_BYTES: u32 = 16;
/// SAVE/FACTORY flash-stall allowance (sec 9.4: erase + program run
/// 5-10 ms; ~50 ms is the protocol's comfortable guidance).
const SLOW_OP_EXTRA_US: u32 = 50_000;
/// How late after a GREAD's end its slots can be ready: every servo on the
/// chain works through the same bus backlog, stretched by its control
/// kernel (protocol sec 6). Sized by the DES pin
/// `host_waits_for_a_reclaim_counted_from_readiness` from the servo's
/// budget table.
pub const CHAIN_READY_LAG_US: u32 = 5_000;
/// The slowest rate the engine times, the tinyboot catalog's floor; below
/// it the byte-time arithmetic leaves u32.
pub const MIN_BPS: u32 = 9_600;
const TX_LEN: usize = 264;
/// TX scratch: the largest legal frame (258 ring bytes) plus alignment.
type TxBuf = FrameBuf<TX_LEN>;
/// Largest tinyboot data the engine sends or accepts: the TX scratch less
/// the frame overhead.
pub const MAX_TB_PAYLOAD: usize = TX_LEN - tb_framer::OVERHEAD;

/// Whether a client window fits: the deadline compare sees half the tick
/// range ahead.
pub const fn window_fits<D: Deadline>(window_us: u32) -> bool {
    match window_us.checked_mul(D::TICKS_PER_US) {
        Some(ticks) => ticks <= u32::MAX / 2,
        None => false,
    }
}

/// The active TbExchange's plan.
#[derive(Copy, Clone)]
struct TbPlan {
    expect: TbExpect,
    /// The fixed reply window, ticks.
    window: u32,
}

pub struct HostBus<P: Providers> {
    ring: P::Ring,
    deadline: P::Deadline,
    tx: P::Tx,
    baud: P::Baud,
    framer: Framer,
    tb_framer: TbFramer,
    buf: TxBuf,
    state: State,
    /// The active Exchange's derived plan; `None` outside one.
    plan: Option<Shape>,
    /// The active TbExchange's plan; `None` outside one.
    tb: Option<TbPlan>,
    /// The TbExchange frame's length in `buf`, from offset 0.
    tb_len: u16,
    /// The UART's rate; every byte-time derives from it.
    bps: u32,
    response_deadline_us: u16,
    /// Enforced-quiet end after garble evidence / a baud change / a
    /// cancel; `None` once reached. Quiet means ring silence (sec 3.4):
    /// bytes received since `quiet_mark` restart it.
    pace_until: Option<u32>,
    quiet_mark: u32,
    /// A cancelled exchange's reply window: while its replies may still
    /// come, each received byte restarts the quiet by this much, as the
    /// await would have re-armed. 0 outside a cancel.
    rearm: u32,
    /// Current state's deadline tick (window end, pulse end, train slot).
    deadline_at: u32,
    /// Await re-arm span: any wire progress pushes the window out by this.
    window: u32,
    /// Statuses delivered for the active command.
    got: u8,
    last_written: u32,
    evidence: WireEvidence,
    pending_done: Option<Terminal>,
    wire: wireop::Wire<P>,
}

impl<P: Providers> HostBus<P> {
    /// `rate` is the UART's current rate as the provider configured it.
    pub fn new(
        ring: P::Ring,
        deadline: P::Deadline,
        tx: P::Tx,
        baud: P::Baud,
        rate: BaudRate,
    ) -> Self {
        Self {
            ring,
            deadline,
            tx,
            baud,
            framer: Framer::new(),
            tb_framer: TbFramer::new(),
            buf: TxBuf::new(),
            state: State::Idle,
            plan: None,
            tb: None,
            tb_len: 0,
            bps: rate.as_hz(),
            response_deadline_us: wire::DEFAULT_RESPONSE_DEADLINE_US,
            pace_until: None,
            quiet_mark: 0,
            rearm: 0,
            deadline_at: 0,
            window: 0,
            got: 0,
            last_written: 0,
            evidence: WireEvidence::default(),
            pending_done: None,
            wire: wireop::Wire::new(),
        }
    }

    /// One outstanding op ever, an unconsumed terminal included.
    pub fn busy(&self) -> bool {
        !matches!(self.state, State::Idle) || self.pending_done.is_some() || self.wire.busy()
    }

    /// The UART's rate, raw bps (off-catalog rates included).
    pub fn bps(&self) -> u32 {
        self.bps
    }

    pub fn response_deadline_us(&self) -> u16 {
        self.response_deadline_us
    }

    /// Abandon the outstanding op for a new link session (link sec 2): stop
    /// driving the wire, mid-span if need be, and drop its unconsumed
    /// events. The next frame waits until whatever the old exchange still
    /// draws from the bus has landed: the rest of its reply window, then a
    /// starve horizon of silence. A UART left at a non-osc rate returns to
    /// the rescue rate.
    pub fn cancel(&mut self) {
        let was_busy = self.busy();
        let now = self.deadline.now();
        // A cut frame may still have left whole (the UART drains what it
        // holds), so a transmitting exchange is owed its replies too.
        // A stream or tinyboot window is fixed: no byte re-arms its drain.
        let drain = match (&self.state, self.plan, self.tb) {
            (_, Some(plan), _) if matches!(plan.replies, Replies::None) => None,
            (_, _, Some(tb)) if matches!(tb.expect, TbExpect::None) => None,
            (State::Awaiting, Some(plan), _) => Some((self.deadline_at, self.rearm_for(&plan))),
            (State::Awaiting, _, Some(_)) => Some((self.deadline_at, 0)),
            (State::Transmitting, Some(plan), _) => {
                let tx = self.buf.frame().len() as u32 * self.byte_ticks();
                let end = now.wrapping_add(tx + self.window_for(&plan));
                Some((end, self.rearm_for(&plan)))
            }
            (State::Transmitting, _, Some(tb)) => {
                let tx = self.tb_len as u32 * self.byte_ticks();
                Some((now.wrapping_add(tx + tb.window), 0))
            }
            _ => None,
        };
        if matches!(
            self.state,
            State::Transmitting | State::Training { .. } | State::RescueHold | State::Wire
        ) {
            self.tx.release();
        }
        if !matches!(self.state, State::Idle) {
            self.deadline.cancel();
        }
        self.state = State::Idle;
        self.plan = None;
        self.tb = None;
        self.pending_done = None;
        self.evidence = WireEvidence::default();
        self.wire.cancel();
        if BaudRate::from_hz(self.bps).is_none() {
            self.set_bps(BaudRate::RESCUE.as_hz());
        } else if was_busy {
            self.pace();
        }
        if let Some((end, rearm)) = drain {
            self.rearm = rearm;
            if self.pace_until.is_none_or(|until| tick_reached(end, until)) {
                self.pace_until = Some(end);
            }
        }
    }

    /// Rate change: apply, then pace (sec 3.4). An osc rate keeps the
    /// provider's exact divisor; any other rate rounds one.
    fn set_bps(&mut self, bps: u32) {
        match BaudRate::from_hz(bps) {
            Some(rate) => self.baud.apply(rate),
            None => self.baud.apply_raw(bps),
        }
        self.bps = bps;
        self.pace();
    }

    pub fn submit(&mut self, cmd: Command<'_>) -> Result<(), SubmitError> {
        if self.busy() {
            return Err(SubmitError::Busy);
        }
        self.evidence = WireEvidence::default();
        match cmd {
            Command::SetResponseDeadline { us } => {
                self.response_deadline_us = us;
                self.finish(Outcome::Complete);
            }
            Command::HostBaud { bps } => {
                if bps < MIN_BPS {
                    return Err(SubmitError::Invalid(InvalidReason::BadPayload));
                }
                self.set_bps(bps);
                self.finish(Outcome::Complete);
            }
            Command::Rescue => {
                self.tx.claim();
                self.tx.hold_low();
                self.state = State::RescueHold;
                self.arm(
                    self.deadline
                        .now()
                        .wrapping_add(RESCUE_HOLD_US * P::Deadline::TICKS_PER_US),
                );
            }
            Command::Exchange { id, inst, payload } => {
                self.start_exchange(id, inst, payload, None)?;
            }
            Command::ExchangeStream {
                id,
                inst,
                payload,
                window_us,
            } => {
                self.start_exchange(id, inst, payload, Some(window_us))?;
            }
            Command::TbExchange {
                cmd,
                addr,
                flags,
                data,
                expect,
                window_us,
            } => {
                self.start_tb(cmd, addr, flags, data, expect, window_us)?;
            }
        }
        Ok(())
    }

    fn start_exchange(
        &mut self,
        id: Id,
        inst: Inst,
        payload: &[u8],
        stream_window_us: Option<u32>,
    ) -> Result<(), SubmitError> {
        let mut plan = Shape::derive(id, inst, payload).map_err(SubmitError::Invalid)?;
        if let Some(us) = stream_window_us {
            Self::window_ticks(us)?;
            plan = plan.stream(us).map_err(SubmitError::Invalid)?;
        }
        self.buf.start(id, inst);
        self.buf.payload_mut()[..payload.len()].copy_from_slice(payload);
        self.buf.finish(payload.len() as u8);
        self.buf.seal();
        self.plan = Some(plan);
        self.gate_tx();
        Ok(())
    }

    /// Seal the tinyboot frame in the TX scratch (link sec 7):
    /// `AA 55 cmd 00 addr(3) flags len(2) data crc(2)`, all LE.
    fn start_tb(
        &mut self,
        cmd: u8,
        addr: u32,
        flags: u8,
        data: &[u8],
        expect: TbExpect,
        window_us: u32,
    ) -> Result<(), SubmitError> {
        if data.len() > MAX_TB_PAYLOAD {
            return Err(SubmitError::Invalid(InvalidReason::TooLong));
        }
        if addr > 0xFF_FFFF || (matches!(expect, TbExpect::None) != (window_us == 0)) {
            return Err(SubmitError::Invalid(InvalidReason::BadPayload));
        }
        let window = Self::window_ticks(window_us)?;
        let b = self.buf.bytes_mut();
        let [a0, a1, a2, _] = addr.to_le_bytes();
        let [n0, n1] = (data.len() as u16).to_le_bytes();
        let [s0, s1] = tb_framer::SYNC;
        b[..tb_framer::HEADER].copy_from_slice(&[
            s0,
            s1,
            cmd,
            tb_framer::STATUS_REQUEST,
            a0,
            a1,
            a2,
            flags,
            n0,
            n1,
        ]);
        let end = tb_framer::HEADER + data.len();
        b[tb_framer::HEADER..end].copy_from_slice(data);
        let crc = tb_framer::tb_crc(0xFFFF, &b[..end]);
        b[end..end + 2].copy_from_slice(&crc.to_le_bytes());
        self.tb_len = (end + 2) as u16;
        self.tb = Some(TbPlan { expect, window });
        self.gate_tx();
        Ok(())
    }

    /// A client window in ticks; one that does not [`window_fits`] is
    /// refused.
    fn window_ticks(window_us: u32) -> Result<u32, SubmitError> {
        if !window_fits::<P::Deadline>(window_us) {
            return Err(SubmitError::Invalid(InvalidReason::BadPayload));
        }
        Ok(window_us * P::Deadline::TICKS_PER_US)
    }

    /// TX now, or pace first (sec 3.4).
    fn gate_tx(&mut self) {
        self.track_quiet(self.deadline.now());
        match self.pace_until {
            Some(until) => {
                self.state = State::Pacing;
                self.arm(until);
            }
            None => self.start_tx(),
        }
    }

    /// Chip TC ISR: the armed frame span drained (never fired for bare
    /// breaks -- see [`TxWire::send_break`]).
    pub fn on_tx_complete(&mut self) {
        if matches!(self.state, State::Wire) {
            self.wire_tx_complete();
            return;
        }
        if !matches!(self.state, State::Transmitting) {
            return;
        }
        if let Some(tb) = self.tb {
            self.tx.release();
            if matches!(tb.expect, TbExpect::None) {
                self.finish(Outcome::Sent);
            } else {
                self.got = 0;
                self.state = State::Awaiting;
                self.arm(self.deadline.now().wrapping_add(tb.window));
            }
            return;
        }
        let Some(plan) = self.plan else {
            // SAFETY-net: a plan always exists while Transmitting; releasing
            // the wire beats wedging it if the invariant ever breaks.
            self.tx.release();
            self.finish(Outcome::Sent);
            return;
        };
        if let Some((gap_us, gaps)) = plan.train {
            // The wire stays claimed for the whole train: broadcast CAL
            // draws no reply.
            let gap = gap_us as u32 * P::Deadline::TICKS_PER_US;
            self.state = State::Training {
                left: gaps + 1,
                gap,
            };
            self.arm(self.deadline.now().wrapping_add(gap));
        } else if matches!(plan.replies, Replies::None) {
            self.tx.release();
            self.finish(Outcome::Sent);
        } else {
            self.tx.release();
            self.window = self.window_for(&plan);
            self.got = 0;
            self.last_written = self.ring.written();
            self.state = State::Awaiting;
            self.arm(self.deadline.now().wrapping_add(self.window));
        }
    }

    /// Chip tick-compare ISR. Only the CAL train acts here (break placement
    /// is time-critical); everything else treats the compare as a wake and
    /// advances in `poll`.
    pub fn on_deadline(&mut self) {
        match self.state {
            State::Training { left, gap } => {
                self.tx.send_break();
                if left <= 1 {
                    self.tx.release();
                    self.finish(Outcome::Sent);
                } else {
                    self.state = State::Training {
                        left: left - 1,
                        gap,
                    };
                    // Next slot on the grid from the previous target, not
                    // from `now` -- ISR entry jitter must not accumulate
                    // down the train (sec 9.3: the host crystal keeps the
                    // spacing).
                    self.deadline_at = self.deadline_at.wrapping_add(gap);
                    self.deadline.set(self.deadline_at);
                }
            }
            State::Wire => self.wire_deadline(),
            _ => {}
        }
    }

    /// Advance the engine from ring + clock state; at most one event per
    /// call, so callers loop until `None`. Callable at any frequency.
    pub fn poll(&mut self) -> Option<Event<'_>> {
        if let Some(done) = self.pending_done.take() {
            return Some(Event::Done(done));
        }
        if let Some(tick) = self.wire.take_done() {
            return Some(Event::WireDone { tick });
        }
        let now = self.deadline.now();
        match self.state {
            State::Idle => self.track_quiet(now),
            State::Transmitting | State::Training { .. } => {}
            State::Pacing => {
                self.track_quiet(now);
                match self.pace_until {
                    None => self.start_tx(),
                    Some(until) if until != self.deadline_at => self.arm(until),
                    Some(_) => {}
                }
            }
            State::RescueHold => {
                if tick_reached(now, self.deadline_at) {
                    self.tx.release();
                    self.set_bps(BaudRate::RESCUE.as_hz());
                    self.finish(Outcome::Sent);
                }
            }
            State::Wire => self.wire_poll(now),
            State::Awaiting if self.tb.is_some() => return self.poll_tb_await(now),
            State::Awaiting => return self.poll_await(now),
        }
        if let Some(tick) = self.wire.take_done() {
            return Some(Event::WireDone { tick });
        }
        self.pending_done.take().map(Event::Done)
    }

    fn poll_await(&mut self, now: u32) -> Option<Event<'_>> {
        let plan = self.plan?;
        let expected = match plan.replies {
            Replies::Single => Some(1),
            Replies::Chain(n) => Some(n),
            Replies::Collect | Replies::Stream { .. } => None,
            // Unreachable by construction (Replies::None never awaits);
            // fall back to a single-reply window rather than wedging.
            Replies::None => Some(1),
        };
        // Stream awaits ride the one wide client window: no per-frame or
        // per-progress re-arm anywhere below.
        let stream = matches!(plan.replies, Replies::Stream { .. });
        // Precomputed: the status arm below runs under a live ring borrow,
        // where whole-`self` helpers can't be called.
        let pace_at = now.wrapping_add(wire::STARVE_HORIZON_BYTE_TIMES * self.byte_ticks());
        // A yielded status is recorded as coordinates and materialized only
        // after every mutation -- returning the ring borrow from inside the
        // walk would pin it across the timeout path below.
        let mut emit: Option<(u8, Id, Inst, usize, usize)> = None;
        loop {
            let marked = self.ring.marked();
            let written = self.ring.written();
            match self
                .framer
                .step(self.ring.bytes(), written, self.ring.marks(), marked)
            {
                Step::Frame(f) if f.inst.is_status() => {
                    self.got = self.got.wrapping_add(1);
                    self.evidence.statuses = self.evidence.statuses.saturating_add(1);
                    self.evidence.garble_after_last_frame = false;
                    let done = if stream {
                        wire::stream_last(f.inst, f.payload)
                    } else {
                        expected == Some(self.got)
                    };
                    if done {
                        self.deadline.cancel();
                        self.state = State::Idle;
                        // Ring content no verdict consumed by terminal time
                        // is evidence, not silence (see the horizon path).
                        let left = self.framer.unresolved(written);
                        if left > 0 {
                            self.evidence.garble = self.evidence.garble.saturating_add(left);
                            self.evidence.garble_after_last_frame = true;
                        }
                        if self.evidence.garble > 0 {
                            self.pace_until = Some(pace_at);
                            self.quiet_mark = written;
                        }
                        self.pending_done = Some(Terminal {
                            outcome: Outcome::Complete,
                            tick: now,
                            evidence: core::mem::take(&mut self.evidence),
                        });
                        self.plan = None;
                    } else if !stream {
                        self.deadline_at = now.wrapping_add(self.window);
                        self.deadline.set(self.deadline_at);
                    }
                    self.last_written = written;
                    emit = Some((
                        self.got.wrapping_sub(1),
                        f.id,
                        f.inst,
                        f.payload_pos,
                        f.payload.len(),
                    ));
                    break;
                }
                Step::Frame(f) => {
                    // A CRC-clean non-status while awaiting: a rogue talker.
                    // Junk for evidence purposes, but still wire progress.
                    let footprint = f.payload.len() as u16 + 6;
                    self.evidence.garble = self.evidence.garble.saturating_add(footprint);
                    self.evidence.garble_after_last_frame = true;
                    if !stream {
                        self.deadline_at = now.wrapping_add(self.window);
                        self.deadline.set(self.deadline_at);
                    }
                }
                Step::Garble(n) => {
                    self.evidence.garble = self.evidence.garble.saturating_add(n);
                    self.evidence.garble_after_last_frame = true;
                    if !stream {
                        self.deadline_at = now.wrapping_add(self.window);
                        self.deadline.set(self.deadline_at);
                    }
                }
                Step::Lapped(n) => {
                    self.evidence.laps = self.evidence.laps.saturating_add(n);
                }
                Step::Partial | Step::Idle => break,
            }
        }
        if let Some((slot, id, inst, pos, len)) = emit {
            return Some(Event::Status {
                slot,
                id,
                inst,
                payload: framer::payload_view(self.ring.bytes(), pos, len),
            });
        }
        // Byte-level progress since the previous poll extends the window;
        // ring-derived, never IDLE-derived.
        let written = self.ring.written();
        let progressed = written != self.last_written;
        self.last_written = written;
        if progressed && !stream {
            self.deadline_at = now.wrapping_add(self.window);
            self.deadline.set(self.deadline_at);
        } else if tick_reached(now, self.deadline_at) {
            self.deadline.cancel();
            // Superimposed ENUM replies can wire-AND into a plausible frame
            // PREFIX that parks the resolver: at the horizon those bytes are
            // collision evidence, not silence -- without this fold an
            // all-matcher collision reads exactly like an empty subtree.
            let left = self.framer.unresolved(written);
            if left > 0 {
                self.evidence.garble = self.evidence.garble.saturating_add(left);
                self.evidence.garble_after_last_frame = true;
            }
            let outcome = match plan.replies {
                // Quiet horizon reached: that IS collect's completion.
                Replies::Collect => Outcome::Complete,
                _ => Outcome::Timeout { slot: self.got },
            };
            self.finish(outcome);
        }
        self.pending_done.take().map(Event::Done)
    }

    /// The tinyboot await: one fixed window, never re-armed by frames,
    /// garble or byte progress (link sec 7).
    fn poll_tb_await(&mut self, now: u32) -> Option<Event<'_>> {
        let tb = self.tb?;
        // Coordinates only, materialized after every mutation (as
        // `poll_await`).
        let mut emit: Option<(u8, u8, u32, u8, usize, usize)> = None;
        loop {
            let written = self.ring.written();
            match self
                .tb_framer
                .step(self.ring.bytes(), written, MAX_TB_PAYLOAD as u16)
            {
                TbStep::Frame(f) => {
                    let (cmd, status, addr, flags) = (f.cmd, f.status, f.addr, f.flags);
                    let (pos, len) = (f.data_pos, f.data.len());
                    self.got = self.got.wrapping_add(1);
                    self.evidence.statuses = self.evidence.statuses.saturating_add(1);
                    self.evidence.garble_after_last_frame = false;
                    emit = Some((cmd, status, addr, flags, pos, len));
                    if matches!(tb.expect, TbExpect::One) {
                        self.deadline.cancel();
                        self.fold_tb_unresolved(written);
                        self.finish(Outcome::Complete);
                    }
                    break;
                }
                TbStep::Garble(n) => {
                    self.evidence.garble = self.evidence.garble.saturating_add(n);
                    self.evidence.garble_after_last_frame = true;
                }
                TbStep::Lapped(n) => {
                    self.evidence.laps = self.evidence.laps.saturating_add(n);
                }
                TbStep::Partial | TbStep::Idle => break,
            }
        }
        if let Some((cmd, status, addr, flags, pos, len)) = emit {
            return Some(Event::TbFrame {
                slot: self.got.wrapping_sub(1),
                cmd,
                status,
                addr,
                flags,
                data: framer::payload_view(self.ring.bytes(), pos, len),
            });
        }
        if tick_reached(now, self.deadline_at) {
            self.deadline.cancel();
            self.fold_tb_unresolved(self.ring.written());
            let outcome = match tb.expect {
                TbExpect::Collect => Outcome::Complete,
                _ => Outcome::Timeout { slot: 0 },
            };
            self.finish(outcome);
        }
        self.pending_done.take().map(Event::Done)
    }

    /// Ring content no verdict consumed by terminal time is evidence, not
    /// silence (as the osc await's horizon fold).
    fn fold_tb_unresolved(&mut self, written: u32) {
        let left = self.tb_framer.unresolved(written);
        if left > 0 {
            self.evidence.garble = self.evidence.garble.saturating_add(left);
            self.evidence.garble_after_last_frame = true;
        }
    }

    fn start_tx(&mut self) {
        // Quiet-bus bootstrap: drop anything unconsumed before the TX
        // window opens (host-side analog of the servo's rule).
        if self.tb.is_some() {
            // tinyboot frames carry no break (link sec 7).
            self.tb_framer.resync(self.ring.written());
            self.tx.claim();
            self.tx.send(&self.buf.bytes()[..self.tb_len as usize]);
        } else {
            self.framer.resync(self.ring.written(), self.ring.marked());
            self.tx.claim();
            self.tx.send_break();
            self.tx.send(&self.buf.frame()[1..]);
        }
        self.state = State::Transmitting;
    }

    /// Close the command: garble evidence arms the sec 3.4 pacing gap, the
    /// terminal is queued for the next `poll`.
    fn finish(&mut self, outcome: Outcome) {
        if self.evidence.garble > 0 {
            self.pace();
        }
        self.pending_done = Some(Terminal {
            outcome,
            tick: self.deadline.now(),
            evidence: core::mem::take(&mut self.evidence),
        });
        self.state = State::Idle;
        self.plan = None;
        self.tb = None;
    }

    /// One starve horizon of enforced bus silence at the current rate.
    fn pace(&mut self) {
        self.pace_until = Some(self.deadline.now().wrapping_add(self.horizon()));
        self.quiet_mark = self.ring.written();
    }

    /// Advance the quiet: bytes received since the last look restart it
    /// (sec 3.4 counts silence), and it ends once reached.
    fn track_quiet(&mut self, now: u32) {
        let Some(until) = self.pace_until else {
            return;
        };
        let written = self.ring.written();
        if written != self.quiet_mark {
            self.quiet_mark = written;
            let fresh = now.wrapping_add(self.horizon().max(self.rearm));
            if tick_reached(fresh, until) {
                self.pace_until = Some(fresh);
            }
        } else if tick_reached(now, until) {
            self.pace_until = None;
            self.rearm = 0;
        }
    }

    fn horizon(&self) -> u32 {
        wire::STARVE_HORIZON_BYTE_TIMES * self.byte_ticks()
    }

    fn arm(&mut self, at: u32) {
        self.deadline_at = at;
        self.deadline.set(at);
    }

    /// Full await window: RESPONSE_DEADLINE + the expected reply's wire
    /// time + margin (+ the ENUM slot draw under Collect, + the slots'
    /// readiness lag for a chain slot's reclaim, + the flash-stall
    /// allowance on slow ops). An upper bound for failure detection, not a
    /// grid -- transport sec 9.4's "elastically late" rule made concrete.
    fn window_for(&self, plan: &Shape) -> u32 {
        let t = P::Deadline::TICKS_PER_US;
        if let Replies::Stream { window_us } = plan.replies {
            return window_us.saturating_mul(t);
        }
        let byte = self.byte_ticks();
        let mut w = self.response_deadline_us as u32 * t
            + plan.reply_footprint as u32 * byte
            + WINDOW_MARGIN_BYTES * byte;
        match plan.replies {
            Replies::Collect => w += wire::ENUM_REPLY_SLOTS as u32 * byte,
            // A silent slot's successor reclaims one RESPONSE_DEADLINE after
            // it is ready, which can trail the host's window by a backlog
            // (protocol sec 6).
            Replies::Chain(_) => w += CHAIN_READY_LAG_US.max(self.response_deadline_us as u32) * t,
            _ => {}
        }
        if plan.slow {
            w += SLOW_OP_EXTRA_US * t;
        }
        w
    }

    /// A cancelled exchange's quiet re-arm: its await's window, unless the
    /// window is fixed.
    fn rearm_for(&self, plan: &Shape) -> u32 {
        if matches!(plan.replies, Replies::Stream { .. }) {
            0
        } else {
            self.window_for(plan)
        }
    }

    /// Ticks per 10-bit character at the current rate. Exact at the osc
    /// rates, rounded up elsewhere so a quiet or window never shortens;
    /// `TICKS_PER_US` up to ~420 stays inside u32.
    fn byte_ticks(&self) -> u32 {
        (P::Deadline::TICKS_PER_US * 10_000_000).div_ceil(self.bps)
    }
}
