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

#[cfg(test)]
mod tests;

use osc_protocol::FrameBytes;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, BaudRate, Id, Inst};

use crate::traits::{Deadline, Providers, RxRing, TxWire, UsartBaud, tick_reached};

pub use wireop::WIRE_CAP;
mod wireop;
use framer::{Framer, Step};
use shape::{InvalidReason, Replies, Shape};

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
    /// Rescue pulse (protocol sec 9.1: ~1 ms dominant low), then the host
    /// UART drops to 0.5M in the same verb -- rescued servos are at the
    /// rescue rate by definition, so there is no way to half-do it.
    Rescue,
    /// Host-side UART rate only. Servo-first ordering is the client
    /// choreography's job (write the fleet's baud registers, then this).
    HostBaud(BaudRate),
    /// Engine-held copy of the fleet's RESPONSE_DEADLINE register.
    SetResponseDeadline { us: u16 },
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
    /// CRC-clean status frames delivered.
    pub statuses: u8,
    /// Ring bytes that anchored no clean status (junk, CRC failures, and
    /// any rogue non-status frame).
    pub garble: u16,
    /// Garble arrived after the last clean frame -- the sec 9.2 trailing-
    /// energy signal.
    pub garble_after_last_frame: bool,
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
        slot: u8,
        id: Id,
        inst: Inst,
        payload: FrameBytes<'a>,
    },
    Done(Terminal),
    /// An instrument wire op (raw send / burst / pulse) finished; `tick` is
    /// the wire-release moment.
    WireDone {
        tick: u32,
    },
}

enum State {
    Idle,
    /// Post-garble / post-baud enforced silence before the TX (sec 3.4).
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
    /// Instrument raw TX in flight (wireop); `on_tx_complete` advances it.
    WireTx,
    /// Instrument low pulse held until the deadline.
    WirePulse,
    /// Instrument break train: `left` bare breaks on the `gap`-tick grid.
    /// The Training twin whose gap is caller-chosen instead of parsed from
    /// the announce -- a lying announce is the trim suite's clock-offset
    /// injector.
    WireTrain {
        left: u8,
        gap: u32,
    },
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
/// TX scratch: the largest legal frame (258 ring bytes) plus alignment.
type TxBuf = FrameBuf<264>;

pub struct HostBus<P: Providers> {
    ring: P::Ring,
    deadline: P::Deadline,
    tx: P::Tx,
    baud: P::Baud,
    framer: Framer,
    buf: TxBuf,
    state: State,
    /// The active Exchange's derived plan; `None` outside one.
    plan: Option<Shape>,
    rate: BaudRate,
    response_deadline_us: u16,
    /// Enforced-quiet end after garble evidence / a baud change.
    pace_until: Option<u32>,
    /// Current state's deadline tick (window end, pulse end, train slot).
    deadline_at: u32,
    /// Await re-arm span: any wire progress pushes the window out by this.
    window: u32,
    /// Statuses delivered for the active command.
    got: u8,
    last_cursor: u16,
    evidence: WireEvidence,
    pending_done: Option<Terminal>,
    edges: P::Edges,
    wire_job: wireop::WireJob,
    pending_wire_done: Option<u32>,
}

impl<P: Providers> HostBus<P> {
    /// `rate` is the UART's current rate as the provider configured it.
    pub fn new(
        ring: P::Ring,
        deadline: P::Deadline,
        tx: P::Tx,
        baud: P::Baud,
        edges: P::Edges,
        rate: BaudRate,
    ) -> Self {
        Self {
            ring,
            deadline,
            tx,
            baud,
            framer: Framer::new(),
            buf: TxBuf::new(),
            state: State::Idle,
            plan: None,
            rate,
            response_deadline_us: wire::DEFAULT_RESPONSE_DEADLINE_US,
            pace_until: None,
            deadline_at: 0,
            window: 0,
            got: 0,
            last_cursor: 0,
            evidence: WireEvidence::default(),
            pending_done: None,
            edges,
            wire_job: wireop::WireJob::new(),
            pending_wire_done: None,
        }
    }

    /// The instrument's capture organ, link-layer served (edge drains).
    pub fn edges(&mut self) -> &mut P::Edges {
        &mut self.edges
    }

    /// Current engine tick -- the drain reply's unwrap reference.
    pub fn now(&self) -> u32 {
        self.deadline.now()
    }

    pub fn submit(&mut self, cmd: Command<'_>) -> Result<(), SubmitError> {
        if !matches!(self.state, State::Idle)
            || self.pending_done.is_some()
            || self.pending_wire_done.is_some()
        {
            return Err(SubmitError::Busy);
        }
        self.evidence = WireEvidence::default();
        match cmd {
            Command::SetResponseDeadline { us } => {
                self.response_deadline_us = us;
                self.finish(Outcome::Complete);
            }
            Command::HostBaud(rate) => {
                self.baud.apply(rate);
                self.rate = rate;
                self.pace();
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
                let plan = Shape::derive(id, inst, payload).map_err(SubmitError::Invalid)?;
                self.buf.start(id, inst);
                self.buf.payload_mut()[..payload.len()].copy_from_slice(payload);
                self.buf.finish(payload.len() as u8);
                self.buf.seal();
                self.plan = Some(plan);
                // Quiet-bus bootstrap: drop anything unconsumed before the
                // TX window opens (host-side analog of the servo's rule).
                self.framer.resync(self.ring.cursor() as usize);
                match self.pace_until.take() {
                    Some(until) if !tick_reached(self.deadline.now(), until) => {
                        self.state = State::Pacing;
                        self.arm(until);
                    }
                    _ => self.start_tx(),
                }
            }
        }
        Ok(())
    }

    /// Chip TC ISR: the armed frame span drained (never fired for bare
    /// breaks -- see [`TxWire::send_break`]).
    pub fn on_tx_complete(&mut self) {
        if matches!(self.state, State::WireTx) {
            self.wire_advance();
            return;
        }
        if !matches!(self.state, State::Transmitting) {
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
            // draws no reply, and steady drive keeps the break edges crisp.
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
            self.last_cursor = self.ring.cursor();
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
            State::WireTrain { left, gap } => {
                self.tx.send_break();
                if left <= 1 {
                    self.wire_release_done();
                } else {
                    self.state = State::WireTrain {
                        left: left - 1,
                        gap,
                    };
                    self.deadline_at = self.deadline_at.wrapping_add(gap);
                    self.deadline.set(self.deadline_at);
                }
            }
            _ => {}
        }
    }

    /// Advance the engine from ring + clock state; at most one event per
    /// call, so callers loop until `None`. Callable at any frequency.
    pub fn poll(&mut self) -> Option<Event<'_>> {
        if let Some(done) = self.pending_done.take() {
            return Some(Event::Done(done));
        }
        if let Some(tick) = self.pending_wire_done.take() {
            return Some(Event::WireDone { tick });
        }
        let now = self.deadline.now();
        match self.state {
            State::Idle
            | State::Transmitting
            | State::Training { .. }
            | State::WireTx
            | State::WireTrain { .. } => {}
            State::Pacing => {
                if tick_reached(now, self.deadline_at) {
                    self.start_tx();
                }
            }
            State::RescueHold => {
                if tick_reached(now, self.deadline_at) {
                    self.tx.release();
                    self.baud.apply(BaudRate::RESCUE);
                    self.rate = BaudRate::RESCUE;
                    self.pace();
                    self.finish(Outcome::Sent);
                }
            }
            State::WirePulse => {
                if tick_reached(now, self.deadline_at) {
                    self.wire_poll_pulse();
                }
            }
            State::Awaiting => return self.poll_await(now),
        }
        if let Some(tick) = self.pending_wire_done.take() {
            return Some(Event::WireDone { tick });
        }
        self.pending_done.take().map(Event::Done)
    }

    fn poll_await(&mut self, now: u32) -> Option<Event<'_>> {
        let plan = self.plan?;
        let expected = match plan.replies {
            Replies::Single => Some(1),
            Replies::Chain(n) => Some(n),
            Replies::Collect => None,
            // Unreachable by construction (Replies::None never awaits);
            // fall back to a single-reply window rather than wedging.
            Replies::None => Some(1),
        };
        // Precomputed: the status arm below runs under a live ring borrow,
        // where whole-`self` helpers can't be called.
        let pace_at = now.wrapping_add(wire::STARVE_HORIZON_BYTE_TIMES * self.byte_ticks());
        let ring_len = self.ring.bytes().len();
        // A yielded status is recorded as coordinates and materialized only
        // after every mutation -- returning the ring borrow from inside the
        // walk would pin it across the timeout path below.
        let mut emit: Option<(u8, Id, Inst, usize, usize)> = None;
        loop {
            let cursor = self.ring.cursor() as usize;
            match self.framer.step(self.ring.bytes(), cursor) {
                Step::Frame(f) if f.inst.is_status() => {
                    self.got = self.got.saturating_add(1);
                    self.evidence.statuses = self.evidence.statuses.saturating_add(1);
                    self.evidence.garble_after_last_frame = false;
                    if expected == Some(self.got) {
                        self.deadline.cancel();
                        self.state = State::Idle;
                        // Ring content no verdict consumed by terminal time
                        // is evidence, not silence (see the horizon path).
                        let left = unresolved(self.framer.anchor(), cursor, ring_len);
                        if left > 0 {
                            self.evidence.garble = self.evidence.garble.saturating_add(left);
                            self.evidence.garble_after_last_frame = true;
                        }
                        if self.evidence.garble > 0 {
                            self.pace_until = Some(pace_at);
                        }
                        self.pending_done = Some(Terminal {
                            outcome: Outcome::Complete,
                            tick: now,
                            evidence: core::mem::take(&mut self.evidence),
                        });
                        self.plan = None;
                    } else {
                        self.deadline_at = now.wrapping_add(self.window);
                        self.deadline.set(self.deadline_at);
                    }
                    self.last_cursor = cursor as u16;
                    emit = Some((self.got - 1, f.id, f.inst, f.payload_pos, f.payload.len()));
                    break;
                }
                Step::Frame(f) => {
                    // A CRC-clean non-status while awaiting: a rogue talker.
                    // Junk for evidence purposes, but still wire progress.
                    let footprint = f.payload.len() as u16 + 6;
                    self.evidence.garble = self.evidence.garble.saturating_add(footprint);
                    self.evidence.garble_after_last_frame = true;
                    self.deadline_at = now.wrapping_add(self.window);
                    self.deadline.set(self.deadline_at);
                }
                Step::Garble(n) => {
                    self.evidence.garble = self.evidence.garble.saturating_add(n);
                    self.evidence.garble_after_last_frame = true;
                    self.deadline_at = now.wrapping_add(self.window);
                    self.deadline.set(self.deadline_at);
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
        let cursor = self.ring.cursor();
        let progressed = cursor != self.last_cursor;
        self.last_cursor = cursor;
        if progressed {
            self.deadline_at = now.wrapping_add(self.window);
            self.deadline.set(self.deadline_at);
        } else if tick_reached(now, self.deadline_at) {
            self.deadline.cancel();
            // Superimposed ENUM replies can wire-AND into a plausible frame
            // PREFIX that parks the resolver: at the horizon those bytes are
            // collision evidence, not silence -- without this fold an
            // all-matcher collision reads exactly like an empty subtree.
            let left = unresolved(self.framer.anchor(), cursor as usize, ring_len);
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

    fn start_tx(&mut self) {
        self.tx.claim();
        self.tx.send_break();
        self.tx.send(&self.buf.frame()[1..]);
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
    }

    /// One starve horizon of enforced bus silence at the current rate.
    fn pace(&mut self) {
        let quiet = wire::STARVE_HORIZON_BYTE_TIMES * self.byte_ticks();
        self.pace_until = Some(self.deadline.now().wrapping_add(quiet));
    }

    fn arm(&mut self, at: u32) {
        self.deadline_at = at;
        self.deadline.set(at);
    }

    /// Full await window: RESPONSE_DEADLINE + the expected reply's wire
    /// time + margin (+ the ENUM slot draw under Collect, + the flash-stall
    /// allowance on slow ops). An upper bound for failure detection, not a
    /// grid -- transport sec 9.4's "elastically late" rule made concrete.
    fn window_for(&self, plan: &Shape) -> u32 {
        let t = P::Deadline::TICKS_PER_US;
        let byte = self.byte_ticks();
        let mut w = self.response_deadline_us as u32 * t
            + plan.reply_footprint as u32 * byte
            + WINDOW_MARGIN_BYTES * byte;
        if matches!(plan.replies, Replies::Collect) {
            w += wire::ENUM_REPLY_SLOTS as u32 * byte;
        }
        if plan.slow {
            w += SLOW_OP_EXTRA_US * t;
        }
        w
    }

    /// Ticks per 10-bit character at the current rate. Exact for every
    /// operational baud; `TICKS_PER_US` up to ~420 stays inside u32.
    fn byte_ticks(&self) -> u32 {
        P::Deadline::TICKS_PER_US * 10_000_000 / self.rate.as_hz()
    }
}

/// Ring span `anchor..cursor` (mod `len`): bytes no framer verdict has
/// consumed. Sound while an await window stays under one ring lap, which
/// every reply window is by orders of magnitude.
fn unresolved(anchor: usize, cursor: usize, len: usize) -> u16 {
    ((cursor + len - anchor) % len) as u16
}
