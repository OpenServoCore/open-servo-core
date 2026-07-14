//! Instrument raw wire ops: the engine-owned side door behind the link
//! 0x6x family. Deliberately outside the command pipeline -- no Shape
//! validation (malformed frames are the point), no reply awaits, no sec
//! 3.4 pacing. What it shares with commands is the wire itself: the same
//! TxWire provider and drive discipline, the same TC completion routing,
//! and the one-owner rule -- a wire op and a command reject each other as
//! Busy through the same State machine.

use osc_protocol::wire::BaudRate;

use crate::traits::{Deadline, Providers, TxWire, UsartBaud};

use super::shape::InvalidReason;
use super::{HostBus, State, SubmitError};

/// Raw TX staging bound (burst streams included).
pub const WIRE_CAP: usize = 640;

enum Mode {
    /// One break-framed span: the whole buffer.
    Single,
    /// Length-prefixed frames (`len(1) bytes` repeated), fired
    /// back-to-back: each TC chains the next break + span.
    Stream,
}

pub(super) struct WireJob {
    buf: [u8; WIRE_CAP],
    len: usize,
    cursor: usize,
    mode: Mode,
    /// Break train staged behind the announce: the announce's TC anchors
    /// the grid, exactly like the engine's own CAL handoff.
    train: Option<(u8, u32)>,
}

impl WireJob {
    pub(super) fn new() -> Self {
        Self {
            buf: [0; WIRE_CAP],
            len: 0,
            cursor: 0,
            mode: Mode::Single,
            train: None,
        }
    }
}

impl<P: Providers> HostBus<P> {
    fn wire_ready(&self) -> Result<(), SubmitError> {
        if !matches!(self.state, State::Idle)
            || self.pending_done.is_some()
            || self.pending_wire_done.is_some()
        {
            return Err(SubmitError::Busy);
        }
        Ok(())
    }

    /// One law break + `bytes`, raw. Completion surfaces as
    /// [`Event::WireDone`](super::Event::WireDone).
    pub fn wire_send(&mut self, bytes: &[u8]) -> Result<(), SubmitError> {
        self.wire_ready()?;
        if bytes.is_empty() {
            // A bare break never raises TC (TxWire contract) -- an empty
            // send would wedge WireTx, so it is not a legal op.
            return Err(SubmitError::Invalid(InvalidReason::BadPayload));
        }
        if bytes.len() > WIRE_CAP {
            return Err(SubmitError::Invalid(InvalidReason::TooLong));
        }
        self.wire_job.buf[..bytes.len()].copy_from_slice(bytes);
        self.wire_job.len = bytes.len();
        self.wire_job.mode = Mode::Single;
        self.wire_job.train = None;
        self.tx.claim();
        self.tx.send_break();
        self.tx.send(&self.wire_job.buf[..self.wire_job.len]);
        self.state = State::WireTx;
        Ok(())
    }

    /// Announce frame + `breaks` bare law breaks spaced `gap_us` apart on
    /// the deadline grid. The Training twin with the gap DECOUPLED from the
    /// announce payload: a lying announce injects a known clock-offset
    /// reading (the trim suite's plant-direction gate), a truthful one is
    /// an ordinary CAL the engine never has to parse.
    pub fn wire_train(
        &mut self,
        announce: &[u8],
        gap_us: u16,
        breaks: u8,
    ) -> Result<(), SubmitError> {
        self.wire_ready()?;
        if announce.is_empty() || gap_us == 0 || breaks == 0 {
            return Err(SubmitError::Invalid(InvalidReason::BadPayload));
        }
        if announce.len() > WIRE_CAP {
            return Err(SubmitError::Invalid(InvalidReason::TooLong));
        }
        self.wire_job.buf[..announce.len()].copy_from_slice(announce);
        self.wire_job.len = announce.len();
        self.wire_job.mode = Mode::Single;
        self.wire_job.train = Some((breaks, gap_us as u32 * P::Deadline::TICKS_PER_US));
        self.tx.claim();
        self.tx.send_break();
        self.tx.send(&self.wire_job.buf[..self.wire_job.len]);
        self.state = State::WireTx;
        Ok(())
    }

    /// Arbitrary host UART rate, divisor-raw (off-catalog detunes are the
    /// clock-tracker's drift injector). The engine's own timing state
    /// tracks the nearest catalog rate; sec 3.4 pacing arms as for any
    /// rate change. Completes immediately.
    pub fn wire_baud(&mut self, bps: u32) -> Result<(), SubmitError> {
        self.wire_ready()?;
        if bps == 0 {
            return Err(SubmitError::Invalid(InvalidReason::BadPayload));
        }
        self.baud.apply_raw(bps);
        self.rate = nearest_rate(bps);
        self.pace();
        self.pending_wire_done = Some(self.deadline.now());
        Ok(())
    }

    /// Length-prefixed frames fired back-to-back, each break-framed. The
    /// inter-frame gap is one TC service -- microseconds, under a character
    /// time at every operational baud.
    pub fn wire_burst(&mut self, stream: &[u8]) -> Result<(), SubmitError> {
        self.wire_ready()?;
        if stream.len() > WIRE_CAP {
            return Err(SubmitError::Invalid(InvalidReason::TooLong));
        }
        let mut at = 0usize;
        while at < stream.len() {
            let flen = stream[at] as usize;
            if flen == 0 || at + 1 + flen > stream.len() {
                return Err(SubmitError::Invalid(InvalidReason::BadPayload));
            }
            at += 1 + flen;
        }
        if at == 0 {
            return Err(SubmitError::Invalid(InvalidReason::BadPayload));
        }
        self.wire_job.buf[..stream.len()].copy_from_slice(stream);
        self.wire_job.len = stream.len();
        self.wire_job.cursor = 0;
        self.wire_job.mode = Mode::Stream;
        self.tx.claim();
        self.wire_next_frame();
        self.state = State::WireTx;
        Ok(())
    }

    /// Hold the wire dominant for `us` microseconds (any width -- the
    /// engine's rescue verb owns the protocol-shaped 1 ms pulse; this is
    /// the instrument's sweepable version).
    pub fn wire_pulse_low(&mut self, us: u16) -> Result<(), SubmitError> {
        self.wire_ready()?;
        if us == 0 {
            return Err(SubmitError::Invalid(InvalidReason::BadPayload));
        }
        self.tx.claim();
        self.tx.hold_low();
        self.state = State::WirePulse;
        self.arm(
            self.deadline
                .now()
                .wrapping_add(us as u32 * P::Deadline::TICKS_PER_US),
        );
        Ok(())
    }

    /// TC while `WireTx`: chain the next burst frame, hand off to the
    /// break-train grid, or close the op.
    pub(super) fn wire_advance(&mut self) {
        if let Some((left, gap)) = self.wire_job.train.take() {
            // The wire stays claimed for the whole train (the engine's own
            // CAL discipline: steady drive keeps the break edges crisp).
            self.state = State::WireTrain { left, gap };
            self.arm(self.deadline.now().wrapping_add(gap));
            return;
        }
        let more =
            matches!(self.wire_job.mode, Mode::Stream) && self.wire_job.cursor < self.wire_job.len;
        if more {
            self.wire_next_frame();
        } else {
            self.wire_finish();
        }
    }

    /// Pulse deadline reached: release and close.
    pub(super) fn wire_poll_pulse(&mut self) {
        self.wire_finish();
    }

    fn wire_next_frame(&mut self) {
        let at = self.wire_job.cursor;
        let flen = self.wire_job.buf[at] as usize;
        self.wire_job.cursor = at + 1 + flen;
        self.tx.send_break();
        self.tx.send(&self.wire_job.buf[at + 1..at + 1 + flen]);
    }

    fn wire_finish(&mut self) {
        self.tx.release();
        self.state = State::Idle;
        self.pending_wire_done = Some(self.deadline.now());
    }

    /// Train's last break sent: release and close (`on_deadline` calls in).
    pub(super) fn wire_release_done(&mut self) {
        self.wire_finish();
    }
}

/// The catalog rate nearest `bps` -- the engine's timing-state stand-in
/// while the UART runs an off-catalog divisor.
fn nearest_rate(bps: u32) -> BaudRate {
    [
        BaudRate::B500000,
        BaudRate::B1000000,
        BaudRate::B2000000,
        BaudRate::B3000000,
    ]
    .into_iter()
    .min_by_key(|r| r.as_hz().abs_diff(bps))
    .unwrap_or(BaudRate::B1000000)
}
