//! osc-native transport composite (`docs/osc-native-protocol.md`, driver-pattern
//! §4, §5.4). Routes break/deadline/TX events between the three sub-drivers
//! (`Framer`, `Chain`, `TxEngine`), owns the one hardware CRC engine both TX
//! generation and RX validation share, and muxes their deadlines onto the
//! single tick-compare.

use osc_core::traits::{Dispatch, Reply, SendError, Status};
use osc_core::{BaudRate, BootMode};
use osc_protocol::wire::{Inst, ResultCode};

use super::FRAME_MAX;
use super::chain::{Chain, ChainOut};
use super::decode::{Decoded, decode};
use super::framer::{FrameSpan, Framer, FramerOut};
use super::tx::{TxEngine, TxOut};
use crate::traits::bus::{
    CrcEngine, Deadline, LineSense, Providers, RxRing, TxWire, UsartBaud, tick_reached,
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

/// Transport health counters the chip publishes into the telemetry region
/// (§5.3 layer 1: dropped frames are counted, never answered).
pub struct LinkDiag {
    pub crc_fail_count: u32,
    pub framing_drop_count: u32,
}

/// Halfword-aligned linearization scratch for the rare ring-seam frame; the
/// hot path decodes in place out of the ring.
#[repr(align(2))]
struct Scratch([u8; FRAME_MAX]);

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
    scratch: Scratch,
    // Deadline mux (§4.1/§6/§9.1): the soonest live slot arms the compare.
    framer_at: Option<u32>,
    chain_at: Option<u32>,
    rescue_at: Option<u32>,
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
            scratch: Scratch([0; FRAME_MAX]),
            framer_at: None,
            chain_at: None,
            rescue_at: None,
        }
    }

    /// USART framing-error ISR: a break (or mid-frame garble) landed.
    pub fn on_break(&mut self) {
        let now = self.deadline.now();
        let out = self
            .framer
            .on_break(self.ring.bytes(), self.ring.cursor(), now, self.tpb);
        let _ = self.apply_framer_out(out); // on_break never yields a Frame
        // §6: a break while we hold a staged chain slot means the predecessor
        // is alive — suspend its reclaim window while the frame plays out.
        let out = self.chain.on_break_observed(now);
        self.route_chain(out);
        // §9.1 rescue candidacy: confirm a held-low line ~100 µs on.
        if self.line.is_low() {
            let at = now.wrapping_add(RESCUE_CONFIRM_US * <P::Deadline as Deadline>::TICKS_PER_US);
            self.rescue_at = Some(at);
        }
        self.arm_deadline();
    }

    /// Tick-compare ISR: one or more muxed deadlines are due.
    pub fn on_deadline<D: Dispatch>(&mut self, d: &mut D) {
        let now = self.deadline.now();
        if due(now, self.framer_at) {
            self.framer_at = None;
            let out = self
                .framer
                .on_deadline(self.ring.bytes(), self.ring.cursor(), now, self.tpb);
            if let Some(span) = self.apply_framer_out(out) {
                self.process_frame(span, d, now);
            }
        }
        if due(now, self.chain_at) {
            self.chain_at = None;
            let out = self.chain.on_deadline(now);
            self.route_chain(out);
        }
        if due(now, self.rescue_at) {
            self.rescue_at = None;
            self.try_rescue();
        }
        self.arm_deadline();
    }

    /// TX DMA arm-complete ISR: stream the next arm, or apply deferred config
    /// once the whole reply has drained (§4.2) — the ack always leaves at the
    /// old id/baud, the change lands after.
    pub fn on_tx_complete(&mut self) {
        if self.tx.on_arm_complete() == TxOut::Released {
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

    /// Arm the compare at the soonest live slot, or cancel if none.
    fn arm_deadline(&mut self) {
        let now = self.deadline.now();
        let mut best: Option<u32> = None;
        for at in [self.framer_at, self.chain_at, self.rescue_at]
            .into_iter()
            .flatten()
        {
            best = Some(match best {
                Some(b) if b.wrapping_sub(now) <= at.wrapping_sub(now) => b,
                _ => at,
            });
        }
        match best {
            Some(at) => self.deadline.set(at),
            None => self.deadline.cancel(),
        }
    }

    fn apply_framer_out(&mut self, out: FramerOut) -> Option<FrameSpan> {
        match out {
            FramerOut::None => {
                self.framer_at = None;
                None
            }
            FramerOut::Wait(t) => {
                self.framer_at = Some(t);
                None
            }
            FramerOut::Rearm => {
                self.ring.rearm();
                self.framer_at = None;
                None
            }
            FramerOut::Frame(span) => {
                self.framer_at = None;
                Some(span)
            }
        }
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
        // Line risen → it was an ordinary break, not a rescue pulse.
        if !self.line.is_low() {
            return;
        }
        // §9.1: volatile rate switch — the config register is untouched.
        self.baud.apply(BaudRate::B500000);
        self.rate = BaudRate::B500000;
        self.tpb = tpb_for::<P>(self.rate);
        self.framer.abort();
        self.chain.reset();
        self.tx.abort();
        self.framer_at = None;
        self.chain_at = None;
    }

    fn process_frame<D: Dispatch>(&mut self, span: FrameSpan, d: &mut D, now: u32) {
        // FrameSpan is not Copy; work from its primitive fields.
        let anchor = span.anchor;
        let footprint = span.footprint;
        // 1. Validate in place; a fail (or spin miss) drops silently (§5.3 L1).
        if !self.crc_ok(anchor, footprint) {
            self.crc_fails = self.crc_fails.wrapping_add(1);
            return;
        }
        // 2. Status frames only advance the snoop chain (§6). `now` as the end
        // tick is conservative by the deadline-B margin — extra gap, never short.
        if self.ring_inst(anchor).is_status() {
            let out = self.chain.on_status_end(now);
            self.route_chain(out);
            return;
        }
        // 3. A fresh instruction supersedes any stale, not-yet-streaming reply.
        self.chain.reset();
        self.chain_at = None;
        if self.tx.staged() {
            self.tx.abort();
        }
        // 4. Linearize: zero-copy from the ring unless the frame wraps the seam.
        let anchor = anchor as usize;
        let footprint = footprint as usize;
        let ring = self.ring.bytes();
        let len = ring.len();
        let frame: &[u8] = if anchor + footprint <= len {
            &ring[anchor..anchor + footprint]
        } else {
            let n1 = len - anchor;
            self.scratch.0[..n1].copy_from_slice(&ring[anchor..]);
            self.scratch.0[n1..footprint].copy_from_slice(&ring[..footprint - n1]);
            &self.scratch.0[..footprint]
        };
        // 5. Decode + dispatch over disjoint borrows (driver-pattern §4.3).
        let (req, ctx, slot) = match decode(frame, self.id) {
            Decoded::Own(req, ctx, slot) => (req, ctx, slot),
            _ => return, // Skip (not ours); Status is handled above
        };
        let mut handle = ReplyHandle {
            tx: &mut self.tx,
            id: self.id,
            pending_id: &mut self.pending_id,
            pending_baud: &mut self.pending_baud,
            pending_reboot: &mut self.pending_reboot,
            response_deadline_us: &mut self.response_deadline_us,
            staged: false,
        };
        d.dispatch(req, ctx, &mut handle);
        let staged = handle.staged;
        // 6. A staged reply enters chain sequencing (slot 0 = unicast).
        if staged {
            let t_turn = self.t_turn();
            let reclaim = self.reclaim();
            let allowance = self.frame_allowance();
            let out = self
                .chain
                .on_reply_staged(slot, now, t_turn, reclaim, allowance);
            self.route_chain(out);
        }
    }

    /// Feed the covered span (1 or 2 wrap halves) and compare against the wire
    /// CRC. Both halves stay even-length and even-addressed (F12): the ring
    /// length and anchor are both even, so any wrap split lands on a halfword
    /// boundary. A spin miss counts as a fail — indistinguishable from a bad
    /// frame, and never wedges the wire (§3.2).
    fn crc_ok(&mut self, anchor: u16, footprint: u16) -> bool {
        let ring = self.ring.bytes();
        let len = ring.len();
        let footprint = footprint as usize;
        if len == 0 || footprint < 2 {
            return false;
        }
        let anchor = anchor as usize;
        let covered = footprint - 2;
        self.crc.reset();
        let end = anchor + covered;
        if end <= len {
            self.crc.feed(&ring[anchor..end]);
        } else {
            self.crc.feed(&ring[anchor..len]);
            self.crc.feed(&ring[..end - len]);
        }
        let wire = u16::from_le_bytes([ring[end % len], ring[(end + 1) % len]]);
        let mut budget = super::SPIN_PER_BYTE * covered as u32;
        loop {
            if let Some(v) = self.crc.result() {
                return v == wire;
            }
            if budget == 0 {
                return false;
            }
            budget -= 1;
            core::hint::spin_loop();
        }
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
struct ReplyHandle<'a, W: TxWire> {
    tx: &'a mut TxEngine<W>,
    id: u8,
    pending_id: &'a mut Option<u8>,
    pending_baud: &'a mut Option<BaudRate>,
    pending_reboot: &'a mut Option<BootMode>,
    response_deadline_us: &'a mut u16,
    staged: bool,
}

impl<W: TxWire> Reply for ReplyHandle<'_, W> {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), SendError> {
        let r = self
            .tx
            .stage(self.id, status.result, status.alert, status.data);
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
