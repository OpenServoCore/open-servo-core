//! Send-side policy sub-composite — the state that decides how this servo
//! participates in a DXL exchange, split out of the `DxlUart` composite per
//! driver-pattern §4.3. Owns [`ConfigMediator`] (bus identity + staged-
//! config mailbox), [`InstructionTracker`] (parse-side FSM over the
//! in-flight host Instruction), and [`ReplyGate`] (reply-side FSM over the
//! staged Status reply); routes the config's identity into the tracker's
//! addressing and hands the tracker's Crc output to the gate.

mod config_mediator;
mod fast_shape;
mod inflight;
mod instruction_tracker;
mod reply_context;
mod reply_gate;

#[cfg(test)]
pub(super) use fast_shape::PING_STATUS_FRAME_BYTES;
pub(super) use reply_context::ReplyContext;

use config_mediator::ConfigMediator;
use dxl_protocol::streaming::{InstructionHeader, InstructionPayload};
use instruction_tracker::InstructionTracker;
use osc_core::BootMode;
use reply_gate::ReplyGate;

use crate::dxl::uart::poll_src::PollSrc;

pub(super) struct SendPolicy {
    config: ConfigMediator,
    instruction: InstructionTracker,
    reply: ReplyGate,
}

impl SendPolicy {
    pub(super) const fn new(id: u8, rdt_us: u32) -> Self {
        Self {
            config: ConfigMediator::new(id, rdt_us),
            instruction: InstructionTracker::new(),
            reply: ReplyGate::new(),
        }
    }
}

// -- events -------------------------------------------------------------

impl SendPolicy {
    /// An Instruction header parsed. Clears any stale predecessor wait on
    /// the reply gate (`dxl-streaming-rx.md` §5.3), then routes the
    /// config's live ID into the tracker's addressing decision: `None` =
    /// ours, keep parsing; `Some(target)` = foreign, the caller
    /// byte-skips the body under `target`'s id.
    pub(super) fn on_instruction_header(&mut self, h: &InstructionHeader) -> Option<u8> {
        self.reply.on_new_instruction();
        self.instruction.on_instruction_header(h, self.config.id())
    }

    /// A Status header parsed — foreign reply frame; drops any tracking.
    pub(super) fn on_status_header(&mut self) {
        self.instruction.on_status_header();
    }

    /// Per-slot demarcation payload — advances the tracker's slot walk
    /// against the config's live ID.
    pub(super) fn on_slot(&mut self, payload: &InstructionPayload) {
        self.instruction.on_slot(payload, self.config.id());
    }

    /// Crc-good on a tracked Instruction: consume the tracker's context,
    /// resolve it against the config's identity into a [`ReplyContext`],
    /// and stage it on the reply gate for the dispatcher's `send_*` call.
    /// Returns the staged packet-end tick, or `None` when nothing was
    /// tracked or the caller couldn't supply a tick (anchor miss with
    /// fallback disallowed) — the reply is dropped.
    pub(super) fn on_crc_good(
        &mut self,
        packet_end_tick: Option<u32>,
        fold_start_cursor: u32,
        src: PollSrc,
    ) -> Option<u32> {
        let ctx = self.instruction.on_crc_good()?;
        let packet_end_tick = packet_end_tick?;
        self.reply.on_reply_context(ctx.into_reply_context(
            self.config.id(),
            self.config.rdt_us(),
            packet_end_tick,
            fold_start_cursor,
            src,
        ));
        Some(packet_end_tick)
    }

    /// Crc-bad / parser resync — the packet is void; drops any tracking
    /// and any predecessor wait.
    pub(super) fn on_resync(&mut self) {
        self.instruction.on_resync();
        self.reply.on_resync();
    }

    /// A foreign packet's byte-skip completed under `pred`'s id. `true`
    /// when it was the awaited chain predecessor — the caller starts the
    /// deferred wire send now.
    pub(super) fn on_skip_complete(&mut self, pred: u8) -> bool {
        self.reply.on_skip_complete(pred)
    }

    /// Our reply fully drained the wire — the safe commit window for
    /// staged config; also drops any stale predecessor wait. Returns any
    /// staged reboot for the composite to route chip-side.
    pub(super) fn on_tx_complete(&mut self) -> Option<BootMode> {
        self.reply.on_tx_complete();
        self.config.on_tx_complete()
    }
}

// -- commands -----------------------------------------------------------

impl SendPolicy {
    /// Consume the staged reply context for a `send_*` call — take-once;
    /// see [`ReplyGate::take_reply_context`].
    pub(super) fn take_reply_context(&mut self) -> Option<ReplyContext> {
        self.reply.take_reply_context()
    }

    /// Hold the encoded reply until `pred`'s Status frame finishes
    /// skipping — the Plain chain k > 0 deferral.
    pub(super) fn defer_to_predecessor(&mut self, pred: u8) {
        self.reply.defer_to_predecessor(pred)
    }

    /// Drop the staged reply context and any predecessor wait.
    pub(super) fn cancel(&mut self) {
        self.reply.cancel()
    }

    pub(super) fn stage_id(&mut self, id: u8) {
        self.config.stage_id(id)
    }

    pub(super) fn stage_rdt(&mut self, us: u32) {
        self.config.stage_rdt(us)
    }

    pub(super) fn stage_reboot(&mut self, mode: BootMode) {
        self.config.stage_reboot(mode)
    }
}

// -- accessors ----------------------------------------------------------

impl SendPolicy {
    pub(super) fn id(&self) -> u8 {
        self.config.id()
    }

    /// An addressed Instruction is mid-walk — the composite resolves a
    /// packet-end tick at Crc (and records anchor-miss telemetry) only in
    /// this state.
    pub(super) fn is_tracking(&self) -> bool {
        self.instruction.is_tracking()
    }

    /// Whether a `packet_end_tick` fallback estimate is safe for the
    /// tracked Instruction — see `InflightCtx::allows_packet_end_fallback`.
    pub(super) fn allows_packet_end_fallback(&self) -> bool {
        self.instruction.allows_packet_end_fallback()
    }

    /// The predecessor id a deferred chain reply is waiting on, if any.
    pub(super) fn awaited_predecessor(&self) -> Option<u8> {
        self.reply.awaited_predecessor()
    }
}

#[cfg(test)]
impl SendPolicy {
    /// Peek the staged reply context without consuming it — composite
    /// tests assert on the derived wire shape before any send.
    pub(super) fn staged_reply_for_test(&self) -> Option<ReplyContext> {
        self.reply.staged()
    }

    /// Stage a hand-built context directly, bypassing the parse-side
    /// tracker — `ReplyHandle` tests drive the send paths in isolation.
    pub(super) fn stage_reply_context_for_test(&mut self, ctx: ReplyContext) {
        self.reply.on_reply_context(ctx);
    }

    pub(super) fn rdt_us(&self) -> u32 {
        self.config.rdt_us()
    }

    pub(super) fn pending_id(&self) -> Option<u8> {
        self.config.pending_id()
    }

    pub(super) fn pending_reboot(&self) -> Option<BootMode> {
        self.config.pending_reboot()
    }
}
