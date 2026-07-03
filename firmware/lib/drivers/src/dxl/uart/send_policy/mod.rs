//! Send-side policy sub-composite — the state that decides how this servo
//! participates in a DXL exchange, split out of the `DxlUart` composite per
//! driver-pattern §4.3. Owns [`ConfigMediator`] (bus identity + staged-
//! config mailbox) and [`InstructionTracker`] (parse-side FSM over the
//! in-flight host Instruction); routes the config's identity into the
//! tracker's addressing and context building.

mod config_mediator;
mod fast_shape;
mod inflight;
mod instruction_tracker;
mod reply_context;

#[cfg(test)]
pub(super) use fast_shape::PING_STATUS_FRAME_BYTES;
pub(super) use fast_shape::header_target;
pub(super) use reply_context::ReplyContext;

use config_mediator::ConfigMediator;
use dxl_protocol::streaming::{InstructionHeader, InstructionPayload};
use instruction_tracker::InstructionTracker;
use osc_core::BootMode;

use crate::dxl::uart::poll_src::PollSrc;

pub(super) struct SendPolicy {
    config: ConfigMediator,
    instruction: InstructionTracker,
}

impl SendPolicy {
    pub(super) const fn new(id: u8, rdt_us: u32) -> Self {
        Self {
            config: ConfigMediator::new(id, rdt_us),
            instruction: InstructionTracker::new(),
        }
    }
}

// -- events -------------------------------------------------------------

impl SendPolicy {
    /// An Instruction header parsed. Routes the config's live ID into the
    /// tracker's addressing decision: `None` = ours, keep parsing;
    /// `Some(target)` = foreign, the caller byte-skips the body under
    /// `target`'s id.
    pub(super) fn on_instruction_header(&mut self, h: &InstructionHeader) -> Option<u8> {
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

    /// Crc-good on a tracked Instruction: consume the tracker's context
    /// and resolve it against the config's identity into the
    /// [`ReplyContext`] the send path consumes. `None` when nothing was
    /// tracked or the caller couldn't supply a packet-end tick (anchor
    /// miss with fallback disallowed) — the reply is dropped.
    pub(super) fn on_crc_good(
        &mut self,
        packet_end_tick: Option<u32>,
        fold_start_cursor: u32,
        src: PollSrc,
    ) -> Option<ReplyContext> {
        let ctx = self.instruction.on_crc_good()?;
        let packet_end_tick = packet_end_tick?;
        Some(ctx.into_reply_context(
            self.config.id(),
            self.config.rdt_us(),
            packet_end_tick,
            fold_start_cursor,
            src,
        ))
    }

    /// Crc-bad / parser resync — the packet is void; drops any tracking.
    pub(super) fn on_resync(&mut self) {
        self.instruction.on_resync();
    }

    /// Our reply fully drained the wire — the safe commit window for
    /// staged config. Returns any staged reboot for the composite to
    /// route chip-side.
    pub(super) fn on_tx_complete(&mut self) -> Option<BootMode> {
        self.config.on_tx_complete()
    }
}

// -- commands -----------------------------------------------------------

impl SendPolicy {
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
}

#[cfg(test)]
impl SendPolicy {
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
