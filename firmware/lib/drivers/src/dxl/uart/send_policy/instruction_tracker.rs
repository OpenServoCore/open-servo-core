//! Parse-side FSM over the in-flight host Instruction. Transitions on
//! parser events only (header / slot / crc / resync); one packet's
//! lifetime. The reply-side lifecycle (staged context, chain-pending) is
//! a separate machine — the two run out of phase, e.g. a new Instruction
//! starts tracking while a prior packet's unconsumed reply context is
//! still staged.

use dxl_protocol::streaming::{InstructionHeader, InstructionPayload};

use super::fast_shape::target_addressable;
use super::inflight::InflightCtx;

/// Parse phase of the in-flight host Instruction.
enum InstructionPhase {
    /// No addressed Instruction between the parser's cursor and its last
    /// packet boundary.
    Idle,
    /// An Instruction addressed to us (own ID or BROADCAST) is being
    /// walked; the payload accumulates slot-walk state until Crc.
    Tracking(InflightCtx),
}

pub(super) struct InstructionTracker {
    phase: InstructionPhase,
}

impl InstructionTracker {
    pub(super) const fn new() -> Self {
        Self {
            phase: InstructionPhase::Idle,
        }
    }
}

// -- events -------------------------------------------------------------

impl InstructionTracker {
    /// An Instruction header parsed. Addressed to `id` or BROADCAST →
    /// start tracking, return `None`. Foreign → drop tracking, return
    /// `Some(target)` — the id the caller's universal byte-skip carries.
    pub(super) fn on_instruction_header(&mut self, h: &InstructionHeader, id: u8) -> Option<u8> {
        if target_addressable(h, id) {
            self.phase = InstructionPhase::Tracking(InflightCtx::new(*h));
            None
        } else {
            self.phase = InstructionPhase::Idle;
            Some(h.target().as_byte())
        }
    }

    /// A Status header parsed — a foreign reply frame; drop any tracking.
    pub(super) fn on_status_header(&mut self) {
        self.phase = InstructionPhase::Idle;
    }

    /// Per-slot demarcation payload — advance the slot walk when tracking.
    pub(super) fn on_slot(&mut self, payload: &InstructionPayload, id: u8) {
        if let InstructionPhase::Tracking(ctx) = &mut self.phase {
            ctx.on_slot(payload, id);
        }
    }

    /// Crc-good — the tracked Instruction is complete; consume its
    /// accumulated context. `None` when nothing was tracked.
    pub(super) fn on_crc_good(&mut self) -> Option<InflightCtx> {
        match core::mem::replace(&mut self.phase, InstructionPhase::Idle) {
            InstructionPhase::Tracking(ctx) => Some(ctx),
            InstructionPhase::Idle => None,
        }
    }

    /// Crc-bad / parser resync — the packet is void; drop any tracking.
    pub(super) fn on_resync(&mut self) {
        self.phase = InstructionPhase::Idle;
    }
}

// -- accessors ----------------------------------------------------------

impl InstructionTracker {
    /// An addressed Instruction is mid-walk — the caller resolves a
    /// packet-end tick at Crc only in this state.
    pub(super) fn is_tracking(&self) -> bool {
        matches!(self.phase, InstructionPhase::Tracking(_))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::test_support::TEST_ID;
    use dxl_protocol::Id;
    use dxl_protocol::wire::BROADCAST_ID;

    fn ping(id: u8) -> InstructionHeader {
        InstructionHeader::Ping { id: Id::new(id) }
    }

    #[test]
    fn own_header_starts_tracking_and_crc_consumes_once() {
        let mut t = InstructionTracker::new();
        assert_eq!(t.on_instruction_header(&ping(TEST_ID), TEST_ID), None);
        assert!(t.is_tracking());
        assert!(t.on_crc_good().is_some());
        assert!(!t.is_tracking());
        assert!(t.on_crc_good().is_none());
    }

    #[test]
    fn broadcast_header_starts_tracking() {
        let mut t = InstructionTracker::new();
        assert_eq!(t.on_instruction_header(&ping(BROADCAST_ID), TEST_ID), None);
        assert!(t.is_tracking());
    }

    #[test]
    fn foreign_header_returns_skip_target_and_stays_idle() {
        let mut t = InstructionTracker::new();
        assert_eq!(t.on_instruction_header(&ping(0x42), TEST_ID), Some(0x42));
        assert!(!t.is_tracking());
        assert!(t.on_crc_good().is_none());
    }

    #[test]
    fn status_header_and_resync_drop_tracking() {
        let mut t = InstructionTracker::new();
        t.on_instruction_header(&ping(TEST_ID), TEST_ID);
        t.on_status_header();
        assert!(!t.is_tracking());

        t.on_instruction_header(&ping(TEST_ID), TEST_ID);
        t.on_resync();
        assert!(!t.is_tracking());
    }
}
