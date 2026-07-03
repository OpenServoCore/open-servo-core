//! Input record for [`super::FastLast::start`] — pure data (driver-pattern
//! §4 carve-out). The composite reads each field and routes it to the half
//! that consumes it: grid fields to the [`FsmScheduler`], fold bounds to the
//! [`FoldEngine`].
//!
//! [`FsmScheduler`]: super::fsm_scheduler::FsmScheduler
//! [`FoldEngine`]: super::fold_engine::FoldEngine

/// Everything the Fast Last pipeline needs to start one Last reply, composed
/// by the composite at `send_slot(Last)` time. `packet_end_tick` is
/// parser-derived in the WireClock u32 domain; all grid timing derives from
/// it.
#[derive(Copy, Clone, Debug)]
pub struct FastLastSchedule {
    /// Parser-derived WireClock u32 value where the host's request ended
    /// (= `BT[last_byte] + 10·tpb`). All grid CMPs land at
    /// `packet_end_tick + offset` modulo u32.
    pub packet_end_tick: u32,
    /// Return-delay-time, in scheduler ticks.
    pub rdt_ticks: u16,
    /// One wire byte time at the active baud, in scheduler ticks.
    pub byte_ticks: u16,
    /// Count of wire bytes the host's request will pull from servos with
    /// earlier slots than ours, before our reply slot. Drives both the grid
    /// span and the fold engine's finalize cap.
    pub predecessor_bytes: u32,
    /// Wire cursor of the predecessor's first reply byte — the fold engine's
    /// lower bound (`ReplyContext::fold_start_cursor` at parse-complete). The
    /// scheduler ignores it; it exists so the composite can start both halves
    /// from one record.
    pub fold_start_cursor: u32,
}
