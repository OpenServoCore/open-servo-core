//! Input record for [`super::FastLast::start`] — pure data (driver-pattern
//! §4 carve-out). The composite reads each field and routes it to the half
//! that consumes it: grid fields to the [`FsmScheduler`], fold bounds to the
//! [`FoldEngine`].
//!
//! [`FsmScheduler`]: super::fsm_scheduler::FsmScheduler
//! [`FoldEngine`]: super::fold_engine::FoldEngine

/// Everything the Fast Last pipeline needs to start one Last reply, composed
/// by the composite at the status-start observation (task #142 model:
/// `DxlUart::on_rx_byte_wake`). `status_start_tick` is the ET-ring-derived
/// start of the chain's single Status packet; all grid timing derives from
/// it — the predecessor window literally begins at the anchor, so no RDT
/// term exists ([[rdt-single-target-only]]).
#[derive(Copy, Clone, Debug)]
pub struct FastLastSchedule {
    /// Observed WireClock u32 tick where the Status packet's first wire
    /// byte started. All grid CMPs land at `status_start_tick + offset`
    /// modulo u32.
    pub status_start_tick: u32,
    /// One wire byte time at the active baud, in scheduler ticks.
    pub byte_ticks: u16,
    /// Count of wire bytes the host's request will pull from servos with
    /// earlier slots than ours, before our reply slot. Drives both the grid
    /// span and the fold engine's finalize cap.
    pub predecessor_bytes: u32,
    /// Wire cursor of the predecessor's first reply byte — the fold engine's
    /// lower bound (the Status packet's first byte lands at exactly this
    /// cursor, so the fold's `cursor < start_cursor` guard skips everything
    /// before it). The scheduler ignores it; it exists so the composite can
    /// start both halves from one record.
    pub fold_start_cursor: u32,
    /// Ring-drain stride in wire bytes — half the RX byte ring's depth,
    /// supplied by the composite (the ring size is its const generic).
    /// Windows longer than this get intermediate drain CMPs: the RX ring's
    /// producer head is an NDTR readback modulo the ring depth, so
    /// `on_publish` must run at least once per ring period or the head
    /// aliases and the checkpoint pickup reads the wrong bytes. The drain
    /// body is O(1) — publish + consume-to-cap, no CRC work.
    pub drain_stride_bytes: u32,
}
