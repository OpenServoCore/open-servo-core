/// Wire-condition telemetry counters — the driver's floor signals,
/// recorded by the composite at the point of detection and stored
/// chip-side (control-table region) for the host to read back. Neither
/// counter is a wire-correctness failure by itself; both are
/// bench-defended design-floor signals.
pub trait Telemetry {
    /// Bump the `edge_anchor_miss` counter. Recorded once per parser Crc
    /// event where the classifier had no anchor (interference / edge
    /// loss) and the composite fell back to a `PollSrc`-derived
    /// `packet_end_tick` estimate.
    fn record_edge_anchor_miss(&mut self);

    /// Bump the `crc_patch_deadline_miss` counter. Recorded once per
    /// Fast Last TX-start residue fold that exits without patching —
    /// both the patch-window-expired route and the predecessor-byte
    /// plateau route feed the same counter; both ship a placeholder CRC
    /// observable to the host as a bad-CRC packet.
    fn record_crc_patch_deadline_miss(&mut self);
}
