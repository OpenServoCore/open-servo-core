/// Wire-condition telemetry counters — the driver's floor signals,
/// recorded by the composite at the point of detection and stored
/// chip-side (control-table region) for the host to read back. The
/// counter is not a wire-correctness failure by itself; it is a
/// bench-defended design-floor signal.
pub trait Telemetry {
    /// Bump the `crc_patch_deadline_miss` counter. Recorded once per
    /// Fast Last TX-start residue fold that exits without patching —
    /// both the patch-window-expired route and the predecessor-byte
    /// plateau route feed the same counter; both ship a placeholder CRC
    /// observable to the host as a bad-CRC packet.
    fn record_crc_patch_deadline_miss(&mut self);
}
