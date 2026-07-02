use osc_core::BaudRate;

/// Single-channel USART baud-rate control. The driver hands a domain-typed
/// `BaudRate`; the chip-side adapter owns the BRR math and any other
/// baud-dependent chip state (e.g., RX edge-capture filter on chips with
/// one) and applies them atomically. `CLOCK_HZ` is the clock that feeds
/// the USART's BRR divisor, which on the chip families this trait is
/// targeted at also ticks the monotonic the driver consumes — the driver
/// uses it for `ticks_per_bit` derivation and ticks↔µs conversion.
pub trait UsartBaud {
    const CLOCK_HZ: u32;
    fn apply_baud(&mut self, baud: BaudRate);

    /// Per-baud RX edge-stamp compensation, in `CLOCK_HZ` ticks. Each IC
    /// stamp the chip writes to the edge ring lands offset from the wire
    /// edge by some chip-specific per-baud constant — e.g. an input-capture
    /// filter delay on chips that ship one. The driver subtracts this value
    /// from every stamp at read-from-ring time so `packet_end_tick`, the
    /// classifier's anchor, and integrator pairs all live in wire-edge time.
    /// A chip with no stamp offset returns `0` for every baud. Pure function
    /// of baud — no per-instance state.
    fn rx_edge_comp_ticks(&self, baud: BaudRate) -> u16;
}
