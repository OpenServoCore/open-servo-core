/// 32-bit-horizon wire-clock counter. Same physical clock that ticks
/// `UsartBaud::CLOCK_HZ`; the chip-side ISR layer stamps the low 16 bits
/// (the edge-capture peripheral's native width) into the edge ring, and
/// `now()` returns the same clock lifted to u32 so the driver can store
/// packet-end ticks, skip-deadline anchors, and scheduler deadlines
/// without wrap concerns at any supported baud (the u32 horizon well
/// exceeds the longest packet at the lowest baud).
///
/// Contract: the low 16 bits of `now()` equal the modular edge-capture
/// stamp at the same instant. The classifier walks capture stamps in u16
/// modular (small bit-time windows fit a single wrap) and lifts the output
/// to u32 using a current `now()` reading. The chip-side provider is
/// responsible for satisfying the contract, e.g. by clocking a u32 counter
/// and the capture timer from the same source and zeroing them together.
pub trait WireClock {
    /// Entry-latency compensation subtracted from the packet-end estimate,
    /// in HCLK ticks (baud-independent). The estimate is the drain-ISR's
    /// entry `now`, which carries the vector's fixed entry latency; biasing
    /// it down by this const keeps the residual wire excess slightly
    /// positive. Bench-tuned at band end via `tool-tune-tx-start`.
    const PACKET_END_ENTRY_COMP_TICKS: u32;

    fn now(&self) -> u32;
}
