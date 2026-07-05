/// 32-bit-horizon wire-clock counter. Same physical clock that ticks
/// `UsartBaud::CLOCK_HZ`; `now()` returns that clock lifted to u32 so the
/// driver can store packet-end ticks, skip-deadline anchors, and scheduler
/// deadlines without wrap concerns at any supported baud (the u32 horizon
/// well exceeds the longest packet at the lowest baud).
///
/// Contract: the low 16 bits of `now()` equal the chip's 16-bit
/// wire-schedule timer (TIM2 on V006) at the same instant, so the chip-side
/// TX scheduler can truncate a u32 `now()`-domain deadline into a single
/// 16-bit compare. The chip-side provider satisfies the contract, e.g. by
/// clocking the u32 counter and the schedule timer from the same source
/// and zeroing them together.
pub trait WireClock {
    /// Entry-latency compensation subtracted from the packet-end estimate,
    /// in HCLK ticks (baud-independent). The estimate is the drain-ISR's
    /// entry `now`, which carries the vector's fixed entry latency; biasing
    /// it down by this const keeps the residual wire excess slightly
    /// positive. Bench-tuned at band end via `tool-tune-tx-start`.
    const PACKET_END_ENTRY_COMP_TICKS: u32;

    fn now(&self) -> u32;
}
