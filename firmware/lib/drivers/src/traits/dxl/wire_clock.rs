/// 32-bit-horizon wire-clock counter. Same physical clock that ticks
/// `UsartBaud::CLOCK_HZ`; the chip-side ISR layer stamps the low 16 bits
/// (the IC peripheral's native width) into the edge ring, and `now()`
/// returns the same clock lifted to u32 so the driver can store packet-end
/// ticks, skip-deadline anchors, and scheduler deadlines without wrap
/// concerns at any supported baud (~89 s horizon at 48 MHz HCLK well
/// exceeds the longest packet at the lowest baud).
///
/// Contract: the low 16 bits of `now()` equal the modular IC-capture stamp
/// at the same instant. The classifier walks IC stamps in u16 modular
/// (small bit-time windows fit a single wrap) and lifts the output to u32
/// using a current `now()` reading. The chip-side provider is responsible
/// for satisfying the contract (e.g. by reading a u32 SysTick that shares
/// the same HCLK as TIM2's IC).
pub trait WireClock {
    fn now(&self) -> u32;
}
