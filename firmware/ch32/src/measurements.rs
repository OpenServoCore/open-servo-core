//! Bench-measured silicon timings. Estimates today (`docs/dxl-hw-timed-transport.md`
//! §5.1 / §5.4 + disassembly-sized hot path); replaced with measured values
//! during the post-M3 bench validation pass. Centralised here so a single
//! edit re-tunes the firmware — the provider / ISR bodies don't reference
//! raw numbers.
//!
//! Successor to the legacy `measurements.rs` (drained in `c95be9f1` along
//! with the SysTick fire path). Constants don't carry forward — the TIM2
//! OC fire path has its own entry-latency profile.

/// CC3-match → first wire bit, in TIM2 ticks at HCLK=48 MHz. Subtracted
/// from `deadline_tick` to back-date CCR3 so the wire-bit lands on time.
///
/// Sized to match the *minimum* observed path so the wire-bit always lands
/// at or after `deadline_tick`: jitter widens the slot (harmless) instead
/// of encroaching on the previous slot's RDT (collision).
///
/// Breakdown (CC3 match → DMA pumps first byte into USART DR). Under O3
/// the full driver dispatch (`on_tim2_cc3` → `Drivers::dxl_uart()` →
/// `on_tx_start` → `handle_start`) inlines into the IRQ vector body. Only
/// the DMA-enable path is on the critical path — the CC3IE mask that
/// follows is housekeeping after DMA is already pumping:
///
/// | Stage                                  | Ticks |
/// | -------------------------------------- | ----- |
/// | PFIC entry (accept + reg save)         | 26-27 |
/// | qingke_rt context save                 | 8-12  |
/// | `dma::enable(CH4)` — CS + RMW          | 6-8   |
/// | DMA AHB arbitration + USART DR write   | 6-8   |
/// |                                        | ~46   |
///
/// USART bit-clock alignment (0-16 ticks at 3 Mbaud) is *excluded* — that
/// jitter pushes the wire-bit later, which is the safe direction.
/// Measured: TBD (bench, `#8`).
pub const TX_START_ENTRY_TICKS: u16 = 46;

/// Set-and-recheck threshold for the §5.4 wrap-into-past guard. Largest
/// legitimate `(CCR3 - CNT) & 0xFFFF` value expected at `schedule` time —
/// `RDT_max + slot_offset_max`. Anything above means modular subtraction
/// wrapped backward (we scheduled in the past). Estimated conservative:
/// 32 k ticks (~667 µs). Measured: TBD (bench).
pub const SCHEDULE_WRAP_GUARD_TICKS: u16 = 0x8000;
