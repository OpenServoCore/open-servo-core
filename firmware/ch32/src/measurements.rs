//! Bench-measured silicon timings. Doc estimates today
//! (`docs/dxl-hw-timed-transport.md` §5.1 / §5.4); replaced with measured
//! values during the post-M3 bench validation pass. Centralised here so a
//! single edit re-tunes the firmware — the provider / ISR bodies don't
//! reference raw numbers.
//!
//! Successor to the legacy `measurements.rs` (drained in `c95be9f1` along
//! with the SysTick fire path). Constants don't carry forward — the TIM2
//! OC fire path has its own entry-latency profile.

// Consumed by the `DxlTxScheduler` provider, landed in a subsequent commit.
#![allow(dead_code)]

/// CC3-match → first wire bit, in TIM2 ticks at HCLK=48 MHz. Subtracted
/// from `deadline_tick` to back-date CCR3 so the wire-bit lands on time.
/// Doc §5.1 estimate ~2.5 µs ≈ 120 ticks. Measured: TBD (bench).
pub const TX_START_ENTRY_TICKS: u16 = 120;

/// CC2-match → first wire bit, in TIM2 ticks. Must exceed
/// `TX_START_ENTRY_TICKS` by the bus-driver settle margin (~1–2 µs).
/// Doc §5.3 estimate ~3–4 µs ≈ 200 ticks. Measured: TBD (bench).
pub const TX_EN_SETUP_TICKS: u16 = 200;

/// Set-and-recheck threshold for the §5.4 wrap-into-past guard. Largest
/// legitimate `(CCR3 - CNT) & 0xFFFF` value expected at `schedule` time —
/// `RDT_max + slot_offset_max`. Anything above means modular subtraction
/// wrapped backward (we scheduled in the past). Estimated conservative:
/// 32 k ticks (~667 µs). Measured: TBD (bench).
pub const SCHEDULE_WRAP_GUARD_TICKS: u16 = 0x8000;
