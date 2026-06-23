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
/// Measured: 35 (tool-tune-tx-start, 4 × 20000 pings @ 3 Mbaud; mean of
/// K_recommended at 3-tick safety margin). Deep tail (1 run in 4) can
/// land 1 HCLK tick before deadline — acceptable at 3 Mbaud where slot
/// RDT absorbs it.
pub const TX_START_ENTRY_TICKS: u16 = 35;

/// Set-and-recheck threshold for the §5.4 wrap-into-past guard. Largest
/// legitimate `(CCR3 - CNT) & 0xFFFF` value expected at `schedule` time —
/// `RDT_max + slot_offset_max`. Anything above means modular subtraction
/// wrapped backward (we scheduled in the past). Estimated conservative:
/// 32 k ticks (~667 µs). Measured: TBD (bench).
pub const SCHEDULE_WRAP_GUARD_TICKS: u16 = 0x8000;

mod fast_last {
    /// SysTick CMP-match → catchup-body fold-start latency, in HCLK ticks.
    /// Driver subtracts this from every grid anchor offset before handing
    /// the CMP target to the scheduler so the body's actual fold-start
    /// lands on the formula's intended wall-clock anchor.
    ///
    /// SysTick PFIC entry (~5 µs) is larger than TIM2's CC vector entry
    /// (~1 µs) because the SysTick vector doesn't get qingke's fast-vector
    /// optimization. Per `docs/dxl-hw-timed-transport.md` §10.6 (open
    /// questions): estimate ~240 ticks (~5 µs at HCLK = 48 MHz); confirm
    /// under bench load.
    pub const FAST_LAST_ENTRY_TICKS: u16 = 240;

    /// Periodic-walk grid step, in predecessor wire bytes. Each fold body
    /// folds up to this many bytes of newly-classified residue before
    /// re-arming the next CMP one grid step ahead. See doc §10.6.2.
    pub const FAST_LAST_BYTES_PER_INTERVAL: u16 = 15;

    /// Pre-start fold residue cap, in predecessor wire bytes. The final
    /// busy-wait exits at `t_prior_end − GUARD × byte_ticks`, leaving up
    /// to this many predecessor bytes for the TX-start body's tail to
    /// fold inline. At 3M GUARD=1 keeps `patch_crc` ahead of CH4's
    /// DMA-prefetch on byte[n − 2]. See doc §10.6.2 and
    /// `[[crc_patch_deadline_miss_semantics]]`.
    pub const FAST_LAST_GUARD_BYTES: u16 = 1;
}
pub use fast_last::{FAST_LAST_BYTES_PER_INTERVAL, FAST_LAST_ENTRY_TICKS, FAST_LAST_GUARD_BYTES};
