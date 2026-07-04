//! Bench-measured silicon timings. Estimates today (`docs/dxl-hw-timed-transport.md`
//! §5.1 / §5.4 + disassembly-sized hot path); replaced with measured values
//! during the post-M3 bench validation pass. Centralised here so a single
//! edit re-tunes the firmware — the provider / ISR bodies don't reference
//! raw numbers.
//!
//! Successor to the legacy `measurements.rs` (drained in `c95be9f1` along
//! with the SysTick fire path). Constants don't carry forward — the TIM2
//! OC fire path has its own entry-latency profile.

/// CC4-match → first wire bit, in TIM2 ticks at HCLK=48 MHz. Subtracted
/// from `deadline_tick` to back-date CCR4 so the wire-bit lands on time.
///
/// The path is pure hardware — no PFIC entry, no ISR body: CC4 compare
/// match → CC4DE DMA request → DMA1_CH7 one-word write of the EN=1 config
/// into DMA1_CH4.CR → CH4 fetches byte 0 → USART DR (doc §5). Sized to
/// the *minimum* path so the wire-bit always lands at or after
/// `deadline_tick`: too-large K starts the shift before TX_EN's CC2 match
/// raises the driver — the missing-first-FF signature per
/// [[tx-en-lead-via-k]].
///
/// Spike-measured upper bound: 24 ticks including the poll-loop read
/// granularity (`osc-dev-v006-bringup` `tim2_ch4_oc_dma_kickoff`, RESULTS
/// slot 6); the true request→DR path is a fraction of that. Start
/// conservative-small (wire-bit late is the safe direction) —
/// re-calibrate with tool-tune-tx-start when the bench returns.
pub const TX_KICKOFF_FLOOR_TICKS: u16 = 8;

/// Set-and-recheck threshold for the §5.4 wrap-into-past guard. Largest
/// legitimate `(CCR4 - CNT) & 0xFFFF` value expected at `schedule` time —
/// `RDT_max + slot_offset_max`. Anything above means modular subtraction
/// wrapped backward (we scheduled in the past). Estimated conservative:
/// 32 k ticks (~667 µs). Measured: TBD (bench).
pub const SCHEDULE_WRAP_GUARD_TICKS: u16 = 0x8000;

/// CNT lead for the past-deadline retry: after the §5.4 recheck detects a
/// missed CC4 match, CCR4 re-aims at `CNT + this` so a REAL compare event
/// fires the kickoff ASAP — TIM2's SWEVGR.CC4G is dead silicon on V006
/// (spike-verified: CC4IF never latches). Must cover the CNT-read →
/// CCR4-write latency (a few ticks) with margin; the whole lead is ~1.3 µs
/// of extra latency on an already-late send.
pub const KICKOFF_RETRY_LEAD_TICKS: u16 = 64;

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
}
pub use fast_last::{FAST_LAST_BYTES_PER_INTERVAL, FAST_LAST_ENTRY_TICKS};
