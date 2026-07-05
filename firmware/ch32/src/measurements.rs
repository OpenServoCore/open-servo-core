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
/// Measured 0 (2026-07-04): at K=8, tool-tune-tx-start put the p0.1 wire
/// excess at −0.15 µs — the first bits of an on-time send emerge BEFORE
/// CCR2 raises TX_EN. Single-target replies lead with `0xFF`, where a
/// clipped start bit is harmless (zero `DroppedLeadingFf` in 20 k), but
/// FAST slot emissions lead with `0x00`/err bytes and the direction-
/// switch clip garbles them (bench: intermittent truncated Last
/// emissions at 3M with on-time fires). K=0 aims CCR4 at the deadline
/// itself; the silicon request→DR path (~20-30 ticks) supplies the
/// positive TX_EN lead. Median excess ≈ +0.6 µs, p0.1 ≈ +0.02 µs.
pub const TX_KICKOFF_FLOOR_TICKS: u16 = 0;

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

/// Entry-latency compensation subtracted from the packet-end estimate, in
/// HCLK ticks (baud-independent). Bench-calibrated to 0 and pinned there
/// by the drain-flavor split: ByteBatch drains (HT/TC lands on the CRC
/// byte) carry near-zero latency (min wire excess +0.17 µs at 3M), while
/// IDLE drains carry ~5 µs — any nonzero comp would fire ByteBatch-drained
/// replies BEFORE the deadline. The residual median excess (~5 µs at RDT
/// scale) is protocol-irrelevant; per-flavor comps are the lever if that
/// ever changes.
pub const PACKET_END_ENTRY_COMP_TICKS: u32 = 0;

mod fast_last {
    /// Early bias for the checkpoint wake CMP, in HCLK ticks: the wake
    /// targets `window_end − this` so the body arrives BEFORE the
    /// predecessor's trailing CRC bytes and spins them in. Sized to
    /// worst-case SysTick CMP-vector entry latency (measured typical
    /// ~240 ticks, with ~5 µs-class jitter on that vector) plus margin —
    /// waking early costs a short NDTR-poll spin; waking late eats the
    /// patch-vs-fetch margin directly. Bench-tune downward once the
    /// pickup timing is characterized.
    pub const FAST_CRC_WAKE_LEAD_TICKS: u16 = 500;

    /// Run the wake body inline at the status-start observation when the
    /// slot's wire-start deadline is within this many ticks (an RXNE-ISR
    /// occupancy budget of 50 µs at HCLK = 48 MHz). Short windows at high
    /// baud can't afford the ~500-tick SysTick pend → body-entry dispatch;
    /// the inline body spins at most until the predecessor window closes.
    /// Windows longer than this bound ride the wake CMP.
    pub const FAST_LAST_INLINE_FOLD_HORIZON_TICKS: u16 = 2400;
}
pub use fast_last::{FAST_CRC_WAKE_LEAD_TICKS, FAST_LAST_INLINE_FOLD_HORIZON_TICKS};
