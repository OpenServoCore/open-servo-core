//! Empirically-measured timing constants. Each value here comes from scope
//! captures or static cycle counting, not from datasheet timing. Re-measure
//! when the affecting code path, PFIC priorities, or HCLK frequency change.
//!
//! Units are HCLK ticks at 48 MHz (1 µs = 48 ticks). The per-constant docs
//! state how each was measured.
//!
//! ISR fire latency decomposes as `PFIC_ENTRY_TICKS + <path>_ENTRY_TICKS`:
//! the first captures qingke V2 HPE + SysTick trampoline + `on_systick`
//! prologue up to the first body instruction; the latter captures only the
//! post-prologue body work for each reply path (plain / fast / catchup).
//! Call sites sum the two so each can be re-measured independently.

/// PFIC trap entry latency for the `SysTick` core interrupt: qingke V2
/// hardware prologue (HPE pushes x1, x5-x7, x10-x15 to sp[-48..0]) +
/// `SysTick` trampoline + `__qingke_rt_SysTick` indirect jump + the
/// `on_systick` Rust prologue up to (and including) the `stat_high()` GPIO
/// BSHR write that opens the bench measurement window.
///
/// Counted statically from `osc-dev-v006-app.elf` with
/// `tools/dxl-bench/scripts/cycle_count.py --start SysTick --highcode`:
/// 30 instruction cycles + 7 HPE cycles (qingke V2 manual §3.4 names the
/// register set but does not publish a cycle count; 7 is a conservative
/// estimate). Re-run the script after any toolchain or `.highcode` layout
/// change.
pub const PFIC_ENTRY_TICKS: u32 = 37;

/// `t_catchup_entry` — periodic-catchup ISR body work between
/// `stat_high()` and the CRC-fold loop. Does NOT include PFIC trap entry
/// or `on_systick` prologue (see [`PFIC_ENTRY_TICKS`]); covers
/// `clear_match` + state-load + match arm + `accumulate_snoop` +
/// post-fold `set_cmp` / `set_irq` / past-tick check.
///
/// Bench 2026-06-02 (`--features scope`, INJ=1 single-body): with
/// `stat_high`/`stat_low` bracketing the ISR body and `dbg_high`/`dbg_low`
/// bracketing only the CRC-fold work, 5 shots gave stat-minus-dbg widths
/// of 38.70−36.39, 38.51−36.25, 39.64−37.34, 39.72−37.42, 38.50−36.22 µs.
/// Mean = 2.29 µs, σ = 0.03 µs → 110 HCLK ticks at 48 MHz. See
/// `docs/dxl-fast-chain-crc-walkloop.md` §1.1.
///
/// Summed with [`PFIC_ENTRY_TICKS`] and subtracted from `last_catchup_tick`
/// in `dxl_fast::start_fast_after` so the body's CRC fold begins at the
/// intended wall-clock anchor instead of `anchor + entry_latency`.
/// Significant at 3 M where the combined entry latency approaches one
/// `byte_time`.
pub const CATCHUP_ENTRY_TICKS: u32 = 110;

/// `t_plain_entry` — plain reply path body work: `on_systick` match arm +
/// `set_state(Idle)` + `fire_now` (TX_EN flip + DMA CH4 enable) + USART
/// start-bit latch. Does NOT include PFIC trap entry or prologue (see
/// [`PFIC_ENTRY_TICKS`]).
///
/// Baud-independent; the per-baud `bt/4` term in `plain_fire_advance_ticks`
/// covers the USART BRR-sync + listener IDLE-detect pipeline separately.
///
/// **Pending bench measurement** with the dbg/stat-pulse methodology
/// described for [`CATCHUP_ENTRY_TICKS`]. Current value carries forward the
/// prior CT-tuned default (836 Q8.8 µs = 156 ticks ≈ 3.25 µs at 48 MHz)
/// from before the runtime tunable was removed and includes some PFIC
/// entry; re-measurement will strip the prologue cost out.
pub const PLAIN_ENTRY_TICKS: u32 = 156;

/// `t_fast_entry` — Fast chain (last-slave) `TxArmed` body work: same path
/// as [`PLAIN_ENTRY_TICKS`] for the Fast reply. Empirically larger than
/// plain because the catchup-to-`TxArmed` handoff runs `set_phase` and a
/// past-tick check before `fire_now`. Does NOT include PFIC trap entry or
/// prologue (see [`PFIC_ENTRY_TICKS`]). Summed with [`PFIC_ENTRY_TICKS`]
/// and subtracted from `t_fire` in `dxl_fast::start_fast_after`.
///
/// **Pending bench measurement** with the same methodology as
/// [`CATCHUP_ENTRY_TICKS`]. Current value carries forward the prior
/// CT-tuned default (884 Q8.8 µs = 165 ticks ≈ 3.44 µs at 48 MHz) from
/// before the runtime tunable was removed and includes some PFIC entry;
/// re-measurement will strip the prologue cost out.
pub const FAST_ENTRY_TICKS: u32 = 165;
