//! Hand-tuned timing constants for the DXL fire scheduler. Units are HCLK
//! ticks at 48 MHz (1 µs = 48 ticks). ISR fire latency decomposes as
//! `PFIC_ENTRY_TICKS + <path>_ENTRY_TICKS` so the trap+prologue side and
//! the per-path body side can be re-tuned independently.

/// PFIC trap entry latency for the `SysTick` core interrupt: qingke V2 HPE
/// register push + the `SysTick` trampoline + the `on_systick` prologue up
/// to the first body instruction.
pub(super) const PFIC_ENTRY_TICKS: u32 = 26;

/// Periodic-catchup ISR body work: state load + `accumulate_snoop` + the
/// post-fold `set_cmp`/`set_irq`. Subtracted from `last_catchup_tick` in
/// `start_fast_after` so the busy-wait fold loop begins on the intended
/// wall-clock anchor rather than after entry latency. Significant at 3M
/// where the combined entry latency approaches one byte_time.
pub(super) const CATCHUP_ENTRY_TICKS: u32 = 154;

/// Plain reply body work: `fire_now` (TX_EN flip + DMA CH4 enable) plus
/// the USART start-bit latch. Baud-independent — the per-baud `bt/4` term
/// in `plain_fire_advance_ticks` covers the USART BRR-sync + listener
/// IDLE-detect pipeline separately.
pub(super) const PLAIN_ENTRY_TICKS: u32 = 154;

/// Fast chain `TxArmed` body work: same fire path as plain, plus the
/// `set_phase` + state-load overhead the catchup-to-`TxArmed` handoff adds
/// before `fire_now`.
pub(super) const FAST_ENTRY_TICKS: u32 = 154;
