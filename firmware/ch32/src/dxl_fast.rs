use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;

use ch32_metapac::USART1;

use dxl_protocol::crc16_continue;

#[cfg(feature = "scope")]
use crate::hal::gpio::Level;
use crate::hal::{dma, gpio, systick, usart};
use crate::measurements::{
    CATCHUP_ENTRY_TICKS, FAST_ENTRY_TICKS, PFIC_ENTRY_TICKS, PLAIN_ENTRY_TICKS,
};
#[cfg(feature = "scope")]
use crate::statics::DXL_DBG_PIN;
use crate::statics::{
    DXL_BYTE_TIME_TICKS, DXL_BYTES_PER_US_Q16, DXL_RX_BUF, DXL_RX_BUF_LEN, DXL_TX_BUF, DXL_TX_EN,
    FIRE_ADVANCE_FINE_TICKS, SHARED,
};

/// Which request end tick detection + fire path a given reply takes.
#[allow(dead_code)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum DxlTimingStrategy {
    /// Per-byte RXNE publishes the request end tick; used when IDLE lags RDT
    /// (low baud).
    LengthCounted,
    /// Single IDLE IRQ, backdated by one char-time; used at high baud.
    IdleBackdated,
    /// Fast Sync/Bulk last-slave — snoop predecessor bytes for chain CRC.
    ChainedFastStatus,
}

/// Errors the chain CRC path can flag; each variant maps 1:1 to a persistent
/// counter in `TelemetryDxlLink`.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum FastChainFault {
    /// `set_phase` rejected a transition not in the legal table — FSM bug.
    IllegalTransition,
    /// Snoop walk saw a delta inconsistent with the TC wrap count.
    #[allow(dead_code)]
    UnexpectedByteCount,
    /// At TxArmed (post-straggle-walk), observed predecessor bytes fell short
    /// of predicted. The Q16 reciprocal carries ±1 LSB rounding noise, so
    /// boundary cases may trip without a real predecessor miss.
    PreviousSlotTimeout,
    /// Fire CMP ran more than one byte-time past `fire_tick`.
    SlotTimingMiss,
    /// `patch_crc` finished with ≤ 2 bytes of DMA prefetch slack left.
    CrcPatchDeadlineMiss,
    /// USART STATR.ORE — RX outpaced DMA drain.
    DmaOverrun,
    /// USART STATR.PE — parity error on the wire.
    ParityError,
    /// USART STATR.FE — framing error (stop bit missing).
    FramingError,
    /// USART STATR.NE — noise on the wire mid-bit.
    NoiseError,
}

/// Where in the chain reply timeline we currently sit.
#[allow(dead_code)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum FastChainPhase {
    /// Periodic SysTick CMP every `BYTES_PER_INTERVAL × byte_time` ticks.
    /// Each body walks whatever's in the ring; the last tick (aligned to
    /// `last_catchup_tick`) runs a busy-wait fold loop until
    /// `walk_deadline`, then hands off to `TxArmed` at `fire_tick`.
    PeriodicCatchup,
    /// Pre-fire walk done; SysTick CMP armed at `fire_tick`. The TxArmed
    /// body fires, runs the post-fire walk, and patches CRC.
    TxArmed,
    /// Fire CMP fired; TX shifting and straggle walk in progress.
    TxStreaming,
    /// Trailing CRC bytes patched into the TX buffer.
    CrcPatched,
    /// USART TC drained the reply; bus released.
    Done,
    /// Reply aborted; the matching counter has been incremented.
    Fault(FastChainFault),
}

/// Scheduler state for the next reply this slave will send.
#[derive(Copy, Clone)]
pub enum ReplyState {
    /// Nothing armed.
    Idle,
    /// SysTick CMP armed for a non-snoop reply (Ping, Read, Sync/Bulk slot,
    /// Fast First/Middle/Only).
    Plain,
    /// SysTick CMP armed for a Fast last-slave reply with snoop.
    Chain {
        phase: FastChainPhase,
        /// SysTick value at which TX must start shifting.
        fire_tick: u32,
        /// SysTick value at which the last periodic catchup body fires —
        /// the *start* of a busy-wait loop that absorbs the predecessor
        /// window. Earlier periodic ticks land at `last_catchup_tick − k ×
        /// interval`.
        last_catchup_tick: u32,
        /// SysTick value at which the pre-fire walk-loop times out, leaving
        /// the trailing GUARD bytes for the post-fire walk. Equals
        /// `t_prior_end − t_guard`.
        walk_deadline: u32,
        /// Ring index up to which `bulk_crc` has folded predecessor bytes.
        snoop_head: u16,
        /// Rolling CRC over predecessor bytes observed so far.
        bulk_crc: u16,
        /// Total predecessor byte count — the walk-loop's exit target. In
        /// practice the loop exits via `walk_deadline` with `n_pred − GUARD`
        /// folded; post-fire absorbs the remaining GUARD bytes.
        expected_predecessor_bytes: u16,
        bytes_walked: u32,
    },
}

static STATE: SyncUnsafeCell<ReplyState> = SyncUnsafeCell::new(ReplyState::Idle);

/// Per-state dispatch handler called from `on_systick`. Recomputed on every
/// state/phase transition so the SysTick ISR pays one indirect call instead
/// of a data-dependent match chain.
type DispatchFn = fn();

static DISPATCH: SyncUnsafeCell<DispatchFn> = SyncUnsafeCell::new(body_noop);

#[inline(always)]
fn dispatch_for(s: &ReplyState) -> DispatchFn {
    match s {
        ReplyState::Idle => body_noop,
        ReplyState::Plain => body_plain_fire,
        ReplyState::Chain {
            phase: FastChainPhase::PeriodicCatchup,
            ..
        } => body_chain_catchup,
        ReplyState::Chain {
            phase: FastChainPhase::TxArmed,
            ..
        } => body_chain_fast_fire,
        ReplyState::Chain { .. } => body_noop,
    }
}

const _: () = assert!(DXL_RX_BUF_LEN.is_power_of_two() && DXL_RX_BUF_LEN <= u16::MAX as usize + 1);
const RX_MASK_U16: u16 = (DXL_RX_BUF_LEN as u16).wrapping_sub(1);
const RX_MASK_U32: u32 = (DXL_RX_BUF_LEN - 1) as u32;

/// Target bytes folded per periodic catchup body. Sized so each body's CRC
/// walk (~0.5 µs/byte) stays well under one kernel tick: 15 × 0.5 = 7.5 µs.
/// Per-baud interval falls out as `BYTES_PER_INTERVAL × byte_time_ticks`.
const BYTES_PER_INTERVAL: u32 = 15;

/// Predecessor bytes the pre-fire walk-loop leaves on the wire for the
/// post-fire walk. Floor of 2 keeps `t_guard > t_fast_entry` at every
/// supported baud, so `walk_deadline` always precedes `fire_tick` — the
/// `TxArmed` handoff yields cleanly and never needs an inline re-entry
/// (see body_chain_catchup post-fold handoff). At 3M the resulting gap is
/// `2 × byte_time − t_fast_entry` ≈ 3.25 µs, enough for one PFIC-LOW ISR
/// (ADC TC) to slip in without pushing fire past pirate's 1-char_time
/// IDLE threshold.
const GUARD_BYTES: u16 = 2;
const _: () = assert!(GUARD_BYTES >= 2, "GUARD_BYTES < 2 puts 3M in recursive-fire mode");

/// Plain advance adds a per-baud `bt/4` (≈ 2.5 bit-times) term covering
/// the USART BRR-sync + listener IDLE-detect pipeline on top of the
/// baud-independent [`PLAIN_ENTRY_TICKS`]. Without it, a single-baud
/// calibration only lands Plain at zero on the calibration baud. Fast
/// keeps the flat form because its bench metric cancels listener lag.
#[inline(always)]
fn plain_fire_advance_ticks() -> u32 {
    let bt = DXL_BYTE_TIME_TICKS.load(Ordering::Relaxed);
    let latency = PFIC_ENTRY_TICKS + PLAIN_ENTRY_TICKS + bt / 4;
    let fine = FIRE_ADVANCE_FINE_TICKS.load(Ordering::Relaxed) as i32;
    (latency as i32 + fine).clamp(0, u16::MAX as i32) as u32
}

/// Fast chain fire shares the `FIRE_ADVANCE_FINE_TICKS` knob with plain so
/// host-side calibration dials both paths together. Per-path independence
/// would only matter if their residuals diverge.
#[inline(always)]
fn fast_fire_advance_ticks() -> u32 {
    let latency = PFIC_ENTRY_TICKS + FAST_ENTRY_TICKS;
    let fine = FIRE_ADVANCE_FINE_TICKS.load(Ordering::Relaxed) as i32;
    (latency as i32 + fine).clamp(0, u16::MAX as i32) as u32
}

/// Catchup anchor backdate: same shared fine-trim knob, so the last-catchup
/// body lands on the same wall-clock offset as fire. Without this, a non-zero
/// fine trim shifts fire without shifting the busy-wait fold loop, breaking
/// the `walk_deadline = t_prior_end − t_guard` invariant.
#[inline(always)]
fn catchup_backdate_ticks() -> u32 {
    let base = PFIC_ENTRY_TICKS + CATCHUP_ENTRY_TICKS;
    let fine = FIRE_ADVANCE_FINE_TICKS.load(Ordering::Relaxed) as i32;
    (base as i32 + fine).clamp(0, u16::MAX as i32) as u32
}

/// TX_EN and DMA CH4 stay off so the bus remains in RX through any preceding
/// snoop window; `fire_now` flips both at the slot deadline.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn arm_tx() -> bool {
    // SAFETY: caller has &mut Ch32Bus (sole writer to DXL_TX_BUF).
    let len = unsafe { (*DXL_TX_BUF.get()).as_slice().len() };
    if len == 0 {
        return false;
    }
    // `dma::set_count` requires CH4 disabled; arming over an in-flight TX
    // silently keeps the old NDTR and corrupts the wire mid-byte.
    if dma::is_enabled(dma::Channel::CH4) {
        return false;
    }
    dma::set_count(dma::Channel::CH4, len as u16);
    usart::clear_tc(USART1);
    usart::set_dma_tx(USART1, true);
    usart::set_tc_irq(USART1, true);
    true
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn fire_now() {
    // SAFETY: written once during bring-up before USART1 IRQ unmask.
    if let Some(t) = unsafe { *DXL_TX_EN.get() } {
        gpio::set_level(t.pin, t.tx_level);
    }
    dma::enable(dma::Channel::CH4);
}

/// Scope trigger pulse on `DXL_DBG_PIN`. Moved between sites as the bench
/// experiment requires. Compile-time no-op without `--features scope`.
#[inline(always)]
fn dbg_pulse() {
    #[cfg(feature = "scope")]
    // SAFETY: written once at bring-up before any ISR can read.
    if let Some(p) = unsafe { *DXL_DBG_PIN.get() } {
        gpio::set_level(p, Level::High);
        gpio::set_level(p, Level::Low);
    }
}

/// Plain (non-snoop) reply: schedule a fire at `request_end + delay_us`.
/// SysTick CMP owns the deadline.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn start_plain_after(request_end_tick: u32, delay_us: u32) {
    systick::set_irq(false);
    systick::clear_match();

    let needed = delay_us
        .saturating_mul(systick::TICKS_PER_US)
        .saturating_sub(plain_fire_advance_ticks());

    if systick::ticks().wrapping_sub(request_end_tick) >= needed {
        if arm_tx() {
            fire_now();
        }
        return;
    }
    if !arm_tx() {
        cancel();
        return;
    }

    let fire_tick = request_end_tick.wrapping_add(needed);
    set_state(ReplyState::Plain);
    systick::set_cmp(fire_tick);
    systick::set_irq(true);

    // CNTIF latches only on CNT==CMP up-count; if CNT crossed the deadline
    // before CMP was written, the next match is ~89 s away on wrap.
    if systick::ticks().wrapping_sub(request_end_tick) >= needed {
        on_systick();
    }
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn start_fast_after(request_end_tick: u32, fire_q88_us: u32, snoop_from: Option<u32>) {
    systick::set_irq(false);
    systick::clear_match();

    if !arm_tx() {
        cancel();
        return;
    }

    // Q8.8 µs × ticks/µs = Q8.8 ticks; shift to whole ticks. The 1/256 µs
    // (≈ 4 ns) resolution is tighter than HCLK, so wire-time math doesn't
    // lose sub-tick precision to integer-µs flooring at high baud.
    let fire_needed_ticks = ((fire_q88_us as u64 * systick::TICKS_PER_US as u64) >> 8) as u32;
    let fire_needed = fire_needed_ticks.saturating_sub(fast_fire_advance_ticks());
    let fire_tick = request_end_tick.wrapping_add(fire_needed);

    let (snoop_head, n_pred) = match snoop_from {
        Some(rx_start) => ((rx_start & RX_MASK_U32) as u16, predict_n_pred(fire_q88_us)),
        None => (0, 0),
    };

    // Anchor the START of the pre-fire busy-wait fold loop. The loop times
    // out at `t_prior_end − t_guard`; the trailing GUARD bytes belong to
    // the post-fire walk. For n_pred ≤ GUARD the walk-loop no-ops and
    // post-fire absorbs everything, so collapse to a single CMP at fire_tick.
    let byte_time = byte_time_ticks_now();
    let rdt_ticks = unsafe {
        (&raw const (*SHARED.table.config.get()).comms.return_delay_2us).read_volatile() as u32
            * 2
            * systick::TICKS_PER_US
    };
    let interval = catchup_interval_ticks();
    let t_guard = byte_time.saturating_mul(GUARD_BYTES as u32);
    let t_prior_duration = byte_time.saturating_mul(n_pred.saturating_sub(1) as u32);
    let t_prior_start = request_end_tick
        .wrapping_add(rdt_ticks)
        .wrapping_add(byte_time);
    let t_prior_end = request_end_tick
        .wrapping_add(rdt_ticks)
        .wrapping_add(byte_time.saturating_mul(n_pred as u32));
    let walk_deadline = t_prior_end.wrapping_sub(t_guard);
    let last_catchup_tick = if n_pred > GUARD_BYTES {
        fire_tick
            .wrapping_sub(interval.min(t_prior_duration))
            .wrapping_sub(t_guard)
            .wrapping_sub(catchup_backdate_ticks())
    } else {
        fire_tick
    };

    // Step back by `interval` from the anchor until one more step would land
    // before `t_prior_start + interval` — i.e., the first catchup straddles
    // wire-start. Wrapping-safe via signed offset against t_prior_start.
    let mut delta = last_catchup_tick.wrapping_sub(t_prior_start);
    while (delta as i32) >= interval as i32 {
        delta = delta.wrapping_sub(interval);
    }
    let mut first_catchup_tick = t_prior_start.wrapping_add(delta);
    // Floor at `now` so we don't schedule into the past.
    let now = systick::ticks();
    if (first_catchup_tick.wrapping_sub(now) as i32) < 0 {
        first_catchup_tick = now;
    }

    set_state(ReplyState::Chain {
        phase: FastChainPhase::PeriodicCatchup,
        fire_tick,
        last_catchup_tick,
        walk_deadline,
        snoop_head,
        bulk_crc: 0,
        expected_predecessor_bytes: n_pred,
        bytes_walked: 0,
    });

    systick::set_cmp(first_catchup_tick);
    systick::set_irq(true);
    if (systick::ticks().wrapping_sub(first_catchup_tick) as i32) >= 0 {
        on_systick();
    }
}

/// Predict predecessor wire bytes for the post-fire fold target and the
/// `PreviousSlotTimeout` floor. The wire window is the *predecessor's*
/// transmission span `fire_q88_us − rdt_q88_us` — not the SysTick scheduling
/// window, which would balloon n_pred by ~5× since it doesn't subtract RDT.
#[inline]
fn predict_n_pred(fire_q88_us: u32) -> u16 {
    // SAFETY: u8 access — no tearing. Boot-seeded or main-loop-written; ISR
    // context here only reads.
    let rdt_us = unsafe {
        (&raw const (*SHARED.table.config.get()).comms.return_delay_2us).read_volatile() as u32 * 2
    };
    let rdt_q88_us = rdt_us << 8;
    let bytes_per_us_q16 = DXL_BYTES_PER_US_Q16.load(Ordering::Relaxed);
    predict_n_pred_pure(fire_q88_us, rdt_q88_us, bytes_per_us_q16)
}

#[inline]
fn predict_n_pred_pure(fire_q88_us: u32, rdt_q88_us: u32, bytes_per_us_q16: u32) -> u16 {
    if bytes_per_us_q16 == 0 {
        return 0;
    }
    let wire_q88 = fire_q88_us.saturating_sub(rdt_q88_us);
    // Q8.8 µs × Q16.16 bytes/µs = Q24.24 bytes. Round-half-up at the >>24
    // boundary so a window that exactly straddles a byte edge doesn't trip
    // PreviousSlotTimeout a byte short.
    let prod = wire_q88 as u64 * bytes_per_us_q16 as u64;
    let n = (prod + (1u64 << 23)) >> 24;
    n.min(u16::MAX as u64) as u16
}

/// Per-baud periodic catchup interval in HCLK ticks. Each body walks
/// `BYTES_PER_INTERVAL` worth of wire bytes regardless of baud.
#[inline(always)]
fn catchup_interval_ticks() -> u32 {
    BYTES_PER_INTERVAL.saturating_mul(byte_time_ticks_now())
}

/// SysTick ISR entry. Path-uniform: load the cached dispatch fn and call it.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn on_systick() {
    systick::clear_match();
    systick::set_irq(false);
    // SAFETY: USART1 / SysTick share PFIC HIGH and never preempt each other;
    // DISPATCH access is uncontested across these handlers.
    let dispatch = unsafe { *DISPATCH.get() };
    dispatch();
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
#[unsafe(no_mangle)]
fn body_noop() {}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
#[unsafe(no_mangle)]
fn body_plain_fire() {
    fire_now();
    set_state(ReplyState::Idle);
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
#[unsafe(no_mangle)]
fn body_chain_catchup() {
    accumulate_snoop();
    // SAFETY: see on_systick. Dispatch == body_chain_catchup implies Chain
    // with PeriodicCatchup phase; fall through to no-op otherwise.
    let (fire_tick, last_catchup_tick, walk_deadline, n_pred) = unsafe {
        match *STATE.get() {
            ReplyState::Chain {
                fire_tick,
                last_catchup_tick,
                walk_deadline,
                expected_predecessor_bytes,
                ..
            } => (
                fire_tick,
                last_catchup_tick,
                walk_deadline,
                expected_predecessor_bytes,
            ),
            _ => return,
        }
    };

    let interval = catchup_interval_ticks();
    let now = systick::ticks();
    let byte_time = byte_time_ticks_now();
    let half_byte = (byte_time / 2) as i32;
    let is_last_catchup = (now.wrapping_sub(last_catchup_tick) as i32) >= -half_byte;

    if !is_last_catchup {
        // Snap to the anchor if `now + interval` would overshoot. ISR-entry
        // drift accumulates across intermediate ticks; anchoring the final
        // tick keeps fire on the wall-clock.
        let candidate = now.wrapping_add(interval);
        let next_tick = if (candidate.wrapping_sub(last_catchup_tick) as i32) >= -half_byte {
            last_catchup_tick
        } else {
            candidate
        };
        systick::set_cmp(next_tick);
        systick::set_irq(true);
        if (systick::ticks().wrapping_sub(next_tick) as i32) >= 0 {
            on_systick();
        }
        return;
    }

    // Last catchup. Busy-wait + fold until `n_pred` reached or
    // `walk_deadline` hits. The deadline exit is normal
    // (`bytes_walked = n_pred − GUARD`); post-fire absorbs the trailing
    // GUARD bytes.
    wait_and_fold_until(n_pred as u32, walk_deadline);
    dbg_pulse();

    // Hand off to TxArmed: arm SysTick at fire_tick, return. The
    // `GUARD_BYTES ≥ 2` invariant guarantees `walk_deadline < fire_tick`,
    // so the CPU yields cleanly and no past-tick re-entry is needed here.
    set_phase(FastChainPhase::TxArmed);
    systick::set_cmp(fire_tick);
    systick::set_irq(true);
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
#[unsafe(no_mangle)]
fn body_chain_fast_fire() {
    // ── Critical path. patch_crc must land bytes [n-2, n-1] of TX_BUF before
    // CH4's read cursor reaches them.
    fire_now();
    // SAFETY: see on_systick. Dispatch == body_chain_fast_fire implies Chain
    // with TxArmed phase; fall through to no-op otherwise.
    let (fire_tick, walk_deadline, n_pred) = unsafe {
        match *STATE.get() {
            ReplyState::Chain {
                fire_tick,
                walk_deadline,
                expected_predecessor_bytes,
                ..
            } => (fire_tick, walk_deadline, expected_predecessor_bytes),
            _ => return,
        }
    };
    if n_pred > 0 {
        // Post-fire walks bytes that arrive between walk_deadline and
        // t_prior_end. Add one byte_time of slack past t_prior_end for DMA
        // latch latency on the final byte.
        let byte_time = byte_time_ticks_now();
        let post_deadline =
            walk_deadline.wrapping_add(byte_time.saturating_mul((GUARD_BYTES + 1) as u32));
        wait_and_fold_until(n_pred as u32, post_deadline);
    }
    patch_crc();

    // ── Off the critical path: diagnostics + FSM transitions.
    if dma::remaining(dma::Channel::CH4) <= 2 {
        report_fault(FastChainFault::CrcPatchDeadlineMiss);
    }
    if current_bytes_walked() < n_pred as u32 {
        report_fault(FastChainFault::PreviousSlotTimeout);
    }
    let byte_time = byte_time_ticks_now();
    let now = systick::ticks();
    if (now.wrapping_sub(fire_tick) as i32) > byte_time as i32 {
        report_fault(FastChainFault::SlotTimingMiss);
    }
    set_phase(FastChainPhase::TxStreaming);
    set_phase(FastChainPhase::CrcPatched);
}

#[inline(always)]
fn byte_time_ticks_now() -> u32 {
    DXL_BYTE_TIME_TICKS.load(Ordering::Relaxed)
}

#[inline(always)]
fn current_bytes_walked() -> u32 {
    // SAFETY: see on_systick.
    match unsafe { *STATE.get() } {
        ReplyState::Chain { bytes_walked, .. } => bytes_walked,
        _ => 0,
    }
}

/// RXNE handler stub. The chain path never enables RXNEIE; the periodic
/// catchup body owns all snoop work. Kept callable so `irq.rs`'s USART1
/// dispatch doesn't fork on framing mode.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
pub fn on_rxne() {}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn cancel() {
    systick::set_irq(false);
    systick::clear_match();
    // Tail RXNE may still be live if cancel races a fire that never
    // reached TxArmed. Mask unconditionally — no-op when already off.
    usart::set_rxne_irq(USART1, false);
    set_state(ReplyState::Idle);
}

#[inline(always)]
pub fn report_dma_overrun() {
    report_fault(FastChainFault::DmaOverrun);
}

#[inline(always)]
pub fn report_parity_error() {
    report_fault(FastChainFault::ParityError);
}

#[inline(always)]
pub fn report_framing_error() {
    report_fault(FastChainFault::FramingError);
}

#[inline(always)]
pub fn report_noise_error() {
    report_fault(FastChainFault::NoiseError);
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn patch_crc() {
    // SAFETY: see on_systick. Sole writer to DXL_TX_BUF in ISR context.
    unsafe {
        let buf = &mut *DXL_TX_BUF.get();
        let n = buf.len();
        let payload_end = n.saturating_sub(2);
        let seed = match *STATE.get() {
            ReplyState::Chain { bulk_crc, .. } => bulk_crc,
            _ => 0,
        };
        let crc = crc16_continue(seed, &buf[..payload_end]);
        let bytes = crc.to_le_bytes();
        let slice = buf.as_mut_slice();
        slice[n - 2] = bytes[0];
        slice[n - 1] = bytes[1];
    }
}

/// Busy-wait + fold until `bytes_walked ≥ target` OR `now ≥ deadline`.
/// Both pre-fire and post-fire walks share this primitive — they differ
/// only in the deadline (absolute SysTick tick).
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn wait_and_fold_until(target: u32, deadline: u32) {
    loop {
        accumulate_snoop();
        if current_bytes_walked() >= target {
            return;
        }
        if (systick::ticks().wrapping_sub(deadline) as i32) >= 0 {
            return;
        }
    }
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn accumulate_snoop() {
    let write_pos = current_rx_write_pos();
    // SAFETY: see on_systick.
    unsafe {
        if let ReplyState::Chain {
            snoop_head,
            bulk_crc,
            bytes_walked,
            ..
        } = &mut *STATE.get()
        {
            if *snoop_head == write_pos {
                return;
            }
            let ring = &*DXL_RX_BUF.get();
            let prior = *snoop_head;
            *bulk_crc = ring_crc(*bulk_crc, ring, prior, write_pos);
            let delta = write_pos.wrapping_sub(prior) & RX_MASK_U16;
            *bytes_walked = bytes_walked.wrapping_add(delta as u32);
            *snoop_head = write_pos;
        }
    }
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn ring_crc(seed: u16, ring: &[u8], head: u16, write_pos: u16) -> u16 {
    if head == write_pos {
        return seed;
    }
    let start = head as usize;
    let end = write_pos as usize;
    if start < end {
        crc16_continue(seed, &ring[start..end])
    } else {
        let mid = crc16_continue(seed, &ring[start..]);
        crc16_continue(mid, &ring[..end])
    }
}

#[inline(always)]
fn current_rx_write_pos() -> u16 {
    let remaining = dma::remaining(dma::Channel::CH5);
    (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining) & RX_MASK_U16
}

#[inline(always)]
fn set_state(new: ReplyState) {
    let dispatch = dispatch_for(&new);
    // SAFETY: see on_systick. Both stores are uncontested under PFIC HIGH.
    unsafe {
        *STATE.get() = new;
        *DISPATCH.get() = dispatch;
    }
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn set_phase(new: FastChainPhase) {
    // SAFETY: see on_systick. Calls outside Chain are silent no-ops so
    // set_phase chains after cancel() don't double-fault on re-entry.
    let illegal = unsafe {
        match &mut *STATE.get() {
            ReplyState::Chain { phase, .. } => {
                if is_legal_transition(*phase, new) {
                    *phase = new;
                    false
                } else {
                    true
                }
            }
            _ => false,
        }
    };
    if illegal {
        report_fault(FastChainFault::IllegalTransition);
        cancel();
        return;
    }
    // Refresh dispatch from the now-updated state. No-op when state isn't Chain.
    unsafe { *DISPATCH.get() = dispatch_for(&*STATE.get()) };
}

#[inline]
fn is_legal_transition(from: FastChainPhase, to: FastChainPhase) -> bool {
    use FastChainPhase::*;
    if matches!(to, Fault(_)) {
        return true;
    }
    matches!(
        (from, to),
        (PeriodicCatchup, TxArmed)
            | (TxArmed, TxStreaming)
            | (TxStreaming, CrcPatched)
            | (CrcPatched, Done)
    )
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn report_fault(f: FastChainFault) {
    unsafe {
        let link = &raw mut (*SHARED.table.telemetry.get()).link;
        let counter: *mut u32 = match f {
            FastChainFault::IllegalTransition => &raw mut (*link).illegal_transition,
            FastChainFault::UnexpectedByteCount => &raw mut (*link).unexpected_byte_count,
            FastChainFault::PreviousSlotTimeout => &raw mut (*link).previous_slot_timeout,
            FastChainFault::SlotTimingMiss => &raw mut (*link).slot_timing_miss,
            FastChainFault::CrcPatchDeadlineMiss => &raw mut (*link).crc_patch_deadline_miss,
            FastChainFault::DmaOverrun => &raw mut (*link).dma_overrun,
            FastChainFault::ParityError => &raw mut (*link).parity_error,
            FastChainFault::FramingError => &raw mut (*link).framing_error,
            FastChainFault::NoiseError => &raw mut (*link).noise_error,
        };
        counter.write_volatile(counter.read_volatile().wrapping_add(1));
    }
}

#[cfg(test)]
mod tests {
    use super::{FastChainPhase, is_legal_transition, predict_n_pred_pure, ring_crc};
    use dxl_protocol::crc16;

    const RING: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

    // 3 Mbaud reference: bytes_per_us_q16 = round((3 << 16) / 10) = 19661.
    const BYTES_PER_US_Q16_3M: u32 = 19661;
    // 2 Mbaud: round((2 << 16) / 10) = 13107.
    const BYTES_PER_US_Q16_2M: u32 = 13107;
    // 1 Mbaud: round((1 << 16) / 10) = 6554.
    const BYTES_PER_US_Q16_1M: u32 = 6554;
    // Default RDT on rev_b boot (comms.return_delay_2us = 125 → 250 µs).
    const RDT_US_DEFAULT: u32 = 250;

    #[test]
    fn empty_walk_returns_seed_unchanged() {
        assert_eq!(ring_crc(0x1234, &RING, 3, 3), 0x1234);
        assert_eq!(ring_crc(0, &RING, 0, 0), 0);
    }

    #[test]
    fn non_wrap_walk_matches_contiguous_crc() {
        assert_eq!(ring_crc(0, &RING, 2, 5), crc16(&[0x30, 0x40, 0x50]));
    }

    #[test]
    fn wrap_walk_stitches_tail_and_head() {
        assert_eq!(ring_crc(0, &RING, 6, 2), crc16(&[0x70, 0x80, 0x10, 0x20]));
    }

    #[test]
    fn seed_chaining_matches_single_pass() {
        let mid = ring_crc(0, &RING, 1, 4);
        let chained = ring_crc(mid, &RING, 4, 7);
        assert_eq!(chained, ring_crc(0, &RING, 1, 7));
    }

    #[test]
    fn seed_chaining_survives_wrap_boundary() {
        let pre = ring_crc(0, &RING, 5, 0);
        let post = ring_crc(pre, &RING, 0, 3);
        assert_eq!(post, ring_crc(0, &RING, 5, 3));
    }

    #[test]
    fn legal_transitions_form_chain() {
        use FastChainPhase::*;
        assert!(is_legal_transition(PeriodicCatchup, TxArmed));
        assert!(is_legal_transition(TxArmed, TxStreaming));
        assert!(is_legal_transition(TxStreaming, CrcPatched));
        assert!(is_legal_transition(CrcPatched, Done));
    }

    #[test]
    fn illegal_transitions_rejected() {
        use FastChainPhase::*;
        assert!(!is_legal_transition(PeriodicCatchup, TxStreaming));
        assert!(!is_legal_transition(TxArmed, PeriodicCatchup));
        assert!(!is_legal_transition(CrcPatched, TxArmed));
        assert!(!is_legal_transition(Done, TxArmed));
    }

    // -- predict_n_pred_pure ----------------------------------------------

    // Q8.8 µs helpers; predict_n_pred_pure now takes both args in Q8.8.
    const fn q88(us: u32) -> u32 {
        us << 8
    }
    // 11 bytes × 10/3 µs/byte = 36.667 µs → 9386 q88 (Q16.16 floored to Q8.8).
    const Q88_3M_INJ1: u32 = (11 * BYTES_PER_US_Q16_3M_INV) >> 8;
    // Per-baud us_per_byte_q16 mirrored from osc-core's regions::config.
    const BYTES_PER_US_Q16_3M_INV: u32 = 218453; // (10 × 1e6 × 65536) / 3e6 floored
    const BYTES_PER_US_Q16_2M_INV: u32 = 327680; // (10 × 1e6 × 65536) / 2e6
    const BYTES_PER_US_Q16_1M_INV: u32 = 655360; // (10 × 1e6 × 65536) / 1e6
    const RDT_Q88_DEFAULT: u32 = q88(RDT_US_DEFAULT);

    #[test]
    fn predict_n_pred_3m_inj1_last() {
        // 3M INJ=1 Last: bytes_before(1) = FAST_SLOT0_PREFIX(10) + 1 = 11.
        // wire_q88 = bytes_to_us_q88(11, 3M) = (11 × 218453) >> 8 = 9386.
        assert_eq!(Q88_3M_INJ1, 9386);
        let fire_q88 = RDT_Q88_DEFAULT + Q88_3M_INJ1;
        assert_eq!(
            predict_n_pred_pure(fire_q88, RDT_Q88_DEFAULT, BYTES_PER_US_Q16_3M),
            11
        );
    }

    #[test]
    fn predict_n_pred_3m_inj4_dut1_last() {
        // 3M INJ=4 dut=1 Last: bytes_before = 10 + 1 + 3×(2+1) = 20.
        // wire = 20 × 10/3 = 66.667 µs → (20 × 218453) >> 8 = 17066 q88.
        let wire_q88 = (20 * BYTES_PER_US_Q16_3M_INV) >> 8;
        let fire_q88 = RDT_Q88_DEFAULT + wire_q88;
        assert_eq!(
            predict_n_pred_pure(fire_q88, RDT_Q88_DEFAULT, BYTES_PER_US_Q16_3M),
            20
        );
    }

    #[test]
    fn predict_n_pred_2m_matches_doubled_byte_time() {
        // 2M INJ=4 dut=1 Last: wire = 100 µs (exact at 2M).
        let wire_q88 = (20 * BYTES_PER_US_Q16_2M_INV) >> 8;
        let fire_q88 = RDT_Q88_DEFAULT + wire_q88;
        assert_eq!(
            predict_n_pred_pure(fire_q88, RDT_Q88_DEFAULT, BYTES_PER_US_Q16_2M),
            20
        );
    }

    #[test]
    fn predict_n_pred_1m_matches_quartered_byte_time() {
        // 1M INJ=1 Last: wire = 110 µs (exact at 1M).
        let wire_q88 = (11 * BYTES_PER_US_Q16_1M_INV) >> 8;
        let fire_q88 = RDT_Q88_DEFAULT + wire_q88;
        assert_eq!(
            predict_n_pred_pure(fire_q88, RDT_Q88_DEFAULT, BYTES_PER_US_Q16_1M),
            11
        );
    }

    #[test]
    fn predict_n_pred_zero_window_when_fire_before_rdt() {
        // Fire scheduled inside RDT (degenerate): wire saturates to 0.
        assert_eq!(
            predict_n_pred_pure(q88(100), RDT_Q88_DEFAULT, BYTES_PER_US_Q16_3M),
            0
        );
    }

    #[test]
    fn predict_n_pred_zero_bytes_per_us_returns_zero() {
        // Boot state before store_baud_derived runs.
        let fire_q88 = RDT_Q88_DEFAULT + Q88_3M_INJ1;
        assert_eq!(predict_n_pred_pure(fire_q88, RDT_Q88_DEFAULT, 0), 0);
    }
}
