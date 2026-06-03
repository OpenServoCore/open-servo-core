use core::sync::atomic::Ordering;

use crate::hal::systick;
use crate::statics::SHARED;

use super::calibration::{
    CATCHUP_ENTRY_TICKS, FAST_ENTRY_TICKS, PFIC_ENTRY_TICKS, PLAIN_ENTRY_TICKS,
};
use super::isr::{arm_tx, fire_now, on_systick};
use super::state::{FastChainPhase, ReplyState, cancel, set_state};
use super::statics::{
    DXL_BYTE_TIME_TICKS, DXL_BYTES_PER_US_Q16, FIRE_ADVANCE_FINE_TICKS, RX_MASK_U32,
};

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
pub(super) const GUARD_BYTES: u16 = 2;
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
    let byte_time = DXL_BYTE_TIME_TICKS.load(Ordering::Relaxed);
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
pub(super) fn catchup_interval_ticks() -> u32 {
    BYTES_PER_INTERVAL.saturating_mul(DXL_BYTE_TIME_TICKS.load(Ordering::Relaxed))
}

#[cfg(test)]
mod tests {
    use super::predict_n_pred_pure;

    // 3 Mbaud reference: bytes_per_us_q16 = round((3 << 16) / 10) = 19661.
    const BYTES_PER_US_Q16_3M: u32 = 19661;
    // 2 Mbaud: round((2 << 16) / 10) = 13107.
    const BYTES_PER_US_Q16_2M: u32 = 13107;
    // 1 Mbaud: round((1 << 16) / 10) = 6554.
    const BYTES_PER_US_Q16_1M: u32 = 6554;
    // Default RDT on rev_b boot (comms.return_delay_2us = 125 → 250 µs).
    const RDT_US_DEFAULT: u32 = 250;

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
