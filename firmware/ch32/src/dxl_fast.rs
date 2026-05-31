use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;

use ch32_metapac::USART1;

use dxl_protocol::crc16_continue;

use crate::hal::{
    dma,
    gpio::{self, Level},
    systick, usart,
};
use crate::statics::{
    DXL_BYTE_TIME_TICKS, DXL_BYTES_PER_US_Q16, DXL_DBG_PIN, DXL_RX_BUF, DXL_RX_BUF_LEN,
    DXL_STAT_PIN, DXL_TX_BUF, FIRE_ADVANCE_FINE_TICKS, SHARED,
};
#[cfg(feature = "dxl-systick-fire")]
use crate::statics::DXL_TX_EN;
#[cfg(feature = "dxl-systick-fire")]
use crate::statics::{TX_FAST_LATENCY_TICKS, TX_PLAIN_LATENCY_TICKS};

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
    /// At WaitingSwitch CMP, observed predecessor bytes fell short of
    /// predicted. The Q16 reciprocal carries ±1 LSB rounding noise, so
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
    /// Armed, WaitingSwitch CMP pending, no DMA TC wrap seen yet.
    CatchupArmed,
    /// At least one TC wrap has folded a full ring into `bulk_crc`.
    Snoop,
    /// WaitingSwitch CMP body running its catch-up walk to current NDTR.
    Catchup,
    /// Catch-up done; fire CMP pending.
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
    /// SysTick (+ optionally DMA CH5 TC) armed for a Fast last-slave reply.
    Chain {
        phase: FastChainPhase,
        /// SysTick value at which TX must start shifting.
        fire_tick: u32,
        /// Ring index up to which `bulk_crc` has folded predecessor bytes.
        snoop_head: u16,
        /// Rolling CRC over predecessor bytes observed so far.
        bulk_crc: u16,
        /// Predicted predecessor byte count; floor for `PreviousSlotTimeout`.
        expected_predecessor_bytes: u16,
        bytes_walked: u32,
        /// `bytes_walked` threshold for masking RXNEIE in the RXNE-tail tier
        /// (chain-crc.md §14.4). `u16::MAX` = tail tier disabled for this
        /// reply (the common case — covered by switch + walk-at-fire alone).
        rxne_tail_at: u16,
    },
}

static STATE: SyncUnsafeCell<ReplyState> = SyncUnsafeCell::new(ReplyState::Idle);

const _: () = assert!(DXL_RX_BUF_LEN.is_power_of_two() && DXL_RX_BUF_LEN <= u16::MAX as usize + 1);
const RX_MASK_U16: u16 = (DXL_RX_BUF_LEN as u16).wrapping_sub(1);
const RX_MASK_U32: u32 = (DXL_RX_BUF_LEN - 1) as u32;

/// Gap between WaitingSwitch CMP and the fire CMP. The switch walk drains
/// whatever bytes have accumulated since the last DMA HT/TC fold; the fire
/// ISR walks the M µs straggle and patches CRC inside the TX DMA prefetch
/// window.
///
/// Bench-measured CRC throughput on V006 is ~0.5 µs/byte (flash-resident
/// `crc16_continue` + ring-index bookkeeping). The chain-CRC doc's original
/// 0.1 µs/byte estimate was 5× off (see chain-crc.md §14.1).
///
/// **Upper bound (Constraint B):** straggle walk + patch must fit the smallest
/// reply's TX prefetch slack. At M=50, 3 Mbaud, n=8 reply (slack 20 µs):
/// straggle ≈ 15 B × 0.5 = 7.5 µs + patch ~2 µs + fire ~1 µs ≈ 10.5 µs
/// total, leaving ~9.5 µs margin. Smaller replies (n ∈ {5..7}) with
/// short wire_window fall through this path and are caught by the
/// RXNE-tail tier (see `decide_rxne_tail` and chain-crc.md §14.4).
///
/// **Lower bound (Constraint A):** switch walk must fit M. With HT folding
/// bytes at the 256-B midpoint, the catchup walk is bounded by ≤ 256 B at
/// 0.5 µs/B = ~128 µs worst-case — switch is sized to cover only the
/// per-iteration residue, not a full ring. Typical: inj=4 catchup walks
/// ~4 B (~2 µs), inj=128 walks ~50 µs, inj=256+ leans on HT to keep the
/// residual under ~20 µs.
const SWITCH_MARGIN_US: u32 = 50;

/// Per-byte cost of the snoop CRC walk, in HCLK ticks. Bench-measured at
/// ~0.5 µs/B on V006 (24 ticks @ 48 MHz). Used by the RXNE-tail decision
/// in `decide_rxne_tail` to estimate the walk-at-fire residue.
const CRC_WALK_TICKS_PER_BYTE: u32 = 24;

/// Post-fire path floor in ticks: fire_now (~1.2 µs) + glue + patch_crc
/// (~1.9 µs) + busy-wait setup. ~4.2 µs total. The fixed cost the
/// RXNE-tail decision adds to the slack-budget check.
const POST_FIRE_FLOOR_TICKS: u32 = 200;

/// Slack margin reserved for SysTick CMP jitter (PFIC trap-entry variance).
const POST_FIRE_JITTER_TICKS: u32 = 24;

/// Bytes left for the post-fire walk after RXNE-tail pre-walks the rest.
/// 2 lets the predecessor's final byte arrive in the busy-wait window
/// without the RXNE-vs-SysTick collision the chain-CRC doc §4.2 warned
/// against.
const RXNE_TAIL_GUARD_BYTES: u16 = 2;

/// Upper bound on the post-fire busy-wait, in predecessor byte-times.
/// 3 byte-times = ~10 µs at 3 Mbaud — long enough to cover the worst
/// case (fire arrives before the last predecessor byte's stop bit) and
/// short enough that a missing/dead predecessor fails open quickly.
const RXNE_TAIL_WAIT_BYTES: u32 = 3;

#[cfg(feature = "dxl-systick-fire")]
#[inline(always)]
fn plain_fire_advance_ticks() -> u32 {
    fire_advance_ticks_for(TX_PLAIN_LATENCY_TICKS.load(Ordering::Relaxed))
}

#[cfg(feature = "dxl-systick-fire")]
#[inline(always)]
fn fast_fire_advance_ticks() -> u32 {
    fire_advance_ticks_for(TX_FAST_LATENCY_TICKS.load(Ordering::Relaxed))
}

/// Under hw-fire the per-path latency atomics are gone (CT field reserved,
/// runtime no longer tunable). The pipeline from `arm` to wire start-bit
/// is a fixed hardware sequence — `HW_FIRE_PIPELINE_TICKS` plus the
/// shared `clock_fine_trim_us` residual. Same signature so the systick
/// CMP-scheduling math elsewhere stays mode-agnostic.
#[cfg(feature = "dxl-hw-fire")]
#[inline(always)]
fn plain_fire_advance_ticks() -> u32 {
    fire_advance_ticks_for(crate::dxl_hw_fire::HW_FIRE_PIPELINE_TICKS as u16)
}

#[cfg(feature = "dxl-hw-fire")]
#[inline(always)]
fn fast_fire_advance_ticks() -> u32 {
    fire_advance_ticks_for(crate::dxl_hw_fire::HW_FIRE_PIPELINE_TICKS as u16)
}

#[inline(always)]
fn fire_advance_ticks_for(latency: u16) -> u32 {
    let fine = FIRE_ADVANCE_FINE_TICKS.load(Ordering::Relaxed) as i32;
    (latency as i32 + fine).clamp(0, u16::MAX as i32) as u32
}

/// TX_EN and DMA CH4 stay off so the bus remains in RX through any preceding
/// snoop window; `fire_now` flips both at the slot deadline.
#[cfg(feature = "dxl-systick-fire")]
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

/// Under hw-fire the bus-disable order flips: CH4 is pre-enabled here with
/// DMAT=0 so the moment TIM2 OPM (DMA Ch2) flips DMAT to 1, the USART
/// starts requesting bytes with no software in the loop. TC handler clears
/// DMAT + disables CH4, restoring the disabled-CH4 precondition for the
/// next arm.
#[cfg(feature = "dxl-hw-fire")]
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn arm_tx() -> bool {
    let len = unsafe { (*DXL_TX_BUF.get()).as_slice().len() };
    if len == 0 {
        return false;
    }
    if dma::is_enabled(dma::Channel::CH4) {
        return false;
    }
    dma::set_count(dma::Channel::CH4, len as u16);
    usart::clear_tc(USART1);
    dma::enable(dma::Channel::CH4);
    usart::set_tc_irq(USART1, true);
    true
}

#[cfg(feature = "dxl-systick-fire")]
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn fire_now() {
    // SAFETY: written once during bring-up before USART1 IRQ unmask.
    if let Some(t) = unsafe { *DXL_TX_EN.get() } {
        gpio::set_level(t.pin, t.tx_level);
    }
    dma::enable(dma::Channel::CH4);
}

/// Under hw-fire, software fire = arm TIM2 OPM with the smallest CCR so it
/// drives TX_EN HIGH and DMAT=1 from hardware, removing PFIC trap-entry
/// jitter from the wire timing. The TIM2 internal pipeline
/// (`HW_FIRE_PIPELINE_TICKS`) is the irreducible wire-start latency.
#[cfg(feature = "dxl-hw-fire")]
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn fire_now() {
    crate::dxl_hw_fire::prepare_fire();
    crate::dxl_hw_fire::arm(1);
}

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
pub fn start_fast_after(request_end_tick: u32, fire_us: u32, snoop_from: Option<u32>) {
    systick::set_irq(false);
    systick::clear_match();

    if !arm_tx() {
        cancel();
        return;
    }

    let fire_needed = fire_us
        .saturating_mul(systick::TICKS_PER_US)
        .saturating_sub(fast_fire_advance_ticks());
    let fire_tick = request_end_tick.wrapping_add(fire_needed);

    let snoop_head = match snoop_from {
        Some(rx_start) => (rx_start & RX_MASK_U32) as u16,
        None => 0,
    };

    let predict = match snoop_from {
        Some(_) => predict_chain(fire_us),
        None => ChainPredict::none(),
    };
    let tx_len = unsafe { (*DXL_TX_BUF.get()).as_slice().len() };
    let rxne_tail_at = decide_rxne_tail(&predict, tx_len);
    let use_rxne_tail = rxne_tail_at != u16::MAX;

    // RXNE-tail covers the "small snoop, tight slack" corner where switch
    // CMP would land before the predecessor's first byte (wire_window < M)
    // and walk-at-fire alone overruns slack. In that case skip CatchupArmed
    // entirely — the tail handler keeps bulk_crc current per byte and the
    // TxArmed body only walks the GUARD-byte residue.
    let (initial_phase, cmp_tick) = match snoop_from {
        None => (FastChainPhase::TxArmed, fire_tick),
        Some(_) if use_rxne_tail => (FastChainPhase::TxArmed, fire_tick),
        Some(_) if fire_us <= SWITCH_MARGIN_US => (FastChainPhase::TxArmed, fire_tick),
        Some(_) => {
            let switch_margin_ticks = SWITCH_MARGIN_US.saturating_mul(systick::TICKS_PER_US);
            (
                FastChainPhase::CatchupArmed,
                fire_tick.wrapping_sub(switch_margin_ticks),
            )
        }
    };

    set_state(ReplyState::Chain {
        phase: initial_phase,
        fire_tick,
        snoop_head,
        bulk_crc: 0,
        expected_predecessor_bytes: predict.n_pred,
        bytes_walked: 0,
        rxne_tail_at,
    });

    if matches!(initial_phase, FastChainPhase::CatchupArmed) {
        dma::clear_tc_flag(dma::Channel::CH5);
        dma::clear_ht_flag(dma::Channel::CH5);
        dma::set_tcie(dma::Channel::CH5, true);
        dma::set_htie(dma::Channel::CH5, true);
    }

    if use_rxne_tail {
        usart::set_rxne_irq(USART1, true);
    }

    systick::set_cmp(cmp_tick);
    systick::set_irq(true);
    if systick::ticks().wrapping_sub(request_end_tick) >= cmp_tick.wrapping_sub(request_end_tick) {
        on_systick();
    }
}

#[derive(Copy, Clone)]
struct ChainPredict {
    /// Predicted predecessor wire bytes.
    n_pred: u16,
    /// Predicted predecessor wire transmission span (fire_us minus RDT), in µs.
    wire_window_us: u32,
}

impl ChainPredict {
    #[inline(always)]
    const fn none() -> Self {
        Self {
            n_pred: 0,
            wire_window_us: 0,
        }
    }
}

#[inline]
fn predict_chain(fire_us: u32) -> ChainPredict {
    // SAFETY: u8 access — no tearing. Boot-seeded or main-loop-written; ISR
    // context here only reads.
    let rdt_us = unsafe {
        (&raw const (*SHARED.table.config.get()).comms.return_delay_2us).read_volatile() as u32 * 2
    };
    let bytes_per_us_q16 = DXL_BYTES_PER_US_Q16.load(Ordering::Relaxed);
    predict_chain_pure(fire_us, rdt_us, bytes_per_us_q16)
}

#[inline]
fn predict_chain_pure(fire_us: u32, rdt_us: u32, bytes_per_us_q16: u32) -> ChainPredict {
    let wire_window_us = fire_us.saturating_sub(rdt_us);
    let n_pred = if bytes_per_us_q16 == 0 {
        0
    } else {
        // Predecessor reply spans (request_end + rdt) .. (request_end + fire_us).
        // PreviousSlotTimeout fires at TxArmed (post-straggle-walk), so we
        // compare bytes_walked against the full predecessor reply length
        // here. Floors twice round-trip (dispatcher's bytes_to_us + this one),
        // so the actual byte count can be up to one higher than n_pred —
        // `decide_rxne_tail_pure` biases its walk estimate by +1 to absorb.
        ((wire_window_us.saturating_mul(bytes_per_us_q16)) >> 16).min(u16::MAX as u32) as u16
    };
    ChainPredict {
        n_pred,
        wire_window_us,
    }
}

/// Per chain-crc.md §14.4: arm the RXNE-tail tier iff switch can't help
/// (wire_window < M) AND walk-at-fire alone would overrun TX prefetch slack.
/// Returns the `bytes_walked` threshold for masking RXNEIE, or `u16::MAX`
/// when the tier stays dormant for this reply (the common case).
#[inline]
fn decide_rxne_tail(predict: &ChainPredict, tx_len: usize) -> u16 {
    decide_rxne_tail_pure(predict, tx_len, byte_time_ticks_now())
}

#[inline]
fn decide_rxne_tail_pure(predict: &ChainPredict, tx_len: usize, byte_time_ticks: u32) -> u16 {
    if predict.n_pred == 0 || tx_len < 4 {
        return u16::MAX;
    }
    if predict.wire_window_us >= SWITCH_MARGIN_US {
        return u16::MAX;
    }
    if byte_time_ticks == 0 {
        return u16::MAX;
    }
    let slack_ticks = (tx_len as u32 - 2).saturating_mul(byte_time_ticks);
    // `predict.n_pred` floors twice on the dispatcher → predict round-trip
    // (bytes_to_us floor, then µs → bytes floor), so true predecessor count
    // is up to one byte higher. Bias the walk estimate up by one byte so the
    // gate doesn't undershoot on tight cases like 3M Last 1B (true n_pred
    // = 11, predicted = 10).
    let walk_at_fire_ticks = (predict.n_pred as u32 + 1)
        .saturating_mul(CRC_WALK_TICKS_PER_BYTE)
        .saturating_add(POST_FIRE_FLOOR_TICKS)
        .saturating_add(POST_FIRE_JITTER_TICKS);
    if walk_at_fire_ticks <= slack_ticks {
        return u16::MAX;
    }
    // Use the same +1 bias for rxne_tail_at so on_rxne masks at the *actual*
    // (one higher) byte count rather than the floored prediction — otherwise
    // mask fires one byte too early and post-fire walks GUARD+1 bytes.
    (predict.n_pred + 1)
        .saturating_sub(RXNE_TAIL_GUARD_BYTES)
        .max(1)
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn on_systick() {
    systick::clear_match();
    systick::set_irq(false);

    // SAFETY: USART1 / SysTick / DMA1_CH5 share PFIC HIGH and never preempt
    // each other; STATE access is uncontested across these handlers.
    let snapshot = unsafe { *STATE.get() };
    match snapshot {
        ReplyState::Idle => {}
        ReplyState::Plain => {
            set_state(ReplyState::Idle);
            fire_now();
        }
        ReplyState::Chain {
            phase,
            fire_tick,
            expected_predecessor_bytes,
            rxne_tail_at,
            ..
        } => match phase {
            FastChainPhase::CatchupArmed | FastChainPhase::Snoop => {
                dbg_high();
                set_phase(FastChainPhase::Catchup);
                accumulate_snoop();
                set_phase(FastChainPhase::TxArmed);
                systick::set_cmp(fire_tick);
                systick::set_irq(true);
                dbg_low();
                let now = systick::ticks();
                if (now.wrapping_sub(fire_tick) as i32) >= 0 {
                    on_systick();
                }
            }
            FastChainPhase::Catchup => {
                // Defensive: only reached if a prior CMP body returned without
                // rescheduling. Re-arm against the fire deadline.
                set_phase(FastChainPhase::TxArmed);
                systick::set_cmp(fire_tick);
                systick::set_irq(true);
            }
            FastChainPhase::TxArmed => {
                let now = systick::ticks();
                let byte_time_ticks = byte_time_ticks_now();
                if (now.wrapping_sub(fire_tick) as i32) > byte_time_ticks as i32 {
                    report_fault(FastChainFault::SlotTimingMiss);
                }
                dma::set_tcie(dma::Channel::CH5, false);
                dma::set_htie(dma::Channel::CH5, false);
                // Mask RXNEIE early: if the tail handler hadn't reached the
                // threshold by fire (e.g. fire fired one byte early), keeping
                // it on through TxStreaming would burn ISR entries during the
                // CRC-patch race.
                if rxne_tail_at != u16::MAX {
                    usart::set_rxne_irq(USART1, false);
                }
                set_phase(FastChainPhase::TxStreaming);
                // Fire before any CRC work: DMA CH4 has only one byte of TX
                // prefetch, so the wire-side jitter window is just the time
                // from here to `dma::enable`. The CRC patch below races a
                // (payload_end - 1) byte-time slack against DMA's read cursor.
                // Bench breakdown of the 7 µs post-fire spike: DBG wraps
                // fire_now, STAT wraps accumulate_snoop, DBG wraps patch_crc.
                dbg_high();
                fire_now();
                dbg_low();
                stat_high();
                if rxne_tail_at != u16::MAX {
                    // RXNE pre-walked all but GUARD bytes; busy-wait the
                    // straggle in.
                    wait_and_accumulate_tail(expected_predecessor_bytes);
                } else {
                    accumulate_snoop();
                }
                stat_low();
                dbg_high();
                patch_crc();
                dbg_low();
                if dma::remaining(dma::Channel::CH4) <= 2 {
                    report_fault(FastChainFault::CrcPatchDeadlineMiss);
                }
                if current_bytes_walked() < expected_predecessor_bytes as u32 {
                    report_fault(FastChainFault::PreviousSlotTimeout);
                }
                set_phase(FastChainPhase::CrcPatched);
            }
            FastChainPhase::TxStreaming
            | FastChainPhase::CrcPatched
            | FastChainPhase::Done
            | FastChainPhase::Fault(_) => {}
        },
    }
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

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
pub fn on_dma1_ch5() {
    let ht = dma::is_ht_flag(dma::Channel::CH5);
    let tc = dma::is_tc_flag(dma::Channel::CH5);

    // Unconditional clears: a stuck IFCR bit re-asserts the IRQ on next unmask
    // even if STATE has moved past the snoop window. HT clears first so the
    // late-IRQ "both flags pending" case still ends at write_pos=0 after TC.
    if ht {
        dma::clear_ht_flag(dma::Channel::CH5);
    }
    if tc {
        dma::clear_tc_flag(dma::Channel::CH5);
    }

    let phase_now = unsafe {
        match &*STATE.get() {
            ReplyState::Chain { phase, .. } => Some(*phase),
            _ => None,
        }
    };
    let Some(phase_now) = phase_now else { return };
    if !matches!(
        phase_now,
        FastChainPhase::CatchupArmed | FastChainPhase::Snoop | FastChainPhase::TxArmed
    ) {
        return;
    }

    // TC implies the ring just wrapped; walk to end. Otherwise (HT only) walk
    // to the midpoint. snoop_head may have already advanced past the target —
    // skip walk and don't move head backward.
    let target = if tc {
        DXL_RX_BUF_LEN
    } else {
        DXL_RX_BUF_LEN / 2
    };
    let new_head = if tc {
        0u16
    } else {
        (DXL_RX_BUF_LEN / 2) as u16
    };

    unsafe {
        if let ReplyState::Chain {
            snoop_head,
            bulk_crc,
            bytes_walked,
            ..
        } = &mut *STATE.get()
        {
            let start = *snoop_head as usize;
            if start < target {
                let ring = &*DXL_RX_BUF.get();
                let walked = target - start;
                *bulk_crc = crc16_continue(*bulk_crc, &ring[start..target]);
                *bytes_walked = bytes_walked.wrapping_add(walked as u32);
                *snoop_head = new_head;
            } else if tc {
                // Late TC after accumulate_snoop crossed the wrap; just reset.
                *snoop_head = 0;
            }
        }
    }

    if matches!(phase_now, FastChainPhase::CatchupArmed) {
        set_phase(FastChainPhase::Snoop);
    }
}

/// Per-byte snoop fold during the predecessor's wire window. Folds bytes
/// arrived since the last entry into `bulk_crc`, advances `snoop_head`, and
/// masks RXNEIE once `bytes_walked` crosses the `rxne_tail_at` threshold —
/// post-fire walk-at-fire owns the final GUARD bytes (chain-crc.md §14.6).
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
pub fn on_rxne() {
    // Cheap self-gate: most on_usart1 entries (IDLE/TC) don't care about
    // RXNE-tail and return immediately.
    let active = unsafe {
        matches!(
            &*STATE.get(),
            ReplyState::Chain {
                phase,
                rxne_tail_at,
                ..
            } if *rxne_tail_at != u16::MAX
                && matches!(
                    phase,
                    FastChainPhase::CatchupArmed
                        | FastChainPhase::Snoop
                        | FastChainPhase::Catchup
                        | FastChainPhase::TxArmed
                )
        )
    };
    if !active {
        return;
    }

    dbg_high();
    let write_pos = current_rx_write_pos();
    let mask_now = unsafe {
        if let ReplyState::Chain {
            snoop_head,
            bulk_crc,
            bytes_walked,
            rxne_tail_at,
            ..
        } = &mut *STATE.get()
        {
            if *snoop_head != write_pos {
                let ring = &*DXL_RX_BUF.get();
                let prior = *snoop_head;
                *bulk_crc = ring_crc(*bulk_crc, ring, prior, write_pos);
                let delta = write_pos.wrapping_sub(prior) & RX_MASK_U16;
                *bytes_walked = bytes_walked.wrapping_add(delta as u32);
                *snoop_head = write_pos;
            }
            *bytes_walked >= *rxne_tail_at as u32
        } else {
            false
        }
    };
    if mask_now {
        usart::set_rxne_irq(USART1, false);
    }
    dbg_low();
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn cancel() {
    systick::set_irq(false);
    systick::clear_match();
    dma::set_tcie(dma::Channel::CH5, false);
    dma::set_htie(dma::Channel::CH5, false);
    // Tail-tier arm may still be live if cancel races a fire that never
    // reached TxArmed (e.g. host abort mid-snoop). Mask unconditionally —
    // it's a no-op when already off, and the framing-RXNE owner doesn't
    // exist on V006 yet (Phase B work).
    usart::set_rxne_irq(USART1, false);
    #[cfg(feature = "dxl-hw-fire")]
    crate::dxl_hw_fire::cancel();
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

/// RXNE-tail post-fire: walk what's already in the ring and busy-wait the
/// trailing predecessor byte(s) — bounded by `RXNE_TAIL_WAIT_BYTES` so a
/// dead/missing predecessor degrades to `PreviousSlotTimeout` instead of
/// hanging the bus. Only called when `rxne_tail_at != u16::MAX`.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn wait_and_accumulate_tail(expected: u16) {
    accumulate_snoop();
    if current_bytes_walked() >= expected as u32 {
        return;
    }
    let byte_time = byte_time_ticks_now();
    if byte_time == 0 {
        return;
    }
    let deadline =
        systick::ticks().wrapping_add(byte_time.saturating_mul(RXNE_TAIL_WAIT_BYTES));
    loop {
        accumulate_snoop();
        if current_bytes_walked() >= expected as u32 {
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
fn dbg_high() {
    // SAFETY: written once at bring-up before any ISR can read.
    if let Some(p) = unsafe { *DXL_DBG_PIN.get() } {
        gpio::set_level(p, Level::High);
    }
}

#[inline(always)]
fn dbg_low() {
    if let Some(p) = unsafe { *DXL_DBG_PIN.get() } {
        gpio::set_level(p, Level::Low);
    }
}

#[inline(always)]
fn stat_high() {
    // SAFETY: written once at bring-up before any ISR can read.
    if let Some(p) = unsafe { *DXL_STAT_PIN.get() } {
        gpio::set_level(p, Level::High);
    }
}

#[inline(always)]
fn stat_low() {
    if let Some(p) = unsafe { *DXL_STAT_PIN.get() } {
        gpio::set_level(p, Level::Low);
    }
}

#[inline(always)]
fn set_state(new: ReplyState) {
    // SAFETY: see on_systick.
    unsafe { *STATE.get() = new };
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn set_phase(new: FastChainPhase) {
    // SAFETY: see on_systick. The borrow on STATE is released before the
    // error path runs, since cancel() re-enters STATE. Calls outside Chain
    // are silent no-ops, not faults — set_phase chains after `cancel()` need
    // to be safe to re-enter without double-faulting.
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
    }
}

#[inline]
fn is_legal_transition(from: FastChainPhase, to: FastChainPhase) -> bool {
    use FastChainPhase::*;
    if matches!(to, Fault(_)) {
        return true;
    }
    matches!(
        (from, to),
        (CatchupArmed, Snoop)
            | (CatchupArmed, Catchup)
            | (Snoop, Catchup)
            | (Catchup, TxArmed)
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
    use super::{
        ChainPredict, FastChainPhase, decide_rxne_tail_pure, is_legal_transition,
        predict_chain_pure, ring_crc,
    };
    use dxl_protocol::crc16;

    const RING: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

    // 3 Mbaud reference values used across the rxne-tail tests:
    //   brr = 48 MHz / 3 MHz = 16; byte_time_ticks = 10 × brr = 160.
    //   bytes_per_us_q16 = round((48 << 16) / 160) = 19661.
    const BYTE_TIME_TICKS_3M: u32 = 160;
    const BYTES_PER_US_Q16_3M: u32 = 19661;
    // Default RDT used on rev_b boot (control table comms.return_delay_2us=125).
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
        assert!(is_legal_transition(CatchupArmed, Snoop));
        assert!(is_legal_transition(CatchupArmed, Catchup));
        assert!(is_legal_transition(Snoop, Catchup));
        assert!(is_legal_transition(Catchup, TxArmed));
        assert!(is_legal_transition(TxArmed, TxStreaming));
        assert!(is_legal_transition(TxStreaming, CrcPatched));
        assert!(is_legal_transition(CrcPatched, Done));
    }

    #[test]
    fn illegal_transitions_rejected() {
        use FastChainPhase::*;
        assert!(!is_legal_transition(CatchupArmed, TxArmed));
        assert!(!is_legal_transition(Snoop, TxStreaming));
        assert!(!is_legal_transition(TxArmed, Catchup));
        assert!(!is_legal_transition(CrcPatched, TxArmed));
    }

    // -- predict_chain ----------------------------------------------------

    #[test]
    fn predict_chain_3m_last_1b_floors_n_pred_by_one() {
        // 3M, 2 slaves, Last with 1B own payload, 1B predecessor payload.
        // Dispatcher: bytes_before(1) = FAST_SLOT0_PREFIX(10) + 1 = 11; then
        // bytes_to_us(11, 3M) = floor(11 × 218453 / 64K) = 36; fire_us = 250 + 36.
        // True n_pred = 11 but predict_chain floors back to 10 — the bug
        // the +1 bias in decide_rxne_tail_pure absorbs.
        let p = predict_chain_pure(286, RDT_US_DEFAULT, BYTES_PER_US_Q16_3M);
        assert_eq!(p.wire_window_us, 36);
        assert_eq!(p.n_pred, 10);
    }

    #[test]
    fn predict_chain_zero_window_when_fire_before_rdt() {
        // Fire scheduled inside RDT (degenerate): saturate wire_window to 0
        // so decide_rxne_tail short-circuits to disabled.
        let p = predict_chain_pure(100, RDT_US_DEFAULT, BYTES_PER_US_Q16_3M);
        assert_eq!(p.wire_window_us, 0);
        assert_eq!(p.n_pred, 0);
    }

    #[test]
    fn predict_chain_zero_bytes_per_us_returns_zero_n_pred() {
        // Boot state before store_baud_derived runs.
        let p = predict_chain_pure(286, RDT_US_DEFAULT, 0);
        assert_eq!(p.wire_window_us, 36);
        assert_eq!(p.n_pred, 0);
    }

    // -- decide_rxne_tail -------------------------------------------------

    #[test]
    fn rxne_tail_arms_for_3m_last_1b_with_predicted_n_pred_10() {
        // Regression: the floored n_pred = 10 must still arm RXNE-tail
        // because the +1 bias on the walk estimate trips the slack gate.
        // Walk = (10+1)×24 + 200 + 24 = 488 ticks; slack = (5-2)×160 = 480.
        // rxne_tail_at = (10+1) - GUARD(2) = 9.
        let predict = ChainPredict { n_pred: 10, wire_window_us: 36 };
        assert_eq!(
            decide_rxne_tail_pure(&predict, 5, BYTE_TIME_TICKS_3M),
            9
        );
    }

    #[test]
    fn rxne_tail_skips_when_switch_can_cover_window() {
        // Wire window ≥ SWITCH_MARGIN_US — switch CMP lands inside the
        // predecessor's transmission and walks bytes pre-fire, so the
        // RXNE-tail tier shouldn't double up.
        let predict = ChainPredict { n_pred: 22, wire_window_us: 73 };
        assert_eq!(
            decide_rxne_tail_pure(&predict, 5, BYTE_TIME_TICKS_3M),
            u16::MAX
        );
    }

    #[test]
    fn rxne_tail_skips_when_slack_covers_walk() {
        // 2 sl × L=4 at 3M: tx_len=8, slack = 6×160 = 960 ticks; walk =
        // (14+1)×24 + 224 = 584 < 960. Post-fire walk fits without the
        // per-byte ISR cost.
        let predict = ChainPredict { n_pred: 14, wire_window_us: 47 };
        assert_eq!(
            decide_rxne_tail_pure(&predict, 8, BYTE_TIME_TICKS_3M),
            u16::MAX
        );
    }

    #[test]
    fn rxne_tail_skips_when_no_snoop_predicted() {
        // No predecessors (Only-slot or boot-time call) → RXNE-tail off.
        let predict = ChainPredict { n_pred: 0, wire_window_us: 0 };
        assert_eq!(
            decide_rxne_tail_pure(&predict, 5, BYTE_TIME_TICKS_3M),
            u16::MAX
        );
    }

    #[test]
    fn rxne_tail_skips_when_tx_too_small_for_slack() {
        // tx_len < 4 → slack would underflow. Skip.
        let predict = ChainPredict { n_pred: 10, wire_window_us: 36 };
        assert_eq!(
            decide_rxne_tail_pure(&predict, 3, BYTE_TIME_TICKS_3M),
            u16::MAX
        );
    }

    #[test]
    fn rxne_tail_skips_when_byte_time_uninitialized() {
        // store_baud_derived hasn't run; slack would be 0. Skip safely.
        let predict = ChainPredict { n_pred: 10, wire_window_us: 36 };
        assert_eq!(decide_rxne_tail_pure(&predict, 5, 0), u16::MAX);
    }

    #[test]
    fn rxne_tail_at_clamps_to_one_for_tiny_n_pred() {
        // n_pred = 1, +1 bias = 2, − GUARD(2) = 0 → clamped to 1 so the
        // mask check `bytes_walked >= rxne_tail_at` is always meaningful.
        // Use tiny byte_time_ticks to force the walk-vs-slack gate to fire.
        let predict = ChainPredict { n_pred: 1, wire_window_us: 10 };
        assert_eq!(decide_rxne_tail_pure(&predict, 4, 100), 1);
    }
}
