//! TIM4 OPM scheduler + DMA1_CH2/CH4 chain. TIM4 OPM CC2 match fires
//! DMA1_CH4 (a single-word circular stamper), which writes a precomputed
//! EN=1 CFGR word into DMA1_CH2.CR. DMA1_CH2 then drains the prepared
//! payload into USART3.DATAR. No IRQ in the path: kickoff arrives via
//! pure DMA chaining.
//!
//! TIM4 PSC=0, ARR=delta in tick32, CCR2=ARR. CCR2=ARR puts the CC2
//! event on the same edge as the OPM overflow.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_metapac::dma::vals::{Dir, Pl, Size};
use ch32_metapac::timer::vals::Urs;
use ch32_metapac::{DMA1, TIM4, USART3};

use crate::tick::read_tick32;
use crate::tx::comp;

/// (send_at - now) below this many `tick32` ticks bypasses TIM4 and writes
/// the run-CFGR word directly into DMA1_CH2. 256 ticks ≈ 1.8 µs at
/// 144 MHz — safely above the register-write latency of the immediate
/// path. A too-close deadline still kicks (wire-edge lands ≈ "now")
/// instead of silently missing on a wrap.
const IMMEDIATE_SEND_THRESHOLD_TICKS: u32 = 256;

/// Headroom added to `read_tick32()` when `send_now` schedules its
/// near-future send through `schedule_or_send_now`. Sized so the
/// resulting `delta` lands comfortably above `IMMEDIATE_SEND_THRESHOLD_TICKS`
/// — keeping the scheduled (TIM4 OPM) path in play rather than falling
/// back to the immediate DMA-direct branch, so `LAST_SEND_TICK` matches
/// the commanded wire-start tick. 512 ticks ≈ 3.5 µs at 144 MHz —
/// negligible added latency, well above the 256-tick scheduler threshold.
pub(super) const IMMEDIATE_SEND_MARGIN_TICKS: u32 = 512;

/// TIM4 ARR (u16) max in tick32 units. PSC=0 → 144 MHz tick = 1:1 with
/// tick32, so ARR = delta directly and max schedule is 65 535 ticks ≈
/// 455 µs. Deadlines beyond this fall back to the immediate path (host
/// should never request them — typical wire-side slot timing is sub-ms).
const TIM4_MAX_DELTA_TICKS: u32 = u16::MAX as u32;

const TIM4_PSC: u16 = 0;

/// Precomputed DMA1_CH2 CFGR word with EN=1. DMA1_CH4 copies this 32-bit
/// word over DMA1_CH2.CR on the TIM4 CC2 kickoff. Written once during
/// init, read-only thereafter so the DMA can treat it as constant memory.
static CH2_CFGR_RUN_WORD: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// `tick32` at the last send kickoff (commanded value for scheduled
/// sends, "now" for immediate sends). Host correlates against per-byte
/// stamps.
static LAST_SEND_TICK: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

#[inline]
fn store_last_send_tick(t: u32) {
    unsafe { ptr::write_volatile(LAST_SEND_TICK.get(), t) }
}

pub fn last_send_tick() -> u32 {
    unsafe { ptr::read_volatile(LAST_SEND_TICK.get()) }
}

pub(super) fn init() {
    init_tim4();
    init_dma();
}

/// TIM4 OPM. PSC=0 (1:1 with tick32). CR1: OPM=1 (auto-clear CEN after
/// first UEV), URS=1 (UG software event resets CNT without generating a
/// UEV that would spuriously kick CC2). DIER CC2DE=1 routes CC2 compare
/// match to DMA1_CH4.
fn init_tim4() {
    TIM4.psc().write_value(TIM4_PSC);
    TIM4.atrlr().write_value(0xFFFF);
    TIM4.ctlr1().write(|w| {
        w.set_cen(false);
        w.set_opm(true);
        w.set_urs(Urs::COUNTERONLY);
        w.set_arpe(false);
    });
    TIM4.dmaintenr().write(|w| {
        w.set_ccde(1, true); // CC2DE (channels 0-indexed)
    });
}

/// DMA1_CH2 = USART3_TX + DMA1_CH4 = TIM4_CC2 stamper that flips CH2.EN
/// on send. CH2.EN stays clear at init; `load_payload` reloads MAR/NDTR
/// with EN=0 between sends. CH4 is circular single-word: copy
/// CH2_CFGR_RUN_WORD → DMA1_CH2.CR. NDTR=1 auto-reloads each cycle
/// (CIRC=1), so we never re-program CH4 from software. CH4.EN=1
/// permanently — the CC2DE gate kicks it, not EN.
fn init_dma() {
    let ch2 = DMA1.ch(1);
    ch2.par().write_value(USART3.datar().as_ptr() as u32);
    ch2.cr().write(|w| {
        w.set_dir(Dir::FROMMEMORY);
        w.set_minc(true);
        w.set_pinc(false);
        w.set_circ(false);
        w.set_msize(Size::BITS8);
        w.set_psize(Size::BITS8);
        // VERYHIGH is defensive: today only CH2 + CH4 are active so
        // inter-channel arbitration doesn't kick in, but pins the send
        // path at top priority against any future DMA channel we add.
        // (Empirically does NOT tighten TX wire-edge jitter — the
        // observed loopback variance is CPU-vs-DMA AHB arbitration,
        // which channel-priority bits don't control.)
        w.set_pl(Pl::VERYHIGH);
        w.set_tcie(false);
    });

    // Precompute the EN=1 CFGR word DMA1_CH4 stamps on send. Same bits
    // as the stopped config above, plus EN=1.
    let mut run = ch32_metapac::dma::regs::Cr(0);
    run.set_dir(Dir::FROMMEMORY);
    run.set_minc(true);
    run.set_pinc(false);
    run.set_circ(false);
    run.set_msize(Size::BITS8);
    run.set_psize(Size::BITS8);
    run.set_pl(Pl::VERYHIGH); // mirror CH2 PL above so the stamp doesn't drop priority on kickoff
    run.set_tcie(false);
    run.set_en(true);
    // SAFETY: CH2_CFGR_RUN_WORD is only read by DMA1_CH4 (set up
    // below). CH4 isn't enabled yet, so no concurrent reader.
    unsafe { ptr::write_volatile(CH2_CFGR_RUN_WORD.get(), run.0) };

    let ch4 = DMA1.ch(3);
    ch4.par().write_value(ch2.cr().as_ptr() as u32);
    ch4.mar().write_value(CH2_CFGR_RUN_WORD.get() as u32);
    ch4.ndtr().write(|w| w.set_ndt(1));
    ch4.cr().write(|w| {
        w.set_dir(Dir::FROMMEMORY);
        w.set_minc(false);
        w.set_pinc(false);
        w.set_circ(true);
        w.set_msize(Size::BITS32);
        w.set_psize(Size::BITS32);
        w.set_pl(Pl::VERYHIGH); // mirror CH2 PL above; defensive against future inter-channel contention
        w.set_tcie(false);
        w.set_en(true);
    });
}

pub(super) fn cancel() {
    TIM4.ctlr1().modify(|w| w.set_cen(false));
    // URS=1 blocks UEV from this UG, so DMA1_CH4 isn't kicked.
    TIM4.swevgr().write(|w| w.set_ug(true));
    TIM4.intfr().write(|w| {
        w.set_uif(false);
        w.set_ccif(1, false);
    });
}

/// Same effect as TIM4 CC2 → DMA1_CH4 kicking CH2, just bypassing the
/// timer.
#[inline]
fn start_dma_send() {
    let run = unsafe { ptr::read_volatile(CH2_CFGR_RUN_WORD.get()) };
    DMA1.ch(1)
        .cr()
        .write_value(ch32_metapac::dma::regs::Cr(run));
}

/// Program TIM4 OPM to kick DMA1_CH2 when `tick32` reaches `at`, or send
/// immediately if the deadline is past / within scheduling overhead /
/// beyond TIM4's 16-bit ARR range.
pub(super) fn schedule_or_send_now(at: u32) {
    let comp = comp::load();
    let scheduled_at = at.wrapping_sub(comp);
    let now = read_tick32();
    let delta = scheduled_at.wrapping_sub(now);
    // `delta` is u32 modular; treat very large values (= past deadline)
    // as "send now". With wrapping subtraction, "past" maps to large
    // positive deltas, so a single threshold covers both close-and-past.
    if !(IMMEDIATE_SEND_THRESHOLD_TICKS..=TIM4_MAX_DELTA_TICKS).contains(&delta) {
        store_last_send_tick(now);
        start_dma_send();
        return;
    }
    // ARR latches the next time CNT==ARR. CCR2=ARR puts CC2 on the same
    // edge as the OPM overflow.
    let arr = delta as u16;
    TIM4.atrlr().write_value(arr);
    TIM4.chcvr(1).write_value(arr); // CCR2 (0-indexed)
    store_last_send_tick(at);
    TIM4.ctlr1().modify(|w| w.set_cen(true));
}
