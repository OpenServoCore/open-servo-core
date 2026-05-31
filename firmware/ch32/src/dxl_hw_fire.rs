//! TIM2 OPM hardware fire path for DXL replies (see
//! `docs/dxl-fast-chain-crc.md` §15).
//!
//! Production wire-fire path. Mirrors `dxl-pirate`'s TIM4-OPM design
//! (`tools/dxl-pirate/src/inject.rs`) but with two DMA events instead of
//! one — V006 carries an external TX_EN buffer, so we assert TX_EN HIGH
//! before kicking the USART TX DMA.
//!
//! Sequence, after `arm_at` writes TIM2 ARR/CCR + CEN=1:
//!
//! ```text
//!     TIM2 CNT ──> counts toward ARR in 48 MHz silicon
//!         CNT==CCR4 -> CC4 event -> CCDE3 routes to DMA1_CH7
//!             CH7 writes TX_EN_ASSERT_PATTERN -> GPIO BSHR -> TX_EN HIGH
//!         CNT==ARR  -> UEV       -> UDE routes to DMA1_CH2
//!             CH2 writes ARMED_CH4_CFGR_WORD -> DMA1_CH4.CFGR.EN=1
//!             USART1 (DMAT=1 permanent) pulls bytes from CH4 -> wire
//!         OPM auto-clears CEN
//! ```
//!
//! No software in the wire-fire path. SW's sole job is loading the TIM2
//! deadline ahead of time. CH7+CH2 are CIRC=1 EN=1 forever — NDTR
//! auto-reloads after each fire (mirrors pirate's CH7).
use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;

use ch32_metapac::{DMA1, TIM2, USART1, dma::vals::Dir, timer::regs::Ctlr1, timer::vals::Urs};
use portable_atomic::AtomicBool;

use crate::board::TxEn;
use crate::hal::{
    dma,
    gpio::{self, Level},
    rcc, systick,
};
use crate::statics::DXL_TX_EN;

/// Plain fire-path pipeline base, in HCLK ticks. Empirically fit from the
/// bench (V006 rev_b, tune_hw): Plain Only/First median offset across
/// 1M/2M/3M sits at +3.72/+2.50/+2.17 µs late with comp=16. Backing out
/// `clock_fine_trim_us` gives a real chip+USART+pirate-RX pipeline of
/// 202/143/127 ticks per baud. Linear fit:
///
/// ```text
/// pipeline = HW_FIRE_PLAIN_PIPELINE_BASE_TICKS + byte_time_ticks / 4
/// ```
///
/// where `byte_time_ticks / 4 = 2.5 bit-times` captures the USART
/// internal start-bit pipeline + pirate RX-side latency. The base
/// constant captures SW exec in arm_at + DMA chain + miscellaneous.
pub const HW_FIRE_PLAIN_PIPELINE_BASE_TICKS: u32 = 83;

/// Fast fire-path pipeline ticks. Smaller than Plain because the bench
/// measures Fast Last as `(chip_first - INJ_first) - expected_span`,
/// which cancels the pirate-side RX latency that Plain's
/// `T_first - T_req` measurement includes. Empirically the Fast path
/// chip-side pipeline already lands gap ≈ +0.7 µs (safely AFTER
/// predecessor's stop bit but inside the coalesce window) at current
/// comp = 16 — bumping it further would push fire INTO predecessor's
/// last byte = bus contention. Kept small and constant; not
/// baud-tunable here (the silicon component is baud-dep but the
/// remaining ~30 ticks of "margin against collision" is the meaningful
/// budget).
pub const HW_FIRE_FAST_PIPELINE_TICKS: u32 = 16;

/// Tick gap from CC4 (TX_EN HIGH via Ch7) to UP (CH4.EN=1 via Ch2). Lands
/// TX_EN inside the predecessor's stop-bit window per chain-CRC §15.2 —
/// too short risks racing the predecessor's last bit, too long pushes UP
/// past the start-bit edge. 24 ticks = 0.5 µs ≈ 1.5 bit-times @ 3 M.
const CC4_TO_UP_TICKS: u16 = 24;

/// DMA Ch7 source: BSHR mask that drives TX_EN to its TX level on
/// whichever port the board wired it to. Seeded by `init`, read by
/// hardware only thereafter.
static TX_EN_ASSERT_PATTERN: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// DMA Ch2 source: pre-baked CH4 CFGR word with EN=1; CH2 stamps this
/// over DMA1_CH4.CFGR on the TIM2 UEV, gating the USART TX. Read by
/// hardware only after init.
static ARMED_CH4_CFGR_WORD: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Pre-baked TIM2.CTLR1 word with CEN=1 and our static config (OPM=1,
/// URS=COUNTERONLY, ARPE=0). Lets `arm_at`'s final write be a single
/// `write_value` instead of a 3-cycle RMW, shaving ~2 cycles off the
/// critical path between `systick::ticks()` and TIM2 starting to count.
static CTLR1_RUNNING_WORD: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Guards against double-init.
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Sole writer to the static patterns + TIM2/DMA Ch7/Ch2 config. Call
/// once during bring-up AFTER `bring_up_dxl` has configured USART1
/// HDSEL/DMAR and CH4's MAR/DIR/etc — the armed CFGR word captures CH4's
/// settled config plus EN=1, and DMAT is set =1 permanently here.
pub fn init(tx_en: TxEn) {
    if INITIALIZED.swap(true, Ordering::AcqRel) {
        return;
    }

    let pin = tx_en.pin;
    let bshr_addr = pin.gpio_regs().bshr().as_ptr() as u32;
    let bit = pin.pin_number() as u32;
    // BSHR low 16 = set, high 16 = reset. Drive TX_EN to its TX level.
    let assert_pattern = match tx_en.tx_level {
        Level::High => 1u32 << bit,
        Level::Low => 1u32 << (bit + 16),
    };

    // Snapshot CH4's current CFGR (set by bring_up_dxl) and OR in EN=1.
    // CH2 stamps this whole word — CFGR is rewritten on every fire, so
    // any CH4 config bits we want to keep MUST live in this snapshot.
    let armed_ch4_cfgr = {
        let mut v = DMA1.ch(3).cr().read();
        v.set_en(true);
        v.0
    };

    // SAFETY: pre-IRQ, single-threaded bring-up.
    unsafe {
        *TX_EN_ASSERT_PATTERN.get() = assert_pattern;
        *ARMED_CH4_CFGR_WORD.get() = armed_ch4_cfgr;
    }

    rcc::enable_tim2();
    rcc::enable_dma1();

    // DMAT permanent =1 under hw-fire — CH4.EN is the gate now, flipped
    // by CH2's HW stamp. TC handler must NOT clear DMAT (cfg-gated).
    USART1.ctlr3().modify(|w| w.set_dmat(true));

    let t = TIM2;
    t.ctlr1().write(|w| {
        w.set_opm(true);
        // URS=COUNTERONLY: UG (used by `cancel` to reset CNT) won't
        // generate a UEV — only CNT==ARR overflow does. Without this,
        // cancel would spuriously trigger CH2 and start an unwanted TX.
        w.set_urs(Urs::COUNTERONLY);
        w.set_arpe(false);
        w.set_cen(false);
    });
    // Stash a pre-baked CTLR1 word with CEN=1 so arm_at's final write
    // (the one that actually launches TIM2) is a single 16-bit store
    // instead of a read-modify-write.
    {
        let mut running = Ctlr1(0);
        running.set_opm(true);
        running.set_urs(Urs::COUNTERONLY);
        running.set_arpe(false);
        running.set_cen(true);
        // SAFETY: pre-IRQ.
        unsafe { *CTLR1_RUNNING_WORD.get() = running.0 };
    }
    t.psc().write_value(0);
    // OC4M=frozen, CC4E=off — CCxIF latches internally for DMA, but no
    // pin output (TIM2_CH4 has no AF route to TX_EN on V006 anyway).
    t.chctlr_output(1).write_value(Default::default());
    t.ccer().write_value(Default::default());

    t.dmaintenr().write(|w| {
        w.set_ude(true);
        w.set_ccde(3, true);
    });

    // CH7: TIM2_CH4 event → write TX_EN_ASSERT_PATTERN into GPIO BSHR.
    // CIRC=1 + NDTR=1 → auto-reloads NDTR every fire, no software re-arm.
    // EN=1 forever — the trigger is TIM2 CCDE3, not the EN bit.
    let cfg32_circ = dma::Config {
        dir: Dir::FROMMEMORY,
        circ: true,
        pinc: false,
        minc: false,
        size: dma::Size::BITS32,
        tcie: false,
        pl: dma::Pl::HIGH,
    };
    dma::configure(
        dma::Channel::CH7,
        &cfg32_circ,
        bshr_addr,
        TX_EN_ASSERT_PATTERN.get() as u32,
        1,
    );
    dma::enable(dma::Channel::CH7);

    // CH2: TIM2_UP event → write ARMED_CH4_CFGR_WORD into DMA1_CH4.CFGR.
    // Same CIRC=1 + EN=1 forever pattern. Mirrors pirate's CH7
    // (`tools/dxl-pirate/src/inject.rs`).
    let ch4_cfgr_addr = DMA1.ch(3).cr().as_ptr() as u32;
    dma::configure(
        dma::Channel::CH2,
        &cfg32_circ,
        ch4_cfgr_addr,
        ARMED_CH4_CFGR_WORD.get() as u32,
        1,
    );
    dma::enable(dma::Channel::CH2);
}

/// Schedule the wire fire at `target_tick` (absolute SysTick value). Sole
/// fire-path entry. Writes TIM2 ARR + CCR4 + CEN=1 — everything after is
/// silicon. No SW fallback path: a deadline already past degrades to the
/// minimum-viable TIM2 countdown so the fire still happens via TIM2, not
/// via a direct SW MMIO stamp.
///
/// **Critical-path ordering matters.** The cycles between
/// `systick::ticks()` and the final CEN=1 write directly add to the
/// wire-fire lateness (TIM2 counts `arr` ticks from CEN=1; we computed
/// `arr` against `now` which is older by that gap). Anything that can
/// run BEFORE `now` is read goes in the cleanup block at the top.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn arm_at(target_tick: u32) {
    let t = TIM2;

    // ── Off-path: cleanup of prior fire state. Independent of `now`, so
    // moving these out of the critical path shaves their cycles off the
    // wire-fire lateness.
    t.ctlr1().modify(|w| w.set_cen(false));
    t.cnt().write_value(0);
    // INTFR is RC_W0: writing 0 clears UIF/CC4IF residue from prior fire.
    // Belt-and-suspenders — UEV/CC4 DMA requests are pulse-triggered on
    // the EVENT not on the FLAG, but clearing keeps state observable.
    t.intfr().write(|_| {});
    // Pre-load the CEN=1 word so the final write below is a single store.
    // SAFETY: written once at `init`, ISR-only reader after.
    let cen1 = Ctlr1(unsafe { *CTLR1_RUNNING_WORD.get() });

    // ── Critical path: minimize cycles from here to CEN=1.
    let now = systick::ticks();
    let delta = target_tick.wrapping_sub(now);
    // ARR clamp: past-deadline (signed-negative delta) and oversize-delta
    // (> u16::MAX) both fold to the minimum-viable countdown
    // (CC4_TO_UP_TICKS = "fire ASAP via TIM2"). Branch is single-compare
    // because we mask via wrapping arith on a sentinel range.
    let arr = if (delta as i32) <= CC4_TO_UP_TICKS as i32 || delta > u16::MAX as u32 {
        CC4_TO_UP_TICKS
    } else {
        delta as u16
    };
    t.chcvr(3).write_value(arr - CC4_TO_UP_TICKS);
    t.atrlr().write_value(arr);
    t.ctlr1().write_value(cen1);
}

/// Halt any pending fire, disable CH4 (in case TIM2 OPM already fired),
/// and drop TX_EN. Idempotent. URS=COUNTERONLY blocks the UG-from-CEN-0
/// path from generating a UEV.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn cancel() {
    let t = TIM2;
    t.ctlr1().modify(|w| w.set_cen(false));
    t.cnt().write_value(0);
    t.intfr().write(|_| {});
    dma::disable(dma::Channel::CH4);
    // SAFETY: written once at bring-up before ISRs unmask; ISR-only reads.
    if let Some(tx_en) = unsafe { *DXL_TX_EN.get() } {
        gpio::set_level(tx_en.pin, tx_en.idle_level());
    }
}
