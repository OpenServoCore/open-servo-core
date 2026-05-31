//! TIM2 OPM hardware fire path for DXL chain-CRC (see
//! `docs/dxl-fast-chain-crc.md` §15).
//!
//! Replaces the SysTick CMP IRQ → `fire_now` software fire with a
//! hardware-clocked sequence: TIM2 in OPM counts to ARR, CC4 match
//! triggers DMA Ch7 to write GPIO BSHR (TX_EN HIGH), then UP triggers
//! DMA Ch2 to write USART1.CTLR3 (DMAT=1) which kicks the already-armed
//! Ch4 TX DMA. PFIC trap entry leaves the fire path; only the post-fire
//! CRC patch runs in software (still scheduled via SysTick CMP, but the
//! wire-side timing no longer depends on PFIC entry latency).
//!
use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;

use ch32_metapac::{TIM2, USART1, dma::vals::Dir, timer::vals::Urs};
use portable_atomic::AtomicBool;

use crate::board::TxEn;
use crate::hal::{dma, gpio::Level, rcc};

/// HCLK ticks from `arm()` to first start-bit on the wire. Pipeline:
/// CEN→CNT-increment + CC4-match→Ch7-write + UP→Ch2-write + USART
/// DMAT-poll + start-bit-sync (≈½ bit-time, 8 ticks @ 3 M). Conservative
/// placeholder until #64 pins it on the bench; a per-chip override could
/// later promote this to an atomic without changing the CT schema (see
/// chain-CRC §15.4).
pub const HW_FIRE_PIPELINE_TICKS: u32 = 16;

/// Tick gap from CC4 match (TX_EN HIGH via Ch7) to UP (DMAT=1 via Ch2).
/// Sized to land TX_EN inside the predecessor's stop-bit window per
/// chain-CRC §15.2 — too short risks racing the predecessor's last bit,
/// too long pushes UP past the start-bit edge. 24 ticks = 0.5 µs ≈ 1.5
/// bit-times @ 3 M. Not on the jitter-critical path.
const CC4_TO_UP_TICKS: u16 = 24;

/// DMA Ch7 source: BSHR mask that drives TX_EN to its TX level on
/// whichever port the board wired it to. Seeded by `init`, read by
/// hardware only thereafter.
static TX_EN_ASSERT_PATTERN: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// DMA Ch2 source: full USART1.CTLR3 word with DMAT=1 ORed against the
/// steady-state HDSEL/DMAR/etc bits sampled at init. Read by hardware
/// only — anything that mutates CTLR3 after `init` (e.g. a future
/// duplex flip) needs to re-sample via `refresh_ctlr3_source`.
static USART1_CTLR3_WITH_DMAT: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Guards against double-init and (debug) against arm-before-init.
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Sole writer to the static patterns + TIM2/DMA Ch7/Ch2 config. Call
/// once during bring-up after `bring_up_dxl` has set HDSEL/DMAR on
/// USART1 — the staged CTLR3 word inherits those bits.
pub fn init(tx_en: TxEn) {
    let pin = tx_en.pin;
    let bshr_addr = pin.gpio_regs().bshr().as_ptr() as u32;
    let bit = pin.pin_number() as u32;
    // BSHR low 16 = set, high 16 = reset. Drive TX_EN to its TX level.
    let assert_pattern = match tx_en.tx_level {
        Level::High => 1u32 << bit,
        Level::Low => 1u32 << (bit + 16),
    };
    let ctlr3_with_dmat = USART1.ctlr3().read().0 | (1u32 << 7);

    // SAFETY: pre-IRQ, single-threaded bring-up.
    unsafe {
        *TX_EN_ASSERT_PATTERN.get() = assert_pattern;
        *USART1_CTLR3_WITH_DMAT.get() = ctlr3_with_dmat;
    }

    rcc::enable_tim2();
    rcc::enable_dma1();

    let t = TIM2;
    t.ctlr1().write(|w| {
        w.set_opm(true);
        w.set_urs(Urs::COUNTERONLY);
        w.set_arpe(false);
        w.set_cen(false);
    });
    t.psc().write_value(0);
    // OC4M = frozen (000) — CCxIF latches on compare regardless of OC4
    // pin output, which we don't use (no AF route to TX_EN on V006). CC4E
    // stays off; we only want the internal compare event.
    t.chctlr_output(1).write_value(Default::default());
    t.ccer().write_value(Default::default());

    t.dmaintenr().write(|w| {
        w.set_ude(true);
        w.set_ccde(3, true);
    });

    let cfg32 = dma::Config {
        dir: Dir::FROMMEMORY,
        circ: false,
        pinc: false,
        minc: false,
        size: dma::Size::BITS32,
        tcie: false,
        pl: dma::Pl::HIGH,
    };
    dma::configure(
        dma::Channel::CH7,
        &cfg32,
        bshr_addr,
        TX_EN_ASSERT_PATTERN.get() as u32,
        1,
    );
    dma::configure(
        dma::Channel::CH2,
        &cfg32,
        USART1.ctlr3().as_ptr() as u32,
        USART1_CTLR3_WITH_DMAT.get() as u32,
        1,
    );

    INITIALIZED.store(true, Ordering::Release);
}

/// Re-prime DMA Ch7 + Ch2 NDTR=1 and EN=1. Each fire consumes one
/// transfer per channel, so the next fire needs the count restored.
/// Called from `dxl_fast`'s arm path before `arm`.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn prepare_fire() {
    dma::disable(dma::Channel::CH7);
    dma::disable(dma::Channel::CH2);
    dma::set_count(dma::Channel::CH7, 1);
    dma::set_count(dma::Channel::CH2, 1);
    dma::enable(dma::Channel::CH7);
    dma::enable(dma::Channel::CH2);
}

/// Start TIM2 OPM counting toward `ticks_from_now`. Caller has already
/// subtracted `HW_FIRE_PIPELINE_TICKS` from the absolute fire deadline.
/// CC4 fires at `ticks_from_now` (→ TX_EN HIGH); UP fires
/// `CC4_TO_UP_TICKS` later (→ DMAT=1).
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn arm(ticks_from_now: u16) {
    let t = TIM2;
    t.ctlr1().modify(|w| w.set_cen(false));
    t.cnt().write_value(0);
    t.chcvr(3).write_value(ticks_from_now);
    t.atrlr()
        .write_value(ticks_from_now.saturating_add(CC4_TO_UP_TICKS));
    t.ctlr1().modify(|w| w.set_cen(true));
}

/// Halt any pending fire and disarm Ch7/Ch2. Idempotent.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn cancel() {
    let t = TIM2;
    t.ctlr1().modify(|w| w.set_cen(false));
    t.cnt().write_value(0);
    dma::disable(dma::Channel::CH7);
    dma::disable(dma::Channel::CH2);
}
