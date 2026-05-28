//! Passive bus listener: USART3 RX (PB11) + DMA1_CH3 circular + IDLE IRQ.
//! Each gap on the wire (≥ 1 byte time idle) latches a `(tick, head)` stamp
//! into a small SPSC ring the host can drain to verify inter-slot timing.
//!
//! Mirrors the V006 dxl RX pattern in `firmware/ch32` but stripped to just the
//! timestamping bits — no parser, no TX state machine.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_hal::pac::Interrupt;
use ch32_hal::pac::dma::vals::{Dir, Size};
use ch32_hal::pac::{DMA1, GPIOB, RCC, SYSTICK, USART3};
use dxl_bus_injector::parse::brr_for;
use portable_atomic::{AtomicU32, Ordering};
use qingke_rt::interrupt;

use crate::inject::{APB1_HZ, BaudError, DEFAULT_BAUD};

const RX_BUF_LEN: usize = 256;
const _: () = assert!(
    RX_BUF_LEN.is_power_of_two(),
    "RX_BUF_LEN must be pow2 for ndtr mask"
);
const RX_BUF_MASK: u32 = (RX_BUF_LEN - 1) as u32;

static RX_BUF: SyncUnsafeCell<[u8; RX_BUF_LEN]> = SyncUnsafeCell::new([0; RX_BUF_LEN]);

const STAMP_RING_LEN: usize = 32;
const STAMP_MASK: usize = STAMP_RING_LEN - 1;
const _: () = assert!(
    STAMP_RING_LEN.is_power_of_two(),
    "STAMP_RING_LEN must be pow2"
);

/// SysTick ticks for one USART3 char-time; ISR backdates by this so `tick`
/// lands at wire-end (IDLE asserts 9 bit-times later).
static CHAR_TIME_SYSTICKS: AtomicU32 = AtomicU32::new(0);

const fn char_time_systicks(brr: u32) -> u32 {
    // USART bit-time = `brr` HCLK ticks; SysTick = HCLK/8 → 9 bit-times = 9 * brr / 8.
    9 * brr / 8
}

#[derive(Copy, Clone, Default)]
pub struct IdleStamp {
    /// SysTick CNT low half at the moment IDLE fired (HCLK/8 ticks).
    pub tick: u32,
    /// DMA write head (bytes received since boot, low 16 bits).
    pub head: u16,
}

static STAMPS: SyncUnsafeCell<[IdleStamp; STAMP_RING_LEN]> =
    SyncUnsafeCell::new([IdleStamp { tick: 0, head: 0 }; STAMP_RING_LEN]);
static STAMP_HEAD: AtomicU32 = AtomicU32::new(0);
static STAMP_TAIL: AtomicU32 = AtomicU32::new(0);

// Single-writer (USART3 ISR), main-task reader. Single-word, tear-free on
// V203's aligned u32 stores → volatile cell instead of an atomic. Wraps; the
// host compares relative deltas.
static BYTES_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
// ISR-only; never touched from anywhere else. Plain cell.
static LAST_NDTR: SyncUnsafeCell<u16> = SyncUnsafeCell::new(RX_BUF_LEN as u16);

pub fn init() {
    unsafe {
        RCC.apb2pcenr().modify(|w| {
            w.set_iopben(true);
            w.set_afioen(true);
        });
        RCC.apb1pcenr().modify(|w| w.set_usart3en(true));

        // PB11 = USART3_RX input floating. CFGHR controls PB8..PB15;
        // PB11 sits in bits [15:12]. Mode=00 (input), CNF=01 (floating).
        let cnf_mode_pb11 = 0b0100u32;
        GPIOB.cfghr().modify(|w| {
            let mut v = w.0;
            v &= !(0xF << 12);
            v |= cnf_mode_pb11 << 12;
            w.0 = v;
        });

        // USART3 on APB1. Default 1 Mbaud to match `inject` + V006 wire rate;
        // host can retune via `set_baud` before bringing the bus up.
        USART3.brr().write(|w| w.0 = APB1_HZ / DEFAULT_BAUD);
        CHAR_TIME_SYSTICKS.store(
            char_time_systicks(APB1_HZ / DEFAULT_BAUD),
            Ordering::Relaxed,
        );
        USART3.ctlr2().modify(|w| w.set_stop(0b00));
        USART3.ctlr3().modify(|w| w.set_dmar(true));
        USART3.ctlr1().modify(|w| {
            w.set_m(false);
            w.set_pce(false);
            w.set_re(true);
            w.set_te(false);
            w.set_idleie(true);
            w.set_ue(true);
        });

        // DMA1 CH3 = USART3_RX, circular, byte-wide.
        let ch = DMA1.ch(2);
        ch.par().write_value(USART3.datar().as_ptr() as u32);
        ch.mar().write_value((*RX_BUF.get()).as_ptr() as u32);
        ch.ndtr().write(|w| w.set_ndt(RX_BUF_LEN as u16));
        ch.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS8);
            w.set_psize(Size::BITS8);
            w.set_tcie(false);
            w.set_en(true);
        });

        qingke::pfic::enable_interrupt(Interrupt::USART3 as u8);
    }
}

pub fn drain_stamp() -> Option<IdleStamp> {
    let tail = STAMP_TAIL.load(Ordering::Relaxed);
    if tail == STAMP_HEAD.load(Ordering::Acquire) {
        return None;
    }
    let stamp = unsafe { (*STAMPS.get())[(tail as usize) & STAMP_MASK] };
    STAMP_TAIL.store(tail.wrapping_add(1), Ordering::Release);
    Some(stamp)
}

pub fn byte_count() -> u32 {
    unsafe { ptr::read_volatile(BYTES_TOTAL.get()) }
}

/// Reconfigure USART3's bit rate. Bounces UE around the BRR write. Caller
/// must quiesce the bus first.
pub fn set_baud(bps: u32) -> Result<(), BaudError> {
    let brr = brr_for(APB1_HZ, bps).ok_or(BaudError::OutOfRange)?;
    USART3.ctlr1().modify(|w| w.set_ue(false));
    USART3.brr().write(|w| w.0 = brr);
    USART3.ctlr1().modify(|w| w.set_ue(true));
    CHAR_TIME_SYSTICKS.store(char_time_systicks(brr), Ordering::Relaxed);
    Ok(())
}

#[interrupt]
fn USART3() {
    // Backdate by one char-time so `tick` = wire-end (IDLE fires 9 bit-times later).
    let tick = SYSTICK
        .cntl()
        .read()
        .wrapping_sub(CHAR_TIME_SYSTICKS.load(Ordering::Relaxed));

    // Scope marker: PA7 high until DMA EN brackets the listener→fire chain.
    crate::debug::set();

    // STATR.IDLE is cleared by read-STATR-then-read-DATAR.
    let _ = USART3.statr().read();
    let _ = USART3.datar().read();

    let ndtr = DMA1.ch(2).ndtr().read().ndt();

    // SAFETY: ISR is the sole writer of LAST_NDTR and BYTES_TOTAL; main-task
    // only reads BYTES_TOTAL via volatile load. No RMW contention.
    let consumed = unsafe {
        let prev = *LAST_NDTR.get();
        *LAST_NDTR.get() = ndtr;
        prev.wrapping_sub(ndtr) as u32 & RX_BUF_MASK
    };
    let head = unsafe {
        let v = ptr::read_volatile(BYTES_TOTAL.get()).wrapping_add(consumed);
        ptr::write_volatile(BYTES_TOTAL.get(), v);
        v
    };

    let stamp = IdleStamp {
        tick,
        head: head as u16,
    };

    let h = STAMP_HEAD.load(Ordering::Relaxed);
    // SAFETY: SPSC producer; Release on STAMP_HEAD below publishes the slot.
    unsafe {
        (*STAMPS.get())[(h as usize) & STAMP_MASK] = stamp;
    }
    STAMP_HEAD.store(h.wrapping_add(1), Ordering::Release);

    // Kick off an `arm_after_idle` fire if one is pending. Cheap when not
    // in use (one atomic load); when armed, schedules SysTick CMP or fires
    // immediately depending on the configured offset.
    crate::inject::on_listen_idle(tick);

    crate::led::signal();
}
