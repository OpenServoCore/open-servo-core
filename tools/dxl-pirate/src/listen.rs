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
use dxl_pirate::parse::brr_for;
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

/// Plain = a bus IDLE we observed but didn't initiate. Round = the slave-reply
/// end of a `master_send` round-trip; carries the matching `T_request_end` /
/// `T_first` so the host gets the full cal triple atomically per DRAIN.
#[derive(Copy, Clone)]
pub enum IdleStamp {
    Plain {
        /// SysTick CNT low half at wire-end (HCLK/8 ticks).
        tick: u32,
        /// DMA write head (bytes received since boot, low 16 bits).
        head: u16,
    },
    Round {
        /// `T_request_end`: master's USART1 TC stamp.
        req: u32,
        /// `T_first`: slave-reply T0 stamp from USART3 RXNE one-shot.
        first: u32,
        /// `T_last`: this IDLE's wire-end stamp.
        last: u32,
        head: u16,
    },
}

static STAMPS: SyncUnsafeCell<[IdleStamp; STAMP_RING_LEN]> =
    SyncUnsafeCell::new([IdleStamp::Plain { tick: 0, head: 0 }; STAMP_RING_LEN]);
static STAMP_HEAD: AtomicU32 = AtomicU32::new(0);
static STAMP_TAIL: AtomicU32 = AtomicU32::new(0);

// Single-writer (USART3 ISR), main-task reader. Single-word, tear-free on
// V203's aligned u32 stores → volatile cell instead of an atomic. Wraps; the
// host compares relative deltas.
static BYTES_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
// ISR-only; never touched from anywhere else. Plain cell.
static LAST_NDTR: SyncUnsafeCell<u16> = SyncUnsafeCell::new(RX_BUF_LEN as u16);

/// SysTick.CNTL captured on the first byte received after a `master_send` —
/// i.e., the slave's reply wire-start. Cal model's `T_first`. Sole writer =
/// USART3 RXNE branch; cleared by reading via `last_t_first`.
static T_FIRST: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

pub fn last_t_first() -> u32 {
    unsafe { ptr::read_volatile(T_FIRST.get()) }
}

/// Called from `master_send` so a no-reply round-trip doesn't carry the
/// prior trip's `T_first` into its ROUND entry. RXNE handler is the only
/// other writer; master_send runs inside a critical section so this write
/// can't race the RXNE branch.
pub(crate) fn reset_t_first() {
    unsafe { ptr::write_volatile(T_FIRST.get(), 0) };
}

/// Arm the USART3 RXNE one-shot outside the MASTER round-trip flow so the
/// next received byte stamps `T_FIRST`. Used by host timing tests that want
/// sample-clock-resolution wire-edge stamps instead of the IDLE flag's
/// ~½ bit-time granularity (FIRE jitter measurement). Held under a CS so the
/// flag/cell/RXNEIE writes can't be split by an in-flight USART3 ISR.
pub fn prime_rxne() {
    critical_section::with(|_| unsafe {
        ptr::write_volatile(T_FIRST.get(), 0);
        crate::inject::EXPECT_FIRST_BYTE.store(true, Ordering::Release);
        USART3.ctlr1().modify(|w| w.set_rxneie(true));
    });
}

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

/// Same as `drain_stamp` but doesn't advance the tail. Used by `XFER` to
/// detect the reply IDLE without consuming the Round entry — DRAIN still
/// surfaces (req, first, last) to the host afterward.
pub fn peek_stamp() -> Option<IdleStamp> {
    let tail = STAMP_TAIL.load(Ordering::Relaxed);
    if tail == STAMP_HEAD.load(Ordering::Acquire) {
        return None;
    }
    Some(unsafe { (*STAMPS.get())[(tail as usize) & STAMP_MASK] })
}

pub fn byte_count() -> u32 {
    unsafe { ptr::read_volatile(BYTES_TOTAL.get()) }
}

/// Read one byte from the circular RX DMA buffer by its absolute byte-count
/// address. Wraps via `RX_BUF_MASK`; the caller must keep `addr` within the
/// last `RX_BUF_LEN` of `byte_count()` or the byte will have been overwritten.
pub fn read_byte(addr: u32) -> u8 {
    unsafe { (*RX_BUF.get())[(addr & RX_BUF_MASK) as usize] }
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
    // Both IDLE and RXNE share this vector; dispatch by IDLE flag. STATR.RXNE
    // reads 0 under DMAR (see reference_v006_rxne_during_dma_rx in memory),
    // so we can't gate on it — but RXNEIE is only ever armed by the IDLE
    // branch below, so IDLE=0 implies "this was a RXNE wake".
    let tick_now = SYSTICK.cntl().read();
    let statr = USART3.statr().read();

    if !statr.idle() {
        // Always one-shot — mask RXNEIE unconditionally so per-byte ISRs don't
        // pile up for the rest of the reply, and so a stale RXNEIE left armed
        // by a no-reply prior trip self-disarms on our own TX-echo byte.
        // EXPECT_FIRST_BYTE is set only by the IDLE-suppress branch below, so
        // a pre-suppress RXNE (= master TX echo) finds it false and skips the
        // stamp; the real slave-reply T0 finds it true and stamps T_FIRST.
        USART3.ctlr1().modify(|w| w.set_rxneie(false));
        if crate::inject::EXPECT_FIRST_BYTE.swap(false, Ordering::AcqRel) {
            unsafe { ptr::write_volatile(T_FIRST.get(), tick_now) };
        }
        crate::led::signal();
        return;
    }

    // ── IDLE path. Backdate by one char-time so `tick` = wire-end.
    let tick = tick_now.wrapping_sub(CHAR_TIME_SYSTICKS.load(Ordering::Relaxed));

    // Scope marker: PA7 high until DMA EN brackets the listener→fire chain.
    crate::debug::set();

    // STATR was already read above; DATAR read clears IDLE.
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

    // Three IDLE flavors, distinguished by `master_send`-set flags:
    //   - SUPPRESS_NEXT_IDLE_STAMP: the master TX-echo IDLE; no stamp, arm
    //     RXNEIE for the slave reply's T0, and flag the next IDLE as the
    //     reply-end so it publishes a Round entry.
    //   - EXPECT_REPLY_END_IDLE: the slave-reply end IDLE; push a Round
    //     entry with (T_request_end, T_first, T_last, head).
    //   - neither: a plain bus IDLE we didn't initiate; push a Plain entry.
    let suppress = crate::inject::SUPPRESS_NEXT_IDLE_STAMP.swap(false, Ordering::AcqRel);
    if suppress {
        crate::inject::EXPECT_REPLY_END_IDLE.store(true, Ordering::Release);
        crate::inject::EXPECT_FIRST_BYTE.store(true, Ordering::Release);
        USART3.ctlr1().modify(|w| w.set_rxneie(true));
    } else {
        let stamp = if crate::inject::EXPECT_REPLY_END_IDLE.swap(false, Ordering::AcqRel) {
            IdleStamp::Round {
                req: crate::inject::last_master_request_end(),
                first: unsafe { ptr::read_volatile(T_FIRST.get()) },
                last: tick,
                head: head as u16,
            }
        } else {
            IdleStamp::Plain {
                tick,
                head: head as u16,
            }
        };

        let h = STAMP_HEAD.load(Ordering::Relaxed);
        // SAFETY: SPSC producer; Release on STAMP_HEAD below publishes the slot.
        unsafe {
            (*STAMPS.get())[(h as usize) & STAMP_MASK] = stamp;
        }
        STAMP_HEAD.store(h.wrapping_add(1), Ordering::Release);
    }

    // Kick off an `arm_after_idle` fire if one is pending. Cheap when not
    // in use (one atomic load); when armed, programs TIM4 OPM or fires
    // immediately depending on the configured offset.
    crate::inject::on_listen_idle(tick);

    crate::led::signal();
}
