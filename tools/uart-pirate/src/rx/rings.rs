//! Wire-side DMA-populated input rings.
//!
//! - `FALLING_LO` (u16) ← TIM2.CCR1 via DMA1_CH5 — low 16 of each IC tick.
//! - `FALLING_HI` (u16) ← TIM3.CCR1 via DMA1_CH6 — high 16, latched on
//!   the same TIM2 TRGO pulse that fired CH5. Pair forms a hardware
//!   atomic 32-bit tick; `falling_at` applies wrap-race correction.
//! - `RX_RING` (u8) ← USART3 DATAR via DMA1_CH3; received bytes (internal
//!   staging, not host-facing).
//!
//! `falling_total` is sourced from CH6's NDTR — CH6 is the trailing
//! writer of the CH5/CH6 pair (TIM3.CCR1 latches one TRC-sync cycle
//! after TIM2.CCR1, and DMA1 services CH5 before CH6 at equal priority),
//! so when CH6 has decremented past an entry, both halves are valid.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_metapac::dma::vals::{Dir, Pl, Size};
use ch32_metapac::{DMA1, TIM2, TIM3, USART3};

// IC ring. With event-driven walker, DMA1_CH6 HT fires at FALL_LEN/2
// entries — the walker drains promptly without waiting for a cadence
// tick. 256 gives ample slack for any sustained burst (3 Mbaud × 5
// edges/byte → 1 edge per 667 ns; HT-to-drain latency is bounded by
// PFIC dispatch + walker run, both sub-µs at PRIO_WALKER).
pub const FALL_LEN: usize = 256;
const FALL_MASK: u32 = (FALL_LEN - 1) as u32;
const _: () = assert!(FALL_LEN.is_power_of_two());

const RX_LEN: usize = 256;
const RX_MASK: u32 = (RX_LEN - 1) as u32;
const _: () = assert!(RX_LEN.is_power_of_two());

static FALLING_LO: SyncUnsafeCell<[u16; FALL_LEN]> = SyncUnsafeCell::new([0; FALL_LEN]);
static FALLING_HI: SyncUnsafeCell<[u16; FALL_LEN]> = SyncUnsafeCell::new([0; FALL_LEN]);
static RX_RING: SyncUnsafeCell<[u8; RX_LEN]> = SyncUnsafeCell::new([0; RX_LEN]);

/// IC entry count safely written by BOTH DMA1_CH5 (lo) and DMA1_CH6
/// (hi). Tracked via CH6's NDTR — CH6 is the trailing writer, so when
/// CH6 has decremented past an entry, both halves are valid.
static FALLING_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static LAST_FALLING_NDTR: SyncUnsafeCell<u16> = SyncUnsafeCell::new(FALL_LEN as u16);

static RX_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static LAST_RX_NDTR: SyncUnsafeCell<u16> = SyncUnsafeCell::new(RX_LEN as u16);

pub(super) fn init() {
    unsafe {
        // DMA1_CH5 = TIM2_CC1 IC → FALLING_LO. Circular, 16-bit. No IRQ;
        // CH6 is the trailing writer and drives the walker.
        let ch5 = DMA1.ch(4);
        ch5.par().write_value(TIM2.chcvr(0).as_ptr() as u32);
        ch5.mar().write_value((*FALLING_LO.get()).as_ptr() as u32);
        ch5.ndtr().write(|w| w.set_ndt(FALL_LEN as u16));
        ch5.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS16);
            w.set_psize(Size::BITS16);
            w.set_pl(Pl::VERYHIGH); // measurement clock — never queue
            w.set_htie(false);
            w.set_tcie(false);
            w.set_en(true);
        });

        // DMA1_CH6 = TIM3_CC1 IC → FALLING_HI. Circular, 16-bit. HTIE+
        // TCIE drive the walker; CH6 services AFTER CH5 (lower channel
        // number wins at equal priority), so when CH6's NDTR has
        // decremented past entry i, both `(lo[i], hi[i])` are valid.
        let ch6 = DMA1.ch(5);
        ch6.par().write_value(TIM3.chcvr(0).as_ptr() as u32);
        ch6.mar().write_value((*FALLING_HI.get()).as_ptr() as u32);
        ch6.ndtr().write(|w| w.set_ndt(FALL_LEN as u16));
        ch6.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS16);
            w.set_psize(Size::BITS16);
            w.set_pl(Pl::VERYHIGH);
            w.set_htie(true);
            w.set_tcie(true);
            w.set_en(true);
        });

        // DMA1_CH3 = RX_RING (USART3 RX). Circular, 8-bit. HTIE+TCIE
        // drive the walker when the RX ring half- or fully-fills.
        let ch3 = DMA1.ch(2);
        ch3.par().write_value(USART3.datar().as_ptr() as u32);
        ch3.mar().write_value((*RX_RING.get()).as_ptr() as u32);
        ch3.ndtr().write(|w| w.set_ndt(RX_LEN as u16));
        ch3.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS8);
            w.set_psize(Size::BITS8);
            w.set_pl(Pl::HIGH); // RX must not drop bytes at 3 Mbaud
            w.set_htie(true);
            w.set_tcie(true);
            w.set_en(true);
        });
    }
}

/// Live-NDTR total without mutating walker state. Returns `(total, ndtr)`
/// so the committing variant can store the matching NDTR without re-
/// reading the register. CH6 (TIM3.CCR1 high half) is the trailing DMA
/// writer: CH5 (TIM2) wins same-priority arbitration AND TIM3's TRC-
/// driven capture latches one sync cycle after TIM2's, so reading CH6's
/// NDTR bounds the count of fully-written 32-bit pairs.
#[inline]
pub(super) fn peek_falling_total() -> (u32, u16) {
    let ndtr = DMA1.ch(5).ndtr().read().ndt();
    unsafe {
        let prev_ndtr = *LAST_FALLING_NDTR.get();
        let consumed = prev_ndtr.wrapping_sub(ndtr) as u32 & FALL_MASK;
        let total = ptr::read_volatile(FALLING_TOTAL.get()).wrapping_add(consumed);
        (total, ndtr)
    }
}

#[inline]
pub(super) fn peek_rx_total() -> (u32, u16) {
    let ndtr = DMA1.ch(2).ndtr().read().ndt();
    unsafe {
        let prev_ndtr = *LAST_RX_NDTR.get();
        let consumed = prev_ndtr.wrapping_sub(ndtr) as u32 & RX_MASK;
        let total = ptr::read_volatile(RX_TOTAL.get()).wrapping_add(consumed);
        (total, ndtr)
    }
}

#[inline]
pub(super) fn refresh_falling_total() -> u32 {
    let (total, ndtr) = peek_falling_total();
    unsafe {
        *LAST_FALLING_NDTR.get() = ndtr;
        ptr::write_volatile(FALLING_TOTAL.get(), total);
    }
    total
}

#[inline]
pub(super) fn refresh_rx_total() -> u32 {
    let (total, ndtr) = peek_rx_total();
    unsafe {
        *LAST_RX_NDTR.get() = ndtr;
        ptr::write_volatile(RX_TOTAL.get(), total);
    }
    total
}

#[inline]
pub(super) fn falling_total_cached() -> u32 {
    unsafe { ptr::read_volatile(FALLING_TOTAL.get()) }
}

#[inline]
pub(super) fn rx_total_cached() -> u32 {
    unsafe { ptr::read_volatile(RX_TOTAL.get()) }
}

/// Read the hardware-atomic 32-bit IC capture pair at `idx` and apply
/// wrap-race correction.
///
/// The phase-locked TIM3 (started one AHB write ahead of TIM2 in
/// `tick::init`) increments a few cycles BEFORE each TIM2 wrap. If a
/// capture latches inside that gap, TIM3.CCR1 latches the new high half
/// while TIM2.CCR1 still holds the pre-wrap low half — combined value
/// reads as `actual + 65536`.
///
/// Detection uses two signals depending on whether `idx` is an interior
/// entry or the tail:
///
/// - **Interior (`idx + 1 < falling_total`)**: capture times in the ring
///   are monotonically non-decreasing (DMA writes them in order), so
///   `combined > combined_next` is a disjoint signal of the race on
///   `idx`. This is timing-independent: it works regardless of how many
///   TIM2 wraps elapse between capture and walker read, which the
///   earlier `combined > ceiling` predicate didn't survive at low baud /
///   long bursts. Two consecutive races would require TIM2 to wrap twice
///   between two captures — impossible since min inter-capture spacing
///   (filter delay = 256 cycles) is far below the wrap period (65536
///   cycles).
/// - **Tail (`idx + 1 == falling_total`)**: no successor exists. `ceiling`
///   was sampled microseconds after the latest IC pair was written
///   (NDTR refresh precedes `read_tick32`), so it is within one wrap of
///   the tail entry's capture time and the original `combined > ceiling`
///   predicate is reliable here.
#[inline]
pub(super) fn falling_at(idx: u32, falling_total: u32, ceiling: u32) -> u32 {
    let i = (idx & FALL_MASK) as usize;
    let lo = unsafe { (*FALLING_LO.get())[i] };
    let hi = unsafe { (*FALLING_HI.get())[i] };
    let combined = ((hi as u32) << 16) | (lo as u32);

    if idx.wrapping_add(1) != falling_total {
        let ni = (idx.wrapping_add(1) & FALL_MASK) as usize;
        let lo_n = unsafe { (*FALLING_LO.get())[ni] };
        let hi_n = unsafe { (*FALLING_HI.get())[ni] };
        let combined_next = ((hi_n as u32) << 16) | (lo_n as u32);
        if combined != combined_next && combined.wrapping_sub(combined_next) <= u32::MAX / 2 {
            return combined.wrapping_sub(1 << 16);
        }
        combined
    } else if combined != ceiling && combined.wrapping_sub(ceiling) <= u32::MAX / 2 {
        combined.wrapping_sub(1 << 16)
    } else {
        combined
    }
}

#[inline]
pub(super) fn rx_at(idx: u32) -> u8 {
    unsafe { (*RX_RING.get())[(idx & RX_MASK) as usize] }
}
