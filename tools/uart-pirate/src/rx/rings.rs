//! RX byte ring: USART3 DATAR -> DMA1_CH3, circular, host-facing. The
//! drain reads bytes straight from this ring (no staging copy); the
//! cumulative `rx_total` count is the shared index domain for the
//! boundary recorder and the drain.
//!
//! CH3 sits ALONE at VERYHIGH on the DMA ladder: the arbiter preempts
//! per-beat, so an inbound byte is always ringed before the USART3
//! error service reads the cursor (`osc-servo-transport.md` transport
//! sec 7). TX (CH2) and its kickoff stamper (CH4) run at HIGH below it.
//!
//! Cursor discipline: `refresh_rx_total` mutates the NDTR bookkeeping
//! pair non-atomically -- callers are either the USART3 vector (the sole
//! wire-priority ISR touching it) or thread mode under a critical
//! section.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_metapac::dma::vals::{Dir, Pl, Size};
use ch32_metapac::{DMA1, USART3};

/// Byte ring length -- also the drain contract: more than `RING_LEN`
/// bytes undrained means DMA has lapped unread data (`stamp_overflow`).
pub const RING_LEN: usize = 1024;
const RING_MASK: u32 = (RING_LEN - 1) as u32;
const _: () = assert!(RING_LEN.is_power_of_two());

static RX_RING: SyncUnsafeCell<[u8; RING_LEN]> = SyncUnsafeCell::new([0; RING_LEN]);

static RX_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static LAST_RX_NDTR: SyncUnsafeCell<u16> = SyncUnsafeCell::new(RING_LEN as u16);

pub(super) fn init() {
    unsafe {
        // DMA1_CH3 = RX_RING (USART3 RX). Circular, 8-bit, no IRQ -- the
        // ring is drained by the host pull, boundaries by the FE service.
        let ch3 = DMA1.ch(2);
        ch3.par().write_value(USART3.datar().as_ptr() as u32);
        ch3.mar().write_value((*RX_RING.get()).as_ptr() as u32);
        ch3.ndtr().write(|w| w.set_ndt(RING_LEN as u16));
        ch3.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS8);
            w.set_psize(Size::BITS8);
            w.set_pl(Pl::VERYHIGH); // alone at the top -- drain beats the IRQ
            w.set_htie(false);
            w.set_tcie(false);
            w.set_en(true);
        });
    }
}

/// Live-NDTR total without committing the bookkeeping.
#[inline]
pub(super) fn peek_rx_total() -> (u32, u16) {
    let ndtr = DMA1.ch(2).ndtr().read().ndt();
    unsafe {
        let prev_ndtr = *LAST_RX_NDTR.get();
        let consumed = prev_ndtr.wrapping_sub(ndtr) as u32 & RING_MASK;
        let total = ptr::read_volatile(RX_TOTAL.get()).wrapping_add(consumed);
        (total, ndtr)
    }
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
pub(super) fn rx_at(idx: u32) -> u8 {
    unsafe { (*RX_RING.get())[(idx & RING_MASK) as usize] }
}
