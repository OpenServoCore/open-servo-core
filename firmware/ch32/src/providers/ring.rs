//! RX ring provider (osc-native §4.1) — binds `RxRing` to DMA1_CH5, the
//! USART1 RX byte ring armed once at boot and read as a counted cursor.
//!
//! The ring is `#[repr(align(2))]`: an even base is load-bearing so every
//! frame anchor lands halfword-aligned for the SPI-CRC engine (§3.2), and
//! V006 DMA cannot feed an odd address anyway (F12).

use core::cell::SyncUnsafeCell;

use osc_drivers::traits::bus;

use crate::hal::dma;

/// Ring depth (§11): must exceed the 258 B max frame with lap margin.
const RING_LEN: usize = 512;

#[repr(align(2))]
struct Ring([u8; RING_LEN]);

static RING: SyncUnsafeCell<Ring> = SyncUnsafeCell::new(Ring([0; RING_LEN]));

/// Production binding to DMA1_CH5 (USART1 RX → the circular ring).
pub struct RxRing;

impl RxRing {
    pub const LEN: usize = RING_LEN;

    /// Ring base for the CH5 arm in `runtime::init`.
    pub fn base_addr() -> u32 {
        // SAFETY: address-of a `'static` cell; the returned pointer is stable
        // for the whole program and only handed to the DMA controller.
        unsafe { (*RING.get()).0.as_ptr() as u32 }
    }
}

impl bus::RxRing for RxRing {
    #[inline(always)]
    fn bytes(&self) -> &[u8] {
        // SAFETY: DMA-owned storage; the driver reads it in place. All `&mut`
        // access into the bus composite is serialized at PFIC HIGH, so no
        // aliasing `&mut [u8]` to the ring exists.
        unsafe { &(*RING.get()).0 }
    }

    #[inline(always)]
    fn cursor(&self) -> u16 {
        RING_LEN as u16 - dma::remaining(dma::Channel::CH5)
    }
}
