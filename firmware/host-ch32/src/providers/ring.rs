//! RX ring provider -- binds the engine's `RxRing` to DMA1_CH3, the USART3
//! RX byte ring armed once at bringup and read as a counted cursor. The
//! "never contains own TX" contract is enforced upstream: RE is off for
//! the whole claim window (`tx_wire`), so the HDSEL echo never generates a
//! DMA request.

use core::cell::SyncUnsafeCell;

use osc_host::traits;

use crate::hal::dma;

/// Power of two exceeding the 258 B max frame with polling margin
/// (trait contract) -- and past a full ENUM collect burst.
const RING_LEN: usize = 1024;

static RING: SyncUnsafeCell<[u8; RING_LEN]> = SyncUnsafeCell::new([0; RING_LEN]);

/// Production binding to DMA1_CH3 (USART3 RX -> the circular ring).
pub struct RxRing;

impl RxRing {
    pub const LEN: usize = RING_LEN;

    /// Ring base for the CH3 arm in `runtime::init`.
    pub fn base_addr() -> u32 {
        // SAFETY: address-of a `'static` cell; stable for the whole program
        // and only handed to the DMA controller.
        unsafe { (*RING.get()).as_ptr() as u32 }
    }
}

impl traits::RxRing for RxRing {
    #[inline(always)]
    fn bytes(&self) -> &[u8] {
        // SAFETY: DMA-owned storage read in place; no `&mut` to the ring
        // exists anywhere (the engine only reads through this view).
        let ring: &[u8; RING_LEN] = unsafe { &*RING.get() };
        ring
    }

    #[inline(always)]
    fn cursor(&self) -> u16 {
        RING_LEN as u16 - dma::remaining(dma::Channel::CH3)
    }
}
