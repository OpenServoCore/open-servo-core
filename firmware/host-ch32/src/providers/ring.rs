//! RX ring provider -- binds the engine's `RxRing` to DMA1_CH3, the USART3
//! RX byte ring armed once at bringup and read as a running byte count.
//! The "never contains own TX" contract is enforced upstream: RE is off
//! for the whole claim window (`tx_wire`), so the HDSEL echo never
//! generates a DMA request.

use core::cell::SyncUnsafeCell;

use osc_host::traits;

use crate::hal::dma;

/// Power of two exceeding the 258 B max frame with polling margin
/// (trait contract) -- and past a full ENUM collect burst.
const RING_LEN: usize = 1024;

static RING: SyncUnsafeCell<[u8; RING_LEN]> = SyncUnsafeCell::new([0; RING_LEN]);

struct Progress {
    /// Ring write position at the last fold.
    pos: u16,
    written: u32,
}

static PROGRESS: SyncUnsafeCell<Progress> = SyncUnsafeCell::new(Progress { pos: 0, written: 0 });

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

    /// Fold DMA progress into the running count. The main loop calls this
    /// every pass, so the count stays exact while under one ring lands
    /// between passes (3.4 ms at 3M) even when nothing walks the ring.
    pub fn poll_accumulate() -> u32 {
        critical_section::with(|_| {
            // SAFETY: every access runs inside this critical section (main
            // loop and the USART3 TC path alike).
            let p = unsafe { &mut *PROGRESS.get() };
            let pos = RING_LEN as u16 - dma::remaining(dma::Channel::CH3);
            let landed = pos.wrapping_sub(p.pos) as u32 & (RING_LEN as u32 - 1);
            p.pos = pos;
            p.written = p.written.wrapping_add(landed);
            p.written
        })
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
    fn written(&self) -> u32 {
        Self::poll_accumulate()
    }
}
