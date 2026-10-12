//! RX ring provider -- binds the engine's `RxRing` to DMA1_CH3, the USART3
//! RX byte ring armed once at bringup and read as a running byte count,
//! plus the break marks the USART3 vector publishes. The "never contains
//! own TX" contract is enforced upstream: RE is off for the whole claim
//! window (`tx_wire`), so the RX pin's echo reaches neither DMA nor the
//! break detector.

use core::cell::SyncUnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering};

use osc_host::traits;
use osc_protocol::wire;

use crate::hal::dma;

/// Power of two exceeding the 258 B max frame with polling margin
/// (trait contract) -- and past a full ENUM collect burst.
const RING_LEN: usize = 1024;

static RING: SyncUnsafeCell<[u8; RING_LEN]> = SyncUnsafeCell::new([0; RING_LEN]);

/// Holds the marks of a whole unread ring of the shortest legal frames, so
/// only junk breaks can lap it before the byte ring laps.
const MARKS_LEN: usize = (RING_LEN / wire::footprint(wire::len_for(0))).next_power_of_two();

static MARKS: SyncUnsafeCell<[u32; MARKS_LEN]> = SyncUnsafeCell::new([0; MARKS_LEN]);
static MARKED: AtomicU32 = AtomicU32::new(0);

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

    /// The break detector fired: the break's `0x00` is the last byte in.
    /// Exact while the caller runs within a byte-time of the break's end
    /// (the USART3 vector preempts every engine body to hold that).
    #[inline(always)]
    pub fn mark_break() {
        let at = Self::poll_accumulate().wrapping_sub(1);
        let n = MARKED.load(Ordering::Relaxed);
        // SAFETY: the USART3 vector is the only writer and never nests in
        // itself; readers hold the slot only until the published count
        // laps it (trait contract).
        unsafe { (*MARKS.get())[n as usize & (MARKS_LEN - 1)] = at };
        MARKED.store(n.wrapping_add(1), Ordering::Release);
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

    #[inline(always)]
    fn marks(&self) -> &[u32] {
        // SAFETY: as `bytes`: read in place, the vector writes only slots
        // past the published count.
        let marks: &[u32; MARKS_LEN] = unsafe { &*MARKS.get() };
        marks
    }

    #[inline(always)]
    fn marked(&self) -> u32 {
        MARKED.load(Ordering::Acquire)
    }
}
