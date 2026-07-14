//! Edge-capture provider -- binds the engine's `EdgeCapture` to the TIM2
//! capture rings (DMA1_CH1 falls / DMA1_CH7 rises, armed once at bringup).
//! Drains are consumer-paced; `poll_accumulate` (main loop) keeps the
//! written totals honest between drains so a full ring lap can never
//! masquerade as fresh data.
//!
//! Access discipline: every method here runs in MAIN-loop context only
//! (the link server's dispatch and the run loop) -- no ISR touches the
//! capture state, so the plain statics need no synchronization beyond the
//! single-context contract.

use core::cell::SyncUnsafeCell;

use osc_host::traits;

use crate::hal::dma;

/// Entries per ring (2 KB each). At the worst-case sustained edge rate
/// (alternating bits at 3M = one edge per bit) a ring holds ~340 us of
/// traffic -- orders past the main loop's poll cadence.
const CAP_LEN: usize = 1024;

static FALLS: SyncUnsafeCell<[u16; CAP_LEN]> = SyncUnsafeCell::new([0; CAP_LEN]);
static RISES: SyncUnsafeCell<[u16; CAP_LEN]> = SyncUnsafeCell::new([0; CAP_LEN]);

struct Side {
    /// Ring write position at the last accumulate, entries.
    last_pos: u16,
    /// Entries ever written (accumulated) / handed out.
    written: u32,
    drained: u32,
}

struct State {
    falls: Side,
    rises: Side,
    overflow: bool,
}

static STATE: SyncUnsafeCell<State> = SyncUnsafeCell::new(State {
    falls: Side {
        last_pos: 0,
        written: 0,
        drained: 0,
    },
    rises: Side {
        last_pos: 0,
        written: 0,
        drained: 0,
    },
    overflow: false,
});

/// Production binding to the TIM2 capture rings.
pub struct Edges;

impl Edges {
    pub const LEN: usize = CAP_LEN;

    pub fn falls_addr() -> u32 {
        // SAFETY: address-of a `'static` cell, handed to the DMA controller.
        unsafe { (*FALLS.get()).as_ptr() as u32 }
    }

    pub fn rises_addr() -> u32 {
        // SAFETY: as above.
        unsafe { (*RISES.get()).as_ptr() as u32 }
    }

    /// Fold ring progress into the running totals; the main loop calls this
    /// every iteration (no self, no CS: the capture state is main-loop-only
    /// by the module contract, never ISR-shared). Sound while under one
    /// full lap lands between calls (~340 us at the worst-case edge rate
    /// vs a us-scale loop).
    pub fn poll_accumulate() {
        // SAFETY: main-loop-only access (module contract).
        let s = unsafe { &mut *STATE.get() };
        for (side, ch) in [
            (&mut s.falls, dma::Channel::CH1),
            (&mut s.rises, dma::Channel::CH7),
        ] {
            let pos = CAP_LEN as u16 - dma::remaining(ch);
            let delta = (pos + CAP_LEN as u16 - side.last_pos) % CAP_LEN as u16;
            side.last_pos = pos;
            side.written = side.written.wrapping_add(delta as u32);
            if side.written.wrapping_sub(side.drained) > CAP_LEN as u32 {
                s.overflow = true;
            }
        }
    }
}

fn drain(side: &mut Side, ring: &SyncUnsafeCell<[u16; CAP_LEN]>, buf: &mut [u16]) -> usize {
    if side.written.wrapping_sub(side.drained) > CAP_LEN as u32 {
        // Lapped (overflow already flagged): realign to the oldest entry
        // still intact so the drain returns the newest window in order.
        side.drained = side.written - CAP_LEN as u32;
    }
    let avail = side.written.wrapping_sub(side.drained);
    let n = buf.len().min(avail as usize);
    // SAFETY: DMA-owned storage read in place; entries below `written` are
    // complete (the controller finishes a 16-bit beat atomically).
    let data = unsafe { &*ring.get() };
    for (k, slot) in buf[..n].iter_mut().enumerate() {
        *slot = data[(side.drained as usize + k) % CAP_LEN];
    }
    side.drained = side.drained.wrapping_add(n as u32);
    n
}

impl traits::EdgeCapture for Edges {
    fn drain_falls(&mut self, buf: &mut [u16]) -> usize {
        Self::poll_accumulate();
        // SAFETY: main-loop-only access (module contract).
        let s = unsafe { &mut *STATE.get() };
        drain(&mut s.falls, &FALLS, buf)
    }

    fn drain_rises(&mut self, buf: &mut [u16]) -> usize {
        Self::poll_accumulate();
        // SAFETY: as above.
        let s = unsafe { &mut *STATE.get() };
        drain(&mut s.rises, &RISES, buf)
    }

    fn overflow(&self) -> bool {
        // SAFETY: as above.
        unsafe { &*STATE.get() }.overflow
    }

    fn reset(&mut self) {
        Self::poll_accumulate();
        // SAFETY: as above.
        let s = unsafe { &mut *STATE.get() };
        s.falls.drained = s.falls.written;
        s.rises.drained = s.rises.written;
        s.overflow = false;
    }
}
