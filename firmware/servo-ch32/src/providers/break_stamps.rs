//! Break-stamp provider (transport sec 8): DMA1 CH2 copies a count into a
//! circular ring on every break-detector overflow (TIM2_UP), one halfword
//! per break however late its wake is served, and none for the servo's own
//! breaks (`BreakWake::muted` drops the request with the interrupt). In
//! traffic the count is the RX ring's (DMA1 CNTR5), so a stamp is the
//! break's ring position, latched at the detector's fire. Through a CAL
//! train it is TIM3's, which counts HCLK, the SysTick rate, so stamp
//! differences are SysTick ticks.

use core::cell::SyncUnsafeCell;

use osc_servo_drivers::traits::bus;

use super::ring::RxRing;
use crate::hal::timer::tim3;
use crate::hal::{dma, rcc};

/// Stamps the ring holds before a lap overwrites the oldest: a CAL wake
/// lagging this many marks loses the train's gaps to the gate (transport
/// sec 8). A power of two: the index wraps by mask.
const DEPTH: u16 = 8;

static RING: SyncUnsafeCell<[u16; DEPTH as usize]> = SyncUnsafeCell::new([0; DEPTH as usize]);

const CFG: dma::Config = dma::Config {
    dir: dma::Dir::FROMPERIPHERAL,
    circ: true,
    pinc: false,
    minc: true,
    size: dma::Size::BITS16,
    htie: false,
    tcie: false,
    // HIGH (see `hal::dma`): under RX, above the reply copies.
    pl: dma::Pl::HIGH,
};

/// `next` once the ring ruler has latched: no entry index matches it.
const LIVE: u16 = DEPTH;

/// Production binding to DMA1_CH2 (TIM2_UP -> CNTR5 or TIM3 CNT -> the
/// stamp ring).
pub struct BreakStamps {
    /// The ring entry the next take reads; on the ring ruler, the entry CH2
    /// wrote next at the re-point, until the first stamp makes it [`LIVE`].
    next: u16,
    /// The latch reads TIM3, for a CAL train.
    time: bool,
}

impl BreakStamps {
    pub const fn new() -> Self {
        Self {
            next: 0,
            time: false,
        }
    }

    /// One-shot bring-up, ahead of the detector's update request
    /// (`BreakWake::init`): TIM3 free-running, CH2 armed on the RX ring.
    pub fn init() {
        rcc::enable_tim3();
        tim3::start();
        dma::configure(
            dma::Channel::CH2,
            &CFG,
            dma::count_addr(dma::Channel::CH5),
            RING.get() as u32,
            DEPTH,
        );
        dma::enable(dma::Channel::CH2);
    }

    /// Point CH2 at `src`, from the ring's first entry. A request latched
    /// while the channel was off lands at the enable, reading `src` then,
    /// before any break could fire (one needs 9.5 bit-times): skipped.
    fn repoint(&mut self, src: u32, time: bool) {
        dma::rearm(
            dma::Channel::CH2,
            dma::cr_of(&CFG),
            src,
            RING.get() as u32,
            DEPTH,
        );
        self.next = written();
        self.time = time;
    }
}

impl Default for BreakStamps {
    fn default() -> Self {
        Self::new()
    }
}

/// The ring entry CH2 writes next. The remaining count reloads to DEPTH
/// with each lap; a transient 0 masks to the same entry.
#[inline(always)]
fn written() -> u16 {
    DEPTH.wrapping_sub(dma::remaining(dma::Channel::CH2)) & (DEPTH - 1)
}

/// Entry `i` of the stamp ring.
#[inline(always)]
fn entry(i: u16) -> u16 {
    // SAFETY: DMA-owned storage, read in place; the caller reads an entry
    // behind the remaining count it observed, so its transfer has
    // completed. The mask keeps the index in bounds.
    unsafe { core::ptr::read_volatile(&raw const (*RING.get())[(i & (DEPTH - 1)) as usize]) }
}

impl bus::BreakStamps for BreakStamps {
    #[inline(always)]
    fn newest(&mut self) -> Option<u16> {
        let w = written();
        if self.time || w == self.next {
            return None;
        }
        self.next = LIVE;
        // CNTR5 counts down from the ring's length; a transient 0 is the
        // reload, index 0.
        let len = RxRing::LEN as u16;
        Some(len.wrapping_sub(entry(w.wrapping_sub(1))) & (len - 1))
    }

    fn take(&mut self) -> Option<u16> {
        if written() == self.next {
            return None;
        }
        let i = self.next;
        self.next = (i + 1) & (DEPTH - 1);
        Some(entry(i))
    }

    fn time(&mut self) {
        self.repoint(tim3::counter_addr(), true);
    }

    fn position(&mut self) {
        self.repoint(dma::count_addr(dma::Channel::CH5), false);
    }
}
