//! Break-stamp provider (transport sec 8): DMA1 CH2 copies TIM3's count
//! into a circular ring on every break-detector overflow (TIM2_UP), one
//! halfword per break however late its wake is served, and none for the
//! servo's own breaks (`BreakWake::muted` drops the request with the
//! interrupt). TIM3 counts HCLK, the SysTick rate, so stamp differences
//! are SysTick ticks.

use core::cell::SyncUnsafeCell;

use osc_servo_drivers::traits::bus;

use crate::hal::timer::tim3;
use crate::hal::{dma, rcc};

/// Stamps the ring holds before a lap overwrites the oldest: a CAL wake
/// lagging this many marks loses the train's gaps to the gate (transport
/// sec 8). A power of two: the index wraps by mask.
const DEPTH: u16 = 8;

static RING: SyncUnsafeCell<[u16; DEPTH as usize]> = SyncUnsafeCell::new([0; DEPTH as usize]);

/// Production binding to DMA1_CH2 (TIM2_UP -> TIM3 CNT -> the stamp ring).
pub struct BreakStamps {
    /// The ring entry the next take reads.
    next: u16,
}

impl BreakStamps {
    pub const fn new() -> Self {
        Self { next: 0 }
    }

    /// One-shot bring-up, ahead of the detector's update request
    /// (`BreakWake::init`): TIM3 free-running, CH2 armed on it.
    pub fn init() {
        rcc::enable_tim3();
        tim3::start();
        let cfg = dma::Config {
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
        dma::configure(
            dma::Channel::CH2,
            &cfg,
            tim3::counter_addr(),
            RING.get() as u32,
            DEPTH,
        );
        dma::enable(dma::Channel::CH2);
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

impl bus::BreakStamps for BreakStamps {
    fn take(&mut self) -> Option<u16> {
        if written() == self.next {
            return None;
        }
        let i = self.next;
        self.next = (i + 1) & (DEPTH - 1);
        // SAFETY: DMA-owned storage, read in place; the entry is behind the
        // remaining count observed above, so its transfer has completed.
        Some(unsafe { core::ptr::read_volatile(&raw const (*RING.get())[i as usize]) })
    }

    fn clear(&mut self) {
        self.next = written();
    }
}
