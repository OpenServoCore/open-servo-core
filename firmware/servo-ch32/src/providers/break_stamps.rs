//! Break-stamp provider (transport sec 8): DMA1 CH2 copies TIM3's count
//! into a circular ring on every break-detector overflow (TIM2_UP), one
//! halfword per break however late its wake is served, and none for the
//! servo's own breaks (`BreakWake::muted` drops the request with the
//! interrupt). TIM3 counts HCLK, the SysTick clock, so a stamp is the low
//! 16 bits of a SysTick tick; the driver places it from its break's ring
//! position.

use core::cell::SyncUnsafeCell;

use osc_servo_drivers::traits::bus;

use crate::hal::timer::tim3;
use crate::hal::{dma, rcc};

/// Stamps the ring holds before a lap overwrites the oldest: a stamp waits
/// for its frame's verdict, so the ring must hold the ladder's backlog -
/// 32 ten-byte frames is 320 B of the 512 B RX ring, whose own lap is the
/// stream's bound. A lap here costs its pairs, never the stream. A power
/// of two: the index wraps by mask.
const DEPTH: u16 = 32;

static RING: SyncUnsafeCell<[u16; DEPTH as usize]> = SyncUnsafeCell::new([0; DEPTH as usize]);

/// Production binding to DMA1_CH2 (TIM2_UP -> TIM3 CNT -> the stamp ring).
pub struct BreakStamps {
    /// CH2's remaining count once the newest stamp taken had landed: equal
    /// to the live count while nothing newer has.
    taken: u16,
}

impl BreakStamps {
    pub const fn new() -> Self {
        Self { taken: DEPTH }
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

/// The remaining count never reads 0 in circular mode: it reloads with the
/// lap. A 0 would read as a lap pending; fold it onto the reload.
#[inline(always)]
fn remaining() -> u16 {
    match dma::remaining(dma::Channel::CH2) {
        0 => DEPTH,
        n => n,
    }
}

impl bus::BreakStamps for BreakStamps {
    fn take(&mut self) -> Option<u16> {
        let stamp = self.peek()?;
        self.taken = if self.taken == 1 {
            DEPTH
        } else {
            self.taken - 1
        };
        Some(stamp)
    }

    fn peek(&self) -> Option<u16> {
        if remaining() == self.taken {
            return None;
        }
        // The entry the transfer that left `taken` remaining wrote next.
        let i = (DEPTH - self.taken) & (DEPTH - 1);
        // SAFETY: DMA-owned storage, read in place; the entry is behind the
        // remaining count observed above, so its transfer has completed.
        Some(unsafe { core::ptr::read_volatile(&raw const (*RING.get())[i as usize]) })
    }

    fn clear(&mut self) -> u16 {
        let now = remaining();
        let dropped = self.taken.wrapping_sub(now) & (DEPTH - 1);
        self.taken = now;
        dropped
    }
}
