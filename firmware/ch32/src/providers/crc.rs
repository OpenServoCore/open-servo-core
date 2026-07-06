//! CRC-engine provider (osc-native §3.2, F6) — SPI1 as a DMA-fed CRC
//! coprocessor. DMA1_CH3 shifts the covered span through SPI1's CRC unit
//! (CRC-16/BUYPASS over 16-bit big-endian halfwords = osc-CRC-16); the
//! accumulator holds across arms so a ring-wrap split or a multi-span read
//! sums into one CRC. No pins are AF-mapped — nothing leaves the package.
//!
//! Register recipe ported from the `spi_crc` bringup spike (case 9: CRCEN
//! held across feeds accumulates).

use ch32_metapac::SPI1;
use ch32_metapac::spi::vals::BaudRate as SpiBaud;
use osc_drivers::traits::bus;

use crate::hal::dma;
use crate::hal::rcc;

/// osc-CRC-16 polynomial (§3.2: CRC-16/BUYPASS).
const OSC_CRC_POLY: u16 = 0x8005;

/// Bounded drain spin between successive feeds (the engine runs ~8× wire
/// speed, F6). An expiry leaves a partial accumulator → the frame fails CRC
/// and drops, which is the sanctioned "spin miss = fail" outcome (§3.2).
const FEED_DRAIN_SPIN: u32 = 4096;

/// Production binding to SPI1 + DMA1_CH3.
pub struct Crc;

impl Crc {
    /// One-shot peripheral bring-up (driver-pattern §5.5): clock-gate SPI1,
    /// lock master / SSM / 16-bit mode, load the poly, enable the calculator
    /// last (resets it with the mode fixed), then SPE + TX-DMA. Held live for
    /// the whole program — `reset` re-zeros the accumulator per frame.
    pub fn init() {
        rcc::enable_spi1();
        SPI1.ctlr1().write(|w| {
            w.set_mstr(true);
            w.set_ssm(true);
            w.set_ssi(true);
            w.set_br(SpiBaud::DIV_2);
            w.set_dff(true); // 16-bit frames → halfword-fed BUYPASS (§3.2)
            w.set_lsbfirst(false);
        });
        SPI1.crcr().write(|w| w.set_crcpoly(OSC_CRC_POLY));
        SPI1.ctlr1().modify(|w| w.set_crcen(true));
        SPI1.ctlr2().write(|w| w.set_txdmaen(true));
        SPI1.ctlr1().modify(|w| w.set_spe(true));
    }

    /// DMA emptied and the shifter idled (spike `drain` predicate).
    #[inline(always)]
    fn drained() -> bool {
        dma::remaining(dma::Channel::CH3) == 0
            && SPI1.statr().read().txe()
            && !SPI1.statr().read().bsy()
    }

    fn arm(maddr: u32, halfwords: u16) {
        dma::disable(dma::Channel::CH3);
        let cfg = dma::Config {
            dir: dma::Dir::FROMMEMORY,
            circ: false,
            pinc: false,
            minc: true,
            size: dma::Size::BITS16,
            htie: false,
            tcie: false,
            pl: dma::Pl::HIGH,
        };
        dma::configure(
            dma::Channel::CH3,
            &cfg,
            SPI1.datar().as_ptr() as u32,
            maddr,
            halfwords,
        );
        dma::enable(dma::Channel::CH3);
    }
}

impl bus::CrcEngine for Crc {
    fn reset(&mut self) {
        dma::disable(dma::Channel::CH3);
        // Toggling CRCEN off→on clears the accumulator with the mode locked.
        SPI1.ctlr1().modify(|w| w.set_crcen(false));
        SPI1.ctlr1().modify(|w| w.set_crcen(true));
    }

    fn feed(&mut self, span: &[u8]) {
        // Drain a prior arm before re-pointing the channel (bounded).
        let mut budget = FEED_DRAIN_SPIN;
        while !Self::drained() && budget != 0 {
            budget -= 1;
            core::hint::spin_loop();
        }
        // Caller guarantees even addr/len (§3.2, F12): DMA reads halfwords.
        Self::arm(span.as_ptr() as u32, (span.len() / 2) as u16);
    }

    fn result(&mut self) -> Option<u16> {
        if Self::drained() {
            dma::disable(dma::Channel::CH3);
            Some(SPI1.tcrcr().read().txcrc())
        } else {
            None
        }
    }
}
