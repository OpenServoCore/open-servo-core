//! CRC-engine provider (osc-native §3.2, F6) — SPI1 as a DMA-fed CRC
//! coprocessor. DMA1_CH3 shifts the covered span through SPI1's CRC unit
//! (16-bit LSB-first = natural-order CRC-16/ARC = osc-CRC-16, §3.2); the
//! accumulator holds across arms so a ring-wrap split or a multi-span read
//! sums into one CRC.
//!
//! A remap places FUNCTIONS, used or not (the TIM1-Remap7 CH1N-on-PC0
//! lesson): SPI1's reset mapping puts SCK/PC5 and MOSI/PC6 on the motor's
//! TIM1 AF pins, where every CRC feed would burst 24 MHz clock into the
//! gate-drive mux. Remap 101 parks SCK/PA1 and MOSI/PA2 on analog-mode
//! pins (digital driver disconnected — provably inert on this board),
//! MISO/PB5 is an input, and NSS stays internal under SSM.
//!
//! Register recipe ported from the `spi_crc` bringup spike (case 9: CRCEN
//! held across feeds accumulates).

use core::cell::SyncUnsafeCell;

use ch32_metapac::SPI1;
use ch32_metapac::spi::vals::BaudRate as SpiBaud;
use osc_drivers::traits::bus;

use crate::hal::{afio, dma, rcc};

/// osc-CRC-16 polynomial (§3.2: CRC-16/ARC; the engine register takes the
/// non-reflected form, LSBFIRST supplies the reflection).
const OSC_CRC_POLY: u16 = 0x8005;

/// Bounded drain spin between successive feeds (the engine runs ~8× wire
/// speed, F6). An expiry leaves a partial accumulator → the frame fails CRC
/// and drops, which is the sanctioned "spin miss = fail" outcome (§3.2).
const FEED_DRAIN_SPIN: u32 = 4096;

/// Staging for the feed stream (§3.2, §5): every span is m2m-copied here
/// before the engine consumes it — one shape, no address discrimination.
/// Sized for the largest feed: a max frame's covered span (256 B).
const STAGING_LEN: usize = 256;

#[repr(align(2))]
struct Staging([u8; STAGING_LEN]);

static STAGING: SyncUnsafeCell<Staging> = SyncUnsafeCell::new(Staging([0; STAGING_LEN]));

/// Production binding to SPI1 + DMA1_CH3.
pub struct Crc;

impl Crc {
    /// One-shot peripheral bring-up (driver-pattern §5.5): clock-gate SPI1,
    /// lock master / SSM / 16-bit mode, load the poly, enable the calculator
    /// last (resets it with the mode fixed), then SPE + TX-DMA. Held live for
    /// the whole program — `reset` re-zeros the accumulator per frame.
    pub fn init() {
        rcc::enable_spi1();
        // Park the SPI functions off the motor pins (module doc). Verify
        // by behavior (CRC stays bit-exact): a debugger PCFR1 dump has
        // disagreed with behaviorally-proven remap state before.
        afio::set_spi_remap(1, 0b101);
        SPI1.ctlr1().write(|w| {
            w.set_mstr(true);
            w.set_ssm(true);
            w.set_ssi(true);
            w.set_br(SpiBaud::DIV_2);
            // 16-bit LSB-first: each little-endian halfword shifts low byte
            // first, low bit first — natural wire byte order, so the engine
            // computes CRC-16/ARC with no pair packing (§3.2, spike
            // spi_crc_lsb_copy). TCRCR holds the bit-reversed checksum;
            // `result` un-reverses it.
            w.set_dff(true);
            w.set_lsbfirst(true);
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
        dma::disable(dma::Channel::CH6);
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
        // Stream processing, one shape (§3.2): CH6 m2m-copies the span into
        // staging, CH3 feeds staging to the engine — no address
        // discrimination, no branch. CH6 runs VERYHIGH, strictly above CH3,
        // so the arbiter completes the copy before the feed's first grant;
        // CH1 (ADC) and CH5 (RX ring), VERYHIGH with lower channel numbers,
        // interleave through the copy and never starve. Zero CPU beyond
        // these register writes.
        debug_assert!(span.len() <= STAGING_LEN, "feed span exceeds staging");
        // SAFETY: single writer — feeds are serialized by the bus driver
        // (one CRC engine, one exchange at a time); the prior feed's chain
        // has drained (spin above), so overwriting staging is safe.
        let dst = unsafe { (*STAGING.get()).0.as_ptr() } as u32;
        dma::disable(dma::Channel::CH6);
        dma::configure_m2m(
            dma::Channel::CH6,
            span.as_ptr() as u32,
            dst,
            span.len() as u16,
            dma::Pl::VERYHIGH,
        );
        // Even length guaranteed by the caller (§3.2 fold contract); DMA
        // reads halfwords.
        Self::arm(dst, (span.len() / 2) as u16);
    }

    fn result(&mut self) -> Option<u16> {
        if Self::drained() {
            dma::disable(dma::Channel::CH3);
            // LSB-first shifter mirrors the reflected algorithm: TCRCR is the
            // bit-reversed osc-CRC-16 (§3.2). Un-reverse once per frame:
            // three parallel in-byte swap stages, then the byte swap.
            let mut x = SPI1.tcrcr().read().txcrc() as u32;
            x = ((x & 0x5555) << 1) | ((x >> 1) & 0x5555);
            x = ((x & 0x3333) << 2) | ((x >> 2) & 0x3333);
            x = ((x & 0x0f0f) << 4) | ((x >> 4) & 0x0f0f);
            Some((x as u16).swap_bytes())
        } else {
            None
        }
    }
}
