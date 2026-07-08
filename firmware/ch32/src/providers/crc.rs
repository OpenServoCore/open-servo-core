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

/// The snapshot buffer (§3.2, §4.2): arbitrary-parity spans are CH6-streamed
/// here, and BOTH the CRC feed (CH3) and the wire arms (CH4) consume the
/// copy — the reply CRC covers exactly the transmitted bytes, and reads
/// carry a best-effort point-in-time image. Sized for the largest covered
/// span (max frame, 256 B). Even base: the feed reads halfwords from it.
const SNAPSHOT_LEN: usize = 256;

#[repr(align(2))]
struct Snapshot([u8; SNAPSHOT_LEN]);

static SNAPSHOT: SyncUnsafeCell<Snapshot> = SyncUnsafeCell::new(Snapshot([0; SNAPSHOT_LEN]));

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
        // Sources are even-based by construction (frame buffer head or the
        // snapshot buffer, trait contract); even length per the fold contract.
        Self::arm(span.as_ptr() as u32, (span.len() / 2) as u16);
    }

    fn snapshot(&mut self, off: u16, src: &[u8]) -> *const u8 {
        // Best-effort streaming copy, fire-and-forget: CH6 runs VERYHIGH,
        // strictly above CH3 (feed) and CH4 (TX), so no consumer can overtake
        // the copy — a late copy stalls the wire into legal inter-byte gaps
        // at worst. CH1 (ADC) and CH5 (RX ring), VERYHIGH with lower channel
        // numbers, interleave through it and never starve. No spin, no lock:
        // a kernel tick landing mid-copy tears the image by a µs-window, but
        // wire and CRC both read the copy, so the reply stays CRC-consistent
        // (§4.2 best-effort snapshot).
        debug_assert!(off as usize + src.len() <= SNAPSHOT_LEN);
        // The offset picks the channel: a ring-wrap linearization issues two
        // back-to-back copies (off 0, then the split point), and re-arming
        // one channel would truncate the first copy mid-flight. Disjoint
        // destinations, both VERYHIGH, CH6 drains before CH7 by number.
        let ch = if off == 0 {
            dma::Channel::CH6
        } else {
            dma::Channel::CH7
        };
        // SAFETY: single writer per exchange — snapshot calls are serialized
        // by the bus driver, and consumers are ordered behind the copy
        // channels by the DMA priority ladder.
        let dst = unsafe { (*SNAPSHOT.get()).0.as_ptr().add(off as usize) };
        dma::disable(ch);
        dma::configure_m2m(
            ch,
            src.as_ptr() as u32,
            dst as u32,
            src.len() as u16,
            dma::Pl::VERYHIGH,
        );
        dst
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
