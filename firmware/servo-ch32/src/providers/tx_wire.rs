//! TX-wire provider (protocol sec 4.2) -- half-duplex drive discipline, the
//! break, and one DMA1_CH4 arm at a time. Arm completion surfaces as the
//! USART1 TC ISR (shifter empty), never a CH4 TC -- so the final release's
//! wire handback and any deferred config can never garble in-flight bits.
//!
//! Two wire modes, selected by the board via the `half-duplex` feature;
//! `claim_wire`/`release_wire` are the only lines that differ:
//!
//! - **Direct** (rev C, and rev B with the buffer bypassed): PC0 under
//!   HDSEL carries the bus. Drive discipline is purely PC0's CNF (spike
//!   `pc0_drive`, break_framing): AF open-drain while listening -- released,
//!   the external bus pull-up holds mark -- and AF push-pull for the servo's
//!   own TX window, so both wire edges are driven at 3M instead of riding
//!   the pull-up's RC rise. HDSEL and RE stay on throughout: V006 HDSEL
//!   does not echo own TX [F9], and with RE on the USART hands the released
//!   line back cleanly at transmission end.
//! - **Buffered** (rev B as designed): TX drives only the 74LVC2G241's
//!   buffer input, so its pin idles AF push-pull and the buffer's tri-state
//!   is the drive discipline -- TX_EN high drives TX onto the wire and mutes
//!   the receive path (inverted enable, same signal), low releases the wire
//!   to the board pull-up. The mute reproduces both HDSEL observables: no
//!   own-TX echo, and an RX input held at mark (the RX pull-up) through the
//!   whole TX window.

use ch32_metapac::USART1;
use osc_servo_drivers::traits::bus;

use crate::hal::{dma, usart};

#[cfg(not(feature = "half-duplex"))]
use crate::cfg::BusWiring;
#[cfg(feature = "half-duplex")]
use crate::cfg::chip;
#[cfg(feature = "half-duplex")]
use crate::hal::gpio::{self, PinMode};
#[cfg(not(feature = "half-duplex"))]
use crate::hal::{Pin, gpio};
#[cfg(not(feature = "half-duplex"))]
use osc_servo_drivers::Level;

/// Production binding to the wire claim/release + USART1 SBK/TCIE + DMA1_CH4.
#[cfg(feature = "half-duplex")]
pub struct TxWire;

/// Production binding to the wire claim/release + USART1 SBK/TCIE + DMA1_CH4.
#[cfg(not(feature = "half-duplex"))]
pub struct TxWire {
    tx_en: Pin,
}

impl TxWire {
    #[cfg(not(feature = "half-duplex"))]
    pub fn new(bus: &BusWiring) -> Self {
        Self { tx_en: bus.tx_en }
    }

    /// Claim the wire for the TX window.
    fn claim_wire(&self) {
        #[cfg(feature = "half-duplex")]
        // PC0 -> AF push-pull: drive both wire edges ourselves.
        gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_PUSH_PULL);
        #[cfg(not(feature = "half-duplex"))]
        // TX_EN high: the buffer drives TX onto the wire and mutes RX.
        gpio::set_level(self.tx_en, Level::High);
    }

    /// Hand the wire back: released, the bus pull-up holds mark.
    fn release_wire(&self) {
        #[cfg(feature = "half-duplex")]
        // PC0 -> AF open-drain; HDSEL RX keeps hearing through the pin.
        gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_OPEN_DRAIN);
        #[cfg(not(feature = "half-duplex"))]
        // TX_EN low: the buffer releases the wire and resumes feeding it to RX.
        gpio::set_level(self.tx_en, Level::Low);
    }
}

impl bus::TxWire for TxWire {
    fn start_frame(&mut self) {
        crate::log::trace!("tx.start");
        self.claim_wire();
        // send_break (the protocol sec 3 law shape -- a bracketed-M 0x00 character)
        // blocks until the break has committed through its stop bit. The
        // shifter is therefore EMPTY when send(arm0) runs: TCIE must
        // stay off until the arm's first byte is in flight, or the gap TC
        // latches, reads as arm-drained on ISR return, and release() tears
        // the reply down mid-byte-0 (bench signature: break, one garbled
        // byte, silence). TCIE is armed per-arm in `send`.
        usart::send_break(USART1);
    }

    fn send(&mut self, span: &[u8]) {
        dma::disable(dma::Channel::CH4);
        dma::clear_tc_flag(dma::Channel::CH4);
        let cfg = dma::Config {
            dir: dma::Dir::FROMMEMORY,
            circ: false,
            pinc: false,
            minc: true,
            size: dma::Size::BITS8,
            htie: false,
            tcie: false,
            pl: dma::Pl::HIGH,
        };
        dma::configure(
            dma::Channel::CH4,
            &cfg,
            usart::data_addr(USART1),
            span.as_ptr() as u32,
            span.len() as u16,
        );
        usart::set_dma_tx(USART1, true);
        dma::enable(dma::Channel::CH4);
        // Byte 0 is in DR within a couple of AHB cycles of the enable.
        // Clear the TC that latched while the shifter sat empty (post-break
        // or between arms), THEN arm the real arm-drained interrupt. A real
        // drain can't race the clear: the shortest arm holds the shifter
        // >= 10 bit-times, orders of magnitude past these two writes.
        usart::clear_tc(USART1);
        usart::set_tc_irq(USART1, true);
    }

    fn release(&mut self) {
        // No flag hygiene on the way out (transport sec 7): FE/NE/ORE have no
        // interrupt enable, so whatever latched during our TX window is
        // inert -- no release-point SR-DR-SR retire is needed.
        self.release_wire();
        usart::set_tc_irq(USART1, false);
        usart::set_dma_tx(USART1, false);
        dma::disable(dma::Channel::CH4);
    }
}
