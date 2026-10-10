//! TX-wire provider (protocol sec 4.2) -- half-duplex drive discipline, the
//! break, and one DMA1_CH4 arm at a time. Arm completion surfaces as the
//! USART1 TC ISR (shifter empty), never a CH4 TC -- so the final release's
//! wire handback and any deferred config can never garble in-flight bits.
//!
//! PC0 under HDSEL carries the bus. Drive discipline is purely PC0's CNF
//! (spike `pc0_drive`, break_framing): AF open-drain while listening
//! (released, the external bus pull-up holds mark) and AF push-pull for the
//! servo's own TX window, so both wire edges are driven at 3M instead of
//! riding the pull-up's RC rise. HDSEL and RE stay on throughout: V006 HDSEL
//! does not echo own TX [F9], and with RE on the USART hands the released
//! line back cleanly at transmission end.

use ch32_metapac::USART1;
use osc_servo_drivers::traits::bus;

use crate::cfg::chip;
use crate::hal::gpio::{self, PinMode};
use crate::hal::{dma, usart};
use crate::providers::break_wake::BreakWake;

/// Production binding to the wire claim/release + USART1 SBK/TCIE + DMA1_CH4.
pub struct TxWire;

impl TxWire {
    /// Claim the wire for the TX window: PC0 -> AF push-pull, both wire
    /// edges driven.
    fn claim_wire(&self) {
        gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_PUSH_PULL);
    }

    /// Hand the wire back: PC0 -> AF open-drain, the bus pull-up holds mark
    /// and HDSEL RX keeps hearing through the pin.
    fn release_wire(&self) {
        gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_OPEN_DRAIN);
    }
}

impl bus::TxWire for TxWire {
    fn start_frame(&mut self) {
        crate::log::trace!("tx.start");
        self.claim_wire();
        // send_break (the protocol sec 3 law shape -- a bracketed-M 0x00 character)
        // blocks until the break has committed through its stop bit. The
        // shifter is therefore EMPTY when send(arm0) runs: TCIE must
        // stay off until `send` has cleared the gap TC, or the gap TC
        // reads as arm-drained on ISR return, and release() tears
        // the reply down mid-byte-0 (bench signature: break, one garbled
        // byte, silence). TCIE is armed per-arm in `send`.
        BreakWake::muted(|| usart::send_break(USART1));
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
        // Clear the TC that latched while the shifter sat empty (post-break
        // or between arms) BEFORE the enable: TC sets only at a frame's
        // completion (RM sec 14.8.1), so from here it can only mean this arm
        // drained. Clearing after the enable erases a real TC whenever a
        // preemption outlasts the arm (a 2 B arm is 6.7 us at 3M) and the
        // TX engine wedges.
        usart::clear_tc(USART1);
        dma::enable(dma::Channel::CH4);
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
