//! TX-wire provider -- HDSEL drive discipline, the law break, one DMA1_CH2
//! arm at a time, and the rescue pulse. Arm completion surfaces as the
//! USART3 TC ISR (shifter empty), never a CH2 flag, so release can never
//! garble in-flight bits.
//!
//! Echo discipline: V305 HDSEL echoes own TX into RX (unlike the V006),
//! so `claim` gates RE off and `release` restores it -- the ring never
//! sees our bytes, no retire dance needed (with RE off nothing latches,
//! and the receiver resyncs cleanly on an idle wire at re-enable).

use ch32_metapac::USART3;
use osc_host::traits;

use crate::hal::{dma, usart};
use crate::providers::pins;

/// Production binding to PB10 drive + USART3 break/TCIE + DMA1_CH2.
pub struct TxWire;

impl traits::TxWire for TxWire {
    fn claim(&mut self) {
        usart::set_re(USART3, false);
        pins::bus_drive(true);
    }

    fn send_break(&mut self) {
        // Blocks through the stop bit (hal contract): the shifter is empty
        // when the following send() arms, and TCIE stays off across the
        // gap so the latched TC can't read as arm-drained.
        usart::send_break(USART3);
    }

    fn send(&mut self, span: &[u8]) {
        dma::disable(dma::Channel::CH2);
        dma::configure(
            dma::Channel::CH2,
            &dma::Config {
                dir: dma::Dir::FROMMEMORY,
                circ: false,
                minc: true,
                pl: dma::Pl::HIGH,
                size: dma::Size::BITS8,
            },
            usart::data_addr(USART3),
            span.as_ptr() as u32,
            span.len() as u16,
        );
        dma::enable(dma::Channel::CH2);
        // Byte 0 is in DR within a couple of AHB cycles of the enable.
        // Clear the TC that latched while the shifter sat empty, THEN arm
        // the real arm-drained interrupt: the shortest arm holds the
        // shifter >= 10 bit-times, orders past these two writes.
        usart::clear_tc(USART3);
        usart::set_tc_irq(USART3, true);
    }

    fn hold_low(&mut self) {
        pins::bus_hold_low();
    }

    fn release(&mut self) {
        pins::bus_release_from_hold();
        usart::set_tc_irq(USART3, false);
        dma::disable(dma::Channel::CH2);
        usart::set_re(USART3, true);
    }
}
