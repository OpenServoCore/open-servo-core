//! TX-wire provider (osc-native §4.2) — half-duplex drive discipline, the
//! break, and one DMA1_CH4 arm at a time. Arm completion surfaces as the
//! USART1 TC ISR (shifter empty), never a CH4 TC — so the final release's
//! CNF flip and any deferred config can never garble in-flight bits.
//!
//! Drive discipline is purely PC0's CNF (spike `pc0_drive`): AF open-drain
//! while listening, AF push-pull for the DUT's own TX window. The
//! 74LVC2G241 direction buffer is bypassed on this board (§2, F7/F8).

use ch32_metapac::USART1;
use osc_drivers::traits::bus;

use crate::cfg::chip;
use crate::hal::gpio::{self, PinMode};
use crate::hal::{dma, usart};

/// Production binding to PC0 drive + USART1 SBK/TCIE + DMA1_CH4.
pub struct TxWire;

impl bus::TxWire for TxWire {
    fn start_frame(&mut self) {
        // Claim the wire: PC0 → AF push-pull for the TX window.
        gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_PUSH_PULL);
        // Clear any stale TC (STATR reset = 0xC0) before enabling TCIE so the
        // break can't raise a spurious TC → on_tx_complete before arm0 is
        // queued: the driver calls start_frame then send(arm0) synchronously,
        // and DMAT is enabled while the break still shifts, so TC stays held
        // off until the arm drains (§4.2, the #134-class TC-flag-gate lesson).
        usart::clear_tc(USART1);
        usart::set_tc_irq(USART1, true);
        // SBK self-times over the break's stop bit — set it and move on (§7).
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
    }

    fn release(&mut self) {
        // Release the wire: PC0 → AF open-drain, TX DMA + TCIE off.
        gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_OPEN_DRAIN);
        usart::set_tc_irq(USART1, false);
        usart::set_dma_tx(USART1, false);
        dma::disable(dma::Channel::CH4);
    }
}
