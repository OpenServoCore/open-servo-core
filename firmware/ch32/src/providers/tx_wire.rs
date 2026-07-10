//! TX-wire provider (osc-native §4.2) — half-duplex drive discipline, the
//! break, and one DMA1_CH4 arm at a time. Arm completion surfaces as the
//! USART1 TC ISR (shifter empty), never a CH4 TC — so the final release's
//! CNF flip and any deferred config can never garble in-flight bits.
//!
//! Drive discipline is purely PC0's CNF (spike `pc0_drive`, break_framing):
//! AF open-drain while listening — released, the external bus pull-up
//! holds mark — and AF push-pull for the servo's own TX window, so both
//! wire edges are driven at 3M instead of riding the pull-up's RC rise.
//! HDSEL and RE stay on throughout: V006 HDSEL does not echo own TX [F9],
//! and with RE on the USART hands the released line back cleanly at
//! transmission end. The 74LVC2G241 direction buffer is bypassed on this
//! board (§2, F7/F8).

use ch32_metapac::USART1;
use osc_drivers::traits::bus;

use crate::cfg::chip;
use crate::hal::gpio::{self, PinMode};
use crate::hal::{dma, usart};

/// Production binding to PC0 drive + USART1 SBK/TCIE + DMA1_CH4.
pub struct TxWire;

impl bus::TxWire for TxWire {
    fn start_frame(&mut self) {
        crate::log::trace!("tx.start");
        // Claim the wire: PC0 → AF push-pull for the TX window.
        gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_PUSH_PULL);
        // send_break blocks until the break has committed through its stop
        // bit (a non-blocking SBK queues behind DR data — bench-observed).
        // The shifter is therefore EMPTY when send(arm0) runs: TCIE must
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
        // ≥ 10 bit-times, orders of magnitude past these two writes.
        usart::clear_tc(USART1);
        usart::set_tc_irq(USART1, true);
    }

    fn release(&mut self) {
        // Retire a still-latched RX error flag before handing the wire
        // back: the shifter is empty and our push-pull drive still holds
        // the line high, so no byte can be mid-reception — the one
        // provably safe instant for the clear's DATAR read (a DATAR read
        // during reception kills the in-flight byte, bench 2026-07-09). A
        // flag whose byte drained before any STATR read never completed
        // the SR-then-DR pair, and with no further RX drains coming it
        // storms the vector once the wire idles (bench signature: the
        // first ack-bearing exchange after a hot-loop leg dies). The
        // clear is CONDITIONAL because it is not free: a DATAR read
        // consumes the armed SR-half, so the next break's flag cannot
        // drain-self-clear and its FE re-fires until the first data byte
        // lands — +12 µs of reply lag at 0.5M (bench-measured). Flags
        // that already self-cleared must be left alone.
        let errs = usart::rx_errors(USART1);
        if errs.fe || errs.ore || errs.pe || errs.ne {
            usart::clear_rx_errors(USART1);
        }
        // Hand the wire back: AF open-drain releases the driver and the
        // bus pull-up holds mark; HDSEL RX keeps hearing through the pin.
        gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_OPEN_DRAIN);
        usart::set_tc_irq(USART1, false);
        usart::set_dma_tx(USART1, false);
        dma::disable(dma::Channel::CH4);
    }
}
