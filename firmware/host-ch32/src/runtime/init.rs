//! Bringup: hardware powered + configured -> engine installed -> IRQs hot
//! (driver-pattern sec 9.4 contract). The adapter boots its wire at the
//! fleet's 1M default; the client moves both ends when it wants more.

use ch32_metapac::USART3;
use osc_protocol::wire::BaudRate;

use crate::hal::{dma, pfic, systick, tim2cap, usart, usbhs};
use crate::providers::clocks::Clocks;
use crate::providers::edges::Edges;
use crate::providers::pins::Pins;
use crate::providers::ring::RxRing;
use crate::providers::usart_baud::brr_for;
use crate::runtime::{Drivers, isr};

pub const BOOT_RATE: BaudRate = BaudRate::B1000000;

/// False = the crystal never came ready. The caller owns the failure
/// policy (LED-only: a dead crystal is a dead adapter, visibly -- no USB
/// attempt on an unreferenced PHY).
pub fn bringup() -> bool {
    // First, always: the loader's jump is not a reset; anything it left
    // pended before this line was retired by the trap backstop.
    pfic::scrub_loader_state();

    let clocks_ok = Clocks::init();
    Pins::init();
    systick::init();
    if !clocks_ok {
        return false;
    }

    usart::init_host(USART3, brr_for(BOOT_RATE));
    dma::configure(
        dma::Channel::CH3,
        &dma::Config {
            dir: dma::Dir::FROMPERIPHERAL,
            circ: true,
            minc: true,
            // RX outranks the TX arm so an inbound byte's drain is never
            // deferred behind a TX burst (the arbiter preempts per-beat).
            pl: dma::Pl::VERYHIGH,
            size: dma::Size::BITS8,
        },
        usart::data_addr(USART3),
        RxRing::base_addr(),
        RxRing::LEN as u16,
    );
    dma::enable(dma::Channel::CH3);

    // The instrument's edge stopwatch: TIM2 IC on the bus pin, both
    // polarities into circular rings, armed once here and never re-armed
    // (the spike's capture-mangle lesson).
    for (ch, paddr, maddr) in [
        (
            dma::Channel::CH1,
            tim2cap::fall_capture_addr(),
            Edges::falls_addr(),
        ),
        (
            dma::Channel::CH7,
            tim2cap::rise_capture_addr(),
            Edges::rises_addr(),
        ),
    ] {
        dma::configure(
            ch,
            &dma::Config {
                dir: dma::Dir::FROMPERIPHERAL,
                circ: true,
                minc: true,
                // Below the RX ring, above nothing that matters: capture
                // beats are sparse (>= 2 bit-times apart per polarity).
                pl: dma::Pl::MEDIUM,
                size: dma::Size::BITS16,
            },
            paddr,
            maddr,
            Edges::LEN as u16,
        );
        dma::enable(ch);
    }
    tim2cap::init();

    usbhs::init_device();

    // SAFETY: bringup-only, pre-IRQ; sole writer.
    unsafe { Drivers::install(BOOT_RATE) };
    isr::install_irqs();

    // Attach last: by the time the host enumerates, the whole stack is up.
    usbhs::attach();
    true
}
