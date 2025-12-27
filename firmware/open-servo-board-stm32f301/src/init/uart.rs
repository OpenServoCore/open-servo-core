//! USART1 + RX DMA configuration.
//!
//! USART1 is configured for:
//! - 1Mbps baud rate (or 115200 for debug)
//! - 8N1
//! - RX DMA circular mode
//! - IDLE interrupt for waking main loop

use stm32f3::stm32f301::{DMA1, USART1};

use crate::config::PCLK2_HZ;
use crate::resources::UART_RX_DMA_BUF;

/// UART baud rate.
pub const BAUD_RATE: u32 = 1_000_000; // 1Mbps for Dynamixel

/// Configure USART1 and RX DMA.
///
/// Does NOT enable USART (UE not set).
pub fn configure_usart() {
    // SAFETY: We have exclusive access during init.
    let usart = unsafe { &*USART1::ptr() };
    let dma = unsafe { &*DMA1::ptr() };

    // Disable USART during configuration
    usart.cr1.modify(|_, w| w.ue().clear_bit());

    // Configure baud rate
    // BRR = PCLK2 / BAUD_RATE
    let brr = PCLK2_HZ / BAUD_RATE;
    usart.brr.write(|w| unsafe { w.bits(brr) });

    // 8N1, oversampling by 16
    usart.cr1.modify(|_, w| {
        w.m().clear_bit(); // 8 data bits
        w.pce().clear_bit(); // No parity
        w.over8().clear_bit() // Oversampling by 16
    });
    usart.cr2.modify(|_, w| unsafe { w.stop().bits(0b00) }); // 1 stop bit

    // Enable RX and TX
    usart.cr1.modify(|_, w| {
        w.re().set_bit(); // Receiver enable
        w.te().set_bit() // Transmitter enable
    });

    // Enable IDLE interrupt (for waking main loop)
    usart.cr1.modify(|_, w| w.idleie().set_bit());

    // Enable DMA receive
    usart.cr3.modify(|_, w| w.dmar().set_bit());

    // Configure DMA1 Channel 5 for USART1_RX
    // Disable channel first
    dma.ch5.cr.modify(|_, w| w.en().clear_bit());

    // Peripheral address = USART1_RDR
    dma.ch5.par.write(|w| unsafe { w.pa().bits(usart.rdr.as_ptr() as u32) });

    // Memory address = UART_RX_DMA_BUF
    dma.ch5.mar.write(|w| unsafe { w.ma().bits(UART_RX_DMA_BUF.as_ptr() as u32) });

    // Number of data items = buffer size
    dma.ch5
        .ndtr
        .write(|w| unsafe { w.ndt().bits(UART_RX_DMA_BUF.len() as u16) });

    // DMA configuration:
    // - Circular mode
    // - Peripheral to memory
    // - Memory increment
    // - 8-bit transfers
    // - No interrupt (IDLE ISR handles wake)
    dma.ch5.cr.modify(|_, w| {
        w.circ().set_bit(); // Circular mode
        w.dir().clear_bit(); // Peripheral to memory
        w.minc().set_bit(); // Memory increment
        w.pinc().clear_bit(); // Peripheral fixed
        unsafe {
            w.msize().bits(0b00); // 8-bit memory
            w.psize().bits(0b00); // 8-bit peripheral
            w.pl().bits(0b01); // Medium priority
        }
        w.tcie().clear_bit() // No transfer complete interrupt
    });
}

/// Start USART RX DMA.
pub fn start_usart_rx_dma() {
    let dma = unsafe { &*DMA1::ptr() };
    dma.ch5.cr.modify(|_, w| w.en().set_bit());
}

/// Enable USART1.
pub fn start_usart() {
    let usart = unsafe { &*USART1::ptr() };
    usart.cr1.modify(|_, w| w.ue().set_bit());
}
