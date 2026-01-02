//! USART1 + RX DMA configuration.
//!
//! USART1 is configured for:
//! - 1Mbps baud rate (Dynamixel protocol)
//! - 8N1
//! - RX DMA circular mode
//! - IDLE interrupt for waking main loop

use core::ptr::addr_of;

use stm32f3::stm32f301::{DMA1, USART1};

use crate::config::PCLK2_HZ;
use crate::resources::{UART_RX_BUF_SIZE, UART_RX_DMA_BUF};

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
    usart.cr1.modify(|_, w| w.ue().disabled());

    // Configure baud rate
    // BRR = PCLK2 / BAUD_RATE
    let brr = PCLK2_HZ / BAUD_RATE;
    usart.brr.write(|w| unsafe { w.bits(brr) });

    // 8N1, oversampling by 16
    usart
        .cr1
        .modify(|_, w| w.m().bit8().pce().disabled().over8().oversampling16());
    usart.cr2.modify(|_, w| w.stop().stop1());

    // Enable RX and TX
    usart.cr1.modify(|_, w| w.re().enabled().te().enabled());

    // Enable IDLE interrupt (for waking main loop)
    usart.cr1.modify(|_, w| w.idleie().enabled());

    // Enable DMA receive
    usart.cr3.modify(|_, w| w.dmar().enabled());

    // Configure DMA1 Channel 5 for USART1_RX
    // Disable channel first
    dma.ch5.cr.modify(|_, w| w.en().disabled());

    // Peripheral address = USART1_RDR
    dma.ch5
        .par
        .write(|w| unsafe { w.pa().bits(usart.rdr.as_ptr() as u32) });

    // Memory address = UART_RX_DMA_BUF
    // SAFETY: UART_RX_DMA_BUF is static and valid for 'static lifetime
    dma.ch5
        .mar
        .write(|w| unsafe { w.ma().bits(addr_of!(UART_RX_DMA_BUF) as u32) });

    // Number of data items = buffer size
    dma.ch5
        .ndtr
        .write(|w| w.ndt().bits(UART_RX_BUF_SIZE as u16));

    // DMA configuration:
    // - Circular mode
    // - Peripheral to memory
    // - Memory increment
    // - 8-bit transfers
    // - No interrupt (IDLE ISR handles wake)
    dma.ch5.cr.modify(|_, w| {
        w.circ()
            .enabled()
            .dir()
            .from_peripheral()
            .minc()
            .enabled()
            .pinc()
            .disabled()
            .msize()
            .bits8()
            .psize()
            .bits8()
            .pl()
            .medium()
            .tcie()
            .disabled()
    });
}

/// Start USART RX DMA.
pub fn start_usart_rx_dma() {
    let dma = unsafe { &*DMA1::ptr() };
    dma.ch5.cr.modify(|_, w| w.en().enabled());
}

/// Enable USART1.
pub fn start_usart() {
    let usart = unsafe { &*USART1::ptr() };
    usart.cr1.modify(|_, w| w.ue().enabled());
}
