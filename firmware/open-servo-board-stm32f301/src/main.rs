#![no_std]
#![no_main]

//! STM32F301 board entry point.
//!
//! Implements the executor-based architecture:
//! - ADC DMA ISR owns the kernel (single-writer)
//! - Main loop handles UART RX parsing and TX flushing
//! - Two-phase init: configure_* then start_*

use core::panic::PanicInfo;

use cortex_m::asm::wfi;
use cortex_m_rt::entry;
use open_servo_device::executor::Executor;
use open_servo_kernel::ServoKernel;
use stm32f3::stm32f301::DMA1;

use open_servo_board_stm32f301::config::kernel_config;
use open_servo_board_stm32f301::init::{
    configure_adc, configure_gpio, configure_nvic, configure_rcc, configure_tim1, configure_tim2,
    configure_usart, enable_interrupts, start_adc_dma, start_tim1, start_tim2, start_usart,
    start_usart_rx_dma,
};
use open_servo_board_stm32f301::resources::{
    init_queues, set_isr_resources, UART_RX_BUF_SIZE, UART_RX_DMA_BUF,
};

/// Panic handler.
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

/// Main entry point.
#[entry]
fn main() -> ! {
    // =========================================================================
    // Phase 1: Configure peripherals (no starts)
    // =========================================================================
    configure_rcc();
    configure_gpio();
    configure_tim1();
    configure_tim2();
    configure_adc();
    configure_usart();
    configure_nvic();

    // =========================================================================
    // Initialize resources
    // =========================================================================

    // Split queues (must happen before interrupts enabled)
    let (_op_prod, _result_cons) = unsafe { init_queues() };

    // Create kernel and executor
    let kernel = ServoKernel::new(kernel_config());
    let executor = Executor::new(kernel);

    // Set ISR resources
    unsafe { set_isr_resources(executor) };

    // =========================================================================
    // Phase 2: Start peripherals in controlled order
    // =========================================================================
    start_tim2(); // Monotonic first
    start_tim1(); // PWM (triggers ADC)
    start_adc_dma(); // ADC conversions
    start_usart_rx_dma(); // UART RX DMA
    start_usart(); // UART enable

    // Enable interrupts last
    enable_interrupts();

    // =========================================================================
    // Main loop
    // =========================================================================

    // UART RX read cursor
    let mut rx_read_idx: usize = 0;

    loop {
        // Consume RX bytes from DMA circular buffer
        // Stage-0: just drain to keep up with DMA, no parsing yet
        let write_idx = read_uart_dma_write_idx();

        while rx_read_idx != write_idx {
            // Stage-0: discard bytes (StubComms)
            let _byte = unsafe { UART_RX_DMA_BUF[rx_read_idx] };
            rx_read_idx = (rx_read_idx + 1) % UART_RX_BUF_SIZE;
        }

        // Stage-0: no TX flushing (StubComms produces nothing)

        // Sleep until interrupt
        wfi();
    }
}

/// Read UART DMA write index from NDTR.
///
/// Reads NDTR twice to ensure stable value (DMA may be mid-update).
#[inline]
fn read_uart_dma_write_idx() -> usize {
    let dma = unsafe { &*DMA1::ptr() };

    loop {
        let a = dma.ch5.ndtr.read().ndt().bits();
        let b = dma.ch5.ndtr.read().ndt().bits();
        if a == b {
            return UART_RX_BUF_SIZE - (a as usize);
        }
    }
}
