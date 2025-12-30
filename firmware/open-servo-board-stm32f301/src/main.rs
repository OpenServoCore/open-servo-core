#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

//! STM32F301 board entry point.
//!
//! Implements the executor-based architecture:
//! - ADC DMA ISR owns the kernel (single-writer)
//! - Embassy executor runs debug shell in main context
//! - Two-phase init: configure_* then start_*

use panic_rtt_target as _;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::entry;
use embassy_executor::Executor;
use open_servo_device::executor::ControlExecutor;
use open_servo_hw_utils::rtt_async::RttAsyncIo;
use open_servo_kernel::ServoKernel;
use open_servo_services::run_debug_shell;
use static_cell::StaticCell;

use open_servo_board_stm32f301::config::kernel_config;
use open_servo_board_stm32f301::init::{
    configure_adc, configure_gpio, configure_nvic, configure_rcc, configure_tim1, configure_tim2,
    configure_usart, enable_interrupts, start_adc_dma, start_tim1, start_tim2, start_usart,
    start_usart_rx_dma,
};
use open_servo_board_stm32f301::resources::{
    debug_tick, get_shadow_storage, init_queues, set_isr_resources,
};

// Define defmt timestamp (required by defmt when enabled)
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u32:us}", { 0 }); // dummy timestamp

/// System clock frequency.
const SYSCLK_HZ: u32 = 72_000_000;

/// Debug shell polling rate.
const DEBUG_TICK_HZ: u32 = 1_000;

/// Main entry point.
#[entry]
fn main() -> ! {
    // Get core peripherals for SysTick setup
    let mut core = cortex_m::Peripherals::take().unwrap();

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
    let executor = ControlExecutor::new(kernel);

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
    // Configure SysTick for debug shell polling
    // =========================================================================
    let syst = &mut core.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(SYSCLK_HZ / DEBUG_TICK_HZ - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();

    // =========================================================================
    // Run Embassy executor with debug shell
    // =========================================================================

    // Create Embassy executor
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());

    // Run executor with debug shell task
    executor.run(|spawner| {
        spawner.must_spawn(debug_shell_task());
    });
}

/// Debug shell embassy task.
///
/// Initializes RTT and runs the debug shell. RTT is initialized here rather
/// than in main() because the task needs ownership of the RttAsyncIo.
#[embassy_executor::task]
async fn debug_shell_task() {
    // Initialize RTT with SysTick-driven polling signal
    let rtt = RttAsyncIo::init(debug_tick());
    run_debug_shell(rtt, get_shadow_storage()).await;
}
