#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

//! STM32F301 board entry point.
//!
//! Implements the executor-based architecture:
//! - ADC DMA ISR owns the kernel (single-writer)
//! - Embassy executor runs RPC service in main context
//! - Two-phase init: configure_* then start_*

use panic_rtt_target as _;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, pre_init};
use embassy_executor::Executor;
use open_servo_device::executor::ControlExecutor;
use open_servo_hw_utils::rtt_async::{RttChannels, RttRpcIo};
use open_servo_kernel::ServoKernel;
use open_servo_services::RpcService;
use static_cell::StaticCell;

use open_servo_board_stm32f301::config::kernel_config;
use open_servo_board_stm32f301::init::{
    configure_adc, configure_gpio, configure_nvic, configure_rcc, configure_tim1, configure_tim2,
    configure_usart, enable_interrupts, start_adc_dma, start_tim1, start_tim2, start_usart,
    start_usart_rx_dma,
};
use open_servo_board_stm32f301::resources::{
    get_shadow_storage, init_queues, rpc_tick, set_isr_resources,
};

// Define defmt timestamp using TIM2 monotonic counter (1µs resolution)
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u32:us}", {
    use stm32f3::stm32f301::TIM2;
    // SAFETY: TIM2 is configured and running, CNT read is always safe
    unsafe { (*TIM2::ptr()).cnt.read().bits() }
});

/// System clock frequency.
const SYSCLK_HZ: u32 = 72_000_000;

/// RPC service polling rate.
const RPC_TICK_HZ: u32 = 1_000;

/// Enable debug during sleep modes before anything else runs.
/// This ensures SWD always works for recovery, even if firmware crashes.
#[pre_init]
unsafe fn pre_init() {
    const DBGMCU_CR: *mut u32 = 0xE004_2004 as *mut u32;
    DBGMCU_CR.write_volatile(0x7); // DBG_SLEEP | DBG_STOP | DBG_STANDBY
}

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
    // Configure SysTick for RPC service polling
    // =========================================================================
    let syst = &mut core.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(SYSCLK_HZ / RPC_TICK_HZ - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();

    // =========================================================================
    // Run Embassy executor with RPC service
    // =========================================================================

    // Create Embassy executor
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());

    // Run executor with tasks
    executor.run(|spawner| {
        spawner.must_spawn(rtt_tasks());
    });
}

/// RTT task - initializes RTT and runs RPC service.
#[embassy_executor::task]
async fn rtt_tasks() {
    let rtt = RttChannels::init(rpc_tick());
    run_rpc_service(rtt.rpc).await;
}

/// RPC service task.
async fn run_rpc_service(rpc_io: RttRpcIo) {
    // Buffer for RPC transport (COBS max for 128-byte msg is ~131 bytes)
    static RPC_BUF: StaticCell<[u8; 192]> = StaticCell::new();
    let buf = RPC_BUF.init([0u8; 192]);

    // Create RPC service with the IO and shadow storage
    let mut service = RpcService::new(rpc_io, buf, get_shadow_storage());

    // Run the RPC service loop
    service.run().await
}
