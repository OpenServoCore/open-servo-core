#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

//! STM32F301 firmware entry point.
//!
//! Architecture:
//! - Single-writer kernel: only ADC DMA ISR calls kernel.tick() / apply_op()
//! - Two-phase init: configure_* then start_* in controlled order
//! - Free-running TIM2 monotonic (1µs resolution)
//! - Critical-section guarded SPSC queues (no atomics)

// Hardware-specific modules
mod adc_config;
mod calibration;
mod config;
mod flash;
mod init;
mod isr;
mod monotonic;
mod pwm;
mod resources;
mod sensors;
mod sinks;
mod time_driver;
mod uart_bus;

use panic_rtt_target as _;

#[cfg(feature = "osctl")]
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, pre_init};
#[cfg(feature = "osctl")]
use embassy_executor::Executor;
#[cfg(feature = "osctl")]
use open_servo_hw_utils::rtt_async::{RttChannels, RttRpcIo};
use open_servo_kernel::ServoKernel;
use open_servo_runtime::executor::ControlExecutor;
#[cfg(feature = "osctl")]
use open_servo_services::{RpcService, Services};
#[cfg(feature = "osctl")]
use static_cell::StaticCell;

#[cfg(feature = "osctl")]
use crate::flash::{EepromFlash, EEPROM_FLASH_SIZE};
#[cfg(feature = "osctl")]
use crate::resources::{Stm32Primitives, PRIMITIVES};

use crate::config::kernel_config;
use crate::init::{
    configure_adc, configure_gpio, configure_nvic, configure_rcc, configure_tim1, configure_tim2,
    configure_usart, enable_interrupts, start_adc_dma, start_tim1, start_tim2, start_usart,
    start_usart_rx_dma,
};
use crate::resources::{
    get_shadow_storage as get_shadow, init_queues, on_eeprom_write, set_isr_resources,
};
#[cfg(feature = "osctl")]
use crate::resources::{get_shadow_storage, rpc_tick, EmbassySignal, EmbassyTimer};

// Define defmt timestamp using TIM2 monotonic counter (1µs resolution)
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u32:us}", {
    use stm32f3::stm32f301::TIM2;
    // SAFETY: TIM2 is configured and running, CNT read is always safe
    unsafe { (*TIM2::ptr()).cnt.read().bits() }
});

/// System clock frequency.
#[cfg(feature = "osctl")]
const SYSCLK_HZ: u32 = 72_000_000;

/// RPC service polling rate.
#[cfg(feature = "osctl")]
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
    // Get core peripherals for SysTick setup (osctl only)
    #[cfg(feature = "osctl")]
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

    // Wire up persist callback (EEPROM writes auto-trigger persist)
    get_shadow().set_persist_callback(on_eeprom_write);

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
    // Configure SysTick for RPC service polling (osctl only)
    // =========================================================================
    #[cfg(feature = "osctl")]
    {
        let syst = &mut core.SYST;
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(SYSCLK_HZ / RPC_TICK_HZ - 1);
        syst.clear_current();
        syst.enable_interrupt();
        syst.enable_counter();
    }

    // =========================================================================
    // Run Embassy executor with services (osctl only)
    // =========================================================================
    #[cfg(feature = "osctl")]
    {
        // Create Embassy executor
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());

        // Run executor with tasks
        executor.run(|spawner| {
            spawner.must_spawn(rtt_tasks());
            spawner.must_spawn(services_task());
        });
    }

    // Without osctl, just idle (kernel runs in ISR)
    #[cfg(not(feature = "osctl"))]
    loop {
        cortex_m::asm::wfi();
    }
}

/// Services task - runs persist and other async services.
#[cfg(feature = "osctl")]
#[embassy_executor::task]
async fn services_task() {
    // Wait for RTT to be initialized (rtt_tasks runs first)
    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

    // Create flash driver
    let flash = unsafe { EepromFlash::new() };

    // Create services bundle
    let mut services: Services<'_, Stm32Primitives, EepromFlash, { crate::resources::SHADOW_TABLE_SIZE }> =
        Services::new(&PRIMITIVES, flash, 0..EEPROM_FLASH_SIZE as u32, get_shadow_storage());

    // Initialize (restore EEPROM from flash)
    if let Err(e) = services.init().await {
        #[cfg(feature = "defmt")]
        defmt::error!("Services init failed: {:?}", e);
    }

    // Main loop
    loop {
        if let Err(e) = services.run_once().await {
            #[cfg(feature = "defmt")]
            defmt::error!("Service error: {:?}", e);
        }
    }
}

/// RTT task - initializes RTT and runs RPC service.
#[cfg(feature = "osctl")]
#[embassy_executor::task]
async fn rtt_tasks() {
    let rtt = RttChannels::init(EmbassySignal(rpc_tick()));
    run_rpc_service(rtt.rpc).await;
}

/// RPC service task.
#[cfg(feature = "osctl")]
async fn run_rpc_service(rpc_io: RttRpcIo<EmbassySignal>) {
    // Buffer for RPC transport (COBS max for 128-byte msg is ~131 bytes)
    static RPC_BUF: StaticCell<[u8; 192]> = StaticCell::new();
    let buf = RPC_BUF.init([0u8; 192]);

    // Create RPC service with the IO, timer, and shadow storage
    let mut service = RpcService::new(rpc_io, buf, EmbassyTimer, get_shadow_storage());

    // Run the RPC service loop
    service.run().await
}
