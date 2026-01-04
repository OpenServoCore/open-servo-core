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

#[cfg(feature = "osctl")]
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, pre_init};
#[cfg(feature = "osctl")]
use embassy_executor::Executor;
use open_servo_device::executor::ControlExecutor;
#[cfg(feature = "osctl")]
use open_servo_hw_utils::rtt_async::{RttChannels, RttRpcIo};
use open_servo_kernel::ServoKernel;
#[cfg(feature = "osctl")]
use open_servo_services::persist::PersistTask;
#[cfg(feature = "osctl")]
use open_servo_services::RpcService;
#[cfg(feature = "osctl")]
use static_cell::StaticCell;

#[cfg(feature = "osctl")]
use open_servo_board_stm32f301::flash::{EepromFlash, EEPROM_FLASH_SIZE};

use open_servo_board_stm32f301::config::kernel_config;
use open_servo_board_stm32f301::init::{
    configure_adc, configure_gpio, configure_nvic, configure_rcc, configure_tim1, configure_tim2,
    configure_usart, enable_interrupts, start_adc_dma, start_tim1, start_tim2, start_usart,
    start_usart_rx_dma,
};
use open_servo_board_stm32f301::resources::{
    get_shadow_storage as get_shadow, init_queues, on_eeprom_write, set_isr_resources,
};
#[cfg(feature = "osctl")]
use open_servo_board_stm32f301::resources::{get_shadow_storage, persist_signal, rpc_tick};

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
    // Initialize persist task (restore EEPROM from flash)
    // =========================================================================
    #[cfg(feature = "osctl")]
    {
        static PERSIST_TASK: StaticCell<PersistTask<EepromFlash>> = StaticCell::new();

        // Create flash driver and persist task
        let flash = unsafe { EepromFlash::new() };
        let persist_task = PERSIST_TASK.init(PersistTask::new(flash, 0..EEPROM_FLASH_SIZE as u32));

        // Initialize from flash (blocking on startup is ok)
        // Note: We use block_on here since we're before the executor starts
        // For now, skip async init and just log if there's an error
        #[cfg(feature = "defmt")]
        defmt::info!("Persist task created, will init in executor");

        // Store reference for the async task
        unsafe { PERSIST_TASK_REF = Some(persist_task as *mut _) };
    }

    // =========================================================================
    // Run Embassy executor with RPC service (osctl only)
    // =========================================================================
    #[cfg(feature = "osctl")]
    {
        // Create Embassy executor
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());

        // Run executor with tasks
        executor.run(|spawner| {
            spawner.must_spawn(rtt_tasks());
            spawner.must_spawn(persist_task_runner());
        });
    }

    // Without osctl, just idle (kernel runs in ISR)
    #[cfg(not(feature = "osctl"))]
    loop {
        cortex_m::asm::wfi();
    }
}

/// Static reference to persist task for async access.
#[cfg(feature = "osctl")]
static mut PERSIST_TASK_REF: Option<*mut PersistTask<EepromFlash>> = None;

/// Get persist task reference.
///
/// # Safety
/// Must only be called after init and from the persist task runner.
#[cfg(feature = "osctl")]
unsafe fn get_persist_task() -> &'static mut PersistTask<EepromFlash> {
    &mut *PERSIST_TASK_REF.unwrap()
}

/// Persist task runner - waits for signal and persists EEPROM.
#[cfg(feature = "osctl")]
#[embassy_executor::task]
async fn persist_task_runner() {
    let signal = persist_signal();
    let shadow = get_shadow_storage();

    // Initialize from flash
    let task = unsafe { get_persist_task() };
    match task.init(shadow).await {
        Ok(()) => {
            #[cfg(feature = "defmt")]
            defmt::info!("Persist init: {:?}", task.last_result());
        }
        Err(e) => {
            #[cfg(feature = "defmt")]
            defmt::error!("Persist init failed: {:?}", e);
        }
    }

    // Main loop: wait for signal, then persist
    loop {
        signal.wait().await;

        let result = task.persist(shadow).await;

        #[cfg(feature = "defmt")]
        defmt::info!("Persist result: {:?}", result);
    }
}

/// RTT task - initializes RTT and runs RPC service.
#[cfg(feature = "osctl")]
#[embassy_executor::task]
async fn rtt_tasks() {
    let rtt = RttChannels::init(rpc_tick());
    run_rpc_service(rtt.rpc).await;
}

/// RPC service task.
#[cfg(feature = "osctl")]
async fn run_rpc_service(rpc_io: RttRpcIo) {
    // Buffer for RPC transport (COBS max for 128-byte msg is ~131 bytes)
    static RPC_BUF: StaticCell<[u8; 192]> = StaticCell::new();
    let buf = RPC_BUF.init([0u8; 192]);

    // Create RPC service with the IO and shadow storage
    let mut service = RpcService::new(rpc_io, buf, get_shadow_storage());

    // Run the RPC service loop
    service.run().await
}
