#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

//! STM32F301 firmware entry point.
//!
//! Architecture:
//! - Runtime owns Board + Kernel + Shadow + Queues
//! - Single-writer kernel: only ADC DMA ISR calls runtime.run_fast_tick()
//! - Two-phase init: configure_* then start_* in controlled order
//! - Free-running TIM2 monotonic (1µs resolution)

// Hardware-specific modules
mod adc_config;
mod board;
mod calibration;
mod config;
mod flash;
mod init;
mod isr;
mod resources;
mod sinks;
mod time_driver;

// Panic handler: use runtime-embassy for osctl, direct for non-osctl
#[cfg(feature = "osctl")]
use open_servo_runtime_embassy::panic_handler as _;
#[cfg(not(feature = "osctl"))]
use panic_rtt_target as _;

#[cfg(feature = "osctl")]
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, pre_init};
use open_servo_hw::config::BoardConfig;
use open_servo_hw::v2::Board;
use open_servo_kernel::{KernelConfig, PidGains, ServoKernel};
#[cfg(feature = "osctl")]
use open_servo_runtime_embassy::{run, set_shadow};
use open_servo_runtime_embassy::{save_signal, EmbassySignal};

use crate::board::Stm32f301Board;
use crate::init::{
    configure_adc, configure_gpio, configure_nvic, configure_rcc, configure_tim1, configure_tim2,
    configure_usart, enable_interrupts, start_adc_dma, start_tim1, start_tim2, start_usart,
    start_usart_rx_dma,
};
use crate::resources::init_runtime;
#[cfg(feature = "osctl")]
use crate::resources::{get_embassy_runtime, init_embassy_runtime};

#[cfg(feature = "osctl")]
use crate::config::SYSCLK_HZ;

// Define defmt timestamp using TIM2 monotonic counter (1µs resolution)
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u32:us}", {
    use stm32f3::stm32f301::TIM2;
    // SAFETY: TIM2 is configured and running, CNT read is always safe
    unsafe { (*TIM2::ptr()).cnt.read().bits() }
});

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

/// Build KernelConfig from Board traits.
fn kernel_config_from_board<B: Board + BoardConfig>(board: &B) -> KernelConfig {
    let (kp, ki, kd) = board.pid_gains();
    KernelConfig {
        pos_pid: PidGains {
            kp: kp as i16,
            ki: ki as i16,
            kd: kd as i16,
        },
        hold_deadband_cdeg: 50,
        effort_limit_raw: 16000,
        servo_pos_kind: board.servo_pos_kind(),
        motor_type: board.motor_type(),
        sensor_caps: board.sensor_capabilities(),
    }
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
    // Create Runtime with Board and Kernel
    // =========================================================================

    // Create board (owns flash, provides sensor/motor access)
    let board = unsafe { Stm32f301Board::new() };

    // Build kernel config from board traits
    let config = kernel_config_from_board(&board);

    // Create kernel
    let kernel = ServoKernel::new(config);

    // Create and initialize runtime with save signal for persist notifications
    let runtime = unsafe { init_runtime(board, kernel, EmbassySignal(save_signal())) };

    // Initialize runtime (splits queues, applies board defaults to shadow)
    unsafe { runtime.init() };

    // Wire up shadow reference for global access
    #[cfg(feature = "osctl")]
    set_shadow(runtime.shadow());

    // Initialize embassy runtime wrapper (owns service lifecycle)
    #[cfg(feature = "osctl")]
    let _embassy_rt = unsafe { init_embassy_runtime(runtime) };

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
        run(|spawner| {
            open_servo_runtime_embassy::spawn_all_services!(
                Stm32f301Board,
                get_embassy_runtime(),
                spawner
            );
        })
    }

    // Without osctl, just idle (kernel runs in ISR)
    #[cfg(not(feature = "osctl"))]
    loop {
        cortex_m::asm::wfi();
    }
}
