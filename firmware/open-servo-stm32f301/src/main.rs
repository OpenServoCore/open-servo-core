#![no_main]
#![no_std]

use panic_rtt_target as _;

mod adc_config;
mod adc_sample;
mod board;
mod calibration;
mod config;
mod init;
mod interrupts;
mod pwm;
mod sensors;
mod system;

// Define defmt timestamp (required by defmt)
#[cfg(feature = "defmt")]
defmt::timestamp!("{=u32:us}", {
    // Return a dummy timestamp - you could implement a real timer here if needed
    0
});

use board::Board;
use open_servo_core::{App, EventQueue};
use system::{SystemState, EVENT_QUEUE_SIZE};

use cortex_m_rt::entry;
use stm32f3::stm32f301 as pac;

#[cfg(feature = "debug-shell")]
use open_servo_hw_utils::rtt_debug::init_rtt;
#[cfg(feature = "debug-shell")]
use open_servo_core::DebugShell;

use heapless::spsc::Queue;

// Global static storage for the event queue
static mut EVENT_QUEUE_STORAGE: Queue<open_servo_core::Event, EVENT_QUEUE_SIZE> = Queue::new();


#[entry]
fn main() -> ! {
    // Initialize RTT channels (when debug-shell enabled)
    // Channel 0: defmt logging (handled by defmt-rtt)
    // Channel 1: REPL (owned by shell, lock-free)
    #[cfg(feature = "debug-shell")]
    let rtt_io = init_rtt();

    // Initialize board with all peripherals
    let board = Board::init();
    
    // Initialize App with PID controller from board config
    let app = App::new_with_pid(&board);
    
    // Store app and board in system state
    SystemState::init_app_and_board(app, board);

    // Initialize event system
    // Safety: Only called once at startup before interrupts
    let event_queue = unsafe {
        let queue_ref = &mut *core::ptr::addr_of_mut!(EVENT_QUEUE_STORAGE);
        EventQueue::from_queue(queue_ref)
    };
    let (producer, consumer) = event_queue.split();
    SystemState::init_event_system(producer, consumer);

    // Initialize debug shell (if enabled)
    #[cfg(feature = "debug-shell")]
    SystemState::init_debug_shell(DebugShell::new(rtt_io));

    // Enable interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM2); // Slow tick timer
        pac::NVIC::unmask(pac::Interrupt::DMA1_CH1); // ADC DMA complete
    }

    #[cfg(feature = "debug-shell")]
    #[cfg(feature = "defmt")]
    defmt::info!("System initialized");

    // Main event loop
    loop {
        // 1. Process all pending events
        SystemState::process_events();

        // 2. Pump debug shell (Tier 3)
        #[cfg(feature = "debug-shell")]
        SystemState::poll_debug_shell();

        // 3. Sleep until next interrupt (low power mode)
        cortex_m::asm::wfi();
    }
}
