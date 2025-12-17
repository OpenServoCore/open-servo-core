#![no_main]
#![no_std]

use panic_halt as _;

mod adc_config;
mod adc_sample;
mod board;
mod hw_impl;
mod init;

#[cfg(feature = "debug-shell")]
mod debug_rtt;

use board::Board;
use heapless::spsc::Queue;
use open_servo_control::PidController;
use open_servo_core::{App, Event, EventConsumer, EventProducer, EventQueue};

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use stm32f3::stm32f301 as pac;

use pac::interrupt;

#[cfg(feature = "debug-shell")]
use debug_rtt::{init_rtt, RttDebugIo, RttLoggerIo};
#[cfg(feature = "debug-shell")]
use open_servo_core::DebugShell;
#[cfg(feature = "debug-shell")]
use open_servo_log::info;

// Event queue size for this board
const EVENT_QUEUE_SIZE: usize = 32;

// Global static storage for the event queue
static mut EVENT_QUEUE_STORAGE: Queue<Event, EVENT_QUEUE_SIZE> = Queue::new();

// Global static storage for App, Hardware, and Event system
static APP: Mutex<RefCell<Option<App<PidController>>>> = Mutex::new(RefCell::new(None));
static BOARD: Mutex<RefCell<Option<Board>>> = Mutex::new(RefCell::new(None));
static EVENT_PRODUCER: Mutex<RefCell<Option<EventProducer<EVENT_QUEUE_SIZE>>>> =
    Mutex::new(RefCell::new(None));
static EVENT_CONSUMER: Mutex<RefCell<Option<EventConsumer<EVENT_QUEUE_SIZE>>>> =
    Mutex::new(RefCell::new(None));

#[cfg(feature = "debug-shell")]
static DEBUG_SHELL: Mutex<RefCell<Option<DebugShell<RttDebugIo>>>> = Mutex::new(RefCell::new(None));

// Static storage for logger I/O (used by open-servo-log)
#[cfg(feature = "debug-shell")]
static mut LOGGER_IO: Option<RttLoggerIo> = None;

#[entry]
fn main() -> ! {
    // Initialize RTT channels (when debug-shell enabled)
    // Channel 0: Logger (global, with critical section)
    // Channel 1: REPL (owned by shell, lock-free)
    #[cfg(feature = "debug-shell")]
    let rtt_io = {
        let (logger_io, repl_io) = init_rtt();

        // Store logger io in static and initialize logger
        // Safety: Only called once at startup before interrupts
        unsafe {
            LOGGER_IO = Some(logger_io);
            open_servo_log::init(LOGGER_IO.as_mut().unwrap());
        }

        repl_io
    };

    // Initialize board with all peripherals
    let board = Board::init();
    free(|cs| BOARD.borrow(cs).replace(Some(board)));

    // Initialize event system
    // Safety: Only called once at startup before interrupts
    let event_queue = unsafe {
        let queue_ref = &mut *core::ptr::addr_of_mut!(EVENT_QUEUE_STORAGE);
        EventQueue::from_queue(queue_ref)
    };
    let (producer, consumer) = event_queue.split();

    // Store event producer for ISR access
    free(|cs| EVENT_PRODUCER.borrow(cs).replace(Some(producer)));

    // Store event consumer for main loop
    free(|cs| EVENT_CONSUMER.borrow(cs).replace(Some(consumer)));

    // Initialize control algorithm
    let controller = PidController::new(open_servo_control::PidConfig::default());

    // Initialize App (core control logic)
    let app = App::new(controller);
    free(|cs| APP.borrow(cs).replace(Some(app)));

    // Initialize debug shell (if enabled)
    #[cfg(feature = "debug-shell")]
    {
        let debug_shell = DebugShell::new(rtt_io);
        free(|cs| DEBUG_SHELL.borrow(cs).replace(Some(debug_shell)));
    }

    // Enable interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM2); // Slow tick timer
        pac::NVIC::unmask(pac::Interrupt::DMA1_CH1); // ADC DMA complete
    }

    #[cfg(feature = "debug-shell")]
    info!("System initialized");

    // Main event loop
    loop {
        // 1. Drain event queue
        loop {
            let event = free(|cs| {
                let mut consumer_ref = EVENT_CONSUMER.borrow(cs).borrow_mut();
                consumer_ref.as_mut().and_then(|c| c.dequeue())
            });

            let Some(event) = event else { break };

            // Process event (needs CS for shared state)
            free(|cs| {
                let mut app_ref = APP.borrow(cs).borrow_mut();
                let mut board_ref = BOARD.borrow(cs).borrow_mut();

                if let (Some(app), Some(board)) = (app_ref.as_mut(), board_ref.as_mut()) {
                    app.handle_event(board, event);
                }
            });
        }

        // 2. Pump debug shell (Tier 3)
        #[cfg(feature = "debug-shell")]
        free(|cs| {
            let mut shell_ref = DEBUG_SHELL.borrow(cs).borrow_mut();
            let mut app_ref = APP.borrow(cs).borrow_mut();
            let mut board_ref = BOARD.borrow(cs).borrow_mut();

            if let (Some(shell), Some(app), Some(board)) =
                (shell_ref.as_mut(), app_ref.as_mut(), board_ref.as_mut())
            {
                shell.poll(app, board);
            }
        });

        // 3. Sleep until next interrupt (low power mode)
        cortex_m::asm::wfi();
    }
}

// TIM2 interrupt - Slow tick for telemetry/status (e.g. 100Hz)
#[interrupt]
fn TIM2() {
    // Check for queue overflow and raise fault if detected
    if EventProducer::<EVENT_QUEUE_SIZE>::check_overflow() {
        free(|cs| {
            if let Some(producer) = EVENT_PRODUCER.borrow(cs).borrow_mut().as_mut() {
                let _ = producer.enqueue(Event::Fault(open_servo_core::FaultKind::QueuePressure));
            }
        });
    }

    // Enqueue SlowTick event
    free(|cs| {
        if let Some(producer) = EVENT_PRODUCER.borrow(cs).borrow_mut().as_mut() {
            // Silently drop if queue full - QueuePressure fault handles this
            let _ = producer.enqueue(Event::SlowTick);
        }
    });

    // Clear interrupt flag
    unsafe {
        (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear());
    }
}

// DMA1_CH1 interrupt - ADC DMA transfer complete (control loop trigger)
#[interrupt]
fn DMA1_CH1() {
    let dma1 = unsafe { &(*pac::DMA1::ptr()) };

    // Check transfer status
    let [is_complete, is_error] = free(|_| {
        [
            dma1.isr.read().tcif1().is_complete(),
            dma1.isr.read().teif1().is_error(),
        ]
    });

    if !is_complete {
        return;
    }

    if is_error {
        // Enqueue fault event for DMA transfer error
        free(|cs| {
            if let Some(producer) = EVENT_PRODUCER.borrow(cs).borrow_mut().as_mut() {
                let _ = producer.enqueue(Event::Fault(open_servo_core::FaultKind::EncoderFault));
            }
        });
        return;
    }

    // Execute hard real-time control tick
    // This runs deterministically without blocking
    free(|cs| {
        let mut app_ref = APP.borrow(cs).borrow_mut();
        let mut board_ref = BOARD.borrow(cs).borrow_mut();

        if let (Some(app), Some(board)) = (app_ref.as_mut(), board_ref.as_mut()) {
            app.on_control_tick(board);
        }
    });

    // Clear interrupt flag
    unsafe {
        (*pac::DMA1::ptr()).ifcr.write(|w| w.cgif1().set_bit());
    }
}
