#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

mod hw_impl;
mod init;
mod board;

use board::Board;
use open_servo_core::{App, EventProducer, EventConsumer, Event};
use open_servo_control::PidController;

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use stm32f3::stm32f301 as pac;

use pac::interrupt;

// Global static storage for App, Hardware, and Event system
static APP: Mutex<RefCell<Option<App<PidController>>>> = Mutex::new(RefCell::new(None));
static BOARD: Mutex<RefCell<Option<Board>>> = Mutex::new(RefCell::new(None));
static EVENT_PRODUCER: Mutex<RefCell<Option<EventProducer>>> = Mutex::new(RefCell::new(None));
static EVENT_CONSUMER: Mutex<RefCell<Option<EventConsumer>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Initialize board with all peripherals
    let board = Board::init();
    free(|cs| BOARD.borrow(cs).replace(Some(board)));

    // Initialize event system
    // Safety: Only called once at startup before interrupts
    let event_queue = unsafe { open_servo_core::EventQueue::init() };
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

    // Enable interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM2);   // Slow tick timer
        pac::NVIC::unmask(pac::Interrupt::DMA1_CH1); // ADC DMA complete
    }

    defmt::info!("System initialized - entering main loop");

    // Main event loop
    loop {
        // Process all pending events
        free(|cs| {
            let mut consumer_ref = EVENT_CONSUMER.borrow(cs).borrow_mut();
            if let Some(consumer) = consumer_ref.as_mut() {
                while let Some(event) = consumer.dequeue() {
                    let mut app_ref = APP.borrow(cs).borrow_mut();
                    let mut board_ref = BOARD.borrow(cs).borrow_mut();
                    
                    if let (Some(app), Some(board)) = (app_ref.as_mut(), board_ref.as_mut()) {
                        app.handle_event(board, event);
                    }
                }
            }
        });
        
        // Wait for interrupt (low power until next event)
        cortex_m::asm::wfi();
    }
}

// TIM2 interrupt - Slow tick for telemetry/status (e.g. 100Hz)
#[interrupt]
fn TIM2() {
    // Enqueue SlowTick event
    free(|cs| {
        if let Some(producer) = EVENT_PRODUCER.borrow(cs).borrow_mut().as_mut() {
            if !producer.enqueue(Event::SlowTick) {
                defmt::warn!("Event queue full - dropped SlowTick");
            }
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
        defmt::warn!("DMA1_CH1: Transfer not complete");
        return;
    }

    if is_error {
        defmt::error!("DMA1_CH1: Transfer error");
        // Could enqueue a fault event here if desired
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
            // For V0, hardcode setpoint (will come from UART protocol in V1)
            app.set_setpoint(2048); // Center position
            
            // Run control loop
            app.on_control_tick(board);
        }
    });

    // Clear interrupt flag
    unsafe {
        (*pac::DMA1::ptr()).ifcr.write(|w| w.cgif1().set_bit());
    }
}