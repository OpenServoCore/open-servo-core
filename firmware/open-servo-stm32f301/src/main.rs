#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

mod hw_impl;
mod init;
mod board;
mod adc_sample;

use board::Board;
use open_servo_core::{App, EventProducer, EventConsumer, Event, EventQueue};
use open_servo_control::{PidController, PositionSensor, CurrentSensor, VoltageSensor, TemperatureSensor};
use open_servo_control::CentiDeg;
use heapless::spsc::Queue;

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use stm32f3::stm32f301 as pac;

use pac::interrupt;

// Event queue size for this board
const EVENT_QUEUE_SIZE: usize = 32;

// Global static storage for the event queue
static mut EVENT_QUEUE_STORAGE: Queue<Event, EVENT_QUEUE_SIZE> = Queue::new();

// Global static storage for App, Hardware, and Event system
static APP: Mutex<RefCell<Option<App<PidController>>>> = Mutex::new(RefCell::new(None));
static BOARD: Mutex<RefCell<Option<Board>>> = Mutex::new(RefCell::new(None));
static EVENT_PRODUCER: Mutex<RefCell<Option<EventProducer<EVENT_QUEUE_SIZE>>>> = Mutex::new(RefCell::new(None));
static EVENT_CONSUMER: Mutex<RefCell<Option<EventConsumer<EVENT_QUEUE_SIZE>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
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

    // Enable interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM2);   // Slow tick timer
        pac::NVIC::unmask(pac::Interrupt::DMA1_CH1); // ADC DMA complete
    }

    defmt::info!("System initialized - entering main loop");

    // Main event loop
    loop {
        // Pop one event at a time to minimize critical section duration
        let event = free(|cs| {
            let mut consumer_ref = EVENT_CONSUMER.borrow(cs).borrow_mut();
            consumer_ref.as_mut().and_then(|c| c.dequeue())
        });
        
        // Process event outside critical section (but still needs CS for shared state)
        if let Some(event) = event {
            free(|cs| {
                let mut app_ref = APP.borrow(cs).borrow_mut();
                let mut board_ref = BOARD.borrow(cs).borrow_mut();
                
                if let (Some(app), Some(board)) = (app_ref.as_mut(), board_ref.as_mut()) {
                    app.handle_event(board, event);
                }
            });
        } else {
            // No events pending - wait for interrupt (low power mode)
            cortex_m::asm::wfi();
        }
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
    
    // Log sensor data directly
    free(|cs| {
        let board_ref = BOARD.borrow(cs).borrow();
        if let Some(board) = board_ref.as_ref() {
            let position_raw = board.read_position_raw();
            let position = board.read_position();
            let current = board.read_current();
            let current_raw = board.read_current_raw();
            let voltage = board.read_voltage();
            let temperature = board.read_temperature();
            
            // Get position in degrees with decimal
            let deg = position.as_deg();
            let deg_frac = (position.as_cdeg() - deg * 100).abs() / 10;
            
            // Already in display units
            let current_ma = current.as_ma();
            let voltage_mv = voltage.as_mv();
            let temp_c = temperature.map(|t| t.as_celsius()).unwrap_or(0);
            
            defmt::info!(
                "Pos: {}.{}° (raw: {}) | I: {}mA (raw: {}) | V: {}mV | T: {}°C",
                deg,
                deg_frac,
                position_raw, 
                current_ma,
                current_raw,
                voltage_mv, 
                temp_c
            );
        }
    });
    
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
            // For V0, hardcode setpoint to 90 degrees
            app.set_setpoint(CentiDeg::from_deg(90));
            
            // Run control loop
            app.on_control_tick(board);
        }
    });

    // Clear interrupt flag
    unsafe {
        (*pac::DMA1::ptr()).ifcr.write(|w| w.cgif1().set_bit());
    }
}