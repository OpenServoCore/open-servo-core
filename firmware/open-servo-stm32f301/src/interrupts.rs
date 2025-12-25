//! Interrupt service routines for STM32F301.

use cortex_m::interrupt::free;
use open_servo_core::{Event, EventProducer, FaultKind};
use pac::interrupt;
use stm32f3::stm32f301 as pac;

use crate::system::SystemState;

/// Event queue size must match main.rs
const EVENT_QUEUE_SIZE: usize = 32;

/// TIM2 interrupt - Slow tick for telemetry/status (e.g. 100Hz)
#[interrupt]
fn TIM2() {
    // Check for queue overflow and raise fault if detected
    if EventProducer::<EVENT_QUEUE_SIZE>::check_overflow() {
        SystemState::with_event_producer(|producer| {
            let _ = producer.enqueue(Event::Fault(FaultKind::QueuePressure));
        });
    }

    // Enqueue SlowTick event
    SystemState::with_event_producer(|producer| {
        // Silently drop if queue full - QueuePressure fault handles this
        let _ = producer.enqueue(Event::SlowTick);
    });

    // Clear interrupt flag
    unsafe {
        (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear());
    }
}

/// DMA1_CH1 interrupt - ADC DMA transfer complete (control loop trigger)
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
        SystemState::with_event_producer(|producer| {
            let _ = producer.enqueue(Event::Fault(FaultKind::EncoderFault));
        });
        return;
    }

    // Execute hard real-time control tick
    // This runs deterministically without blocking
    SystemState::with_app_and_board(|app, board| {
        app.on_control_tick(board);
    });

    // Clear interrupt flag
    unsafe {
        (*pac::DMA1::ptr()).ifcr.write(|w| w.cgif1().set_bit());
    }
}
