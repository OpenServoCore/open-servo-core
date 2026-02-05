//! Interrupt service routines for STM32F301.

use cortex_m::interrupt::free;
use open_servo_core::{Event, EventProducer, FaultKind};
use pac::interrupt;
use stm32f3::stm32f301 as pac;

use crate::config::{CONTROL_FAST_DT_US, CONTROL_MEDIUM_DECIMATE, CONTROL_MEDIUM_DT_US};
use crate::system::{SystemState, EVENT_QUEUE_SIZE};

/// TIM2 interrupt - System tick domain (housekeeping/telemetry, 100Hz).
/// This is System tick (housekeeping) domain; later we may split ControlSlow vs System.
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

/// Counter for ControlMedium decimation (ControlFast domain).
static mut CONTROL_MEDIUM_COUNTER: u8 = 0;

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
        app.on_control_fast_tick(board, CONTROL_FAST_DT_US);

        // ControlMedium tick - decimated from ControlFast to stay aligned to ADC samples.
        let fire_medium = unsafe {
            CONTROL_MEDIUM_COUNTER += 1;
            if CONTROL_MEDIUM_COUNTER >= CONTROL_MEDIUM_DECIMATE {
                CONTROL_MEDIUM_COUNTER = 0;
                true
            } else {
                false
            }
        };
        if fire_medium {
            app.on_control_medium_tick_dt(CONTROL_MEDIUM_DT_US);
        }
    });

    // Clear interrupt flag
    unsafe {
        (*pac::DMA1::ptr()).ifcr.write(|w| w.cgif1().set_bit());
    }
}
