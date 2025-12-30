//! Interrupt service routines.
//!
//! DMA1_CH1: ADC transfer complete - highest priority, runs control tick
//! USART1: IDLE interrupt - wakes main loop from WFI
//! SysTick: Debug shell polling signal

use cortex_m_rt::exception;
use open_servo_units::MicroSecond;
use stm32f3::stm32f301::interrupt;
use stm32f3::stm32f301::{DMA1, USART1};

use crate::config::FAST_DT_US;
use crate::monotonic::now_us;
use crate::pwm::apply_motor_command;
use crate::resources::{
    debug_tick, get_executor, get_fault_sink, get_isr_queues, get_shadow_storage, get_telem_sink,
};
use crate::sensors::read_sensor_frame;

/// DMA1 Channel 1 interrupt (ADC transfer complete).
///
/// This is the control fast tick entry point.
/// Runs at 10kHz, triggered by ADC DMA completion.
#[interrupt]
fn DMA1_CH1() {
    // Clear DMA transfer complete flag
    let dma = unsafe { &*DMA1::ptr() };

    // Check if this is transfer complete (not half-transfer or error)
    if dma.isr.read().tcif1().bit_is_set() {
        // Clear the flag
        dma.ifcr.write(|w| w.ctcif1().set_bit());

        // Read sensor frame from DMA buffer
        let frame = unsafe { read_sensor_frame() };

        // Get current time
        let now = now_us();

        // Get ISR resources
        let executor = unsafe { get_executor() };
        let (op_cons, result_prod) = unsafe { get_isr_queues() };
        let faults = unsafe { get_fault_sink() };
        let telem = unsafe { get_telem_sink() };
        let shadow = get_shadow_storage();

        // Run executor tick
        let cmd = executor.on_adc_complete(
            frame,
            MicroSecond::from_us(FAST_DT_US),
            now,
            op_cons,
            result_prod,
            faults,
            telem,
            shadow,
        );

        // Apply motor command
        apply_motor_command(cmd);
    }
}

/// USART1 interrupt (IDLE detection).
///
/// Purpose: Wake main loop from WFI when UART goes idle after receiving data.
/// No kernel calls, no queue ops - just wake up.
#[interrupt]
fn USART1_EXTI25() {
    let usart = unsafe { &*USART1::ptr() };

    // Check and clear IDLE flag
    if usart.isr.read().idle().bit_is_set() {
        // Clear IDLE flag by writing to ICR (not by reading RDR!)
        usart.icr.write(|w| w.idlecf().set_bit());
    }

    // Interrupt return itself wakes main from WFI
    // No further action needed
}

/// SysTick exception (debug shell polling).
///
/// Signals the debug shell to check for RTT input.
/// Runs at 1kHz, configured in main.rs.
#[exception]
fn SysTick() {
    debug_tick().signal(());
}
