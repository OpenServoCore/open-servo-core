//! Interrupt service routines.
//!
//! DMA1_CH1: ADC transfer complete - highest priority, runs control tick via Runtime
//! USART1: IDLE interrupt - wakes main loop from WFI
//! SysTick: RPC service polling signal

use cortex_m_rt::exception;
use open_servo_units::MicroSecond;
use stm32f3::stm32f301::interrupt;
use stm32f3::stm32f301::{DMA1, USART1};

use crate::config::FAST_DT_US;
use crate::resources::get_runtime;
use crate::sinks::{StubFaultSink, StubTelemetrySink};

#[cfg(feature = "osctl")]
use open_servo_runtime_embassy::rpc_tick;

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

        // Get runtime and run fast tick
        let runtime = get_runtime();

        // Sinks for fault/telemetry (stubs for now)
        static mut FAULT_SINK: StubFaultSink = StubFaultSink;
        static mut TELEM_SINK: StubTelemetrySink = StubTelemetrySink;

        unsafe {
            runtime.run_fast_tick(
                MicroSecond::from_us(FAST_DT_US),
                &mut *core::ptr::addr_of_mut!(FAULT_SINK),
                &mut *core::ptr::addr_of_mut!(TELEM_SINK),
            );
        }
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

/// SysTick exception (async task polling).
///
/// Signals RPC task to check for RTT input (osctl only).
/// Also checks embassy-time alarms for expired timers.
/// Runs at 1kHz, configured in main.rs.
#[exception]
fn SysTick() {
    #[cfg(feature = "osctl")]
    rpc_tick().signal(());
    crate::time_driver::check_alarms();
}
