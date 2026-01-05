//! NVIC (interrupt) configuration.
//!
//! Priority hierarchy:
//! - DMA1_CH1 (ADC complete): priority 0 (highest)
//! - USART1 (IDLE only): priority 2
//!
//! Lower number = higher priority.

use cortex_m::peripheral::NVIC;
use stm32f3::stm32f301::Interrupt;

/// Configure NVIC interrupt priorities.
///
/// Does NOT enable interrupts (masking happens in enable_interrupts).
pub fn configure_nvic() {
    // Set priorities before enabling interrupts
    // SAFETY: We have exclusive access during init and interrupts are disabled.
    unsafe {
        // DMA1_CH1 = highest priority (0)
        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(Interrupt::DMA1_CH1, 0);
        nvic.set_priority(Interrupt::USART1_EXTI25, 32); // Lower priority
    }
}

/// Enable interrupts in NVIC.
///
/// Call this after all resources are initialized.
pub fn enable_interrupts() {
    unsafe {
        NVIC::unmask(Interrupt::DMA1_CH1);
        NVIC::unmask(Interrupt::USART1_EXTI25);
    }
}
