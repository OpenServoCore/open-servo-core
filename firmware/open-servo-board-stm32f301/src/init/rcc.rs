//! Clock tree configuration.
//!
//! Target: 72MHz SYSCLK from 16MHz HSE.
//! - HSE = 16MHz (external crystal)
//! - PREDIV = /2 → 8MHz to PLL
//! - PLLMUL = x9 → 72MHz PLL output
//! - SYSCLK = PLL = 72MHz
//! - HCLK = SYSCLK = 72MHz
//! - PCLK1 = HCLK/2 = 36MHz (APB1, max 36MHz)
//! - PCLK2 = HCLK = 72MHz (APB2)

use stm32f3::stm32f301::{FLASH, RCC};

/// Configure clock tree for 72MHz operation.
///
/// This function:
/// 1. Enables HSE and waits for it to stabilize
/// 2. Configures PLL (HSE/2 * 9 = 72MHz)
/// 3. Sets flash latency for 72MHz
/// 4. Switches SYSCLK to PLL
/// 5. Enables peripheral clocks
///
/// # Panics
/// Panics if HSE or PLL fail to start (hardware fault).
pub fn configure_rcc() {
    // SAFETY: We have exclusive access during init.
    let rcc = unsafe { &*RCC::ptr() };
    let flash = unsafe { &*FLASH::ptr() };

    // Enable HSE (external 16MHz crystal)
    rcc.cr.modify(|_, w| w.hseon().set_bit());

    // Wait for HSE ready
    while rcc.cr.read().hserdy().bit_is_clear() {}

    // Configure PLL: HSE/2 * 9 = 72MHz
    // PREDIV = /2
    rcc.cfgr2.modify(|_, w| unsafe { w.prediv().bits(0b0001) }); // /2

    // PLLSRC = HSE/PREDIV, PLLMUL = x9
    rcc.cfgr.modify(|_, w| {
        w.pllsrc().hse_div_prediv(); // PLL source = HSE/PREDIV
        unsafe { w.pllmul().bits(0b0111) } // x9
    });

    // Enable PLL
    rcc.cr.modify(|_, w| w.pllon().set_bit());

    // Wait for PLL ready
    while rcc.cr.read().pllrdy().bit_is_clear() {}

    // Set flash latency for 72MHz (2 wait states)
    flash.acr.modify(|_, w| unsafe { w.latency().bits(0b010) });

    // Configure prescalers before switching to PLL
    // HPRE = /1 (AHB = 72MHz)
    // PPRE1 = /2 (APB1 = 36MHz, max for APB1)
    // PPRE2 = /1 (APB2 = 72MHz)
    rcc.cfgr.modify(|_, w| {
        w.hpre().div1();
        w.ppre1().div2();
        w.ppre2().div1()
    });

    // Switch SYSCLK to PLL
    rcc.cfgr.modify(|_, w| w.sw().pll());

    // Wait for switch complete
    while !rcc.cfgr.read().sws().is_pll() {}

    // Enable peripheral clocks
    // AHB: DMA1
    rcc.ahbenr.modify(|_, w| w.dma1en().set_bit());

    // APB1: TIM2, USART2 (if used)
    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());

    // APB2: TIM1, USART1, ADC1, GPIOA, GPIOB, SYSCFG
    rcc.apb2enr.modify(|_, w| {
        w.tim1en().set_bit();
        w.usart1en().set_bit();
        w.syscfgen().set_bit()
    });

    // ADC clock enable (separate register on F3)
    rcc.ahbenr.modify(|_, w| {
        w.adc1en().set_bit();
        w.iopaen().set_bit();
        w.iopben().set_bit()
    });
}
