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

use cortex_m::asm;
use stm32f3::stm32f301::{FLASH, RCC};

/// Configure clock tree for 72MHz operation.
///
/// This function matches the old open-servo-stm32f301 crate's init sequence:
/// 1. Enable HSI and set as system clock (safe starting point)
/// 2. Enable HSE and configure PLL
/// 3. Switch to PLL
/// 4. Disable HSI
/// 5. Enable peripheral clocks with reset sequences
///
/// # Panics
/// Panics if HSE or PLL fail to start (hardware fault).
pub fn configure_rcc() {
    // SAFETY: We have exclusive access during init.
    let rcc = unsafe { &*RCC::ptr() };
    let flash = unsafe { &*FLASH::ptr() };

    // 1. Enable HSI clock FIRST (safe starting point)
    rcc.cr.modify(|_, w| w.hsion().on());
    while rcc.cr.read().hsirdy().is_not_ready() {}

    // 2. Set HSI as system clock
    rcc.cfgr.modify(|_, w| w.sw().hsi());
    while !rcc.cfgr.read().sws().is_hsi() {}

    // 3. Enable HSE clock (with bypass config)
    rcc.cr.modify(|_, w| w.hsebyp().not_bypassed().hseon().on());
    while rcc.cr.read().hserdy().is_not_ready() {}

    // 4. Configure PLL: HSE/2 * 9 = 72MHz
    rcc.cfgr2.modify(|_, w| w.prediv().div2());
    rcc.cfgr
        .modify(|_, w| w.pllsrc().hse_div_prediv().pllmul().mul9());

    // 5. Enable PLL
    rcc.cr.modify(|_, w| w.pllon().on());
    while rcc.cr.read().pllrdy().is_not_ready() {}

    // 6. Set flash latency for 72MHz (2 wait states)
    flash.acr.write(|w| w.latency().ws2());

    // 7. Set prescalers (includes mcopre like old crate)
    rcc.cfgr.modify(|_, w| {
        w.hpre()
            .div1()
            .mcopre()
            .div1()
            .ppre1()
            .div2()
            .ppre2()
            .div1()
    });

    // 8. Wait for prescalers to take effect
    asm::delay(16);

    // 9. Switch SYSCLK to PLL
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}

    // 10. Disable HSI (no longer needed)
    rcc.cr.modify(|_, w| w.hsion().off());
    while rcc.cr.read().hsirdy().is_ready() {}

    // 11. Enable FLITF clock (flash interface)
    rcc.ahbenr.modify(|_, w| w.flitfen().enabled());

    // 12. Enable TIM1 with reset sequence
    rcc.apb2enr.modify(|_, w| w.tim1en().enabled());
    rcc.apb2rstr.modify(|_, w| w.tim1rst().set_bit());
    rcc.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());

    // 13. Enable TIM2 with reset sequence
    rcc.apb1enr.modify(|_, w| w.tim2en().enabled());
    rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());

    // 14. Enable GPIOA, GPIOB, GPIOC
    rcc.ahbenr
        .modify(|_, w| w.iopaen().enabled().iopben().enabled().iopcen().enabled());

    // 15. Enable ADC1 with reset sequence
    rcc.ahbenr.modify(|_, w| w.adc1en().enabled());
    rcc.ahbrstr.modify(|_, w| w.adc1rst().set_bit());
    rcc.ahbrstr.modify(|_, w| w.adc1rst().clear_bit());

    // 16. Enable DMA1
    rcc.ahbenr.modify(|_, w| w.dma1en().enabled());

    // 17. Enable USART1, SYSCFG (new crate additions)
    rcc.apb2enr
        .modify(|_, w| w.usart1en().enabled().syscfgen().enabled());

    // 18. Final delay for clocks to settle
    asm::delay(65535);
}
