use cortex_m::asm;
use stm32f3::stm32f301 as pac;

pub const RCC_SYSCLK_HZ: u32 = 72_000_000;

pub fn init_rcc(p: &pac::Peripherals) {
    let rcc = &p.RCC;
    let flash = &p.FLASH;

    // enable HSI clock
    rcc.cr.modify(|_, w| w.hsion().on());
    while rcc.cr.read().hsirdy().is_not_ready() {}

    // set HSI as system clock
    rcc.cfgr.modify(|_, w| w.sw().hsi());
    while !rcc.cfgr.read().sws().is_hsi() {}

    // enable HSE clock
    rcc.cr.modify(|_, w| w.hsebyp().not_bypassed().hseon().on());
    while rcc.cr.read().hserdy().is_not_ready() {}

    // configure PLL
    rcc.cfgr2.modify(|_, w| w.prediv().div2()); // HSE is 16 MHz, so divide by 2 to get 8 MHz
    rcc.cfgr.modify(|_, w| {
        w.pllsrc()
            .hse_div_prediv() // Use HSE as PLL source
            .pllmul()
            .mul9() // 8 MHz * 9 = 72 MHz
    });

    // enable PLL
    rcc.cr.modify(|_, w| w.pllon().on());
    while rcc.cr.read().pllrdy().is_not_ready() {}

    // set Flash latency to 2 wait states since we're running at 72 MHz
    flash.acr.write(|w| w.latency().ws2());

    // set AHB prescaler
    rcc.cfgr.modify(|_, w| {
        w.hpre()
            .div1() // AHB is 72 MHz
            .mcopre()
            .div1() // MCO is 72 MHz
            .ppre1()
            .div2() // APB1 is 36 MHz
            .ppre2()
            .div1() // APB2 is 72 MHz
    });

    // Wait for the new prescalers to kick in
    // "The clocks are divided with the new prescaler factor from
    //  1 to 16 AHB cycles after write"
    asm::delay(16);

    // switch to PLL
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}

    // disable HSI
    rcc.cr.modify(|_, w| w.hsion().off());
    while rcc.cr.read().hsirdy().is_ready() {}

    // enable FLITF clock
    // This provides read / write access to the Flash memory
    rcc.ahbenr.modify(|_, w| w.flitfen().enabled());

    // enable TIM1
    rcc.apb2enr.modify(|_, w| w.tim1en().enabled());
    rcc.apb2rstr.modify(|_, w| w.tim1rst().set_bit());
    rcc.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());

    // enable TIM2
    rcc.apb1enr.modify(|_, w| w.tim2en().enabled());
    rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());

    // enable GPIOA, GPIOB, GPIOC
    rcc.ahbenr
        .modify(|_, w| w.iopaen().enabled().iopben().enabled().iopcen().enabled());

    // enable ADC1
    rcc.ahbenr.modify(|_, w| w.adc1en().enabled());
    rcc.ahbrstr.modify(|_, w| w.adc1rst().set_bit());
    rcc.ahbrstr.modify(|_, w| w.adc1rst().clear_bit());

    // enable DMA1
    rcc.ahbenr.modify(|_, w| w.dma1en().enabled());

    // delay for a bit to let the clocks settle
    asm::delay(65535);
}
