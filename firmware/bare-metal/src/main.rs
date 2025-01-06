#![no_main]
#![no_std]

use panic_probe as _;
use defmt_rtt as _;

use stm32f3::stm32f301 as pac;
use stm32f3::stm32f301::interrupt;
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    init_rcc(&p);
    init_tim1_pwm(&p);
    init_tim2_counter(&p);
    init_led(&p);

    defmt::info!("Starting PWM");

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM1_CC);
        pac::NVIC::unmask(pac::Interrupt::TIM2);
    }

    loop {
    }
}

// interrrupt handler for OC3REF
#[interrupt]
fn TIM1_CC() {
    // defmt::info!("TIM1_CC interrupt");

    // toggle PA2
    unsafe {
        (*pac::GPIOA::ptr()).odr.modify(|r, w| w.odr2().bit(!r.odr2().bit()));
    }

    // clear interrupt flag
    unsafe {
        (*pac::TIM1::ptr()).sr.modify(|_, w| w.cc2if().clear());
    }
}

// interrupt handler for TIM2
#[interrupt]
fn TIM2() {
    defmt::info!("TIM2 interrupt");

    // toggle PA2
    // unsafe {
    //     (*pac::GPIOA::ptr()).odr.modify(|r, w| w.odr2().bit(!r.odr2().bit()));
    // }

    // clear interrupt flag
    unsafe {
        (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear());
    }
}

fn init_led(p: &pac::Peripherals) {
    let gpioa = &p.GPIOA;

    // configure PA2 as output
    gpioa.moder.modify(|_, w| w.moder2().output());

    // configure PA2 as push-pull
    gpioa.otyper.modify(|_, w| w.ot2().push_pull());

    // configure PA2 as high speed output
    gpioa.ospeedr.modify(|_, w| w.ospeedr2().high_speed());

    // configure PA2 as no pull-up, no pull-down
    gpioa.pupdr.modify(|_, w| w.pupdr2().floating());

    // set PA2 to low
    gpioa.odr.modify(|_, w| w.odr2().low());
}

fn init_tim2_counter(p: &pac::Peripherals) {
    let tim2 = &p.TIM2;

    // set prescaler to 72, so that the counter will increment every 1 us
    tim2.psc.write(|w| w.psc().bits(72));

    // set auto-reload register to 1_000_000, so that the counter will overflow every 1 second
    tim2.arr.write(|w| unsafe { w.arr().bits(1_000_000) });

    // generate an update event to load the prescaler value to the counter
    tim2.egr.write(|w| w.ug().update());

    // enable update interrupt
    tim2.dier.write(|w| w.uie().enabled());

    // enable TIM2
    tim2.cr1.write(|w| w.cen().enabled());
}

fn init_tim1_pwm(p: &pac::Peripherals) {
    let tim1 = &p.TIM1;

    // Enable auto-reload preload
    tim1.cr1.modify(|_, w| w.arpe().enabled());

    // set pwm mode to center aligned count up
    tim1.cr1.modify(|_, w| w.cms().center_aligned1().dir().up());

    // set PWM frequency to 10 kHz center aligned ( 20 KHz PWM frequency)
    tim1.arr.write(|w| w.arr().bits(3599)); // 72 MHz / 3600 = 20 kHz
    tim1.psc.write(|w| w.psc().bits(0)); // 72 MHz / 1 = 72 MHz

    // generate an update event to load the prescaler value to the counter
    tim1.egr.write(|w| w.ug().update());

    // enable main output enable
    tim1.bdtr.modify(|_, w| w.moe().enabled());

    // set trigger output to output compare 2 (OC2REF)
    // this is used to trigger ADC conversions
    tim1.cr2.modify(|_, w| unsafe {
        // See STM32F301 Reference Manual section 17.4.2.
        // MMS2 register is used to trigger ADC conversions
        w.mms2().bits(0b0101) // OC2REF
    });

    // enable TIM1
    tim1.cr1.modify(|_, w| w.cen().enabled());

    init_tim1_pwm_channels(p);
}

fn init_tim1_pwm_channels(p: &pac::Peripherals) {
    let gpioa = &p.GPIOA;
    let tim1 = &p.TIM1;

    // configure PA9 ( channel 2 ) as PWM output
    {
        // configure PA9 as alternate function
        gpioa.moder.modify(|_, w| w.moder9().alternate());

        // set PA9 alternate function map to TIM1_CH2 ( AF6 )
        gpioa.afrh.modify(|_, w| w.afrh9().af6());

        // configure PA9 as high speed output
        gpioa.ospeedr.modify(|_, w| w.ospeedr9().high_speed());

        // configure PA9 as push-pull
        gpioa.otyper.modify(|_, w| w.ot9().push_pull());

        // configure PA9 as pull down
        gpioa.pupdr.modify(|_, w| w.pupdr9().pull_down());

        // configure TIM1 as PWM mode 1 and enable preload register
        tim1.ccmr1_output().modify(|_, w| {
            w.oc2pe().enabled() // enable preload register for Channel 2
             .oc2m().pwm_mode1() // PWM mode 1 for Channel 2
        });

        // set PWM duty cycle to 50%
        tim1.ccr2().modify(|_, w| w.ccr().bits(1800));

        // enable PWM output
        tim1.ccer.modify(|_, w| w.cc2e().set_bit());
    }

    //  // configure PA8 as PWM output
    // {
    //     // configure PA8 as alternate function
    //     gpioa.afrh.modify(|_, w| w.afrh8().af2());

    //     // configure PA8 as high speed output
    //     gpioa.ospeedr.modify(|_, w| w.ospeedr8().high_speed());

    //     // configure PA8 as push-pull
    //     gpioa.otyper.modify(|_, w| w.ot8().push_pull());

    //     // configure PA8 as alternate function
    //     gpioa.moder.modify(|_, w| w.moder8().alternate());

    //     // set PWM duty cycle to 0
    //     tim1.ccr1().write(|w| w.ccr().bits(1800));

    //     // enable PWM output
    //     tim1.ccer.write(|w| w.cc1e().set_bit());

    //     // enable counter
    //     tim1.cr1.write(|w| w.cen().set_bit());
    // }

    // configure PA11 as PWM output
    // {
    //     // configure PA11 as alternate function
    //     gpioa.afrh.modify(|_, w| w.afrh11().af2());

    //     // configure PA11 as high speed output
    //     gpioa.ospeedr.modify(|_, w| w.ospeedr11().high_speed());

    //     // configure PA11 as push-pull
    //     gpioa.otyper.modify(|_, w| w.ot11().push_pull());

    //     // configure PA11 as alternate function
    //     gpioa.moder.modify(|_, w| w.moder11().alternate());

    //     // set PWM duty cycle to 0
    //     tim1.ccr2().write(|w| w.ccr().bits(0));

    //     // enable PWM output
    //     tim1.ccer.write(|w| w.cc2e().set_bit());

    //     // enable counter
    //     tim1.cr1.write(|w| w.cen().set_bit());
    // }

    // configure pwm channel 3 without output
    // {
    //     // set PWM duty cycle to 1 to trigger ADC conversion
    //     tim1.ccr3().write(|w| w.ccr().bits(1));

    //     // enable PWM output
    //     tim1.ccer.write(|w| w.cc3e().set_bit());

    //     // enable counter
    //     tim1.cr1.write(|w| w.cen().set_bit());
    // }
}

fn init_rcc(p: &pac::Peripherals) {
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
        w.pllsrc().hse_div_prediv() // Use HSE as PLL source
         .pllmul().mul9() // 8 MHz * 9 = 72 MHz
    });

    // enable PLL
    rcc.cr.modify(|_, w| w.pllon().on());
    while rcc.cr.read().pllrdy().is_not_ready() {}

    // set Flash latency to 2 wait states since we're running at 72 MHz
    flash.acr.write(|w| w.latency().ws2());

    // set AHB prescaler
    rcc.cfgr.modify(|_, w| {
        w.hpre().div1() // AHB is 72 MHz
            .mcopre().div1() // MCO is 72 MHz
            .ppre1().div2() // APB1 is 36 MHz
            .ppre2().div1() // APB2 is 72 MHz
    });

    // Wait for the new prescalers to kick in
    // "The clocks are divided with the new prescaler factor from
    //  1 to 16 AHB cycles after write"
    cortex_m::asm::delay(16);

    // switch to PLL
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}

    // disable HSI
    rcc.cr.modify(|_, w| w.hsion().off());
    while rcc.cr.read().hsirdy().is_ready() {}

    // reset and enable Power Control
    rcc.apb1enr.modify(|_, w| w.pwren().enabled());
    rcc.apb1rstr.modify(|_, w| w.pwrrst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.pwrrst().clear_bit());

    // reset and enable System Configuration Controller
    rcc.apb2enr.modify(|_, w| w.syscfgen().enabled());
    rcc.apb2rstr.modify(|_, w| w.syscfgrst().set_bit());
    rcc.apb2rstr.modify(|_, w| w.syscfgrst().clear_bit());

    // enable FLITF clock
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
    rcc.ahbenr.modify(|_, w| {
        w.iopaen().enabled()
         .iopben().enabled()
         .iopcen().enabled()
    });

    // enable ADC1
    rcc.ahbenr.modify(|_, w| w.adc1en().enabled());
    rcc.ahbrstr.modify(|_, w| w.adc1rst().set_bit());
    rcc.ahbrstr.modify(|_, w| w.adc1rst().clear_bit());

    // enable DMA1
    rcc.ahbenr.modify(|_, w| w.dma1en().enabled());

    // delay for a bit to let the clocks settle
    cortex_m::asm::delay(65535);
}
