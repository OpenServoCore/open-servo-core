use super::rcc::RCC_SYSCLK_HZ;
use stm32f3::stm32f301 as pac;

// PWM parameters
const PWM_FREQUENCY: u32 = 20_000;
pub const PWM_MAX_DUTY: u16 = ((RCC_SYSCLK_HZ / PWM_FREQUENCY) - 1) as u16;

pub fn init_tim1_pwm(p: &pac::Peripherals) {
    let tim1 = &p.TIM1;

    // Enable auto-reload preload
    tim1.cr1.modify(|_, w| w.arpe().enabled());

    // set pwm mode to center aligned count up
    tim1.cr1.modify(|_, w| w.cms().center_aligned1().dir().up());

    // set PWM frequency to 10 kHz center aligned ( 20 KHz PWM frequency)
    tim1.arr.write(|w| w.arr().bits(PWM_MAX_DUTY)); // 72 MHz / 3600 = 20 kHz
    tim1.psc.write(|w| w.psc().bits(0)); // 72 MHz / 1 = 72 MHz

    // generate an update event to load the prescaler value to the counter
    tim1.egr.write(|w| w.ug().update());

    // enable main output enable
    tim1.bdtr.modify(|_, w| w.moe().enabled());

    // set trigger output to output compare 2 (OC3REF)
    // this is used to trigger ADC conversions
    tim1.cr2.modify(|_, w| unsafe {
        // See STM32F301 Reference Manual section 17.4.2.
        // MMS2 register is used to trigger ADC conversions
        w.mms2().bits(0b0110) // OC3REF
    });

    // enable TIM1
    tim1.cr1.modify(|_, w| w.cen().enabled());

    init_tim1_pwm_channels(p);
}

fn init_tim1_pwm_channels(p: &pac::Peripherals) {
    let gpioa = &p.GPIOA;
    let tim1 = &p.TIM1;

    // configure PA8 ( channel 1 ) as PWM output
    {
        // configure PA8 as alternate function
        gpioa.moder.modify(|_, w| w.moder8().alternate());

        // set PA8 alternate function map to TIM1_CH1 ( AF6 )
        gpioa.afrh.modify(|_, w| w.afrh8().af6());

        // configure PA8 as high speed output
        gpioa.ospeedr.modify(|_, w| w.ospeedr8().high_speed());

        // configure PA8 as push-pull
        gpioa.otyper.modify(|_, w| w.ot8().push_pull());

        // configure PA8 as pull down
        gpioa.pupdr.modify(|_, w| w.pupdr8().pull_down());

        // configure Channel 1 as PWM mode 1 and enable preload register
        tim1.ccmr1_output().modify(|_, w| {
            w.oc1pe()
                .enabled() // enable preload register for Channel 1
                .oc1m()
                .pwm_mode1() // PWM mode 1 for Channel 1
        });

        // set PWM duty cycle to 0
        tim1.ccr1().write(|w| w.ccr().bits(0));

        // enable PWM output
        tim1.ccer.write(|w| w.cc1e().set_bit());
    }

    // configure TIM channel 3 to PWM output
    {
        // configure TIM1 as output compare and enable preload register
        tim1.ccmr2_output().modify(|_, w| {
            w.oc3pe()
                .enabled() // enable preload register for Channel 3
                .oc3m()
                .pwm_mode1() // PWM mode 1 for Channel 3
        });

        // Set CCR3 to center of PWM period for ADC trigger
        // This ensures ADC samples IPROPI at a stable point, avoiding switching transients
        tim1.ccr3().modify(|_, w| w.ccr().bits(PWM_MAX_DUTY / 2));

        // enable PWM output
        tim1.ccer.modify(|_, w| w.cc3e().set_bit());
    }

    // configure PA11 ( channel 4 ) as PWM output
    {
        // configure PA11 as alternate function
        gpioa.moder.modify(|_, w| w.moder11().alternate());

        // set PA11 alternate function map to TIM1_CH4 ( AF11 )
        gpioa.afrh.modify(|_, w| w.afrh11().af11());

        // configure PA11 as high speed output
        gpioa.ospeedr.modify(|_, w| w.ospeedr11().high_speed());

        // configure PA11 as push-pull
        gpioa.otyper.modify(|_, w| w.ot11().push_pull());

        // configure PA11 as pull down
        gpioa.pupdr.modify(|_, w| w.pupdr11().pull_down());

        // configure TIM1 as PWM mode 1 and enable preload register
        tim1.ccmr2_output().modify(|_, w| {
            w.oc4pe()
                .enabled() // enable preload register for Channel 4
                .oc4m()
                .pwm_mode1() // PWM mode 1 for Channel 4
        });

        // set PWM duty cycle to 0
        tim1.ccr4().modify(|_, w| w.ccr().bits(0));

        // enable PWM output
        tim1.ccer.modify(|_, w| w.cc4e().set_bit());
    }
}

pub fn init_tim2_counter(p: &pac::Peripherals) {
    let tim2 = &p.TIM2;

    // TIM2 timer clock is 72MHz (APB1 div2 => timer x2).
    // PSC=71 => counter tick is 1us (72MHz / 72 = 1MHz).
    tim2.psc.write(|w| w.psc().bits(71));

    // ARR=9999 => overflow period is 10ms (100Hz).
    tim2.arr.write(|w| unsafe { w.arr().bits(9_999) });

    // generate an update event to load the prescaler value to the counter
    tim2.egr.write(|w| w.ug().update());

    // enable update interrupt
    tim2.dier.write(|w| w.uie().enabled());

    // enable TIM2
    tim2.cr1.write(|w| w.cen().enabled());
}
