//! GPIO pin configuration.
//!
//! Pin assignments:
//! - PA8: TIM1_CH1 (PWM forward) - AF6
//! - PA11: TIM1_CH4 (PWM reverse) - AF11
//! - PA0: ADC1_IN1 (position sensor) - Analog
//! - PA1: ADC1_IN2 (current sense, optional) - Analog
//! - PA2: ADC1_IN3 (voltage sense A, optional) - Analog
//! - PA3: ADC1_IN4 (voltage sense B, optional) - Analog
//! - PA4: ADC1_IN5 (motor temp, optional) - Analog
//! - PA9: USART1_TX - AF7
//! - PA10: USART1_RX - AF7

use stm32f3::stm32f301::GPIOA;

/// Configure GPIO pin modes.
///
/// Sets up all pins for their intended functions but doesn't start peripherals.
pub fn configure_gpio() {
    // SAFETY: We have exclusive access during init.
    let gpioa = unsafe { &*GPIOA::ptr() };

    // PA0-PA4: Analog mode for ADC
    // MODER = 0b11 for analog
    gpioa.moder.modify(|_, w| {
        w.moder0().analog();
        w.moder1().analog();
        w.moder2().analog();
        w.moder3().analog();
        w.moder4().analog()
    });

    // PA8: AF6 (TIM1_CH1), push-pull, high speed, pull-down
    gpioa.moder.modify(|_, w| w.moder8().alternate());
    gpioa.otyper.modify(|_, w| w.ot8().push_pull());
    gpioa.ospeedr.modify(|_, w| w.ospeedr8().high_speed());
    gpioa.pupdr.modify(|_, w| w.pupdr8().pull_down());
    gpioa.afrh.modify(|_, w| w.afrh8().af6());

    // PA11: AF11 (TIM1_CH4), push-pull, high speed, pull-down
    gpioa.moder.modify(|_, w| w.moder11().alternate());
    gpioa.otyper.modify(|_, w| w.ot11().push_pull());
    gpioa.ospeedr.modify(|_, w| w.ospeedr11().high_speed());
    gpioa.pupdr.modify(|_, w| w.pupdr11().pull_down());
    gpioa.afrh.modify(|_, w| w.afrh11().af11());

    // PA9: AF7 (USART1_TX), push-pull
    gpioa.moder.modify(|_, w| w.moder9().alternate());
    gpioa.otyper.modify(|_, w| w.ot9().push_pull());
    gpioa.ospeedr.modify(|_, w| w.ospeedr9().high_speed());
    gpioa.afrh.modify(|_, w| w.afrh9().af7());

    // PA10: AF7 (USART1_RX), input with pull-up (idle high when disconnected)
    gpioa.moder.modify(|_, w| w.moder10().alternate());
    gpioa.pupdr.modify(|_, w| w.pupdr10().pull_up());
    gpioa.afrh.modify(|_, w| w.afrh10().af7());
}
