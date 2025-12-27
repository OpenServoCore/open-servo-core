//! Timer configuration.
//!
//! TIM1: PWM generation + ADC trigger
//! - 20kHz PWM, center-aligned mode
//! - CH1/CH4 for H-bridge drive
//! - TRGO2 triggers ADC at PWM center
//!
//! TIM2: Monotonic microsecond counter
//! - 1MHz tick (1µs resolution)
//! - 32-bit counter (~4295 second wrap)

use stm32f3::stm32f301::{TIM1, TIM2};

use crate::config::PWM_ARR;

/// Configure TIM1 for PWM and ADC triggering.
///
/// Does NOT start the timer (CEN not set).
pub fn configure_tim1() {
    // SAFETY: We have exclusive access during init.
    let tim1 = unsafe { &*TIM1::ptr() };

    // Prescaler = 0 (72MHz timer clock)
    tim1.psc.write(|w| unsafe { w.psc().bits(0) });

    // ARR = 3599 for 20kHz PWM in center-aligned mode
    // f_pwm = f_tim / (2 * (ARR + 1)) for center-aligned
    // 20kHz = 72MHz / (2 * 3600) = 72MHz / 7200 = 10kHz... wait that's wrong
    // Actually for center-aligned: f_pwm = f_tim / ((ARR+1) * 2)
    // So ARR = f_tim / (f_pwm * 2) - 1 = 72M / 40k - 1 = 1799
    // But we want 20kHz update rate for ADC, so ARR = 3599 gives 10kHz PWM
    // Let's use ARR = 1799 for 20kHz PWM
    tim1.arr.write(|w| unsafe { w.arr().bits(PWM_ARR) });

    // Center-aligned mode 1 (counter counts up and down)
    tim1.cr1.modify(|_, w| unsafe { w.cms().bits(0b01) });

    // CH1: PWM mode 1, preload enabled
    tim1.ccmr1_output().modify(|_, w| {
        w.oc1pe().set_bit(); // Preload enable
        unsafe { w.oc1m().bits(0b0110) } // PWM mode 1
    });

    // CH4: PWM mode 1, preload enabled
    tim1.ccmr2_output().modify(|_, w| {
        w.oc4pe().set_bit(); // Preload enable
        unsafe { w.oc4m().bits(0b0110) } // PWM mode 1
    });

    // Enable CH1 and CH4 outputs
    tim1.ccer.modify(|_, w| {
        w.cc1e().set_bit();
        w.cc4e().set_bit()
    });

    // Configure TRGO2 = OC1REF (or use update event)
    // MMS2 bits in CR2 control TRGO2
    // 0b0100 = OC1REF used as TRGO2
    tim1.cr2.modify(|_, w| unsafe { w.mms2().bits(0b0100) });

    // Enable main output (required for advanced timers)
    tim1.bdtr.modify(|_, w| w.moe().set_bit());

    // Enable ARR preload
    tim1.cr1.modify(|_, w| w.arpe().set_bit());

    // Initial duty cycle = 0 (safe)
    tim1.ccr1().write(|w| unsafe { w.ccr().bits(0) });
    tim1.ccr4().write(|w| unsafe { w.ccr().bits(0) });
}

/// Start TIM1 (enable counter).
pub fn start_tim1() {
    let tim1 = unsafe { &*TIM1::ptr() };
    tim1.cr1.modify(|_, w| w.cen().set_bit());
}

/// Configure TIM2 as monotonic microsecond counter.
///
/// Does NOT start the timer (CEN not set).
pub fn configure_tim2() {
    // SAFETY: We have exclusive access during init.
    let tim2 = unsafe { &*TIM2::ptr() };

    // Prescaler: 72MHz / 72 = 1MHz (1µs tick)
    tim2.psc.write(|w| unsafe { w.psc().bits(71) });

    // ARR = max (full 32-bit range, ~4295 seconds)
    tim2.arr.write(|w| unsafe { w.arr().bits(0xFFFF_FFFF) });

    // No interrupts, just free-running
    tim2.dier.write(|w| unsafe { w.bits(0) });
}

/// Start TIM2 (enable counter).
pub fn start_tim2() {
    let tim2 = unsafe { &*TIM2::ptr() };
    tim2.cr1.modify(|_, w| w.cen().set_bit());
}
