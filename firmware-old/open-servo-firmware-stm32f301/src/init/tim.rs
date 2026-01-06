//! Timer configuration.
//!
//! TIM1: PWM generation + ADC trigger
//! - 10kHz electrical PWM, center-aligned mode (counter 0→ARR→0)
//! - CH1/CH4 for H-bridge drive
//! - Update event triggers ADC at valley (TRGO2)
//!
//! TIM2: Monotonic microsecond counter
//! - 1MHz tick (1µs resolution)
//! - 32-bit counter (~4295 second wrap)

use stm32f3::stm32f301::{TIM1, TIM2};

use crate::config::PWM_ARR;

/// Configure TIM1 for PWM and ADC triggering.
///
/// Register order: arpe → cms/dir → arr/psc → bdtr → mms2 → channels → egr → ccer
///
/// Does NOT start the timer (CEN not set until start_tim1).
pub fn configure_tim1() {
    // SAFETY: We have exclusive access during init.
    let tim1 = unsafe { &*TIM1::ptr() };

    // 1. Enable auto-reload preload FIRST (like old crate)
    tim1.cr1.modify(|_, w| w.arpe().enabled());

    // 2. Set center-aligned mode with direction
    tim1.cr1.modify(|_, w| w.cms().center_aligned1().dir().up());

    // 3. Set ARR and PSC
    // f_pwm = f_tim / (2 * (ARR + 1)) = 72MHz / 7200 = 10kHz electrical
    tim1.arr.write(|w| w.arr().bits(PWM_ARR));
    tim1.psc.write(|w| w.psc().bits(0)); // No prescaler

    // 4. Enable main output (required for advanced timers)
    tim1.bdtr.modify(|_, w| w.moe().enabled());

    // 5. Set TRGO2 = Update event (0b0010) for ADC trigger at valley (CNT=0)
    tim1.cr2.modify(|_, w| unsafe { w.mms2().bits(0b0010) });

    // === Channel 1: PWM output ===
    tim1.ccmr1_output()
        .modify(|_, w| w.oc1pe().enabled().oc1m().pwm_mode1());
    tim1.ccr1().write(|w| w.ccr().bits(0)); // Initial duty = 0

    // === Channel 4: PWM output ===
    tim1.ccmr2_output()
        .modify(|_, w| w.oc4pe().enabled().oc4m().pwm_mode1());
    tim1.ccr4().write(|w| w.ccr().bits(0)); // Initial duty = 0

    // 6. Generate update event to load preload registers (ARR, PSC, CCRx)
    tim1.egr.write(|w| w.ug().update());

    // 7. Enable CH1, CH4 outputs (CH3 is internal trigger only, no CC3E needed)
    tim1.ccer.modify(|_, w| w.cc1e().set_bit().cc4e().set_bit());
}

/// Start TIM1 (enable counter).
pub fn start_tim1() {
    let tim1 = unsafe { &*TIM1::ptr() };
    tim1.cr1.modify(|_, w| w.cen().enabled());
}

/// Configure TIM2 as monotonic microsecond counter.
///
/// Does NOT start the timer (CEN not set until start_tim2).
pub fn configure_tim2() {
    // SAFETY: We have exclusive access during init.
    let tim2 = unsafe { &*TIM2::ptr() };

    // Prescaler: 72MHz / 72 = 1MHz (1µs tick)
    tim2.psc.write(|w| w.psc().bits(71));

    // ARR = max (full 32-bit range, ~4295 seconds)
    // SAFETY: Any 32-bit value is valid for ARR
    tim2.arr.write(|w| unsafe { w.arr().bits(0xFFFF_FFFF) });

    // Generate update event to load prescaler
    tim2.egr.write(|w| w.ug().update());

    // No interrupts, just free-running
    tim2.dier.reset();
}

/// Start TIM2 (enable counter).
pub fn start_tim2() {
    let tim2 = unsafe { &*TIM2::ptr() };
    tim2.cr1.modify(|_, w| w.cen().enabled());
}
