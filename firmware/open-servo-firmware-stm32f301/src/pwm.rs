//! PWM output control.
//!
//! Maps MotorCommand to TIM1 CH1/CH4 duty cycles for H-bridge drive.

use open_servo_hw::v2::io::{DriveMode, MotorCommand};
use stm32f3::stm32f301::TIM1;

use crate::config::PWM_ARR;

/// Motor polarity reversal. Set true if motor runs backwards.
const REVERSE_MOTOR: bool = false;

/// Apply motor command to PWM outputs.
///
/// Maps effort to duty cycle on CH1 (forward) or CH4 (reverse).
/// Coast mode sets both to 0. Brake mode sets both high (not implemented yet).
#[inline]
pub fn apply_motor_command(cmd: MotorCommand) {
    // SAFETY: Single-writer from ADC ISR.
    let tim1 = unsafe { &*TIM1::ptr() };

    if !cmd.driver_en {
        // Driver disabled: coast
        tim1.ccr1().write(|w| w.ccr().bits(0));
        tim1.ccr4().write(|w| w.ccr().bits(0));
        return;
    }

    match cmd.mode {
        DriveMode::Coast => {
            tim1.ccr1().write(|w| w.ccr().bits(0));
            tim1.ccr4().write(|w| w.ccr().bits(0));
        }
        DriveMode::Drive => {
            // Map effort to duty cycle
            // Effort is i16 (-32768 to 32767)
            // Positive = forward (CH1), negative = reverse (CH4)
            let mut effort = cmd.effort.as_raw();
            if REVERSE_MOTOR {
                effort = effort.saturating_neg();
            }

            if effort >= 0 {
                // Forward drive
                let duty = effort_to_duty(effort);
                tim1.ccr1().write(|w| w.ccr().bits(duty));
                tim1.ccr4().write(|w| w.ccr().bits(0));
            } else {
                // Reverse drive
                let duty = effort_to_duty(effort.saturating_neg());
                tim1.ccr1().write(|w| w.ccr().bits(0));
                tim1.ccr4().write(|w| w.ccr().bits(duty));
            }
        }
        DriveMode::Brake => {
            // Active brake: both high (slow decay)
            // Stage-0: treat as coast for safety
            tim1.ccr1().write(|w| w.ccr().bits(0));
            tim1.ccr4().write(|w| w.ccr().bits(0));
        }
    }
}

/// Convert effort (0..32767) to duty cycle (0..PWM_ARR).
#[inline]
fn effort_to_duty(effort: i16) -> u16 {
    // Linear mapping: effort / 32767 * PWM_ARR
    // Use u32 intermediate to avoid overflow
    let duty = (effort as u32 * PWM_ARR as u32) / 32767;
    duty as u16
}
