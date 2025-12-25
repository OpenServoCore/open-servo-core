//! PWM motor control logic for STM32F301 with DRV8231A driver.

use crate::init::tim::PWM_MAX_DUTY;
use open_servo_math::Duty;
use stm32f3::stm32f301 as pac;

/// Motor polarity configuration
/// Set to true if motor is wired backwards (swaps PWM channels)
const REVERSE_MOTOR_POLARITY: bool = false;

/// Threshold for switching between coast and brake mode
/// Use brake mode above 50% duty for better control
const BRAKE_MODE_THRESHOLD: u16 = PWM_MAX_DUTY / 2;

/// PWM motor controller for DRV8231A H-bridge driver
pub struct PwmController;

impl PwmController {
    /// Set motor PWM duty cycle and direction
    ///
    /// Implements hybrid PWM pattern:
    /// - Low duty (<50%): Use drive-coast for compliance
    /// - High duty (>50%): Use drive-brake for better control
    ///
    /// DRV8231A truth table:
    /// - IN1=1, IN2=0: Forward drive
    /// - IN1=0, IN2=1: Reverse drive  
    /// - IN1=1, IN2=1: Brake (low-side slow decay)
    /// - IN1=0, IN2=0: Coast (high-Z)
    pub fn set_pwm(duty: Duty) {
        let tim1 = unsafe { &(*pac::TIM1::ptr()) };

        // Scale normalized duty to hardware PWM range
        let scaled = duty.scale_to(PWM_MAX_DUTY);
        let scaled = scaled.clamp(-(PWM_MAX_DUTY as i32), PWM_MAX_DUTY as i32);

        // Apply polarity reversal if configured
        let scaled = if REVERSE_MOTOR_POLARITY {
            -scaled
        } else {
            scaled
        };

        if scaled > 0 {
            let drive_duty = scaled as u16;
            if drive_duty > BRAKE_MODE_THRESHOLD {
                // High duty: use drive-brake for better control
                tim1.ccr1().write(|w| w.ccr().bits(PWM_MAX_DUTY)); // IN1 always HIGH
                tim1.ccr4()
                    .write(|w| w.ccr().bits(PWM_MAX_DUTY - drive_duty)); // IN2 inverted PWM
            } else {
                // Low duty: use drive-coast for compliance
                tim1.ccr1().write(|w| w.ccr().bits(drive_duty)); // IN1 PWM
                tim1.ccr4().write(|w| w.ccr().bits(0)); // IN2 always LOW
            }
        } else if scaled < 0 {
            let drive_duty = (-scaled) as u16;
            if drive_duty > BRAKE_MODE_THRESHOLD {
                // High duty: use drive-brake for better control
                tim1.ccr1()
                    .write(|w| w.ccr().bits(PWM_MAX_DUTY - drive_duty)); // IN1 inverted PWM
                tim1.ccr4().write(|w| w.ccr().bits(PWM_MAX_DUTY)); // IN2 always HIGH
            } else {
                // Low duty: use drive-coast for compliance
                tim1.ccr1().write(|w| w.ccr().bits(0)); // IN1 always LOW
                tim1.ccr4().write(|w| w.ccr().bits(drive_duty)); // IN2 PWM
            }
        } else {
            // Stopped: coast mode (both LOW) for less resistance
            tim1.ccr1().write(|w| w.ccr().bits(0));
            tim1.ccr4().write(|w| w.ccr().bits(0));
        }
    }

    /// Set motor to coast (high impedance)
    pub fn coast() {
        let tim1 = unsafe { &(*pac::TIM1::ptr()) };
        // Coast mode: both outputs LOW
        tim1.ccr1().write(|w| w.ccr().bits(0));
        tim1.ccr4().write(|w| w.ccr().bits(0));
    }

    /// Set motor to brake (short motor terminals)
    pub fn brake() {
        let tim1 = unsafe { &(*pac::TIM1::ptr()) };
        // Brake mode: both outputs HIGH
        tim1.ccr1().write(|w| w.ccr().bits(PWM_MAX_DUTY));
        tim1.ccr4().write(|w| w.ccr().bits(PWM_MAX_DUTY));
    }
}
