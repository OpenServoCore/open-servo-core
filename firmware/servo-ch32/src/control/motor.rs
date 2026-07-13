//! Chip-side motor service. Translates `osc_servo_core::MotorCmd` into TIM1
//! H-bridge writes per the DRV8212P truth table:
//! (1,1)=BRAKE, (0,0)=COAST, (1,0)=fwd, (0,1)=rev.
//! Slow decay holds the idle leg HIGH; Fast decay holds it LOW. Effort ->
//! duty math uses `>>15` to dodge soft-div in the ISR.

use osc_servo_core::{DecayMode, Motor as MotorTrait, MotorCmd};
use osc_servo_drivers::Level;

use crate::hal::{Pin, gpio, timer};

pub struct Ch32Motor {
    in1: timer::Channel,
    in2: timer::Channel,
    drv_en: Pin,
    drv_en_active: Level,
    drv_en_inactive: Level,
    pwm_arr: u16,
}

impl Ch32Motor {
    pub const fn new(
        in1: timer::Channel,
        in2: timer::Channel,
        drv_en: Pin,
        drv_en_active: Level,
        pwm_arr: u16,
    ) -> Self {
        let drv_en_inactive = match drv_en_active {
            Level::High => Level::Low,
            Level::Low => Level::High,
        };
        Self {
            in1,
            in2,
            drv_en,
            drv_en_active,
            drv_en_inactive,
            pwm_arr,
        }
    }
}

impl MotorTrait for Ch32Motor {
    fn write(&mut self, cmd: MotorCmd) {
        const STATIC_HIGH: u16 = u16::MAX;
        match cmd {
            MotorCmd::Disabled => {
                gpio::set_level(self.drv_en, self.drv_en_inactive);
                timer::set_duty(self.in1, 0);
                timer::set_duty(self.in2, 0);
            }
            MotorCmd::Coast => {
                gpio::set_level(self.drv_en, self.drv_en_active);
                timer::set_duty(self.in1, 0);
                timer::set_duty(self.in2, 0);
            }
            MotorCmd::Brake => {
                gpio::set_level(self.drv_en, self.drv_en_active);
                timer::set_duty(self.in1, STATIC_HIGH);
                timer::set_duty(self.in2, STATIC_HIGH);
            }
            MotorCmd::Drive { duty, decay } => {
                gpio::set_level(self.drv_en, self.drv_en_active);
                let ticks = effort_to_ticks(duty.0.unsigned_abs(), self.pwm_arr);
                if ticks == 0 {
                    // Slow decay at zero ticks would lock both legs HIGH = BRAKE; coast instead.
                    timer::set_duty(self.in1, 0);
                    timer::set_duty(self.in2, 0);
                    return;
                }
                let arr_minus = self.pwm_arr.saturating_sub(ticks);
                let (in1, in2) = match (decay, duty.0 >= 0) {
                    (DecayMode::Slow, true) => (STATIC_HIGH, arr_minus),
                    (DecayMode::Slow, false) => (arr_minus, STATIC_HIGH),
                    (DecayMode::Fast, true) => (ticks, 0),
                    (DecayMode::Fast, false) => (0, ticks),
                };
                timer::set_duty(self.in1, in1);
                timer::set_duty(self.in2, in2);
            }
        }
    }
}

/// `mag * arr / 32767`, approximated with `>>15` to dodge soft-div in the ISR.
fn effort_to_ticks(mag: u16, pwm_arr: u16) -> u16 {
    let m = mag.min(i16::MAX as u16) as u32;
    let prod = m * pwm_arr as u32;
    ((prod + (1 << 14)) >> 15) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn effort_zero_maps_to_zero_ticks() {
        assert_eq!(effort_to_ticks(0, 1200), 0);
    }

    #[test]
    fn effort_full_scale_maps_to_arr() {
        assert_eq!(effort_to_ticks(i16::MAX as u16, 1200), 1200);
    }

    #[test]
    fn effort_clamps_above_i16_max() {
        assert_eq!(effort_to_ticks(u16::MAX, 1200), 1200);
    }

    #[test]
    fn effort_to_ticks_monotonic() {
        let arr = 1200;
        let mut last = 0;
        for mag in (0..=i16::MAX as u16).step_by(317) {
            let t = effort_to_ticks(mag, arr);
            assert!(t >= last, "non-monotonic at mag={mag}: {last} -> {t}");
            last = t;
        }
    }
}
