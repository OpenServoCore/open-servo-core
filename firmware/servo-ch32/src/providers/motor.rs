//! Chip-side motor service. Translates `osc_servo_core::MotorCmd` into TIM1
//! H-bridge writes per the DRV8212P truth table:
//! (1,1)=BRAKE, (0,0)=COAST, (1,0)=fwd, (0,1)=rev.
//! A drive holds the idle leg HIGH: slow decay, the off-window a brake.

use osc_servo_core::{Motor as MotorTrait, MotorCmd};
use osc_servo_drivers::Level;

use crate::hal::Pin;
use crate::hal::gpio::LevelWrite;
use crate::hal::timer::{self, DutyReg};
use crate::providers::sensors::scan::{self, Direction};

const STATIC_HIGH: u16 = u16::MAX;

pub struct Ch32Motor {
    in1: DutyReg,
    in2: DutyReg,
    drv_en_on: LevelWrite,
    drv_en_off: LevelWrite,
    pwm_arr: u16,
}

impl Ch32Motor {
    pub fn new(
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
            in1: DutyReg::new(in1),
            in2: DutyReg::new(in2),
            drv_en_on: LevelWrite::new(drv_en, drv_en_active),
            drv_en_off: LevelWrite::new(drv_en, drv_en_inactive),
            pwm_arr,
        }
    }
}

impl MotorTrait for Ch32Motor {
    /// TIM1 counts up: past the trough, so the compares load at the next
    /// crest, mid-pulse.
    #[inline(always)]
    fn late(&self) -> bool {
        !timer::counting_down()
    }

    fn write(&mut self, cmd: MotorCmd) {
        match cmd {
            MotorCmd::Disabled => {
                self.drv_en_off.apply();
                self.in1.write(0);
                self.in2.write(0);
            }
            // Slow decay at zero ticks would lock both legs HIGH = BRAKE; coast instead.
            MotorCmd::Coast | MotorCmd::Drive { ticks: 0, .. } => {
                self.drv_en_on.apply();
                self.in1.write(0);
                self.in2.write(0);
            }
            MotorCmd::Brake => {
                self.drv_en_on.apply();
                self.in1.write(STATIC_HIGH);
                self.in2.write(STATIC_HIGH);
            }
            MotorCmd::Drive { ticks, reverse } => {
                self.drv_en_on.apply();
                let (in1, in2) = legs(ticks, reverse, self.pwm_arr);
                self.in1.write(in1);
                self.in2.write(in2);
                scan::note_drive(if reverse {
                    Direction::Reverse
                } else {
                    Direction::Forward
                });
            }
        }
    }
}

/// `(in1, in2)` compares for a drive of `ticks`: the idle leg held HIGH and
/// the PWM leg's compare.
#[inline(always)]
fn legs(ticks: u16, reverse: bool, pwm_arr: u16) -> (u16, u16) {
    let pwm = pwm_arr.saturating_sub(ticks);
    if reverse {
        (pwm, STATIC_HIGH)
    } else {
        (STATIC_HIGH, pwm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legs_follow_the_truth_table() {
        let arr = 1200;
        let t = 300;
        assert_eq!(legs(t, false, arr), (STATIC_HIGH, arr - t));
        assert_eq!(legs(t, true, arr), (arr - t, STATIC_HIGH));
    }
}
