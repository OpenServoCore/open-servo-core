//! Chip-side motor service. Translates `osc_servo_core::MotorCmd` into TIM1
//! H-bridge writes per the DRV8212P truth table:
//! (1,1)=BRAKE, (0,0)=COAST, (1,0)=fwd, (0,1)=rev.
//! Slow decay holds the idle leg HIGH; Fast decay holds it LOW.

use osc_servo_core::{DecayMode, Motor as MotorTrait, MotorCmd};
use osc_servo_drivers::Level;

use crate::control::sensors::scan::{self, Direction};
use crate::hal::Pin;
use crate::hal::gpio::LevelWrite;
use crate::hal::timer::{self, DutyReg};

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
            MotorCmd::Drive {
                ticks,
                reverse,
                decay,
            } => {
                self.drv_en_on.apply();
                let (in1, in2) = legs(ticks, reverse, decay, self.pwm_arr);
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

/// `(in1, in2)` compares for a drive of `ticks`: the PWM leg's compare and
/// the idle leg held per `decay`.
#[inline(always)]
fn legs(ticks: u16, reverse: bool, decay: DecayMode, pwm_arr: u16) -> (u16, u16) {
    let fwd = match decay {
        DecayMode::Slow => (STATIC_HIGH, pwm_arr.saturating_sub(ticks)),
        DecayMode::Fast => (ticks, 0),
    };
    if reverse { (fwd.1, fwd.0) } else { fwd }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legs_follow_the_truth_table() {
        let arr = 1200;
        let t = 300;
        assert_eq!(legs(t, false, DecayMode::Slow, arr), (STATIC_HIGH, arr - t));
        assert_eq!(legs(t, true, DecayMode::Slow, arr), (arr - t, STATIC_HIGH));
        assert_eq!(legs(t, false, DecayMode::Fast, arr), (t, 0));
        assert_eq!(legs(t, true, DecayMode::Fast, arr), (0, t));
    }
}
