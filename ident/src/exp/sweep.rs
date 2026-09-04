//! Dedicated constant-duty ripple sweep: one clean rail-to-rail traverse
//! whose per-tick TEL current carries an uninterrupted commutation-ripple
//! signal for the tachometer and pot LUT.
//!
//! Why a separate motion and not the end-stop seek: the seek must poll pos to
//! detect each stall, and every poll is a bus read the pump cannot drain TEL
//! through - so the seek capture is shredded into sub-millisecond seq-
//! contiguous fragments, too short for the autocorr window, and it spends
//! most of its time dwelling stalled at a rail (flat current, no ripple).
//! This experiment instead drives one fixed duty and does nothing but Pause,
//! so the pump drains TEL continuously and the whole traverse lands as a
//! single long contiguous run.
//!
//! No safety Reads during the traverse (they would re-fragment the capture):
//! the firmware current limit + fault protection are the winding's backstop,
//! and the identical seek duty was just proven safe against both hard stops
//! by [`super::endstop`]. The caller parks duty 0 + torque off on exit.

use super::{Cmd, Experiment};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct SweepCfg {
    /// Constant sweep drive magnitude, q15 (mirrors the end-stop seek duty so
    /// the winding load is the same proven-safe operating point).
    pub duty_q15: i16,
    /// Drive to the start rail for this long before the capturing traverse, so
    /// the sweep begins from a known end rather than mid-travel.
    pub prime_ms: u32,
    /// Constant-duty traverse duration. Longer than a full traverse takes: the
    /// tail dwells stalled at the far rail and is trimmed by the moving-run
    /// selector, so overshoot is free.
    pub sweep_ms: u32,
}

impl Default for SweepCfg {
    fn default() -> Self {
        Self {
            duty_q15: 8520,
            prime_ms: 500,
            sweep_ms: 800,
        }
    }
}

enum Phase {
    ModeWrite,
    TorqueOn,
    PrimeDuty,
    PrimePause,
    SweepDuty,
    SweepPause,
    FinishDuty,
    FinishTorque,
    Finished,
}

pub struct Sweep {
    cfg: SweepCfg,
    phase: Phase,
    /// Duty sign that drives pos toward the far rail (the capture direction);
    /// the caller derives it from the measured drive polarity.
    sweep_sign: i8,
}

impl Sweep {
    pub fn new(cfg: SweepCfg, sweep_sign: i8) -> Self {
        Self {
            cfg,
            phase: Phase::ModeWrite,
            sweep_sign: if sweep_sign < 0 { -1 } else { 1 },
        }
    }

    fn sweep_duty(&self) -> i32 {
        self.sweep_sign as i32 * self.cfg.duty_q15 as i32
    }
}

impl Experiment for Sweep {
    fn step(&mut self, _obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::ModeWrite => {
                self.phase = Phase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            Phase::TorqueOn => {
                self.phase = Phase::PrimeDuty;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::PrimeDuty => {
                self.phase = Phase::PrimePause;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: -self.sweep_duty(),
                }
            }
            Phase::PrimePause => {
                self.phase = Phase::SweepDuty;
                Cmd::Pause {
                    ms: self.cfg.prime_ms,
                }
            }
            Phase::SweepDuty => {
                self.phase = Phase::SweepPause;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.sweep_duty(),
                }
            }
            Phase::SweepPause => {
                self.phase = Phase::FinishDuty;
                Cmd::Pause {
                    ms: self.cfg.sweep_ms,
                }
            }
            Phase::FinishDuty => {
                self.phase = Phase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::FinishTorque => {
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, pump};
    use super::*;

    fn run(sweep_sign: i8) -> Vec<String> {
        let mut servo = FakeServo::new(3.37);
        servo.ends = (321.0, 3702.0);
        servo.pos = 2000.0;
        let mut exp = Sweep::new(SweepCfg::default(), sweep_sign);
        let log = pump(&mut exp, &mut servo, 2_000);
        assert!(!log.contains(&"OVERRUN".to_string()));
        log
    }

    #[test]
    fn choreography_is_safe_and_primes_opposite_the_sweep() {
        let log = run(1);
        // mode set before anything drives
        assert_eq!(log[0], "write mode 0");
        // torque on before any nonzero duty
        let torque_on = log
            .iter()
            .position(|l| l == "write torque_enable 1")
            .unwrap();
        let first_duty = log
            .iter()
            .position(|l| l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            .unwrap();
        assert!(torque_on < first_duty);
        // prime drives negative (toward the start rail), sweep positive
        let duties: Vec<&String> = log
            .iter()
            .filter(|l| l.starts_with("write goal_duty"))
            .collect();
        assert_eq!(
            duties[0], "write goal_duty -8520",
            "prime toward start rail"
        );
        assert_eq!(duties[1], "write goal_duty 8520", "sweep toward far rail");
        // ends parked: duty 0 then torque off
        let tail: Vec<&String> = log.iter().rev().take(2).collect();
        assert_eq!(*tail[1], "write goal_duty 0");
        assert_eq!(*tail[0], "write torque_enable 0");
    }

    #[test]
    fn negative_polarity_flips_prime_and_sweep() {
        let log = run(-1);
        let duties: Vec<&String> = log
            .iter()
            .filter(|l| l.starts_with("write goal_duty"))
            .collect();
        assert_eq!(duties[0], "write goal_duty 8520");
        assert_eq!(duties[1], "write goal_duty -8520");
    }
}
