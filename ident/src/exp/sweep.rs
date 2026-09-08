//! Dedicated constant-duty ripple capture: one clean constant-duty traverse
//! whose per-tick TEL current carries an uninterrupted commutation-ripple
//! signal for the tachometer and pot LUT.
//!
//! The whole traverse is ONE burst ([`Cmd::Stream`]): the constant duty and
//! the capture arm commit in the same instant, and the bus carries nothing
//! but the stream until LAST - so the capture lands seq-contiguous by
//! construction, with no poll seams to shred the autocorr window.
//!
//! No positioning or speed probe here: the caller positions to the capture
//! start rail-inset first (polling is free before the arm) and sizes
//! `samples` from a measured speed so the traverse stays off BOTH
//! mechanical rails - the clone can jam at either end. The firmware
//! current limit + fault protection guard the winding during the silent
//! window; the caller parks duty 0 + torque off on exit.

use super::{Cmd, Experiment};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct SweepCfg {
    /// Constant capture drive magnitude, q15 (mirrors the end-stop seek duty so
    /// the winding load is the same proven-safe operating point).
    pub duty_q15: i16,
    /// Burst length in fast ticks. The caller sizes this from a measured
    /// speed so the motion covers the count-inset span without reaching a rail.
    pub samples: u16,
    /// TEL field selection, written before the arm (mask is sticky).
    pub mask: u16,
}

impl Default for SweepCfg {
    fn default() -> Self {
        Self {
            duty_q15: 8520,
            samples: 16_000,
            mask: 0x1B,
        }
    }
}

enum Phase {
    ModeWrite,
    TorqueOn,
    MaskOn,
    Stream,
    ZeroDuty,
    TorqueOff,
    MaskOff,
    Finished,
}

pub struct Sweep {
    cfg: SweepCfg,
    phase: Phase,
    /// Duty sign that drives pos toward the capture end (increasing pos); the
    /// caller derives it from the measured drive polarity.
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

    fn capture_duty(&self) -> i32 {
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
                self.phase = Phase::MaskOn;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::MaskOn => {
                self.phase = Phase::Stream;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: self.cfg.mask as i32,
                }
            }
            Phase::Stream => {
                self.phase = Phase::ZeroDuty;
                Cmd::Stream {
                    samples: self.cfg.samples,
                    goal: Some((control::GOAL_DUTY, self.capture_duty())),
                }
            }
            Phase::ZeroDuty => {
                self.phase = Phase::TorqueOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::TorqueOff => {
                self.phase = Phase::MaskOff;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::MaskOff => {
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::TEL_MASK,
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
    fn choreography_is_safe() {
        let log = run(1);
        // mode set before anything drives
        assert_eq!(log[0], "write mode 0");
        // torque on and mask written before the burst arms
        let torque_on = log
            .iter()
            .position(|l| l == "write torque_enable 1")
            .unwrap();
        let mask_on = log.iter().position(|l| l == "write tel_mask 27").unwrap();
        let stream = log.iter().position(|l| l.starts_with("stream ")).unwrap();
        assert!(torque_on < stream);
        assert!(mask_on < stream);
        // one burst carrying the constant capture duty
        assert_eq!(log[stream], "stream 16000 goal_duty 8520");
        // ends parked: duty 0, torque off, mask off
        let tail: Vec<&String> = log.iter().rev().take(3).collect();
        assert_eq!(*tail[2], "write goal_duty 0");
        assert_eq!(*tail[1], "write torque_enable 0");
        assert_eq!(*tail[0], "write tel_mask 0");
    }

    #[test]
    fn negative_sign_flips_capture_duty() {
        let neg = run(-1);
        let neg_duty = neg.iter().find(|l| l.starts_with("stream ")).unwrap();
        assert_eq!(neg_duty, "stream 16000 goal_duty -8520");
    }
}
