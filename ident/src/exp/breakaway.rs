//! Static-friction breakaway: from rest, ramp |duty| in small steps until
//! the pot moves, both directions. The breakaway currents sit below the
//! drive-window floor so nothing is measured - the result is MODEL-DERIVED
//! from the duty at first motion: fric = (duty_bk / 32767) * vbus / R,
//! which [`BreakawayResult`] flags. A direction that reaches the ramp cap
//! without motion reports None (jam / mechanics bound) instead of aborting
//! the run.

use super::{Cmd, Experiment};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct BreakawayCfg {
    /// Ramp start, q15 (8%).
    pub start_q15: i16,
    /// Ramp increment per dwell, q15 (0.5%).
    pub step_q15: i16,
    /// Ramp cap, q15 (30%): no motion by here = jammed.
    pub cap_q15: i16,
    /// Pot delta vs the dwell start that counts as motion.
    pub move_counts: u16,
    pub dwell_polls: u32,
    pub poll_ms: u32,
    /// Rest between the two directions.
    pub rest_ms: u32,
}

impl Default for BreakawayCfg {
    fn default() -> Self {
        Self {
            start_q15: 2621,
            step_q15: 164,
            cap_q15: 9830,
            move_counts: 8,
            dwell_polls: 5,
            poll_ms: 20,
            rest_ms: 400,
        }
    }
}

#[derive(Clone, Debug)]
pub struct BreakawayResult {
    pub duty_bk_fwd: Option<i16>,
    pub duty_bk_rev: Option<i16>,
    /// (duty_bk / 32767) * vbus / R per direction - MODEL-DERIVED, no
    /// current was measured at these duties.
    pub fric_fwd_counts: Option<f64>,
    pub fric_rev_counts: Option<f64>,
    pub model_derived: bool,
    /// |fwd - rev| / mean of the duty_bk pair, when both exist.
    pub asymmetry: Option<f64>,
}

enum Phase {
    ModeWrite,
    TorqueOn,
    FirstRead,
    DirStart,
    DutySet,
    DwellRead,
    DwellEval,
    DirDone,
    DirRest,
    FinishTorque,
    Finished,
}

pub struct Breakaway {
    cfg: BreakawayCfg,
    phase: Phase,
    /// First direction is away from the nearer end; second is opposite.
    first_dir: i8,
    dir_idx: u8,
    duty: i16,
    ref_pos: Option<u16>,
    polls_left: u32,
    duty_bk: [Option<i16>; 2],
}

impl Breakaway {
    pub fn new(cfg: BreakawayCfg) -> Self {
        Self {
            cfg,
            phase: Phase::ModeWrite,
            first_dir: 1,
            dir_idx: 0,
            duty: 0,
            ref_pos: None,
            polls_left: 0,
            duty_bk: [None; 2],
        }
    }

    fn dir(&self) -> i8 {
        if self.dir_idx == 0 {
            self.first_dir
        } else {
            -self.first_dir
        }
    }

    fn bk(&self, dir: i8) -> Option<i16> {
        let idx = if dir == self.first_dir { 0 } else { 1 };
        self.duty_bk[idx]
    }

    /// R in vcounts per ccount and vbus in vcounts come from the earlier
    /// resistance and bias runs.
    pub fn fit(&self, r_vpc: f64, vbus_mean: f64) -> BreakawayResult {
        let fric = |d: Option<i16>| d.map(|d| d as f64 / 32767.0 * vbus_mean / r_vpc);
        let (fwd, rev) = (self.bk(1), self.bk(-1));
        let asymmetry = match (fwd, rev) {
            (Some(f), Some(r)) => {
                let (f, r) = (f as f64, r as f64);
                Some((f - r).abs() / ((f + r) / 2.0))
            }
            _ => None,
        };
        BreakawayResult {
            duty_bk_fwd: fwd,
            duty_bk_rev: rev,
            fric_fwd_counts: fric(fwd),
            fric_rev_counts: fric(rev),
            model_derived: true,
            asymmetry,
        }
    }
}

impl Experiment for Breakaway {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::ModeWrite => {
                self.phase = Phase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            Phase::TorqueOn => {
                self.phase = Phase::FirstRead;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::FirstRead => {
                self.phase = Phase::DirStart;
                Cmd::Read
            }
            Phase::DirStart => {
                // ramp away from the nearer end so travel is not the limit
                if self.dir_idx == 0
                    && let Some(o) = obs
                {
                    self.first_dir = if o.pos < 2048 { 1 } else { -1 };
                }
                self.duty = self.cfg.start_q15;
                self.ref_pos = obs.map(|o| o.pos);
                self.phase = Phase::DutySet;
                Cmd::Pause { ms: 0 }
            }
            Phase::DutySet => {
                self.polls_left = self.cfg.dwell_polls;
                self.phase = Phase::DwellRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.dir() as i32 * self.duty as i32,
                }
            }
            Phase::DwellRead => {
                self.phase = Phase::DwellEval;
                Cmd::Read
            }
            Phase::DwellEval => {
                let moved = match (obs, self.ref_pos) {
                    (Some(o), Some(r)) => o.pos.abs_diff(r) > self.cfg.move_counts,
                    _ => false,
                };
                if moved {
                    self.duty_bk[self.dir_idx as usize] = Some(self.duty);
                    self.phase = Phase::DirDone;
                    return Cmd::Pause { ms: 0 };
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if self.polls_left > 0 {
                    self.phase = Phase::DwellRead;
                    return Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    };
                }
                self.duty = self.duty.saturating_add(self.cfg.step_q15);
                if self.duty > self.cfg.cap_q15 {
                    // jammed: this direction stays None, run continues
                    self.phase = Phase::DirDone;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::DutySet;
                    Cmd::Pause { ms: 0 }
                }
            }
            Phase::DirDone => {
                self.phase = if self.dir_idx == 0 {
                    Phase::DirRest
                } else {
                    Phase::FinishTorque
                };
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::DirRest => {
                self.dir_idx = 1;
                // re-anchor ref_pos where the first direction left the pot
                self.phase = Phase::FirstRead;
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
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
    use super::super::{Guarded, RigParams};
    use super::*;

    fn run_e1(servo: &mut FakeServo) -> (Breakaway, Vec<String>) {
        let params = RigParams::default();
        let mut exp = Guarded::new(Breakaway::new(BreakawayCfg::default()), params);
        let log = pump(&mut exp, servo, 200_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    #[test]
    fn recovers_planted_breakaway_duty() {
        let mut servo = FakeServo::new(3.37);
        servo.breakaway_q15 = 4000;
        let (exp, log) = run_e1(&mut servo);
        assert!(!log.contains(&"OVERRUN".to_string()));
        let fit = exp.fit(3.37, 1731.0);
        let fwd = fit.duty_bk_fwd.expect("fwd found");
        let rev = fit.duty_bk_rev.expect("rev found");
        // first motion lands within one ramp step above the plant threshold
        for bk in [fwd, rev] {
            assert!(
                bk > 4000 && bk <= 4000 + 2 * BreakawayCfg::default().step_q15,
                "bk {bk}"
            );
        }
        assert!(fit.model_derived);
        let fr = fit.fric_fwd_counts.unwrap();
        let expect = fwd as f64 / 32767.0 * 1731.0 / 3.37;
        assert!((fr - expect).abs() < 1e-9);
        assert!(fit.asymmetry.unwrap() < 0.1);
    }

    #[test]
    fn jam_reports_none_and_finishes_safe() {
        let mut servo = FakeServo::new(3.37);
        servo.breakaway_q15 = i16::MAX; // never moves
        let (exp, log) = run_e1(&mut servo);
        let fit = exp.fit(3.37, 1731.0);
        assert!(fit.duty_bk_fwd.is_none());
        assert!(fit.duty_bk_rev.is_none());
        assert!(fit.asymmetry.is_none());
        assert_eq!(*log.last().unwrap(), "write torque_enable 0");
    }
}
