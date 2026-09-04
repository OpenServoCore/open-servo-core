//! Winding resistance from end-stop stalls: seek each physical end at
//! modest duty, then step a duty ladder with rests between dwells; stalled,
//! every window gives v = duty * vdiff / 32767 across the winding and
//! i = that v over R, so origin LS on (i, v) is R in vcounts per ccount.
//! Runs FIRST while the winding is cold; the per-dwell R drift over time is
//! reported (heating metric), not compensated.
//!
//! Run with the pos guard disabled ([`super::RigParams::without_pos_guard`]):
//! stalling at the physical ends is the method. The rig's stall detector
//! must be non-latching (stall_response Yield) or tripped above the ladder
//! currents; a latched STALL aborts the run via the fault check.

use super::{Cmd, Experiment, RigParams, WindowSample, WindowStream};
use crate::fitmath::{linear_ls, origin_ls};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct ResistanceCfg {
    /// Seek drive toward each end, q15 (26% clears the current window
    /// floor with margin).
    pub seek_duty_q15: i16,
    /// Dwell duties, q15, applied with the seek direction's sign.
    pub ladder_q15: Vec<i16>,
    /// Polls per dwell; with `poll_ms` this sets the ~400 ms dwell.
    pub dwell_polls: u32,
    pub poll_ms: u32,
    /// Cool-down between dwells, duty 0.
    pub rest_ms: u32,
    /// Stall detect: pos delta <= eps across `stall_polls` consecutive
    /// seek polls.
    pub stall_eps: u16,
    pub stall_polls: u32,
    pub seek_poll_ms: u32,
}

impl Default for ResistanceCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 8520,
            // 26/29/32/35% of 32767
            ladder_q15: vec![8520, 9503, 10485, 11468],
            dwell_polls: 200,
            poll_ms: 2,
            rest_ms: 300,
            stall_eps: 3,
            stall_polls: 8,
            seek_poll_ms: 30,
        }
    }
}

/// One accepted window tagged with its dwell.
#[derive(Copy, Clone, Debug)]
pub struct DwellSample {
    pub dwell: u32,
    pub dir: i8,
    pub w: WindowSample,
}

#[derive(Clone, Debug)]
pub struct ResistanceResult {
    /// Combined origin-LS R, vcounts per ccount.
    pub r_vpc: f64,
    pub r_fwd: Option<f64>,
    pub r_rev: Option<f64>,
    pub r2: f64,
    pub rms: f64,
    pub n: usize,
    /// Per-dwell R trend over the run, vcounts per ccount per second -
    /// winding heating shows as a positive slope.
    pub drift_vpc_per_s: f64,
}

enum Phase {
    ModeWrite,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    DwellSet,
    DwellRead,
    DwellEval,
    RestOff,
    RestPause,
    FinishDuty,
    FinishTorque,
    Finished,
}

pub struct Resistance {
    cfg: ResistanceCfg,
    phase: Phase,
    dir_idx: u8,
    ladder_idx: usize,
    dwell: u32,
    polls_left: u32,
    last_pos: Option<u16>,
    still: u32,
    windows: WindowStream,
    samples: Vec<DwellSample>,
}

impl Resistance {
    pub fn new(cfg: ResistanceCfg, params: &RigParams) -> Self {
        Self {
            cfg,
            phase: Phase::ModeWrite,
            dir_idx: 0,
            ladder_idx: 0,
            dwell: 0,
            polls_left: 0,
            last_pos: None,
            still: 0,
            windows: WindowStream::new(params),
            samples: Vec::new(),
        }
    }

    fn dir(&self) -> i8 {
        if self.dir_idx == 0 { 1 } else { -1 }
    }

    pub fn samples(&self) -> &[DwellSample] {
        &self.samples
    }

    /// Fold and fit: v = duty * vdiff / 32767 (positive both directions -
    /// duty and vdiff carry the same sign), i folded by drive sign so
    /// current into the stall is positive.
    pub fn fit(&self) -> Option<ResistanceResult> {
        let iv = |s: &DwellSample| {
            let v = s.w.duty_q15 * s.w.vdiff / 32767.0;
            let i = s.w.i * s.w.duty_q15.signum();
            (i, v)
        };
        let all: Vec<(f64, f64)> = self.samples.iter().map(iv).collect();
        let fit = origin_ls(&all)?;
        let side = |d: i8| {
            let pts: Vec<(f64, f64)> = self.samples.iter().filter(|s| s.dir == d).map(iv).collect();
            origin_ls(&pts).map(|f| f.slope)
        };
        // per-dwell R centroid vs time -> heating drift
        let mut dwell_rs: Vec<(f64, f64)> = Vec::new();
        let mut d = 0;
        while let Some(pts) = {
            let pts: Vec<&DwellSample> = self.samples.iter().filter(|s| s.dwell == d).collect();
            if pts.is_empty() { None } else { Some(pts) }
        } {
            let t = pts.iter().map(|s| s.w.t_ms).sum::<f64>() / pts.len() as f64 / 1000.0;
            let xy: Vec<(f64, f64)> = pts.iter().map(|s| iv(s)).collect();
            if let Some(f) = origin_ls(&xy) {
                dwell_rs.push((t, f.slope));
            }
            d += 1;
        }
        let drift = linear_ls(&dwell_rs).map(|f| f.b).unwrap_or(0.0);
        Some(ResistanceResult {
            r_vpc: fit.slope,
            r_fwd: side(1),
            r_rev: side(-1),
            r2: fit.r2,
            rms: fit.rms,
            n: fit.n,
            drift_vpc_per_s: drift,
        })
    }
}

impl Experiment for Resistance {
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
                self.phase = Phase::SeekSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::SeekSet => {
                self.phase = Phase::SeekRead;
                self.last_pos = None;
                self.still = 0;
                self.windows.mark_transition();
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.dir() as i32 * self.cfg.seek_duty_q15 as i32,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => {
                if let Some(o) = obs {
                    if let Some(last) = self.last_pos
                        && o.pos.abs_diff(last) <= self.cfg.stall_eps
                    {
                        self.still += 1;
                    } else {
                        self.still = 0;
                    }
                    self.last_pos = Some(o.pos);
                }
                if self.still >= self.cfg.stall_polls {
                    self.ladder_idx = 0;
                    self.phase = Phase::DwellSet;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            Phase::DwellSet => {
                self.phase = Phase::DwellRead;
                self.polls_left = self.cfg.dwell_polls;
                self.windows.mark_transition();
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.dir() as i32 * self.cfg.ladder_q15[self.ladder_idx] as i32,
                }
            }
            Phase::DwellRead => {
                self.phase = Phase::DwellEval;
                Cmd::Read
            }
            Phase::DwellEval => {
                if let Some(o) = obs
                    && let Some(w) = self.windows.push(o)
                {
                    self.samples.push(DwellSample {
                        dwell: self.dwell,
                        dir: self.dir(),
                        w,
                    });
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if self.polls_left == 0 {
                    self.phase = Phase::RestOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::DwellRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            Phase::RestOff => {
                self.phase = Phase::RestPause;
                self.dwell += 1;
                self.windows.mark_transition();
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::RestPause => {
                self.ladder_idx += 1;
                if self.ladder_idx < self.cfg.ladder_q15.len() {
                    self.phase = Phase::DwellSet;
                } else if self.dir_idx == 0 {
                    self.dir_idx = 1;
                    self.phase = Phase::SeekSet;
                } else {
                    self.phase = Phase::FinishDuty;
                }
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
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
    use super::super::{Guarded, RigParams};
    use super::*;

    fn run_e2(servo: &mut FakeServo) -> (Resistance, Vec<String>) {
        let params = RigParams::default().without_pos_guard();
        let mut exp = Guarded::new(Resistance::new(ResistanceCfg::default(), &params), params);
        let log = pump(&mut exp, servo, 2_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    #[test]
    fn recovers_planted_r_end_to_end() {
        let mut servo = FakeServo::new(3.37);
        let (exp, log) = run_e2(&mut servo);
        assert!(!log.contains(&"OVERRUN".to_string()));
        let fit = exp.fit().expect("samples collected");
        // settle discard keeps the planted 1.5x transient out of the fit,
        // so R lands on the plant value despite it
        assert!(
            (fit.r_vpc - 3.37).abs() < 0.02,
            "r {} (n {})",
            fit.r_vpc,
            fit.n
        );
        assert!(fit.r2 > 0.999, "r2 {}", fit.r2);
        assert!(fit.n > 100, "n {}", fit.n);
        let fwd = fit.r_fwd.unwrap();
        let rev = fit.r_rev.unwrap();
        assert!((fwd - rev).abs() < 0.02, "fwd {fwd} rev {rev}");
        assert!(
            fit.drift_vpc_per_s.abs() < 0.005,
            "drift {}",
            fit.drift_vpc_per_s
        );
    }

    #[test]
    fn command_choreography_is_safe() {
        let mut servo = FakeServo::new(3.37);
        let (_, log) = run_e2(&mut servo);
        let idx = |s: &str| log.iter().position(|l| l == s);
        // torque on before any nonzero duty
        let torque_on = idx("write torque_enable 1").expect("torque on");
        let first_duty = log
            .iter()
            .position(|l| l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            .expect("a drive command");
        assert!(torque_on < first_duty);
        // rests between dwells: a duty-0 write between successive ladder duties
        let zeros = log.iter().filter(|l| *l == "write goal_duty 0").count();
        assert!(zeros >= 8, "one rest per dwell, got {zeros}");
        // final safety pair
        let tail: Vec<&String> = log.iter().rev().take(2).collect();
        assert_eq!(*tail[1], "write goal_duty 0");
        assert_eq!(*tail[0], "write torque_enable 0");
    }

    #[test]
    fn transient_windows_do_not_pollute_the_fit() {
        // crank the transient way up; only the settle discard protects R
        let mut servo = FakeServo::new(3.37);
        servo.transient_gain = 4.0;
        let (exp, _) = run_e2(&mut servo);
        let fit = exp.fit().unwrap();
        assert!((fit.r_vpc - 3.37).abs() < 0.02, "r {}", fit.r_vpc);
    }
}
