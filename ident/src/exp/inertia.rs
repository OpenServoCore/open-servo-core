//! Inertia duty steps: from rest, step the duty and capture the
//! accelerating transient, several amplitudes in both directions. Each step
//! is one TEL burst ([`Cmd::Stream`]): the goal-duty write and the capture
//! arm commit in the same instant, so tick 0 IS the step edge and the
//! per-tick pos/current/duty/vdiff series (20 kHz, ladder mask) covers the
//! whole transient. Seek polling between bursts stays ordinary Read/Pause.
//! The firmware's soft-limit duty clamp bounds the traverse during the
//! silent capture window.
//!
//! Fit inputs and estimators (direct-alpha and exponential-rise) live in
//! [`crate::fits`]; [`Inertia::fit`] assembles per-step series and picks
//! `b_best` by fit quality.

use super::{Cmd, Experiment, RigParams};
use crate::fits::{BDirect, BExp, InertiaPriors, StepSeries, b_direct_fit, b_exp_fit};
use crate::frame::{TelFrame, TelemetrySnapshot};
use crate::regs::control;

/// TEL frame layout the choreography arms: pos + current + duty + vdiff.
pub const TEL_LADDER_MASK: u16 = 0x1B;

pub struct InertiaCfg {
    /// Step duties, q15, each run + then - (35/45/55%).
    pub steps_q15: Vec<i16>,
    pub seek_duty_q15: i16,
    /// Seek target distance from the band edge: wide enough that the
    /// coast-down after the seek duty drops stays inside the guard.
    pub seek_margin: u16,
    pub seek_poll_ms: u32,
    pub rest_ms: u32,
    /// Step burst duration; with `tick_hz` this sizes the TEL arm.
    pub capture_ms: u32,
    pub stall_eps: u16,
    pub stall_polls: u32,
    pub seek_cap_polls: u32,
    pub tick_hz: f64,
}

impl Default for InertiaCfg {
    fn default() -> Self {
        Self {
            // 35/45/55% of 32767
            steps_q15: vec![11468, 14745, 18022],
            seek_duty_q15: 8520,
            seek_margin: 700,
            seek_poll_ms: 30,
            rest_ms: 300,
            capture_ms: 400,
            stall_eps: 3,
            stall_polls: 10,
            seek_cap_polls: 400,
            tick_hz: 20_100.0,
        }
    }
}

/// One captured step: the burst's decoded frames.
#[derive(Clone, Debug, Default)]
struct StepCapture {
    duty_q15: i16,
    tel: Vec<TelFrame>,
}

#[derive(Clone, Debug)]
pub struct InertiaResult {
    pub b_direct: Option<BDirect>,
    pub b_exp: Option<BExp>,
    /// The chosen estimate: exp-rise unless the direct fit's r2 beats it.
    pub b_best: f64,
    /// 1 / b_best - the velocity feed-forward the table wants.
    pub j_ff: f64,
    pub tel_steps: usize,
    pub warnings: Vec<String>,
}

enum Phase {
    ModeWrite,
    TorqueOn,
    TelMask,
    SeekSet,
    SeekRead,
    SeekEval,
    SeekOff,
    SeekRest,
    StepStream,
    StepOff,
    StepRest,
    TelMaskOff,
    FinishTorque,
    Finished,
}

pub struct Inertia {
    cfg: InertiaCfg,
    params: RigParams,
    band: (u16, u16),
    phase: Phase,
    /// Step index: amplitude = step / 2, direction = +1 then -1.
    step: usize,
    last_pos: Option<u16>,
    still: u32,
    polls: u32,
    capturing: bool,
    cur: StepCapture,
    captures: Vec<StepCapture>,
    warnings: Vec<String>,
}

impl Inertia {
    pub fn new(cfg: InertiaCfg, params: &RigParams) -> Self {
        let band = params.pos_guard.unwrap_or((150, 3950));
        Self {
            cfg,
            params: *params,
            band,
            phase: Phase::ModeWrite,
            step: 0,
            last_pos: None,
            still: 0,
            polls: 0,
            capturing: false,
            cur: StepCapture::default(),
            captures: Vec::new(),
            warnings: Vec::new(),
        }
    }

    fn dir(&self) -> i8 {
        if self.step.is_multiple_of(2) { 1 } else { -1 }
    }

    fn duty(&self) -> i16 {
        let d = self.cfg.steps_q15[self.step / 2];
        if self.dir() > 0 { d } else { -d }
    }

    fn capture_samples(&self) -> u16 {
        let n = self.cfg.capture_ms as f64 * self.cfg.tick_hz / 1000.0;
        n.min(u16::MAX as f64) as u16
    }

    fn seek_done(&self, pos: u16) -> bool {
        if self.dir() > 0 {
            pos <= self.band.0 + self.cfg.seek_margin
        } else {
            pos >= self.band.1 - self.cfg.seek_margin
        }
    }

    fn reset_motion_track(&mut self) {
        self.last_pos = None;
        self.still = 0;
        self.polls = 0;
    }

    fn track_still(&mut self, pos: u16) {
        if let Some(last) = self.last_pos
            && pos.abs_diff(last) <= self.cfg.stall_eps
        {
            self.still += 1;
        } else {
            self.still = 0;
        }
        self.last_pos = Some(pos);
    }

    fn close_step(&mut self) {
        self.capturing = false;
        let done = core::mem::take(&mut self.cur);
        self.captures.push(done);
    }

    /// One captured step -> the fit series. Time is tick / tick_hz with
    /// tick 0 at the step edge (the COMMIT); `mask` folds in per-sample
    /// window_valid and slip-zone exclusion.
    fn series_of(&self, c: &StepCapture) -> Option<StepSeries> {
        let tick_s = 1.0 / self.cfg.tick_hz;
        let mut t = Vec::new();
        let mut pos = Vec::new();
        let mut i = Vec::new();
        let mut mask = Vec::new();
        for f in &c.tel {
            let (Some(p), Some(cur)) = (f.pos, f.current) else {
                continue;
            };
            t.push(f.tick as f64 * tick_s);
            pos.push(p as f64);
            i.push(cur as f64);
            mask.push(f.window_valid && !self.params.in_slip(p));
        }
        (t.len() >= 32).then_some(StepSeries {
            t,
            pos,
            i,
            mask,
            duty_q15: c.duty_q15 as f64,
        })
    }

    /// Every captured step reduced to its fit series, tagged TEL-sourced -
    /// the CLI records these for offline refits (recorded aggregate-tagged
    /// series from old runs refit the same way).
    pub fn step_series(&self) -> Vec<(StepSeries, bool)> {
        self.captures
            .iter()
            .filter_map(|c| self.series_of(c).map(|s| (s, true)))
            .collect()
    }

    /// Assemble series and run both estimators; the smoothing half-window
    /// spans ~10 ms of ticks.
    pub fn fit(&self, priors: &InertiaPriors) -> Option<InertiaResult> {
        let mut series = Vec::new();
        let mut warnings = self.warnings.clone();
        for c in &self.captures {
            match self.series_of(c) {
                Some(s) => series.push(s),
                None => warnings.push(format!("step {}: too few samples, dropped", c.duty_q15)),
            }
        }
        if series.is_empty() {
            return None;
        }
        let tel_steps = series.len();
        let hw = (0.010 * self.cfg.tick_hz) as usize;
        let b_direct = b_direct_fit(&series, priors, hw, 5.0);
        let b_exp = b_exp_fit(&series, priors, hw);
        let b_best = match (&b_exp, &b_direct) {
            (Some(e), Some(d)) => {
                // exp-rise wins unless the direct pool is decisively cleaner
                if d.r2 > 0.98 && d.r2 > 1.0 - e.spread {
                    d.b
                } else {
                    e.b
                }
            }
            (Some(e), None) => e.b,
            (None, Some(d)) => d.b,
            (None, None) => return None,
        };
        Some(InertiaResult {
            b_direct,
            b_exp,
            b_best,
            j_ff: 1.0 / b_best,
            tel_steps,
            warnings,
        })
    }
}

impl Experiment for Inertia {
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
                self.phase = Phase::TelMask;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::TelMask => {
                self.phase = Phase::SeekSet;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: TEL_LADDER_MASK as i32,
                }
            }
            Phase::SeekSet => {
                self.reset_motion_track();
                self.phase = Phase::SeekRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: -(self.dir() as i32) * self.cfg.seek_duty_q15 as i32,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => {
                let mut done = false;
                if let Some(o) = obs {
                    self.track_still(o.pos);
                    done = self.seek_done(o.pos) || self.still >= self.cfg.stall_polls;
                }
                self.polls += 1;
                if !done && self.polls >= self.cfg.seek_cap_polls {
                    self.warnings
                        .push(format!("step {}: seek gave up, starting here", self.step));
                    done = true;
                }
                if done {
                    self.phase = Phase::SeekOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            Phase::SeekOff => {
                self.phase = Phase::SeekRest;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::SeekRest => {
                self.phase = Phase::StepStream;
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::StepStream => {
                self.capturing = true;
                self.cur = StepCapture {
                    duty_q15: self.duty(),
                    ..StepCapture::default()
                };
                self.phase = Phase::StepOff;
                Cmd::Stream {
                    samples: self.capture_samples(),
                    goal: Some((control::GOAL_DUTY, self.duty() as i32)),
                }
            }
            Phase::StepOff => {
                self.close_step();
                self.phase = Phase::StepRest;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::StepRest => {
                self.step += 1;
                self.phase = if self.step < 2 * self.cfg.steps_q15.len() {
                    Phase::SeekSet
                } else {
                    Phase::TelMaskOff
                };
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::TelMaskOff => {
                self.phase = Phase::FinishTorque;
                Cmd::Write {
                    reg: control::TEL_MASK,
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

    /// Frames arriving outside a step burst are dropped.
    fn push_tel(&mut self, frames: &[TelFrame]) {
        if self.capturing {
            self.cur.tel.extend_from_slice(frames);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, pump};
    use super::super::{Guarded, RigParams};
    use super::*;

    const B_PLANT: f64 = 0.1;

    fn dynamic_servo() -> FakeServo {
        let mut s = FakeServo::new(3.37);
        s.dynamic = true;
        s.b = B_PLANT;
        s.fv = 0.006;
        s.pos_noise = 1.5;
        s
    }

    fn priors() -> InertiaPriors {
        InertiaPriors {
            r_vpc: 3.37,
            ke_vpc: 0.1731,
            fc: 20.0,
            fv: 0.006,
            tick_hz: 20_100.0,
        }
    }

    fn run() -> (Inertia, Vec<String>) {
        let mut servo = dynamic_servo();
        let params = RigParams::default();
        let mut exp = Guarded::new(Inertia::new(InertiaCfg::default(), &params), params);
        let log = pump(&mut exp, &mut servo, 4_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    #[test]
    fn recovers_planted_b_from_burst_captures() {
        let (exp, log) = run();
        assert!(!log.contains(&"OVERRUN".to_string()));
        let fit = exp.fit(&priors()).expect("fit");
        assert_eq!(fit.tel_steps, 6, "every step captures one burst");
        let e = fit.b_exp.as_ref().expect("exp-rise fit");
        assert!(
            (e.b - B_PLANT).abs() / B_PLANT < 0.05,
            "b_exp {} spread {}",
            e.b,
            e.spread
        );
        let d = fit.b_direct.as_ref().expect("direct fit");
        assert!((d.b - B_PLANT).abs() / B_PLANT < 0.10, "b_direct {}", d.b);
        assert!((fit.b_best - B_PLANT).abs() / B_PLANT < 0.05);
        assert!((fit.j_ff - 1.0 / fit.b_best).abs() < 1e-12);
    }

    #[test]
    fn choreography_arms_masked_bursts_and_disarms() {
        let (_, log) = run();
        let idx = |needle: &str| log.iter().position(|l| l == needle).unwrap();
        let mask_on = idx("write tel_mask 27");
        let mask_off = idx("write tel_mask 0");
        let first_stream = log
            .iter()
            .position(|l| l.starts_with("stream "))
            .expect("a burst");
        let torque_off = log
            .iter()
            .rposition(|l| l == "write torque_enable 0")
            .unwrap();
        assert!(mask_on < first_stream, "mask written before the first arm");
        assert!(mask_off < torque_off, "tel down before torque off");
        // six step bursts, each a goal+arm commit
        let streams: Vec<&String> = log.iter().filter(|l| l.starts_with("stream ")).collect();
        assert_eq!(streams.len(), 6);
        assert_eq!(*streams[0], "stream 8040 goal_duty 11468");
        assert_eq!(*streams[1], "stream 8040 goal_duty -11468");
        // last duty write is the safety zero
        let last_duty = log
            .iter()
            .rev()
            .find(|l| l.starts_with("write goal_duty"))
            .unwrap();
        assert_eq!(last_duty, "write goal_duty 0");
    }

    #[test]
    fn frames_outside_a_burst_are_dropped() {
        let params = RigParams::default();
        let mut exp = Inertia::new(InertiaCfg::default(), &params);
        exp.push_tel(&[TelFrame {
            tick: 0,
            pos: Some(2000),
            current: Some(10),
            ..Default::default()
        }]);
        assert!(exp.captures.is_empty());
        assert!(exp.cur.tel.is_empty(), "not capturing: frame dropped");
    }
}
