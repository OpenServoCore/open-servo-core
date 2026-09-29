//! Inertia duty steps from a moving base: the shaft runs at the base duty,
//! the base's running current is read, then the duty steps up and the
//! accelerating transient is captured, several amplitudes in both
//! directions. From rest the steps the current limit leaves room for sit
//! inside a servo's breakaway; running, friction is already paid.
//!
//! A step is sized so its first edge stays under the band the firmware
//! limiter holds ([`DutyPlan::step_run`] at the base's current), and each
//! is one TEL burst ([`Cmd::Stream`]): the goal-duty write and the capture
//! arm commit in the same instant, so tick 0 IS the step edge. The fit's
//! input is the applied duty the burst reports, never the step as
//! commanded: samples still slewing are trimmed, and a step the limiter
//! held under its goal is declined. Seek and base polling stay ordinary
//! Read/Pause. The firmware's soft-limit duty clamp bounds the traverse
//! during the silent capture window. A seek that comes to rest short of
//! both its target and a stop ends the run ([`super::seek::at_stop`]); so
//! does a base that does not travel, or that the firmware's stall timer
//! folds.
//!
//! Fit inputs and estimators (direct-alpha and exponential-rise) live in
//! [`crate::fits`]; [`Inertia::fit`] assembles per-step series and picks
//! `b_best` by fit quality.

use super::seek::Watch;
use super::{
    AbortReason, Applied, Cmd, Experiment, GOVERNED, LIMIT_YIELD_FOLDED, RigParams, judge, seek,
};
use crate::fits::{BDirect, BExp, InertiaPriors, StepSeries, b_direct_fit, b_exp_fit};
use crate::frame::{TelFrame, TelemetrySnapshot};
use crate::limits::{DutyPlan, q15_floor};
use crate::regs::control;

/// TEL frame layout the choreography arms: pos + current + duty + vdiff,
/// plus pos_lin while the rig's position table is live ([`crate::pot::Pot`]).
pub const TEL_LADDER_MASK: u16 = 0x1B;

/// The base over the seek duty, a fraction of full scale.
pub const BASE_OVER_SEEK: f64 = 0.05;

pub struct InertiaCfg {
    /// Step sizes, fractions of [`DutyPlan::step_run`] at the base's
    /// running current, each run + then -.
    pub step_fracs: Vec<f64>,
    pub seek_duty_q15: i16,
    /// The moving base every step starts from, q15.
    pub base_q15: i16,
    /// Base polls before the step; the running current is the mean over
    /// the second half.
    pub base_polls: u32,
    pub base_poll_ms: u32,
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
            step_fracs: vec![0.5, 0.75, 1.0],
            seek_duty_q15: 8520,
            // 26% + 5%
            base_q15: 10158,
            // 100 ms: five motor time constants of the class
            base_polls: 10,
            base_poll_ms: 10,
            seek_margin: 700,
            seek_poll_ms: 30,
            rest_ms: 300,
            // Sized to the runway, not the transient: after the base's 100 ms
            // the step travels under 1000 of the ~2900 counts from the seek
            // band to the far soft wall on the bench servo, either rail.
            capture_ms: 150,
            stall_eps: 3,
            stall_polls: 10,
            seek_cap_polls: 400,
            tick_hz: 20_100.0,
        }
    }
}

impl InertiaCfg {
    /// The step goals over the base, fractions of full scale, for a base
    /// drawing `i_run` counts.
    pub fn step_duties(&self, plan: &DutyPlan, i_run: f64) -> Vec<f64> {
        let base = self.base_q15 as f64 / 32767.0;
        let run = plan.step_run(i_run);
        self.step_fracs.iter().map(|f| base + f * run).collect()
    }
}

/// One captured step: the goal, the base it slewed from, and the burst's
/// decoded frames.
#[derive(Clone, Debug, Default)]
struct StepCapture {
    goal_q15: i16,
    base_q15: i16,
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
    BaseSet,
    BaseWait,
    BaseRead,
    BaseEval,
    StepStream,
    StepOff,
    StepRest,
    TelMaskOff,
    FinishTorque,
    Finished,
}

pub struct Inertia {
    cfg: InertiaCfg,
    plan: DutyPlan,
    params: RigParams,
    band: (u16, u16),
    phase: Phase,
    /// Step index: amplitude = step / 2, direction = +1 then -1.
    step: usize,
    last_pos: Option<u16>,
    still: u32,
    polls: u32,
    start: Option<u16>,
    halt: Option<AbortReason>,
    watch: Option<Watch>,
    /// Dir-folded running current summed over the base's second half, and
    /// the reads it sums.
    i_run: (f64, u32),
    /// The limiter held the base under its goal over that half.
    base_governed: bool,
    goal_q15: i16,
    capturing: bool,
    cur: StepCapture,
    captures: Vec<StepCapture>,
    warnings: Vec<String>,
}

impl Inertia {
    /// `plan` sizes the steps from the servo's limit, R and rail.
    pub fn new(cfg: InertiaCfg, plan: DutyPlan, params: &RigParams) -> Self {
        let band = params.pos_guard.unwrap_or((150, 3950));
        Self {
            cfg,
            plan,
            params: *params,
            band,
            phase: Phase::ModeWrite,
            step: 0,
            last_pos: None,
            still: 0,
            polls: 0,
            start: None,
            halt: None,
            watch: None,
            i_run: (0.0, 0),
            base_governed: false,
            goal_q15: 0,
            capturing: false,
            cur: StepCapture::default(),
            captures: Vec::new(),
            warnings: Vec::new(),
        }
    }

    fn dir(&self) -> i8 {
        if self.step.is_multiple_of(2) { 1 } else { -1 }
    }

    fn base(&self) -> i16 {
        self.dir() as i16 * self.cfg.base_q15
    }

    fn steps(&self) -> usize {
        2 * self.cfg.step_fracs.len()
    }

    /// The step's goal once the base's running current is known: the one
    /// place a step is sized, with the shaft moving at the base.
    fn plan_step(&self, i_run: f64) -> i16 {
        let duty = self.cfg.step_duties(&self.plan, i_run)[self.step / 2];
        self.dir() as i16 * q15_floor(duty)
    }

    /// What the run noted, and every step the fit leaves out and why.
    /// Worth printing even when [`Inertia::fit`] comes back None: that is
    /// when they explain it.
    pub fn notes(&self) -> Vec<String> {
        let mut notes = self.warnings.clone();
        for c in &self.captures {
            if let Err(why) = self.series_of(c) {
                notes.push(format!(
                    "step to {:.1}%: {why}",
                    c.goal_q15 as f64 * 100.0 / 32767.0
                ));
            }
        }
        notes
    }

    fn halt_on(&mut self, reason: AbortReason) -> Cmd {
        self.halt = Some(reason);
        self.phase = Phase::TelMaskOff;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: 0,
        }
    }

    /// One base poll: the shaft must travel and the stall timer must not
    /// fold it; over the second half the running current is read and the
    /// applied duty checked against the base.
    fn base_eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let (eps, polls) = (self.cfg.stall_eps, self.cfg.stall_polls);
        let watch = self.watch.get_or_insert(Watch::new(o.pos, eps, polls));
        let start = watch.start();
        if o.limit_flags & LIMIT_YIELD_FOLDED != 0 || watch.still(o.pos) {
            return self.halt_on(seek::blocked(start, o.pos));
        }
        self.polls += 1;
        if self.polls > self.cfg.base_polls / 2 {
            self.i_run.0 += o.i_mean_counts as f64 * self.dir() as f64;
            self.i_run.1 += 1;
            self.base_governed |= o.duty_applied_q15 != self.base();
        }
        if self.polls < self.cfg.base_polls {
            self.phase = Phase::BaseRead;
            return Cmd::Pause {
                ms: self.cfg.base_poll_ms,
            };
        }
        if self.base_governed {
            self.warnings.push(format!(
                "base {:.1}%: {GOVERNED}",
                self.base() as f64 * 100.0 / 32767.0
            ));
            self.phase = Phase::StepOff;
            return Cmd::Pause { ms: 0 };
        }
        self.goal_q15 = self.plan_step(self.i_run.0 / self.i_run.1.max(1) as f64);
        self.phase = Phase::StepStream;
        Cmd::Pause { ms: 0 }
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
        self.start = None;
    }

    fn track_still(&mut self, pos: u16) {
        self.start.get_or_insert(pos);
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
    /// tick 0 at the step edge (the COMMIT); `mask` keeps the samples that
    /// applied the goal, window-valid and outside the slip zone, and the
    /// duty is the one the burst reports applied. A sample the limiter held
    /// under the goal declines the whole step: its transient is the
    /// limiter's, not the rotor's.
    fn series_of(&self, c: &StepCapture) -> Result<StepSeries, &'static str> {
        let tick_s = 1.0 / self.cfg.tick_hz;
        let frames: Vec<(&TelFrame, f64, i16, i16)> = c
            .tel
            .iter()
            .filter_map(|f| Some((f, f.counts()?, f.current?, f.duty_q15?)))
            .collect();
        let judged = judge(
            frames.iter().map(|(f, _, _, d)| (f.tick, *d)),
            c.goal_q15,
            c.base_q15,
        );
        if judged.contains(&Applied::Governed) {
            return Err(GOVERNED);
        }
        let mut s = StepSeries::default();
        for ((f, p, cur, duty), a) in frames.iter().zip(&judged) {
            let raw = f.pos.unwrap_or(p.round() as u16);
            let clean = *a == Applied::Clean;
            if clean && s.duty_q15 == 0.0 {
                s.duty_q15 = *duty as f64;
            }
            s.t.push(f.tick as f64 * tick_s);
            s.pos.push(*p);
            s.i.push(*cur as f64);
            s.mask
                .push(clean && f.window_valid && !self.params.in_slip(raw));
        }
        if s.mask.iter().filter(|m| **m).count() < 32 {
            return Err("too few samples, dropped");
        }
        Ok(s)
    }

    /// Every captured step reduced to its fit series, tagged TEL-sourced -
    /// the CLI records these for offline refits (recorded aggregate-tagged
    /// series from old runs refit the same way).
    pub fn step_series(&self) -> Vec<(StepSeries, bool)> {
        self.captures
            .iter()
            .filter_map(|c| self.series_of(c).ok().map(|s| (s, true)))
            .collect()
    }

    /// Assemble series and run both estimators; the smoothing half-window
    /// spans ~10 ms of ticks.
    pub fn fit(&self, priors: &InertiaPriors) -> Option<InertiaResult> {
        let series: Vec<StepSeries> = self
            .captures
            .iter()
            .filter_map(|c| self.series_of(c).ok())
            .collect();
        let warnings = self.notes();
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
                    value: self.params.pot.tel_mask(TEL_LADDER_MASK) as i32,
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
                    let start = self.start.unwrap_or(o.pos);
                    if !self.seek_done(o.pos)
                        && self.still >= self.cfg.stall_polls
                        && let Err(reason) =
                            seek::at_stop(start, o.pos, -self.dir(), self.params.stops)
                    {
                        return self.halt_on(reason);
                    }
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
                self.phase = Phase::BaseSet;
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::BaseSet => {
                self.reset_motion_track();
                self.watch = None;
                self.i_run = (0.0, 0);
                self.base_governed = false;
                self.phase = Phase::BaseWait;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.base() as i32,
                }
            }
            Phase::BaseWait => {
                self.phase = Phase::BaseRead;
                Cmd::Pause {
                    ms: self.cfg.base_poll_ms,
                }
            }
            Phase::BaseRead => {
                self.phase = Phase::BaseEval;
                Cmd::Read
            }
            Phase::BaseEval => match obs {
                Some(o) => self.base_eval(o),
                None => {
                    self.phase = Phase::BaseRead;
                    Cmd::Pause { ms: 0 }
                }
            },
            Phase::StepStream => {
                self.capturing = true;
                self.cur = StepCapture {
                    goal_q15: self.goal_q15,
                    base_q15: self.base(),
                    ..StepCapture::default()
                };
                self.phase = Phase::StepOff;
                Cmd::Stream {
                    samples: self.capture_samples(),
                    goal: Some((control::GOAL_DUTY, self.goal_q15 as i32)),
                }
            }
            Phase::StepOff => {
                if self.capturing {
                    self.close_step();
                }
                self.phase = Phase::StepRest;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::StepRest => {
                self.step += 1;
                self.phase = if self.step < self.steps() {
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

    fn halted(&self) -> Option<AbortReason> {
        self.halt
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
    use super::super::testkit::{FakeServo, bent_pot, pump};
    use super::super::{Guarded, RigParams};
    use super::*;
    use crate::pot::Pot;

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

    /// The fake's winding on its rail under a limit of `i_lim`: steps of
    /// about 12, 18 and 24% over the default 31% base.
    fn plan(i_lim: u16) -> DutyPlan {
        let lim = crate::limits::ServoLimits {
            i_lim,
            stall_yield: i_lim / 2,
            tau_trip: i_lim,
            soft: (200, 4000),
            phys: (200, 4000),
            raw: (200, 4000),
            r_q12: 0,
            vbus: 1731,
            i_floor_ticks: 160,
            amps_per_count: 0.0,
        };
        DutyPlan::new(&lim, 3.37, None)
    }

    fn run() -> (Inertia, Vec<String>) {
        let mut servo = dynamic_servo();
        let params = crate::exp::testkit::rig();
        let exp = Inertia::new(InertiaCfg::default(), plan(180), &params);
        let mut exp = Guarded::new(exp, params);
        let log = pump(&mut exp, &mut servo, 4_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    /// The goal of every step burst, in order.
    fn stream_goals(log: &[String]) -> Vec<i32> {
        log.iter()
            .filter_map(|l| l.strip_prefix("stream 3015 goal_duty "))
            .map(|v| v.parse().unwrap())
            .collect()
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
        // six step bursts, each a goal+arm commit over a base of its sign
        let streams: Vec<usize> = log
            .iter()
            .enumerate()
            .filter(|(_, l)| l.starts_with("stream "))
            .map(|(k, _)| k)
            .collect();
        assert_eq!(streams.len(), 6);
        for (n, k) in streams.iter().enumerate() {
            let base = if n % 2 == 0 { 10158 } else { -10158 };
            let last_write = log[..*k]
                .iter()
                .rfind(|l| l.starts_with("write goal_duty"))
                .unwrap();
            assert_eq!(*last_write, format!("write goal_duty {base}"));
        }
        let goals = stream_goals(&log);
        assert!(goals[0] > 10158 && goals[1] < -10158, "{goals:?}");
        // last duty write is the safety zero
        let last_duty = log
            .iter()
            .rev()
            .find(|l| l.starts_with("write goal_duty"))
            .unwrap();
        assert_eq!(last_duty, "write goal_duty 0");
    }

    /// With a table live the arm carries pos_lin and the series is the
    /// kernel's word, not the bent raw count: B comes back planted where
    /// the raw fit, its friction priors in the wrong counts, reads low.
    #[test]
    fn live_table_arms_pos_lin_and_fits_the_kernels_counts() {
        let table = bent_pot();
        let run = |pot: Pot| {
            let mut servo = dynamic_servo();
            servo.pot = Some(table);
            let params = RigParams {
                pot,
                ..crate::exp::testkit::rig()
            };
            let exp = Inertia::new(InertiaCfg::default(), plan(180), &params);
            let mut exp = Guarded::new(exp, params);
            let log = pump(&mut exp, &mut servo, 4_000_000);
            assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
            let exp = exp.into_inner();
            let b = exp.fit(&priors()).expect("fit").b_best;
            (log, exp.step_series(), b)
        };
        let (raw_log, raw, b_raw) = run(Pot::RAW);
        let (lin_log, lin, b_lin) = run(Pot::live(table));
        assert!(raw_log.contains(&"write tel_mask 27".to_string()));
        assert!(lin_log.contains(&"write tel_mask 2075".to_string()));
        assert!((b_raw - B_PLANT) / B_PLANT < -0.05, "raw b {b_raw}");
        assert!(
            (b_lin - B_PLANT).abs() / B_PLANT < 0.05,
            "linearized b {b_lin}"
        );
        assert_eq!(lin.len(), 6);
        // the seek coasts to rest near raw 500, which the bent pot reads
        // ~60 counts under the true position the live series starts from
        let (r0, l0) = (raw[0].0.pos[0], lin[0].0.pos[0]);
        assert!(l0 - r0 > 40.0, "raw {r0} vs linearized {l0}");
        assert!(
            (l0 - table.counts(r0 as u16)).abs() <= 0.5,
            "raw {r0} vs linearized {l0}"
        );
    }

    /// Each step leaves from the base, shaft moving: the burst opens on
    /// the base duty, not on zero, and the step over it is the plan's
    /// room over the running current the base drew. The limit governs the
    /// fake as the firmware would, and no step reaches it. The fake's
    /// motor time constant is 87 ms, so its base runs 500 ms to settle.
    #[test]
    fn inertia_steps_from_a_moving_base() {
        let mut servo = dynamic_servo();
        servo.current_limit = Some(180);
        let params = crate::exp::testkit::rig();
        let cfg = InertiaCfg {
            base_polls: 50,
            ..InertiaCfg::default()
        };
        let exp = Inertia::new(cfg, plan(180), &params);
        let mut exp = Guarded::new(exp, params);
        let log = pump(&mut exp, &mut servo, 4_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        let exp = exp.into_inner();
        // the base's running current: friction at its speed
        let w = (0.31 * 1731.0 - 3.37 * 20.0) / (0.1731 + 3.37 * 0.006);
        let steps = InertiaCfg::default().step_duties(&plan(180), 20.0 + 0.006 * w);
        let goals = stream_goals(&log);
        assert_eq!(goals.len(), 6);
        for (k, g) in goals.iter().enumerate() {
            let want = steps[k / 2] * 32767.0;
            assert!(
                (g.unsigned_abs() as f64 - want).abs() < 250.0,
                "step {k}: {g} for {want:.0}"
            );
            assert_eq!(g.signum(), if k % 2 == 0 { 1 } else { -1 });
        }
        for (c, g) in exp.captures.iter().zip(&goals) {
            let first = &c.tel[0];
            assert!(
                first.duty_q15.unwrap().unsigned_abs() >= 10158,
                "the step slewed from zero"
            );
            // 5 ms from the edge: under a tenth of a time constant
            let (p0, p1) = (c.tel[0].pos.unwrap(), c.tel[100].pos.unwrap());
            let moved = (p1 as f64 - p0 as f64) * g.signum() as f64;
            assert!(moved > 8.0, "the shaft was not moving at the edge: {moved}");
            assert!(c.tel.iter().any(|f| f.duty_q15 == Some(*g as i16)));
        }
        let fit = exp.fit(&priors()).expect("fit");
        assert_eq!(fit.tel_steps, 6, "{:?}", fit.warnings);
        assert!(
            (fit.b_best - B_PLANT).abs() / B_PLANT < 0.05,
            "{}",
            fit.b_best
        );
    }

    /// A step the limiter holds under its goal is declined, never fitted.
    /// Planned under 180 counts but run under 120, the steps' first edges
    /// draw about 96, 127 and 158: the smallest pair fits, the rest are
    /// governed.
    #[test]
    fn a_governed_step_is_declined() {
        let mut servo = dynamic_servo();
        servo.current_limit = Some(120);
        let params = crate::exp::testkit::rig();
        let cfg = InertiaCfg {
            base_polls: 50,
            ..InertiaCfg::default()
        };
        let exp = Inertia::new(cfg, plan(180), &params);
        let mut exp = Guarded::new(exp, params);
        pump(&mut exp, &mut servo, 4_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        let exp = exp.into_inner();
        assert_eq!(exp.step_series().len(), 2);
        let fit = exp.fit(&priors()).expect("the clean pair fits");
        let declined: Vec<&String> = fit
            .warnings
            .iter()
            .filter(|w| w.ends_with(GOVERNED))
            .collect();
        assert_eq!(declined.len(), 4, "{:?}", fit.warnings);
        assert!(declined[0].starts_with("step to 48."), "{declined:?}");
    }

    /// The base runs into a jam the limiter holds at the current limit;
    /// the stall timer folds it, and the run ends blocked there, before
    /// any step.
    #[test]
    fn a_yield_fold_on_the_base_ends_the_run_blocked() {
        let mut servo = dynamic_servo();
        servo.pos = 500.0;
        servo.jam = Some(600.0);
        servo.current_limit = Some(120);
        servo.stall_ms = Some(20.0);
        servo.stall_yield = 60;
        let params = crate::exp::testkit::rig();
        let cfg = InertiaCfg {
            base_polls: 20,
            ..InertiaCfg::default()
        };
        let mut exp = Guarded::new(Inertia::new(cfg, plan(180), &params), params);
        let log = pump(&mut exp, &mut servo, 4_000_000);
        assert!(
            matches!(exp.abort(), Some(AbortReason::Blocked { pos: 600, moved }) if moved > 90),
            "{:?}",
            exp.abort()
        );
        assert!(!log.iter().any(|l| l.starts_with("stream")), "{log:?}");
        assert_eq!(
            log.last().map(String::as_str),
            Some("write torque_enable 0")
        );
        assert!(!servo.torque);
    }

    #[test]
    fn frames_outside_a_burst_are_dropped() {
        let params = crate::exp::testkit::rig();
        let mut exp = Inertia::new(InertiaCfg::default(), plan(180), &params);
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
