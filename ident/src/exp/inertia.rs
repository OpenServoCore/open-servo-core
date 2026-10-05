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
//! Read/Pause. The base and every step run inside the [`Runway`]: a base or
//! step whose need does not fit is noted and skipped, and each drive -
//! seek, base, step - ends with the brake idiom, the seek at the start
//! band, the base at its brake point if it gets there, a step at the end
//! of its capture, which the fit check keeps short of the brake point. A
//! seek that comes to rest short of both its target and a stop ends the
//! run ([`super::seek::at_stop`]); so does a base that does not travel, or
//! that the firmware's stall timer folds.
//!
//! B comes from the current each step captures ([`fit_steps`]): after the
//! step the current falls as the back-EMF rises, a clean exponential whose
//! time constant is the rotor's, with the ladder's governed climbs as a
//! cross-check. The pot-speed estimators (direct-alpha and exponential-rise)
//! stay as diagnostics and decide nothing: the steps' speed sits on pot
//! noise of the same size. Too few decays, or a spread over
//! [`DECAY_SPREAD_MAX`], declines the fit in plain words. Its Ke and
//! friction come from the ladder, never from one that declined
//! ([`priors`]). The estimators live in [`crate::fits`].

use super::ladder::LadderResult;
use super::seek::Watch;
use super::{
    AbortReason, Applied, Cmd, Experiment, GOVERNED, LIMIT_YIELD_FOLDED, RigParams, judge, seek,
};
use crate::fits::{
    BClimb, BDecay, BDirect, BExp, Climb, InertiaPriors, StepSeries, b_climb_fit, b_decay_pool,
    b_decay_step, b_direct_fit, b_exp_fit,
};
use crate::frame::{TelBurst, TelFrame, TelemetrySnapshot};
use crate::limits::{DutyPlan, q15_floor};
use crate::regs::control;
use crate::runway::{
    BRAKE_DUTY_Q15, BRAKE_POLL_MS, BRAKE_POLLS, BRAKE_REST_EPS, Need, Runway, fits,
};

/// TEL frame layout the choreography arms: pos + current + duty + vdiff,
/// plus pos_lin while the rig's position table is live ([`crate::pot::Pot`]).
pub const TEL_LADDER_MASK: u16 = 0x1B;

/// The base over the seek duty, a fraction of full scale.
pub const BASE_OVER_SEEK: f64 = 0.05;

/// Steps with a current decay B needs.
pub const MIN_DECAY_STEPS: usize = 3;

/// Most the steps' B may spread, sd over mean, before the fit is not
/// trusted: on the bench servo they spread 4%.
pub const DECAY_SPREAD_MAX: f64 = 0.10;

/// How far the governed climbs' B may sit off the decay's before the run
/// says so: the bench servo's climbs spread 10%.
pub const CLIMB_AGREE: f64 = 0.15;

/// Ke and friction the servo carries from an earlier identification.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StoredMotion {
    pub ke_vpc: f64,
    pub fc: f64,
    pub fv: f64,
}

impl StoredMotion {
    /// From the calib fields as gain synthesis encodes them; None unless
    /// the servo carries a Ke.
    pub fn read(ke_vpc_q: u16, fric_fc_counts: u16, fric_fv_q016: u16) -> Option<Self> {
        (ke_vpc_q != 0).then(|| Self {
            ke_vpc: ke_vpc_q as f64 / 4096.0,
            fc: fric_fc_counts as f64,
            fv: fric_fv_q016 as f64 / 65536.0,
        })
    }
}

/// Where the inertia fit's Ke and friction came from.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PriorsFrom {
    Ladder,
    Stored,
}

impl PriorsFrom {
    pub fn as_str(self) -> &'static str {
        match self {
            PriorsFrom::Ladder => "the ladder",
            PriorsFrom::Stored => "the servo's stored Ke and friction",
        }
    }
}

/// The inertia fit's Ke and friction: the ladder's, when it fitted. A
/// ladder that declined hands on nothing, so then the ones the servo
/// carries from an earlier identification, else none, in plain words.
/// `r_loop_vpc` is the current loop's R: the back-EMF damping a step sees
/// is Ke over the incremental R.
pub fn priors(
    ladder: Option<&LadderResult>,
    stored: Option<StoredMotion>,
    r_loop_vpc: f64,
    tick_hz: f64,
) -> Result<(InertiaPriors, PriorsFrom), String> {
    let mean = |a: Option<f64>, b: Option<f64>| match (a, b) {
        (Some(a), Some(b)) => (a + b) / 2.0,
        (Some(a), None) | (None, Some(a)) => a,
        (None, None) => 0.0,
    };
    let (m, from) = match (ladder, stored) {
        (Some(l), _) => (
            StoredMotion {
                ke_vpc: l.ke.ke_vpc,
                fc: mean(l.fric_fwd.map(|f| f.fc), l.fric_rev.map(|f| f.fc)),
                fv: mean(l.fric_fwd.map(|f| f.fv), l.fric_rev.map(|f| f.fv)),
            },
            PriorsFrom::Ladder,
        ),
        (None, Some(s)) => (s, PriorsFrom::Stored),
        (None, None) => {
            return Err(
                "the ladder gave no Ke and friction line and the servo carries none from an \
                 earlier identification: B has nothing to be read against"
                    .into(),
            );
        }
    };
    Ok((
        InertiaPriors {
            r_vpc: r_loop_vpc,
            ke_vpc: m.ke_vpc,
            fc: m.fc,
            fv: m.fv,
            tick_hz,
        },
        from,
    ))
}

/// B from the steps' current decays, checked against the ladder's governed
/// `climbs`; the pot-speed estimators ride along as diagnostics. `series`
/// are the steps as [`Inertia::step_series`] tags them, TEL or aggregate.
/// Err, in plain words, when too few steps decay or their B spreads.
pub fn fit_steps(
    series: &[(StepSeries, bool)],
    climbs: &[Climb],
    priors: &InertiaPriors,
) -> Result<InertiaResult, String> {
    if series.is_empty() {
        return Err("no inertia step was captured to fit".into());
    }
    let tel_steps = series.iter().filter(|(_, tel)| *tel).count();
    let mut warnings = Vec::new();
    let mut decays = Vec::new();
    for (s, _) in series {
        match b_decay_step(s, priors) {
            Ok(d) => decays.push(d),
            Err(why) => warnings.push(format!(
                "step to {:+.1}%: {why}",
                s.duty_q15 * 100.0 / 32767.0
            )),
        }
    }
    if decays.len() < MIN_DECAY_STEPS {
        return Err(format!(
            "{} of the {} inertia steps show a current decay to fit, and B needs \
             {MIN_DECAY_STEPS}{}",
            decays.len(),
            series.len(),
            match warnings.as_slice() {
                [] => String::new(),
                w => format!(" ({})", w.join("; ")),
            }
        ));
    }
    let n = decays.len();
    let b_decay: BDecay = b_decay_pool(decays).ok_or("no step decay to pool")?;
    if b_decay.spread > DECAY_SPREAD_MAX {
        return Err(format!(
            "B spreads {:.0}% across the {n} inertia steps, over the {:.0}% one fit may: the fit is \
             not trusted",
            b_decay.spread * 100.0,
            DECAY_SPREAD_MAX * 100.0
        ));
    }
    let b_climb = b_climb_fit(climbs, priors);
    if let Some(c) = b_climb
        && (c.b / b_decay.b - 1.0).abs() > CLIMB_AGREE
    {
        warnings.push(format!(
            "the ladder's governed climbs read B {:.3}, {:.0}% off the current decay's {:.3}",
            c.b,
            (c.b / b_decay.b - 1.0).abs() * 100.0,
            b_decay.b
        ));
    }
    // the same smoothing window the pot-speed estimators always took
    let hw = if tel_steps > 0 {
        (0.010 * priors.tick_hz) as usize
    } else {
        12
    };
    let steps: Vec<StepSeries> = series.iter().map(|(s, _)| s.clone()).collect();
    Ok(InertiaResult {
        b_best: b_decay.b,
        j_ff: 1.0 / b_decay.b,
        b_decay,
        b_climb,
        b_direct: b_direct_fit(&steps, priors, hw, 5.0),
        b_exp: b_exp_fit(&steps, priors, hw),
        tel_steps,
        warnings,
    })
}

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
    pub seek_poll_ms: u32,
    pub rest_ms: u32,
    /// Step burst duration; with `tick_hz` this sizes the TEL arm.
    pub capture_ms: u32,
    pub stall_eps: u16,
    pub stall_polls: u32,
    pub seek_cap_polls: u32,
    /// The servo's fast tick rate, Hz, as it reports it: the TEL clock.
    pub tick_hz: f64,
}

impl InertiaCfg {
    /// The default steps on a servo ticking at `tick_hz`.
    pub fn new(tick_hz: f64) -> Self {
        Self {
            step_fracs: vec![0.5, 0.75, 1.0],
            seek_duty_q15: 8520,
            // 26% + 5%
            base_q15: 10158,
            // 100 ms: five motor time constants of the class
            base_polls: 10,
            base_poll_ms: 10,
            seek_poll_ms: 30,
            rest_ms: 300,
            // Sized to the runway, not the transient: after the base's 100 ms
            // the step travels under 1000 of the ~2900 counts from the seek
            // band to the far soft wall on the bench servo, either rail.
            capture_ms: 150,
            stall_eps: 3,
            stall_polls: 10,
            seek_cap_polls: 400,
            tick_hz,
        }
    }

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
    /// B from the current decay after each step.
    pub b_decay: BDecay,
    /// B from the ladder's governed climbs, when the run has them: the
    /// cross-check.
    pub b_climb: Option<BClimb>,
    /// The pot-speed estimators: diagnostics that decide nothing.
    pub b_direct: Option<BDirect>,
    pub b_exp: Option<BExp>,
    /// What the gains take: the decay's B.
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
    SeekRest,
    BaseSet,
    BaseWait,
    BaseRead,
    BaseEval,
    StepStream,
    StepOff,
    BrakeWait,
    BrakeRead,
    BrakeEval,
    StepRest,
    TelMaskOff,
    FinishTorque,
    Finished,
}

pub struct Inertia {
    cfg: InertiaCfg,
    plan: DutyPlan,
    runway: Runway,
    params: RigParams,
    phase: Phase,
    /// Step index: amplitude = step / 2, direction = +1 then -1.
    step: usize,
    last_pos: Option<u16>,
    /// When the last seek read was taken, host ms.
    last_ms: Option<f64>,
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
    /// Where the base brakes if it gets there.
    base_brake: f64,
    /// The brake in progress: the last position read, polls so far, and
    /// the phase after it.
    brake: Option<(u16, u32, Phase)>,
    capturing: bool,
    cur: StepCapture,
    captures: Vec<StepCapture>,
    warnings: Vec<String>,
}

impl Inertia {
    /// `plan` sizes the steps from the servo's limit, R and rail; `runway`
    /// holds them inside the travel.
    pub fn new(cfg: InertiaCfg, plan: DutyPlan, runway: Runway, params: &RigParams) -> Self {
        Self {
            cfg,
            plan,
            runway,
            params: *params,
            phase: Phase::ModeWrite,
            step: 0,
            last_pos: None,
            last_ms: None,
            still: 0,
            polls: 0,
            start: None,
            halt: None,
            watch: None,
            i_run: (0.0, 0),
            base_governed: false,
            goal_q15: 0,
            base_brake: 0.0,
            brake: None,
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
    /// place a step is sized, with the shaft moving at the base. Err, in
    /// plain words, when the base and the step together do not fit the
    /// runway.
    fn plan_step(&self, i_run: f64) -> Result<i16, String> {
        let duty = self.cfg.step_duties(&self.plan, i_run)[self.step / 2];
        let goal = self.dir() as i16 * q15_floor(duty);
        let base = self.base_need().map(|n| n.v * self.base_ms());
        self.fit_drive(goal, self.cfg.capture_ms as f64, base)
            .map(|()| goal)
    }

    fn base_ms(&self) -> f64 {
        (self.cfg.base_polls * self.cfg.base_poll_ms) as f64
    }

    fn base_need(&self) -> Option<Need> {
        let duty = self.cfg.base_q15 as f64 / 32767.0;
        self.runway.plan(self.dir(), duty, self.base_ms())
    }

    /// A drive from the start band reaching `duty_q15`'s speed and running
    /// `ms` at it, after `before` counts of other travel, fits the runway.
    /// A step is charged its whole climb from rest on top of the base's
    /// travel: an over-count, never an under-count.
    fn fit_drive(&self, duty_q15: i16, ms: f64, before: Option<f64>) -> Result<(), String> {
        let what = if duty_q15 == self.base() {
            "base"
        } else {
            "step to"
        };
        let pct = format!("{:+.1}%", duty_q15 as f64 * 100.0 / 32767.0);
        let room = self.runway.room();
        let need = self
            .runway
            .plan(self.dir(), duty_q15 as f64 / 32767.0, ms)
            .zip(before)
            .map(|(n, d)| Need {
                run: n.run + d,
                ..n
            });
        match need {
            Some(n) if fits(&n, room) => Ok(()),
            Some(n) => Err(format!(
                "the {what} {pct} needs {:.0} counts of travel and {room:.0} are free: skipped",
                n.total()
            )),
            None => Err(format!(
                "the {what} {pct} has no measured speed or stop to size it by: skipped"
            )),
        }
    }

    /// Brake a drive moving `motion`, then go on to `after`.
    fn brake(&mut self, motion: i8, pos: Option<u16>, after: Phase) -> Cmd {
        self.brake = Some((pos.unwrap_or(0), 0, after));
        self.phase = Phase::BrakeWait;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: -(motion as i32) * BRAKE_DUTY_Q15 as i32,
        }
    }

    fn brake_eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let Some((last, polls, after)) = self.brake.take() else {
            self.phase = Phase::StepRest;
            return Cmd::Pause { ms: 0 };
        };
        if o.pos.abs_diff(last) >= BRAKE_REST_EPS && polls + 1 < BRAKE_POLLS {
            self.brake = Some((o.pos, polls + 1, after));
            self.phase = Phase::BrakeWait;
            return Cmd::Pause { ms: 0 };
        }
        self.phase = after;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: 0,
        }
    }

    /// `pos` plus one more poll of travel has reached `at` driving `motion`.
    fn reached(&self, pos: u16, motion: i8, at: f64) -> bool {
        let lead = self.last_pos.map_or(0, |l| pos.abs_diff(l)) as f64;
        let ahead = pos as f64 + motion as f64 * lead;
        if motion > 0 { ahead >= at } else { ahead <= at }
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
        if self.reached(o.pos, self.dir(), self.base_brake) {
            self.warnings.push(format!(
                "base {:+.1}%: reached its brake point before its step, skipped",
                self.base() as f64 * 100.0 / 32767.0
            ));
            return self.brake(self.dir(), Some(o.pos), Phase::StepRest);
        }
        self.last_pos = Some(o.pos);
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
        match self.plan_step(self.i_run.0 / self.i_run.1.max(1) as f64) {
            Ok(goal) => {
                self.goal_q15 = goal;
                self.phase = Phase::StepStream;
            }
            Err(why) => {
                self.warnings.push(why);
                self.phase = Phase::StepOff;
            }
        }
        Cmd::Pause { ms: 0 }
    }

    fn capture_samples(&self) -> u16 {
        let n = self.cfg.capture_ms as f64 * self.cfg.tick_hz / 1000.0;
        n.min(u16::MAX as f64) as u16
    }

    /// The seek brakes early by the stop it will take, so it comes to rest
    /// in the start band. Its speed is timed on the host's clock.
    fn seek_done(&self, o: &TelemetrySnapshot) -> bool {
        let v = match (self.last_pos, self.last_ms) {
            (Some(p), Some(t)) if o.host_ms > t => p.abs_diff(o.pos) as f64 / (o.host_ms - t),
            _ => 0.0,
        };
        let stop = self.runway.stop(v).unwrap_or(0.0);
        let motion = -self.dir();
        let target = self.runway.start(self.dir()) as f64;
        self.reached(o.pos, motion, target - motion as f64 * stop)
    }

    fn reset_motion_track(&mut self) {
        self.last_pos = None;
        self.last_ms = None;
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

    /// B from the captured steps ([`fit_steps`]), the run's notes among its
    /// warnings; Err names why it declines, the notes with it.
    pub fn fit(&self, priors: &InertiaPriors, climbs: &[Climb]) -> Result<InertiaResult, String> {
        let notes = self.notes();
        match fit_steps(&self.step_series(), climbs, priors) {
            Ok(mut r) => {
                r.warnings.splice(0..0, notes);
                Ok(r)
            }
            Err(why) if notes.is_empty() => Err(why),
            Err(why) => Err(format!("{why} (notes: {})", notes.join("; "))),
        }
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
                    let arrived = self.seek_done(o);
                    self.track_still(o.pos);
                    self.last_ms = Some(o.host_ms);
                    done = arrived || self.still >= self.cfg.stall_polls;
                    let start = self.start.unwrap_or(o.pos);
                    if !arrived
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
                    self.brake(-self.dir(), obs.map(|o| o.pos), Phase::SeekRest)
                } else {
                    self.phase = Phase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
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
                if let Err(why) = self.fit_drive(self.base(), self.base_ms(), Some(0.0)) {
                    self.warnings.push(why);
                    self.phase = Phase::StepOff;
                    return Cmd::Pause { ms: 0 };
                }
                let stop = self.base_need().map_or(0.0, |n| n.stop);
                self.base_brake = self.runway.brake_at(self.dir(), stop);
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
                self.brake(self.dir(), self.last_pos, Phase::StepRest)
            }
            Phase::BrakeWait => {
                self.phase = Phase::BrakeRead;
                Cmd::Pause { ms: BRAKE_POLL_MS }
            }
            Phase::BrakeRead => {
                self.phase = Phase::BrakeEval;
                Cmd::Read
            }
            Phase::BrakeEval => match obs {
                Some(o) => self.brake_eval(o),
                None => {
                    self.phase = Phase::BrakeRead;
                    Cmd::Pause { ms: 0 }
                }
            },
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
    fn push_tel(&mut self, burst: &TelBurst) {
        if self.capturing {
            self.cur.tel.extend_from_slice(&burst.frames);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::ladder::{Ladder, LadderCfg};
    use super::super::testkit::{Bus, FakeServo, TICK_HZ, bench_mg90, bent_pot, pump, pump_on};
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
            tick_hz: TICK_HZ,
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
            window_floor_q15: 4356,
            window_v_floor_q15: 4356,
            amps_per_count: 0.0,
            drive_polarity: true,
        };
        DutyPlan::new(&lim, 3.37, None)
    }

    /// The fake's steady speed at `duty`, counts/ms.
    fn v_fake(duty: f64) -> f64 {
        (duty * 1731.0 - 3.37 * 20.0) / (0.1731 + 3.37 * 0.006) / 1000.0
    }

    /// The runway the ladder leaves the fake: its speeds at 26 and 64%,
    /// a braked stop of 500 counts from 64%, a climb of 60.
    fn runway(params: &RigParams) -> Runway {
        let mut r = Runway::new(params.pos_guard.unwrap()).with_accel(60.0);
        r.ran(0.26, v_fake(0.26));
        r.ran(0.64, v_fake(0.64));
        r.stopped(v_fake(0.64), 500.0);
        r
    }

    fn inertia(cfg: InertiaCfg, i_lim: u16, params: &RigParams) -> Inertia {
        Inertia::new(cfg, plan(i_lim), runway(params), params)
    }

    fn run() -> (Inertia, Vec<String>) {
        let mut servo = dynamic_servo();
        let params = crate::exp::testkit::rig();
        let exp = inertia(InertiaCfg::new(TICK_HZ), 180, &params);
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
        let fit = exp.fit(&priors(), &[]).expect("fit");
        assert_eq!(fit.tel_steps, 6, "every step captures one burst");
        assert_eq!(fit.b_decay.steps.len(), 6);
        assert_eq!(fit.b_best, fit.b_decay.b, "the decay decides");
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

    /// The honest fake, its pot read through 1.2 counts of noise, the
    /// steps' speed as noisy as the bench's: B comes from the current and
    /// lands within 3% of the planted one. On the bench servo after its
    /// ladder, the ladder's governed climbs read it to 10%.
    #[test]
    fn inertia_fits_the_current_decay() {
        let mut servo = dynamic_servo();
        servo.pos_noise = 1.2 * 12f64.sqrt();
        let params = crate::exp::testkit::rig();
        let mut exp = Guarded::new(inertia(InertiaCfg::new(TICK_HZ), 180, &params), params);
        pump(&mut exp, &mut servo, 4_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        let fit = exp.into_inner().fit(&priors(), &[]).expect("fit");
        assert_eq!(fit.b_decay.steps.len(), 6);
        assert!(
            (fit.b_best / B_PLANT - 1.0).abs() < 0.03,
            "b {} planted {B_PLANT}",
            fit.b_best
        );
        assert!(fit.b_decay.spread < 0.03, "{}", fit.b_decay.spread);

        let mut servo = bench_mg90(3204);
        servo.pos = 2029.0;
        let guard = (532, 3526);
        let params = RigParams::new(Some(guard), 350).with_stops((209, 3849));
        let cfg = LadderCfg {
            seek_duty_q15: 4915,
            ..LadderCfg::new(TICK_HZ)
        };
        let mut g = Guarded::new(
            Ladder::new(cfg, &params, Runway::new(guard)),
            params.abort_at_soft((432, 3626)),
        );
        pump(&mut g, &mut servo, 4_000_000);
        let ladder = g.into_inner();
        assert!(
            ladder.climbs().len() >= 4,
            "{} climbs",
            ladder.climbs().len()
        );
        let bench = InertiaPriors {
            r_vpc: servo.r,
            ke_vpc: servo.ke,
            fc: servo.fc,
            fv: servo.fv,
            tick_hz: TICK_HZ,
        };
        let c = crate::fits::b_climb_fit(ladder.climbs(), &bench).expect("a climb");
        assert!(
            (c.b / servo.b - 1.0).abs() < 0.10,
            "climbs read {c:?}, planted {}",
            servo.b
        );
    }

    /// A recorded bench run whose pot-speed fit reads B 0.406: its six step
    /// captures read through the current decay against the plant the bench
    /// servo carries. B is the bench value, 0.335 to 5%,
    /// the six steps within 10% of each other.
    #[test]
    fn inertia_on_the_recorded_run_reads_the_bench_value() {
        use std::io::BufRead;
        let file = std::fs::File::open(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/ident-mg90/inertia_tel.csv.gz"
        ))
        .unwrap();
        let mut bursts: Vec<Vec<TelFrame>> = Vec::new();
        for line in std::io::BufReader::new(flate2::read::GzDecoder::new(file))
            .lines()
            .skip(1)
        {
            let line = line.unwrap();
            let v: Vec<i64> = line.split(',').map(|x| x.parse().unwrap()).collect();
            let f = TelFrame {
                tick: v[0] as u64,
                window_valid: v[1] != 0,
                pos: Some(v[2] as u16),
                current: Some(v[3] as i16),
                duty_q15: Some(v[4] as i16),
                pos_lin: Some(v[5] as u16),
                ..Default::default()
            };
            if f.tick == 0 {
                bursts.push(Vec::new());
            }
            bursts.last_mut().unwrap().push(f);
        }
        assert_eq!(bursts.len(), 6);
        let lim = crate::limits::ServoLimits {
            i_lim: 280,
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 7270,
            vbus: 2917,
            window_floor_q15: 4356,
            window_v_floor_q15: 4356,
            amps_per_count: 0.0,
            drive_polarity: true,
        };
        let r = 7270.0 / 4096.0;
        let params = RigParams::new(Some((532, 3526)), 350).with_stops((209, 3849));
        let mut exp = Inertia::new(
            InertiaCfg::new(20_000.0),
            DutyPlan::new(&lim, r, None),
            Runway::new((532, 3526)),
            &params,
        );
        for tel in bursts {
            let duty = |f: &TelFrame| f.duty_q15.unwrap();
            exp.captures.push(StepCapture {
                goal_q15: duty(tel.last().unwrap()),
                base_q15: duty(&tel[0]),
                tel,
            });
        }
        let bench = InertiaPriors {
            r_vpc: r,
            ke_vpc: 0.14168,
            fc: 53.26,
            fv: 0.005130,
            tick_hz: 20_000.0,
        };
        let fit = exp.fit(&bench, &[]).expect("fit");
        println!(
            "B {:.4} spread {:.3} over {} steps; pot-speed exp-rise {:?}",
            fit.b_best,
            fit.b_decay.spread,
            fit.b_decay.steps.len(),
            fit.b_exp.as_ref().map(|e| (e.b, e.spread))
        );
        assert_eq!(fit.b_decay.steps.len(), 6);
        assert!((fit.b_best / 0.335 - 1.0).abs() < 0.05, "B {}", fit.b_best);
        assert!(fit.b_decay.spread < DECAY_SPREAD_MAX);
    }

    /// Steps whose decays read B 0.08 to 0.13: the spread is the fit's
    /// verdict on itself, and it declines in plain words rather than
    /// promote a mean. Two decays are too few whatever they read, and a
    /// step whose current rises is not one.
    #[test]
    fn a_scattered_inertia_is_declined_not_promoted() {
        let p = priors();
        let step = |duty: f64, b: f64| {
            let tau = 1.0 / (b * p.f_med() * (p.fv + p.ke_vpc / p.r_vpc));
            let t: Vec<f64> = (0..3000).map(|k| k as f64 / TICK_HZ).collect();
            let i = t
                .iter()
                .map(|t| duty.signum() * (60.0 + 90.0 * (-t / tau).exp()))
                .collect();
            let s = StepSeries {
                mask: vec![true; t.len()],
                pos: vec![2000.0; t.len()],
                t,
                i,
                duty_q15: duty,
            };
            (s, true)
        };
        let scattered: Vec<(StepSeries, bool)> = [0.08, 0.13, 0.10, 0.12, 0.09, 0.11]
            .iter()
            .enumerate()
            .map(|(k, b)| step(if k % 2 == 0 { 9000.0 } else { -9000.0 }, *b))
            .collect();
        let why = fit_steps(&scattered, &[], &p).unwrap_err();
        assert_eq!(
            why,
            "B spreads 18% across the 6 inertia steps, over the 10% one fit may: the fit is not \
             trusted"
        );
        let tight: Vec<(StepSeries, bool)> = [0.100, 0.102, 0.098]
            .iter()
            .map(|b| step(9000.0, *b))
            .collect();
        let fit = fit_steps(&tight, &[], &p).expect("a tight set");
        assert!((fit.b_best / 0.1 - 1.0).abs() < 0.01, "{}", fit.b_best);

        let mut rising = step(9000.0, 0.1);
        for i in rising.0.i.iter_mut() {
            *i = 210.0 - *i;
        }
        let why = fit_steps(&[step(9000.0, 0.1), step(-9000.0, 0.1), rising], &[], &p).unwrap_err();
        assert_eq!(
            why,
            "2 of the 3 inertia steps show a current decay to fit, and B needs 3 (step to \
             +27.5%: its current does not fall after the step)"
        );
    }

    /// A ladder its servo's back-EMF speed disagrees with declines, and
    /// inertia takes none of it: its B reads against the Ke and friction
    /// the servo carries, and without them it has nothing to read against.
    #[test]
    fn inertia_never_inherits_a_declined_ladder() {
        let mut servo = FakeServo::new(3.37);
        servo.physical_motion = true;
        servo.fv = 0.006;
        servo.ke_stored = Some(0.1731 * 1.15);
        let params = RigParams::new(Some((250, 3950)), 1100).with_stops((200, 4000));
        let cfg = LadderCfg {
            servo_ke: true,
            ..LadderCfg::new(TICK_HZ)
        };
        let mut g = Guarded::new(
            Ladder::new(cfg, &params, Runway::new((250, 3950))),
            params.abort_at_soft((150, 4050)),
        );
        pump(&mut g, &mut servo, 2_000_000);
        let ladder = g.into_inner();
        assert!(ladder.declined().is_some());
        let accepted = ladder
            .declined()
            .is_none()
            .then(|| ladder.fit(3.37))
            .flatten();
        let stored = StoredMotion::read(810, 58, 336);
        let (p, from) = super::priors(accepted.as_ref(), stored, 3.37, TICK_HZ).unwrap();
        assert_eq!(from, PriorsFrom::Stored);
        assert_eq!(
            (p.ke_vpc, p.fc, p.fv),
            (810.0 / 4096.0, 58.0, 336.0 / 65536.0)
        );
        let fitted = ladder.fit(3.37).unwrap();
        assert!(
            (fitted.ke.ke_vpc - p.ke_vpc).abs() > 0.01,
            "it would have fitted"
        );
        assert_eq!(
            super::priors(accepted.as_ref(), None, 3.37, TICK_HZ).unwrap_err(),
            "the ladder gave no Ke and friction line and the servo carries none from an earlier \
             identification: B has nothing to be read against"
        );
        let (p, from) = super::priors(Some(&fitted), stored, 3.37, TICK_HZ).unwrap();
        assert_eq!((from, p.ke_vpc), (PriorsFrom::Ladder, fitted.ke.ke_vpc));
        assert_eq!(StoredMotion::read(0, 58, 336), None);
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
    /// kernel's word, not the bent raw count.
    /// The current decay never sees the pot: it reads B planted either
    /// way. The pot-speed diagnostic, its friction priors in the wrong
    /// counts, reads low on the raw count.
    #[test]
    fn live_table_arms_pos_lin_and_fits_the_kernels_counts() {
        let table = bent_pot();
        let run = |pot: Pot| {
            let mut servo = dynamic_servo();
            servo.pot = Some(table);
            // start bands at 500 and 3600, where the bent pot is bent
            let params = RigParams {
                pot,
                pos_guard: Some((425, 3675)),
                ..crate::exp::testkit::rig()
            };
            let exp = inertia(InertiaCfg::new(TICK_HZ), 180, &params);
            let mut exp = Guarded::new(exp, params);
            let log = pump(&mut exp, &mut servo, 4_000_000);
            assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
            let exp = exp.into_inner();
            let fit = exp.fit(&priors(), &[]).expect("fit");
            let b = (fit.b_best, fit.b_exp.expect("exp-rise").b);
            (log, exp.step_series(), b)
        };
        let (raw_log, raw, (b_raw, exp_raw)) = run(Pot::RAW);
        let (lin_log, lin, (b_lin, exp_lin)) = run(Pot::live(table));
        assert!(raw_log.contains(&"write tel_mask 27".to_string()));
        assert!(lin_log.contains(&"write tel_mask 2075".to_string()));
        for b in [b_raw, b_lin] {
            assert!((b - B_PLANT).abs() / B_PLANT < 0.03, "decay b {b}");
        }
        assert!(
            (exp_raw - B_PLANT) / B_PLANT < -0.05,
            "raw exp-rise b {exp_raw}"
        );
        assert!(
            (exp_lin - B_PLANT).abs() / B_PLANT < 0.05,
            "linearized exp-rise b {exp_lin}"
        );
        assert_eq!(lin.len(), 6);
        // the seek brakes to rest near raw 500, which the bent pot reads
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
        // a start band at 850
        let params = RigParams {
            pos_guard: Some((775, 3950)),
            ..crate::exp::testkit::rig()
        };
        let cfg = InertiaCfg {
            base_polls: 50,
            ..InertiaCfg::new(TICK_HZ)
        };
        let exp = inertia(cfg, 180, &params);
        let mut exp = Guarded::new(exp, params);
        let log = pump(&mut exp, &mut servo, 4_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        let exp = exp.into_inner();
        // the base's running current: friction at its speed
        let w = (0.31 * 1731.0 - 3.37 * 20.0) / (0.1731 + 3.37 * 0.006);
        let steps = InertiaCfg::new(TICK_HZ).step_duties(&plan(180), 20.0 + 0.006 * w);
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
        let fit = exp.fit(&priors(), &[]).expect("fit");
        assert_eq!(fit.tel_steps, 6, "{:?}", fit.warnings);
        assert!(
            (fit.b_best - B_PLANT).abs() / B_PLANT < 0.05,
            "{}",
            fit.b_best
        );
    }

    /// A step the limiter holds under its goal is declined, never fitted.
    /// Planned under 180 counts but run under 120, the steps' first edges
    /// draw about 96, 127 and 158: the smallest pair is clean, the rest are
    /// governed, and two steps are too few for B: inertia declines.
    #[test]
    fn a_governed_step_is_declined() {
        let mut servo = dynamic_servo();
        servo.current_limit = Some(120);
        let params = crate::exp::testkit::rig();
        let cfg = InertiaCfg {
            base_polls: 50,
            ..InertiaCfg::new(TICK_HZ)
        };
        let exp = inertia(cfg, 180, &params);
        let mut exp = Guarded::new(exp, params);
        pump(&mut exp, &mut servo, 4_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        let exp = exp.into_inner();
        assert_eq!(exp.step_series().len(), 2);
        let why = exp.fit(&priors(), &[]).unwrap_err();
        assert!(
            why.starts_with(
                "2 of the 2 inertia steps show a current decay to fit, and B needs 3 (notes: step \
                 to 48."
            ),
            "{why}"
        );
        let notes = exp.notes();
        let declined: Vec<&String> = notes.iter().filter(|w| w.ends_with(GOVERNED)).collect();
        assert_eq!(declined.len(), 4, "{notes:?}");
        assert!(declined[0].starts_with("step to 48."), "{declined:?}");
    }

    /// The base runs into a jam the limiter holds at the current limit;
    /// the stall timer folds it, and the run ends blocked there, before
    /// any step.
    #[test]
    fn a_yield_fold_on_the_base_ends_the_run_blocked() {
        let mut servo = dynamic_servo();
        servo.pos = 250.0;
        servo.jam = Some(350.0);
        servo.current_limit = Some(120);
        servo.stall_ms = Some(20.0);
        servo.stall_yield = 60;
        let params = crate::exp::testkit::rig();
        let cfg = InertiaCfg {
            base_polls: 20,
            ..InertiaCfg::new(TICK_HZ)
        };
        let mut exp = Guarded::new(inertia(cfg, 180, &params), params);
        let log = pump(&mut exp, &mut servo, 4_000_000);
        assert!(
            matches!(exp.abort(), Some(AbortReason::Blocked { pos, moved })
                if pos.abs_diff(350) <= 2 && moved > 90),
            "{:?}",
            exp.abort()
        );
        assert!(!log.iter().any(|l| l.starts_with("stream")), "{log:?}");
        assert_eq!(
            log[log.len() - 2..],
            ["write torque_enable 0", "write ident_agg 0"]
        );
        assert!(!servo.torque);
    }

    fn brakes(log: &[String]) -> usize {
        log.iter()
            .filter(|l| {
                l.strip_prefix("write goal_duty ")
                    .and_then(|v| v.parse::<i32>().ok())
                    .is_some_and(|v| v.unsigned_abs() == BRAKE_DUTY_Q15 as u32)
            })
            .count()
    }

    /// Every base and step is sized inside the runway before it runs, and
    /// every drive ends braked: each seek at the start band, each step at
    /// the end of its capture, short of the step's brake point.
    #[test]
    fn inertia_base_and_steps_fit_the_runway() {
        let mut servo = dynamic_servo();
        let params = crate::exp::testkit::rig();
        let mut g = Guarded::new(inertia(InertiaCfg::new(TICK_HZ), 180, &params), params);
        let log = pump(&mut g, &mut servo, 4_000_000);
        assert_eq!(g.abort(), None);
        let exp = g.into_inner();
        assert!(exp.warnings.is_empty(), "{:?}", exp.warnings);
        assert_eq!(stream_goals(&log).len(), 6);
        assert_eq!(brakes(&log), 12, "six seeks and six steps");
        for c in &exp.captures {
            let d = c.goal_q15.signum() as i8;
            let v = exp.runway.speed(d, c.goal_q15 as f64 / 32767.0).unwrap();
            let end = exp.runway.brake_at(d, exp.runway.stop(v).unwrap());
            let far = c
                .tel
                .iter()
                .filter_map(|f| f.pos)
                .map(|p| d as f64 * p as f64)
                .fold(f64::MIN, f64::max);
            assert!(far <= d as f64 * end, "{}: to {far} past {end}", c.goal_q15);
        }
        assert!(!servo.torque);
    }

    /// On a short travel the base and the smallest step fit and the two
    /// larger steps do not: those are noted in plain words and skipped,
    /// the base braked, and the run goes on.
    #[test]
    fn an_inertia_step_that_does_not_fit_is_skipped() {
        let mut servo = dynamic_servo();
        servo.pos = 900.0;
        let params = RigParams {
            pos_guard: Some((150, 1425)),
            ..crate::exp::testkit::rig()
        };
        let mut g = Guarded::new(inertia(InertiaCfg::new(TICK_HZ), 180, &params), params);
        let log = pump(&mut g, &mut servo, 4_000_000);
        assert_eq!(g.abort(), None);
        let exp = g.into_inner();
        let goals = stream_goals(&log);
        assert_eq!(goals.len(), 2, "{:?}", exp.warnings);
        assert_eq!(exp.warnings.len(), 4, "{:?}", exp.warnings);
        for w in &exp.warnings {
            assert!(
                w.starts_with("the step to ") && w.ends_with("and 1200 are free: skipped"),
                "{w}"
            );
        }
        assert_eq!(brakes(&log), 12, "a skipped step's base brakes too");
        assert!(!servo.torque);
    }

    #[test]
    fn frames_outside_a_burst_are_dropped() {
        let params = crate::exp::testkit::rig();
        let mut exp = inertia(InertiaCfg::new(TICK_HZ), 180, &params);
        exp.push_tel(&TelBurst {
            frames: vec![TelFrame {
                tick: 0,
                pos: Some(2000),
                current: Some(10),
                ..Default::default()
            }],
            rows_dropped: 0,
        });
        assert!(exp.captures.is_empty());
        assert!(exp.cur.tel.is_empty(), "not capturing: frame dropped");
    }

    /// Every read and TEL sample's raw position an experiment met.
    struct Positions<E> {
        exp: E,
        seen: Vec<u16>,
    }

    impl<E: Experiment> Experiment for Positions<E> {
        fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
            self.seen.extend(obs.map(|o| o.pos));
            self.exp.step(obs)
        }

        fn push_tel(&mut self, burst: &TelBurst) {
            self.seen.extend(burst.frames.iter().filter_map(|f| f.pos));
            self.exp.push_tel(burst)
        }

        fn halted(&self) -> Option<AbortReason> {
            self.exp.halted()
        }
    }

    /// Inertia on the bench servo after the ladder that sized its runway,
    /// at the bench bus's cadence with a seeded slow read: a drive brakes
    /// late and comes to rest past the inset guard it planned on, inside
    /// the soft limits. That is no abort - inertia aborts on the soft
    /// limits, as the ladder does - and every step is captured. Aborting on
    /// the guard, the same run would have ended there.
    #[test]
    fn a_late_inertia_brake_is_not_an_abort() {
        const GUARD: (u16, u16) = (532, 3526);
        let lim = crate::limits::ServoLimits {
            i_lim: 280,
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 7270,
            vbus: 3204,
            window_floor_q15: 4356,
            window_v_floor_q15: 4356,
            amps_per_count: 0.0,
            drive_polarity: true,
        };
        let plan = DutyPlan::new(&lim, 7270.0 / 4096.0, Some(0.145));
        let params = RigParams::new(Some(GUARD), 350).with_stops((209, 3849));
        let bus = Bus::BENCH.with_slow_reads(1);
        let run = |aborts: RigParams| {
            let mut servo = bench_mg90(3204);
            servo.pos = 2029.0;
            let cfg = LadderCfg {
                seek_duty_q15: q15_floor(plan.seek),
                ..LadderCfg::new(TICK_HZ)
            };
            let ladder = Ladder::new(cfg, &params, Runway::new(GUARD));
            let mut g = Guarded::new(ladder, params.abort_at_soft(lim.soft));
            pump_on(&mut g, &mut servo, 4_000_000, bus);
            assert_eq!(g.abort(), None);
            let runway = g.into_inner().runway().clone();
            let base = plan.seek + BASE_OVER_SEEK;
            let cfg = crate::run::inertia_cfg(plan.seek, base, InertiaCfg::new(TICK_HZ));
            let mut s = Positions {
                exp: Guarded::new(Inertia::new(cfg, plan, runway, &params), aborts),
                seen: Vec::new(),
            };
            pump_on(&mut s, &mut servo, 4_000_000, bus);
            assert!(!servo.torque);
            s
        };
        let s = run(params.abort_at_soft(lim.soft));
        assert_eq!(s.exp.abort(), None);
        let past: Vec<u16> = s
            .seen
            .iter()
            .copied()
            .filter(|p| !(GUARD.0..=GUARD.1).contains(p))
            .collect();
        assert!(!past.is_empty(), "nothing landed past the guard");
        assert!(past.iter().all(|p| (432..=3626).contains(p)), "{past:?}");
        assert_eq!(s.exp.into_inner().captures.len(), 6);

        let s = run(params);
        assert!(matches!(s.exp.abort(), Some(AbortReason::PosGuard { .. })));
    }
}
