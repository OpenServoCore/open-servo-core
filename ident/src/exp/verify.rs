//! E5/E6 closed-loop verification, run AFTER the fitted gains are written:
//! Current-mode steps against an end-stop (the loop must settle onto the
//! goal) and Velocity-mode travel legs (the pot slope must track the
//! goal). Both report per-step/leg errors and an overall pass verdict -
//! the check that the synthesized bandwidths hold on the real plant.
//!
//! Both are planned like the run plans its drives ([`DutyPlan`]): every
//! seek drives at the plan's seek duty, whose stall the current limit
//! holds, and is never raised - a seek that comes to rest anywhere but
//! where it was going ends the check. The current steps are stall
//! currents between the current sensor's window floor and the limit, held
//! with the stall permit; a supply that leaves no room between the two is
//! refused before anything moves.

use super::{AbortReason, Cmd, Experiment, RigParams, WindowSample, WindowStream, seek};
use crate::fitmath::linear_ls;
use crate::frame::TelemetrySnapshot;
use crate::limits::{DutyPlan, Refusal, q15_floor};
use crate::regs::control;

/// Verify current, as its refusal names it.
pub const VERIFY_CURRENT: &str = "verify current";

/// E5: goal_current steps into a stall, each direction at its stop.
pub struct VerifyCurrentCfg {
    /// Seek drive toward each end, q15, OpenLoop.
    pub seek_duty_q15: i16,
    /// Step amplitudes, ccounts, applied with the stall direction's sign.
    pub steps_counts: Vec<i16>,
    pub dwell_polls: u32,
    pub poll_ms: u32,
    pub rest_ms: u32,
    pub stall_eps: u16,
    pub stall_polls: u32,
    pub seek_poll_ms: u32,
    /// Settle band around the goal, fraction.
    pub tol: f64,
}

impl VerifyCurrentCfg {
    /// Seeks at `plan`'s seek duty; steps to the stall currents of the
    /// dwells a ladder from the window floor, `floor_q15`, to the
    /// stall-safe cap takes ([`DutyPlan::stall_ladder`]), its ends
    /// dropped: at the floor the shunt reads only some windows, at the
    /// limit the derate may hold the loop under its goal.
    pub fn planned(plan: &DutyPlan, floor_q15: i16) -> Result<Self, Refusal> {
        let rungs = plan.stall_ladder(VERIFY_CURRENT, floor_q15)?;
        let steps_counts = rungs[1..rungs.len() - 1]
            .iter()
            .map(|d| plan.stall(*d).floor() as i16)
            .collect();
        Ok(Self {
            seek_duty_q15: q15_floor(plan.seek),
            steps_counts,
            dwell_polls: 250,
            poll_ms: 2,
            rest_ms: 300,
            stall_eps: 3,
            stall_polls: 8,
            seek_poll_ms: 30,
            tol: 0.10,
        })
    }
}

#[derive(Clone, Debug)]
pub struct CurrentStep {
    /// Signed goal, ccounts.
    pub goal: i16,
    /// Mean window current over the dwell tail, ccounts.
    pub mean_i: f64,
    pub err_pct: f64,
    /// First in-band window relative to the step, ms; None = never settled.
    pub settle_ms: Option<f64>,
    pub windows: usize,
    pub pass: bool,
}

#[derive(Clone, Debug)]
pub struct VerifyCurrentResult {
    pub steps: Vec<CurrentStep>,
    pub pass: bool,
    pub warnings: Vec<String>,
}

enum CPhase {
    ModeOpen,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    SeekOff,
    SwitchTorqueOff,
    SwitchMode,
    SwitchTorqueOn,
    StepSet,
    StepRead,
    StepEval,
    StepOff,
    StepRest,
    DirTorqueOff,
    DirModeOpen,
    FinishGoal,
    FinishTorque,
    Finished,
}

/// One step's raw windows; settle time comes from the unfiltered stream,
/// the mean from the tail past the settle discard.
struct StepCapture {
    goal: i16,
    t0: Option<f64>,
    windows: Vec<WindowSample>,
}

pub struct VerifyCurrent {
    cfg: VerifyCurrentCfg,
    stops: Option<(u16, u16)>,
    seek_start: Option<u16>,
    halt: Option<AbortReason>,
    settle_windows: u32,
    phase: CPhase,
    dir_idx: u8,
    step_idx: usize,
    polls_left: u32,
    last_pos: Option<u16>,
    still: u32,
    /// settle_windows = 0: the capture keeps every window so settle time is
    /// measurable; the mean discards the head manually.
    windows: WindowStream,
    cur: Option<StepCapture>,
    captures: Vec<StepCapture>,
    warnings: Vec<String>,
}

impl VerifyCurrent {
    pub fn new(cfg: VerifyCurrentCfg, params: &RigParams) -> Self {
        let mut raw = *params;
        raw.settle_windows = 0;
        Self {
            cfg,
            stops: params.stops,
            seek_start: None,
            halt: None,
            settle_windows: params.settle_windows,
            phase: CPhase::ModeOpen,
            dir_idx: 0,
            step_idx: 0,
            polls_left: 0,
            last_pos: None,
            still: 0,
            windows: WindowStream::new(&raw),
            cur: None,
            captures: Vec::new(),
            warnings: Vec::new(),
        }
    }

    fn dir(&self) -> i8 {
        if self.dir_idx == 0 { 1 } else { -1 }
    }

    pub fn result(&self) -> VerifyCurrentResult {
        let mut steps = Vec::new();
        for c in &self.captures {
            let goal = c.goal as f64;
            let tail: Vec<&WindowSample> = c
                .windows
                .iter()
                .skip(self.settle_windows as usize)
                .collect();
            if tail.is_empty() {
                steps.push(CurrentStep {
                    goal: c.goal,
                    mean_i: 0.0,
                    err_pct: 100.0,
                    settle_ms: None,
                    windows: 0,
                    pass: false,
                });
                continue;
            }
            let mean_i = tail.iter().map(|w| w.i).sum::<f64>() / tail.len() as f64;
            let err_pct = (mean_i - goal).abs() / goal.abs() * 100.0;
            let settle_ms = c.t0.and_then(|t0| {
                c.windows
                    .iter()
                    .find(|w| (w.i - goal).abs() <= self.cfg.tol * goal.abs())
                    .map(|w| w.t_ms - t0)
            });
            steps.push(CurrentStep {
                goal: c.goal,
                mean_i,
                err_pct,
                settle_ms,
                windows: tail.len(),
                pass: err_pct <= self.cfg.tol * 100.0,
            });
        }
        let pass = !steps.is_empty() && steps.iter().all(|s| s.pass);
        VerifyCurrentResult {
            steps,
            pass,
            warnings: self.warnings.clone(),
        }
    }
}

impl Experiment for VerifyCurrent {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            CPhase::ModeOpen => {
                self.phase = CPhase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            CPhase::TorqueOn => {
                self.phase = CPhase::SeekSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            CPhase::SeekSet => {
                self.phase = CPhase::SeekRead;
                self.last_pos = None;
                self.seek_start = None;
                self.still = 0;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.dir() as i32 * self.cfg.seek_duty_q15 as i32,
                }
            }
            CPhase::SeekRead => {
                self.phase = CPhase::SeekEval;
                Cmd::Read
            }
            CPhase::SeekEval => {
                if let Some(o) = obs {
                    if let Some(last) = self.last_pos
                        && o.pos.abs_diff(last) <= self.cfg.stall_eps
                    {
                        self.still += 1;
                    } else {
                        self.still = 0;
                    }
                    self.last_pos = Some(o.pos);
                    let start = *self.seek_start.get_or_insert(o.pos);
                    if self.still >= self.cfg.stall_polls
                        && let Err(reason) = seek::at_stop(start, o.pos, self.dir(), self.stops)
                    {
                        self.halt = Some(reason);
                        self.phase = CPhase::FinishGoal;
                        return Cmd::Pause { ms: 0 };
                    }
                }
                if self.still >= self.cfg.stall_polls {
                    self.phase = CPhase::SeekOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = CPhase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            CPhase::SeekOff => {
                self.phase = CPhase::SwitchTorqueOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            // mode changes ride a torque-off gap: the enable edge reseeds
            // the loop chain cleanly (kernel run-edge contract)
            CPhase::SwitchTorqueOff => {
                self.phase = CPhase::SwitchMode;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            CPhase::SwitchMode => {
                self.phase = CPhase::SwitchTorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 1,
                }
            }
            CPhase::SwitchTorqueOn => {
                self.step_idx = 0;
                self.phase = CPhase::StepSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            CPhase::StepSet => {
                let goal = self.dir() as i32 * self.cfg.steps_counts[self.step_idx] as i32;
                self.polls_left = self.cfg.dwell_polls;
                self.windows.mark_transition();
                self.cur = Some(StepCapture {
                    goal: goal as i16,
                    t0: None,
                    windows: Vec::new(),
                });
                self.phase = CPhase::StepRead;
                Cmd::Write {
                    reg: control::GOAL_CURRENT,
                    value: goal,
                }
            }
            CPhase::StepRead => {
                self.phase = CPhase::StepEval;
                Cmd::Read
            }
            CPhase::StepEval => {
                if let Some(o) = obs
                    && let Some(w) = self.windows.push(o)
                    && let Some(c) = self.cur.as_mut()
                {
                    c.t0.get_or_insert(w.t_ms);
                    c.windows.push(w);
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if self.polls_left == 0 {
                    self.phase = CPhase::StepOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = CPhase::StepRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            CPhase::StepOff => {
                if let Some(c) = self.cur.take() {
                    self.captures.push(c);
                }
                self.phase = CPhase::StepRest;
                Cmd::Write {
                    reg: control::GOAL_CURRENT,
                    value: 0,
                }
            }
            CPhase::StepRest => {
                self.step_idx += 1;
                if self.step_idx < self.cfg.steps_counts.len() {
                    self.phase = CPhase::StepSet;
                } else if self.dir_idx == 0 {
                    self.dir_idx = 1;
                    self.phase = CPhase::DirTorqueOff;
                } else {
                    self.phase = CPhase::FinishGoal;
                }
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            CPhase::DirTorqueOff => {
                self.phase = CPhase::DirModeOpen;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            CPhase::DirModeOpen => {
                self.phase = CPhase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            CPhase::FinishGoal => {
                self.phase = CPhase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_CURRENT,
                    value: 0,
                }
            }
            CPhase::FinishTorque => {
                self.phase = CPhase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            CPhase::Finished => Cmd::Done,
        }
    }

    fn halted(&self) -> Option<AbortReason> {
        self.halt
    }
}

/// E6: Velocity-mode legs across the travel; the pot slope vs goal is the
/// tracking check. Legs alternate direction so each ends where the next
/// starts.
pub struct VerifyVelocityCfg {
    pub seek_duty_q15: i16,
    /// Leg speeds, c/s; keep under the table velocity limit.
    pub legs_cps: Vec<i32>,
    pub poll_ms: u32,
    pub rest_ms: u32,
    pub seek_poll_ms: u32,
    /// Travel margin off each guard edge where a leg turns around. Wide
    /// enough that goal-0 braking on unproven gains stays inside the
    /// guard through the rest pause (no envelope read until the next leg).
    pub margin: u16,
    /// Park distance off the low guard edge for the open-loop seek.
    /// Separate from `margin`: the seek cannot brake, and the coast after
    /// seek-off runs through the mode-switch writes with no envelope read
    /// in between (bench: a 26% seek parked at margin 350 coasted into
    /// the low rail and the first leg read aborted at pos 8).
    pub seek_margin: u16,
    /// Head trim before the slope fit (trajectory accel ramp), ms.
    pub accel_trim_ms: f64,
    pub tol: f64,
    /// Give-up cap per leg (stall protection), polls.
    pub leg_cap_polls: u32,
}

impl VerifyVelocityCfg {
    /// Parks for the first leg at `plan`'s seek duty.
    pub fn planned(plan: &DutyPlan) -> Self {
        Self {
            seek_duty_q15: q15_floor(plan.seek),
            legs_cps: vec![600, 1200],
            poll_ms: 4,
            rest_ms: 300,
            seek_poll_ms: 30,
            margin: 550,
            seek_margin: 1300,
            accel_trim_ms: 250.0,
            tol: 0.15,
            leg_cap_polls: 4000,
        }
    }
}

#[derive(Clone, Debug)]
pub struct VelocityLeg {
    pub goal_cps: i32,
    pub meas_cps: f64,
    pub err_pct: f64,
    pub r2: f64,
    pub n: usize,
    pub pass: bool,
}

#[derive(Clone, Debug)]
pub struct VerifyVelocityResult {
    pub legs: Vec<VelocityLeg>,
    pub pass: bool,
    pub warnings: Vec<String>,
}

enum VPhase {
    ModeOpen,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    SeekOff,
    SwitchTorqueOff,
    SwitchMode,
    SwitchTorqueOn,
    LegSet,
    LegRead,
    LegEval,
    LegOff,
    LegRest,
    FinishGoal,
    FinishTorque,
    Finished,
}

struct LegCapture {
    goal_cps: i32,
    /// (t_s on the host's clock, pos) pairs, slip-zone samples excluded:
    /// the servo's tick counter runs slow under polling.
    pts: Vec<(f64, f64)>,
}

pub struct VerifyVelocity {
    cfg: VerifyVelocityCfg,
    params: RigParams,
    band: (u16, u16),
    phase: VPhase,
    leg_idx: usize,
    polls: u32,
    last_pos: Option<u16>,
    still: u32,
    seek_start: Option<u16>,
    halt: Option<AbortReason>,
    t0: Option<f64>,
    cur: Option<LegCapture>,
    captures: Vec<LegCapture>,
    warnings: Vec<String>,
}

impl VerifyVelocity {
    pub fn new(cfg: VerifyVelocityCfg, params: &RigParams) -> Self {
        let band = params.pos_guard.unwrap_or((150, 3950));
        Self {
            cfg,
            params: *params,
            band,
            phase: VPhase::ModeOpen,
            leg_idx: 0,
            polls: 0,
            last_pos: None,
            still: 0,
            seek_start: None,
            halt: None,
            t0: None,
            cur: None,
            captures: Vec::new(),
            warnings: Vec::new(),
        }
    }

    /// Leg list: each speed runs + then -, back and forth across the band.
    fn goal(&self) -> i32 {
        let speed = self.cfg.legs_cps[self.leg_idx / 2];
        if self.leg_idx.is_multiple_of(2) {
            speed
        } else {
            -speed
        }
    }

    fn leg_done(&self, pos: u16) -> bool {
        if self.goal() > 0 {
            pos >= self.band.1 - self.cfg.margin
        } else {
            pos <= self.band.0 + self.cfg.margin
        }
    }

    pub fn result(&self) -> VerifyVelocityResult {
        let mut legs = Vec::new();
        for c in &self.captures {
            let t_start = c.pts.first().map(|p| p.0).unwrap_or(0.0);
            let pts: Vec<(f64, f64)> = c
                .pts
                .iter()
                .filter(|p| (p.0 - t_start) * 1000.0 >= self.cfg.accel_trim_ms)
                .copied()
                .collect();
            let Some(f) = linear_ls(&pts) else {
                legs.push(VelocityLeg {
                    goal_cps: c.goal_cps,
                    meas_cps: 0.0,
                    err_pct: 100.0,
                    r2: 0.0,
                    n: pts.len(),
                    pass: false,
                });
                continue;
            };
            let goal = c.goal_cps as f64;
            let err_pct = (f.b - goal).abs() / goal.abs() * 100.0;
            legs.push(VelocityLeg {
                goal_cps: c.goal_cps,
                meas_cps: f.b,
                err_pct,
                r2: f.r2,
                n: f.n,
                pass: err_pct <= self.cfg.tol * 100.0,
            });
        }
        let pass = !legs.is_empty() && legs.iter().all(|l| l.pass);
        VerifyVelocityResult {
            legs,
            pass,
            warnings: self.warnings.clone(),
        }
    }
}

impl Experiment for VerifyVelocity {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            VPhase::ModeOpen => {
                self.phase = VPhase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            VPhase::TorqueOn => {
                self.phase = VPhase::SeekSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            // park at the low edge so the first (+) leg has the full band
            VPhase::SeekSet => {
                self.phase = VPhase::SeekRead;
                self.last_pos = None;
                self.seek_start = None;
                self.still = 0;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: -(self.cfg.seek_duty_q15 as i32),
                }
            }
            VPhase::SeekRead => {
                self.phase = VPhase::SeekEval;
                Cmd::Read
            }
            VPhase::SeekEval => {
                let mut parked = false;
                if let Some(o) = obs {
                    parked = o.pos <= self.band.0 + self.cfg.seek_margin;
                    if let Some(last) = self.last_pos
                        && o.pos.abs_diff(last) <= 3
                    {
                        self.still += 1;
                    } else {
                        self.still = 0;
                    }
                    self.last_pos = Some(o.pos);
                    let start = *self.seek_start.get_or_insert(o.pos);
                    if !parked
                        && self.still >= 8
                        && let Err(reason) = seek::at_stop(start, o.pos, -1, self.params.stops)
                    {
                        self.halt = Some(reason);
                        self.phase = VPhase::FinishGoal;
                        return Cmd::Pause { ms: 0 };
                    }
                }
                if parked || self.still >= 8 {
                    self.phase = VPhase::SeekOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = VPhase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            VPhase::SeekOff => {
                self.phase = VPhase::SwitchTorqueOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            VPhase::SwitchTorqueOff => {
                self.phase = VPhase::SwitchMode;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            VPhase::SwitchMode => {
                self.phase = VPhase::SwitchTorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 2,
                }
            }
            VPhase::SwitchTorqueOn => {
                self.phase = VPhase::LegSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            VPhase::LegSet => {
                self.polls = 0;
                self.t0 = None;
                self.cur = Some(LegCapture {
                    goal_cps: self.goal(),
                    pts: Vec::new(),
                });
                self.phase = VPhase::LegRead;
                Cmd::Write {
                    reg: control::GOAL_VELOCITY,
                    value: self.goal(),
                }
            }
            VPhase::LegRead => {
                self.phase = VPhase::LegEval;
                Cmd::Read
            }
            VPhase::LegEval => {
                let mut done = false;
                if let Some(o) = obs {
                    let t0 = *self.t0.get_or_insert(o.host_ms);
                    if let Some(c) = self.cur.as_mut()
                        && !self.params.in_slip(o.pos)
                    {
                        let t = (o.host_ms - t0) / 1000.0;
                        c.pts.push((t, self.params.pot.counts(o.pos)));
                    }
                    done = self.leg_done(o.pos);
                }
                self.polls += 1;
                if self.polls >= self.cfg.leg_cap_polls {
                    self.warnings
                        .push(format!("leg {}: capped before the far edge", self.goal()));
                    done = true;
                }
                if done {
                    self.phase = VPhase::LegOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = VPhase::LegRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            VPhase::LegOff => {
                if let Some(c) = self.cur.take() {
                    self.captures.push(c);
                }
                self.phase = VPhase::LegRest;
                Cmd::Write {
                    reg: control::GOAL_VELOCITY,
                    value: 0,
                }
            }
            VPhase::LegRest => {
                self.leg_idx += 1;
                if self.leg_idx < self.cfg.legs_cps.len() * 2 {
                    self.phase = VPhase::LegSet;
                } else {
                    self.phase = VPhase::FinishGoal;
                }
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            VPhase::FinishGoal => {
                self.phase = VPhase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_VELOCITY,
                    value: 0,
                }
            }
            VPhase::FinishTorque => {
                self.phase = VPhase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            VPhase::Finished => Cmd::Done,
        }
    }

    fn halted(&self) -> Option<AbortReason> {
        self.halt
    }
}

/// Combined E5+E6 verdict the CLI assembles.
#[derive(Clone, Debug)]
pub struct VerifyResult {
    pub current: Option<VerifyCurrentResult>,
    pub velocity: Option<VerifyVelocityResult>,
    pub pass: bool,
}

impl VerifyResult {
    pub fn assemble(
        current: Option<VerifyCurrentResult>,
        velocity: Option<VerifyVelocityResult>,
    ) -> Self {
        let pass = current.as_ref().is_none_or(|c| c.pass)
            && velocity.as_ref().is_none_or(|v| v.pass)
            && (current.is_some() || velocity.is_some());
        Self {
            current,
            velocity,
            pass,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exp::testkit::{FakeServo, bench_mg90, pump};
    use crate::exp::{Guarded, Permitted};
    use crate::limits::{ServoLimits, stall_counts};
    use crate::regs::control;

    const R: f64 = 7270.0 / 4096.0;
    const RAIL_2S: u16 = 3204;
    const RAIL_USB: u16 = 1780;
    const Q15: f64 = 32767.0;

    /// The bench MG90 on a rail of `vbus`: limit 280, its stops and soft
    /// limits, the 13.3% window floor.
    fn limits(vbus: u16) -> ServoLimits {
        ServoLimits {
            i_lim: 280,
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 7270,
            vbus,
            window_floor_q15: 4356,
            window_v_floor_q15: 4356,
            amps_per_count: 3.3 / 4096.0 / (15.0 * 0.060),
        }
    }

    fn plan(vbus: u16) -> DutyPlan {
        DutyPlan::new(&limits(vbus), R, None)
    }

    /// Scripted E5: the "servo" sits at the stop each seek drives at and
    /// echoes the last goal_current into i_mean with a fixed -5% bias.
    #[test]
    fn current_verify_measures_settle_and_error() {
        let cfg = VerifyCurrentCfg::planned(&plan(RAIL_USB), 4356).unwrap();
        let mut exp = VerifyCurrent::new(cfg, &crate::exp::testkit::rig());
        let goal = std::cell::Cell::new(0i16);
        let seq = std::cell::Cell::new(0u16);
        let pos = std::cell::Cell::new(2000u16);
        let mut log = Vec::new();
        let mut pending: Option<TelemetrySnapshot> = None;
        for _ in 0..2_000_000 {
            match exp.step(pending.take().as_ref()) {
                Cmd::Write { reg, value } => {
                    if reg == control::GOAL_CURRENT {
                        goal.set(value as i16);
                    }
                    if reg == control::GOAL_DUTY && value != 0 {
                        pos.set(if value > 0 { 3990 } else { 210 });
                    }
                    log.push((reg, value));
                }
                Cmd::Read => {
                    seq.set(seq.get().wrapping_add(1));
                    pending = Some(TelemetrySnapshot {
                        host_ms: seq.get() as f64 * 2.0,
                        pos: pos.get(),
                        agg_seq: seq.get(),
                        i_mean_counts: (goal.get() as f64 * 0.95) as i16,
                        duty_mean_q15: if goal.get() != 0 { 6000 } else { 0 },
                        ..Default::default()
                    });
                }
                Cmd::Pause { .. } => {}
                Cmd::Stream { .. } | Cmd::Burst { .. } => unreachable!("verify never streams"),
                Cmd::Done => break,
            }
        }
        let r = exp.result();
        assert_eq!(r.steps.len(), 4, "2 amplitudes x 2 directions");
        for s in &r.steps {
            assert!(s.pass, "step {} err {:.1}%", s.goal, s.err_pct);
            assert!((s.err_pct - 5.0).abs() < 1.5, "err {:.2}", s.err_pct);
            assert!(s.settle_ms.is_some());
        }
        assert!(r.pass);
        // mode switch rides a torque-off gap, both directions
        let mode_writes: Vec<i32> = log
            .iter()
            .filter(|(r, _)| *r == control::MODE)
            .map(|(_, v)| *v)
            .collect();
        assert_eq!(mode_writes, [0, 1, 0, 1], "open, current, per direction");
    }

    /// Scripted E6: pos advances at 97% of the goal each 4 ms poll.
    #[test]
    fn velocity_verify_measures_tracking() {
        let mut exp = VerifyVelocity::new(
            VerifyVelocityCfg::planned(&plan(RAIL_2S)),
            &crate::exp::testkit::rig(),
        );
        let goal = std::cell::Cell::new(0i32);
        let pos = std::cell::Cell::new(2000.0f64);
        let t_ms = std::cell::Cell::new(0.0f64);
        let mut pending: Option<TelemetrySnapshot> = None;
        for _ in 0..4_000_000 {
            match exp.step(pending.take().as_ref()) {
                Cmd::Write { reg, value } => {
                    if reg == control::GOAL_VELOCITY {
                        goal.set(value);
                    }
                    if reg == control::GOAL_DUTY && value < 0 {
                        pos.set(400.0); // seek lands at the low edge
                    }
                }
                Cmd::Read => {
                    // 4 ms of motion at 97% tracking, the tick counter
                    // losing a tenth of it to the bus
                    pos.set((pos.get() + goal.get() as f64 * 0.97 * 0.004).clamp(160.0, 3940.0));
                    t_ms.set(t_ms.get() + 4.0);
                    let tick = (t_ms.get() * 20.1 * 0.9) as u32;
                    pending = Some(TelemetrySnapshot {
                        host_ms: t_ms.get(),
                        pos: pos.get() as u16,
                        sample_tick: tick,
                        agg_seq: (tick / 16) as u16,
                        ..Default::default()
                    });
                }
                Cmd::Pause { .. } => {}
                Cmd::Stream { .. } | Cmd::Burst { .. } => unreachable!("verify never streams"),
                Cmd::Done => break,
            }
        }
        let r = exp.result();
        assert_eq!(r.legs.len(), 4, "2 speeds x 2 directions");
        for l in &r.legs {
            assert!(l.pass, "leg {} err {:.1}%", l.goal_cps, l.err_pct);
            assert!((l.err_pct - 3.0).abs() < 1.0, "err {:.2}", l.err_pct);
        }
        assert!(r.pass);
    }

    #[test]
    fn assemble_requires_at_least_one_section() {
        assert!(!VerifyResult::assemble(None, None).pass);
    }

    /// The envelope the CLI gives the bench servo: the guard inside its
    /// soft limits, its stops as calibrated.
    fn bench_rig() -> RigParams {
        RigParams::new(Some((532, 3526)), 350).with_stops((209, 3849))
    }

    fn run_current(
        servo: &mut FakeServo,
        cfg: VerifyCurrentCfg,
    ) -> (Option<AbortReason>, Vec<String>) {
        let params = bench_rig();
        let exp = Permitted::new(VerifyCurrent::new(cfg, &params));
        let mut g = Guarded::new(exp, params.without_pos_guard());
        let log = pump(&mut g, servo, 400_000);
        (g.abort(), log)
    }

    fn run_velocity(
        servo: &mut FakeServo,
        cfg: VerifyVelocityCfg,
    ) -> (Option<AbortReason>, Vec<String>) {
        let params = bench_rig();
        let mut g = Guarded::new(VerifyVelocity::new(cfg, &params), params);
        let log = pump(&mut g, servo, 400_000);
        (g.abort(), log)
    }

    /// Every value a log writes to `reg`.
    fn writes(log: &[String], reg: &str) -> Vec<i32> {
        let head = format!("write {reg} ");
        log.iter()
            .filter_map(|l| l.strip_prefix(&head)?.parse().ok())
            .collect()
    }

    /// The bench servo, limit 280 at 4.9 ohm: every seek of both checks
    /// drives at the plan's seek duty, whose stall the limit holds, and
    /// every current step sits between the window floor's stall current
    /// and the limit, with the permit held. On 2S the band from the 13.3%
    /// floor to the 15.5% cap has no room for a step: verify current is
    /// refused before anything moves.
    #[test]
    fn verify_seeks_stay_under_the_limit() {
        let err = VerifyCurrentCfg::planned(&plan(RAIL_2S), 4356)
            .err()
            .expect("no room on 2S");
        assert_eq!(
            err.to_string(),
            "verify current has no room on this supply: the servo reads what it needs from \
             13.3% duty and the current limit allows 15.5% at a stop; run it on a lower supply \
             voltage (USB) or raise the current limit"
        );

        let p = plan(RAIL_USB);
        let cfg = VerifyCurrentCfg::planned(&p, 4356).unwrap();
        assert_eq!(cfg.steps_counts, [182, 231]);
        let seek = cfg.seek_duty_q15;
        assert_eq!(seek, q15_floor(p.stop_cap));
        let floor_i = stall_counts(4356.0 / Q15, R, RAIL_USB as f64);
        for s in &cfg.steps_counts {
            assert!(*s as f64 > floor_i && *s < 280, "{s}");
        }
        let mut servo = bench_mg90(RAIL_USB);
        servo.pos = 2600.0;
        let (abort, log) = run_current(&mut servo, cfg);
        assert_eq!(abort, None);
        let duties = writes(&log, "goal_duty");
        assert!(
            duties.iter().all(|d| d.abs() == seek as i32 || *d == 0),
            "{duties:?}"
        );
        assert!(duties.contains(&(seek as i32)) && duties.contains(&-(seek as i32)));
        assert!(p.stall(seek as f64 / Q15) <= 280.0);
        let goals = writes(&log, "goal_current");
        assert!(goals.iter().all(|g| g.abs() < 280), "{goals:?}");
        assert!(goals.contains(&231) && goals.contains(&-231));
        let permit = log
            .iter()
            .position(|l| l == "write stall_permit 1")
            .unwrap();
        let first = log
            .iter()
            .position(|l| l.starts_with("write goal_"))
            .unwrap();
        assert!(permit < first, "the permit comes before the first drive");
        assert!(servo.pressed_ms > 0.0, "the stops were never reached");
        assert!(!servo.torque && !servo.permit_live());

        for vbus in [RAIL_2S, RAIL_USB] {
            let p = plan(vbus);
            let mut servo = bench_mg90(vbus);
            servo.pos = 2600.0;
            let (abort, log) = run_velocity(&mut servo, VerifyVelocityCfg::planned(&p));
            assert_eq!(abort, None, "{vbus}");
            let q = q15_floor(p.seek) as i32;
            assert_eq!(writes(&log, "goal_duty"), [-q, 0], "{vbus}");
            assert!(p.stall(q as f64 / Q15) <= 280.0);
            assert!(!log.iter().any(|l| l.starts_with("write stall_permit")));
            assert!(!servo.torque);
        }
    }

    /// A shaft that sticks on the way to a stop: the seek holds its one
    /// duty, never raises it, and the check ends there, blocked, torque
    /// off and the permit withdrawn.
    #[test]
    fn verify_never_raises_the_duty_toward_a_stop() {
        let p = plan(RAIL_USB);
        let cfg = VerifyCurrentCfg::planned(&p, 4356).unwrap();
        let seek = cfg.seek_duty_q15 as i32;
        let mut servo = bench_mg90(RAIL_USB);
        servo.pos = 2600.0;
        servo.jam = Some(2600.0);
        let (abort, log) = run_current(&mut servo, cfg);
        assert_eq!(
            abort,
            Some(AbortReason::Blocked {
                pos: 2600,
                moved: 0
            })
        );
        assert_eq!(writes(&log, "goal_duty"), [seek, 0]);
        assert!(
            writes(&log, "goal_current").is_empty(),
            "no step on a blocked shaft"
        );
        assert_eq!(
            log[log.len() - 2..],
            ["write stall_permit 0", "write ident_agg 0"]
        );
        assert!(!servo.torque && !servo.permit_live());

        let mut servo = bench_mg90(RAIL_USB);
        servo.pos = 2600.0;
        servo.jam = Some(2600.0);
        let (abort, log) = run_velocity(&mut servo, VerifyVelocityCfg::planned(&p));
        assert_eq!(
            abort,
            Some(AbortReason::Blocked {
                pos: 2600,
                moved: 0
            })
        );
        let q = q15_floor(p.seek) as i32;
        assert_eq!(writes(&log, "goal_duty"), [-q, 0]);
        assert!(!servo.torque);
    }
}
