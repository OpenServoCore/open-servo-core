//! Inertia duty steps: from rest, step the duty and capture the
//! accelerating transient, several amplitudes in both directions. Two data
//! paths feed the same fits: the ident aggregate windows (always
//! available, 1.25 kHz) and the TEL side channel (preferred - per-tick
//! pos/current/duty/vdiff at 20 kHz, which the choreography arms with the
//! ladder mask). The experiment stays sans-io: the driver pumps TEL bytes
//! through [`crate::frame::TelDeframer`] between commands and hands the
//! decoded frames in via [`Inertia::push_tel`]; frames arriving outside a
//! capture are dropped.
//!
//! Fit inputs and estimators (direct-alpha and exponential-rise) live in
//! [`crate::fits`]; [`Inertia::fit`] assembles per-step series - TEL when
//! a step captured frames, aggregate otherwise - and picks `b_best` by
//! fit quality.

use super::{Cmd, Experiment, RigParams, WindowStream};
use crate::fits::{BDirect, BExp, InertiaPriors, StepSeries, b_direct_fit, b_exp_fit};
use crate::frame::{TelFrame, TelemetrySnapshot};
use crate::regs::control;

/// TEL frame layout the choreography arms: pos + current + duty + vdiff
/// (11 B at 3 Mbaud - the LinkE-clean ladder set).
pub const TEL_LADDER_MASK: u16 = 0x1B;

pub struct InertiaCfg {
    /// Step duties, q15, each run + then - (35/45/55%).
    pub steps_q15: Vec<i16>,
    pub seek_duty_q15: i16,
    /// Seek target distance from the band edge: wide enough that the
    /// coast-down after the seek duty drops stays inside the guard.
    pub seek_margin: u16,
    /// Capture stop distance from the far band edge: OpenLoop cannot
    /// brake, so the rotor coasts ~w^2 / (2 * B * f_med * fric) past the
    /// stop - about 1100 counts from the 55% step's steady speed.
    pub coast_margin: u16,
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
    pub rest_ms: u32,
    /// Capture ends at the far margin or after this long.
    pub capture_max_ms: u32,
    /// Steps that captured for less than this get a short-capture warning.
    pub capture_min_ms: u32,
    pub stall_eps: u16,
    pub stall_polls: u32,
    /// Capture stall detect arms only after this long: the transient's
    /// launch is slower than the still-detect eps for tens of ms.
    pub stall_arm_ms: u32,
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
            coast_margin: 1200,
            poll_ms: 2,
            seek_poll_ms: 30,
            rest_ms: 300,
            capture_max_ms: 400,
            capture_min_ms: 60,
            stall_eps: 3,
            stall_polls: 10,
            stall_arm_ms: 120,
            seek_cap_polls: 400,
            tick_hz: 20_100.0,
        }
    }
}

/// One captured step: the aggregate samples always, the TEL frames when
/// the side channel was live.
#[derive(Clone, Debug, Default)]
struct StepCapture {
    duty_q15: i16,
    agg: Vec<AggSample>,
    tel: Vec<TelFrame>,
    t_start_ms: f64,
    t_end_ms: f64,
}

#[derive(Copy, Clone, Debug)]
struct AggSample {
    t_ms: f64,
    i: f64,
    pos: u16,
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
    TelOn,
    SeekSet,
    SeekRead,
    SeekEval,
    SeekOff,
    SeekRest,
    StepSet,
    StepRead,
    StepEval,
    StepOff,
    StepRest,
    TelOff,
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
    t_ms: f64,
    capture_start_ms: f64,
    capturing: bool,
    windows: WindowStream,
    cur: StepCapture,
    /// u8 seq unwrap for the TEL tick index: accumulated wrapping deltas
    /// survive dropped frames (a drop shows as a gap, per the firmware's
    /// seq-per-attempt contract).
    tel_last_seq: Option<u8>,
    tel_tick: u64,
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
            t_ms: 0.0,
            capture_start_ms: 0.0,
            capturing: false,
            windows: WindowStream::new(params),
            cur: StepCapture::default(),
            tel_last_seq: None,
            tel_tick: 0,
            captures: Vec::new(),
            warnings: Vec::new(),
        }
    }

    /// Feed decoded TEL frames (driver: deframer output since the last
    /// command). Only frames arriving during a step capture are kept.
    pub fn push_tel(&mut self, frames: &[TelFrame]) {
        for f in frames {
            if let Some(last) = self.tel_last_seq {
                self.tel_tick += f.seq.wrapping_sub(last) as u64;
            }
            self.tel_last_seq = Some(f.seq);
            if self.capturing {
                self.cur.tel.push(*f);
            }
        }
    }

    fn dir(&self) -> i8 {
        if self.step.is_multiple_of(2) { 1 } else { -1 }
    }

    fn duty(&self) -> i16 {
        let d = self.cfg.steps_q15[self.step / 2];
        if self.dir() > 0 { d } else { -d }
    }

    fn seek_done(&self, pos: u16) -> bool {
        if self.dir() > 0 {
            pos <= self.band.0 + self.cfg.seek_margin
        } else {
            pos >= self.band.1 - self.cfg.seek_margin
        }
    }

    fn capture_done(&self, pos: u16) -> bool {
        let at_margin = if self.dir() > 0 {
            pos >= self.band.1 - self.cfg.coast_margin
        } else {
            pos <= self.band.0 + self.cfg.coast_margin
        };
        at_margin || self.t_ms - self.capture_start_ms >= self.cfg.capture_max_ms as f64
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
        self.cur.t_end_ms = self.t_ms;
        let took = self.cur.t_end_ms - self.cur.t_start_ms;
        if took < self.cfg.capture_min_ms as f64 {
            self.warnings.push(format!(
                "step {}: capture only {took:.0} ms (transient may be cut)",
                self.cur.duty_q15
            ));
        }
        let done = core::mem::take(&mut self.cur);
        self.captures.push(done);
    }

    /// One captured step -> the fit series; TEL when frames exist,
    /// aggregate otherwise. `mask` folds in slip-zone exclusion.
    fn series_of(&self, c: &StepCapture) -> Option<(StepSeries, bool)> {
        let tick_s = 1.0 / self.cfg.tick_hz;
        if c.tel.len() >= 32 {
            // rebuild the tick index across drops from the u8 seq deltas
            let mut t = Vec::new();
            let mut pos = Vec::new();
            let mut i = Vec::new();
            let mut mask = Vec::new();
            let mut last: Option<u8> = None;
            let mut tick = 0u64;
            for f in &c.tel {
                if let Some(l) = last {
                    tick += f.seq.wrapping_sub(l) as u64;
                }
                last = Some(f.seq);
                let (Some(p), Some(cur)) = (f.pos, f.current) else {
                    continue;
                };
                t.push(tick as f64 * tick_s);
                pos.push(p as f64);
                i.push(cur as f64);
                mask.push(f.window_valid && !self.params.in_slip(p));
            }
            if t.len() >= 32 {
                return Some((
                    StepSeries {
                        t,
                        pos,
                        i,
                        mask,
                        duty_q15: c.duty_q15 as f64,
                    },
                    true,
                ));
            }
        }
        if c.agg.len() < 8 {
            return None;
        }
        let t0 = c.agg[0].t_ms;
        Some((
            StepSeries {
                t: c.agg.iter().map(|s| (s.t_ms - t0) / 1000.0).collect(),
                pos: c.agg.iter().map(|s| s.pos as f64).collect(),
                i: c.agg.iter().map(|s| s.i).collect(),
                mask: c.agg.iter().map(|s| !self.params.in_slip(s.pos)).collect(),
                duty_q15: c.duty_q15 as f64,
            },
            false,
        ))
    }

    /// Every captured step reduced to its fit series, tagged TEL/aggregate -
    /// the CLI records these for offline refits.
    pub fn step_series(&self) -> Vec<(StepSeries, bool)> {
        self.captures
            .iter()
            .filter_map(|c| self.series_of(c))
            .collect()
    }

    /// Assemble series and run both estimators. Half-windows: TEL gets
    /// ~10 ms of ticks, the aggregate path the same span in windows.
    pub fn fit(&self, priors: &InertiaPriors) -> Option<InertiaResult> {
        let mut series = Vec::new();
        let mut tel_steps = 0;
        let mut warnings = self.warnings.clone();
        for c in &self.captures {
            match self.series_of(c) {
                Some((s, from_tel)) => {
                    tel_steps += from_tel as usize;
                    series.push(s);
                }
                None => warnings.push(format!("step {}: too few samples, dropped", c.duty_q15)),
            }
        }
        if series.is_empty() {
            return None;
        }
        let hw_tel = (0.010 * self.cfg.tick_hz) as usize; // ~10 ms of ticks
        let hw = if tel_steps > 0 { hw_tel } else { 12 };
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
                self.phase = Phase::TelOn;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: TEL_LADDER_MASK as i32,
                }
            }
            Phase::TelOn => {
                self.phase = Phase::SeekSet;
                Cmd::Write {
                    reg: control::TEL_ENABLE,
                    value: 1,
                }
            }
            Phase::SeekSet => {
                self.reset_motion_track();
                self.windows.mark_transition();
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
                    self.t_ms += self.cfg.seek_poll_ms as f64;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            Phase::SeekOff => {
                self.windows.mark_transition();
                self.phase = Phase::SeekRest;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::SeekRest => {
                self.phase = Phase::StepSet;
                self.t_ms += self.cfg.rest_ms as f64;
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::StepSet => {
                self.reset_motion_track();
                self.windows.mark_transition();
                self.capturing = true;
                self.capture_start_ms = self.t_ms;
                self.cur = StepCapture {
                    duty_q15: self.duty(),
                    t_start_ms: self.t_ms,
                    ..StepCapture::default()
                };
                self.phase = Phase::StepRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.duty() as i32,
                }
            }
            Phase::StepRead => {
                self.phase = Phase::StepEval;
                Cmd::Read
            }
            Phase::StepEval => {
                let mut done = false;
                if let Some(o) = obs {
                    if let Some(w) = self.windows.push(o) {
                        self.cur.agg.push(AggSample {
                            t_ms: w.t_ms,
                            i: w.i,
                            pos: o.pos,
                        });
                    }
                    if self.t_ms - self.capture_start_ms >= self.cfg.stall_arm_ms as f64 {
                        self.track_still(o.pos);
                        if self.still >= self.cfg.stall_polls {
                            self.warnings
                                .push(format!("step {}: stalled mid-capture", self.duty()));
                            done = true;
                        }
                    }
                    done = done || self.capture_done(o.pos);
                }
                if done {
                    self.phase = Phase::StepOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::StepRead;
                    self.t_ms += self.cfg.poll_ms as f64;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            Phase::StepOff => {
                self.close_step();
                self.windows.mark_transition();
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
                    Phase::TelOff
                };
                self.t_ms += self.cfg.rest_ms as f64;
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::TelOff => {
                self.phase = Phase::TelMaskOff;
                Cmd::Write {
                    reg: control::TEL_ENABLE,
                    value: 0,
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
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, pump, pump_tel};
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

    fn run_agg() -> (Inertia, Vec<String>) {
        let mut servo = dynamic_servo();
        let params = RigParams::default();
        let mut exp = Guarded::new(Inertia::new(InertiaCfg::default(), &params), params);
        let log = pump(&mut exp, &mut servo, 4_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    #[test]
    fn recovers_planted_b_aggregate() {
        let (exp, log) = run_agg();
        assert!(!log.contains(&"OVERRUN".to_string()));
        let fit = exp.fit(&priors()).expect("fit");
        assert_eq!(fit.tel_steps, 0);
        let e = fit.b_exp.as_ref().expect("exp-rise fit");
        assert!(
            (e.b - B_PLANT).abs() / B_PLANT < 0.10,
            "b_exp {} spread {}",
            e.b,
            e.spread
        );
        // the double derivative at window spacing is loose by construction
        if let Some(d) = &fit.b_direct {
            assert!((d.b - B_PLANT).abs() / B_PLANT < 0.30, "b_direct {}", d.b);
        }
        assert!((fit.b_best - B_PLANT).abs() / B_PLANT < 0.10);
        assert!((fit.j_ff - 1.0 / fit.b_best).abs() < 1e-12);
    }

    #[test]
    fn tel_path_tightens_the_direct_estimate() {
        let mut servo = dynamic_servo();
        let params = RigParams::default();
        let mut exp = Inertia::new(InertiaCfg::default(), &params);
        let log = pump_tel(&mut exp, &mut servo, 4_000_000);
        assert!(!log.contains(&"OVERRUN".to_string()));
        let fit = exp.fit(&priors()).expect("fit");
        assert_eq!(fit.tel_steps, 6, "every step should capture TEL frames");
        let tel_direct = fit.b_direct.expect("tel direct fit");
        assert!(
            (tel_direct.b - B_PLANT).abs() / B_PLANT < 0.10,
            "tel b_direct {}",
            tel_direct.b
        );
        let e = fit.b_exp.as_ref().expect("tel exp fit");
        assert!((e.b - B_PLANT).abs() / B_PLANT < 0.05, "tel b_exp {}", e.b);

        let (agg_exp, _) = run_agg();
        let agg_fit = agg_exp.fit(&priors()).expect("agg fit");
        let agg_err = agg_fit
            .b_direct
            .map(|d| (d.b - B_PLANT).abs())
            .unwrap_or(f64::INFINITY);
        assert!(
            (tel_direct.b - B_PLANT).abs() < agg_err,
            "tel direct {} must beat aggregate direct err {}",
            tel_direct.b,
            agg_err
        );
    }

    #[test]
    fn choreography_arms_and_disarms_tel() {
        let (_, log) = run_agg();
        let idx = |needle: &str| log.iter().position(|l| l == needle).unwrap();
        let mask_on = idx("write tel_mask 27");
        let tel_on = idx("write tel_enable 1");
        let tel_off = idx("write tel_enable 0");
        let mask_off = idx("write tel_mask 0");
        let torque_off = log
            .iter()
            .rposition(|l| l == "write torque_enable 0")
            .unwrap();
        assert!(mask_on < tel_on, "mask armed before enable");
        assert!(tel_on < tel_off && tel_off < mask_off, "disarm order");
        assert!(mask_off < torque_off, "tel down before torque off");
        // last duty write is the safety zero
        let last_duty = log
            .iter()
            .rev()
            .find(|l| l.starts_with("write goal_duty"))
            .unwrap();
        assert_eq!(last_duty, "write goal_duty 0");
    }
}
