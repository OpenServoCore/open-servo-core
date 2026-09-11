//! E8 held at a stop. On a free shaft the burst's step spins the rotor up
//! inside the backlash, the envelope integrates that rotor, and L reads
//! 35-55% low with a time constant that scatters across repeats. Seated
//! against a mechanical stop with the gear train wound by a steady hold,
//! the rotor cannot follow the step and the winding is all it sees.
//!
//! Choreography, with `stall_permit` set before the first drive and cleared
//! on every way out (the [`super::Guarded`] envelope clears it on an
//! abort): centre, take a zero-duty rest reference burst while the shaft is
//! still, seek the stop at the hold duty, require real travel before
//! believing a stop, confirm it by stillness plus a non-zero applied duty,
//! keep holding, then burst from the hold to each step duty in the SAME
//! direction - never away from the stop while seated. Release (duty 0,
//! torque off, permit off) at the end.
//!
//! Analysis: the per-period regression of [`super::winding`] does not care
//! where the current starts, so the seated captures pool into one
//! regression. The shunt zero is the servo's published bias (the pre-step
//! half is driven, not at rest); the taps are zeroed on the rest reference,
//! so the terminal difference loses the dividers' split; and each capture's
//! hold level is one more settled row.

use super::inductance::{CaptureFit, FitCfg, duty_cap, fit_capture, fold, gate, worst_spread};
use super::rl::{Gate, Scales};
use super::winding::{CaptureVolts, Period, PeriodFit, Route, TapZeros, capture_volts, period_fit};
use super::{Cmd, Experiment, RigParams};
use crate::burst::{Capture, Chans};
use crate::fitmath::median;
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

const Q15: f64 = 32767.0;

/// Counts of travel before a seek believes it has gone somewhere, and of
/// progress per stall window before it believes it is still going. A few
/// counts of elastic wind-up at a stop pass any stillness threshold, and a
/// seek that mistakes wind-up for travel declares the stop it is leaning
/// on to be the one it was sent to find (bench: a ladder ran eleven rungs
/// against the wrong stop that way).
pub const SEEK_TRAVEL_MIN: u16 = 100;

/// Breakout escalation per stalled window while the shaft has not moved:
/// leaving rest costs more than holding against a stop. 5% of full scale.
pub const SEEK_STEP_Q15: i16 = 1638;

/// Mid travel with no soft guard configured: the pot's own midpoint.
const POT_MID: u16 = 2048;

// --- analysis ---------------------------------------------------------------

/// The held route feeds gain synthesis when every one of these passes: the
/// trace gates over the seated captures (cadence, step-index, windows), at
/// least two captures over two step duties, the voltage measured in the
/// burst, a physical pooled regression, and the spread of the per-capture
/// L across the repeats of one duty within `l_agree_tol` - the gate the
/// free shaft fails.
pub const HELD_GATES: [&str; 7] = [
    "captures",
    "cadence",
    "step-index",
    "windows",
    "supply",
    "regression",
    "l-spread",
];

/// One stop the run held against.
#[derive(Clone, Debug, PartialEq)]
pub struct Seat {
    /// Drive sign toward the stop.
    pub dir: i8,
    /// Position the bursts armed at, median.
    pub pos: u16,
    pub hold_duty: f64,
    /// Hold current by charge balance, amps, median over the captures.
    pub i_hold_a: Option<f64>,
    pub captures: usize,
}

#[derive(Clone, Debug, Default)]
pub struct HeldRun {
    pub captures: usize,
    pub seats: Vec<Seat>,
    /// Step duties, fractions of full scale, ascending.
    pub duties: Vec<f64>,
    /// Weakest voltage route among the captures.
    pub route: Option<Route>,
    /// The taps were zeroed on a rest reference; otherwise their split
    /// rides in the regression's intercept.
    pub rest_zeroed: bool,
    pub reg: Option<PeriodFit>,
    pub hold_rows: usize,
    /// Worst (max - min) / median of the per-capture L over the repeats of
    /// one duty.
    pub l_spread: f64,
    pub gates: Vec<Gate>,
}

impl HeldRun {
    pub fn promotable(&self) -> bool {
        self.captures > 0
            && self
                .gates
                .iter()
                .filter(|g| HELD_GATES.contains(&g.name))
                .all(|g| g.pass)
    }

    pub fn blocking(&self) -> Vec<&'static str> {
        self.gates
            .iter()
            .filter(|g| HELD_GATES.contains(&g.name) && !g.pass)
            .map(|g| g.name)
            .collect()
    }
}

/// Pool the seated captures. A capture counts only when it steps from a
/// hold toward the stop: same sign, larger magnitude.
pub fn held_run(
    fitted: &[(&Capture, CaptureFit)],
    zeros: &TapZeros,
    sc: &Scales,
    cfg: &FitCfg,
) -> HeldRun {
    let caps: Vec<(&Capture, &CaptureFit, CaptureVolts)> = fitted
        .iter()
        .filter(|(c, _)| {
            let (p, s) = (c.meta.pre_q15, c.meta.step_q15);
            p != 0 && p.signum() == s.signum() && s.unsigned_abs() > p.unsigned_abs()
        })
        .filter_map(|(c, f)| capture_volts(c, f, sc, true, zeros).map(|v| (*c, f, v)))
        .collect();
    let rows = |v: &CaptureVolts| v.periods.iter().copied().chain(v.hold).collect::<Vec<_>>();
    let period_s = median(
        &caps
            .iter()
            .map(|(_, f, _)| f.period_samples * f.sample_us * 1e-6)
            .collect::<Vec<_>>(),
    )
    .unwrap_or(0.0);
    let pooled: Vec<Period> = caps.iter().flat_map(|(_, _, v)| rows(v)).collect();
    let reg = period_fit(&pooled, period_s);

    let mut duties: Vec<f64> = caps.iter().map(|(_, f, _)| f.duty).collect();
    duties.sort_by(f64::total_cmp);
    duties.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let by_duty: Vec<Vec<f64>> = duties
        .iter()
        .map(|d| {
            caps.iter()
                .filter(|(_, f, _)| (f.duty - d).abs() < 1e-6)
                .filter_map(|(_, _, v)| period_fit(&rows(v), period_s))
                .map(|g| g.l_h)
                .filter(|l| *l > 0.0)
                .collect()
        })
        .collect();
    let repeated = by_duty.iter().filter(|g| g.len() > 1).count();
    let l_spread = worst_spread(&by_duty);

    let mut seats = Vec::new();
    for dir in [-1i8, 1] {
        let at: Vec<&(&Capture, &CaptureFit, CaptureVolts)> = caps
            .iter()
            .filter(|(c, _, _)| c.meta.step_q15.signum() as i8 == dir)
            .collect();
        if at.is_empty() {
            continue;
        }
        let pos: Vec<f64> = at.iter().map(|(c, _, _)| c.meta.pos as f64).collect();
        let hold: Vec<f64> = at.iter().map(|(_, f, _)| f.duty_pre).collect();
        let i: Vec<f64> = at
            .iter()
            .filter_map(|(_, _, v)| v.hold.map(|h| h.i0))
            .collect();
        seats.push(Seat {
            dir,
            pos: median(&pos).unwrap_or(0.0).round() as u16,
            hold_duty: median(&hold).unwrap_or(0.0),
            i_hold_a: median(&i),
            captures: at.len(),
        });
    }

    let route = caps.iter().filter_map(|(_, _, v)| v.route).min();
    let fits: Vec<&CaptureFit> = caps.iter().map(|(_, f, _)| *f).collect();
    let gates = vec![
        gate(
            "captures",
            caps.len() >= 2 && duties.len() >= 2,
            format!("{} seated at {} step duties", caps.len(), duties.len()),
        ),
        fold("cadence", &fits),
        fold("step-index", &fits),
        fold("windows", &fits),
        gate(
            "supply",
            route.is_some(),
            match route {
                Some(r) => format!("measured in the burst: {}", r.as_str()),
                None => "not measured: the held route needs a voltage channel".into(),
            },
        ),
        gate(
            "regression",
            reg.is_some_and(|g| g.r_ohm > 0.0 && g.l_h > 0.0),
            match reg {
                Some(g) => format!("{} rows", g.n),
                None => "degenerate".into(),
            },
        ),
        gate(
            "l-spread",
            repeated > 0 && l_spread <= cfg.l_agree_tol,
            match repeated {
                0 => "no duty carries two per-capture L to spread".into(),
                _ => format!(
                    "per-capture L {:.0}% across the repeats of one duty",
                    l_spread * 100.0
                ),
            },
        ),
    ];
    HeldRun {
        captures: caps.len(),
        seats,
        duties,
        route,
        rest_zeroed: zeros.a.is_some() || zeros.b.is_some(),
        reg,
        hold_rows: caps.iter().filter(|(_, _, v)| v.hold.is_some()).count(),
        l_spread,
        gates,
    }
}

// --- experiment -------------------------------------------------------------

/// Which mechanical stops the run seats against. Low by default: this
/// unit's high-count end has slipped its gears at ~0.17 A of steady torque,
/// under what a 40% burst draws, while the low end takes the whole ladder.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Stops {
    Low,
    High,
    Both,
}

impl Stops {
    /// Drive signs toward each stop, in visiting order. Positive duty moves
    /// the pot up.
    fn dirs(self) -> &'static [i8] {
        match self {
            Stops::Low => &[-1],
            Stops::High => &[1],
            Stops::Both => &[-1, 1],
        }
    }
}

#[derive(Clone, Debug)]
pub struct HeldCfg {
    pub stops: Stops,
    /// Seek and hold duty, percent of full scale: gentle enough to arrive
    /// softly, firm enough to keep the train wound between bursts.
    pub hold_pct: u8,
    /// Step duties, percent of full scale, low first so the i-max rule has
    /// a measured rung before the bigger ones arm. A rung at or under the
    /// hold is dropped: every burst steps toward the stop.
    pub step_pct: Vec<u8>,
    pub repeats: u32,
    /// Settled winding current the plan stays under, amps.
    pub i_max_a: f64,
    pub chans: Chans,
    /// Escalation ceiling while the shaft has not moved, percent of full
    /// scale. Under the top rung, so a seek never loads a stop harder than
    /// the bursts do.
    pub seek_cap_pct: u8,
    /// The same for the centring leg, which drives away from any stop and
    /// has nothing to hit. Leaving a stop costs far more than holding one:
    /// ~40% frees this servo on 2S, ~63% on USB.
    pub centre_cap_pct: u8,
    pub seek_poll_ms: u32,
    /// Poll budget per seek: 500 x 20 ms is 10 s of travel.
    pub seek_polls_max: u32,
    /// Pause at rest before the reference burst, and at the hold before
    /// the first step.
    pub settle_ms: u32,
    /// Pause at the hold between bursts.
    pub rest_ms: u32,
    /// Distance from mid travel the centring leg stops at, counts.
    pub centre_margin: u16,
    pub fit: FitCfg,
}

impl Default for HeldCfg {
    fn default() -> Self {
        Self {
            stops: Stops::Low,
            hold_pct: 12,
            step_pct: vec![20, 30, 40],
            repeats: 4,
            i_max_a: 0.4,
            chans: Chans::Driven,
            seek_cap_pct: 30,
            centre_cap_pct: 45,
            seek_poll_ms: 20,
            seek_polls_max: 500,
            settle_ms: 200,
            rest_ms: 150,
            centre_margin: 300,
            fit: FitCfg::default(),
        }
    }
}

fn pct_q15(pct: u8) -> i16 {
    (pct as i32 * Q15 as i32 / 100) as i16
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Phase {
    ModeWrite,
    TelOff,
    PermitOn,
    TorqueOn,
    CentreRead,
    CentreEval,
    CentreWait,
    RestDuty,
    RestSettle,
    RestBurst,
    SeekStart,
    SeekDrive,
    SeekWait,
    SeekRead,
    SeekEval,
    SeatSettle,
    SeatRead,
    SeatEval,
    BurstRead,
    BurstEval,
    Burst,
    BurstPause,
    Release,
    FinishDuty,
    FinishTorque,
    FinishPermit,
    Finished,
}

/// Where the shaft seated.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Seated {
    pub dir: i8,
    pub pos: u16,
    /// Applied duty confirmed at the hold, q15.
    pub duty_applied_q15: i16,
    /// Duty the seek arrived at, q15; above the hold when it escalated.
    pub arrived_q15: i16,
}

pub struct Held {
    cfg: HeldCfg,
    sc: Scales,
    band: (u16, u16),
    phase: Phase,
    stop: usize,
    rungs: Vec<i16>,
    at: usize,
    polls: u32,
    mag: i16,
    start: u16,
    last: u16,
    moved: bool,
    window: u32,
    stall_polls: u32,
    drift_max: u16,
    seat: Option<Seated>,
    seats: Vec<Seated>,
    caps: Vec<Capture>,
    warnings: Vec<String>,
}

impl Held {
    /// `params` places the centring band; run it under
    /// [`RigParams::without_pos_guard`], since a stop can sit past the soft
    /// travel guard.
    pub fn new(cfg: HeldCfg, params: &RigParams, sc: Scales) -> Self {
        let mid = params.pos_guard.map_or(POT_MID, |(lo, hi)| (lo + hi) / 2);
        let band = (
            mid.saturating_sub(cfg.centre_margin),
            mid.saturating_add(cfg.centre_margin),
        );
        let mut steps: Vec<i16> = cfg
            .step_pct
            .iter()
            .filter(|p| **p > cfg.hold_pct)
            .map(|p| pct_q15(*p))
            .collect();
        steps.sort_unstable();
        steps.dedup();
        let rungs = steps
            .iter()
            .flat_map(|r| std::iter::repeat_n(*r, cfg.repeats as usize))
            .collect();
        Self {
            cfg,
            sc,
            band,
            phase: Phase::ModeWrite,
            stop: 0,
            rungs,
            at: 0,
            polls: 0,
            mag: 0,
            start: 0,
            last: 0,
            moved: false,
            window: 0,
            stall_polls: params.stall_polls,
            drift_max: params.stall_eps.saturating_mul(params.stall_polls as u16),
            seat: None,
            seats: Vec::new(),
            caps: Vec::new(),
            warnings: Vec::new(),
        }
    }

    pub fn captures(&self) -> &[Capture] {
        &self.caps
    }

    pub fn seats(&self) -> &[Seated] {
        &self.seats
    }

    pub fn warnings(&self) -> &[String] {
        &self.warnings
    }

    fn dir(&self) -> i8 {
        self.cfg.stops.dirs()[self.stop]
    }

    fn hold_q15(&self) -> i16 {
        self.dir() as i16 * pct_q15(self.cfg.hold_pct)
    }

    fn duty(&self, q15: i16) -> Cmd {
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: q15 as i32,
        }
    }

    /// Straight to the exit, which clears the permit.
    fn fail(&mut self, why: String) -> Cmd {
        self.warnings.push(why);
        self.phase = Phase::FinishDuty;
        Cmd::Pause { ms: 0 }
    }

    /// Within a stall window's jitter of the seat (a shaft still creeping
    /// toward the stop at the hold duty leaves it during the settle), the
    /// applied duty still pushing toward the stop.
    fn still_seated(&self, o: &TelemetrySnapshot, seat: &Seated) -> Result<(), String> {
        if o.pos.abs_diff(seat.pos) > self.drift_max {
            return Err(format!(
                "the shaft moved from the seat at {} to pos {}",
                seat.pos, o.pos
            ));
        }
        if o.duty_applied_q15 == 0 || o.duty_applied_q15.signum() as i8 != seat.dir {
            return Err(format!(
                "applied duty {} at pos {}: the kernel is not driving into the stop",
                o.duty_applied_q15, o.pos
            ));
        }
        Ok(())
    }
}

impl Experiment for Held {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::ModeWrite => {
                self.phase = Phase::TelOff;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            // The servo refuses an arm while TEL is still streaming.
            Phase::TelOff => {
                self.phase = Phase::PermitOn;
                Cmd::Write {
                    reg: control::TEL_COUNT,
                    value: 0,
                }
            }
            Phase::PermitOn => {
                self.phase = Phase::TorqueOn;
                Cmd::Write {
                    reg: control::STALL_PERMIT,
                    value: 1,
                }
            }
            Phase::TorqueOn => {
                self.phase = Phase::CentreRead;
                self.polls = 0;
                self.mag = pct_q15(self.cfg.hold_pct);
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            // Mid travel first: the seek has to see real travel, and leaving
            // a stop is this leg's job, where an escalated duty has nothing
            // to hit.
            Phase::CentreRead => {
                self.phase = Phase::CentreEval;
                Cmd::Read
            }
            Phase::CentreEval => {
                let Some(o) = obs else {
                    return self.fail("centring read returned nothing".into());
                };
                if (self.band.0..=self.band.1).contains(&o.pos) {
                    self.phase = Phase::RestDuty;
                    return Cmd::Pause { ms: 0 };
                }
                if self.polls == 0 {
                    self.last = o.pos;
                }
                self.polls += 1;
                if self.polls > self.cfg.seek_polls_max {
                    return self.fail(format!("centring stuck at pos {}", o.pos));
                }
                if self.polls.is_multiple_of(self.stall_polls) {
                    if o.pos.abs_diff(self.last) < SEEK_TRAVEL_MIN {
                        self.mag = self
                            .mag
                            .saturating_add(SEEK_STEP_Q15)
                            .min(pct_q15(self.cfg.centre_cap_pct));
                    }
                    self.last = o.pos;
                }
                self.phase = Phase::CentreWait;
                self.duty(if o.pos < self.band.0 {
                    self.mag
                } else {
                    -self.mag
                })
            }
            Phase::CentreWait => {
                self.phase = Phase::CentreRead;
                Cmd::Pause {
                    ms: self.cfg.seek_poll_ms,
                }
            }
            Phase::RestDuty => {
                self.phase = Phase::RestSettle;
                self.duty(0)
            }
            Phase::RestSettle => {
                self.phase = Phase::RestBurst;
                Cmd::Pause {
                    ms: self.cfg.settle_ms,
                }
            }
            // The taps' rest levels in the burst's own aperture and slot
            // order: the seated captures' pre-step halves are driven.
            Phase::RestBurst => {
                self.phase = Phase::SeekStart;
                Cmd::Burst {
                    duty_q15: 0,
                    pre_q15: 0,
                    chans: self.cfg.chans.for_step(self.hold_q15()),
                    seated: false,
                }
            }
            Phase::SeekStart => {
                self.phase = Phase::SeekDrive;
                self.polls = 0;
                self.window = 0;
                self.moved = false;
                self.mag = pct_q15(self.cfg.hold_pct);
                Cmd::Read
            }
            Phase::SeekDrive => {
                let Some(o) = obs else {
                    return self.fail("seek read returned nothing".into());
                };
                self.start = o.pos;
                self.last = o.pos;
                self.phase = Phase::SeekWait;
                self.duty(self.dir() as i16 * self.mag)
            }
            Phase::SeekWait => {
                self.phase = Phase::SeekRead;
                Cmd::Pause {
                    ms: self.cfg.seek_poll_ms,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            // Progress over a window, not stillness poll to poll: a couple
            // of counts of ADC jitter at the rail resets a consecutive count
            // and the escalation never fires.
            Phase::SeekEval => {
                let Some(o) = obs else {
                    return self.fail("seek read returned nothing".into());
                };
                self.polls += 1;
                self.moved |= o.pos.abs_diff(self.start) >= SEEK_TRAVEL_MIN;
                if self.polls > self.cfg.seek_polls_max {
                    return self.fail(format!(
                        "no stop within {} polls driving {:+} from pos {}",
                        self.cfg.seek_polls_max,
                        self.dir(),
                        self.start
                    ));
                }
                self.window += 1;
                self.phase = Phase::SeekWait;
                if self.window < self.stall_polls {
                    return Cmd::Pause { ms: 0 };
                }
                self.window = 0;
                let progress = o.pos.abs_diff(self.last) >= SEEK_TRAVEL_MIN;
                self.last = o.pos;
                match (progress, self.moved) {
                    (true, _) => Cmd::Pause { ms: 0 },
                    (false, true) => {
                        self.seat = Some(Seated {
                            dir: self.dir(),
                            pos: o.pos,
                            duty_applied_q15: o.duty_applied_q15,
                            arrived_q15: self.dir() as i16 * self.mag,
                        });
                        self.phase = Phase::SeatSettle;
                        self.duty(self.hold_q15())
                    }
                    (false, false) => {
                        let next = self.mag.saturating_add(SEEK_STEP_Q15);
                        if next > pct_q15(self.cfg.seek_cap_pct) {
                            return self.fail(format!(
                                "the shaft never moved from pos {} up to {} q15: jammed, or the \
                                 kernel is zeroing the duty",
                                o.pos, self.mag
                            ));
                        }
                        self.mag = next;
                        self.duty(self.dir() as i16 * self.mag)
                    }
                }
            }
            Phase::SeatSettle => {
                self.phase = Phase::SeatRead;
                Cmd::Pause {
                    ms: self.cfg.settle_ms,
                }
            }
            Phase::SeatRead => {
                self.phase = Phase::SeatEval;
                Cmd::Read
            }
            // A soft limit stops the shaft too - the kernel zeroes the
            // outbound duty there - so stillness alone is not a stop. The
            // applied duty tells them apart: it holds at a real stop and
            // reads zero at a soft limit the permit did not open.
            Phase::SeatEval => {
                let (Some(o), Some(mut seat)) = (obs, self.seat) else {
                    return self.fail("seat read returned nothing".into());
                };
                if let Err(why) = self.still_seated(o, &seat) {
                    return self.fail(format!("not a stop: {why}"));
                }
                seat.pos = o.pos;
                seat.duty_applied_q15 = o.duty_applied_q15;
                self.seat = Some(seat);
                self.seats.push(seat);
                self.at = 0;
                self.phase = Phase::BurstRead;
                Cmd::Pause { ms: 0 }
            }
            Phase::BurstRead => {
                if self.at >= self.rungs.len() {
                    self.phase = Phase::Release;
                    return Cmd::Pause { ms: 0 };
                }
                self.phase = Phase::BurstEval;
                Cmd::Read
            }
            Phase::BurstEval => {
                let (Some(o), Some(seat)) = (obs, self.seat) else {
                    return self.fail("pre-burst read returned nothing".into());
                };
                if let Err(why) = self.still_seated(o, &seat) {
                    return self.fail(format!("run cut short: {why}"));
                }
                self.phase = Phase::Burst;
                Cmd::Pause { ms: 0 }
            }
            Phase::Burst => {
                let step = self.dir() as i16 * self.rungs[self.at];
                self.phase = Phase::BurstPause;
                Cmd::Burst {
                    duty_q15: step,
                    pre_q15: self.hold_q15(),
                    chans: self.cfg.chans.for_step(step),
                    seated: true,
                }
            }
            Phase::BurstPause => {
                self.at += 1;
                self.phase = Phase::BurstRead;
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::Release => {
                self.seat = None;
                self.stop += 1;
                self.phase = if self.stop < self.cfg.stops.dirs().len() {
                    self.polls = 0;
                    self.mag = pct_q15(self.cfg.hold_pct);
                    Phase::CentreRead
                } else {
                    Phase::FinishDuty
                };
                self.duty(0)
            }
            Phase::FinishDuty => {
                self.phase = Phase::FinishTorque;
                self.duty(0)
            }
            Phase::FinishTorque => {
                self.phase = Phase::FinishPermit;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::FinishPermit => {
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::STALL_PERMIT,
                    value: 0,
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }

    fn push_burst(&mut self, cap: &Capture) {
        if cap.meta.seated
            && let Some(f) = fit_capture(cap, &self.sc, &self.cfg.fit)
            && let Some(cap_duty) = duty_cap(&f, self.cfg.i_max_a)
        {
            let cap_q15 = (cap_duty * Q15) as i16;
            let before = self.rungs.len();
            let at = self.at;
            let mut k = 0;
            self.rungs.retain(|r| {
                let keep = k <= at || *r <= cap_q15;
                k += 1;
                keep
            });
            if self.rungs.len() < before {
                self.warnings.push(format!(
                    "{} rungs dropped: at {:.0}% the settled current reads {:.3} A, so anything \
                     over {:.0}% would clear the {:.2} A envelope",
                    before - self.rungs.len(),
                    f.duty * 100.0,
                    f.asymptote_a.max(f.asymptote_cb_a),
                    cap_duty * 100.0,
                    self.cfg.i_max_a
                ));
            }
        }
        self.caps.push(cap.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::super::inductance::{BurstRoute, fit_captures};
    use super::super::testkit::{FakeServo, SynthBurst, pump};
    use super::super::{AbortReason, Guarded};
    use super::*;
    use crate::burst::{CHAN_VBUS, CHAN_VMOTOR_A, CHAN_VMOTOR_B, from_csv};
    use crate::units::SenseParams;

    fn scales() -> Scales {
        let s = SenseParams {
            shunt_r_mohm: 60,
            gain_milli: 15_000,
            vmotor_div_top: 6_800,
            vmotor_div_bot: 3_300,
            vdd_mv: 3_300,
            tick_hz: 20_100,
        };
        Scales::from_sense(&s, 15_000, 10_000).unwrap()
    }

    /// Board D on USB, sg90 with a fresh gear train, seated at the low stop
    /// from a 12% hold, bursts to 20/30/40% with both taps and the rail
    /// (frame_len 4), four each. The recordings carry no rest reference.
    fn bench_held() -> Vec<Capture> {
        macro_rules! fixtures {
            ($($n:literal),*) => {
                [$(include_str!(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/testdata/burst/held/",
                    $n,
                    ".csv"
                ))),*]
            };
        }
        fixtures!(
            "c7-n20-0", "c7-n20-1", "c7-n20-2", "c7-n20-3", "c7-n30-0", "c7-n30-1", "c7-n30-2",
            "c7-n30-3", "c7-n40-0", "c7-n40-1", "c7-n40-2", "c7-n40-3"
        )
        .iter()
        .map(|t| from_csv(t).expect("fixture parses"))
        .collect()
    }

    /// A smoke band around the locked 2S ladder, not a pin: one motor on
    /// one soft rail.
    #[test]
    fn bench_held_captures_replay_into_the_locked_band() {
        let r = fit_captures(&bench_held(), &scales(), &FitCfg::default()).expect("fit");
        let h = &r.held;
        let g = h.reg.expect("regression");
        println!(
            "held: R {:.3} ohm L {:.4} mH tau {:.1} us c {:.3} V, {} rows, {} hold levels, \
             l-spread {:.2}, seats {:?}",
            g.r_ohm,
            g.l_h * 1e3,
            g.tau_us,
            g.c_volts,
            g.n,
            h.hold_rows,
            h.l_spread,
            h.seats
        );
        assert_eq!(h.captures, 12);
        assert_eq!(h.route, Some(Route::Terminals));
        assert!((3.6..=4.6).contains(&g.r_ohm), "R {}", g.r_ohm);
        assert!((0.5e-3..=0.9e-3).contains(&g.l_h), "L {}", g.l_h);
        assert_eq!(h.hold_rows, 12);
        assert_eq!(h.seats.len(), 1);
        assert_eq!((h.seats[0].dir, h.seats[0].pos), (-1, 122));
        assert!(h.promotable(), "{:?}", h.blocking());
        assert_eq!(r.route(), Some(BurstRoute::Held));
        assert_eq!(r.gain_r_l(), Some((g.r_ohm, g.l_h)));
    }

    /// A 4 ohm, 0.6 mH winding with a 0.2 V brush drop behind board D's
    /// bridge, the rotor held (no back-EMF), the taps 4 counts apart at
    /// rest as the bench's are.
    fn winding() -> SynthBurst {
        SynthBurst {
            r: 4.0,
            l: 0.6e-3,
            v0: 0.2,
            settle_us: 2.0,
            split: 4.0,
            ..SynthBurst::board_d().with_bridge()
        }
    }

    /// 2S behind a stiff source, with a 10 uF island inside the shunt's
    /// ground.
    fn stiff() -> SynthBurst {
        SynthBurst {
            v_rail: 7.3,
            c_island: 10e-6,
            r_feed: 0.1,
            ..winding()
        }
    }

    /// USB: 1.3 ohm behind a 100 uF bulk that sags inside every ON window.
    fn soft() -> SynthBurst {
        SynthBurst {
            v_rail: 4.37,
            r_src: 1.3,
            ..winding()
        }
    }

    /// The choreography's captures: a rest reference when `rest`, then
    /// 20/30/40% from a 12% hold toward the low stop, twice each.
    fn seated_run(plant: &SynthBurst, chans: Chans, rest: bool) -> HeldRun {
        let hold = -pct_q15(12);
        let at = |q: i16| SynthBurst {
            chans: chans.for_step(q),
            ..plant.clone()
        };
        let mut caps: Vec<Capture> = rest.then(|| at(hold).capture(0, 0)).into_iter().collect();
        for pct in [20, 30, 40] {
            for _ in 0..2 {
                let q = -pct_q15(pct);
                let mut c = at(q).capture(q, hold);
                c.meta.seated = true;
                caps.push(c);
            }
        }
        let r = fit_captures(&caps, &scales(), &FitCfg::default()).expect("fit");
        assert_eq!(r.route(), Some(BurstRoute::Held), "{:?}", r.held.blocking());
        r.held
    }

    fn rel(got: f64, want: f64) -> f64 {
        (got - want) / want
    }

    #[test]
    fn a_seated_winding_recovers_r_and_l_on_both_rails() {
        for (rail, plant) in [("stiff", stiff()), ("soft", soft())] {
            for chans in [Chans::Driven, Chans::Fixed(CHAN_VMOTOR_A | CHAN_VMOTOR_B)] {
                let h = seated_run(&plant, chans, true);
                let g = h.reg.expect("regression");
                println!(
                    "{rail:5} {chans:?}: {:?} R {:.3} ohm L {:.4} mH c {:.3} V, hold {:.4} A",
                    h.route,
                    g.r_ohm,
                    g.l_h * 1e3,
                    g.c_volts,
                    h.seats[0].i_hold_a.unwrap_or(0.0)
                );
                assert!(h.rest_zeroed);
                assert!(
                    rel(g.r_ohm, plant.r).abs() < 0.03,
                    "{rail} {chans:?} R {}",
                    g.r_ohm
                );
                assert!(
                    rel(g.l_h, plant.l).abs() < 0.05,
                    "{rail} {chans:?} L {}",
                    g.l_h
                );
                assert!((g.c_volts - plant.v0).abs() < 0.02, "{rail} {chans:?} c");
            }
        }
    }

    /// The taps' split rides in the terminal difference unless the rest
    /// reference takes it out: the intercept moves by it, R and L do not.
    #[test]
    fn the_rest_reference_takes_the_tap_split_out_of_the_intercept() {
        let chans = Chans::Fixed(CHAN_VMOTOR_A | CHAN_VMOTOR_B);
        let with = seated_run(&stiff(), chans, true).reg.unwrap();
        let without = seated_run(&stiff(), chans, false).reg.unwrap();
        let split_v = stiff().split * scales().v_term_per_count;
        assert!(((without.c_volts - with.c_volts) - split_v).abs() < 0.002);
        assert!(rel(without.r_ohm, with.r_ohm).abs() < 0.005);
        assert!(rel(without.l_h, with.l_h).abs() < 0.005);
    }

    /// At frame_len 4 a 20% ON window is 2.3 shunt samples and every burst
    /// launches tick-synchronous, so the charge balance's quantisation
    /// lands on the same phases each repeat and does not dither away: L
    /// reads low, R holds. The choreography samples at frame_len 2 or 3.
    #[test]
    fn frame_len_4_keeps_r_and_reads_l_low() {
        for plant in [stiff(), soft()] {
            let g = seated_run(&plant, Chans::Fixed(CHANS_ALL), true)
                .reg
                .unwrap();
            println!("frame_len 4: R {:.3} L {:.4} mH", g.r_ohm, g.l_h * 1e3);
            assert!(rel(g.r_ohm, plant.r).abs() < 0.03, "R {}", g.r_ohm);
            assert!((-0.12..-0.03).contains(&rel(g.l_h, plant.l)), "L {}", g.l_h);
        }
    }

    const CHANS_ALL: u8 = CHAN_VMOTOR_A | CHAN_VMOTOR_B | CHAN_VBUS;

    fn rig() -> RigParams {
        RigParams::default()
    }

    /// Soft limits just inside the mechanical ends, as a calibrated servo
    /// has them: only the permit lets the seek reach the stop.
    fn servo() -> FakeServo {
        let mut s = FakeServo::new(3.37);
        s.dynamic = true;
        s.soft = Some((230.0, 3970.0));
        s
    }

    fn run(servo: &mut FakeServo) -> (Guarded<Held>, Vec<String>) {
        let cfg = HeldCfg {
            repeats: 2,
            ..HeldCfg::default()
        };
        let params = rig().without_pos_guard();
        let mut exp = Guarded::new(Held::new(cfg, &rig(), scales()), params);
        let log = pump(&mut exp, servo, 400_000);
        assert!(!log.contains(&"OVERRUN".to_string()));
        (exp, log)
    }

    fn at(log: &[String], what: &str) -> usize {
        log.iter()
            .position(|l| l == what)
            .unwrap_or_else(|| panic!("no `{what}` in the log"))
    }

    /// The permit precedes the first drive and is the last write, whatever
    /// ended the run.
    fn permit_brackets(log: &[String]) {
        let first_drive = log
            .iter()
            .position(|l| {
                l.starts_with("burst") || (l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            })
            .expect("a drive");
        assert!(at(log, "write stall_permit 1") < first_drive);
        assert_eq!(log.last().map(String::as_str), Some("write stall_permit 0"));
        assert!(log.contains(&"write torque_enable 0".to_string()));
    }

    #[test]
    fn seats_at_the_low_stop_and_bursts_only_toward_it() {
        let mut s = servo();
        let (exp, log) = run(&mut s);
        assert!(exp.abort().is_none(), "{:?}", exp.abort());
        permit_brackets(&log);
        let tail: Vec<&str> = log.iter().rev().take(3).map(String::as_str).collect();
        assert_eq!(
            tail,
            [
                "write stall_permit 0",
                "write torque_enable 0",
                "write goal_duty 0"
            ]
        );
        let exp = exp.into_inner();
        assert!(exp.warnings().is_empty(), "{:?}", exp.warnings());
        let seat = exp.seats()[0];
        assert_eq!((seat.dir, seat.pos), (-1, 200));
        assert_eq!(seat.duty_applied_q15, -pct_q15(12));
        let seated: Vec<&Capture> = exp.captures().iter().filter(|c| c.meta.seated).collect();
        assert_eq!(seated.len(), 3 * 2);
        for c in &seated {
            let (p, q) = (c.meta.pre_q15, c.meta.step_q15);
            assert_eq!(p, -pct_q15(12));
            assert!(q < p, "a burst stepped away from the stop: {q} from {p}");
            assert_eq!(c.meta.pos, 200);
        }
        // one rest reference, before the seek
        let refs: Vec<&Capture> = exp
            .captures()
            .iter()
            .filter(|c| c.meta.step_q15 == 0)
            .collect();
        assert_eq!(refs.len(), 1);
        assert!(!refs[0].meta.seated);
        let r = exp_fit(&exp);
        assert_eq!(r.route(), Some(BurstRoute::Held), "{:?}", r.held.blocking());
    }

    fn exp_fit(exp: &Held) -> crate::exp::inductance::InductanceResult {
        fit_captures(exp.captures(), &scales(), &FitCfg::default()).expect("fit")
    }

    #[test]
    fn both_stops_seat_in_turn() {
        let mut s = servo();
        let cfg = HeldCfg {
            stops: Stops::Both,
            repeats: 1,
            ..HeldCfg::default()
        };
        let mut exp = Guarded::new(Held::new(cfg, &rig(), scales()), rig().without_pos_guard());
        let log = pump(&mut exp, &mut s, 400_000);
        assert!(exp.abort().is_none());
        permit_brackets(&log);
        let exp = exp.into_inner();
        let seats: Vec<(i8, u16)> = exp.seats().iter().map(|s| (s.dir, s.pos)).collect();
        assert_eq!(seats, [(-1, 200), (1, 4000)]);
        for c in exp.captures().iter().filter(|c| c.meta.seated) {
            let dir = if c.meta.pos == 200 { -1 } else { 1 };
            assert_eq!(c.meta.step_q15.signum(), dir);
            assert!(c.meta.step_q15.unsigned_abs() > c.meta.pre_q15.unsigned_abs());
        }
    }

    #[test]
    fn an_abort_mid_burst_still_withdraws_the_permit() {
        let mut s = servo();
        s.fault_after_bursts = Some(3);
        let (exp, log) = run(&mut s);
        assert!(matches!(exp.abort(), Some(AbortReason::Fault { .. })));
        permit_brackets(&log);
        assert!(!s.permit);
        let tail: Vec<&str> = log.iter().rev().take(3).map(String::as_str).collect();
        assert_eq!(
            tail,
            [
                "write stall_permit 0",
                "write torque_enable 0",
                "write goal_duty 0"
            ]
        );
        assert_eq!(exp.into_inner().captures().len(), 3);
    }

    /// Stillness at a soft limit looks like a stop; the applied duty says
    /// it is not one.
    #[test]
    fn a_soft_limit_the_permit_does_not_open_is_not_a_stop() {
        let mut s = servo();
        s.honors_permit = false;
        let (exp, log) = run(&mut s);
        assert!(exp.abort().is_none());
        permit_brackets(&log);
        let exp = exp.into_inner();
        assert!(exp.seats().is_empty());
        assert!(exp.captures().iter().all(|c| !c.meta.seated));
        assert!(
            exp.warnings().iter().any(|w| w.contains("not a stop")),
            "{:?}",
            exp.warnings()
        );
    }

    #[test]
    fn a_jammed_shaft_escalates_to_the_cap_and_gives_up() {
        let mut s = servo();
        s.fc = 1e6;
        s.pos = 2000.0;
        let (exp, log) = run(&mut s);
        permit_brackets(&log);
        let cap = pct_q15(HeldCfg::default().seek_cap_pct) as i32;
        assert!(
            log.iter()
                .filter_map(|l| l.strip_prefix("write goal_duty "))
                .all(|v| v.parse::<i32>().unwrap().abs() <= cap)
        );
        let exp = exp.into_inner();
        assert!(exp.captures().iter().all(|c| !c.meta.seated));
        assert!(
            exp.warnings().iter().any(|w| w.contains("never moved")),
            "{:?}",
            exp.warnings()
        );
    }
}
