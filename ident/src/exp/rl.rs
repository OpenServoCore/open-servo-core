//! Winding R and L from free-shaft duty toggles. At mid travel the duty
//! steps between two levels every `step_periods` PWM periods, each step
//! one chained TEL burst ([`Cmd::Stream`]) at full tick rate, ~40 steps
//! per chain, both directions and two bias levels. The method rests on the
//! rotor being unable to follow the toggle, so the back-EMF is the same on
//! both sides of a transition and cancels in the difference.
//!
//! NOTHING HERE FEEDS GAIN SYNTHESIS YET. At the 20-period step the rotor
//! DOES follow (see [`RlCfg`]), so R and L come out biased; the run is
//! recorded, fitted and reported, and [`super::resistance`] remains the R
//! source the table is built from. `step_periods` is a parameter because
//! the fix is a step short enough to outrun the rotor, which needs a
//! firmware high-rate burst.
//!
//! Per transition:
//!   - tau from the normalised rise. The applied-duty register flips one
//!     sample BEFORE the current answers, so k = 0 is the first sample
//!     reporting the new level and the fit runs k = 1.. on the ENSEMBLE
//!     MEDIAN of every transition, x_k = 1 - A*a^k with A free (it
//!     absorbs the sub-sample phase of the step).
//!   - R_loop = delta_i / delta(D*V) over the settled windows either side,
//!     D = |duty_q15|/32767 and V the ON-state terminal difference. Both
//!     terminal referenced (the winding) and rail referenced (the winding
//!     plus the bridge) come out of the same transitions.
//!   - L = tau * R_loop, terminal referenced.
//!
//! Gates on every run: a null chain (both halves the same duty) must show
//! no step, forward/reverse and the two bias levels must agree, and a soft
//! supply is flagged - crest-sampled volt-seconds over-read the drive on a
//! high source impedance and R reads high with them, while tau is immune.
//! The gates cannot see the rotor-follow bias, which is why they are not
//! enough to promote this to the R source.

use super::{Cmd, Experiment, RigParams};
use crate::fitmath::{linear_ls, median, origin_ls, quantile};
use crate::frame::{
    TEL_BIT_CURRENT_RAW, TEL_BIT_DUTY, TEL_BIT_POS, TEL_BIT_VBUS_RAW, TEL_BIT_VMOTOR_A,
    TEL_BIT_VMOTOR_B, TelFrame, TelemetrySnapshot,
};
use crate::regs::control;
use crate::units::{self, SenseParams};

/// Measurements only: pot, applied duty, raw current, both terminals and
/// the raw rail. Six fields is the wire budget at 20 kHz.
pub const TEL_RL_MASK: u16 = TEL_BIT_POS
    | TEL_BIT_DUTY
    | TEL_BIT_CURRENT_RAW
    | TEL_BIT_VMOTOR_A
    | TEL_BIT_VMOTOR_B
    | TEL_BIT_VBUS_RAW;

const Q15: f64 = 32767.0;

/// Mid travel with no soft guard configured: the pot's own midpoint.
const POT_MID: u16 = 2048;

/// How far below the pooled R the bracket opens on a soft supply. Measured
/// on a 1.3 ohm USB source: at matched D x V_crest the winding draws 17.5%
/// less than on a stiff pack, so the crest-sampled drive - and R with it -
/// reads that much high.
const SOFT_SUPPLY_R_LOW: f64 = 0.82;

/// Count -> SI scales for the three channels this experiment reads. All
/// three reference VDD through the same 12-bit converter.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Scales {
    pub amps_per_count: f64,
    /// Volts per count of TERMINAL DIFFERENCE (vmotor_a - vmotor_b). Both
    /// taps return to one bias node, so the difference is bias free and
    /// scales by the plain divider ratio.
    pub v_term_per_count: f64,
    pub v_rail_per_count: f64,
}

impl Scales {
    /// None if any front-end scale is degenerate: a zero scale would turn
    /// R silently into 0 or infinity instead of failing.
    pub fn from_sense(sense: &SenseParams, vbus_div_top: u16, vbus_div_bot: u16) -> Option<Self> {
        let sc = Self {
            amps_per_count: units::amps_per_count(sense),
            v_term_per_count: units::volts_per_count(sense),
            v_rail_per_count: units::div_volts_per_count(sense, vbus_div_top, vbus_div_bot),
        };
        (sc.amps_per_count > 0.0 && sc.v_term_per_count > 0.0 && sc.v_rail_per_count > 0.0)
            .then_some(sc)
    }

    /// Ohms -> the control table's vcounts per ccount.
    pub fn r_vpc(&self, r_ohm: f64) -> f64 {
        r_ohm * self.amps_per_count / self.v_term_per_count
    }
}

/// What a captured burst was for. `Rest` is the torque-off baseline that
/// pins the current-sense bias; `Probe` sizes the toggle amplitude.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SegKind {
    Rest,
    Probe,
    Toggle,
    Null,
}

impl SegKind {
    pub fn as_str(self) -> &'static str {
        match self {
            SegKind::Rest => "rest",
            SegKind::Probe => "probe",
            SegKind::Toggle => "toggle",
            SegKind::Null => "null",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        [
            SegKind::Rest,
            SegKind::Probe,
            SegKind::Toggle,
            SegKind::Null,
        ]
        .into_iter()
        .find(|k| k.as_str() == s)
    }
}

/// One captured burst: what was commanded, and the frames it returned.
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    pub chain: u32,
    pub kind: SegKind,
    pub dir: i8,
    /// Bias level index within the chain plan.
    pub bias: u8,
    pub cmd_duty_q15: i16,
    pub tel: Vec<TelFrame>,
}

/// Fit knobs, shared by the live fit and the offline refit.
#[derive(Copy, Clone, Debug)]
pub struct RlFitCfg {
    pub tick_hz: f64,
    /// Samples averaged at the tail of a step for its settled level.
    pub tail: usize,
    /// Rise samples fitted, k = 1..=k_fit.
    pub k_fit: usize,
    /// Smallest settled current step a transition may carry, counts.
    pub step_min_counts: f64,
    /// Transitions needed before the fit is allowed to mean anything.
    pub min_transitions: usize,
    /// Null-chain gate: the median step must stay under this, counts.
    pub null_max_counts: f64,
    /// Source impedance above this flags a soft supply, ohms.
    pub soft_supply_ohm: f64,
    /// Largest relative spread between fwd/rev, the two bias levels, and
    /// up/down steps before the run refuses to feed gain synthesis.
    pub agree_tol: f64,
}

impl Default for RlFitCfg {
    fn default() -> Self {
        Self {
            tick_hz: 20_100.0,
            tail: 6,
            k_fit: 10,
            step_min_counts: 20.0,
            min_transitions: 8,
            null_max_counts: 4.0,
            soft_supply_ohm: 0.5,
            agree_tol: 0.15,
        }
    }
}

/// The toggle plan. `step_periods` is the load-bearing one: at the 20-period
/// (1 ms, 500 Hz) default the OUTPUT SHAFT still tracks the toggle - the pot
/// shows its speed modulating 30 to 50% across the two halves (2.3 counts of
/// travel per low step against 1.2 per high step at 15/25%), so the back-EMF
/// toggles with the duty instead of cancelling. It lands straight in the
/// differential slope: R reads 8 to 9 ohm against 3.9 locked, easing to 5.5
/// at 60 to 70% bias, and tau reads 130 us against 154. Only a step the
/// rotor cannot follow fixes it.
pub struct RlCfg {
    /// One toggle step, in PWM periods (one TEL sample each).
    pub step_periods: u16,
    pub steps_per_chain: u32,
    /// Probe duties, percent of full scale - the duty -> current gain the
    /// amplitude planner needs. Both must clear the servo's current-window
    /// floor (i_window_min_ticks): under it the shunt sample falls outside
    /// the ON phase, window_valid goes 0 and the rung reads nothing.
    /// Bench: 10% gave 0 valid windows of 20, 20% gave 19.
    pub probe_pct: (u8, u8),
    pub probe_steps: u32,
    pub null_steps: u32,
    /// Winding current the toggles stay inside, amps.
    pub i_band_a: (f64, f64),
    /// Smallest settled current step to aim for, counts.
    pub step_target_counts: f64,
    /// Duty the planner may ask for, percent of full scale.
    pub duty_pct_limits: (u8, u8),
    /// Half width of the centre band the chains launch from, counts.
    pub centre_margin: u16,
    pub seek_duty_q15: i16,
    pub seek_poll_ms: u32,
    pub seek_cap_polls: u32,
    pub settle_ms: u32,
    pub rest_ms: u32,
    pub baseline_ms: u32,
    pub fit: RlFitCfg,
}

impl Default for RlCfg {
    fn default() -> Self {
        Self {
            step_periods: 20,
            steps_per_chain: 40,
            probe_pct: (25, 40),
            probe_steps: 6,
            null_steps: 20,
            i_band_a: (0.10, 0.35),
            step_target_counts: 40.0,
            duty_pct_limits: (4, 60),
            centre_margin: 300,
            seek_duty_q15: 8520,
            seek_poll_ms: 25,
            seek_cap_polls: 400,
            settle_ms: 200,
            rest_ms: 300,
            baseline_ms: 50,
            fit: RlFitCfg::default(),
        }
    }
}

/// One pass/fail check the numbers had to clear.
#[derive(Clone, Debug)]
pub struct Gate {
    pub name: &'static str,
    pub pass: bool,
    pub detail: String,
}

#[derive(Clone, Debug)]
pub struct RlResult {
    /// Terminal-referenced loop resistance, ohms (free intercept).
    pub r_ohm: f64,
    pub r_origin_ohm: f64,
    /// Rail referenced: the same winding plus the bridge.
    pub r_rail_ohm: f64,
    pub r_bracket: (f64, f64),
    /// R in the control table's vcounts per ccount.
    pub r_vpc: f64,
    /// Free-intercept offset of the pooled delta fit, volts. A settled
    /// step pair has none, so a large value means the windows were not
    /// settled or the terminals do not report the drive.
    pub v0_volts: f64,
    pub r2: f64,
    pub r_fwd: Option<f64>,
    pub r_rev: Option<f64>,
    pub r_bias_lo: Option<f64>,
    pub r_bias_hi: Option<f64>,
    pub r_up: Option<f64>,
    pub r_down: Option<f64>,
    pub tau_us: f64,
    pub tau_bracket: (f64, f64),
    pub l_henries: f64,
    pub l_bracket: (f64, f64),
    /// Rail sag per amp across the chains, ohms.
    pub src_ohm: f64,
    pub supply_soft: bool,
    pub bias_counts: f64,
    pub null_step_counts: Option<f64>,
    pub transitions: usize,
    pub gates: Vec<Gate>,
    /// Every gate passed. Advisory: the gates do not test whether the
    /// rotor followed the toggle, so nothing consumes this yet.
    pub ok: bool,
    pub warnings: Vec<String>,
}

// --- experiment -------------------------------------------------------------

/// One planned chain: the duty the bursts walk through, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Chain {
    pub kind: SegKind,
    pub dir: i8,
    pub bias: u8,
    pub duties: Vec<i16>,
}

/// The probe chain reduced to a duty -> settled-current model.
struct ProbeModel {
    /// Counts per q15 of duty.
    k: f64,
    /// The anchor point the model passes through.
    d0: f64,
    i0: f64,
    /// Lowest probe duty the servo called a VALID drive window. Below it
    /// the shunt sample sits outside the ON phase, every settled read
    /// comes back empty, and a chain planned there measures nothing.
    duty_floor: i16,
}

/// Fit the probe. The gain comes from the alternating transitions, whose
/// up and down halves carry the spin-up drift with opposite sign, so
/// averaging the two medians cancels it. A probe that produced only one
/// usable duty falls back to the single-point ratio - good enough to SIZE
/// an amplitude, never reported as a slope.
fn probe_model(
    segs: &[Segment],
    sc: &Scales,
    cfg: &RlFitCfg,
    notes: &mut Vec<String>,
) -> Option<ProbeModel> {
    let Some(bias) = rest_bias(segs) else {
        notes.push("no torque-off baseline burst: current-sense bias unknown".into());
        return None;
    };
    let (tail, _) = budget(segs, cfg);
    let pts: Vec<(&Segment, Option<Settled>)> = segs
        .iter()
        .filter(|s| s.kind == SegKind::Probe)
        .map(|s| (s, settled(s, bias, sc, tail)))
        .collect();
    let live: Vec<(f64, f64)> = pts
        .iter()
        .filter_map(|(s, w)| w.map(|w| (s.cmd_duty_q15 as f64, w.i)))
        .collect();
    if live.is_empty() {
        notes.push(format!(
            "no probe burst produced {tail} valid drive windows (duty under the servo's \
             current-window floor?); raise probe_pct"
        ));
        return None;
    }
    let (mut up, mut down) = (Vec::new(), Vec::new());
    for w in pts.windows(2) {
        let (Some(a), Some(b)) = (w[0].1, w[1].1) else {
            continue;
        };
        let dd = (w[1].0.cmd_duty_q15 - w[0].0.cmd_duty_q15) as f64;
        if dd > 0.0 {
            up.push((b.i - a.i) / dd);
        } else if dd < 0.0 {
            down.push((b.i - a.i) / dd);
        }
    }
    let anchor = live
        .iter()
        .copied()
        .fold((0.0, 0.0), |acc, p| if p.0 > acc.0 { p } else { acc });
    let k = match (median(&up), median(&down)) {
        (Some(u), Some(d)) => (u + d) / 2.0,
        (Some(v), None) | (None, Some(v)) => v,
        (None, None) => {
            notes.push(format!(
                "probe transitions unusable ({} of {} bursts had a valid window); sizing off \
                 the single point {:.0} counts at {} q15",
                live.len(),
                pts.len(),
                anchor.1,
                anchor.0
            ));
            anchor.1 / anchor.0
        }
    };
    if k <= 0.0 || !k.is_finite() {
        notes.push(format!("probe duty-to-current gain {k:.5} is not positive"));
        return None;
    }
    Some(ProbeModel {
        k,
        d0: anchor.0,
        i0: anchor.1,
        duty_floor: live.iter().map(|p| p.0).fold(f64::MAX, f64::min) as i16,
    })
}

/// Size the toggle chains from a recorded rest + probe capture: two bias
/// levels x both directions, then the null control. Pure, so a bench
/// capture replays through exactly what ran on the rig.
///
/// The amplitude is sized to what the duty limits can actually REACH. A
/// free shaft on a soft rail never draws the top of `i_band_a` - back-EMF
/// eats the drive - so the band is lowered onto the reachable span rather
/// than dropping the chain, and the ceiling of `i_band_a` still caps it.
pub fn plan_toggles(segs: &[Segment], sc: &Scales, cfg: &RlCfg) -> (Vec<Chain>, Vec<String>) {
    let mut notes = Vec::new();
    let Some(m) = probe_model(segs, sc, &cfg.fit, &mut notes) else {
        return (Vec::new(), notes);
    };
    let dmin = pct_q15(cfg.duty_pct_limits.0).max(m.duty_floor) as f64;
    let dmax = pct_q15(cfg.duty_pct_limits.1) as f64;
    if dmax <= dmin {
        notes.push(format!(
            "duty ceiling {dmax:.0} q15 is at or under the usable floor {dmin:.0} q15"
        ));
        return (Vec::new(), notes);
    }
    let i_at = |d: f64| m.i0 + (d - m.d0) * m.k;
    let duty_at = |i: f64| (m.d0 + (i - m.i0) / m.k).clamp(dmin, dmax) as i16;
    let want = (
        cfg.i_band_a.0 / sc.amps_per_count,
        cfg.i_band_a.1 / sc.amps_per_count,
    );
    let reach = (i_at(dmin), i_at(dmax));
    let (mut lo, mut hi) = (want.0.max(reach.0), want.1.min(reach.1));
    if hi - lo < cfg.fit.step_min_counts {
        lo = reach.0;
        hi = reach.1.min(want.1);
        notes.push(format!(
            "current band {:.0}..{:.0} counts out of reach inside {}..{}% duty; using the \
             reachable {:.0}..{:.0}",
            want.0, want.1, cfg.duty_pct_limits.0, cfg.duty_pct_limits.1, lo, hi
        ));
    }
    if hi - lo < cfg.fit.step_min_counts {
        notes.push(format!(
            "the whole duty range buys only {:.0} counts of current step (need {:.0}); \
             nothing to toggle",
            hi - lo,
            cfg.fit.step_min_counts
        ));
        return (Vec::new(), notes);
    }
    let di = ((hi - lo) / 3.0).max(cfg.step_target_counts).min(hi - lo);
    let mut levels = vec![(lo, lo + di)];
    if hi - di > lo + di / 2.0 {
        levels.push((hi - di, hi));
    } else {
        notes.push("reachable span fits one bias level only; linearity check dropped".into());
    }
    let mut out: Vec<Chain> = Vec::new();
    let mut null_duty = None;
    for (bias, (blo, bhi)) in levels.into_iter().enumerate() {
        let (dlo, dhi) = (duty_at(blo), duty_at(bhi));
        let step = (dhi - dlo) as f64 * m.k;
        if step < cfg.fit.step_min_counts {
            notes.push(format!(
                "bias {bias}: duty span {dlo}..{dhi} q15 buys {step:.0} counts, chain dropped"
            ));
            continue;
        }
        null_duty = Some(dhi);
        for dir in [1i8, -1] {
            out.push(Chain {
                kind: SegKind::Toggle,
                dir,
                bias: bias as u8,
                duties: (0..cfg.steps_per_chain)
                    .map(|n| dir as i16 * if n.is_multiple_of(2) { dlo } else { dhi })
                    .collect(),
            });
        }
    }
    if let Some(d) = null_duty {
        out.push(Chain {
            kind: SegKind::Null,
            dir: 1,
            bias: 0,
            duties: vec![d; cfg.null_steps as usize],
        });
    }
    (out, notes)
}

enum Phase {
    ModeWrite,
    TelMask,
    SeekTorqueOn,
    SeekRead,
    SeekEval,
    SeekWait,
    Settle,
    BaselineOff,
    BaselineStream,
    ChainTorqueOn,
    ChainStream,
    ChainOff,
    ChainRead,
    ChainTorqueOff,
    ChainPause,
    FinishDuty,
    MaskOff,
    FinishTorque,
    Finished,
}

pub struct Rl {
    cfg: RlCfg,
    sc: Scales,
    band: (u16, u16),
    phase: Phase,
    plan: Vec<Chain>,
    planned: bool,
    baselined: bool,
    chain: usize,
    chain_id: u32,
    step: usize,
    polls: u32,
    /// Metadata of the burst in flight, claimed by `push_tel`.
    armed: Option<(SegKind, i8, u8, i16)>,
    segs: Vec<Segment>,
    warnings: Vec<String>,
}

impl Rl {
    pub fn new(cfg: RlCfg, params: &RigParams, sc: Scales) -> Self {
        let mid = params.pos_guard.map_or(POT_MID, |(lo, hi)| (lo + hi) / 2);
        let band = (
            mid.saturating_sub(cfg.centre_margin),
            mid.saturating_add(cfg.centre_margin),
        );
        let probe = Chain {
            kind: SegKind::Probe,
            dir: 1,
            bias: 0,
            duties: (0..cfg.probe_steps)
                .map(|k| {
                    let pct = if k.is_multiple_of(2) {
                        cfg.probe_pct.0
                    } else {
                        cfg.probe_pct.1
                    };
                    pct_q15(pct)
                })
                .collect(),
        };
        Self {
            cfg,
            sc,
            band,
            phase: Phase::ModeWrite,
            plan: vec![probe],
            planned: false,
            baselined: false,
            chain: 0,
            chain_id: 0,
            step: 0,
            polls: 0,
            armed: None,
            segs: Vec::new(),
            warnings: Vec::new(),
        }
    }

    pub fn segments(&self) -> &[Segment] {
        &self.segs
    }

    /// Notes from the run itself (planner mostly). Worth printing even
    /// when [`Rl::fit`] comes back None - that is when they explain why.
    pub fn warnings(&self) -> &[String] {
        &self.warnings
    }

    pub fn fit(&self) -> Option<RlResult> {
        let mut r = fit_segments(&self.segs, &self.sc, &self.cfg.fit)?;
        r.warnings.splice(0..0, self.warnings.iter().cloned());
        Some(r)
    }

    fn samples(&self, ms: u32) -> u16 {
        let n = ms as f64 * self.cfg.fit.tick_hz / 1000.0;
        n.clamp(1.0, u16::MAX as f64) as u16
    }

    fn arm(&mut self, kind: SegKind, dir: i8, bias: u8, duty: i16) -> Cmd {
        self.armed = Some((kind, dir, bias, duty));
        Cmd::Stream {
            samples: self.cfg.step_periods.max(1),
            goal: Some((control::GOAL_DUTY, duty as i32)),
        }
    }

    /// Size the toggles off the probe and append the measurement chains.
    fn plan_chains(&mut self) {
        let (chains, notes) = plan_toggles(&self.segs, &self.sc, &self.cfg);
        self.warnings.extend(notes);
        self.plan.extend(chains);
    }

    fn advance_chain(&mut self) {
        self.chain += 1;
        self.chain_id += 1;
        if self.chain == self.plan.len() && !self.planned {
            self.planned = true;
            self.plan_chains();
        }
        self.phase = if self.chain < self.plan.len() {
            Phase::SeekTorqueOn
        } else {
            Phase::FinishDuty
        };
    }
}

fn pct_q15(pct: u8) -> i16 {
    (pct as i32 * Q15 as i32 / 100) as i16
}

impl Experiment for Rl {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::ModeWrite => {
                self.phase = Phase::TelMask;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            Phase::TelMask => {
                self.phase = Phase::SeekTorqueOn;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: TEL_RL_MASK as i32,
                }
            }
            Phase::SeekTorqueOn => {
                self.phase = Phase::SeekRead;
                self.polls = 0;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => {
                let pos = obs.map(|o| o.pos).unwrap_or(POT_MID);
                self.polls += 1;
                if (self.band.0..=self.band.1).contains(&pos) {
                    self.phase = Phase::Settle;
                    return Cmd::Write {
                        reg: control::GOAL_DUTY,
                        value: 0,
                    };
                }
                if self.polls >= self.cfg.seek_cap_polls {
                    self.warnings
                        .push(format!("seek stuck at pos {pos}; run cut short"));
                    self.phase = Phase::FinishDuty;
                    return Cmd::Pause { ms: 0 };
                }
                self.phase = Phase::SeekWait;
                let duty = if pos < self.band.0 {
                    self.cfg.seek_duty_q15
                } else {
                    -self.cfg.seek_duty_q15
                };
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: duty as i32,
                }
            }
            Phase::SeekWait => {
                self.phase = Phase::SeekRead;
                Cmd::Pause {
                    ms: self.cfg.seek_poll_ms,
                }
            }
            Phase::Settle => {
                self.phase = if self.baselined {
                    Phase::ChainTorqueOn
                } else {
                    Phase::BaselineOff
                };
                Cmd::Pause {
                    ms: self.cfg.settle_ms,
                }
            }
            Phase::BaselineOff => {
                self.phase = Phase::BaselineStream;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::BaselineStream => {
                self.phase = Phase::ChainTorqueOn;
                self.baselined = true;
                self.armed = Some((SegKind::Rest, 0, 0, 0));
                Cmd::Stream {
                    samples: self.samples(self.cfg.baseline_ms),
                    goal: None,
                }
            }
            Phase::ChainTorqueOn => {
                self.phase = Phase::ChainStream;
                self.step = 0;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::ChainStream => {
                match self
                    .plan
                    .get(self.chain)
                    .and_then(|c| c.duties.get(self.step).map(|d| (c.kind, c.dir, c.bias, *d)))
                {
                    Some((kind, dir, bias, duty)) => {
                        self.step += 1;
                        self.arm(kind, dir, bias, duty)
                    }
                    None => {
                        self.phase = Phase::ChainOff;
                        Cmd::Pause { ms: 0 }
                    }
                }
            }
            Phase::ChainOff => {
                self.phase = Phase::ChainRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::ChainRead => {
                self.phase = Phase::ChainTorqueOff;
                Cmd::Read
            }
            Phase::ChainTorqueOff => {
                self.phase = Phase::ChainPause;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::ChainPause => {
                self.advance_chain();
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::FinishDuty => {
                self.phase = Phase::MaskOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::MaskOff => {
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

    fn push_tel(&mut self, frames: &[TelFrame]) {
        if let Some((kind, dir, bias, cmd_duty_q15)) = self.armed.take() {
            self.segs.push(Segment {
                chain: self.chain_id,
                kind,
                dir,
                bias,
                cmd_duty_q15,
                tel: frames.to_vec(),
            });
        }
    }
}

// --- fit --------------------------------------------------------------------

/// A step's settled operating point, averaged over its tail.
#[derive(Copy, Clone, Debug)]
struct Settled {
    /// Bias-subtracted current, counts.
    i: f64,
    /// Duty as applied, fraction of full scale.
    d: f64,
    /// ON-state terminal difference, volts (positive both directions).
    v_term: f64,
    /// Raw rail, volts.
    v_rail: f64,
}

impl Settled {
    /// Average volts the winding sees over a PWM period: duty x the
    /// ON-state terminal difference.
    fn drive(&self) -> f64 {
        self.d * self.v_term
    }

    fn drive_rail(&self) -> f64 {
        self.d * self.v_rail
    }
}

#[derive(Clone, Debug)]
struct Tr {
    dir: i8,
    bias: u8,
    up: bool,
    di_a: f64,
    ddv_term: f64,
    ddv_rail: f64,
    /// Normalised rise, index j = k - 1.
    x: Vec<f64>,
}

/// Settled-tail and rise budgets for the step actually captured: a short
/// step cannot spare six settled samples and ten rise samples, and the
/// offline refit only learns the step length from the frames.
fn budget(segs: &[Segment], cfg: &RlFitCfg) -> (usize, usize) {
    // Median, not minimum: a rung whose drive window the servo called
    // invalid contributes no samples at all, and letting it set the budget
    // starves every rung that did capture.
    let counts: Vec<f64> = segs
        .iter()
        .filter(|s| s.kind != SegKind::Rest)
        .map(|s| s.tel.iter().filter(|f| f.window_valid).count() as f64)
        .filter(|n| *n >= 2.0)
        .collect();
    let n = median(&counts).unwrap_or(0.0) as usize;
    let tail = cfg.tail.min(n / 3).max(1);
    (tail, cfg.k_fit.min(n.saturating_sub(tail + 1)))
}

/// Current-sense zero from the torque-off baseline, counts.
fn rest_bias(segs: &[Segment]) -> Option<f64> {
    let v: Vec<f64> = segs
        .iter()
        .filter(|s| s.kind == SegKind::Rest)
        .flat_map(|s| s.tel.iter())
        .filter_map(|f| f.current_raw.map(f64::from))
        .collect();
    median(&v)
}

fn settled(seg: &Segment, bias: f64, sc: &Scales, tail: usize) -> Option<Settled> {
    let live: Vec<&TelFrame> = seg.tel.iter().filter(|f| f.window_valid).collect();
    if live.len() < tail || tail == 0 {
        return None;
    }
    let sgn = if seg.cmd_duty_q15 >= 0 { 1.0 } else { -1.0 };
    let (mut i, mut d, mut vt, mut vr) = (0.0, 0.0, 0.0, 0.0);
    for f in &live[live.len() - tail..] {
        i += f.current_raw? as f64 - bias;
        d += (f.duty_q15? as f64).abs() / Q15;
        vt += (f.vmotor_a? as f64 - f.vmotor_b? as f64) * sgn * sc.v_term_per_count;
        vr += f.vbus_raw? as f64 * sc.v_rail_per_count;
    }
    let n = tail as f64;
    Some(Settled {
        i: i / n,
        d: d / n,
        v_term: vt / n,
        v_rail: vr / n,
    })
}

/// Normalised rise of the step, k = 1..=k_fit. k = 0 is the first sample
/// whose APPLIED duty reports the new level; the current answers from
/// k = 1, and taking k = 0 as the step instant drags tau ~15% low.
fn rise(seg: &Segment, bias: f64, pre: f64, post: f64, k_fit: usize) -> Option<Vec<f64>> {
    let denom = post - pre;
    let j = seg
        .tel
        .iter()
        .position(|f| f.duty_q15 == Some(seg.cmd_duty_q15) && f.window_valid)?;
    let mut x = Vec::with_capacity(k_fit);
    for k in 1..=k_fit {
        let f = seg.tel.get(j + k)?;
        x.push((f.current_raw? as f64 - bias - pre) / denom);
    }
    Some(x)
}

/// Profile the discrete pole of a normalised rise: x_k = 1 - A*a^k over
/// k = 1..=x.len() with A closed form at each a. None when the optimum
/// lands on an edge of the search - that is not a first-order rise.
fn fit_pole(x: &[f64]) -> Option<f64> {
    const LO: f64 = 0.05;
    const HI: f64 = 0.98;
    const GRID: usize = 200;
    if x.len() < 3 || !x.iter().all(|v| v.is_finite()) {
        return None;
    }
    let rss = |a: f64| -> f64 {
        let (mut sya, mut saa) = (0.0, 0.0);
        for (j, xk) in x.iter().enumerate() {
            let ak = a.powi(j as i32 + 1);
            sya += (1.0 - xk) * ak;
            saa += ak * ak;
        }
        if saa <= 0.0 {
            return f64::INFINITY;
        }
        let amp = sya / saa;
        x.iter()
            .enumerate()
            .map(|(j, xk)| ((1.0 - xk) - amp * a.powi(j as i32 + 1)).powi(2))
            .sum()
    };
    let step = (HI - LO) / GRID as f64;
    let mut best = (f64::INFINITY, LO);
    for g in 0..=GRID {
        let a = LO + step * g as f64;
        let r = rss(a);
        if r < best.0 {
            best = (r, a);
        }
    }
    if best.1 <= LO + step || best.1 >= HI - step {
        return None;
    }
    let phi = (5f64.sqrt() - 1.0) / 2.0;
    let (mut lo, mut hi) = (best.1 - step, best.1 + step);
    for _ in 0..40 {
        let x1 = hi - phi * (hi - lo);
        let x2 = lo + phi * (hi - lo);
        if rss(x1) < rss(x2) {
            hi = x2;
        } else {
            lo = x1;
        }
    }
    let a = (lo + hi) / 2.0;
    (a > 0.0 && a < 1.0).then_some(a)
}

fn tau_us_of(a: f64, tick_hz: f64) -> f64 {
    -1e6 / (tick_hz * a.ln())
}

/// delta_i on delta(D*V): the slope is 1/R, so R is its reciprocal.
/// Returns (free-intercept R, through-origin R, intercept volts, r2).
fn r_of(pts: &[(f64, f64)]) -> Option<(f64, f64, f64, f64)> {
    let free = linear_ls(pts)?;
    let orig = origin_ls(pts)?;
    if free.b <= 0.0 || orig.slope <= 0.0 {
        return None;
    }
    Some((1.0 / free.b, 1.0 / orig.slope, -free.a / free.b, free.r2))
}

fn r_subset(trs: &[Tr], keep: impl Fn(&Tr) -> bool) -> Option<f64> {
    let pts: Vec<(f64, f64)> = trs
        .iter()
        .filter(|t| keep(t))
        .map(|t| (t.ddv_term, t.di_a))
        .collect();
    r_of(&pts).map(|(r, _, _, _)| r)
}

fn spread(a: Option<f64>, b: Option<f64>, r: f64) -> Option<f64> {
    match (a, b) {
        (Some(a), Some(b)) if r > 0.0 => Some((a - b).abs() / r),
        _ => None,
    }
}

fn gate(name: &'static str, pass: bool, detail: String) -> Gate {
    Gate { name, pass, detail }
}

/// Every number the experiment reports, from recorded segments alone - so
/// the offline refit and the live run cannot diverge.
pub fn fit_segments(segs: &[Segment], sc: &Scales, cfg: &RlFitCfg) -> Option<RlResult> {
    let bias = rest_bias(segs)?;
    let (tail, k_fit) = budget(segs, cfg);
    let mut trs: Vec<Tr> = Vec::new();
    let mut taus: Vec<f64> = Vec::new();
    let mut nulls: Vec<f64> = Vec::new();
    for w in segs.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        if a.chain != b.chain || a.kind != b.kind {
            continue;
        }
        let (Some(sa), Some(sb)) = (settled(a, bias, sc, tail), settled(b, bias, sc, tail)) else {
            continue;
        };
        let di = sb.i - sa.i;
        match b.kind {
            SegKind::Null => nulls.push(di),
            SegKind::Toggle if di.abs() >= cfg.step_min_counts => {
                let x = rise(b, bias, sa.i, sb.i, k_fit);
                if let Some(x) = &x
                    && let Some(a) = fit_pole(x)
                {
                    taus.push(tau_us_of(a, cfg.tick_hz));
                }
                trs.push(Tr {
                    dir: b.dir,
                    bias: b.bias,
                    up: di > 0.0,
                    di_a: di * sc.amps_per_count,
                    ddv_term: sb.drive() - sa.drive(),
                    ddv_rail: sb.drive_rail() - sa.drive_rail(),
                    x: x.unwrap_or_default(),
                });
            }
            _ => {}
        }
    }

    let pts: Vec<(f64, f64)> = trs.iter().map(|t| (t.ddv_term, t.di_a)).collect();
    let (r_ohm, r_origin_ohm, v0_volts, r2) = r_of(&pts)?;
    let rail: Vec<(f64, f64)> = trs.iter().map(|t| (t.ddv_rail, t.di_a)).collect();
    let r_rail_ohm = r_of(&rail).map(|(r, _, _, _)| r).unwrap_or(0.0);

    // ensemble median rise, one median per k over every transition
    let mut ens = Vec::with_capacity(k_fit);
    for k in 0..k_fit {
        let col: Vec<f64> = trs.iter().filter_map(|t| t.x.get(k).copied()).collect();
        match median(&col) {
            Some(m) => ens.push(m),
            None => break,
        }
    }
    let tau_us = fit_pole(&ens)
        .map(|a| tau_us_of(a, cfg.tick_hz))
        .unwrap_or(0.0);
    let tau_bracket = match (quantile(&taus, 0.16), quantile(&taus, 0.84)) {
        (Some(lo), Some(hi)) => (lo, hi),
        _ => (tau_us, tau_us),
    };

    // Settled tails only: a bulk-capped rail does not track the winding's
    // own transient, so pairing mid-rise current with the rail reads the
    // source soft-pedalled.
    let src_pts: Vec<(f64, f64)> = segs
        .iter()
        .filter(|s| matches!(s.kind, SegKind::Toggle | SegKind::Null))
        .filter_map(|s| settled(s, bias, sc, tail))
        .map(|s| (s.i * sc.amps_per_count, s.v_rail))
        .collect();
    let src_ohm = linear_ls(&src_pts).map(|f| -f.b).unwrap_or(0.0);
    let supply_soft = src_ohm > cfg.soft_supply_ohm;

    let r_fwd = r_subset(&trs, |t| t.dir > 0);
    let r_rev = r_subset(&trs, |t| t.dir < 0);
    let r_bias_lo = r_subset(&trs, |t| t.bias == 0);
    let r_bias_hi = r_subset(&trs, |t| t.bias == 1);
    let r_up = r_subset(&trs, |t| t.up);
    let r_down = r_subset(&trs, |t| !t.up);

    let mut lo = r_ohm.min(r_origin_ohm);
    let mut hi = r_ohm.max(r_origin_ohm);
    for v in [r_fwd, r_rev, r_bias_lo, r_bias_hi, r_up, r_down]
        .into_iter()
        .flatten()
    {
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if supply_soft {
        lo *= SOFT_SUPPLY_R_LOW;
    }
    let l_henries = tau_us * 1e-6 * r_ohm;
    let l_bracket = (tau_bracket.0 * 1e-6 * lo, tau_bracket.1 * 1e-6 * hi);

    // A null chain drifts (the shaft is accelerating) but must not STEP.
    // Half the difference of consecutive deltas is the alternating part:
    // a locally linear drift cancels, a real step survives at full size.
    let alt: Vec<f64> = nulls
        .windows(2)
        .map(|w| (w[0] - w[1]).abs() / 2.0)
        .collect();
    let null_step_counts = median(&alt);
    let dirs = spread(r_fwd, r_rev, r_ohm);
    let biases = spread(r_bias_lo, r_bias_hi, r_ohm);
    let updown = spread(r_up, r_down, r_ohm);
    let agrees = |name: &'static str, v: Option<f64>| match v {
        Some(v) => gate(name, v <= cfg.agree_tol, format!("{:.1}% apart", v * 100.0)),
        None => gate(name, false, "one side missing".into()),
    };
    let gates = vec![
        gate(
            "transitions",
            trs.len() >= cfg.min_transitions,
            format!("{} usable", trs.len()),
        ),
        gate(
            "pole",
            tau_us.is_finite() && tau_us > 0.0,
            format!("tau {tau_us:.1} us"),
        ),
        match null_step_counts {
            Some(n) => gate(
                "null",
                n <= cfg.null_max_counts,
                format!("{n:.1} counts of step"),
            ),
            None => gate("null", false, "no null chain".into()),
        },
        agrees("fwd-rev", dirs),
        agrees("bias", biases),
        agrees("up-down", updown),
    ];
    let ok = gates.iter().all(|g| g.pass);

    Some(RlResult {
        r_ohm,
        r_origin_ohm,
        r_rail_ohm,
        r_bracket: (lo, hi),
        r_vpc: sc.r_vpc(r_ohm),
        v0_volts,
        r2,
        r_fwd,
        r_rev,
        r_bias_lo,
        r_bias_hi,
        r_up,
        r_down,
        tau_us,
        tau_bracket,
        l_henries,
        l_bracket,
        src_ohm,
        supply_soft,
        bias_counts: bias,
        null_step_counts,
        transitions: trs.len(),
        gates,
        ok,
        warnings: if supply_soft {
            vec![format!(
                "supply source impedance {src_ohm:.2} ohm: R reads high, bracket widened (tau is immune)"
            )]
        } else {
            Vec::new()
        },
    })
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, pump};
    use super::super::{Guarded, RigParams};
    use super::*;
    use std::collections::BTreeSet;

    /// Bodged board D as fitted: 60 mohm shunt, G 15, 6k8/3k3 terminal
    /// taps, 15k/10k rail tap.
    const BOARD_D: SenseParams = SenseParams {
        shunt_r_mohm: 60,
        gain_milli: 15_000,
        vmotor_div_top: 6_800,
        vmotor_div_bot: 3_300,
        vdd_mv: 3_300,
        tick_hz: 20_100,
    };
    const VBUS_DIV: (u16, u16) = (15_000, 10_000);

    /// The fake servo's count-domain plant (r_vpc 3.37) is the rev-2A
    /// front end; e2e assertions ride on that profile.
    const REV_2A: SenseParams = SenseParams {
        shunt_r_mohm: 33,
        gain_milli: 15_000,
        vmotor_div_top: 18_200,
        vmotor_div_bot: 10_000,
        vdd_mv: 3_300,
        tick_hz: 20_100,
    };

    struct Plant {
        r: f64,
        l: f64,
        /// Fixed series drop (bridge + brushes), volts.
        v0: f64,
        /// Open-circuit rail and its source impedance.
        voc: f64,
        rs: f64,
        /// Bridge resistance between the rail tap and the motor pins.
        rb: f64,
        /// Back-EMF ceiling of the slow mechanical rise, volts.
        bemf: f64,
        noise: f64,
    }

    impl Default for Plant {
        fn default() -> Self {
            Self {
                r: 4.0,
                l: 0.6e-3,
                v0: 0.4,
                voc: 8.0,
                rs: 0.27,
                rb: 0.28,
                bemf: 0.3,
                noise: 2.0,
            }
        }
    }

    #[derive(Copy, Clone)]
    struct Tag {
        chain: u32,
        kind: SegKind,
        dir: i8,
        bias: u8,
    }

    struct Synth {
        i: f64,
        t: f64,
        lcg: u64,
        bias: f64,
        sc: Scales,
        tick_hz: f64,
    }

    impl Synth {
        fn new(sc: Scales) -> Self {
            Self {
                i: 0.0,
                t: 0.0,
                lcg: 0x2545F4914F6CDD1D,
                bias: 512.0,
                sc,
                tick_hz: 20_100.0,
            }
        }

        fn noise(&mut self, scale: f64) -> f64 {
            self.lcg = self
                .lcg
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * scale
        }

        fn rest(&mut self, chain: u32, samples: usize) -> Segment {
            let tel = (0..samples)
                .map(|k| TelFrame {
                    tick: k as u64,
                    window_valid: false,
                    pos: Some(2048),
                    duty_q15: Some(0),
                    current_raw: Some((self.bias + self.noise(2.0)).round() as u16),
                    vmotor_a: Some(775),
                    vmotor_b: Some(775),
                    vbus_raw: Some(2000),
                    ..Default::default()
                })
                .collect();
            Segment {
                chain,
                kind: SegKind::Rest,
                dir: 0,
                bias: 0,
                cmd_duty_q15: 0,
                tel,
            }
        }

        /// One step: `samples` ticks at `duty`, current continuing from
        /// the previous step. The duty register reports the new level from
        /// k = 0 and the current answers from k = 1. Rail and terminals
        /// sag at the step's SETTLED current and hold there - 100 uF of
        /// bulk does not track a 150 us winding transient.
        fn step(&mut self, tag: Tag, duty: i16, p: &Plant, samples: usize) -> Segment {
            let dt = 1.0 / self.tick_hz;
            let a = (-dt / (p.l / p.r)).exp();
            let d = (duty as f64).abs() / Q15;
            let sgn = if duty >= 0 { 1.0 } else { -1.0 };
            let e = p.bemf * (1.0 - (-self.t / 0.022).exp());
            let iss = ((d * p.voc - p.v0 - e) / (p.r + d * (p.rs + p.rb))).max(0.0);
            let rail = p.voc - iss * p.rs;
            let term = rail - iss * p.rb;
            let vb = 775.0;
            let tap = term / self.sc.v_term_per_count;
            let mut tel = Vec::with_capacity(samples);
            for k in 0..samples {
                if k > 0 {
                    self.i = iss + (self.i - iss) * a;
                }
                self.t += dt;
                let ic = self.i / self.sc.amps_per_count;
                tel.push(TelFrame {
                    tick: k as u64,
                    window_valid: true,
                    pos: Some(2048),
                    duty_q15: Some(duty),
                    current_raw: Some((self.bias + ic + self.noise(p.noise)).round() as u16),
                    vmotor_a: Some(if sgn > 0.0 { vb + tap } else { vb } as u16),
                    vmotor_b: Some(if sgn > 0.0 { vb } else { vb + tap } as u16),
                    vbus_raw: Some((rail / self.sc.v_rail_per_count).round() as u16),
                    ..Default::default()
                });
            }
            Segment {
                chain: tag.chain,
                kind: tag.kind,
                dir: tag.dir,
                bias: tag.bias,
                cmd_duty_q15: duty,
                tel,
            }
        }

        fn chain(&mut self, tag: Tag, duties: &[i16], p: &Plant) -> Vec<Segment> {
            duties.iter().map(|&d| self.step(tag, d, p, 20)).collect()
        }
    }

    /// A full run's segments: baseline, two bias levels x both
    /// directions, then the null control.
    fn synth_run(p: &Plant, sc: Scales, null_duty_step: i16) -> Vec<Segment> {
        let mut s = Synth::new(sc);
        let mut segs = vec![s.rest(0, 200)];
        let levels = [(pct_q15(10), pct_q15(18)), (pct_q15(22), pct_q15(30))];
        let mut chain = 1;
        for (bias, (lo, hi)) in levels.into_iter().enumerate() {
            for dir in [1i8, -1] {
                let duties: Vec<i16> = (0..40)
                    .map(|k| dir as i16 * if k % 2 == 0 { lo } else { hi })
                    .collect();
                let tag = Tag {
                    chain,
                    kind: SegKind::Toggle,
                    dir,
                    bias: bias as u8,
                };
                segs.extend(s.chain(tag, &duties, p));
                chain += 1;
                s.t = 0.0;
            }
        }
        let null: Vec<i16> = (0..20)
            .map(|k| pct_q15(30) + if k % 2 == 0 { 0 } else { null_duty_step })
            .collect();
        let tag = Tag {
            chain,
            kind: SegKind::Null,
            dir: 1,
            bias: 0,
        };
        segs.extend(s.chain(tag, &null, p));
        segs
    }

    fn scales() -> Scales {
        Scales::from_sense(&BOARD_D, VBUS_DIV.0, VBUS_DIV.1).unwrap()
    }

    #[test]
    fn recovers_planted_r_tau_and_l() {
        let p = Plant::default();
        let sc = scales();
        let fit = fit_segments(&synth_run(&p, sc, 0), &sc, &RlFitCfg::default()).expect("fit");
        assert!(
            (fit.r_ohm - p.r).abs() / p.r < 0.02,
            "R {} vs {} (r2 {})",
            fit.r_ohm,
            p.r,
            fit.r2
        );
        let tau = p.l / p.r * 1e6;
        assert!(
            (fit.tau_us - tau).abs() / tau < 0.03,
            "tau {} vs {tau}",
            fit.tau_us
        );
        assert!(
            (fit.l_henries - p.l).abs() / p.l < 0.05,
            "L {} vs {}",
            fit.l_henries,
            p.l
        );
        assert!(fit.ok, "gates: {:?}", fit.gates);
        assert!(fit.transitions >= 100, "n {}", fit.transitions);
        assert!(!fit.supply_soft, "src {}", fit.src_ohm);
        // rail referenced carries the bridge on top of the winding
        assert!(fit.r_rail_ohm > fit.r_ohm, "rail {}", fit.r_rail_ohm);
        // the settled delta pair leaves no intercept behind
        assert!(fit.v0_volts.abs() < 0.05, "v0 {}", fit.v0_volts);
        assert!(fit.r_bracket.0 <= fit.r_ohm && fit.r_ohm <= fit.r_bracket.1);
    }

    #[test]
    fn null_chain_gates_the_run() {
        let p = Plant::default();
        let sc = scales();
        let clean = fit_segments(&synth_run(&p, sc, 0), &sc, &RlFitCfg::default()).unwrap();
        assert!(clean.null_step_counts.unwrap() < 2.0);
        assert!(clean.gates.iter().find(|g| g.name == "null").unwrap().pass);

        // a null chain that actually steps means the toggles were never
        // measuring what the chain said they were
        let dirty =
            fit_segments(&synth_run(&p, sc, pct_q15(4)), &sc, &RlFitCfg::default()).unwrap();
        assert!(dirty.null_step_counts.unwrap() > 4.0);
        assert!(!dirty.gates.iter().find(|g| g.name == "null").unwrap().pass);
        assert!(!dirty.ok);
    }

    #[test]
    fn soft_supply_flags_and_widens_the_bracket() {
        let sc = scales();
        let usb = Plant {
            rs: 1.3,
            ..Plant::default()
        };
        let fit = fit_segments(&synth_run(&usb, sc, 0), &sc, &RlFitCfg::default()).unwrap();
        assert!(fit.supply_soft, "src {}", fit.src_ohm);
        assert!(
            (fit.src_ohm - usb.rs).abs() < 0.15,
            "src {} vs {}",
            fit.src_ohm,
            usb.rs
        );
        assert!(
            fit.r_bracket.0 < fit.r_ohm * 0.9,
            "bracket {:?} not widened",
            fit.r_bracket
        );
        assert!(!fit.warnings.is_empty());
        // tau does not ride on the sampled volt-seconds
        let tau = usb.l / usb.r * 1e6;
        assert!(
            (fit.tau_us - tau).abs() / tau < 0.03,
            "tau {} vs {tau}",
            fit.tau_us
        );
    }

    #[test]
    fn r_vpc_matches_the_count_domain_sanity() {
        // gains.rs pins the same map: a 4.7 ohm winding on the rev-2A
        // front end is r_vpc ~3.37
        let sc = Scales::from_sense(&REV_2A, VBUS_DIV.0, VBUS_DIV.1).unwrap();
        assert!((sc.r_vpc(4.7) - 3.37).abs() < 0.01, "{}", sc.r_vpc(4.7));
        assert!(Scales::from_sense(&REV_2A, 15_000, 0).is_none());
    }

    fn run_e2e(servo: &mut FakeServo) -> (Rl, Vec<String>) {
        let params = RigParams::default();
        let sc = Scales::from_sense(&REV_2A, VBUS_DIV.0, VBUS_DIV.1).unwrap();
        let mut exp = Guarded::new(Rl::new(RlCfg::default(), &params, sc), params);
        let log = pump(&mut exp, servo, 500_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    #[test]
    fn drives_the_rig_safely_end_to_end() {
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        let (exp, log) = run_e2e(&mut servo);
        assert!(!log.contains(&"OVERRUN".to_string()));
        let torque_on = log
            .iter()
            .position(|l| l == "write torque_enable 1")
            .unwrap();
        let first_drive = log
            .iter()
            .position(|l| {
                (l.starts_with("write goal_duty") || l.starts_with("stream"))
                    && !l.ends_with(" 0")
                    && l != "write goal_duty 0"
            })
            .unwrap();
        assert!(torque_on < first_drive, "torque on before any drive");
        let tail: Vec<&String> = log.iter().rev().take(3).collect();
        assert_eq!(*tail[2], "write goal_duty 0");
        assert_eq!(*tail[1], "write tel_mask 0");
        assert_eq!(*tail[0], "write torque_enable 0");
        // a rest baseline, a probe chain, both directions at two biases,
        // and the null control
        let kinds: Vec<SegKind> = [
            SegKind::Rest,
            SegKind::Probe,
            SegKind::Toggle,
            SegKind::Null,
        ]
        .into_iter()
        .filter(|k| exp.segments().iter().any(|s| s.kind == *k))
        .collect();
        assert_eq!(kinds.len(), 4, "captured kinds {kinds:?}");
        assert_eq!(
            exp.segments()
                .iter()
                .filter(|s| s.kind == SegKind::Toggle)
                .map(|s| (s.dir, s.bias))
                .collect::<BTreeSet<_>>()
                .len(),
            4
        );
    }

    /// The first bench run's rest + probe capture (board D on a 4.3 V USB
    /// rail; the 1000-sample rest burst trimmed to its last 40 - the bias
    /// is a median). Its 10% probe rungs came back with no valid drive
    /// window at all, which is what stalled the planner.
    fn bench_probe_segments() -> Vec<Segment> {
        let csv = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/rl-probe-bench.csv"
        ));
        let mut out: Vec<Segment> = Vec::new();
        let mut cur = usize::MAX;
        for line in csv.lines().skip(1).filter(|l| !l.is_empty()) {
            let c: Vec<&str> = line.split(',').collect();
            let n = |i: usize| c[i].parse::<i64>().expect("number");
            if cur != n(0) as usize {
                cur = n(0) as usize;
                out.push(Segment {
                    chain: n(1) as u32,
                    kind: SegKind::parse(c[2]).expect("kind"),
                    dir: n(3) as i8,
                    bias: n(4) as u8,
                    cmd_duty_q15: n(5) as i16,
                    tel: Vec::new(),
                });
            }
            out.last_mut().expect("pushed").tel.push(TelFrame {
                tick: n(6) as u64,
                window_valid: c[7] == "1",
                pos: Some(n(8) as u16),
                duty_q15: Some(n(9) as i16),
                current_raw: Some(n(10) as u16),
                vmotor_a: Some(n(11) as u16),
                vmotor_b: Some(n(12) as u16),
                vbus_raw: Some(n(13) as u16),
                ..Default::default()
            });
        }
        out
    }

    #[test]
    fn plans_chains_from_the_bench_probe_capture() {
        let segs = bench_probe_segments();
        let sc = scales();
        // the stall itself: the 10% rungs carry no settled window to read
        let low_valid: usize = segs
            .iter()
            .filter(|s| s.kind == SegKind::Probe && s.cmd_duty_q15 == pct_q15(10))
            .map(|s| s.tel.iter().filter(|f| f.window_valid).count())
            .sum();
        assert!(low_valid <= 3, "10% rungs had {low_valid} valid windows");
        // board D scales against the capture: the rest rail is the 4.3 V USB one
        let rail = rest_bias(&segs)
            .map(|_| segs[0].tel.last().unwrap().vbus_raw.unwrap() as f64 * sc.v_rail_per_count);
        assert!(
            matches!(rail, Some(v) if (4.0..4.8).contains(&v)),
            "rail {rail:?}"
        );

        let cfg = RlCfg::default();
        let (chains, notes) = plan_toggles(&segs, &sc, &cfg);
        assert!(!chains.is_empty(), "no chains planned; notes {notes:?}");
        let toggles: Vec<&Chain> = chains
            .iter()
            .filter(|c| c.kind == SegKind::Toggle)
            .collect();
        assert!(toggles.iter().any(|c| c.dir > 0) && toggles.iter().any(|c| c.dir < 0));
        assert!(chains.iter().any(|c| c.kind == SegKind::Null), "null chain");
        // every duty inside the configured limits and above the floor the
        // probe discovered (the 10% rungs are below it)
        let ceiling = pct_q15(cfg.duty_pct_limits.1);
        for c in &chains {
            for d in &c.duties {
                let mag = d.unsigned_abs() as i16;
                assert!(
                    mag >= pct_q15(20) && mag <= ceiling,
                    "{:?} duty {d} outside the usable band",
                    c.kind
                );
            }
        }
        // and the run explains how it sized itself
        assert!(
            notes.iter().any(|n| n.contains("single point")),
            "notes {notes:?}"
        );
        // 0.35 A is out of reach on this rail, so the top level rides the
        // duty ceiling instead of the chain being dropped
        let top = chains
            .iter()
            .flat_map(|c| c.duties.iter())
            .map(|d| d.unsigned_abs())
            .max()
            .expect("duties");
        assert_eq!(top as i16, ceiling);
        assert_eq!(
            toggles
                .iter()
                .map(|c| c.bias)
                .collect::<BTreeSet<_>>()
                .len(),
            2,
            "two bias levels"
        );
    }

    #[test]
    fn step_periods_sizes_the_burst_and_the_fit_budget() {
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        let params = RigParams::default();
        let sc = Scales::from_sense(&REV_2A, VBUS_DIV.0, VBUS_DIV.1).unwrap();
        let cfg = RlCfg {
            step_periods: 8,
            ..RlCfg::default()
        };
        let mut exp = Guarded::new(Rl::new(cfg, &params, sc), params);
        let log = pump(&mut exp, &mut servo, 500_000);
        assert!(!log.contains(&"OVERRUN".to_string()));
        assert!(
            log.iter().any(|l| l.starts_with("stream 8 goal_duty")),
            "toggles arm 8 periods"
        );
        let exp = exp.into_inner();
        // the settled and rise budgets shrink with the step rather than
        // starving the fit
        assert_eq!(budget(exp.segments(), &RlFitCfg::default()), (2, 5));
        assert!(exp.fit().is_some(), "a short step still fits");
    }

    #[test]
    fn recovers_the_fake_rigs_count_domain_r() {
        // the fake plant is ohmic with no L, so tau has nothing to find;
        // R still comes straight out of the settled steps
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        let (exp, _) = run_e2e(&mut servo);
        let fit = exp.fit().expect("fit");
        assert!(
            (fit.r_vpc - 3.37).abs() < 0.1,
            "r_vpc {} (r_ohm {}, n {})",
            fit.r_vpc,
            fit.r_ohm,
            fit.transitions
        );
        assert!(fit.gates.iter().find(|g| g.name == "null").unwrap().pass);
    }
}
