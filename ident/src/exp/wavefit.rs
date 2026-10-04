//! Winding R, L and V0 from the whole waveform of the from-rest bursts: one
//! model of the PWM period, stepped at [`DT_US`] through every capture and
//! read through the shunt amplifier, fitted to the shunt samples of all the
//! captures at once. Slow decay, the driven terminal measured once per ON
//! window, the rotor starting from rest:
//!
//!   ON:  L di/dt = V_on - (Rds + Rsh) i - V0 - R i - Ke w
//!   OFF: L di/dt =      - 2 Rds i       - V0 - R i - Ke w
//!        J dw/dt = Ke i
//!   shunt = the winding current while ON, 0 while OFF, through a
//!           first-order lag tau_a, sampled delta late
//!
//! V_on is the driven terminal's settled ON level; the edges come from the
//! same stream, each placed to a fraction of a conversion by the partial
//! sample it fell inside.
//!
//! A burst that sampled BOTH terminals (`--burst-chans diff`, the shunt
//! still every second conversion) is fitted on what the winding itself
//! sees instead:
//!
//!   L di/dt = (vA - vB) - V0 - R i - Ke w
//!
//! with vA - vB the terminals' difference, each tap read against its own
//! rest level and interpolated to the shunt sample times through its
//! settled samples of the same phase, averaged per ON window and per OFF
//! gap. Neither the low-side path (FET, shunt, copper) nor the divider
//! bias is then assumed: they cancel in the difference, and R is the
//! winding's. The terminals move on the current's ~140 us time constant,
//! so a tap sampled every 4.3 us interpolates across a ~10 us window. A
//! terminal that sparse catches too few edges window by window, so its
//! samples are folded onto one period and the edges fitted there
//! ([`fold`]). The same captures are fitted the driven-terminal way too,
//! for comparison ([`WaveRun::driven`]).
//!
//! Five parameters are fitted: R, L, V0, tau_a and delta. Ke and J are
//! PRIORS: the back-EMF is what that rotor makes from the current the
//! model already carries, never a free term. A free term
//! soaks up whatever else is wrong - one capture that reads low, a pooling
//! that is not robust - and then reads twenty times what the rotor can
//! make; the rotor itself moves R by about 1.4%. The loss is soft L1, so a
//! sample the model does not describe weighs less than one it does.
//!
//! Inside one duty R and V0 are not separable: only the difference between
//! the rungs separates them, and what each capture pins well is V at a
//! current. So the result is a LINE, read three ways: its slope R (the
//! winding, and with the bridge as a duty x rail over current sees it - the
//! current loop's plant), V0, and V/I at the servo's current limit - the
//! number a stall-safe duty plans from. On the bench MG90 V/I there reads
//! 5.3 ohm over a winding slope of 4.3.
//!
//! The winding depends on rotor angle: now and then one capture reads R and
//! L both low (a brush bridging two commutator segments is the suspect). A
//! least squares that pools it bends every number, so each capture is fitted
//! alone first, one reading under [`WaveCfg::low_r_frac`] of the median is
//! set aside and named, and a run where more than [`WaveCfg::set_aside_max`]
//! of them are set aside declines.

use super::rl::{Gate, Scales};
use super::winding::{EDGE_GUARD_US, RDS_ON_OHM, TAP_CLEAR_US};
use crate::burst::{CHAN_VMOTOR_A, CHAN_VMOTOR_B, Capture, HCLK_MHZ, SAMPLE_US};
use crate::fitmath::{linear_ls, median};

/// Model time step, microseconds.
pub const DT_US: f64 = 0.25;

/// The class's rotor, from the bench MG90's plant (tau_m 19 ms): back-EMF
/// constant at the motor shaft, V s/rad, and rotor inertia, kg m2. Priors,
/// not measurements of this servo: halving or doubling J moves R by -1.4 /
/// +0.8%, leaving the back-EMF out by +1.4%.
pub const KE_PRIOR: f64 = 1.334e-3;
pub const J_PRIOR: f64 = 6.9e-9;

/// The soft-L1 knee, counts: a residual past it counts linearly.
const F_SCALE: f64 = 8.0;

/// The model starts this far before the first rising edge, microseconds,
/// and the fitted samples two microseconds inside either end of it.
const LEAD_US: f64 = 5.0;
const EDGE_MARGIN_US: f64 = 2.0;

/// Edges a capture must show to be fitted.
const MIN_EDGES: usize = 6;

/// A driven terminal sampled this many conversions apart is folded onto
/// one period ([`fold`]) rather than read window by window: a 25% window
/// then holds two or three samples, the edges among them.
const FOLD_FRAME_LEN: usize = 4;

/// The fold fits the samples this far either side of the window, us: past
/// it the tap has settled and says nothing about the edge.
const FOLD_SPAN_US: f64 = 3.0;

/// The tap's RC time constant the fold starts from, us: 2.2k x 150 pF.
const FOLD_TAU0_US: f64 = 0.33;

/// Parameter order: R ohms, L mH, V0 volts, tau_a us, delta us.
const P0: [f64; 5] = [4.7, 0.8, 0.3, 1.5, 0.0];
const P_LO: [f64; 5] = [1.0, 0.2, -1.0, 0.3, -3.0];
const P_HI: [f64; 5] = [12.0, 3.0, 2.0, 6.0, 3.0];
const P_SCALE: [f64; 5] = [1.0, 0.2, 0.2, 0.5, 0.5];

/// L floor on a static load, mH: a resistor's L is its leads'. The model's
/// explicit step stays stable while (R + bridge) x DT_US / L is under 2;
/// this floor holds it under 1 up to the 12 ohm ceiling. The fit starts at
/// ten times it.
const STATIC_L_MH: f64 = 0.003;

#[derive(Clone, Debug)]
pub struct WaveCfg {
    /// Rotor priors, see [`KE_PRIOR`]. Zero `ke` leaves the rotor out.
    pub ke_v_s_per_rad: f64,
    pub j_kg_m2: f64,
    /// Per bridge FET, ohms: assumed, R moves 1.7% per 0.05 ohm of it.
    pub rds_ohm: f64,
    /// Gate 1: pooled rms of the fit, counts.
    pub residual_max_counts: f64,
    /// Gate 2: a capture whose own R reads under this fraction of the
    /// median is set aside; more than `set_aside_max` of them set aside, or
    /// a robust spread of the per-capture R over `r_spread_max`, declines.
    pub low_r_frac: f64,
    pub set_aside_max: f64,
    pub r_spread_max: f64,
    /// Gate 3: L / (R + 2 Rds), the brake path's time constant, and V0.
    pub tau_band_us: (f64, f64),
    pub v0_band: (f64, f64),
    /// Gate 4: V/I at the limit of the odd and the even captures, apart
    /// relative to the run's.
    pub halves_tol: f64,
    /// Gate 5: V/I at the limit against a start from rest in TEL.
    pub cross_tol: f64,
    /// The servo's current limit, amps: where the line is read as V/I for
    /// the stall-safe duties and `r_q12`. None reads no V/I, and a burst
    /// without it supplies no winding.
    pub i_lim_a: Option<f64>,
    /// V/I at the current limit from a start from rest in TEL of the same
    /// run, ohms, duty x rail over current. None when the run has none: the
    /// cross route is then reported not available.
    pub tel_v_over_i_ohm: Option<f64>,
    /// The load is a resistor: no rotor, L down to [`STATIC_L_MH`], and V0
    /// pinned at zero - a resistor draws the same ON current at every duty,
    /// so the line has one point and its slope and V0 do not separate.
    pub static_load: bool,
}

impl Default for WaveCfg {
    fn default() -> Self {
        Self {
            ke_v_s_per_rad: KE_PRIOR,
            j_kg_m2: J_PRIOR,
            rds_ohm: RDS_ON_OHM,
            residual_max_counts: 25.0,
            low_r_frac: 0.85,
            set_aside_max: 0.25,
            r_spread_max: 0.06,
            tau_band_us: (100.0, 300.0),
            v0_band: (-0.1, 0.5),
            halves_tol: 0.04,
            cross_tol: 0.08,
            i_lim_a: None,
            tel_v_over_i_ohm: None,
            static_load: false,
        }
    }
}

impl WaveCfg {
    /// A resistor in place of the motor.
    pub fn static_load(self) -> Self {
        Self {
            ke_v_s_per_rad: 0.0,
            j_kg_m2: 0.0,
            static_load: true,
            ..self
        }
    }
}

/// The pooled fit over a set of captures.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WaveFit {
    /// Winding R, ohms, and its formal standard error.
    pub r_ohm: f64,
    pub r_sd_ohm: f64,
    pub l_h: f64,
    pub v0_volts: f64,
    /// Shunt amplifier lag and how late the samples sit, microseconds.
    pub tau_a_us: f64,
    pub delta_us: f64,
    pub rms_counts: f64,
    pub captures: usize,
}

/// One from-rest capture fitted alone: V0 and the amplifier pinned at the
/// pooled values, R and L free.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CaptureR {
    /// Its place in the run's captures, the N of `burst-N.csv`.
    pub index: usize,
    pub duty: f64,
    pub forward: bool,
    pub pos: u16,
    pub r_ohm: f64,
    pub l_h: f64,
    pub rms_counts: f64,
    pub set_aside: bool,
}

/// The line read at the servo's current limit, duty x rail over current.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AtLimit {
    pub i_a: f64,
    /// What the stall-safe duties and `r_q12` take.
    pub v_over_i_ohm: f64,
    /// The line's slope there, bridge included: the current loop's plant.
    pub slope_ohm: f64,
}

#[derive(Clone, Debug)]
pub struct WaveRun {
    /// Over the captures kept.
    pub fit: WaveFit,
    /// Every from-rest capture that was fitted, in run order.
    pub captures: Vec<CaptureR>,
    pub r_median_ohm: f64,
    /// Robust standard deviation of the per-capture R over its median.
    pub r_spread: f64,
    /// The even and the odd kept captures, fitted apart.
    pub halves: Option<(WaveFit, WaveFit)>,
    /// How much shorter the driven terminal's ON window is than commanded,
    /// microseconds: dead time and propagation.
    pub on_loss_us: f64,
    /// Captures whose half window has an edge no terminal sample caught,
    /// placed at the midpoint of its gap.
    pub half_blind: usize,
    /// PWM period, microseconds.
    pub period_us: f64,
    /// The rail as read before the arms, volts.
    pub rail_v: f64,
    /// Fitted on the terminals' difference, both taps sampled.
    pub both_terminals: bool,
    /// The ON level against the winding current - the driven terminal's,
    /// or with both terminals their difference: open-circuit volts and the
    /// source it sags through, ohms.
    pub v_on_open: f64,
    pub z_on_ohm: f64,
    /// What the winding loses to the bridge per amp: during ON under that
    /// level (Rds + Rsh assumed, 0 when the difference measured it), and
    /// across the brake (2 Rds assumed, or the OFF difference over the
    /// current).
    pub on_drop_ohm: f64,
    pub off_ohm: f64,
    /// The idle terminal's ON level over the current: the low-side path,
    /// FET, shunt and copper. Both terminals only.
    pub lo_side_ohm: Option<f64>,
    /// The same captures fitted on the driven terminal alone, and its
    /// line at the limit. Both terminals only.
    pub driven: Option<(WaveFit, Option<AtLimit>)>,
    pub rds_ohm: f64,
    pub shunt_ohm: f64,
    /// L / (R + the brake path), microseconds.
    pub tau_us: f64,
    /// The back-EMF the rotor priors put at the end of the top rung, volts.
    pub emf_end_v: f64,
    pub at_limit: Option<AtLimit>,
    pub gates: Vec<Gate>,
    /// Gates with nothing to judge, and why.
    pub skipped: Vec<(&'static str, String)>,
    pub notes: Vec<String>,
}

impl WaveRun {
    pub fn set_aside(&self) -> impl Iterator<Item = &CaptureR> {
        self.captures.iter().filter(|c| c.set_aside)
    }

    /// Duty x rail, volts, whose settled winding current is `i_a`: the line
    /// in the convention every stall-safe duty and the firmware's cap use.
    /// Averaged over a period the winding sees D_on (V_on - on_drop i) and
    /// the brake -off i for the rest, V_on sagging with the current, and the
    /// commanded duty is D_on plus the ON window the edges lose.
    pub fn duty_volts(&self, i_a: f64) -> f64 {
        self.duty_volts_of(&self.fit, i_a)
    }

    /// The same for another fit of this run's captures, one half of them.
    fn duty_volts_of(&self, f: &WaveFit, i_a: f64) -> f64 {
        let v_on = self.v_on_open - self.z_on_ohm * i_a;
        let d_on = (f.v0_volts + (f.r_ohm + self.off_ohm) * i_a)
            / (v_on + (self.off_ohm - self.on_drop_ohm) * i_a);
        (d_on + self.on_loss_us / self.period_us) * self.rail_v
    }

    /// The line read at `i_a` amps.
    pub fn at(&self, i_a: f64) -> Option<AtLimit> {
        if !i_a.is_finite() || i_a <= 0.0 {
            return None;
        }
        let h = 0.01 * i_a;
        let a = AtLimit {
            i_a,
            v_over_i_ohm: self.duty_volts(i_a) / i_a,
            slope_ohm: (self.duty_volts(i_a + h) - self.duty_volts(i_a - h)) / (2.0 * h),
        };
        (a.v_over_i_ohm.is_finite() && a.slope_ohm.is_finite() && a.v_over_i_ohm > 0.0).then_some(a)
    }

    /// V/I at the current limit of the even and the odd captures, fitted
    /// apart: the number that is written and planned from, twice.
    pub fn halves_v_over_i(&self) -> Option<(f64, f64)> {
        let (a, b) = self.halves?;
        let i = self.at_limit?.i_a;
        Some((self.duty_volts_of(&a, i) / i, self.duty_volts_of(&b, i) / i))
    }
}

/// The driven terminal's rising and falling edge of one ON window,
/// microseconds from the capture's first conversion, its settled level in
/// raw counts. An edge that fell between two samples both clear of it is
/// known only to lie between them: `rise` or `fall` is then the midpoint
/// and its span the half-width either side, zero for an edge a sample
/// caught.
#[derive(Copy, Clone, Debug)]
struct Edge {
    rise: f64,
    fall: f64,
    hi: f64,
    rise_span: f64,
    fall_span: f64,
}

/// A capture with its streams split and its edges found.
struct Raw {
    index: usize,
    duty_q15: u16,
    forward: bool,
    pos: u16,
    rail_v: f64,
    edges: Vec<Edge>,
    /// The grid folded off the driven terminal, in place of `edges`.
    folded: Option<Folded>,
    /// Terminal tap at rest: the divider's bias node.
    pre_tap: f64,
    /// (sample instant us, shunt counts over the bias) for every shunt code.
    shunt: Vec<(f64, f64)>,
    /// (sample instant us, volts over the tap's own rest level) of the
    /// driven and the idle terminal, when both were sampled.
    taps: Option<[Vec<(f64, f64)>; 2]>,
}

/// One capture on the model's time grid.
struct Prepared {
    index: usize,
    duty: f64,
    t0_us: f64,
    /// ON coverage of each step, and coverage x V_on.
    on: Vec<f64>,
    v: Vec<f64>,
    /// (grid step at the window's centre, V_on) per window.
    windows: Vec<(usize, f64)>,
    /// `v` is the terminals' difference through every phase, so the model
    /// subtracts no bridge drop.
    measured: bool,
    /// Both terminals only: (grid step at the centre, level) of the idle
    /// terminal per ON window, and of the difference per OFF gap.
    lo: Vec<(usize, f64)>,
    gaps: Vec<(usize, f64)>,
    ts: Vec<f64>,
    y: Vec<f64>,
}

/// The sub-conversion instant an edge crossed half its swing between
/// samples `i` and `i + 1`: a sample reading fraction f of the swing closed
/// its aperture when a linear transition one conversion wide was at f.
/// When neither sample caught the transition, the midpoint and half the gap.
fn edge_at(
    vt: &[f64],
    t_of: &dyn Fn(usize) -> f64,
    i: usize,
    lo: f64,
    hi: f64,
    rising: bool,
) -> (f64, f64) {
    let span = hi - lo;
    let (mut fa, mut fb) = ((vt[i] - lo) / span, (vt[i + 1] - lo) / span);
    if !rising {
        fa = 1.0 - fa;
        fb = 1.0 - fb;
    }
    if fa > 0.02 {
        (t_of(i) + (0.5 - fa) * SAMPLE_US, 0.0)
    } else if fb < 0.98 {
        (t_of(i + 1) + (0.5 - fb) * SAMPLE_US, 0.0)
    } else {
        (0.5 * (t_of(i) + t_of(i + 1)), 0.5 * (t_of(i + 1) - t_of(i)))
    }
}

/// One edge's place relative to the PWM grid, from its instances in
/// successive windows, each `(t - k period, span)`. The grid walks a sixth
/// of a microsecond a period across the terminal's 2.2 us frames, so the
/// edges no sample caught are intervals that shift window to window: their
/// intersection pins the edge to about half a microsecond where any one
/// midpoint is off by up to a microsecond, and the midpoints do not
/// average out over the few windows one capture holds. An edge a sample
/// caught is a point and outranks them.
fn locate(at: &[(f64, f64)]) -> f64 {
    let caught: Vec<f64> = at.iter().filter(|a| a.1 == 0.0).map(|a| a.0).collect();
    if !caught.is_empty() {
        return caught.iter().sum::<f64>() / caught.len() as f64;
    }
    let lo = at.iter().map(|a| a.0 - a.1).fold(f64::MIN, f64::max);
    let hi = at.iter().map(|a| a.0 + a.1).fold(f64::MAX, f64::min);
    if lo <= hi {
        0.5 * (lo + hi)
    } else {
        at.iter().map(|a| a.0).sum::<f64>() / at.len() as f64
    }
}

/// The ON windows on the driven terminal's stream from `step` on. A sample
/// either side of each window is an edge, so the level is the median of
/// what lies between.
fn find_edges(vt: &[f64], t_of: &dyn Fn(usize) -> f64, step: usize) -> Vec<Edge> {
    let n = vt.len();
    let (Some(max), Some(off)) = (
        vt.iter().copied().reduce(f64::max),
        median(vt.get(step..).unwrap_or(&[])),
    ) else {
        return Vec::new();
    };
    let thr = 0.5 * (max + off);
    let hi: Vec<bool> = vt.iter().map(|v| *v > thr).collect();
    let from = step.saturating_sub(2);
    let rises: Vec<usize> = (from..n - 1).filter(|&i| !hi[i] && hi[i + 1]).collect();
    let falls: Vec<usize> = (from..n - 1).filter(|&i| hi[i] && !hi[i + 1]).collect();
    let mut out = Vec::new();
    for &r in &rises {
        let Some(&f) = falls.iter().find(|&&x| x > r) else {
            break;
        };
        let off_end = rises.iter().copied().find(|&x| x > f).unwrap_or(n - 1);
        let on = vt.get(r + 2..f).unwrap_or(&[]);
        let off = vt.get(f + 3..off_end).unwrap_or(&[]);
        let (Some(level), Some(lo)) = (median(on), median(off)) else {
            continue;
        };
        if off.len() < 3 {
            continue;
        }
        let prev = if r > 8 {
            median(&vt[r - 8..r]).unwrap_or(lo)
        } else {
            lo
        };
        let (rise, rise_span) = edge_at(vt, t_of, r, prev, level, true);
        let (fall, fall_span) = edge_at(vt, t_of, f, lo, level, false);
        out.push(Edge {
            rise,
            fall,
            hi: level,
            rise_span,
            fall_span,
        });
    }
    out
}

/// One capture's streams, its edges found. `both` reads the idle terminal
/// too when the capture sampled it.
fn split(index: usize, cap: &Capture, sc: &Scales, both: bool) -> Result<Raw, &'static str> {
    let forward = cap.meta.step_q15 >= 0;
    let (bit, idle) = if forward {
        (CHAN_VMOTOR_A, CHAN_VMOTOR_B)
    } else {
        (CHAN_VMOTOR_B, CHAN_VMOTOR_A)
    };
    let slot = cap.slot(bit).ok_or("no driven-terminal channel")?;
    let (fl, st) = (cap.frame_len(), cap.shunt_stride());
    let n = cap.samples.len() / fl;
    let step = cap.meta.step_index as usize / fl;
    if step < 12 || step + 12 > n {
        return Err("the step sits at an end of the capture");
    }
    let sh: Vec<f64> = cap.shunt().iter().map(|&c| c as f64).collect();
    let stream = |s: usize| -> Vec<f64> { cap.stream(s).iter().map(|&c| c as f64).collect() };
    let vt = stream(slot);
    let rest = cap.meta.step_index as usize / st - 4 * fl / st;
    let (Some(pre_tap), Some(bias)) = (median(&vt[..step - 4]), median(&sh[..rest])) else {
        return Err("no rest before the step");
    };
    let t_of = |s: usize| move |k: usize| (k * fl + s) as f64 * SAMPLE_US;
    let (edges, folded) = match fl >= FOLD_FRAME_LEN {
        false => (find_edges(&vt, &t_of(slot), step), None),
        true => {
            let period_us = 2.0 * cap.meta.pwm_arr as f64 / HCLK_MHZ;
            let t_step = cap.meta.step_index as f64 * SAMPLE_US;
            (Vec::new(), fold(&vt, &t_of(slot), t_step, period_us))
        }
    };
    if edges.len() < MIN_EDGES && folded.is_none() {
        return Err("too few ON windows on the driven terminal");
    }
    let taps = match cap.slot(idle).filter(|_| both) {
        None => None,
        Some(lo) => {
            let tap = |s: usize, v: &[f64]| -> Option<Vec<(f64, f64)>> {
                let z = median(&v[..step - 4])?;
                let t = t_of(s);
                Some(
                    v.iter()
                        .enumerate()
                        .map(|(k, c)| (t(k), sc.v_term_per_count * (c - z)))
                        .collect(),
                )
            };
            Some([
                tap(slot, &vt).ok_or("no rest before the step")?,
                tap(lo, &stream(lo)).ok_or("no rest before the step")?,
            ])
        }
    };
    Ok(Raw {
        index,
        duty_q15: cap.meta.step_q15.unsigned_abs(),
        forward,
        pos: cap.meta.pos,
        rail_v: cap.meta.vbus_raw as f64 * sc.v_rail_per_count,
        edges,
        folded,
        pre_tap,
        shunt: sh
            .iter()
            .enumerate()
            .map(|(k, s)| ((k * st) as f64 * SAMPLE_US, s - bias))
            .collect(),
        taps,
    })
}

/// A capture's PWM grid read off a driven terminal sampled too sparsely to
/// catch every window, microseconds: `phi` and the half window as
/// [`place`] gives them, the ON width, and the settled ON level in counts.
#[derive(Copy, Clone, Debug)]
struct Folded {
    phi: f64,
    width: f64,
    level: f64,
    /// The new duty latched at a crest, so the step opened a half window
    /// at `phi`; at a trough it opened none and the window at `phi + P` is
    /// whole.
    half: bool,
}

/// Every driven-terminal sample from a period after the step on, folded
/// onto one PWM period. A terminal sampled every 4.3 us walks 2.3 us a
/// period, so a capture's ten periods put a sample every half microsecond
/// or so across the fold whatever the duty, where a single window may hold
/// none. The samples above half the swing gather around the ON window's
/// centre; the edges are where the tap's RC response to a rectangle from
/// `-l` to `r` meets the samples near them, its time constant free. The
/// outermost ON and nearest OFF sample either side start it. The duty
/// latches at the first update event after the step, crest or trough.
fn fold(vt: &[f64], t_of: &dyn Fn(usize) -> f64, t_step: f64, period_us: f64) -> Option<Folded> {
    use core::f64::consts::TAU;
    let pts: Vec<(f64, f64)> = vt
        .iter()
        .enumerate()
        .map(|(k, v)| (t_of(k), *v))
        .filter(|p| p.0 >= t_step + period_us)
        .collect();
    let levels: Vec<f64> = pts.iter().map(|p| p.1).collect();
    let max = levels.iter().copied().reduce(f64::max)?;
    let thr = 0.5 * (max + median(&levels)?);
    let (on, off): (Vec<f64>, Vec<f64>) = levels.iter().partition(|v| **v > thr);
    if on.len() < MIN_EDGES {
        return None;
    }
    let (hi, lo) = (median(&on)?, median(&off)?);
    let (x, y) = pts
        .iter()
        .filter(|p| p.1 > thr)
        .fold((0.0, 0.0), |(x, y), p| {
            let a = TAU * p.0 / period_us;
            (x + a.cos(), y + a.sin())
        });
    let c = y.atan2(x) / TAU * period_us;
    let half_p = period_us / 2.0;
    let near: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| ((p.0 - c + half_p).rem_euclid(period_us) - half_p, p.1))
        .collect();
    // (outermost ON, nearest OFF) distance either side of the centre
    let (mut left, mut right) = ((0.0f64, half_p), (0.0f64, half_p));
    for &(d, v) in &near {
        let side = if d < 0.0 { &mut left } else { &mut right };
        match v > thr {
            true => side.0 = side.0.max(d.abs()),
            false => side.1 = side.1.min(d.abs()),
        }
    }
    let (l0, r0) = (0.5 * (left.0 + left.1), 0.5 * (right.0 + right.1));
    let near: Vec<(f64, f64)> = near
        .into_iter()
        .filter(|(d, _)| (-l0 - FOLD_SPAN_US..=r0 + FOLD_SPAN_US).contains(d))
        .collect();
    let mut rc = |q: &[f64], out: &mut Vec<f64>| {
        let (l, r, tau) = (q[0], q[1], q[2]);
        out.clear();
        out.extend(near.iter().map(|&(d, v)| {
            let g = if d < -l {
                0.0
            } else if d <= r {
                1.0 - (-(d + l) / tau).exp()
            } else {
                (1.0 - (-(l + r) / tau).exp()) * (-(d - r) / tau).exp()
            };
            lo + (hi - lo) * g - v
        }));
    };
    let s = robust_lm(
        &[l0, r0, FOLD_TAU0_US],
        &[0.0, 0.0, 0.02],
        &[half_p, half_p, 3.0],
        &[0.1, 0.1, 0.1],
        &mut rc,
    )?;
    let (l, r) = (s.p[0], s.p[1]);
    let centre = c + 0.5 * (r - l);
    let crest = centre + ((t_step - centre) / period_us).ceil() * period_us;
    let half = crest - half_p < t_step;
    Some(Folded {
        phi: if half { crest } else { crest - period_us },
        width: l + r,
        level: hi,
        half,
    })
}

/// The capture's PWM grid from its whole windows, microseconds: the crest
/// the step latched at, one period before the first whole window's centre,
/// and the ON width.
fn place(raw: &Raw, period_us: f64) -> (f64, f64) {
    let at = |edge: fn(&Edge) -> (f64, f64)| -> Vec<(f64, f64)> {
        raw.edges[1..]
            .iter()
            .enumerate()
            .map(|(j, x)| {
                let (t, span) = edge(x);
                (t - (j + 1) as f64 * period_us, span)
            })
            .collect()
    };
    let rise = locate(&at(|x| (x.rise, x.rise_span)));
    let fall = locate(&at(|x| (x.fall, x.fall_span)));
    (0.5 * (rise + fall), fall - rise)
}

/// The capture on the model grid. Every window after the first sits on the
/// PWM grid the capture's own edges place, `width_us` wide - one width per
/// duty, pooled across the run: the ON time is the bridge's, not the
/// capture's. The first is the half window the step latched and keeps its
/// own edges. Where no sample caught one it is the midpoint of the gap,
/// off by up to a microsecond on one window the fit cannot average over;
/// the run counts those captures ([`WaveRun::half_blind`]). Placing the
/// edge from the grid instead reads the stiff synthetic plant 0.3% closer
/// and the soft one 4% under, so the midpoint stands. Two windows past the
/// last edge cover the capture's end. With both terminals every window and
/// every gap between them takes the terminals' measured difference; None
/// when a terminal has no settled sample in one of the two phases.
fn prepare(raw: &Raw, width_us: f64, period_us: f64, sc: &Scales) -> Option<Prepared> {
    let e = &raw.edges;
    let t_last = raw.shunt.last().map_or(0.0, |s| s.0);
    let (phi, half, whole) = match raw.folded {
        None => (place(raw, period_us).0, (e[0].rise, e[0].fall), e.len() + 1),
        Some(f) => (
            f.phi,
            (f.phi, f.phi + if f.half { width_us / 2.0 } else { 0.0 }),
            ((t_last - f.phi) / period_us).ceil().max(0.0) as usize + 1,
        ),
    };
    let hi = |k: usize| match raw.folded {
        Some(f) => f.level,
        None => e.get(k).unwrap_or(&e[e.len() - 1]).hi,
    };
    let mut wins = vec![(half.0, half.1, hi(0))];
    for k in 1..=whole {
        let c = phi + k as f64 * period_us;
        wins.push((c - width_us / 2.0, c + width_us / 2.0, hi(k)));
    }
    let gaps: Vec<(f64, f64)> = wins.windows(2).map(|w| (w[0].1, w[1].0)).collect();
    // (span, level, ON) for every phase the model is driven through, the
    // idle terminal's ON level beside each window
    let mut phases = Vec::new();
    let mut lo = Vec::new();
    match &raw.taps {
        None => {
            for &(a, b, hi) in &wins {
                phases.push(((a, b), sc.terminal_volts(hi, raw.pre_tap), true));
            }
        }
        Some([hi, idle]) => {
            let ons: Vec<(f64, f64)> = wins.iter().map(|w| (w.0, w.1)).collect();
            for (spans, is_on) in [(&ons, true), (&gaps, false)] {
                let (h, l) = (settled(hi, spans), settled(idle, spans));
                for &at in spans {
                    let (h, l) = (mean_at(&h, &raw.shunt, at)?, mean_at(&l, &raw.shunt, at)?);
                    phases.push((at, h - l, is_on));
                    if is_on {
                        lo.push((at, l));
                    }
                }
            }
        }
    }
    let t0 = wins[0].0 - LEAD_US;
    let nst = ((t_last + 3.0 - t0) / DT_US).floor().max(1.0) as usize;
    let mut on = vec![0.0; nst];
    let mut v = vec![0.0; nst];
    let step_at = |t: f64| {
        let k = ((t - t0) / DT_US).round();
        (k >= 0.0 && (k as usize) < nst).then_some(k as usize)
    };
    let mut windows = Vec::new();
    let mut off_mid = Vec::new();
    for &((a, b), vv, is_on) in &phases {
        let k0 = ((a - t0) / DT_US).floor().max(0.0) as usize;
        let k1 = (((b - t0) / DT_US).ceil().max(0.0) as usize).min(nst);
        for k in k0..k1 {
            let t = t0 + DT_US * k as f64;
            let cov = ((t + DT_US).min(b) - t.max(a)) / DT_US;
            let cov = cov.clamp(0.0, 1.0);
            if is_on {
                on[k] += cov;
            }
            v[k] += cov * vv;
        }
        if let Some(k) = step_at(0.5 * (a + b)) {
            match is_on {
                true => windows.push((k, vv)),
                false => off_mid.push((k, vv)),
            }
        }
    }
    let lo = lo
        .iter()
        .filter_map(|&((a, b), l)| step_at(0.5 * (a + b)).map(|k| (k, l)))
        .collect();
    let grid_end = t0 + DT_US * (nst - 1) as f64;
    let (ts, y): (Vec<f64>, Vec<f64>) = raw
        .shunt
        .iter()
        .filter(|(t, _)| *t > t0 + EDGE_MARGIN_US && *t < grid_end - EDGE_MARGIN_US)
        .copied()
        .unzip();
    Some(Prepared {
        index: raw.index,
        duty: raw.duty_q15 as f64 / 32767.0,
        t0_us: t0,
        on,
        v,
        windows,
        measured: raw.taps.is_some(),
        lo,
        gaps: off_mid,
        ts,
        y,
    })
}

/// A tap's samples inside the spans of one phase, clear of their edges.
fn settled(tap: &[(f64, f64)], spans: &[(f64, f64)]) -> Vec<(f64, f64)> {
    tap.iter()
        .filter(|(t, _)| {
            spans
                .iter()
                .any(|(a, b)| (a + TAP_CLEAR_US..=b - EDGE_GUARD_US).contains(t))
        })
        .copied()
        .collect()
}

/// Linear through time-ordered points, held beyond either end.
fn interp(pts: &[(f64, f64)], t: f64) -> Option<f64> {
    let k = pts.partition_point(|p| p.0 <= t);
    match (k.checked_sub(1).and_then(|j| pts.get(j)), pts.get(k)) {
        (Some(a), Some(b)) => Some(a.1 + (t - a.0) / (b.0 - a.0) * (b.1 - a.1)),
        (Some(a), None) | (None, Some(a)) => Some(a.1),
        (None, None) => None,
    }
}

/// The tap's mean over the shunt sample times inside `[a, b]`, or at its
/// centre when no shunt sample falls inside.
fn mean_at(pts: &[(f64, f64)], shunt: &[(f64, f64)], (a, b): (f64, f64)) -> Option<f64> {
    let mut ts: Vec<f64> = shunt
        .iter()
        .map(|s| s.0)
        .filter(|t| (a..=b).contains(t))
        .collect();
    if ts.is_empty() {
        ts.push(0.5 * (a + b));
    }
    let v: Vec<f64> = ts.iter().filter_map(|&t| interp(pts, t)).collect();
    (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
}

/// What the model holds fixed.
#[derive(Copy, Clone)]
struct Model {
    rds: f64,
    rsh: f64,
    ke: f64,
    j: f64,
    /// Shunt counts per amp.
    g: f64,
    /// See [`WaveCfg::static_load`].
    static_load: bool,
}

impl Model {
    fn lo(&self) -> [f64; 5] {
        let mut lo = P_LO;
        if self.static_load {
            lo[1] = STATIC_L_MH;
        }
        lo
    }

    /// The parameters the fit moves.
    fn free(&self) -> &'static [usize] {
        if self.static_load {
            &[0, 1, 3, 4]
        } else {
            &[0, 1, 2, 3, 4]
        }
    }
}

/// The shunt as the amplifier shows it at every grid step into `ys`, the
/// winding current into `cur` when asked; returns the back-EMF at the end.
fn simulate(
    c: &Prepared,
    m: &Model,
    p: &[f64; 5],
    ys: &mut Vec<f64>,
    cur: Option<&mut Vec<f64>>,
) -> f64 {
    let [r, l_mh, v0, tau_a, _] = *p;
    let dt = DT_US * 1e-6;
    let over_l = dt / (l_mh * 1e-3);
    let lag = if tau_a > DT_US { DT_US / tau_a } else { 1.0 };
    let spin = if m.j > 0.0 { m.ke / m.j * dt } else { 0.0 };
    let (on_r, off_r) = match c.measured {
        true => (0.0, 0.0),
        false => (m.rds + m.rsh, 2.0 * m.rds),
    };
    let (mut i, mut w, mut y) = (0.0f64, 0.0f64, 0.0f64);
    ys.clear();
    let mut cur = cur;
    if let Some(c) = cur.as_deref_mut() {
        c.clear();
    }
    for (o, vk) in c.on.iter().zip(&c.v) {
        let drop = if i > 1e-4 || *o > 0.0 { v0 } else { 0.0 };
        let vd = vk - o * on_r * i - (1.0 - o) * off_r * i - drop - r * i - m.ke * w;
        i = (i + over_l * vd).max(0.0);
        w += spin * i;
        y += lag * (o * i - y);
        ys.push(y);
        if let Some(c) = cur.as_deref_mut() {
            c.push(i);
        }
    }
    m.ke * w
}

/// Model minus samples, counts, appended to `out`.
fn residuals(c: &Prepared, m: &Model, p: &[f64; 5], ys: &mut Vec<f64>, out: &mut Vec<f64>) {
    simulate(c, m, p, ys, None);
    for (t, y) in c.ts.iter().zip(&c.y) {
        out.push(m.g * shunt_at(c, ys, t + p[4]) - y);
    }
}

/// The simulated shunt at `t` microseconds: `ys[k]` is the level at the end
/// of step k, held beyond either end.
fn shunt_at(c: &Prepared, ys: &[f64], t: f64) -> f64 {
    let n = ys.len();
    let x = (t - c.t0_us) / DT_US - 1.0;
    if x <= 0.0 {
        ys[0]
    } else if x >= (n - 1) as f64 {
        ys[n - 1]
    } else {
        let k = x.floor() as usize;
        ys[k] + (x - k as f64) * (ys[k + 1] - ys[k])
    }
}

struct Solved {
    p: Vec<f64>,
    sd: Vec<f64>,
    rms: f64,
}

/// Soft-L1 cost of a residual vector.
fn robust_cost(r: &[f64]) -> f64 {
    r.iter()
        .map(|x| F_SCALE * F_SCALE * ((1.0 + (x / F_SCALE).powi(2)).sqrt() - 1.0))
        .sum()
}

/// Solve `a x = b` in place, a small dense system. None when singular.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let m = b.len();
    for col in 0..m {
        let piv = (col..m).max_by(|&p, &q| a[p][col].abs().total_cmp(&a[q][col].abs()))?;
        if !a[piv][col].is_finite() || a[piv][col].abs() <= 1e-300 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for r in col + 1..m {
            let f = a[r][col] / a[col][col];
            for c in col..m {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = vec![0.0; m];
    for r in (0..m).rev() {
        let s: f64 = (r + 1..m).map(|c| a[r][c] * x[c]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// Levenberg-Marquardt on the soft-L1 cost, the robust weights refreshed
/// every iteration, the Jacobian by forward differences, the parameters
/// held inside [lo, hi].
fn robust_lm(
    p0: &[f64],
    lo: &[f64],
    hi: &[f64],
    scale: &[f64],
    f: &mut dyn FnMut(&[f64], &mut Vec<f64>),
) -> Option<Solved> {
    const MAX_ITER: usize = 100;
    let m = p0.len();
    let mut p: Vec<f64> = p0
        .iter()
        .zip(lo.iter().zip(hi))
        .map(|(x, (l, h))| x.clamp(*l, *h))
        .collect();
    let mut r = Vec::new();
    f(&p, &mut r);
    if r.len() <= m || !r.iter().all(|x| x.is_finite()) {
        return None;
    }
    let mut cost = robust_cost(&r);
    let mut lambda = 1e-3;
    let mut rj = Vec::new();
    let mut a = vec![vec![0.0; m]; m];
    for _ in 0..MAX_ITER {
        let mut cols = Vec::with_capacity(m);
        for j in 0..m {
            let mut h = 1e-6 * scale[j].max(p[j].abs());
            if p[j] + h > hi[j] {
                h = -h;
            }
            let mut q = p.clone();
            q[j] += h;
            f(&q, &mut rj);
            if rj.len() != r.len() {
                return None;
            }
            cols.push(
                rj.iter()
                    .zip(&r)
                    .map(|(b, a)| (b - a) / h)
                    .collect::<Vec<f64>>(),
            );
        }
        let w: Vec<f64> = r
            .iter()
            .map(|x| 1.0 / (1.0 + (x / F_SCALE).powi(2)).sqrt())
            .collect();
        let mut g = vec![0.0; m];
        for j in 0..m {
            g[j] = cols[j]
                .iter()
                .zip(&w)
                .zip(&r)
                .map(|((c, w), r)| c * w * r)
                .sum();
            for k in 0..=j {
                let s: f64 = cols[j]
                    .iter()
                    .zip(&cols[k])
                    .zip(&w)
                    .map(|((a, b), w)| a * b * w)
                    .sum();
                a[j][k] = s;
                a[k][j] = s;
            }
        }
        let mut improved = None;
        while lambda < 1e10 {
            let damped: Vec<Vec<f64>> = (0..m)
                .map(|j| {
                    (0..m)
                        .map(|k| {
                            a[j][k]
                                + if j == k {
                                    lambda * a[j][j].max(1e-12)
                                } else {
                                    0.0
                                }
                        })
                        .collect()
                })
                .collect();
            let Some(dp) = solve(damped, g.iter().map(|x| -x).collect()) else {
                lambda *= 10.0;
                continue;
            };
            let q: Vec<f64> = (0..m).map(|j| (p[j] + dp[j]).clamp(lo[j], hi[j])).collect();
            f(&q, &mut rj);
            let c = robust_cost(&rj);
            if rj.len() == r.len() && c.is_finite() && c < cost {
                lambda = (lambda / 10.0).max(1e-12);
                let small = (0..m).all(|j| (q[j] - p[j]).abs() <= 1e-8 * scale[j]);
                improved = Some((q, c, small));
                break;
            }
            lambda *= 10.0;
        }
        let Some((q, c, small)) = improved else {
            break;
        };
        let gain = cost - c;
        p = q;
        cost = c;
        core::mem::swap(&mut r, &mut rj);
        if small || gain <= 1e-10 * cost {
            break;
        }
    }
    let dof = (r.len() - m) as f64;
    let rss: f64 = r.iter().map(|x| x * x).sum();
    let sd = (0..m)
        .map(|j| {
            let mut e = vec![0.0; m];
            e[j] = 1.0;
            solve(a.clone(), e).map_or(f64::NAN, |x| (x[j] * rss / dof).max(0.0).sqrt())
        })
        .collect();
    Some(Solved {
        p,
        sd,
        rms: (rss / r.len() as f64).sqrt(),
    })
}

fn pooled(caps: &[&Prepared], m: &Model, start: &[f64; 5]) -> Option<WaveFit> {
    let free = m.free();
    let pick = |a: &[f64; 5]| -> Vec<f64> { free.iter().map(|&j| a[j]).collect() };
    let put = |q: &[f64]| {
        let mut p = *start;
        for (&j, x) in free.iter().zip(q) {
            p[j] = *x;
        }
        p
    };
    let mut ys = Vec::new();
    let mut f = |q: &[f64], out: &mut Vec<f64>| {
        out.clear();
        let p = put(q);
        for c in caps {
            residuals(c, m, &p, &mut ys, out);
        }
    };
    let s = robust_lm(
        &pick(start),
        &pick(&m.lo()),
        &pick(&P_HI),
        &pick(&P_SCALE),
        &mut f,
    )?;
    let p = put(&s.p);
    Some(WaveFit {
        r_ohm: p[0],
        r_sd_ohm: s.sd[0],
        l_h: p[1] * 1e-3,
        v0_volts: p[2],
        tau_a_us: p[3],
        delta_us: p[4],
        rms_counts: s.rms,
        captures: caps.len(),
    })
}

fn params(f: &WaveFit) -> [f64; 5] {
    [f.r_ohm, f.l_h * 1e3, f.v0_volts, f.tau_a_us, f.delta_us]
}

/// One capture's own R and L, the rest of the model pinned at `with`.
fn alone(c: &Prepared, m: &Model, with: &WaveFit) -> Option<(f64, f64, f64)> {
    let base = params(with);
    let mut ys = Vec::new();
    let mut f = |q: &[f64], out: &mut Vec<f64>| {
        out.clear();
        residuals(c, m, &[q[0], q[1], base[2], base[3], base[4]], &mut ys, out);
    };
    let s = robust_lm(&base[..2], &m.lo()[..2], &P_HI[..2], &P_SCALE[..2], &mut f)?;
    Some((s.p[0], s.p[1] * 1e-3, s.rms))
}

fn gate(name: &'static str, pass: bool, detail: String) -> Gate {
    Gate { name, pass, detail }
}

/// Median absolute deviation scaled to a normal sd, over the median.
fn robust_spread(v: &[f64]) -> Option<f64> {
    let m = median(v)?;
    let dev: Vec<f64> = v.iter().map(|x| (x - m).abs()).collect();
    Some(1.4826 * median(&dev)? / m)
}

fn describe(c: &CaptureR) -> String {
    format!(
        "capture {} ({:.0}% {}, pos {})",
        c.index,
        c.duty * 100.0,
        if c.forward { "forward" } else { "reverse" },
        c.pos
    )
}

/// Every from-rest capture split, its edges found and put on the model
/// grid, with the PWM period and the ON time the edges lose, microseconds.
struct Grid {
    raws: Vec<Raw>,
    prepared: Vec<Prepared>,
    period_us: f64,
    on_loss_us: f64,
    /// Captures whose half window had an edge no sample caught.
    half_blind: usize,
}

fn grid(
    rest: &[(usize, &Capture)],
    sc: &Scales,
    both: bool,
    notes: &mut Vec<String>,
) -> Result<Grid, String> {
    let mut raws = Vec::new();
    for (k, cap) in rest {
        match split(*k, cap, sc, both) {
            Ok(r) => raws.push(r),
            Err(why) => notes.push(format!("capture {k}: {why}; not fitted")),
        }
    }
    if rest.is_empty() {
        return Err("no capture from rest".into());
    }
    if raws.len() < 2 {
        return Err(
            if rest.iter().all(|(_, c)| {
                let bit = if c.meta.step_q15 >= 0 {
                    CHAN_VMOTOR_A
                } else {
                    CHAN_VMOTOR_B
                };
                c.slot(bit).is_none()
            }) {
                "no capture from rest sampled the driven terminal".into()
            } else {
                format!(
                    "{} of {} captures from rest could be fitted",
                    raws.len(),
                    rest.len()
                )
            },
        );
    }
    let arr = rest.first().map_or(0, |(_, c)| c.meta.pwm_arr);
    let period_us = 2.0 * arr as f64 / HCLK_MHZ;
    // The ON width per duty, pooled over the captures' whole windows.
    let mut duties: Vec<u16> = raws.iter().map(|r| r.duty_q15).collect();
    duties.sort_unstable();
    duties.dedup();
    let widths: Vec<(u16, f64)> = duties
        .iter()
        .map(|d| {
            let w: Vec<f64> = raws
                .iter()
                .filter(|r| r.duty_q15 == *d)
                .map(|r| r.folded.map_or_else(|| place(r, period_us).1, |f| f.width))
                .collect();
            (*d, w.iter().sum::<f64>() / w.len() as f64)
        })
        .collect();
    let width = |d: u16| widths.iter().find(|w| w.0 == d).map_or(0.0, |w| w.1);
    let on_loss_us = median(
        &widths
            .iter()
            .map(|(d, w)| *d as f64 / 32767.0 * period_us - w)
            .collect::<Vec<_>>(),
    )
    .unwrap_or(0.0);
    let half_blind = raws
        .iter()
        .filter(|r| {
            r.edges
                .first()
                .is_some_and(|e| e.rise_span > 0.0 || e.fall_span > 0.0)
        })
        .count();
    let mut prepared = Vec::new();
    raws.retain(|r| match prepare(r, width(r.duty_q15), period_us, sc) {
        Some(p) => {
            prepared.push(p);
            true
        }
        None => {
            notes.push(format!(
                "capture {}: a terminal has no settled sample in a phase; not fitted",
                r.index
            ));
            false
        }
    });
    if prepared.len() < 2 {
        return Err(format!(
            "{} of {} captures from rest could be fitted",
            prepared.len(),
            rest.len()
        ));
    }
    Ok(Grid {
        raws,
        prepared,
        period_us,
        on_loss_us,
        half_blind,
    })
}

/// The whole-waveform fit over the from-rest captures, `(index, capture)`
/// in run order, with its gates. Err says why nothing could be fitted.
/// Captures that sampled both terminals are fitted on their difference,
/// and on the driven terminal alone beside it.
pub fn fit_run(rest: &[(usize, &Capture)], sc: &Scales, cfg: &WaveCfg) -> Result<WaveRun, String> {
    let mut run = fit_with(rest, sc, cfg, true)?;
    if run.both_terminals {
        run.driven = fit_with(rest, sc, cfg, false)
            .ok()
            .map(|d| (d.fit, d.at_limit));
    }
    Ok(run)
}

fn fit_with(
    rest: &[(usize, &Capture)],
    sc: &Scales,
    cfg: &WaveCfg,
    both: bool,
) -> Result<WaveRun, String> {
    let mut notes = Vec::new();
    let Grid {
        raws,
        prepared,
        period_us,
        on_loss_us,
        half_blind,
    } = grid(rest, sc, both, &mut notes)?;
    let model = Model {
        rds: cfg.rds_ohm,
        rsh: sc.shunt_ohm,
        ke: cfg.ke_v_s_per_rad,
        j: cfg.j_kg_m2,
        g: 1.0 / sc.amps_per_count,
        static_load: cfg.static_load,
    };
    let start = if cfg.static_load {
        [P0[0], 10.0 * STATIC_L_MH, 0.0, P0[3], P0[4]]
    } else {
        P0
    };
    let all: Vec<&Prepared> = prepared.iter().collect();
    let first = pooled(&all, &model, &start).ok_or("the pooled fit is degenerate")?;

    let mut captures = Vec::new();
    for (c, raw) in prepared.iter().zip(&raws) {
        if let Some((r, l, rms)) = alone(c, &model, &first) {
            captures.push(CaptureR {
                index: c.index,
                duty: c.duty,
                forward: raw.forward,
                pos: raw.pos,
                r_ohm: r,
                l_h: l,
                rms_counts: rms,
                set_aside: false,
            });
        }
    }
    let rs: Vec<f64> = captures.iter().map(|c| c.r_ohm).collect();
    let r_median = median(&rs).ok_or("no capture fitted alone")?;
    for c in &mut captures {
        c.set_aside = c.r_ohm < cfg.low_r_frac * r_median;
    }
    let r_spread = robust_spread(&rs).unwrap_or(f64::INFINITY);
    let kept: Vec<&Prepared> = prepared
        .iter()
        .filter(|p| captures.iter().any(|c| c.index == p.index && !c.set_aside))
        .collect();
    if kept.len() < 2 {
        return Err("fewer than two captures agree".into());
    }
    let fit = pooled(&kept, &model, &params(&first)).ok_or("the pooled fit is degenerate")?;
    let evens: Vec<&Prepared> = kept.iter().copied().step_by(2).collect();
    let odds: Vec<&Prepared> = kept.iter().copied().skip(1).step_by(2).collect();
    let halves = (odds.len() >= 2)
        .then(|| {
            Some((
                pooled(&evens, &model, &params(&fit))?,
                pooled(&odds, &model, &params(&fit))?,
            ))
        })
        .flatten();

    // The ON level against the winding current, and the back-EMF the priors
    // put at the end of each capture.
    let p = params(&fit);
    let (mut ys, mut cur) = (Vec::new(), Vec::new());
    let (mut pts, mut lo, mut off) = (Vec::new(), Vec::new(), Vec::new());
    let mut emf_end_v = 0.0f64;
    for c in &kept {
        emf_end_v = emf_end_v.max(simulate(c, &model, &p, &mut ys, Some(&mut cur)));
        let at = |w: &[(usize, f64)]| w.iter().map(|(k, v)| (cur[*k], *v)).collect::<Vec<_>>();
        pts.extend(at(&c.windows));
        lo.extend(at(&c.lo));
        off.extend(at(&c.gaps));
    }
    let both_terminals = kept.iter().all(|c| c.measured);
    // The brake's difference is the OFF path's drop alone, through zero.
    let off_ohm = match both_terminals {
        true => {
            let ii: f64 = off.iter().map(|(i, _)| i * i).sum();
            -off.iter().map(|(i, v)| i * v).sum::<f64>() / ii
        }
        false => 2.0 * cfg.rds_ohm,
    };
    let on_drop_ohm = match both_terminals {
        true => 0.0,
        false => cfg.rds_ohm + sc.shunt_ohm,
    };
    let lo_side_ohm = linear_ls(&lo).map(|l| l.b);
    let (v_on_open, z_on_ohm) = match linear_ls(&pts) {
        Some(l) => (l.a, -l.b),
        None => (
            median(&pts.iter().map(|p| p.1).collect::<Vec<_>>()).unwrap_or(0.0),
            0.0,
        ),
    };
    let rail_v = median(
        &raws
            .iter()
            .filter(|r| kept.iter().any(|k| k.index == r.index))
            .map(|r| r.rail_v)
            .collect::<Vec<_>>(),
    )
    .unwrap_or(0.0);
    let tau_us = fit.l_h / (fit.r_ohm + off_ohm) * 1e6;
    let mut run = WaveRun {
        fit,
        captures,
        r_median_ohm: r_median,
        r_spread,
        halves,
        on_loss_us,
        half_blind,
        period_us,
        rail_v,
        both_terminals,
        v_on_open,
        z_on_ohm,
        on_drop_ohm,
        off_ohm,
        lo_side_ohm,
        driven: None,
        rds_ohm: cfg.rds_ohm,
        shunt_ohm: sc.shunt_ohm,
        tau_us,
        emf_end_v,
        at_limit: None,
        gates: Vec::new(),
        skipped: Vec::new(),
        notes,
    };
    run.at_limit = cfg.i_lim_a.and_then(|i| run.at(i));
    run.gates = gates(&run, cfg);
    match (cfg.tel_v_over_i_ohm, run.at_limit) {
        (Some(tel), Some(at)) => run
            .gates
            .push(cross_route(at.v_over_i_ohm, tel, cfg.cross_tol)),
        (Some(_), None) => run.skipped.push((
            "cross-route",
            "not available: the servo's current limit was not given".into(),
        )),
        (None, _) => run.skipped.push((
            "cross-route",
            "not available: the run has no start from rest in TEL".into(),
        )),
    }
    Ok(run)
}

/// Gate 5: the only one that compares two estimators of one quantity.
pub fn cross_route(burst_ohm: f64, tel_ohm: f64, tol: f64) -> Gate {
    let gap = (burst_ohm - tel_ohm).abs() / tel_ohm;
    gate(
        "cross-route",
        gap <= tol,
        format!(
            "V/I at the limit {burst_ohm:.2} ohm from the burst, {tel_ohm:.2} from the start from \
             rest, {:.1}% apart (within {:.0}%)",
            gap * 100.0,
            tol * 100.0
        ),
    )
}

fn gates(run: &WaveRun, cfg: &WaveCfg) -> Vec<Gate> {
    let f = &run.fit;
    let n = run.captures.len();
    let aside: Vec<&CaptureR> = run.set_aside().collect();
    let too_many = aside.len() as f64 > cfg.set_aside_max * n as f64;
    let named = match aside.as_slice() {
        [] => String::new(),
        a => format!(
            "; set aside: {}",
            a.iter()
                .map(|c| format!(
                    "{} at {:.2} of the median",
                    describe(c),
                    c.r_ohm / run.r_median_ohm
                ))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    };
    let (t_lo, t_hi) = cfg.tau_band_us;
    let (v_lo, v_hi) = cfg.v0_band;
    vec![
        gate(
            "residual",
            f.rms_counts <= cfg.residual_max_counts,
            format!(
                "rms {:.1} counts over {} captures (under {:.0})",
                f.rms_counts, f.captures, cfg.residual_max_counts
            ),
        ),
        gate(
            "capture-agreement",
            run.r_spread <= cfg.r_spread_max && !too_many,
            format!(
                "per-capture R median {:.3} ohm, robust sd {:.1}% (under {:.0}%); {} of {n} set \
                 aside (at most a quarter){named}",
                run.r_median_ohm,
                run.r_spread * 100.0,
                cfg.r_spread_max * 100.0,
                aside.len()
            ),
        ),
        gate(
            "physical-bounds",
            (t_lo..=t_hi).contains(&run.tau_us) && (v_lo..=v_hi).contains(&f.v0_volts),
            format!(
                "tau {:.0} us ({t_lo:.0}..{t_hi:.0}), V0 {:.3} V ({v_lo:.2}..{v_hi:.2}); back-EMF \
                 from the rotor priors only, {:.0} mV at the end of the top rung",
                run.tau_us,
                f.v0_volts,
                run.emf_end_v * 1e3
            ),
        ),
        split_halves(run, cfg),
    ]
}

/// Gate 4, on the number that is written and planned from: V/I at the
/// current limit of the even and the odd captures. Their slopes scatter
/// more, R trading against V0 inside a few captures, and are reported only.
fn split_halves(run: &WaveRun, cfg: &WaveCfg) -> Gate {
    let (Some((a, b)), Some(at)) = (run.halves, run.at_limit) else {
        let why = match run.halves {
            None => "too few captures to fit two halves",
            Some(_) => "the servo's current limit was not given, so there is no V/I to compare",
        };
        return gate("split-halves", false, why.into());
    };
    let Some((va, vb)) = run.halves_v_over_i() else {
        return gate("split-halves", false, "a half reads no V/I".into());
    };
    let gap = (va - vb).abs() / at.v_over_i_ohm;
    gate(
        "split-halves",
        gap <= cfg.halves_tol,
        format!(
            "V/I at the limit: even captures {va:.3}, odd {vb:.3} ohm, {:.1}% apart (within \
             {:.0}%); slope R {:.3} and {:.3} ohm",
            gap * 100.0,
            cfg.halves_tol * 100.0,
            a.r_ohm,
            b.r_ohm
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exp::inductance::{FitCfg, fit_captures};
    use crate::exp::testkit::{board_d_scales, mg90_2s};

    /// The bench servo's 280-count limit, amps.
    fn i_lim() -> f64 {
        280.0 * board_d_scales().amps_per_count
    }

    fn at_limit() -> WaveCfg {
        WaveCfg {
            i_lim_a: Some(i_lim()),
            ..WaveCfg::default()
        }
    }

    fn rest(caps: &[Capture]) -> Vec<(usize, &Capture)> {
        caps.iter().enumerate().collect()
    }

    fn fit(caps: &[Capture], cfg: &WaveCfg) -> WaveRun {
        fit_run(&rest(caps), &board_d_scales(), cfg).expect("a waveform fit")
    }

    fn gate<'a>(run: &'a WaveRun, name: &str) -> &'a Gate {
        run.gates.iter().find(|g| g.name == name).expect(name)
    }

    /// R, L (mH), V0, tau_a, delta: the bench winding, round numbers.
    const TRUTH: [f64; 5] = [4.28, 0.79, 0.12, 1.29, 0.43];

    /// A brush bridging two commutator segments: R at three quarters, L
    /// down too.
    const BRIDGED: [f64; 5] = [4.28 * 0.75, 0.79 * 0.88, 0.12, 1.29, 0.43];

    /// The bench captures with their shunt stream replaced by the model's
    /// own waveform at `truth(k)` for capture k, on the captures' real
    /// edges, terminal levels and sample instants, with `sd` counts of noise
    /// and the ADC's rounding: a check of the fit, not of the edge finder.
    fn synthetic(truth: impl Fn(usize) -> [f64; 5], sd: f64) -> Vec<Capture> {
        let sc = board_d_scales();
        let mut caps = mg90_2s();
        let g = grid(&rest(&caps), &sc, true, &mut Vec::new()).expect("the bench grid");
        let m = Model {
            rds: RDS_ON_OHM,
            rsh: sc.shunt_ohm,
            ke: KE_PRIOR,
            j: J_PRIOR,
            g: 1.0 / sc.amps_per_count,
            static_load: false,
        };
        let mut lcg = 0x2545_F491_4F6C_DD1Du64;
        let mut uniform = || {
            lcg = lcg
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5
        };
        let mut ys = Vec::new();
        for p in &g.prepared {
            let t = truth(p.index);
            simulate(p, &m, &t, &mut ys, None);
            let cap = &mut caps[p.index];
            let (fl, bias) = (cap.shunt_stride(), cap.meta.bias as f64);
            for k in 0..cap.samples.len() / fl {
                let at = shunt_at(p, &ys, (k * fl) as f64 * SAMPLE_US + t[4]);
                // four uniforms scaled by sqrt(3) sum to unit variance
                let n = (0..4).map(|_| uniform()).sum::<f64>() * 3f64.sqrt();
                cap.samples[k * fl] = (bias + m.g * at + sd * n).round().clamp(0.0, 4095.0) as u16;
            }
        }
        caps
    }

    /// The bench MG90 on 2S: R 4.28 ohm within 3%, L 0.78 to 0.82 mH, V0
    /// 0.05 to 0.20 V, every gate passing. An independent numpy/scipy
    /// implementation of the same model reads 4.356 ohm, 0.803 mH and 0.096 V
    /// on the same fifteen captures at this Rds. It zeroes the terminal at
    /// two thirds of the rest level where the dividers' own ratio gives
    /// 0.673, and places an edge no sample caught at the midpoint of its
    /// gap; the edge placement here moves V0 by 9 mV. V/I at the limit reads 5.3
    /// ohm, within 3% of a locked rotor and of a start from rest at the limit
    /// on the same servo.
    #[test]
    fn waveform_fit_recovers_the_bench_winding() {
        let run = fit(&mg90_2s(), &at_limit());
        let f = run.fit;
        assert!((f.r_ohm / 4.28 - 1.0).abs() < 0.03, "{f:?}");
        assert!((0.78e-3..=0.82e-3).contains(&f.l_h), "{f:?}");
        assert!((0.05..=0.20).contains(&f.v0_volts), "{f:?}");
        assert!((f.r_ohm / 4.3556 - 1.0).abs() < 0.005, "{f:?}");
        assert!((f.l_h / 0.8030e-3 - 1.0).abs() < 0.005, "{f:?}");
        assert!((f.v0_volts - 0.0961).abs() < 0.015, "{f:?}");
        assert!(run.gates.iter().all(|g| g.pass), "{:?}", run.gates);
        let at = run.at_limit.expect("read at the limit");
        assert!((at.v_over_i_ohm / 5.3 - 1.0).abs() < 0.03, "{at:?}");
        assert!(
            f.r_ohm < at.slope_ohm && at.slope_ohm < at.v_over_i_ohm,
            "{at:?}"
        );
    }

    /// A known winding on the bench captures' own edges, 1.5 counts of
    /// noise, one capture bridged: the fit recovers R, L and V0 within 2%
    /// and sets the bridged capture aside.
    #[test]
    fn waveform_fit_recovers_a_synthetic_winding() {
        let caps = synthetic(|k| if k == 6 { BRIDGED } else { TRUTH }, 1.5);
        let run = fit(&caps, &at_limit());
        let f = run.fit;
        for (got, want) in [
            (f.r_ohm, TRUTH[0]),
            (f.l_h * 1e3, TRUTH[1]),
            (f.v0_volts, TRUTH[2]),
        ] {
            assert!((got / want - 1.0).abs() < 0.02, "{got} of {want}: {f:?}");
        }
        let aside: Vec<usize> = run.set_aside().map(|c| c.index).collect();
        assert_eq!(aside, [6]);
        assert_eq!(f.captures, 15);
        assert!(run.gates.iter().all(|g| g.pass), "{:?}", run.gates);
    }

    /// Capture 11 of the bench run reads 0.74 of the median: it is named and
    /// set aside, and the line comes from the other fifteen. Pooled, it
    /// drags R down by over a percent and doubles the residual.
    #[test]
    fn a_low_reading_capture_is_set_aside_not_pooled() {
        let caps = mg90_2s();
        let run = fit(&caps, &at_limit());
        let aside: Vec<&CaptureR> = run.set_aside().collect();
        assert_eq!(aside.len(), 1);
        let c = aside[0];
        assert_eq!((c.index, c.forward, c.pos), (11, true, 1782));
        assert!(
            (0.70..0.80).contains(&(c.r_ohm / run.r_median_ohm)),
            "{c:?}"
        );
        assert!(c.l_h < run.fit.l_h, "L reads low with R: {c:?}");
        assert_eq!(run.fit.captures, 15);
        let agree = gate(&run, "capture-agreement");
        assert!(agree.pass);
        assert!(
            agree
                .detail
                .contains("set aside: capture 11 (40% forward, pos 1782) at 0.74 of the median"),
            "{}",
            agree.detail
        );
        let pooled = fit(
            &caps,
            &WaveCfg {
                low_r_frac: 0.0,
                ..at_limit()
            },
        );
        assert_eq!(pooled.fit.captures, 16);
        assert!(pooled.fit.r_ohm < run.fit.r_ohm * 0.99, "{:?}", pooled.fit);
        assert!(pooled.fit.rms_counts > 1.8 * run.fit.rms_counts);
    }

    /// Five of sixteen captures bridged is more than a quarter: the burst
    /// declines on capture agreement, and says so.
    #[test]
    fn too_many_outliers_decline_the_burst() {
        let low = [1, 5, 9, 13, 14];
        let caps = synthetic(|k| if low.contains(&k) { BRIDGED } else { TRUTH }, 1.5);
        let run = fit(&caps, &at_limit());
        let aside: Vec<usize> = run.set_aside().map(|c| c.index).collect();
        assert_eq!(aside, low);
        let agree = gate(&run, "capture-agreement");
        assert!(!agree.pass);
        assert!(
            agree.detail.contains("5 of 16 set aside"),
            "{}",
            agree.detail
        );
        let cfg = FitCfg::default().with_limit(i_lim());
        let r = fit_captures(&caps, &board_d_scales(), &cfg).expect("the run fits");
        assert_eq!(r.blocking(), vec!["capture-agreement"]);
        assert!(!r.promotable());
        assert_eq!(r.reason(), "5 of its bursts read the winding low");
    }

    /// The fit has five parameters, none of them a back-EMF: the rotor's
    /// is what the Ke and J priors make of the fitted current. On the bench
    /// run that is 51 mV at the end of the 40% rung; twice the inertia makes
    /// half of it, a rotor left out none, and R then moves by about 1.4%.
    #[test]
    fn back_emf_is_a_prior_never_a_free_term() {
        assert_eq!([P0.len(), P_LO.len(), P_HI.len(), P_SCALE.len()], [5; 4]);
        let caps = mg90_2s();
        let with = fit(&caps, &WaveCfg::default());
        assert!(
            (0.045..0.057).contains(&with.emf_end_v),
            "{}",
            with.emf_end_v
        );
        assert!(
            gate(&with, "physical-bounds")
                .detail
                .contains("back-EMF from the rotor priors only, 51 mV")
        );
        let heavy = fit(
            &caps,
            &WaveCfg {
                j_kg_m2: 2.0 * J_PRIOR,
                ..WaveCfg::default()
            },
        );
        assert!(
            (heavy.emf_end_v / with.emf_end_v - 0.5).abs() < 0.03,
            "{}",
            heavy.emf_end_v
        );
        let still = fit(
            &caps,
            &WaveCfg {
                ke_v_s_per_rad: 0.0,
                ..WaveCfg::default()
            },
        );
        assert_eq!(still.emf_end_v, 0.0);
        let moved = still.fit.r_ohm / with.fit.r_ohm - 1.0;
        assert!((0.008..0.02).contains(&moved), "R moved {moved}");
    }

    /// The even and the odd captures of the bench run read V/I at the
    /// limit 5.36 and 5.27 ohm, 1.8% apart, and the gate says so with both
    /// slopes beside it. A run whose odd captures read a tenth higher in R
    /// at the same V0 reads about 8% higher at the limit and fails.
    #[test]
    fn split_halves_agree() {
        let run = fit(&mg90_2s(), &at_limit());
        let (a, b) = run.halves.expect("two halves");
        assert_eq!((a.captures, b.captures), (8, 7));
        let (va, vb) = run.halves_v_over_i().expect("V/I of both halves");
        let v = run.at_limit.unwrap().v_over_i_ohm;
        assert!((va - vb).abs() / v < 0.02, "{va} {vb}");
        let halves = gate(&run, "split-halves");
        assert!(halves.pass, "{}", halves.detail);
        assert!(
            halves.detail.starts_with(&format!(
                "V/I at the limit: even captures {va:.3}, odd {vb:.3} ohm"
            )),
            "{}",
            halves.detail
        );
        assert!(
            halves
                .detail
                .ends_with(&format!("slope R {:.3} and {:.3} ohm", a.r_ohm, b.r_ohm)),
            "{}",
            halves.detail
        );
        let odd = [TRUTH[0] * 1.1, TRUTH[1], TRUTH[2], TRUTH[3], TRUTH[4]];
        let caps = synthetic(|k| if k % 2 == 1 { odd } else { TRUTH }, 1.5);
        let run = fit(&caps, &at_limit());
        let halves = gate(&run, "split-halves");
        assert!(!halves.pass, "{}", halves.detail);
    }

    /// The halves of a run can trade R against V0 and still read one line
    /// at the limit, as the bench's third run did: 7.7% apart in slope, 2.2%
    /// in V/I at the limit. The odd captures here carry R 7.7% higher and V0
    /// low enough to land 2.2% higher at the limit: the run promotes, the
    /// slopes reported beside it.
    #[test]
    fn halves_that_split_in_slope_but_agree_at_the_limit_promote() {
        let i = i_lim();
        let r_odd = TRUTH[0] * 1.077;
        // V/I = R + 2 Rds + V0 / i near enough for the target: move V0 so
        // the odd half lands 2.2% of the whole over at the limit
        let target = 0.022 * (TRUTH[0] + 2.0 * RDS_ON_OHM + TRUTH[2] / i);
        let v0_odd = TRUTH[2] + (target - (r_odd - TRUTH[0])) * i;
        let odd = [r_odd, TRUTH[1], v0_odd, TRUTH[3], TRUTH[4]];
        let caps = synthetic(|k| if k % 2 == 1 { odd } else { TRUTH }, 1.5);
        let run = fit(&caps, &at_limit());
        let (a, b) = run.halves.expect("two halves");
        let slope_gap = (b.r_ohm - a.r_ohm) / run.fit.r_ohm;
        assert!((0.06..0.10).contains(&slope_gap), "slopes {a:?} {b:?}");
        let (va, vb) = run.halves_v_over_i().unwrap();
        let v = run.at_limit.unwrap().v_over_i_ohm;
        assert!((0.01..0.035).contains(&((vb - va) / v)), "{va} {vb}");
        assert!(gate(&run, "split-halves").pass);
        let cfg = FitCfg::default().with_limit(i);
        let r = fit_captures(&caps, &board_d_scales(), &cfg).expect("the run fits");
        assert!(r.promotable(), "{:?}", r.blocking());
    }

    /// The whole windows' edges a terminal sample did not catch are placed
    /// from the intervals they shift through; the half window keeps its
    /// midpoints, counted. What error is left is a bound here, on the
    /// synthetic plant whose taps settle as the board's do (0.33 us) and
    /// whose every capture has a half-window edge no sample caught: V/I at
    /// the limit reads 2.9% over the plant's own, where midpoints on every
    /// window read 4.2%.
    #[test]
    fn edges_no_sample_caught_stay_inside_their_bound() {
        use crate::burst::Chans;
        use crate::exp::testkit::SynthBurst;
        let plant = SynthBurst {
            r: 4.28,
            l: 0.79e-3,
            v0: 0.12,
            settle_us: 1.29,
            v_rail: 7.2,
            ..SynthBurst::board_d().with_bridge()
        };
        let mut caps = Vec::new();
        for pct in [25i32, 40] {
            for sgn in [1i32, -1] {
                for _ in 0..4 {
                    let q = (sgn * pct * 32767 / 100) as i16;
                    let p = SynthBurst {
                        chans: Chans::Driven.for_step(q),
                        ..plant.clone()
                    };
                    caps.push(p.capture(q, 0));
                }
            }
        }
        // the plant has no rotor, and no ON time lost at the edges
        let i = 0.25;
        let d = (plant.v0 + (plant.r + 2.0 * plant.rds) * i) / (plant.v_rail - plant.r_shunt * i);
        let truth = d * plant.v_rail / i;
        let run = fit(
            &caps,
            &WaveCfg {
                ke_v_s_per_rad: 0.0,
                i_lim_a: Some(i),
                ..WaveCfg::default()
            },
        );
        assert_eq!(run.half_blind, 16);
        let bias = run.at_limit.unwrap().v_over_i_ohm / truth - 1.0;
        assert!(bias.abs() < 0.035, "V/I {bias:+} of the plant's");
    }

    /// A rest-to-step run on a synthetic plant at two duties, both signs,
    /// four of each, sampling `chans`.
    fn plant_run(
        plant: &crate::exp::testkit::SynthBurst,
        chans: crate::burst::Chans,
        pcts: [i32; 2],
    ) -> Vec<Capture> {
        use crate::exp::testkit::SynthBurst;
        let mut caps = Vec::new();
        for pct in pcts {
            for sgn in [1i32, -1] {
                for _ in 0..4 {
                    let q = (sgn * pct * 32767 / 100) as i16;
                    let p = SynthBurst {
                        chans: chans.for_step(q),
                        ..plant.clone()
                    };
                    caps.push(p.capture(q, 0));
                }
            }
        }
        caps
    }

    /// The run's own rungs on 2S, percent.
    const RUNGS: [i32; 2] = [25, 40];

    /// A hot bridge (Rds 0.20 ohm where the fit assumes 0.14), 80 mohm of
    /// copper in each low-side leg and dividers 6 counts apart, sampled
    /// shunt, A, shunt, B. Fitted on the terminals' difference, R reads
    /// within 1% of what it reads on a nominal bridge (0.14 ohm, no copper):
    /// the low side drops out. The driven terminal alone reads 0.15 ohm or
    /// more higher on the hot bridge than on the nominal one, beside the
    /// difference on the same captures and on the plant sampled shunt, A:
    /// the low side it assumes is the one it got wrong. The idle terminal
    /// reads the low-side path (FET, copper and shunt) and the OFF
    /// difference the brake, each within 3%; R, L and V/I at the limit read
    /// within 1.5% of the plant's and V0 within 25 mV.
    #[test]
    fn both_terminals_take_the_low_side_out_of_r() {
        use crate::burst::{CHANS_DIFF, Chans};
        use crate::exp::testkit::SynthBurst;
        let nominal = SynthBurst {
            r: 4.28,
            l: 0.79e-3,
            v0: 0.12,
            settle_us: 1.29,
            v_rail: 7.2,
            ..SynthBurst::board_d().with_bridge()
        };
        let hot = SynthBurst {
            rds: 0.20,
            r_low: 0.08,
            split: 6.0,
            ..nominal.clone()
        };
        let i = 0.25;
        let cfg = WaveCfg {
            ke_v_s_per_rad: 0.0,
            i_lim_a: Some(i),
            ..WaveCfg::default()
        };
        let caps = plant_run(&hot, Chans::Diff, RUNGS);
        assert!(
            caps.iter()
                .all(|c| c.meta.chans == CHANS_DIFF && c.shunt_stride() == 2)
        );
        let run = fit(&caps, &cfg);
        let base = fit(&plant_run(&nominal, Chans::Diff, RUNGS), &cfg);
        let f = run.fit;
        assert!(run.both_terminals && base.both_terminals);
        assert_eq!(f.captures, 16);
        assert!(
            (f.r_ohm / base.fit.r_ohm - 1.0).abs() < 0.01,
            "{f:?} {:?}",
            base.fit
        );

        let driven_r = |run: &WaveRun| run.driven.expect("the driven terminal beside it").0.r_ohm;
        let one = |p: &SynthBurst| fit(&plant_run(p, Chans::Driven, RUNGS), &cfg);
        assert!(one(&hot).driven.is_none() && !one(&hot).both_terminals);
        for (hot_r, nominal_r) in [
            (driven_r(&run), driven_r(&base)),
            (one(&hot).fit.r_ohm, one(&nominal).fit.r_ohm),
        ] {
            assert!(
                hot_r - nominal_r > 0.15,
                "driven R {hot_r} hot, {nominal_r} nominal"
            );
        }

        let low = hot.rds + hot.r_low;
        let lo = run.lo_side_ohm.expect("the idle terminal's line");
        assert!(
            (lo / (low + hot.r_shunt) - 1.0).abs() < 0.03,
            "low side {lo}"
        );
        assert!(
            (run.off_ohm / (2.0 * low) - 1.0).abs() < 0.03,
            "brake {}",
            run.off_ohm
        );
        assert!((f.r_ohm / hot.r - 1.0).abs() < 0.015, "{f:?}");
        assert!((f.l_h / hot.l - 1.0).abs() < 0.015, "{f:?}");
        assert!((f.v0_volts - hot.v0).abs() < 0.025, "{f:?}");
        let d = (hot.v0 + (hot.r + 2.0 * low) * i) / (hot.v_rail + (hot.r_low - hot.r_shunt) * i);
        let bias = run.at_limit.unwrap().v_over_i_ohm / (d * hot.v_rail / i) - 1.0;
        assert!(bias.abs() < 0.015, "V/I {bias:+} of the plant's");
    }

    /// The terminals sampled every fourth conversion still place the grid
    /// at the low rungs, where a 13% window may hold no sample of the
    /// driven one: folded, a 13/20% run reads R within 1.5% of the plant's.
    #[test]
    fn a_folded_terminal_reads_the_low_rungs() {
        use crate::burst::Chans;
        use crate::exp::testkit::SynthBurst;
        let plant = SynthBurst {
            r: 4.28,
            l: 0.79e-3,
            v0: 0.12,
            settle_us: 1.29,
            v_rail: 7.2,
            ..SynthBurst::board_d().with_bridge()
        };
        let caps = plant_run(&plant, Chans::Diff, [13, 20]);
        let run = fit(
            &caps,
            &WaveCfg {
                ke_v_s_per_rad: 0.0,
                ..WaveCfg::default()
            },
        );
        assert_eq!(run.fit.captures, 16, "{:?}", run.notes);
        assert!(
            (run.fit.r_ohm / plant.r - 1.0).abs() < 0.015,
            "{:?}",
            run.fit
        );
    }

    /// Nothing in an identification run starts from rest in TEL yet, so
    /// the cross route has nothing to compare: it is reported not
    /// available and stays out of the verdict. Given an intercept it
    /// passes within 8% and fails beyond.
    #[test]
    fn the_cross_route_is_reported_not_available_never_passed() {
        let caps = mg90_2s();
        let run = fit(&caps, &at_limit());
        assert!(run.gates.iter().all(|g| g.name != "cross-route"));
        assert_eq!(
            run.skipped,
            [(
                "cross-route",
                "not available: the run has no start from rest in TEL".to_string()
            )]
        );
        let v_over_i = run.at_limit.unwrap().v_over_i_ohm;
        for (tel, pass) in [(5.18, true), (4.8, false)] {
            let run = fit(
                &caps,
                &WaveCfg {
                    tel_v_over_i_ohm: Some(tel),
                    ..at_limit()
                },
            );
            assert!(run.skipped.is_empty());
            assert_eq!(
                gate(&run, "cross-route").pass,
                pass,
                "{v_over_i} against {tel}"
            );
        }
    }

    /// Sixteen captures fit, screen and split in well under a second. The
    /// test profile is optimized, as the crate's slow tests need.
    #[test]
    fn a_sixteen_capture_fit_runs_under_a_second() {
        let caps = mg90_2s();
        let t = std::time::Instant::now();
        let run = fit(&caps, &at_limit());
        let took = t.elapsed();
        assert_eq!(run.captures.len(), 16);
        assert!(took.as_secs_f64() < 1.0, "{took:?}");
    }
}
