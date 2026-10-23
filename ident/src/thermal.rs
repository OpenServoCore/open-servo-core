//! The winding estimate's constants (core `estimator::thermal`): the
//! kernel reads `t = t_ntc + rise x heat`, the heat the I2t EWMA of the held
//! current squared on `thermal_tau_s`. The board NTC's counts-to-centi-C
//! curve is a board fact, encoded from its beta constants ([`NtcCurve`]).
//! Two host routes to the rise, `th_rise_q016`: a nameplate thermal
//! resistance ([`rise_from_rth`]), or a hold's own copper against the
//! replayed heat ([`fit_rise`]). A cool-down after the hold reads the
//! copper's own time constant for `thermal_tau_s`, and the rise is fitted
//! at the leak the kernel runs for it ([`fit_thermal`]).

use osc_servo_core::kernel::phases::loops::limits::{LimitState, SLOW_PERIOD_MS, tau_shift};

use crate::exp::WindowSample;
use crate::gains::{Encoded, enc};

/// Where copper's resistance extrapolates to zero, degrees C: R(T) is
/// proportional to (234.5 + T) for annealed copper (IEC 60287-1-1).
pub const COPPER_ZERO_R_C: f64 = 234.5;

/// The NTC curve's reference temperature, C.
pub const NTC_REF_C: f64 = 25.0;

/// The board's NTC divider constants (`CalibSenseExt`).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NtcBoard {
    pub pullup_ohm: f64,
    pub r25_ohm: f64,
    pub beta: f64,
}

impl NtcBoard {
    /// The beta model: degrees C at an ADC reading, for a thermistor to
    /// GND under a pull-up to VDD. None at the rails.
    pub fn beta_c(&self, raw: f64) -> Option<f64> {
        if !(raw > 0.0 && raw < 4096.0)
            || self.pullup_ohm <= 0.0
            || self.r25_ohm <= 0.0
            || self.beta <= 0.0
        {
            return None;
        }
        let r = self.pullup_ohm * raw / (4096.0 - raw);
        Some(1.0 / (1.0 / 298.15 + (r / self.r25_ohm).ln() / self.beta) - 273.15)
    }
}

/// The board NTC's curve as the kernel runs it (`estimator::thermal::
/// NtcCfg`): `t = t_ref + k1 d + k2 d^2`, `d = raw_ref - raw`, about the
/// 25 C reading.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NtcCurve {
    pub ntc_raw_ref: Encoded,
    pub ntc_t_ref_cc: Encoded,
    pub ntc_k1_q88: Encoded,
    pub ntc_k2_q24: Encoded,
}

impl NtcCurve {
    /// The central-difference slope at the 25 C reading and the curvature
    /// that lands the ~+18 C point. None on a degenerate divider.
    pub fn new(ntc: NtcBoard) -> Option<Self> {
        let t = |raw: f64| ntc.beta_c(raw);
        let raw_ref = (4096.0 * ntc.r25_ohm / (ntc.pullup_ohm + ntc.r25_ohm)).round();
        let (lo, hi, far) = (
            t(raw_ref + 100.0)?,
            t(raw_ref - 100.0)?,
            t(raw_ref - 800.0)?,
        );
        let k1 = (hi - lo) / 200.0 * 100.0;
        let k2 = ((far - NTC_REF_C) * 100.0 - k1 * 800.0) / (800.0 * 800.0);
        if !(k1.is_finite() && k1 > 0.0 && k2.is_finite()) {
            return None;
        }
        Some(Self {
            ntc_raw_ref: enc(raw_ref, 1.0),
            ntc_t_ref_cc: enc(NTC_REF_C * 100.0, 1.0),
            ntc_k1_q88: enc(k1, 256.0),
            ntc_k2_q24: enc(k2, 16_777_216.0),
        })
    }

    /// (name, field) pairs in table order.
    pub fn fields(&self) -> [(&'static str, Encoded); 4] {
        [
            ("ntc_raw_ref", self.ntc_raw_ref),
            ("ntc_t_ref_cc", self.ntc_t_ref_cc),
            ("ntc_k1_q88", self.ntc_k1_q88),
            ("ntc_k2_q24", self.ntc_k2_q24),
        ]
    }

    /// The temperature the kernel reads for an NTC count, degrees C, in its
    /// own arithmetic.
    pub fn reads_c(&self, raw: u16) -> f64 {
        let d = self.ntc_raw_ref.raw as i64 - raw as i64;
        let t = self.ntc_t_ref_cc.raw as i16 as i64
            + ((d * self.ntc_k1_q88.raw as i16 as i64) >> 8)
            + ((d * d * self.ntc_k2_q24.raw as i64) >> 24);
        t as f64 / 100.0
    }
}

/// The rise in centi-C per count squared of heat: a held current i
/// dissipates R i^2 (A/count)^2 watts, which stands R_th C/W over the NTC.
pub fn rise_per_count2(rth_c_per_w: f64, r_ohm: f64, a_per_count: f64) -> f64 {
    100.0 * rth_c_per_w * r_ohm * a_per_count * a_per_count
}

/// `th_rise_q016` from a winding-to-NTC thermal resistance, saturating at
/// the u16 rail.
pub fn rise_from_rth(rth_c_per_w: f64, r_ohm: f64, a_per_count: f64) -> Encoded {
    enc(rise_per_count2(rth_c_per_w, r_ohm, a_per_count), 65536.0)
}

/// The thermal resistance a rise implies, C/W.
pub fn rth_of(k_cc_per_count2: f64, r_ohm: f64, a_per_count: f64) -> f64 {
    k_cc_per_count2 / (100.0 * r_ohm * a_per_count * a_per_count)
}

/// One poll of the hold: host time, the tick block's signed held current,
/// the board NTC's reading and the kernel's own winding estimate.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HoldPoll {
    pub t_ms: f64,
    pub i_counts: f64,
    pub t_ntc_cc: f64,
    pub t_winding_cc: f64,
}

/// The reference R is the mean window R over this long of the hold, from
/// the first second its current is steady.
pub const R_REF_MS: f64 = 2000.0;
/// Steady: the reference's seconds and the one after them agree on their
/// mean |i| within this fraction. The I2t fold can pull the current down
/// over a hold's first seconds, and an R read there is not the R the hold
/// runs at; a later fold or the copper's slow drift is steady at this
/// scale.
pub const R_REF_TOL: f64 = 0.03;
/// The reference lies inside this much of the hold's start, where the
/// copper still sits at the NTC.
pub const R_REF_WITHIN_MS: f64 = 10_000.0;
/// The NTC base is its mean over this long before the hold.
pub const BASE_MS: f64 = 5000.0;
/// The fit's grid.
pub const BIN_MS: f64 = 1000.0;
/// The hold's end is the mean over this many closing bins.
pub const SETTLED_BINS: usize = 10;
/// Bins a hold must cover before its rise stands.
pub const MIN_BINS: usize = 30;
/// A cool-down pulse whose copper reads under this over the NTC, C, is
/// noise to the time constant's fit.
pub const COOL_FLOOR_C: f64 = 1.0;
/// Pulses over the floor the copper's time constant needs.
pub const MIN_PULSES: usize = 5;

/// The copper's own time constant, from the cool-down's pulses.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CoolFit {
    pub tau_s: f64,
    /// Pulses over [`COOL_FLOOR_C`] the fit ran on.
    pub pulses: usize,
    /// The pulses' scatter around the fitted decay, C.
    pub residual_sd_c: f64,
    /// The fitted decay's rise over the NTC at the hold's end, C.
    pub r_end_c: f64,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RiseFit {
    /// centi-C per count squared of heat.
    pub k: f64,
    pub rise_q016: Encoded,
    pub r_ref_ohm: f64,
    /// Where the reference window starts, ms into the hold.
    pub r_ref_start_ms: f64,
    /// The C/W the rise implies on this winding and sense scale.
    pub rth_c_per_w: f64,
    /// The NTC base, C.
    pub t0_c: f64,
    /// Per-bin scatter of the copper's excess over the NTC around k x h, C.
    pub residual_sd_c: f64,
    /// The copper's excess over the NTC, the mean over the last
    /// [`SETTLED_BINS`] bins, C.
    pub settled_rise_c: f64,
    /// The settled rise over the replayed heat across the same bins,
    /// centi-C per count squared: a second k, free of the regression's
    /// weight on the transient.
    pub plateau_k: f64,
    pub plateau_rise_q016: Encoded,
    /// The kernel's winding estimate less the copper's temperature over the
    /// same closing bins, C; None while the estimate read UNSET.
    pub estimate_err_c: Option<f64>,
    pub bins: usize,
    /// The `thermal_tau_s` the heat was replayed at.
    pub thermal_tau_s: u16,
    /// The leak the kernel runs for it ([`tau_shift`]).
    pub tau_shift: u8,
    /// The copper's time constant the rise was fitted at; None for a rise
    /// fitted at a given `thermal_tau_s`.
    pub cool: Option<CoolFit>,
}

impl RiseFit {
    /// The kernel's leak in seconds: 2^shift SLOW periods.
    pub fn octave_s(&self) -> f64 {
        (1u32 << self.tau_shift) as f64 * SLOW_PERIOD_MS as f64 / 1000.0
    }
}

#[derive(Copy, Clone, Default)]
struct Acc {
    s: f64,
    n: usize,
}

impl Acc {
    fn add(&mut self, x: f64) {
        self.s += x;
        self.n += 1;
    }

    fn mean(&self) -> Option<f64> {
        (self.n > 0).then(|| self.s / self.n as f64)
    }
}

fn mean(xs: impl Iterator<Item = f64>) -> Option<f64> {
    let mut a = Acc::default();
    xs.for_each(|x| a.add(x));
    a.mean()
}

/// A window's R, vcounts per ccount: duty and vdiff carry the drive's
/// sign, i is folded by it, so every term is positive into the stop.
fn window_r_vpc(w: &WindowSample) -> Option<f64> {
    (w.i != 0.0 && w.duty_q15 != 0.0)
        .then(|| (w.duty_q15 * w.vdiff / 32767.0) / (w.i * w.duty_q15.signum()))
}

/// The kernel's heat over the polls: its own integer EWMA of the held
/// current squared stepped on the SLOW grid, each step holding the last
/// poll at or before it. Sampled at `at_ms` (ascending).
pub fn replay_heat(polls: &[HoldPoll], thermal_tau_s: u16, at_ms: &[f64]) -> Vec<f64> {
    let shift = tau_shift(thermal_tau_s);
    let Some(first) = polls.first() else {
        return vec![0.0; at_ms.len()];
    };
    let mut st = LimitState::new();
    let mut out = Vec::with_capacity(at_ms.len());
    let (mut p, mut g) = (0usize, first.t_ms);
    for &at in at_ms {
        while g <= at {
            while p + 1 < polls.len() && polls[p + 1].t_ms <= g {
                p += 1;
            }
            let i = polls[p].i_counts.abs().round().min(i16::MAX as f64) as i16;
            st.heat(i, shift);
            g += SLOW_PERIOD_MS as f64;
        }
        out.push(st.heat_counts2() as f64);
    }
    out
}

/// One second of the hold.
#[derive(Copy, Clone, Default)]
struct Second {
    r: Acc,
    i: Acc,
    ntc: Acc,
    winding: Acc,
    unset: bool,
}

/// What every fit over a hold reads the copper against: the NTC base T_0,
/// the reference R and the hold's seconds.
struct Reference {
    t0_c: f64,
    r_ref: f64,
    r_ref_start_ms: f64,
    seconds: Vec<Second>,
}

impl Reference {
    /// The copper's rise over the NTC base for a window R, C.
    fn copper(&self, r: f64) -> f64 {
        (r / self.r_ref - 1.0) * (COPPER_ZERO_R_C + self.t0_c)
    }
}

/// The hold from `t0_ms` to `end_ms` in one-second bins, T_0 the NTC's
/// mean over the [`BASE_MS`] before it, R_ref the mean window R over the
/// first [`R_REF_MS`] inside [`R_REF_WITHIN_MS`] whose seconds and the one
/// after them agree on their mean |i| within [`R_REF_TOL`].
fn reference(
    polls: &[HoldPoll],
    windows: &[WindowSample],
    t0_ms: f64,
    end_ms: f64,
    r_ohm: impl Fn(&WindowSample) -> Option<f64>,
) -> Result<Reference, String> {
    let t0_c = mean(
        polls
            .iter()
            .filter(|p| p.t_ms >= t0_ms - BASE_MS && p.t_ms < t0_ms)
            .map(|p| p.t_ntc_cc / 100.0),
    )
    .ok_or("no NTC reading in the 5 s before the hold")?;
    let held = |t: f64| t >= t0_ms && t < end_ms;
    let last = windows
        .iter()
        .map(|w| w.t_ms)
        .filter(|t| held(*t))
        .fold(t0_ms, f64::max);
    let n = ((last - t0_ms) / BIN_MS).floor() as usize + 1;
    let bin = |t: f64| ((t - t0_ms) / BIN_MS).floor() as usize;
    let mut seconds = vec![Second::default(); n];
    for w in windows.iter().filter(|w| held(w.t_ms)) {
        if let (Some(r), Some(s)) = (r_ohm(w), seconds.get_mut(bin(w.t_ms))) {
            s.r.add(r);
            s.i.add(w.i.abs());
        }
    }
    for p in polls.iter().filter(|p| held(p.t_ms)) {
        if let Some(s) = seconds.get_mut(bin(p.t_ms)) {
            s.ntc.add(p.t_ntc_cc / 100.0);
            s.winding.add(p.t_winding_cc / 100.0);
            s.unset |= p.t_winding_cc <= i16::MIN as f64;
        }
    }
    let span = (R_REF_MS / BIN_MS).round().max(1.0) as usize;
    let within = (R_REF_WITHIN_MS / BIN_MS).round() as usize;
    let steady = |w: &[Second]| {
        let i: Option<Vec<f64>> = w.iter().map(|s| s.i.mean()).collect();
        i.is_some_and(|i| {
            let lo = i.iter().copied().fold(f64::INFINITY, f64::min);
            let hi = i.iter().copied().fold(0.0, f64::max);
            lo > 0.0 && hi - lo <= R_REF_TOL * lo
        })
    };
    let (j, r_ref) = seconds
        .windows(span + 1)
        .take(within.saturating_sub(span) + 1)
        .enumerate()
        .find(|(_, w)| steady(w))
        .map(|(j, w)| {
            let r = w[..span].iter().fold(Acc::default(), |a, s| Acc {
                s: a.s + s.r.s,
                n: a.n + s.r.n,
            });
            (j, r.mean())
        })
        .ok_or_else(|| {
            format!(
                "the current never settled in the hold's first {:.0} s: no {:.0} s of it and the \
                 second after agree within {:.0} %",
                R_REF_WITHIN_MS / 1000.0,
                R_REF_MS / 1000.0,
                R_REF_TOL * 100.0
            )
        })?;
    let r_ref = r_ref
        .filter(|r| r.is_finite() && *r > 0.0)
        .ok_or("the reference window reads no R")?;
    Ok(Reference {
        t0_c,
        r_ref,
        r_ref_start_ms: j as f64 * BIN_MS,
        seconds,
    })
}

fn r_ohm_at(
    v_per_count: f64,
    a_per_count: f64,
) -> Result<impl Fn(&WindowSample) -> Option<f64>, String> {
    if !(v_per_count > 0.0 && a_per_count > 0.0) {
        return Err("the sense scales are degenerate".into());
    }
    Ok(move |w: &WindowSample| window_r_vpc(w).map(|r| r * v_per_count / a_per_count))
}

struct Bin {
    centre_ms: f64,
    x_w: f64,
    x_n: f64,
    t_winding_c: Option<f64>,
}

/// The rise from a hold's own copper: per 1 s bin from the hold start
/// `t0_ms` to its end `end_ms`, the copper's rise
/// `x_w = (R / R_ref - 1)(234.5 + T_0)` less the NTC's `x_n = T_ntc - T_0`,
/// regressed through the origin on the heat replayed at `thermal_tau_s`
/// ([`reference`] for R_ref and T_0). `v_per_count` / `a_per_count` the
/// board's terminal and current scales.
pub fn fit_rise(
    polls: &[HoldPoll],
    windows: &[WindowSample],
    t0_ms: f64,
    end_ms: f64,
    thermal_tau_s: u16,
    v_per_count: f64,
    a_per_count: f64,
) -> Result<RiseFit, String> {
    let r_ohm = r_ohm_at(v_per_count, a_per_count)?;
    let rf = reference(polls, windows, t0_ms, end_ms, r_ohm)?;
    rise_at(polls, &rf, t0_ms, thermal_tau_s, a_per_count)
}

fn rise_at(
    polls: &[HoldPoll],
    rf: &Reference,
    t0_ms: f64,
    thermal_tau_s: u16,
    a_per_count: f64,
) -> Result<RiseFit, String> {
    let bins: Vec<Bin> = rf
        .seconds
        .iter()
        .enumerate()
        .filter_map(|(j, s)| {
            let (r, t) = (s.r.mean()?, s.ntc.mean()?);
            Some(Bin {
                centre_ms: t0_ms + (j as f64 + 0.5) * BIN_MS,
                x_w: rf.copper(r),
                x_n: t - rf.t0_c,
                t_winding_c: s.winding.mean().filter(|_| !s.unset),
            })
        })
        .collect();
    if bins.len() < MIN_BINS {
        return Err(format!(
            "the hold covers {} one-second bins with a window and an NTC reading, under the \
             {MIN_BINS} a rise needs",
            bins.len()
        ));
    }
    let at: Vec<f64> = bins.iter().map(|b| b.centre_ms).collect();
    let h = replay_heat(polls, thermal_tau_s, &at);
    let shh: f64 = h.iter().map(|h| h * h).sum();
    if shh <= 0.0 {
        return Err("the hold carried no heat".into());
    }
    let k_c = h
        .iter()
        .zip(&bins)
        .map(|(h, b)| h * (b.x_w - b.x_n))
        .sum::<f64>()
        / shh;
    let res: Vec<f64> = h
        .iter()
        .zip(&bins)
        .map(|(h, b)| b.x_w - b.x_n - k_c * h)
        .collect();
    let residual_sd_c = (res.iter().map(|e| e * e).sum::<f64>() / (res.len() - 1) as f64).sqrt();
    let from = bins.len().saturating_sub(SETTLED_BINS);
    let settled = &bins[from..];
    let m = settled.len() as f64;
    let settled_rise_c = settled.iter().map(|b| b.x_w - b.x_n).sum::<f64>() / m;
    let settled_heat = h[from..].iter().sum::<f64>() / m;
    if settled_heat <= 0.0 {
        return Err("the hold's closing seconds carried no heat".into());
    }
    let copper_c = rf.t0_c + settled.iter().map(|b| b.x_w).sum::<f64>() / m;
    let estimate_err_c = settled
        .iter()
        .map(|b| b.t_winding_c)
        .sum::<Option<f64>>()
        .map(|s| s / m - copper_c);
    let k = k_c * 100.0;
    let plateau_k = settled_rise_c / settled_heat * 100.0;
    Ok(RiseFit {
        k,
        rise_q016: enc(k, 65536.0),
        r_ref_ohm: rf.r_ref,
        r_ref_start_ms: rf.r_ref_start_ms,
        rth_c_per_w: rth_of(k, rf.r_ref, a_per_count),
        t0_c: rf.t0_c,
        residual_sd_c,
        settled_rise_c,
        plateau_k,
        plateau_rise_q016: enc(plateau_k, 65536.0),
        estimate_err_c,
        bins: bins.len(),
        thermal_tau_s,
        tau_shift: tau_shift(thermal_tau_s),
        cool: None,
    })
}

/// The copper's time constant from the cool-down that starts at `cool_ms`:
/// each pulse (a run of polls with current) reads the copper's rise over
/// the NTC `r` from the later half of its windows, and `ln r` against the
/// time since `cool_ms` is a line of slope `-1 / tau` over the pulses whose
/// `r` reads over [`COOL_FLOOR_C`]. The pulses' own heat is left out of the
/// model: it holds the copper up by about the pulse duty cycle of the
/// hold's rise, which reads tau long by about that fraction.
fn cool_at(
    polls: &[HoldPoll],
    windows: &[WindowSample],
    rf: &Reference,
    cool_ms: f64,
    r_ohm: impl Fn(&WindowSample) -> Option<f64>,
) -> Result<CoolFit, String> {
    let cool: Vec<&HoldPoll> = polls.iter().filter(|p| p.t_ms >= cool_ms).collect();
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut start = None;
    for (n, p) in cool.iter().enumerate() {
        match (p.i_counts != 0.0, start) {
            (true, None) => start = Some(n),
            (false, Some(s)) => {
                runs.push((s, n));
                start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = start {
        runs.push((s, cool.len()));
    }
    let pulses: Vec<(f64, f64)> = runs
        .iter()
        .filter_map(|&(a, z)| {
            let from = cool[a].t_ms;
            let to = cool.get(z).map_or(f64::INFINITY, |p| p.t_ms);
            let rs: Vec<(f64, f64)> = windows
                .iter()
                .filter(|w| w.t_ms >= from && w.t_ms < to)
                .filter_map(|w| Some((w.t_ms, r_ohm(w)?)))
                .collect();
            let late = &rs[rs.len() / 2..];
            let t = mean(late.iter().map(|x| x.0))?;
            let r = mean(late.iter().map(|x| x.1))?;
            let ntc = mean(cool[a..z].iter().map(|p| p.t_ntc_cc / 100.0))?;
            Some(((t - cool_ms) / 1000.0, rf.copper(r) - (ntc - rf.t0_c)))
        })
        .collect();
    if pulses.is_empty() {
        return Err("the cool-down landed no measurement pulse".into());
    }
    let over: Vec<(f64, f64)> = pulses
        .iter()
        .copied()
        .filter(|p| p.1 > COOL_FLOOR_C)
        .collect();
    if over.is_empty() {
        let top = pulses.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
        return Err(format!(
            "the copper's rise over the NTC never read over the {COOL_FLOOR_C} C floor through \
             the cool-down (at most {top:.2} C)"
        ));
    }
    if over.len() < MIN_PULSES {
        return Err(format!(
            "{} of the cool-down's {} pulses read the copper over the {COOL_FLOOR_C} C floor, \
             under the {MIN_PULSES} its time constant needs",
            over.len(),
            pulses.len()
        ));
    }
    let n = over.len() as f64;
    let xm = over.iter().map(|p| p.0).sum::<f64>() / n;
    let ym = over.iter().map(|p| p.1.ln()).sum::<f64>() / n;
    let sxx: f64 = over.iter().map(|p| (p.0 - xm).powi(2)).sum();
    let sxy: f64 = over.iter().map(|p| (p.0 - xm) * (p.1.ln() - ym)).sum();
    let slope = sxy / sxx;
    if !(slope < 0.0 && slope.is_finite()) {
        return Err(format!(
            "the copper did not cool over the cool-down's {} pulses over the floor",
            over.len()
        ));
    }
    let tau_s = -1.0 / slope;
    let r_end_c = (ym - slope * xm).exp();
    let ss: f64 = over
        .iter()
        .map(|p| (p.1 - r_end_c * (-p.0 / tau_s).exp()).powi(2))
        .sum();
    Ok(CoolFit {
        tau_s,
        pulses: over.len(),
        residual_sd_c: (ss / (n - 2.0)).sqrt(),
        r_end_c,
    })
}

/// The hold and its cool-down from `cool_ms`: the copper's time constant
/// from the cool-down ([`cool_at`]), rounded to the second as
/// `thermal_tau_s`, then the rise over the hold ([`fit_rise`]) on the heat
/// replayed at the leak the kernel runs for it.
pub fn fit_thermal(
    polls: &[HoldPoll],
    windows: &[WindowSample],
    t0_ms: f64,
    cool_ms: f64,
    v_per_count: f64,
    a_per_count: f64,
) -> Result<RiseFit, String> {
    let r_ohm = r_ohm_at(v_per_count, a_per_count)?;
    let rf = reference(polls, windows, t0_ms, cool_ms, &r_ohm)?;
    let cool = cool_at(polls, windows, &rf, cool_ms, &r_ohm)?;
    let tau_s = cool.tau_s.round().max(1.0).min(u16::MAX as f64) as u16;
    Ok(RiseFit {
        cool: Some(cool),
        ..rise_at(polls, &rf, t0_ms, tau_s, a_per_count)?
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{self, SenseParams};

    /// The dev-v006 rev 2A front end (units' own pin).
    const DEV: SenseParams = SenseParams {
        shunt_r_mohm: 60,
        gain_milli: 14_884,
        vmotor_div_top: 6_400,
        vmotor_div_bot: 1_600,
        vdd_mv: 3300,
        tick_hz: 20_000,
    };

    #[test]
    fn tau_shift_rounds_to_the_nearest_octave() {
        assert_eq!(SLOW_PERIOD_MS, 16);
        assert_eq!(tau_shift(0), 0);
        assert_eq!(tau_shift(1), 6);
        assert_eq!(tau_shift(4), 8);
        assert_eq!(tau_shift(66), 12);
        assert_eq!(tau_shift(115), 13);
        assert_eq!(tau_shift(262), 14);
        assert_eq!(tau_shift(u16::MAX), 14);
    }

    /// The bench MG90 on the dev board, 4.51 ohm at 21.0 C/W over the NTC.
    #[test]
    fn a_nameplate_rth_encodes_the_rise() {
        let a = units::amps_per_count(&DEV);
        let e = rise_from_rth(21.0, 4.51, a);
        assert_eq!(e.raw, 505, "{a} A/count -> {e:?}");
        assert!(!e.saturated);
        assert!((rth_of(e.physical, 4.51, a) - 21.0).abs() < 1e-9);
        assert!(rise_from_rth(1e9, 4.51, a).saturated);
        assert_eq!(rise_from_rth(1e9, 4.51, a).raw, u16::MAX);
    }

    /// A hold whose copper rises over the NTC by exactly k x h at each bin
    /// centre, the NTC climbing on its own: the fit reads k back.
    #[test]
    fn recovers_the_rise_of_a_synthetic_hold() {
        const K_C: f64 = 0.0105;
        const TAU_S: u16 = 4;
        const T0: f64 = 0.0;
        const T_NTC0: f64 = 27.5;
        const I: f64 = -270.0;
        const DUTY: f64 = -5000.0;
        let v = units::volts_per_count(&DEV);
        let a = units::amps_per_count(&DEV);
        let r_ref_vpc = 1.0;
        let polls: Vec<HoldPoll> = (-500..30_000)
            .map(|n| {
                let t_ms = n as f64 * SLOW_PERIOD_MS as f64;
                HoldPoll {
                    t_ms,
                    i_counts: if t_ms >= T0 { I } else { 0.0 },
                    t_ntc_cc: 0.0,
                    t_winding_cc: 0.0,
                }
            })
            .collect();
        let bins = 470;
        let centres: Vec<f64> = (0..bins).map(|j| T0 + (j as f64 + 0.5) * BIN_MS).collect();
        let h = replay_heat(&polls, TAU_S, &centres);
        // the NTC climbs on its own; for the first three bins it falls
        // instead by the copper's excess, so R reads R_ref where R_ref is
        // read
        let x_n: Vec<f64> = (0..bins)
            .map(|j| if j < 3 { -K_C * h[j] } else { 0.004 * j as f64 })
            .collect();
        let polls: Vec<HoldPoll> = polls
            .into_iter()
            .map(|p| {
                let j = ((p.t_ms - T0) / BIN_MS).floor();
                let x = if j >= 0.0 {
                    x_n.get(j as usize).copied().unwrap_or(0.0)
                } else {
                    0.0
                };
                // the kernel's estimate reads 0.5 C over the copper
                let x_w = x + K_C * h.get(j.max(0.0) as usize).copied().unwrap_or(0.0);
                HoldPoll {
                    t_ntc_cc: (T_NTC0 + x) * 100.0,
                    t_winding_cc: (T_NTC0 + x_w + 0.5) * 100.0,
                    ..p
                }
            })
            .collect();
        let windows: Vec<WindowSample> = (0..bins)
            .map(|j| {
                let x_w = x_n[j] + K_C * h[j];
                let r = r_ref_vpc * (1.0 + x_w / (COPPER_ZERO_R_C + T_NTC0));
                WindowSample {
                    t_ms: centres[j],
                    i: I,
                    vdiff: r * I * DUTY.signum() * 32767.0 / DUTY,
                    duty_q15: DUTY,
                }
            })
            .collect();
        let f = fit_rise(&polls, &windows, T0, f64::INFINITY, TAU_S, v, a).unwrap();
        assert!((f.k - K_C * 100.0).abs() < 1e-9, "{f:?}");
        assert_eq!(f.rise_q016.raw, (K_C * 100.0 * 65536.0).round() as u16);
        assert!(f.residual_sd_c < 1e-9, "{f:?}");
        assert!((f.t0_c - T_NTC0).abs() < 1e-12);
        assert!((f.r_ref_ohm - r_ref_vpc * v / a).abs() < 1e-9);
        assert!((f.rth_c_per_w - rth_of(f.k, f.r_ref_ohm, a)).abs() < 1e-12);
        assert_eq!(f.bins, bins);
        let steady = K_C * h[bins - 1];
        assert!((f.settled_rise_c - steady).abs() < 0.01 * steady, "{f:?}");
        assert!((f.estimate_err_c.unwrap() - 0.5).abs() < 1e-9, "{f:?}");
        assert_eq!(f.r_ref_start_ms, 0.0);
        assert_eq!((f.thermal_tau_s, f.tau_shift, f.cool), (TAU_S, 8, None));
    }

    const I: f64 = -270.0;
    const DUTY: f64 = -5000.0;
    const T_NTC0: f64 = 27.5;
    const POLL_MS: f64 = 20.0;
    /// 40 C over the NTC at 270 counts, centi-C per count squared.
    const K: f64 = 4000.0 / (270.0 * 270.0);

    /// A window at `t_ms` whose R reads the copper `x_w` C over the NTC
    /// base, on a 1 vcount per ccount R_ref.
    fn window(t_ms: f64, i: f64, x_w: f64) -> WindowSample {
        let r = 1.0 + x_w / (COPPER_ZERO_R_C + T_NTC0);
        WindowSample {
            t_ms,
            i,
            vdiff: r * i * DUTY.signum() * 32767.0 / DUTY,
            duty_q15: DUTY,
        }
    }

    /// A synthetic hold from 0 to `hold_ms` at [`I`] times `scale` (`first`
    /// through its first second) whose copper rises over the NTC by exactly k x h on
    /// the heat replayed at `tau_s`, one window per second at the bin's
    /// centre. The NTC sits at its base but for the first four seconds,
    /// where it falls by the copper's excess so R reads R_ref there. Then
    /// `cool` = (length, every, pulse ms, tau s): pulses at [`I`] whose
    /// copper decays from the hold's end on its own tau, a window every
    /// 5 ms.
    struct Synth {
        hold_ms: f64,
        k: f64,
        tau_s: u16,
        first: f64,
        scale: fn(f64) -> f64,
        cool: Option<(f64, f64, f64, f64)>,
    }

    impl Synth {
        fn hold(hold_ms: f64, tau_s: u16) -> Self {
            Self {
                hold_ms,
                k: K,
                tau_s,
                first: I,
                scale: |_| 1.0,
                cool: None,
            }
        }

        fn pulse(&self, t_ms: f64) -> bool {
            self.cool.is_some_and(|(len, every, pulse, _)| {
                let e = t_ms - self.hold_ms;
                let k = (e / every).floor();
                e >= 0.0 && k >= 1.0 && e - k * every < pulse && k * every + pulse <= len
            })
        }

        fn current(&self, t_ms: f64) -> f64 {
            match t_ms {
                t if t < 0.0 => 0.0,
                t if t < 1000.0 => self.first,
                t if t < self.hold_ms => I * (self.scale)(t),
                t if self.pulse(t) => I,
                _ => 0.0,
            }
        }

        fn build(&self) -> (Vec<HoldPoll>, Vec<WindowSample>) {
            let k_c = self.k / 100.0;
            let end = self.hold_ms + self.cool.map_or(0.0, |c| c.0);
            let mut polls: Vec<HoldPoll> = (-300..=(end / POLL_MS) as i64)
                .map(|n| {
                    let t_ms = n as f64 * POLL_MS;
                    HoldPoll {
                        t_ms,
                        i_counts: self.current(t_ms),
                        t_ntc_cc: T_NTC0 * 100.0,
                        t_winding_cc: T_NTC0 * 100.0,
                    }
                })
                .collect();
            let bins = (self.hold_ms / BIN_MS) as usize;
            let centres: Vec<f64> = (0..bins).map(|j| (j as f64 + 0.5) * BIN_MS).collect();
            let h = replay_heat(&polls, self.tau_s, &centres);
            for p in polls.iter_mut() {
                if (0.0..4000.0).contains(&p.t_ms) {
                    let j = (p.t_ms / BIN_MS) as usize;
                    p.t_ntc_cc -= k_c * h[j] * 100.0;
                }
            }
            let mut windows: Vec<WindowSample> = centres
                .iter()
                .zip(&h)
                .map(|(&t, h)| {
                    let x_w = if t < 4000.0 { 0.0 } else { k_c * h };
                    window(t, self.current(t), x_w)
                })
                .collect();
            if let Some((_, _, _, tau)) = self.cool {
                let r_end = k_c * replay_heat(&polls, self.tau_s, &[self.hold_ms])[0];
                let mut t = self.hold_ms;
                while t < end {
                    if self.pulse(t) {
                        let x_w = r_end * (-(t - self.hold_ms) / 1000.0 / tau).exp();
                        windows.push(window(t, I, x_w));
                    }
                    t += 5.0;
                }
            }
            (polls, windows)
        }
    }

    fn scales() -> (f64, f64) {
        (units::volts_per_count(&DEV), units::amps_per_count(&DEV))
    }

    /// A hold and a cool-down whose copper decays on 115 s: the fit reads
    /// the tau back and the rise at the octave the kernel runs for it.
    #[test]
    fn a_cool_down_reads_the_copper_tau_and_the_rise_at_its_octave() {
        let (v, a) = scales();
        let s = Synth {
            cool: Some((480_000.0, 10_000.0, 200.0, 115.0)),
            ..Synth::hold(480_000.0, 115)
        };
        let (polls, windows) = s.build();
        let f = fit_thermal(&polls, &windows, 0.0, s.hold_ms, v, a).unwrap();
        let cool = f.cool.expect("a cool-down fit");
        assert!((cool.tau_s - 115.0).abs() < 0.02 * 115.0, "{f:?}");
        assert_eq!((f.thermal_tau_s, f.tau_shift), (115, 13), "{f:?}");
        assert!((f.octave_s() - 131.072).abs() < 1e-9);
        assert!((f.k - K).abs() < 0.02 * K, "{f:?}");
        assert!((f.plateau_k - K).abs() < 0.02 * K, "{f:?}");
        assert_eq!(f.bins, 480);
        // r_end is ~39 C: the pulses over 1 C run to ~421 s
        assert!((40..=43).contains(&cool.pulses), "{cool:?}");
        assert!(cool.residual_sd_c < 0.01, "{cool:?}");
        let r_end = K / 100.0 * replay_heat(&polls, 115, &[s.hold_ms])[0];
        assert!((cool.r_end_c - r_end).abs() < 0.01 * r_end, "{cool:?}");
    }

    /// A first second at twice the current reads a different R: the
    /// reference waits for the current to settle.
    #[test]
    fn the_reference_waits_for_the_current_to_settle() {
        let (v, a) = scales();
        let s = Synth {
            first: 2.0 * I,
            ..Synth::hold(120_000.0, 4)
        };
        let (polls, mut windows) = s.build();
        let first = windows[0];
        windows[0] = window(first.t_ms, first.i, 0.1 * (COPPER_ZERO_R_C + T_NTC0));
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 4, v, a).unwrap();
        assert_eq!(f.r_ref_start_ms, 1000.0, "{f:?}");
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9 * v / a, "{f:?}");
        assert!((f.k - K).abs() < 0.02 * K, "{f:?}");
    }

    /// A fold a minute into the hold, as a long tau lands it, is steady at
    /// the reference's scale: the reference stays at the hold's start.
    #[test]
    fn a_late_fold_leaves_the_reference_at_the_start() {
        let (v, a) = scales();
        let s = Synth {
            scale: |t| if t < 60_000.0 { 1.0 } else { 0.6 },
            ..Synth::hold(240_000.0, 115)
        };
        let (polls, windows) = s.build();
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 115, v, a).unwrap();
        assert_eq!(f.r_ref_start_ms, 0.0, "{f:?}");
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9 * v / a, "{f:?}");
        assert!((f.k - K).abs() < 0.02 * K, "{f:?}");
    }

    /// A hold the limiter does not govern: its current falls 0.4 %/C as
    /// the copper rises to 15 C, slow at the reference's scale.
    #[test]
    fn a_current_that_drifts_with_the_copper_leaves_the_reference_at_the_start() {
        let (v, a) = scales();
        let s = Synth {
            k: 1500.0 / (270.0 * 270.0),
            scale: |t| 1.0 - 0.004 * 15.0 * (1.0 - (-t / 131_072.0).exp()),
            ..Synth::hold(480_000.0, 115)
        };
        let (polls, windows) = s.build();
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 115, v, a).unwrap();
        assert_eq!(f.r_ref_start_ms, 0.0, "{f:?}");
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9 * v / a, "{f:?}");
    }

    /// A current that ramps 5 %/s through the first 12 s never settles
    /// where the copper still sits at the NTC: no reference, no rise.
    #[test]
    fn a_current_that_never_settles_early_is_refused() {
        let (v, a) = scales();
        let s = Synth {
            scale: |t| 1.0 - 0.05 * t.min(12_000.0) / 1000.0,
            ..Synth::hold(120_000.0, 4)
        };
        let (polls, windows) = s.build();
        let why = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 4, v, a).unwrap_err();
        assert!(
            why.contains("the current never settled in the hold's first 10 s"),
            "{why}"
        );
    }

    /// A copper that follows the replayed heat exactly settles where the
    /// regression says: the plateau's k is the regression's.
    #[test]
    fn the_plateau_reads_the_regression_on_a_copper_that_follows_the_heat() {
        let (v, a) = scales();
        let s = Synth::hold(300_000.0, 66);
        let (polls, windows) = s.build();
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 66, v, a).unwrap();
        assert!((f.k - K).abs() < 1e-9 * K, "{f:?}");
        assert!((f.plateau_k - f.k).abs() < 1e-9 * K, "{f:?}");
        assert_eq!(f.plateau_rise_q016.raw, f.rise_q016.raw);
    }

    #[test]
    fn a_cool_down_short_of_pulses_or_rise_is_refused() {
        let (v, a) = scales();
        let fit = |s: &Synth| {
            let (polls, windows) = s.build();
            fit_thermal(&polls, &windows, 0.0, s.hold_ms, v, a).unwrap_err()
        };
        let few = Synth {
            cool: Some((40_000.0, 10_000.0, 200.0, 115.0)),
            ..Synth::hold(120_000.0, 115)
        };
        let why = fit(&few);
        assert!(why.contains("3 of the cool-down's 3 pulses"), "{why}");
        assert!(why.contains("under the 5"), "{why}");
        let cold = Synth {
            k: 0.5 / (270.0 * 270.0) * 100.0,
            cool: Some((120_000.0, 10_000.0, 200.0, 115.0)),
            ..Synth::hold(120_000.0, 115)
        };
        let why = fit(&cold);
        assert!(why.contains("never read over the 1 C floor"), "{why}");
        let none = Synth::hold(120_000.0, 115);
        let why = fit(&none);
        assert!(why.contains("no measurement pulse"), "{why}");
    }

    #[test]
    fn a_short_or_unbased_hold_is_refused() {
        let v = units::volts_per_count(&DEV);
        let a = units::amps_per_count(&DEV);
        let w = |t_ms| WindowSample {
            t_ms,
            i: -270.0,
            vdiff: -800.0,
            duty_q15: -5000.0,
        };
        let p = |t_ms| HoldPoll {
            t_ms,
            i_counts: -270.0,
            t_ntc_cc: 2750.0,
            t_winding_cc: 2750.0,
        };
        let short: Vec<WindowSample> = (0..10).map(|k| w(k as f64 * 500.0)).collect();
        let polls: Vec<HoldPoll> = (-10..10).map(|k| p(k as f64 * 500.0)).collect();
        assert!(
            fit_rise(&polls, &short, 0.0, f64::INFINITY, 4, v, a)
                .unwrap_err()
                .contains("bins")
        );
        let late: Vec<HoldPoll> = (0..10).map(|k| p(k as f64 * 500.0)).collect();
        assert!(
            fit_rise(&late, &short, 0.0, f64::INFINITY, 4, v, a)
                .unwrap_err()
                .contains("before the hold")
        );
    }

    const NTC: NtcBoard = NtcBoard {
        pullup_ohm: 10_000.0,
        r25_ohm: 10_000.0,
        beta: 3950.0,
    };

    #[test]
    fn encodes_the_10k_3950_ntc_curve() {
        let c = NtcCurve::new(NTC).expect("a curve");
        assert_eq!(c.ntc_raw_ref.raw, 2048);
        assert_eq!(c.ntc_t_ref_cc.raw, 2500);
        assert!(
            (c.ntc_k1_q88.raw as i32 - 563).abs() <= 4,
            "{}",
            c.ntc_k1_q88.raw
        );
        assert!(
            (c.ntc_k2_q24.raw as i32 - 5800).abs() <= 300,
            "{}",
            c.ntc_k2_q24.raw
        );
        assert!(c.fields().iter().all(|(_, f)| !f.saturated));
        assert!(NtcCurve::new(NtcBoard { beta: 0.0, ..NTC }).is_none());
        assert!(
            NtcCurve::new(NtcBoard {
                r25_ohm: 0.0,
                ..NTC
            })
            .is_none()
        );
    }

    /// Through the kernel's arithmetic the quadratic reads the beta model
    /// exactly at 25 C and within 0.7 C from 15 to 50 C.
    #[test]
    fn the_kernel_reads_the_beta_curve_through_the_quadratic() {
        let c = NtcCurve::new(NTC).unwrap();
        let at_ref = c.ntc_raw_ref.raw;
        assert!((NTC.beta_c(at_ref as f64).unwrap() - 25.0).abs() < 1e-9);
        assert_eq!(c.reads_c(at_ref), 25.0);
        for t in [15.0f64, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0] {
            let r = 10_000.0 * (3950.0 * (1.0 / (t + 273.15) - 1.0 / 298.15)).exp();
            let raw = (4096.0 * r / (10_000.0 + r)).round();
            let truth = NTC.beta_c(raw).unwrap();
            let read = c.reads_c(raw as u16);
            assert!(
                (read - truth).abs() < 0.7,
                "{t} C: reads {read} for {truth}"
            );
        }
    }
}
