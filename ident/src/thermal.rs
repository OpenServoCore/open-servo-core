//! The winding estimate's constants (core `estimator::thermal`): the
//! kernel reads `t = t_ntc + rise x heat`, the heat the I2t EWMA of the held
//! current squared on `thermal_tau_s`. The board NTC's counts-to-centi-C
//! curve is a board fact, encoded from its beta constants ([`NtcCurve`]).
//! Two host routes to the rise, `th_rise_q016`: a nameplate thermal
//! resistance ([`rise_from_rth`]), or a hold's own copper against the
//! replayed heat ([`fit_rise`]). A cool-down after the hold reads the
//! copper's own time constant for `thermal_tau_s`, and the rise is fitted
//! at the leak the kernel runs for it ([`fit_thermal`]). R moves with the
//! operating point by a few percent across duties, so a copper reading
//! never takes a ratio across duties: the cool-down re-reads at the hold's
//! own duty, against the hold's own start.

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
/// the board NTC's reading, the kernel's own winding estimate and the
/// current ceiling it composed.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HoldPoll {
    pub t_ms: f64,
    pub i_counts: f64,
    pub t_ntc_cc: f64,
    pub t_winding_cc: f64,
    pub i_lim_counts: f64,
}

/// One window of the thermal run, judged as it landed.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ThermWindow {
    pub w: WindowSample,
    /// The pot at the window's end: its last row's, else the poll's that
    /// drained it.
    pub pos: u16,
    /// Its applied duty is the duty commanded for it: no slew still
    /// climbing, no limiter holding it under.
    pub commanded: bool,
    /// Its applied duty clears the higher published window floor: the
    /// firmware's own test for `I_VALID` and `V_VALID` (`window::select`).
    pub floors: bool,
    /// The shaft sat within the seat's stillness.
    pub still: bool,
}

impl ThermWindow {
    /// The window reads the winding: over the floors, carrying current,
    /// the shaft seated.
    pub fn reads(&self) -> bool {
        self.floors && self.still && self.w.i != 0.0
    }
}

/// A re-read pulse of the cool-down, at the hold's duty: the polls its
/// duty was written on and off at. Its windows land between the two.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Pulse {
    pub on_ms: f64,
    pub off_ms: f64,
}

/// What [`fit_thermal`] reads: every poll and window of the run, the
/// cool-down's pulses, where the hold and the cool-down start, and the
/// electrical settle a pulse's windows wait out.
#[derive(Copy, Clone, Debug)]
pub struct Recording<'a> {
    pub polls: &'a [HoldPoll],
    pub windows: &'a [ThermWindow],
    pub pulses: &'a [Pulse],
    pub t0_ms: f64,
    pub cool_ms: f64,
    pub settle_ms: f64,
}

/// R_ref is a line through the hold's windows at its commanded duty over
/// this long from the first, read at the first: the bare copper warms
/// several C in the hold's first second, and a mean over it reads R_ref
/// warm.
pub const LINE_MS: f64 = 1000.0;
/// The fewest windows the line takes.
pub const LINE_MIN: usize = 3;
/// The reference lies inside this much of the hold's start, where the
/// copper still sits at the NTC.
pub const R_REF_WITHIN_MS: f64 = 10_000.0;
/// The hold's cold current sits at least this fraction under the current
/// ceiling: the copper only lowers it from there, so the cool-down's
/// re-reads at the hold's duty are never chopped.
pub const HOLD_MARGIN: f64 = 0.10;
/// The hold's stall is planned at this fraction of the stall-safe cap: the
/// plan comes from the stored R and the live current varies a few percent
/// run to run, so the plan sits a margin's width inside the refusal.
pub const HOLD_TARGET: f64 = 0.80;
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

/// One cool-down pulse's copper.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CoolPulse {
    /// Its settled windows' mean time, s into the cool-down.
    pub t_s: f64,
    pub windows: usize,
    /// The copper over the NTC, C.
    pub rise_c: f64,
    /// Its windows' scatter, as copper C.
    pub sd_c: f64,
}

/// The cool-down's windows the validity filter dropped, by the first
/// reason each failed, and the pulses left with none.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Dropped {
    pub pulses: usize,
    /// Applied off the commanded duty: the slew, or the limiter.
    pub off_duty: usize,
    /// Under the floors, or no current.
    pub unread: usize,
    /// The shaft off its seat.
    pub moved: usize,
}

/// The copper's own time constant, from the cool-down's pulses.
#[derive(Clone, Debug, PartialEq)]
pub struct CoolFit {
    pub tau_s: f64,
    /// Pulses over [`COOL_FLOOR_C`] the fit ran on.
    pub pulses: usize,
    /// The pulses' scatter around the fitted decay, C.
    pub residual_sd_c: f64,
    /// The fitted decay's rise over the NTC at the hold's end, C.
    pub r_end_c: f64,
    /// Every cool-down pulse that read the winding, in order.
    pub readings: Vec<CoolPulse>,
    pub dropped: Dropped,
    /// One pulse's i^2 R t over the hold's, the cool-down's mean.
    pub pulse_energy: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RiseFit {
    /// centi-C per count squared of heat.
    pub k: f64,
    pub rise_q016: Encoded,
    pub r_ref_ohm: f64,
    /// The first window at the hold's commanded duty, where R_ref is read,
    /// ms into the hold.
    pub r_ref_start_ms: f64,
    /// The windows the R_ref line ran through.
    pub r_ref_windows: usize,
    /// How far the line reads back from its windows' mean time, ms.
    pub r_ref_extrap_ms: f64,
    /// The copper's warming along the line, C/s.
    pub r_ref_slope_c_per_s: f64,
    /// The hold's cold current, the line's windows' mean |i|, counts.
    pub i_cold_counts: f64,
    /// The current ceiling over the line, counts.
    pub ceiling_counts: f64,
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

    /// How far the cold current sits under the ceiling, a fraction of it.
    pub fn margin(&self) -> f64 {
        1.0 - self.i_cold_counts / self.ceiling_counts
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

/// The sample standard deviation, 0 for fewer than two.
fn sd(xs: &[f64]) -> f64 {
    let n = xs.len();
    if n < 2 {
        return 0.0;
    }
    let m = xs.iter().sum::<f64>() / n as f64;
    (xs.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (n - 1) as f64).sqrt()
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

/// The hold's cold current against the current ceiling.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Cold {
    /// The first window at the commanded duty, ms on the host's clock.
    pub first_ms: f64,
    /// The mean |i| over the [`LINE_MS`] from it, counts.
    pub i_counts: f64,
    /// The lowest `i_lim_counts` polled from the hold's start through
    /// that span.
    pub ceiling: f64,
}

/// The hold's cold current from its windows at the commanded duty over the
/// [`LINE_MS`] from the first of them, once they cover it: refused when it
/// sits under the ceiling by less than [`HOLD_MARGIN`], or when nothing
/// read the winding at the command inside [`R_REF_WITHIN_MS`].
pub fn cold_current(
    polls: &[HoldPoll],
    windows: &[ThermWindow],
    t0_ms: f64,
) -> Option<Result<Cold, String>> {
    let from = windows.partition_point(|x| x.w.t_ms < t0_ms);
    let held = &windows[from..];
    let last = held.last()?.w.t_ms;
    let Some(first) = held.iter().find(|x| x.commanded && x.reads()) else {
        return (last - t0_ms > R_REF_WITHIN_MS).then(|| {
            Err(format!(
                "no window of the hold's first {:.0} s read the winding at its commanded duty: \
                 the limiter held it under the command",
                R_REF_WITHIN_MS / 1000.0
            ))
        });
    };
    let first_ms = first.w.t_ms;
    if last < first_ms + LINE_MS {
        return None;
    }
    let i_counts = mean(
        held.iter()
            .take_while(|x| x.w.t_ms < first_ms + LINE_MS)
            .filter(|x| x.commanded && x.reads())
            .map(|x| x.w.i.abs()),
    )?;
    let ceiling = polls
        .iter()
        .filter(|p| p.t_ms >= t0_ms && p.t_ms < first_ms + LINE_MS)
        .map(|p| p.i_lim_counts)
        .fold(f64::INFINITY, f64::min);
    if !ceiling.is_finite() || i_counts > (1.0 - HOLD_MARGIN) * ceiling {
        return Some(Err(format!(
            "the hold's cold current {i_counts:.0} counts sits {:.1} % under the {ceiling:.0} \
             counts current ceiling, inside the {:.0} % margin that keeps a re-read at the hold's \
             duty from being chopped",
            (1.0 - i_counts / ceiling) * 100.0,
            HOLD_MARGIN * 100.0
        )));
    }
    Some(Ok(Cold {
        first_ms,
        i_counts,
        ceiling,
    }))
}

/// One second of the hold.
#[derive(Copy, Clone, Default)]
struct Second {
    r: Acc,
    /// i^2 R, counts squared ohm.
    p: Acc,
    ntc: Acc,
    winding: Acc,
    unset: bool,
}

/// What every fit over a hold reads the copper against: the NTC base T_0,
/// the reference R and the hold's seconds.
struct Reference {
    t0_c: f64,
    r_ref: f64,
    start_ms: f64,
    windows: usize,
    extrap_ms: f64,
    slope_c_per_s: f64,
    cold: Cold,
    seconds: Vec<Second>,
}

impl Reference {
    /// The copper's rise over the NTC base for a window R, C.
    fn copper(&self, r: f64) -> f64 {
        (r / self.r_ref - 1.0) * (COPPER_ZERO_R_C + self.t0_c)
    }

    /// The hold's i^2 R t, counts squared ohm seconds.
    fn energy(&self) -> f64 {
        self.seconds.iter().filter_map(|s| s.p.mean()).sum::<f64>() * BIN_MS / 1000.0
    }
}

/// The hold from `t0_ms` to `end_ms` in one-second bins of the windows
/// that read the winding, T_0 the NTC's mean over the [`BASE_MS`] before
/// it. R_ref is a line through the windows at the commanded duty over the
/// [`LINE_MS`] from the first of them, read at that first window, which
/// lies inside [`R_REF_WITHIN_MS`]; their current sits [`HOLD_MARGIN`]
/// under the ceiling ([`cold_current`]). The current falls as the copper
/// warms at the held duty, and the line reads through that: a fold off the
/// duty and a shaft off its seat are each window's own flags.
fn reference(
    polls: &[HoldPoll],
    windows: &[ThermWindow],
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
    let hold: Vec<ThermWindow> = windows.iter().filter(|x| held(x.w.t_ms)).copied().collect();
    let cold = cold_current(polls, &hold, t0_ms).unwrap_or_else(|| {
        Err(format!(
            "the hold's windows at its commanded duty cover under the {:.0} s its R_ref line \
             reads",
            LINE_MS / 1000.0
        ))
    })?;
    let reads: Vec<(&ThermWindow, f64)> = hold
        .iter()
        .filter(|x| x.reads())
        .filter_map(|x| Some((x, r_ohm(&x.w)?)))
        .collect();
    let last = reads.iter().map(|x| x.0.w.t_ms).fold(t0_ms, f64::max);
    let n = ((last - t0_ms) / BIN_MS).floor() as usize + 1;
    let bin = |t: f64| ((t - t0_ms) / BIN_MS).floor() as usize;
    let mut seconds = vec![Second::default(); n];
    for (x, r) in &reads {
        if let Some(s) = seconds.get_mut(bin(x.w.t_ms)) {
            s.r.add(*r);
            s.p.add(x.w.i * x.w.i * r);
        }
    }
    for p in polls.iter().filter(|p| held(p.t_ms)) {
        if let Some(s) = seconds.get_mut(bin(p.t_ms)) {
            s.ntc.add(p.t_ntc_cc / 100.0);
            s.winding.add(p.t_winding_cc / 100.0);
            s.unset |= p.t_winding_cc <= i16::MIN as f64;
        }
    }
    let first = cold.first_ms;
    let line: Vec<(f64, f64)> = reads
        .iter()
        .filter(|x| x.0.commanded && x.0.w.t_ms >= first)
        .map(|(x, r)| (x.w.t_ms, *r))
        .take_while(|x| x.0 < first + LINE_MS)
        .collect();
    let m = line.len() as f64;
    let tm = line.iter().map(|x| x.0).sum::<f64>() / m;
    let rm = line.iter().map(|x| x.1).sum::<f64>() / m;
    let sxx: f64 = line.iter().map(|x| (x.0 - tm).powi(2)).sum();
    if line.len() < LINE_MIN || sxx <= 0.0 {
        return Err(format!(
            "the hold's first {:.0} s at its commanded duty carried {} windows, under the \
             {LINE_MIN} its R_ref line needs",
            LINE_MS / 1000.0,
            line.len()
        ));
    }
    let slope = line.iter().map(|x| (x.0 - tm) * (x.1 - rm)).sum::<f64>() / sxx;
    let r_ref = rm + slope * (first - tm);
    if !(r_ref.is_finite() && r_ref > 0.0) {
        return Err("the hold's R_ref line reads no R".into());
    }
    Ok(Reference {
        t0_c,
        r_ref,
        start_ms: first - t0_ms,
        windows: line.len(),
        extrap_ms: tm - first,
        slope_c_per_s: slope * 1000.0 / r_ref * (COPPER_ZERO_R_C + t0_c),
        cold,
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
    windows: &[ThermWindow],
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
        r_ref_start_ms: rf.start_ms,
        r_ref_windows: rf.windows,
        r_ref_extrap_ms: rf.extrap_ms,
        r_ref_slope_c_per_s: rf.slope_c_per_s,
        i_cold_counts: rf.cold.i_counts,
        ceiling_counts: rf.cold.ceiling,
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

/// One pulse read: its settled windows' mean time and R, their R scatter,
/// their mean i^2 R times the pulse's length, and the NTC over it.
struct PulseRead {
    t_ms: f64,
    windows: usize,
    r: f64,
    sd_r: f64,
    energy: f64,
    ntc_c: f64,
}

/// The windows of `p` that read the winding at its commanded duty, from
/// `settle_ms` after the first of them: the current's L/R transient is
/// over there. Every window the filter drops is counted in `dropped` by
/// the first reason it failed.
fn read_pulse(
    p: &Pulse,
    rec: &Recording,
    r_ohm: &impl Fn(&WindowSample) -> Option<f64>,
    dropped: &mut Dropped,
) -> Option<PulseRead> {
    let a = rec.windows.partition_point(|x| x.w.t_ms <= p.on_ms);
    let z = rec.windows.partition_point(|x| x.w.t_ms <= p.off_ms);
    let mut kept = Vec::new();
    for x in &rec.windows[a..z] {
        if !x.commanded {
            dropped.off_duty += 1;
        } else if !(x.floors && x.w.i != 0.0) {
            dropped.unread += 1;
        } else if !x.still {
            dropped.moved += 1;
        } else {
            kept.push(x);
        }
    }
    let settled = kept.first()?.w.t_ms + rec.settle_ms;
    let used: Vec<(f64, f64, f64)> = kept
        .iter()
        .filter(|x| x.w.t_ms >= settled)
        .filter_map(|x| Some((x.w.t_ms, r_ohm(&x.w)?, x.w.i)))
        .collect();
    let n = used.len() as f64;
    let rs: Vec<f64> = used.iter().map(|x| x.1).collect();
    let ntc_c = mean(
        rec.polls
            .iter()
            .filter(|q| q.t_ms >= p.on_ms && q.t_ms <= p.off_ms)
            .map(|q| q.t_ntc_cc / 100.0),
    )?;
    Some(PulseRead {
        t_ms: mean(used.iter().map(|x| x.0))?,
        windows: used.len(),
        r: rs.iter().sum::<f64>() / n,
        sd_r: sd(&rs),
        energy: used.iter().map(|x| x.2 * x.2 * x.1).sum::<f64>() / n * (p.off_ms - p.on_ms)
            / 1000.0,
        ntc_c,
    })
}

/// The copper's time constant from the cool-down that starts at `cool_ms`:
/// each re-read pulse, at the hold's own duty, reads the copper's rise over
/// the NTC `r` against the hold's R_ref ([`read_pulse`]), and `ln r`
/// against the time since `cool_ms` is a line of slope `-1 / tau` over the
/// pulses whose `r` reads over [`COOL_FLOOR_C`]. The pulses' own heat is
/// left out of the model: it holds the copper up by about the pulse duty
/// cycle of the hold's rise, which reads tau long by about that fraction.
fn cool_at(
    rec: &Recording,
    rf: &Reference,
    r_ohm: &impl Fn(&WindowSample) -> Option<f64>,
) -> Result<CoolFit, String> {
    let mut dropped = Dropped::default();
    let cool: Vec<PulseRead> = rec
        .pulses
        .iter()
        .filter_map(|p| read_pulse(p, rec, r_ohm, &mut dropped))
        .collect();
    dropped.pulses = rec.pulses.len() - cool.len();
    if cool.is_empty() {
        return Err(if rec.pulses.is_empty() {
            "the cool-down landed no measurement pulse".into()
        } else {
            format!(
                "none of the cool-down's {} pulses read the winding at the hold's duty: of their \
                 windows {} ran off it, {} read nothing and {} came off the seat",
                rec.pulses.len(),
                dropped.off_duty,
                dropped.unread,
                dropped.moved
            )
        });
    }
    let hold = rf.energy();
    let pulse_energy = if hold > 0.0 {
        cool.iter().map(|p| p.energy).sum::<f64>() / cool.len() as f64 / hold
    } else {
        f64::NAN
    };
    let as_c = |dr: f64| dr / rf.r_ref * (COPPER_ZERO_R_C + rf.t0_c);
    let readings: Vec<CoolPulse> = cool
        .iter()
        .map(|p| CoolPulse {
            t_s: (p.t_ms - rec.cool_ms) / 1000.0,
            windows: p.windows,
            rise_c: rf.copper(p.r) - (p.ntc_c - rf.t0_c),
            sd_c: as_c(p.sd_r),
        })
        .collect();
    let over: Vec<(f64, f64)> = readings
        .iter()
        .map(|p| (p.t_s, p.rise_c))
        .filter(|p| p.1 > COOL_FLOOR_C)
        .collect();
    if over.is_empty() {
        let top = readings
            .iter()
            .map(|p| p.rise_c)
            .fold(f64::NEG_INFINITY, f64::max);
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
            readings.len()
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
        readings,
        dropped,
        pulse_energy,
    })
}

/// The hold and its cool-down: the copper's time constant from the
/// cool-down ([`cool_at`]), rounded to the second as `thermal_tau_s`, then
/// the rise over the hold ([`fit_rise`]) on the heat replayed at the leak
/// the kernel runs for it.
pub fn fit_thermal(rec: &Recording, v_per_count: f64, a_per_count: f64) -> Result<RiseFit, String> {
    let r_ohm = r_ohm_at(v_per_count, a_per_count)?;
    let rf = reference(rec.polls, rec.windows, rec.t0_ms, rec.cool_ms, &r_ohm)?;
    let cool = cool_at(rec, &rf, &r_ohm)?;
    let tau_s = cool.tau_s.round().max(1.0).min(u16::MAX as f64) as u16;
    Ok(RiseFit {
        cool: Some(cool),
        ..rise_at(rec.polls, &rf, rec.t0_ms, tau_s, a_per_count)?
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

    /// A window that read the winding at its commanded duty.
    fn tw(w: WindowSample) -> ThermWindow {
        ThermWindow {
            w,
            pos: 0,
            commanded: true,
            floors: true,
            still: true,
        }
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
                    i_lim_counts: CEILING,
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
        let at = |t_ms: f64, x_w: f64| {
            let r = r_ref_vpc * (1.0 + x_w / (COPPER_ZERO_R_C + T_NTC0));
            tw(WindowSample {
                t_ms,
                i: I,
                vdiff: r * I * DUTY.signum() * 32767.0 / DUTY,
                duty_q15: DUTY,
            })
        };
        // the line's first second, then a window a bin
        let mut windows: Vec<ThermWindow> =
            (0..10).map(|k| at(T0 + k as f64 * 100.0, 0.0)).collect();
        windows.extend((0..bins).map(|j| at(centres[j], x_n[j] + K_C * h[j])));
        windows.sort_by(|a, b| a.w.t_ms.total_cmp(&b.w.t_ms));
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
    /// The current ceiling the hold runs under, counts: 270 sits 32 %
    /// under it.
    const CEILING: f64 = 400.0;
    const T_NTC0: f64 = 27.5;
    const POLL_MS: f64 = 20.0;
    /// A pulse window every this often.
    const PULSE_WIN_MS: f64 = 5.0;
    /// The electrical settle a pulse's windows wait out; the first window
    /// of each pulse reads inside it.
    const SETTLE_MS: f64 = 3.0;
    /// 40 C over the NTC at 270 counts, centi-C per count squared.
    const K: f64 = 4000.0 / (270.0 * 270.0);

    /// A window at `t_ms` at duty `duty` whose R, `r0` cold, reads the
    /// copper `x_w` C over the NTC base.
    fn window_at(t_ms: f64, i: f64, duty: f64, r0: f64, x_w: f64) -> WindowSample {
        let r = r0 * (1.0 + x_w / (COPPER_ZERO_R_C + T_NTC0));
        WindowSample {
            t_ms,
            i,
            vdiff: r * i * duty.signum() * 32767.0 / duty,
            duty_q15: duty,
        }
    }

    /// A hold window on a 1 vcount per ccount R_ref.
    fn window(t_ms: f64, i: f64, x_w: f64) -> WindowSample {
        window_at(t_ms, i, DUTY, 1.0, x_w)
    }

    /// A synthetic run: the hold from 0 to `hold_ms` under `ceiling`, whose
    /// copper rises over
    /// the NTC by exactly k x h on the heat replayed at `tau_s`, a window
    /// every 50 ms through its first 5 s and one at each later bin's
    /// centre. The applied duty is `duty(t)` of the command, the current
    /// `scale(t)` of [`I`]. The NTC sits at its base but for the hold's
    /// first four seconds, where it falls by the copper's excess so R reads
    /// R_ref there. Then `cool` = (length, every, pulse ms, tau s): pulses
    /// at the hold's duty whose copper decays from the hold's end on its own
    /// tau, a window every [`PULSE_WIN_MS`].
    struct Synth {
        hold_ms: f64,
        k: f64,
        tau_s: u16,
        duty: fn(f64) -> f64,
        scale: fn(f64) -> f64,
        ceiling: f64,
        cool: Option<(f64, f64, f64, f64)>,
    }

    impl Synth {
        fn hold(hold_ms: f64, tau_s: u16) -> Self {
            Self {
                hold_ms,
                k: K,
                tau_s,
                duty: |_| 1.0,
                scale: |_| 1.0,
                ceiling: CEILING,
                cool: None,
            }
        }

        fn pulses(&self) -> Vec<Pulse> {
            let mut out = Vec::new();
            if let Some((len, every, pulse, _)) = self.cool {
                let mut on = every;
                while on + pulse <= len {
                    out.push(Pulse {
                        on_ms: self.hold_ms + on,
                        off_ms: self.hold_ms + on + pulse,
                    });
                    on += every;
                }
            }
            out
        }

        fn in_pulse(&self, t_ms: f64) -> bool {
            self.pulses()
                .iter()
                .any(|p| t_ms > p.on_ms && t_ms <= p.off_ms)
        }

        fn current(&self, t_ms: f64) -> f64 {
            match t_ms {
                t if (0.0..self.hold_ms).contains(&t) => I * (self.scale)(t),
                t if self.in_pulse(t) => I,
                _ => 0.0,
            }
        }

        fn recording<'a>(
            &self,
            polls: &'a [HoldPoll],
            windows: &'a [ThermWindow],
            pulses: &'a [Pulse],
        ) -> Recording<'a> {
            Recording {
                polls,
                windows,
                pulses,
                t0_ms: 0.0,
                cool_ms: self.hold_ms,
                settle_ms: SETTLE_MS,
            }
        }

        fn build(&self) -> (Vec<HoldPoll>, Vec<ThermWindow>, Vec<Pulse>) {
            let k_c = self.k / 100.0;
            let end = self.hold_ms + self.cool.map_or(0.0, |c| c.0);
            let mut polls: Vec<HoldPoll> = (-400..=(end / POLL_MS) as i64)
                .map(|n| {
                    let t_ms = n as f64 * POLL_MS;
                    HoldPoll {
                        t_ms,
                        i_counts: self.current(t_ms),
                        t_ntc_cc: T_NTC0 * 100.0,
                        t_winding_cc: T_NTC0 * 100.0,
                        i_lim_counts: self.ceiling,
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
            let hold_at = |t: f64| {
                let j = (t / BIN_MS) as usize;
                let x_w = if t < 4000.0 { 0.0 } else { k_c * h[j] };
                let d = (self.duty)(t);
                ThermWindow {
                    commanded: d == 1.0,
                    ..tw(window_at(t, I * (self.scale)(t), DUTY * d, 1.0, x_w))
                }
            };
            let mut windows: Vec<ThermWindow> =
                (0..100).map(|k| hold_at(k as f64 * 50.0)).collect();
            windows.extend(
                centres
                    .iter()
                    .filter(|&&t| t >= 5000.0)
                    .map(|&t| hold_at(t)),
            );
            let r_end = k_c * replay_heat(&polls, self.tau_s, &[self.hold_ms])[0];
            let tau = self.cool.map_or(f64::INFINITY, |c| c.3);
            for p in self.pulses() {
                let mut t = p.on_ms + PULSE_WIN_MS;
                while t <= p.off_ms {
                    let x_w = r_end * (-(t - self.hold_ms) / 1000.0 / tau).exp();
                    // the first window reads inside the L/R transient
                    let r0 = if t == p.on_ms + PULSE_WIN_MS {
                        0.9
                    } else {
                        1.0
                    };
                    windows.push(tw(window_at(t, I, DUTY, r0, x_w)));
                    t += PULSE_WIN_MS;
                }
            }
            windows.sort_by(|a, b| a.w.t_ms.total_cmp(&b.w.t_ms));
            (polls, windows, self.pulses())
        }
    }

    fn scales() -> (f64, f64) {
        (units::volts_per_count(&DEV), units::amps_per_count(&DEV))
    }

    /// A hold and a cool-down whose copper decays on 115 s: the fit reads
    /// the tau back and the rise at the octave the kernel runs for it, its
    /// 500 ms re-reads every 5 s against the hold's own R_ref.
    #[test]
    fn a_cool_down_reads_the_copper_tau_and_the_rise_at_its_octave() {
        let (v, a) = scales();
        let s = Synth {
            cool: Some((480_000.0, 5000.0, 500.0, 115.0)),
            ..Synth::hold(480_000.0, 115)
        };
        let (polls, windows, pulses) = s.build();
        let f = fit_thermal(&s.recording(&polls, &windows, &pulses), v, a).unwrap();
        let cool = f.cool.clone().expect("a cool-down fit");
        assert!((cool.tau_s - 115.0).abs() < 0.02 * 115.0, "{f:?}");
        assert_eq!((f.thermal_tau_s, f.tau_shift), (115, 13), "{f:?}");
        assert!((f.octave_s() - 131.072).abs() < 1e-9);
        assert!((f.k - K).abs() < 0.02 * K, "{f:?}");
        assert!((f.plateau_k - K).abs() < 0.02 * K, "{f:?}");
        assert_eq!(f.bins, 480);
        // r_end is ~39 C: the pulses over 1 C run to ~421 s
        assert!((82..=86).contains(&cool.pulses), "{}", cool.pulses);
        assert!(cool.residual_sd_c < 0.01, "{cool:?}");
        let r_end = K / 100.0 * replay_heat(&polls, 115, &[s.hold_ms])[0];
        assert!((cool.r_end_c - r_end).abs() < 0.01 * r_end, "{cool:?}");
        assert_eq!(cool.readings.len(), 95);
        assert_eq!(cool.dropped, Dropped::default());
        assert!(
            cool.readings
                .iter()
                .all(|p| p.windows == 99 && p.sd_c < 0.1),
            "{:?}",
            cool.readings
        );
        // one 500 ms re-read at the hold's current over its 480 s, less the
        // hold's copper running warmer than the re-reads'
        let want = 0.5 / 480.0;
        assert!(
            (0.85..1.0).contains(&(cool.pulse_energy / want)),
            "{} of {want}",
            cool.pulse_energy
        );
    }

    /// Pulse windows the limiter chopped, that sat under the floors or that
    /// the shaft moved through, and the first window inside the electrical
    /// settle, all read a wrong R; the fit drops them, counting each by its
    /// reason, and reads what the clean pulse reads.
    #[test]
    fn a_chopped_pulse_window_is_dropped() {
        let (v, a) = scales();
        let s = Synth {
            cool: Some((240_000.0, 5000.0, 500.0, 115.0)),
            ..Synth::hold(240_000.0, 115)
        };
        let (polls, windows, pulses) = s.build();
        let clean = fit_thermal(&s.recording(&polls, &windows, &pulses), v, a).unwrap();
        let mut spoilt = windows.clone();
        let p = pulses[0];
        for (n, x) in spoilt
            .iter_mut()
            .filter(|x| x.w.t_ms > p.on_ms && x.w.t_ms <= p.off_ms)
            .enumerate()
        {
            match n % 6 {
                1 => x.commanded = false,
                3 => x.floors = false,
                5 => x.still = false,
                _ => continue,
            }
            x.w.vdiff *= 0.8;
            x.w.duty_q15 *= 0.95;
        }
        let f = fit_thermal(&s.recording(&polls, &spoilt, &pulses), v, a).unwrap();
        let (c, f) = (clean.cool.unwrap(), f.cool.unwrap());
        assert_eq!(c.readings.len(), f.readings.len());
        let (r0, r1) = (c.readings[0], f.readings[0]);
        assert!(r1.windows < r0.windows, "{r1:?}");
        let d = f.dropped;
        assert_eq!(
            (d.pulses, d.off_duty, d.unread, d.moved),
            (0, 17, 17, 16),
            "{d:?}"
        );
        assert!((r1.rise_c - r0.rise_c).abs() < 0.01, "{r0:?} {r1:?}");
        assert!(
            (f.tau_s - c.tau_s).abs() < 1e-3 * c.tau_s,
            "{} {}",
            f.tau_s,
            c.tau_s
        );
        // with the settle waived the transient's window reads into it
        let rec = Recording {
            settle_ms: 0.0,
            ..s.recording(&polls, &windows, &pulses)
        };
        let raw = fit_thermal(&rec, v, a).unwrap().cool.unwrap();
        assert_eq!(raw.readings[0].windows, r0.windows + 1);
        assert!(
            raw.readings[0].sd_c > 10.0 * r0.sd_c.max(1e-3),
            "{:?}",
            raw.readings[0]
        );
    }

    /// A bare copper node warming 5 C/s from the hold's start: R_ref, the
    /// line through the first second read at the first window, is the cold
    /// copper's R within 0.2 C, where a mean over the second reads it 2.5 C
    /// warm.
    #[test]
    fn the_reference_reads_the_copper_cold_under_a_warming_start() {
        const RATE_C_PER_S: f64 = 5.0;
        let (v, a) = scales();
        let polls: Vec<HoldPoll> = (-300..300)
            .map(|n| HoldPoll {
                t_ms: n as f64 * POLL_MS,
                i_counts: I,
                t_ntc_cc: T_NTC0 * 100.0,
                t_winding_cc: T_NTC0 * 100.0,
                i_lim_counts: CEILING,
            })
            .collect();
        let windows: Vec<ThermWindow> = (0..4000)
            .map(|k| {
                let t = 2.0 + k as f64;
                tw(window(t, I, RATE_C_PER_S * t / 1000.0))
            })
            .collect();
        let rf = reference(
            &polls,
            &windows,
            0.0,
            f64::INFINITY,
            r_ohm_at(v, a).unwrap(),
        )
        .unwrap();
        let cold = v / a;
        let off_c = (rf.r_ref / cold - 1.0) * (COPPER_ZERO_R_C + T_NTC0);
        assert!(
            off_c.abs() < 0.2,
            "R_ref reads {off_c:.3} C off the cold copper"
        );
        assert!((off_c - RATE_C_PER_S * 0.002).abs() < 1e-6, "{off_c}");
        assert!(
            (rf.slope_c_per_s - RATE_C_PER_S).abs() < 0.03,
            "{}",
            rf.slope_c_per_s
        );
        assert_eq!((rf.start_ms, rf.windows), (2.0, 1000));
        assert!((rf.extrap_ms - 499.5).abs() < 1e-9, "{}", rf.extrap_ms);
        let mean_c = windows[..1000]
            .iter()
            .map(|x| x.w.t_ms * RATE_C_PER_S / 1000.0)
            .sum::<f64>()
            / 1000.0;
        assert!(mean_c > 2.5, "{mean_c}");
    }

    /// The limiter's slew holds the first 200 ms under the command and the
    /// ceiling folds the duty 1.5 s in: both are duty changes, not
    /// unsteadiness. R_ref is read from the first window at the command,
    /// the rise from every window.
    #[test]
    fn a_fold_in_the_first_seconds_is_a_duty_change_not_unsteadiness() {
        let (v, a) = scales();
        let s = Synth {
            duty: |t| match t {
                t if t < 200.0 => 0.5,
                t if t < 1500.0 => 1.0,
                _ => 0.7,
            },
            scale: |t| match t {
                t if t < 200.0 => 0.5,
                t if t < 1500.0 => 1.0,
                _ => 0.7,
            },
            ..Synth::hold(120_000.0, 4)
        };
        let (polls, windows, _) = s.build();
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 4, v, a).unwrap();
        assert_eq!(f.r_ref_start_ms, 200.0, "{f:?}");
        assert_eq!(f.r_ref_windows, 20, "{f:?}");
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9 * v / a, "{f:?}");
        assert!((f.k - K).abs() < 0.02 * K, "{f:?}");
    }

    /// A hold the limiter governs from its first window never reads the
    /// winding at its command: no reference, no rise.
    #[test]
    fn a_hold_never_at_its_command_is_refused() {
        let (v, a) = scales();
        let s = Synth {
            duty: |_| 0.9,
            ..Synth::hold(120_000.0, 4)
        };
        let (polls, windows, _) = s.build();
        let why = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 4, v, a).unwrap_err();
        assert!(why.contains("at its commanded duty"), "{why}");
    }

    /// A hold whose cold current sits 7 % under the ceiling leaves a
    /// re-read at its duty no room before the limiter: refused, named.
    #[test]
    fn a_hold_inside_the_ceiling_margin_is_refused() {
        let (v, a) = scales();
        let s = Synth {
            ceiling: 290.0,
            ..Synth::hold(120_000.0, 4)
        };
        let (polls, windows, _) = s.build();
        let why = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 4, v, a).unwrap_err();
        assert!(
            why.contains("cold current 270 counts sits 6.9 % under the 290 counts current ceiling"),
            "{why}"
        );
        assert!(why.contains("inside the 10 % margin"), "{why}");
        let ok = Synth {
            ceiling: 301.0,
            ..Synth::hold(120_000.0, 4)
        };
        let (polls, windows, _) = ok.build();
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 4, v, a).unwrap();
        assert_eq!((f.i_cold_counts, f.ceiling_counts), (270.0, 301.0));
        assert!((f.margin() - (1.0 - 270.0 / 301.0)).abs() < 1e-12);
    }

    /// A fold a minute into the hold, as a long tau lands it, moves the
    /// duty well after the reference: it stays at the hold's start.
    #[test]
    fn a_late_fold_leaves_the_reference_at_the_start() {
        let (v, a) = scales();
        let s = Synth {
            duty: |t| if t < 60_000.0 { 1.0 } else { 0.6 },
            scale: |t| if t < 60_000.0 { 1.0 } else { 0.6 },
            ..Synth::hold(240_000.0, 115)
        };
        let (polls, windows, _) = s.build();
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
        let (polls, windows, _) = s.build();
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 115, v, a).unwrap();
        assert_eq!(f.r_ref_start_ms, 0.0, "{f:?}");
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9 * v / a, "{f:?}");
    }

    /// A seated hold whose current falls 6 % over its first 3 s at one
    /// duty, its copper warming 5 C/s and more: that is the motor being
    /// measured, not an unsettled current. Accepted, and R_ref reads the
    /// cold copper within 0.2 C.
    #[test]
    fn a_current_falling_with_the_warming_copper_is_accepted() {
        let (v, a) = scales();
        let polls: Vec<HoldPoll> = (-300..300)
            .map(|n| HoldPoll {
                t_ms: n as f64 * POLL_MS,
                i_counts: I,
                t_ntc_cc: T_NTC0 * 100.0,
                t_winding_cc: T_NTC0 * 100.0,
                i_lim_counts: CEILING,
            })
            .collect();
        let windows: Vec<ThermWindow> = (0..4000)
            .map(|k| {
                let t = 2.0 + k as f64;
                let fall = 1.0 - 0.02 * t / 1000.0;
                let x_w = (1.0 / fall - 1.0) * (COPPER_ZERO_R_C + T_NTC0);
                tw(window_at(t, I * fall, DUTY, 1.0, x_w))
            })
            .collect();
        let at_3s = windows[2998].w.i / windows[0].w.i;
        assert!((at_3s - 0.94).abs() < 1e-3, "{at_3s}");
        let rf = reference(
            &polls,
            &windows,
            0.0,
            f64::INFINITY,
            r_ohm_at(v, a).unwrap(),
        )
        .unwrap();
        let off_c = (rf.r_ref / (v / a) - 1.0) * (COPPER_ZERO_R_C + T_NTC0);
        assert!(
            off_c.abs() < 0.2,
            "R_ref reads {off_c:.3} C off the cold copper"
        );
        assert!(rf.slope_c_per_s > 5.0, "{}", rf.slope_c_per_s);
    }

    /// A copper that follows the replayed heat exactly settles where the
    /// regression says: the plateau's k is the regression's.
    #[test]
    fn the_plateau_reads_the_regression_on_a_copper_that_follows_the_heat() {
        let (v, a) = scales();
        let s = Synth::hold(300_000.0, 66);
        let (polls, windows, _) = s.build();
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, 66, v, a).unwrap();
        assert!((f.k - K).abs() < 1e-9 * K, "{f:?}");
        assert!((f.plateau_k - f.k).abs() < 1e-9 * K, "{f:?}");
        assert_eq!(f.plateau_rise_q016.raw, f.rise_q016.raw);
    }

    #[test]
    fn a_cool_down_short_of_pulses_or_rise_is_refused() {
        let (v, a) = scales();
        let fit = |s: &Synth| {
            let (polls, windows, pulses) = s.build();
            fit_thermal(&s.recording(&polls, &windows, &pulses), v, a).unwrap_err()
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
        let chopped = Synth {
            cool: Some((120_000.0, 10_000.0, 200.0, 115.0)),
            ..Synth::hold(120_000.0, 115)
        };
        let (polls, mut windows, pulses) = chopped.build();
        for x in windows.iter_mut().filter(|x| x.w.t_ms >= chopped.hold_ms) {
            x.commanded = false;
        }
        let why = fit_thermal(&chopped.recording(&polls, &windows, &pulses), v, a).unwrap_err();
        assert!(why.contains("none of the cool-down's 11 pulses"), "{why}");
        assert!(why.contains("440 ran off it"), "{why}");
    }

    #[test]
    fn a_short_or_unbased_hold_is_refused() {
        let v = units::volts_per_count(&DEV);
        let a = units::amps_per_count(&DEV);
        let w = |t_ms| {
            tw(WindowSample {
                t_ms,
                i: -270.0,
                vdiff: -800.0,
                duty_q15: -5000.0,
            })
        };
        let p = |t_ms| HoldPoll {
            t_ms,
            i_counts: -270.0,
            t_ntc_cc: 2750.0,
            t_winding_cc: 2750.0,
            i_lim_counts: CEILING,
        };
        let short: Vec<ThermWindow> = (0..50).map(|k| w(k as f64 * 100.0)).collect();
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
