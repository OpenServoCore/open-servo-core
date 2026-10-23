//! The winding estimate's constants (core `estimator::thermal`): the
//! kernel reads `t = t_ntc + rise x heat`, the heat the I2t EWMA of the held
//! current squared on `thermal_tau_s`. The board NTC's counts-to-centi-C
//! curve is a board fact, encoded from its beta constants ([`NtcCurve`]).
//! Two host routes to the rise, `th_rise_q016`: a nameplate thermal
//! resistance ([`rise_from_rth`]), or a hold's own copper against the heat
//! replayed at the servo's `thermal_tau_s` ([`fit_rise`]). R moves with the
//! operating point by a few percent across duties, so a copper reading
//! never takes a ratio across duties: every reading of the hold is against
//! the hold's own R_ref, read after its seat and its dip settle
//! ([`settle_dip`]).

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

/// The hold's current is judged steady per bin this long, ms: the four
/// campaign holds on board D settled at 50-75 ms.
pub const SETTLE_BIN_MS: f64 = 25.0;
/// The current's own sd floor is the median bin's from this far into the
/// hold, ms, past the seat and the dip.
pub const FLOOR_FROM_MS: f64 = 10_000.0;
/// ... to this far, ms, inside every hold ([`crate::exp::anchor::HOLD_MIN_S`]):
/// 0.53-0.55 counts per 25 ms on board D.
pub const FLOOR_TO_MS: f64 = 60_000.0;
/// A bin is steady when its current sd sits within this many of the floor.
pub const SETTLE_SD_FACTOR: f64 = 3.0;
/// A steady bin is seated when every window's pot sits this close to the
/// hold's seat, counts: the seat still walks while the current climbs.
pub const SEAT_WITHIN_COUNTS: u16 = 2;
/// R dips after the current settles (the brushes settling): the dip's
/// bottom is the minimum of R's running mean over this long, ms.
pub const DIP_MEAN_MS: f64 = 100.0;
/// The dip is looked for this long after the settle, ms: it ended
/// 130-475 ms in on the four campaign holds.
pub const DIP_MS: f64 = 1500.0;
/// R_ref is a line through the hold's windows at its commanded duty over
/// this long from the dip's bottom, read there: over the four campaign
/// holds it landed +0.01 +/- 1.1 C of the model reference.
pub const LINE_MS: f64 = 4000.0;
/// The fewest windows the line takes.
pub const LINE_MIN: usize = 3;
/// The hold settles inside this much of its start, where the copper still
/// sits at the NTC.
pub const R_REF_WITHIN_MS: f64 = 10_000.0;
/// The hold's cold current sits at least this fraction under the current
/// ceiling: the copper only lowers it from there, which keeps the I2t fold
/// off the hold.
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

#[derive(Clone, Debug, PartialEq)]
pub struct RiseFit {
    /// centi-C per count squared of heat.
    pub k: f64,
    pub rise_q016: Encoded,
    pub r_ref_ohm: f64,
    /// Where R_ref is read, the dip's bottom, ms into the hold.
    pub r_ref_start_ms: f64,
    /// Where the hold settled at its seat, ms into the hold.
    pub r_ref_settle_ms: f64,
    /// The copper's fall from the settle to the dip's bottom, C; positive
    /// fell.
    pub r_ref_dip_c: f64,
    /// The windows the R_ref line ran through.
    pub r_ref_windows: usize,
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

/// The step-down cool-down after the hold, as recorded.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CoolSummary {
    /// Its length, s.
    pub s: f64,
    pub windows: usize,
    /// Those that read the winding at the cool-down's duty.
    pub read: usize,
    /// Their mean |i|, counts; 0 when none read.
    pub i_counts: f64,
}

/// The cool-down from `cool_ms` to the run's last poll `end_ms`; None
/// when it never started.
pub fn cool_summary(windows: &[ThermWindow], cool_ms: f64, end_ms: f64) -> Option<CoolSummary> {
    if end_ms < cool_ms {
        return None;
    }
    let cool = &windows[windows.partition_point(|x| x.w.t_ms < cool_ms)..];
    let mut i = Acc::default();
    cool.iter()
        .filter(|x| at_command(x))
        .for_each(|x| i.add(x.w.i.abs()));
    Some(CoolSummary {
        s: (end_ms - cool_ms) / 1000.0,
        windows: cool.len(),
        read: i.n,
        i_counts: i.mean().unwrap_or(0.0),
    })
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

/// The window read the winding at the duty commanded for it.
fn at_command(x: &ThermWindow) -> bool {
    x.commanded && x.reads()
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
            st.heat(i, 0, [shift, 0]);
            g += SLOW_PERIOD_MS as f64;
        }
        out.push(st.heats()[0] as f64);
    }
    out
}

/// Where the hold settled at its seat and where R's dip after it bottoms
/// out ([`settle_dip`]).
#[derive(Copy, Clone, Debug, PartialEq)]
struct Dip {
    /// The settle bin's start, ms on the host's clock.
    settle_ms: f64,
    /// The settle bin's mean R.
    r_settle: f64,
    /// The running mean's minimum: its span's centre, ms on the host's
    /// clock, and its value.
    t_min: f64,
    r_min: f64,
}

/// The windows at the command among `read` from `lo` to `hi` ms into the
/// hold, grouped by their [`SETTLE_BIN_MS`] bin.
fn settle_bins<'a>(
    read: &[&'a ThermWindow],
    t0_ms: f64,
    lo: f64,
    hi: f64,
) -> Vec<(usize, Vec<&'a ThermWindow>)> {
    let bin = |x: &ThermWindow| ((x.w.t_ms - t0_ms) / SETTLE_BIN_MS).floor() as usize;
    let a = read.partition_point(|x| x.w.t_ms < t0_ms + lo);
    let z = read.partition_point(|x| x.w.t_ms < t0_ms + hi);
    read[a..z]
        .chunk_by(|x, y| bin(x) == bin(y))
        .map(|c| (bin(c[0]), c.to_vec()))
        .collect()
}

fn i_sd(bin: &[&ThermWindow]) -> f64 {
    sd(&bin.iter().map(|x| x.w.i.abs()).collect::<Vec<f64>>())
}

/// The median of `xs`, None when empty.
fn median(mut xs: Vec<f64>) -> Option<f64> {
    xs.sort_by(f64::total_cmp);
    let n = xs.len();
    (n > 0).then(|| (xs[(n - 1) / 2] + xs[n / 2]) / 2.0)
}

/// The settle and the dip of the hold's windows `held` from `t0_ms`, every
/// threshold from the hold itself: the floor is the median current sd of
/// the [`SETTLE_BIN_MS`] bins from [`FLOOR_FROM_MS`] to [`FLOOR_TO_MS`]; the
/// settle the first bin whose sd sits within [`SETTLE_SD_FACTOR`] of it
/// with every pot within [`SEAT_WITHIN_COUNTS`] of `seat`, inside
/// [`R_REF_WITHIN_MS`]; the dip's bottom the minimum of the [`DIP_MEAN_MS`]
/// running mean of R over the [`DIP_MS`] from the settle. Every window read
/// the winding at the command. A bin counts only with half the floor span's
/// median bin population, a span only with half that population's
/// [`DIP_MEAN_MS`] worth: a near-empty bin reads steady on a climbing
/// current, and a sparse span's minimum is biased low.
fn settle_dip(held: &[ThermWindow], t0_ms: f64, seat: u16) -> Result<Dip, String> {
    let read: Vec<&ThermWindow> = held.iter().filter(|x| at_command(x)).collect();
    let floor_bins = settle_bins(&read, t0_ms, FLOOR_FROM_MS, FLOOR_TO_MS);
    // the hold's own window rate, not a constant
    let population =
        median(floor_bins.iter().map(|(_, b)| b.len() as f64).collect()).ok_or_else(|| {
            format!(
                "no window read the winding at the hold's commanded duty from {:.0} to {:.0} s \
                 into it, where its current's sd floor is read",
                FLOOR_FROM_MS / 1000.0,
                FLOOR_TO_MS / 1000.0
            )
        })?;
    let full = |b: &[&ThermWindow]| b.len() as f64 >= population / 2.0;
    let floor = median(
        floor_bins
            .iter()
            .filter(|(_, b)| full(b))
            .map(|(_, b)| i_sd(b))
            .collect(),
    )
    .ok_or("the hold's current floor read no full bin")?;
    let span_min = population * DIP_MEAN_MS / SETTLE_BIN_MS / 2.0;
    let (k, bin) = settle_bins(&read, t0_ms, 0.0, R_REF_WITHIN_MS)
        .into_iter()
        .find(|(_, b)| {
            full(b)
                && i_sd(b) <= SETTLE_SD_FACTOR * floor
                && b.iter().all(|x| x.pos.abs_diff(seat) <= SEAT_WITHIN_COUNTS)
        })
        .ok_or_else(|| {
            format!(
                "the hold never settled at its seat inside {:.0} s: no {SETTLE_BIN_MS:.0} ms of \
                 its current sat within {SETTLE_SD_FACTOR:.0} x its {floor:.2} counts sd floor \
                 with every pot within {SEAT_WITHIN_COUNTS} of the seat {seat}",
                R_REF_WITHIN_MS / 1000.0
            )
        })?;
    let settle_ms = t0_ms + k as f64 * SETTLE_BIN_MS;
    let r_settle = mean(bin.iter().filter_map(|x| window_r_vpc(&x.w)))
        .ok_or("the hold's settle bin reads no R")?;
    let seg: Vec<(f64, f64)> = read
        .iter()
        .filter(|x| x.w.t_ms >= settle_ms && x.w.t_ms < settle_ms + DIP_MS)
        .filter_map(|x| Some((x.w.t_ms, window_r_vpc(&x.w)?)))
        .collect();
    let mut cs = vec![0.0];
    for (_, r) in &seg {
        cs.push(cs[cs.len() - 1] + r);
    }
    let mut best: Option<(f64, f64)> = None;
    let mut j = 0;
    for (i, (t, _)) in seg.iter().enumerate() {
        if t + DIP_MEAN_MS > settle_ms + DIP_MS {
            break;
        }
        while j < seg.len() && seg[j].0 < t + DIP_MEAN_MS {
            j += 1;
        }
        if ((j - i) as f64) < span_min {
            continue;
        }
        let m = (cs[j] - cs[i]) / (j - i) as f64;
        if best.is_none_or(|(_, b)| m < b) {
            best = Some((t + DIP_MEAN_MS / 2.0, m));
        }
    }
    let (t_min, r_min) = best.ok_or_else(|| {
        format!(
            "no {DIP_MEAN_MS:.0} ms of the {:.1} s after the hold settled carried {span_min:.0} \
             windows at its commanded duty, half its own window rate",
            DIP_MS / 1000.0
        )
    })?;
    Ok(Dip {
        settle_ms,
        r_settle,
        t_min,
        r_min,
    })
}

/// The hold's cold current against the current ceiling.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Cold {
    /// The dip's bottom, where R_ref is read, ms on the host's clock.
    pub first_ms: f64,
    /// The mean |i| over the [`LINE_MS`] from it, counts.
    pub i_counts: f64,
    /// The lowest `i_lim_counts` polled from the hold's start through
    /// that span.
    pub ceiling: f64,
}

fn never_at_command() -> String {
    format!(
        "no window read the winding seated at its commanded duty inside {:.0} s: the limiter \
         held it under the command, or the shaft never settled at its seat",
        R_REF_WITHIN_MS / 1000.0
    )
}

/// The cold current over the [`LINE_MS`] from the dip's bottom `t_min` of
/// the hold's windows `held`, refused when it sits under the ceiling by
/// less than [`HOLD_MARGIN`].
fn cold_at(
    polls: &[HoldPoll],
    held: &[ThermWindow],
    t0_ms: f64,
    t_min: f64,
) -> Result<Cold, String> {
    let i_counts = mean(
        held.iter()
            .filter(|x| x.w.t_ms >= t_min && x.w.t_ms < t_min + LINE_MS)
            .filter(|x| at_command(x))
            .map(|x| x.w.i.abs()),
    )
    .ok_or("no window read the winding at the hold's commanded duty along its R_ref line")?;
    let ceiling = polls
        .iter()
        .filter(|p| p.t_ms >= t0_ms && p.t_ms < t_min + LINE_MS)
        .map(|p| p.i_lim_counts)
        .fold(f64::INFINITY, f64::min);
    if !ceiling.is_finite() || i_counts > (1.0 - HOLD_MARGIN) * ceiling {
        return Err(format!(
            "the hold's cold current {i_counts:.0} counts sits {:.1} % under the {ceiling:.0} \
             counts current ceiling, inside the {:.0} % margin that keeps the I2t fold off the \
             hold",
            (1.0 - i_counts / ceiling) * 100.0,
            HOLD_MARGIN * 100.0
        ));
    }
    Ok(Cold {
        first_ms: t_min,
        i_counts,
        ceiling,
    })
}

/// The hold's cold current from its windows at the commanded duty over the
/// [`LINE_MS`] from the dip's bottom ([`settle_dip`] against the hold's
/// `seat`), once the windows cover the current's sd floor and that line:
/// refused when it sits under the ceiling by less than [`HOLD_MARGIN`], when
/// the hold never settled, or when nothing read the winding at the command
/// inside [`R_REF_WITHIN_MS`]. None until then, and until the seat is read.
pub fn cold_current(
    polls: &[HoldPoll],
    windows: &[ThermWindow],
    t0_ms: f64,
    seat: Option<u16>,
) -> Option<Result<Cold, String>> {
    let from = windows.partition_point(|x| x.w.t_ms < t0_ms);
    let held = &windows[from..];
    let last = held.last()?.w.t_ms;
    if !held.iter().any(at_command) {
        return (last - t0_ms > R_REF_WITHIN_MS).then(|| Err(never_at_command()));
    }
    let seat = seat?;
    if last < t0_ms + FLOOR_TO_MS {
        return None;
    }
    let dip = match settle_dip(held, t0_ms, seat) {
        Ok(d) => d,
        Err(why) => return Some(Err(why)),
    };
    if last < dip.t_min + LINE_MS {
        return None;
    }
    Some(cold_at(polls, held, t0_ms, dip.t_min))
}

/// One second of the hold.
#[derive(Copy, Clone, Default)]
struct Second {
    r: Acc,
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
    settle_ms: f64,
    dip_c: f64,
    windows: usize,
    slope_c_per_s: f64,
    cold: Cold,
    seconds: Vec<Second>,
}

impl Reference {
    /// The copper's rise over the NTC base for a window R, C.
    fn copper(&self, r: f64) -> f64 {
        (r / self.r_ref - 1.0) * (COPPER_ZERO_R_C + self.t0_c)
    }
}

/// The hold from `t0_ms` to `end_ms` in one-second bins of the windows
/// that read the winding, T_0 the NTC's mean over the [`BASE_MS`] before
/// it. R_ref is a line through the windows at the commanded duty over the
/// [`LINE_MS`] from the dip's bottom after the hold settled at `seat`
/// ([`settle_dip`]), read there; their current sits [`HOLD_MARGIN`] under
/// the ceiling. The current falls as the copper warms at the held duty, and
/// the line reads through that: a fold off the duty and a shaft off its
/// seat are each window's own flags.
fn reference(
    polls: &[HoldPoll],
    windows: &[ThermWindow],
    t0_ms: f64,
    end_ms: f64,
    seat: u16,
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
    if !hold
        .iter()
        .any(|x| at_command(x) && x.w.t_ms < t0_ms + R_REF_WITHIN_MS)
    {
        return Err(never_at_command());
    }
    let dip = settle_dip(&hold, t0_ms, seat)?;
    let cold = cold_at(polls, &hold, t0_ms, dip.t_min)?;
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
        }
    }
    for p in polls.iter().filter(|p| held(p.t_ms)) {
        if let Some(s) = seconds.get_mut(bin(p.t_ms)) {
            s.ntc.add(p.t_ntc_cc / 100.0);
            s.winding.add(p.t_winding_cc / 100.0);
            s.unset |= p.t_winding_cc <= i16::MIN as f64;
        }
    }
    let first = dip.t_min;
    let line: Vec<(f64, f64)> = reads
        .iter()
        .filter(|x| x.0.commanded && x.0.w.t_ms >= first && x.0.w.t_ms < first + LINE_MS)
        .map(|(x, r)| (x.w.t_ms, *r))
        .collect();
    let m = line.len() as f64;
    let tm = line.iter().map(|x| x.0).sum::<f64>() / m;
    let rm = line.iter().map(|x| x.1).sum::<f64>() / m;
    let sxx: f64 = line.iter().map(|x| (x.0 - tm).powi(2)).sum();
    if line.len() < LINE_MIN || sxx <= 0.0 {
        return Err(format!(
            "the {:.0} s of the hold from its dip at its commanded duty carried {} windows, \
             under the {LINE_MIN} its R_ref line needs",
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
        settle_ms: dip.settle_ms - t0_ms,
        dip_c: (dip.r_settle / dip.r_min - 1.0) * (COPPER_ZERO_R_C + t0_c),
        windows: line.len(),
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
/// ([`reference`] for R_ref and T_0, the settle judged against the hold's
/// `seat`). `v_per_count` / `a_per_count` the board's terminal and current
/// scales.
#[allow(clippy::too_many_arguments)]
pub fn fit_rise(
    polls: &[HoldPoll],
    windows: &[ThermWindow],
    t0_ms: f64,
    end_ms: f64,
    seat: u16,
    thermal_tau_s: u16,
    v_per_count: f64,
    a_per_count: f64,
) -> Result<RiseFit, String> {
    let r_ohm = r_ohm_at(v_per_count, a_per_count)?;
    let rf = reference(polls, windows, t0_ms, end_ms, seat, r_ohm)?;
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
        r_ref_settle_ms: rf.settle_ms,
        r_ref_dip_c: rf.dip_c,
        r_ref_windows: rf.windows,
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

    /// The hold's seat, counts.
    const SEAT: u16 = 341;
    const I: f64 = -270.0;
    const DUTY: f64 = -5000.0;
    /// The current ceiling the hold runs under, counts: 270 sits 32 %
    /// under it.
    const CEILING: f64 = 400.0;
    const T_NTC0: f64 = 27.5;
    const POLL_MS: f64 = 20.0;
    /// A window every this often through the hold's first [`DENSE_MS`].
    const WIN_MS: f64 = 2.0;
    /// The span the windows come dense through: past the current's floor.
    const DENSE_MS: f64 = FLOOR_TO_MS + 1000.0;
    /// 40 C over the NTC at 270 counts, centi-C per count squared.
    const K: f64 = 4000.0 / (270.0 * 270.0);

    /// A window that read the winding at its commanded duty, at the seat.
    fn tw(w: WindowSample) -> ThermWindow {
        ThermWindow {
            w,
            pos: SEAT,
            commanded: true,
            floors: true,
            still: true,
        }
    }

    /// Half a count either way, window to window: the current's own
    /// scatter, which sets its floor.
    fn wobble(k: usize) -> f64 {
        if k % 2 == 0 { 0.5 } else { -0.5 }
    }

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

    /// Polls every [`POLL_MS`] from 8 s before the hold to `end_ms`, the
    /// held current from the hold's start, the NTC at its base.
    fn flat_polls(end_ms: f64, ceiling: f64) -> Vec<HoldPoll> {
        (-400..=(end_ms / POLL_MS) as i64)
            .map(|n| {
                let t_ms = n as f64 * POLL_MS;
                HoldPoll {
                    t_ms,
                    i_counts: if t_ms >= 0.0 { I } else { 0.0 },
                    t_ntc_cc: T_NTC0 * 100.0,
                    t_winding_cc: T_NTC0 * 100.0,
                    i_lim_counts: ceiling,
                }
            })
            .collect()
    }

    fn scales() -> (f64, f64) {
        (units::volts_per_count(&DEV), units::amps_per_count(&DEV))
    }

    /// A hold whose copper rises over the NTC by exactly k x h at each bin
    /// centre, the NTC climbing on its own: the fit reads k back.
    #[test]
    fn recovers_the_rise_of_a_synthetic_hold() {
        const K_C: f64 = 0.0105;
        const TAU_S: u16 = 4;
        let (v, a) = scales();
        let polls: Vec<HoldPoll> = (-500..30_000)
            .map(|n| {
                let t_ms = n as f64 * SLOW_PERIOD_MS as f64;
                HoldPoll {
                    t_ms,
                    i_counts: if t_ms >= 0.0 { I } else { 0.0 },
                    t_ntc_cc: 0.0,
                    t_winding_cc: 0.0,
                    i_lim_counts: CEILING,
                }
            })
            .collect();
        let bins = 470;
        let centres: Vec<f64> = (0..bins).map(|j| (j as f64 + 0.5) * BIN_MS).collect();
        let h = replay_heat(&polls, TAU_S, &centres);
        // the NTC climbs on its own; for the first six bins it falls
        // instead by the copper's excess, so R reads R_ref where R_ref is
        // read
        let x_n: Vec<f64> = (0..bins)
            .map(|j| if j < 6 { -K_C * h[j] } else { 0.004 * j as f64 })
            .collect();
        let polls: Vec<HoldPoll> = polls
            .into_iter()
            .map(|p| {
                let j = (p.t_ms / BIN_MS).floor();
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
        let x_w = |t: f64| {
            let j = (t / BIN_MS) as usize;
            x_n[j] + K_C * h[j]
        };
        // dense through the floor's span, then a window a bin
        let mut windows: Vec<ThermWindow> = (0..(DENSE_MS / WIN_MS) as usize)
            .map(|k| {
                let t = k as f64 * WIN_MS;
                tw(window_at(t, I + wobble(k), DUTY, 1.0, x_w(t)))
            })
            .collect();
        windows.extend(
            centres
                .iter()
                .filter(|&&t| t >= DENSE_MS)
                .map(|&t| tw(window_at(t, I, DUTY, 1.0, x_w(t)))),
        );
        let f = fit_rise(&polls, &windows, 0.0, f64::INFINITY, SEAT, TAU_S, v, a).unwrap();
        assert!((f.k - K_C * 100.0).abs() < 1e-9, "{f:?}");
        assert_eq!(f.rise_q016.raw, (K_C * 100.0 * 65536.0).round() as u16);
        assert!(f.residual_sd_c < 1e-9, "{f:?}");
        assert!((f.t0_c - T_NTC0).abs() < 1e-12);
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9);
        assert!((f.rth_c_per_w - rth_of(f.k, f.r_ref_ohm, a)).abs() < 1e-12);
        assert_eq!(f.bins, bins);
        let steady = K_C * h[bins - 1];
        assert!((f.settled_rise_c - steady).abs() < 0.01 * steady, "{f:?}");
        assert!((f.estimate_err_c.unwrap() - 0.5).abs() < 1e-9, "{f:?}");
        assert_eq!(f.r_ref_settle_ms, 0.0, "{f:?}");
        assert!(f.r_ref_start_ms < DIP_MS, "{f:?}");
        assert_eq!((f.thermal_tau_s, f.tau_shift), (TAU_S, 8));
    }

    /// A synthetic hold from 0 to `hold_ms` under `ceiling`, seated from
    /// its first window, whose copper rises over the NTC by exactly k x h on
    /// the heat replayed at `tau_s`: a window every [`WIN_MS`] through the
    /// current's floor and one at each later bin's centre, the current
    /// wobbling half a count. The applied duty is `duty(t)` of the command,
    /// the current `scale(t)` of [`I`]. The NTC sits at its base but for the
    /// hold's first six seconds, where it falls by the copper's excess so R
    /// reads R_ref along its line.
    struct Synth {
        hold_ms: f64,
        k: f64,
        tau_s: u16,
        duty: fn(f64) -> f64,
        scale: fn(f64) -> f64,
        ceiling: f64,
    }

    /// The span at the hold's start where R reads R_ref.
    const FLAT_MS: f64 = 6000.0;

    impl Synth {
        fn hold(hold_ms: f64, tau_s: u16) -> Self {
            Self {
                hold_ms,
                k: K,
                tau_s,
                duty: |_| 1.0,
                scale: |_| 1.0,
                ceiling: CEILING,
            }
        }

        fn build(&self) -> (Vec<HoldPoll>, Vec<ThermWindow>) {
            let k_c = self.k / 100.0;
            let mut polls = flat_polls(self.hold_ms, self.ceiling);
            for p in polls.iter_mut() {
                if (0.0..self.hold_ms).contains(&p.t_ms) {
                    p.i_counts = I * (self.scale)(p.t_ms);
                } else {
                    p.i_counts = 0.0;
                }
            }
            let bins = (self.hold_ms / BIN_MS) as usize;
            let centres: Vec<f64> = (0..bins).map(|j| (j as f64 + 0.5) * BIN_MS).collect();
            let h = replay_heat(&polls, self.tau_s, &centres);
            for p in polls.iter_mut() {
                if (0.0..FLAT_MS).contains(&p.t_ms) {
                    let j = (p.t_ms / BIN_MS) as usize;
                    p.t_ntc_cc -= k_c * h[j] * 100.0;
                }
            }
            let hold_at = |t: f64, wob: f64| {
                let j = (t / BIN_MS) as usize;
                let x_w = if t < FLAT_MS { 0.0 } else { k_c * h[j] };
                let d = (self.duty)(t);
                ThermWindow {
                    commanded: d == 1.0,
                    ..tw(window_at(t, I * (self.scale)(t) + wob, DUTY * d, 1.0, x_w))
                }
            };
            let dense = DENSE_MS.min(self.hold_ms);
            let mut windows: Vec<ThermWindow> = (0..(dense / WIN_MS) as usize)
                .map(|k| hold_at(k as f64 * WIN_MS, wobble(k)))
                .collect();
            windows.extend(
                centres
                    .iter()
                    .filter(|&&t| t >= dense)
                    .map(|&t| hold_at(t, 0.0)),
            );
            (polls, windows)
        }

        fn fit(&self) -> Result<RiseFit, String> {
            let (v, a) = scales();
            let (polls, windows) = self.build();
            fit_rise(&polls, &windows, 0.0, f64::INFINITY, SEAT, self.tau_s, v, a)
        }
    }

    /// The bench's hold, as `seated` builds it.
    struct Seated {
        /// The copper's warming from the dip's end, C/s.
        rate: f64,
        /// How long the pot sits 3 counts off the seat after the seat's
        /// walk, still by the stillness tolerance but not seated, ms.
        off_seat_ms: f64,
        /// How long the current keeps a 5 count swing, ms.
        rough_ms: f64,
    }

    /// The seat's 50 ms as the bench reads them: the current climbing 320
    /// to 450 counts, R_app 40 to 20 % high, the pot walking from 350 to
    /// the seat (or 3 off it until `off_seat_ms`); then R_app falling 0.5 % over 0.3 s as the
    /// brushes settle; then the copper warming `rate` C/s with the current
    /// falling with it. A window every [`WIN_MS`] through the current's
    /// floor, the current wobbling half a count.
    fn seated(s: &Seated) -> (Vec<HoldPoll>, Vec<ThermWindow>) {
        const SEAT_MS: f64 = 50.0;
        const DIP_END_MS: f64 = 350.0;
        let polls = flat_polls(DENSE_MS, 600.0);
        let windows = (0..(DENSE_MS / WIN_MS) as usize)
            .map(|k| {
                let t = k as f64 * WIN_MS;
                let r_app = match t {
                    t if t < SEAT_MS => 1.4 - 0.2 * t / SEAT_MS,
                    t if t < DIP_END_MS => 1.0 + 0.005 * (DIP_END_MS - t) / (DIP_END_MS - SEAT_MS),
                    t => 1.0 + s.rate * (t - DIP_END_MS) / 1000.0 / (COPPER_ZERO_R_C + T_NTC0),
                };
                let mut i = if t < SEAT_MS {
                    320.0 + 130.0 * t / SEAT_MS
                } else {
                    450.0 / r_app.max(1.0)
                };
                i += wobble(k);
                if t < s.rough_ms {
                    i += 10.0 * wobble(k);
                }
                let land = if s.off_seat_ms > 0.0 { SEAT + 3 } else { SEAT };
                let pos = match t {
                    t if t < SEAT_MS => {
                        (350.0 - (350.0 - land as f64) * t / SEAT_MS).round() as u16
                    }
                    t if t < s.off_seat_ms => land,
                    _ => SEAT,
                };
                ThermWindow {
                    pos,
                    still: pos.abs_diff(SEAT) <= 3,
                    ..tw(window_at(t, -i, DUTY, r_app, 0.0))
                }
            })
            .collect();
        (polls, windows)
    }

    fn bench(rate: f64) -> Seated {
        Seated {
            rate,
            off_seat_ms: 0.0,
            rough_ms: 0.0,
        }
    }

    fn reference_of(polls: &[HoldPoll], windows: &[ThermWindow]) -> Result<Reference, String> {
        let (v, a) = scales();
        reference(
            polls,
            windows,
            0.0,
            f64::INFINITY,
            SEAT,
            r_ohm_at(v, a).unwrap(),
        )
    }

    /// The copper's offset from the synthetic cold R, C.
    fn off_c(rf: &Reference) -> f64 {
        let (v, a) = scales();
        (rf.r_ref / (v / a) - 1.0) * (COPPER_ZERO_R_C + T_NTC0)
    }

    /// The seat's climbing current and high R_app and the brushes' dip
    /// after it read the copper warm: R_ref waits for the current to steady
    /// at the seat, then for the dip's bottom, and reads the cold copper
    /// there within 0.3 C. The cold current reads the same span, once the
    /// windows cover the current's floor.
    #[test]
    fn the_reference_reads_at_the_dip_after_the_seat_settles() {
        let (polls, windows) = seated(&bench(3.0));
        let rf = reference_of(&polls, &windows).unwrap();
        assert!(
            off_c(&rf).abs() < 0.3,
            "R_ref reads {:.3} C off",
            off_c(&rf)
        );
        assert!((50.0..=75.0).contains(&rf.settle_ms), "{}", rf.settle_ms);
        assert!((1.1..1.4).contains(&rf.dip_c), "{}", rf.dip_c);
        assert!((340.0..380.0).contains(&rf.start_ms), "{}", rf.start_ms);
        assert!((rf.slope_c_per_s - 3.0).abs() < 0.1, "{}", rf.slope_c_per_s);
        assert!(rf.windows > 1900, "{}", rf.windows);
        let early = windows.partition_point(|x| x.w.t_ms < FLOOR_TO_MS - 1000.0);
        assert_eq!(
            cold_current(&polls, &windows[..early], 0.0, Some(SEAT)),
            None
        );
        assert_eq!(cold_current(&polls, &windows, 0.0, None), None);
        let cold = cold_current(&polls, &windows, 0.0, Some(SEAT))
            .unwrap()
            .unwrap();
        assert_eq!(cold, rf.cold);
        assert_eq!(cold.first_ms, rf.start_ms);
    }

    /// The slew's tail lands two windows at the command, seated and
    /// steady, ahead of the current's climb: two windows are no bin, and
    /// the settle waits for the current to steady at the seat.
    #[test]
    fn a_near_empty_first_bin_at_the_command_does_not_settle() {
        let (polls, mut windows) = seated(&bench(3.0));
        let mut early = 0;
        for x in windows.iter_mut().filter(|x| x.w.t_ms < SETTLE_BIN_MS) {
            if early < 2 {
                x.w = window_at(x.w.t_ms, -330.0, DUTY, 1.3, 0.0);
                x.pos = SEAT;
                x.still = true;
                early += 1;
            } else {
                x.commanded = false;
            }
        }
        let rf = reference_of(&polls, &windows).unwrap();
        assert!((50.0..=75.0).contains(&rf.settle_ms), "{}", rf.settle_ms);
        assert!(
            off_c(&rf).abs() < 0.3,
            "R_ref reads {:.3} C off",
            off_c(&rf)
        );
    }

    /// A pot still 3 counts off its seat through the first 10 s, inside
    /// the stillness tolerance but never seated while R_ref may still read
    /// the copper cold: refused, named.
    #[test]
    fn a_hold_that_never_settles_is_refused() {
        let (polls, windows) = seated(&Seated {
            off_seat_ms: 11_000.0,
            ..bench(3.0)
        });
        let why = reference_of(&polls, &windows).err().unwrap();
        assert!(
            why.contains("never settled at its seat inside 10 s"),
            "{why}"
        );
        let live = cold_current(&polls, &windows, 0.0, Some(SEAT));
        assert_eq!(live, Some(Err(why)));
    }

    /// A current that swings through the first 10 s never steadies to its
    /// own floor: refused, named.
    #[test]
    fn a_hold_whose_current_never_steadies_is_refused() {
        let (polls, windows) = seated(&Seated {
            rough_ms: 11_000.0,
            ..bench(3.0)
        });
        let why = reference_of(&polls, &windows).err().unwrap();
        assert!(
            why.contains("never settled at its seat inside 10 s"),
            "{why}"
        );
        assert!(why.contains("sd floor"), "{why}");
    }

    /// A bare copper node warming 5 C/s from the hold's start, seated and
    /// steady from its first window: R never dips, so the line's start is
    /// its first 100 ms, and R_ref, the line through the 4 s from there
    /// read at it, is the copper's R there, where a mean over the line
    /// reads it 10 C warm.
    #[test]
    fn the_reference_reads_the_copper_cold_under_a_warming_start() {
        const RATE_C_PER_S: f64 = 5.0;
        let polls = flat_polls(DENSE_MS, CEILING);
        let windows: Vec<ThermWindow> = (0..(DENSE_MS / WIN_MS) as usize)
            .map(|k| {
                let t = 2.0 + k as f64 * WIN_MS;
                tw(window_at(
                    t,
                    I + wobble(k),
                    DUTY,
                    1.0,
                    RATE_C_PER_S * t / 1000.0,
                ))
            })
            .collect();
        let rf = reference_of(&polls, &windows).unwrap();
        assert_eq!((rf.settle_ms, rf.start_ms), (0.0, 52.0));
        assert!(
            (off_c(&rf) - RATE_C_PER_S * 0.052).abs() < 1e-6,
            "{}",
            off_c(&rf)
        );
        assert!(
            (rf.slope_c_per_s - RATE_C_PER_S).abs() < 0.03,
            "{}",
            rf.slope_c_per_s
        );
        assert!(rf.dip_c < 0.0, "R rose: {}", rf.dip_c);
        assert_eq!(rf.windows, 2000);
        let mean_c = windows[25..2025]
            .iter()
            .map(|x| x.w.t_ms * RATE_C_PER_S / 1000.0)
            .sum::<f64>()
            / 2000.0;
        assert!(mean_c > 10.0, "{mean_c}");
    }

    /// The limiter's slew holds the first 200 ms under the command and the
    /// ceiling folds the duty from 1.5 to 2.5 s: both are duty changes, not
    /// unsteadiness. The settle and R_ref read the windows at the command
    /// alone, the rise every window.
    #[test]
    fn a_fold_in_the_first_seconds_is_a_duty_change_not_unsteadiness() {
        let step = |t: f64| match t {
            t if t < 200.0 => 0.5,
            t if (1500.0..2500.0).contains(&t) => 0.7,
            _ => 1.0,
        };
        let s = Synth {
            duty: step,
            scale: step,
            ..Synth::hold(120_000.0, 4)
        };
        let f = s.fit().unwrap();
        let (v, a) = scales();
        assert_eq!(f.r_ref_settle_ms, 200.0, "{f:?}");
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9 * v / a, "{f:?}");
        assert!((1000..=1600).contains(&f.r_ref_windows), "{f:?}");
        assert!((f.k - K).abs() < 0.02 * K, "{f:?}");
    }

    /// A hold the limiter governs from its first window never reads the
    /// winding at its command: no reference, no rise.
    #[test]
    fn a_hold_never_at_its_command_is_refused() {
        let s = Synth {
            duty: |_| 0.9,
            ..Synth::hold(120_000.0, 4)
        };
        let why = s.fit().unwrap_err();
        assert_eq!(
            why,
            "no window read the winding seated at its commanded duty inside 10 s: the limiter \
             held it under the command, or the shaft never settled at its seat"
        );
        let (polls, windows) = s.build();
        assert_eq!(cold_current(&polls, &windows, 0.0, None), Some(Err(why)));
    }

    /// A hold whose cold current sits 7 % under the ceiling leaves the I2t
    /// fold no room: refused, named.
    #[test]
    fn a_hold_inside_the_ceiling_margin_is_refused() {
        let s = Synth {
            ceiling: 290.0,
            ..Synth::hold(120_000.0, 4)
        };
        let why = s.fit().unwrap_err();
        assert!(
            why.contains("cold current 270 counts sits 6.9 % under the 290 counts current ceiling"),
            "{why}"
        );
        assert!(why.contains("inside the 10 % margin"), "{why}");
        let ok = Synth {
            ceiling: 301.0,
            ..Synth::hold(120_000.0, 4)
        };
        let f = ok.fit().unwrap();
        assert!((f.i_cold_counts - 270.0).abs() < 1e-3, "{f:?}");
        assert_eq!(f.ceiling_counts, 301.0);
        assert!((f.margin() - (1.0 - f.i_cold_counts / 301.0)).abs() < 1e-12);
    }

    /// A fold a minute into the hold, as a long tau lands it, moves the
    /// duty well after the reference: it stays at the hold's start.
    #[test]
    fn a_late_fold_leaves_the_reference_at_the_start() {
        let s = Synth {
            duty: |t| if t < 60_000.0 { 1.0 } else { 0.6 },
            scale: |t| if t < 60_000.0 { 1.0 } else { 0.6 },
            ..Synth::hold(240_000.0, 115)
        };
        let f = s.fit().unwrap();
        let (v, a) = scales();
        assert_eq!(f.r_ref_settle_ms, 0.0, "{f:?}");
        assert!(f.r_ref_start_ms < DIP_MS, "{f:?}");
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9 * v / a, "{f:?}");
        assert!((f.k - K).abs() < 0.02 * K, "{f:?}");
    }

    /// A hold the limiter does not govern: its current falls 0.4 %/C as
    /// the copper rises to 15 C, slow at the reference's scale.
    #[test]
    fn a_current_that_drifts_with_the_copper_leaves_the_reference_at_the_start() {
        let s = Synth {
            k: 1500.0 / (270.0 * 270.0),
            scale: |t| 1.0 - 0.004 * 15.0 * (1.0 - (-t / 131_072.0).exp()),
            ..Synth::hold(480_000.0, 115)
        };
        let f = s.fit().unwrap();
        let (v, a) = scales();
        assert_eq!(f.r_ref_settle_ms, 0.0, "{f:?}");
        assert!(f.r_ref_start_ms < DIP_MS, "{f:?}");
        assert!((f.r_ref_ohm - v / a).abs() < 1e-9 * v / a, "{f:?}");
    }

    /// A seated hold whose current falls 2 %/s over its first 5 s at one
    /// duty, its copper warming 5 C/s and more: that is the motor being
    /// measured, not an unsettled current. Accepted, and R_ref reads the
    /// cold copper within 0.3 C.
    #[test]
    fn a_current_falling_with_the_warming_copper_is_accepted() {
        let polls = flat_polls(DENSE_MS, CEILING);
        let fall = |t: f64| 1.0 - 0.02 * t.min(5000.0) / 1000.0;
        assert!((fall(3000.0) - 0.94).abs() < 1e-12);
        let windows: Vec<ThermWindow> = (0..(DENSE_MS / WIN_MS) as usize)
            .map(|k| {
                let t = 2.0 + k as f64 * WIN_MS;
                let x_w = (1.0 / fall(t) - 1.0) * (COPPER_ZERO_R_C + T_NTC0);
                tw(window_at(t, I * fall(t) + wobble(k), DUTY, 1.0, x_w))
            })
            .collect();
        let rf = reference_of(&polls, &windows).unwrap();
        assert_eq!(rf.settle_ms, 0.0);
        assert!(
            off_c(&rf).abs() < 0.3,
            "R_ref reads {:.3} C off",
            off_c(&rf)
        );
        assert!(rf.slope_c_per_s > 5.0, "{}", rf.slope_c_per_s);
    }

    /// A copper that follows the replayed heat exactly settles where the
    /// regression says: the plateau's k is the regression's.
    #[test]
    fn the_plateau_reads_the_regression_on_a_copper_that_follows_the_heat() {
        let f = Synth::hold(300_000.0, 66).fit().unwrap();
        assert!((f.k - K).abs() < 1e-9 * K, "{f:?}");
        assert!((f.plateau_k - f.k).abs() < 1e-9 * K, "{f:?}");
        assert_eq!(f.plateau_rise_q016.raw, f.rise_q016.raw);
    }

    /// A hold whose windows stop short of the current's floor, whose NTC
    /// readings cover too few bins, or that has no NTC base: refused, each
    /// named.
    #[test]
    fn a_short_or_unbased_hold_is_refused() {
        let (v, a) = scales();
        let w = |k: usize| {
            tw(WindowSample {
                t_ms: k as f64 * WIN_MS,
                i: -270.0 + wobble(k),
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
        let fit = |polls: &[HoldPoll], windows: &[ThermWindow]| {
            fit_rise(polls, windows, 0.0, f64::INFINITY, SEAT, 4, v, a).unwrap_err()
        };
        let polls: Vec<HoldPoll> = (-10..10).map(|k| p(k as f64 * 500.0)).collect();
        let held: Vec<ThermWindow> = (0..(DENSE_MS / WIN_MS) as usize).map(w).collect();
        let why = fit(&polls, &held);
        assert!(why.contains("bins"), "{why}");
        let short: Vec<ThermWindow> = (0..2500).map(w).collect();
        let why = fit(&polls, &short);
        assert!(why.contains("sd floor"), "{why}");
        let late: Vec<HoldPoll> = (0..10).map(|k| p(k as f64 * 500.0)).collect();
        let why = fit(&late, &held);
        assert!(why.contains("before the hold"), "{why}");
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
