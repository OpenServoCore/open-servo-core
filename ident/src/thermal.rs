//! The winding estimate's constants (core `estimator::thermal`): the
//! kernel reads `t = t_ntc + rise x heat + rise2 x heat2`, the heats the
//! kernel's EWMAs of the held current squared, warmed by the copper's
//! rise, on `thermal_tau_s` and `thermal_tau2_s`. The board NTC's
//! counts-to-centi-C curve is a board fact, encoded from its beta constants
//! ([`NtcCurve`]). Two host routes to the gains: a nameplate thermal
//! resistance ([`rise_from_rth`]) for the rotor alone, or one hold at one
//! duty, torque on and the rotor never released, whose own copper against
//! the two heats replayed in the kernel's arithmetic gives all four
//! constants ([`fit_two_node`]). Any duty change moves the brush contact
//! behind the gear train and shifts R by percents, so every reading of the
//! hold is against the hold's own R_ref at the same duty, read after its
//! seat and its dip settle ([`settle_dip`]). The contact keeps settling
//! for seconds after, so the fit leaves the hold's first [`FIT_FROM_MS`]
//! out and checks its first fitted block against the hold's own noise,
//! moving the blank out to [`FIT_FROM_MAX_MS`] before it refuses.

use osc_servo_core::estimator::thermal::{rise_cc, warm_q16};
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
/// ceiling: the copper only lowers it from there, which keeps the limiter
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
             counts current ceiling, inside the {:.0} % margin that keeps the limiter off the \
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

/// The two-node fit's first blank, ms: the brush contact settles over the
/// hold's first seconds and reads as copper, no node the estimate runs.
pub const FIT_FROM_MS: f64 = 20_000.0;
/// The fitted residual's first block, ms, judged flat against the hold's
/// own noise; a block that is not moves the blank out by its length.
pub const FLAT_BLOCK_MS: f64 = 20_000.0;
/// The blank never extends past this, ms: a hold whose residual is not
/// flat by then is a contact that had not settled, refused.
pub const FIT_FROM_MAX_MS: f64 = 60_000.0;
/// The first block is flat when its mean sits within this many floors of
/// the hold's own noise, the residual's sd inflated for its lag-1
/// autocorrelation over the block's bins.
pub const FLAT_SIGMAS: f64 = 2.0;
/// A block mean inside this, C, is flat whatever the hold's noise says:
/// every threshold the estimate is held against is set in whole degrees,
/// and a noiseless residual is tiny but structured, so the sigma rule
/// alone would refuse a perfect hold.
pub const FLAT_MIN_C: f64 = 0.1;
/// The lag-1 coefficient the inflation reads at most: at 1 the floor is
/// unbounded.
const RHO_MAX: f64 = 0.95;
/// The rotor leaks the fit chooses between, s: the SLOW period's octaves
/// either side of the 33 s every hold on record picks.
pub const ROTOR_TAUS_S: [u16; 2] = [33, 66];
/// The body leaks, s: the two longest octaves the kernel runs.
pub const BODY_TAUS_S: [u16; 2] = [262, 524];
/// One octave beats the other when its SSE sits under the other's by this
/// ratio; under it the hold decides nothing and the safe octave is written.
pub const OCTAVE_DECIDES: f64 = 1.10;
/// An undecided body writes the slowest octave the hold covers this many
/// times over: 960 s covers 524 s, 480 s covers 262 s alone.
pub const BODY_COVER: f64 = 1.6;
/// The feedback passes have settled when neither gain moves by more.
const PASS_SETTLED_Q016: u16 = 2;
const PASSES_MAX: usize = 8;
/// The residual's block means are over this long.
pub const BLOCK_MS: f64 = 60_000.0;

/// One heat node as the fit reads it.
#[derive(Clone, Debug, PartialEq)]
pub struct NodeFit {
    /// The leak written, s.
    pub tau_s: u16,
    /// The shift the kernel runs for it ([`tau_shift`]).
    pub shift: u8,
    /// centi-C per count squared of its heat.
    pub k: f64,
    pub rise_q016: Encoded,
    /// The slower octave's SSE over the written one's: over
    /// [`OCTAVE_DECIDES`] the hold decided it.
    pub sse_ratio: f64,
    pub decided: bool,
}

impl NodeFit {
    /// The kernel's leak in seconds: 2^shift SLOW periods.
    pub fn octave_s(&self) -> f64 {
        (1u32 << self.shift) as f64 * SLOW_PERIOD_MS as f64 / 1000.0
    }
}

/// One hold's part of a fit: its reference and its own diagnostics.
#[derive(Clone, Debug, PartialEq)]
pub struct HoldFit {
    pub r_ref_ohm: f64,
    /// Where R_ref is read, the dip's bottom, ms into the hold.
    pub r_ref_start_ms: f64,
    pub r_ref_settle_ms: f64,
    pub r_ref_dip_c: f64,
    pub r_ref_windows: usize,
    pub r_ref_slope_c_per_s: f64,
    pub i_cold_counts: f64,
    pub ceiling_counts: f64,
    pub t0_c: f64,
    /// The hold's length the fit saw, s.
    pub hold_s: f64,
    /// The blank this hold was fitted from, ms into it: [`FIT_FROM_MS`], or
    /// [`FLAT_BLOCK_MS`] steps past it to [`FIT_FROM_MAX_MS`].
    pub fit_from_ms: f64,
    /// The residual's mean over the bins before the blank, C: the brush
    /// node, reported, never fitted.
    pub first_c: f64,
    /// The residual's mean per [`BLOCK_MS`] block of the fitted bins, C.
    pub block_means_c: Vec<f64>,
    /// The copper over the NTC when R_ref was read, C, by the fit's own
    /// replay at the written gains: about 0 on a cold start, the rise
    /// carried from the hold before on a warm one. R/R_ref converts to
    /// degrees on the copper's temperature there.
    pub warm_ref_c: f64,
    /// The copper's excess over the NTC at the end, the mean over the last
    /// [`SETTLED_BINS`] bins, C: `warm_ref_c` and the rise since.
    pub settled_rise_c: f64,
    /// The kernel's winding estimate less the copper over the same closing
    /// bins, C; None while the estimate read UNSET.
    pub estimate_err_c: Option<f64>,
    /// The fitted bins this hold contributed.
    pub bins: usize,
}

impl HoldFit {
    /// How far the cold current sits under the ceiling, a fraction of it.
    pub fn margin(&self) -> f64 {
        1.0 - self.i_cold_counts / self.ceiling_counts
    }
}

/// The winding estimate's four constants from one hold or a pool of them
/// ([`fit_two_node_pooled`]).
#[derive(Clone, Debug, PartialEq)]
pub struct TwoNodeFit {
    pub rotor: NodeFit,
    pub body: NodeFit,
    /// Per-bin scatter of the copper's excess over the NTC around the two
    /// nodes, C, pooled over every hold's fitted bins.
    pub residual_sd_c: f64,
    /// Every hold's fitted bins together.
    pub bins: usize,
    /// The feedback passes the written pair took.
    pub passes: usize,
    /// In the order the holds were given; one for a single hold.
    pub holds: Vec<HoldFit>,
}

impl TwoNodeFit {
    /// The copper's settled rise over the NTC at a held current, C: the
    /// fixed point of both gains on the warmed square ([`plateau_c`]).
    pub fn plateau_c(&self, i_counts: f64) -> f64 {
        plateau_c(self.rotor.k + self.body.k, i_counts)
    }
}

/// The copper's settled rise over the NTC at a held current, C, of the
/// gains' sum `g_cc`, centi-C per count squared: the fixed point on the
/// warmed square, `G i^2 / (1 - alpha G i^2)` with G in C per count
/// squared.
fn plateau_c(g_cc: f64, i_counts: f64) -> f64 {
    let g = g_cc / 100.0 * i_counts * i_counts;
    g / (1.0 - COPPER_ALPHA_PER_C * g)
}

/// A pool's spread reads from this many single-hold fits: fewer cannot
/// tell a seat's share from the fit's own scatter.
pub const SPREAD_MIN_HOLDS: usize = 3;

/// How far the rotor gains of a pool of pinned single-hold fits spread
/// ([`pool_spread`]): each hold is one brush seat, so the spread is the
/// seat's share of the gain.
#[derive(Clone, Debug, PartialEq)]
pub struct PoolSpread {
    pub n: usize,
    /// The rotor gains' mean, centi-C per count squared.
    pub k_mean: f64,
    /// Their sample sd.
    pub k_sd: f64,
    /// The mean's standard error, sd over root n, as degrees at the
    /// ceiling: the plateau there with the rotor at `k_mean` plus one
    /// error, less the plateau at `k_mean`, the body as pinned.
    pub se_c_at_ceiling: f64,
    /// The high stop's mean plateau at the ceiling less the low stop's,
    /// C; None until both stops have a hold.
    pub split_c: Option<f64>,
}

/// The rotor gains' spread over `singles`, each one hold fitted alone with
/// the body pinned ([`fit_rotor_pinned`]), as degrees at `ceiling_counts`
/// on the first single's body; `dirs[i]` is the stop `singles[i]` pressed,
/// -1 low, +1 high. None under [`SPREAD_MIN_HOLDS`].
pub fn pool_spread(singles: &[TwoNodeFit], ceiling_counts: f64, dirs: &[i8]) -> Option<PoolSpread> {
    let n = singles.len();
    if n < SPREAD_MIN_HOLDS {
        return None;
    }
    let ks: Vec<f64> = singles.iter().map(|f| f.rotor.k).collect();
    let k_mean = mean(ks.iter().copied())?;
    let k_sd = sd(&ks);
    let k_body = singles[0].body.k;
    let at_ceiling = |k: f64| plateau_c(k + k_body, ceiling_counts);
    let stop = |sign: i8| {
        mean(
            ks.iter()
                .zip(dirs)
                .filter(|(_, d)| d.signum() == sign)
                .map(|(k, _)| *k),
        )
    };
    Some(PoolSpread {
        n,
        k_mean,
        k_sd,
        se_c_at_ceiling: at_ceiling(k_mean + k_sd / (n as f64).sqrt()) - at_ceiling(k_mean),
        split_c: stop(1)
            .zip(stop(-1))
            .map(|(high, low)| at_ceiling(high) - at_ceiling(low)),
    })
}

/// Copper's resistance rise per C, the kernel's `ALPHA_Q26` in physical
/// units.
pub const COPPER_ALPHA_PER_C: f64 = 0.00393;

/// The kernel's two heats over the polls, its own integer loop: each SLOW
/// step squares the last poll's held current, warms it by the rise the
/// step before published ([`warm_q16`]) and steps both EWMAs on `shifts`;
/// the rise is the two heats on `gains` ([`rise_cc`]). Sampled at `at_ms`
/// (ascending), rotor then body, counts squared.
pub fn replay_two(
    polls: &[HoldPoll],
    shifts: [u8; 2],
    gains: [u16; 2],
    at_ms: &[f64],
) -> Vec<[f64; 2]> {
    let Some(first) = polls.first() else {
        return vec![[0.0; 2]; at_ms.len()];
    };
    let mut st = LimitState::new();
    let mut rise: u16 = 0;
    let mut out = Vec::with_capacity(at_ms.len());
    let (mut p, mut g) = (0usize, first.t_ms);
    for &at in at_ms {
        while g <= at {
            while p + 1 < polls.len() && polls[p + 1].t_ms <= g {
                p += 1;
            }
            let i = polls[p].i_counts.abs().round().min(i16::MAX as f64) as i16;
            st.heat(i, warm_q16(rise), shifts);
            let [h1, h2] = st.heats();
            rise = (rise_cc(h1, gains[0]) + rise_cc(h2, gains[1])).min(i16::MAX as i32) as u16;
            g += SLOW_PERIOD_MS as f64;
        }
        out.push(st.heats().map(|h| h as f64));
    }
    out
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}

/// Least squares of `y` on one column with the coefficient held at or over
/// zero; 0 on a column without scatter.
fn nnls1(x: &[f64], y: &[f64]) -> f64 {
    let sxx = dot(x, x);
    if sxx > 0.0 {
        (dot(x, y) / sxx).max(0.0)
    } else {
        0.0
    }
}

/// Least squares of `y` on two columns with both coefficients held at or
/// over zero: the free solution when it is non-negative, else the better of
/// the two one-column solutions ([`nnls1`]) with the other at zero. Exact
/// for two unknowns.
fn nnls2(x1: &[f64], x2: &[f64], y: &[f64]) -> [f64; 2] {
    let (s11, s22, s12) = (dot(x1, x1), dot(x2, x2), dot(x1, x2));
    let (s1y, s2y) = (dot(x1, y), dot(x2, y));
    let det = s11 * s22 - s12 * s12;
    if det > 0.0 {
        let k1 = (s22 * s1y - s12 * s2y) / det;
        let k2 = (s11 * s2y - s12 * s1y) / det;
        if k1 >= 0.0 && k2 >= 0.0 {
            return [k1, k2];
        }
    }
    let a = [nnls1(x1, y), 0.0];
    let b = [0.0, nnls1(x2, y)];
    let sse = |k: [f64; 2]| {
        x1.iter()
            .zip(x2)
            .zip(y)
            .map(|((a, b), y)| (y - k[0] * a - k[1] * b).powi(2))
            .sum::<f64>()
    };
    if sse(a) <= sse(b) { a } else { b }
}

/// One recorded hold as the pooled fit reads it.
pub struct Hold<'a> {
    pub polls: &'a [HoldPoll],
    pub windows: &'a [ThermWindow],
    pub t0_ms: f64,
    pub end_ms: f64,
    pub seat: u16,
}

/// One 1 s bin of a hold that read.
struct Bin {
    /// Its centre, ms on the host's clock.
    t_ms: f64,
    /// The copper's rise over its reference, C, converted on the NTC
    /// base's temperature ([`Reference::copper`]).
    copper_c: f64,
    /// The NTC over its base, C.
    ntc_c: f64,
    /// The kernel's estimate, C, when it was set.
    estimate: Option<f64>,
}

/// A hold ready for the pair fits: its reference, its 1 s bins and where
/// the fitted ones start.
struct Prepared<'a> {
    polls: &'a [HoldPoll],
    rf: Reference,
    t0_ms: f64,
    hold_s: f64,
    all: Vec<Bin>,
    /// Where the replay samples: t_ref first, then every bin's centre.
    at: Vec<f64>,
    /// The blank in force, ms into the hold, and the first fitted bin.
    from_ms: f64,
    from: usize,
}

impl Prepared<'_> {
    /// What a bin's `copper_c` scales by when the copper sat `warm_ref_c`
    /// over the NTC at the reference: R(T) runs with `COPPER_ZERO_R_C + T`,
    /// so R/R_ref - 1 converts to degrees on the copper's own temperature
    /// there, not the NTC's.
    fn scale(&self, warm_ref_c: f64) -> f64 {
        (COPPER_ZERO_R_C + self.rf.t0_c + warm_ref_c) / (COPPER_ZERO_R_C + self.rf.t0_c)
    }

    /// Every bin's copper over the NTC, less the NTC's own move, C, with
    /// the copper `warm_ref_c` over the NTC at the reference.
    fn y(&self, warm_ref_c: f64) -> Vec<f64> {
        let scale = self.scale(warm_ref_c);
        self.all
            .iter()
            .map(|b| b.copper_c * scale - b.ntc_c)
            .collect()
    }

    /// Blank the hold to `from_ms`: the bins before it leave the fit, and
    /// the fit refuses a hold with fewer than [`MIN_BINS`] left.
    fn blank(&mut self, from_ms: f64) -> Result<(), String> {
        self.from_ms = from_ms;
        self.from = self.all.partition_point(|b| b.t_ms < self.t0_ms + from_ms);
        let n = self.all.len() - self.from;
        if n < MIN_BINS {
            return Err(format!(
                "the hold covers {n} one-second bins with a window and an NTC reading past \
                 {:.0} s, under the {MIN_BINS} the fit needs",
                from_ms / 1000.0
            ));
        }
        Ok(())
    }

    /// The first [`FLAT_BLOCK_MS`] of the hold's fitted residual `res`
    /// against the hold's own noise: the block's mean and the bound it
    /// had to sit inside, when it did not. The bound is [`FLAT_SIGMAS`]
    /// floors, the floor the residual's sd over the fitted bins after the
    /// block inflated for its lag-1 autocorrelation over the root of the
    /// block's bins, or [`FLAT_MIN_C`] when that is more, and alone when
    /// under three bins follow. The block is judged against the rest of
    /// the hold, never against itself: on a short hold an unsettled
    /// contact is most of the residual's scatter and would pass as noise.
    #[allow(clippy::manual_clamp)]
    fn unflat(&self, res: &[f64]) -> Option<(f64, f64)> {
        let fitted = &res[self.from..];
        let end = self.all[self.from..]
            .partition_point(|b| b.t_ms < self.t0_ms + self.from_ms + FLAT_BLOCK_MS);
        let m = mean(fitted[..end].iter().copied())?;
        let rest = &fitted[end..];
        let floor = if rest.len() < 3 {
            0.0
        } else {
            let rho = lag1(rest).max(0.0).min(RHO_MAX);
            sd(rest) * ((1.0 + rho) / (1.0 - rho) / end as f64).sqrt()
        };
        let bound = (FLAT_SIGMAS * floor).max(FLAT_MIN_C);
        (m.abs() > bound).then_some((m, bound))
    }
}

/// The lag-1 autocorrelation coefficient of `xs`, 0 for fewer than three
/// or no scatter.
fn lag1(xs: &[f64]) -> f64 {
    if xs.len() < 3 {
        return 0.0;
    }
    let m = xs.iter().sum::<f64>() / xs.len() as f64;
    let var = xs.iter().map(|x| (x - m).powi(2)).sum::<f64>();
    if var <= 0.0 {
        return 0.0;
    }
    xs.windows(2).map(|w| (w[0] - m) * (w[1] - m)).sum::<f64>() / var
}

fn prepare<'a>(
    h: &Hold<'a>,
    r_ohm: &impl Fn(&WindowSample) -> Option<f64>,
) -> Result<Prepared<'a>, String> {
    let rf = reference(h.polls, h.windows, h.t0_ms, h.end_ms, h.seat, r_ohm)?;
    let hold_s = rf.seconds.len() as f64 * BIN_MS / 1000.0;
    let all: Vec<Bin> = rf
        .seconds
        .iter()
        .enumerate()
        .filter_map(|(j, s)| {
            let (r, t) = (s.r.mean()?, s.ntc.mean()?);
            Some(Bin {
                t_ms: h.t0_ms + (j as f64 + 0.5) * BIN_MS,
                copper_c: rf.copper(r),
                ntc_c: t - rf.t0_c,
                estimate: s.winding.mean().filter(|_| !s.unset),
            })
        })
        .collect();
    let mut at = Vec::with_capacity(all.len() + 1);
    at.push(h.t0_ms + rf.start_ms);
    at.extend(all.iter().map(|b| b.t_ms));
    let mut p = Prepared {
        polls: h.polls,
        rf,
        t0_ms: h.t0_ms,
        hold_s,
        all,
        at,
        from_ms: 0.0,
        from: 0,
    };
    p.blank(FIT_FROM_MS)?;
    Ok(p)
}

/// One octave pair fitted: the shared gains, the SSE over every hold's
/// fitted bins, the residual per bin of each hold and the passes the
/// feedback took.
struct PairFit {
    taus: [u16; 2],
    k: [f64; 2],
    q: [Encoded; 2],
    sse: f64,
    res: Vec<Vec<f64>>,
    /// Each hold's copper over the NTC at its reference, C, on the last
    /// pass's gains.
    warm_ref: Vec<f64>,
    passes: usize,
}

/// The pair `taus` fitted over every hold's bins from its `from` on, with
/// its reference at its own t_ref: the replay at the last pass's gains
/// gives each hold the columns `heat(t) - heat(t_ref)` and the copper it
/// carried at t_ref, which sets the hold's y ([`Prepared::y`]); the fitted
/// parts of every hold's columns and y are concatenated and solved once by
/// the non-negative solve (the gains shared), and the loop closes when
/// both q016 values hold within [`PASS_SETTLED_Q016`]. A `pinned` body is
/// known: the replay runs its shift and gain and the rotor alone is
/// solved on what it leaves of y. Each hold's residual covers every bin,
/// the SSE the fitted ones.
fn fit_pair(holds: &[Prepared<'_>], taus: [u16; 2], pinned: Option<&NodeFit>) -> PairFit {
    let shifts = match pinned {
        Some(b) => [tau_shift(taus[0]), b.shift],
        None => taus.map(tau_shift),
    };
    let q_body = pinned.map_or(0, |b| b.rise_q016.raw);
    let mut q = [0u16, q_body];
    let mut passes = 0;
    let (k, res, warm_ref): ([f64; 2], Vec<Vec<f64>>, Vec<f64>) = loop {
        passes += 1;
        let mut warm_ref = Vec::with_capacity(holds.len());
        let cols: Vec<[Vec<f64>; 2]> = holds
            .iter()
            .map(|h| {
                let r = replay_two(h.polls, shifts, q, &h.at);
                warm_ref.push((q[0] as f64 * r[0][0] + q[1] as f64 * r[0][1]) / 65536.0 / 100.0);
                [0, 1].map(|n| r[1..].iter().map(|v| v[n] - r[0][n]).collect())
            })
            .collect();
        let ys: Vec<Vec<f64>> = holds.iter().zip(&warm_ref).map(|(h, &w)| h.y(w)).collect();
        let (mut x1, mut x2, mut y) = (Vec::new(), Vec::new(), Vec::new());
        for ((h, c), hy) in holds.iter().zip(&cols).zip(&ys) {
            x1.extend_from_slice(&c[0][h.from..]);
            x2.extend_from_slice(&c[1][h.from..]);
            y.extend_from_slice(&hy[h.from..]);
        }
        let k = match pinned {
            Some(b) => {
                let kb = b.k / 100.0;
                let left: Vec<f64> = y.iter().zip(&x2).map(|(y, b)| y - kb * b).collect();
                [nnls1(&x1, &left), kb]
            }
            None => nnls2(&x1, &x2, &y),
        };
        let res = cols
            .iter()
            .zip(&ys)
            .map(|([x1, x2], hy)| {
                x1.iter()
                    .zip(x2)
                    .zip(hy)
                    .map(|((a, b), y)| y - k[0] * a - k[1] * b)
                    .collect()
            })
            .collect();
        let mut next = k.map(|k| enc(k * 100.0, 65536.0).raw);
        if pinned.is_some() {
            next[1] = q_body;
        }
        let settled = next
            .iter()
            .zip(&q)
            .all(|(n, q)| n.abs_diff(*q) <= PASS_SETTLED_Q016);
        q = next;
        if settled || passes >= PASSES_MAX {
            break (k, res, warm_ref);
        }
    };
    let sse = holds
        .iter()
        .zip(&res)
        .map(|(h, r)| r[h.from..].iter().map(|e| e * e).sum::<f64>())
        .sum();
    PairFit {
        taus,
        k: k.map(|k| k * 100.0),
        q: k.map(|k| enc(k * 100.0, 65536.0)),
        sse,
        res,
        warm_ref,
        passes,
    }
}

/// The four constants from one hold's own copper ([`fit_two_node_pooled`]
/// over a pool of one). R_ref and T_0 as [`reference`], the settle judged
/// against `seat`.
#[allow(clippy::too_many_arguments)]
pub fn fit_two_node(
    polls: &[HoldPoll],
    windows: &[ThermWindow],
    t0_ms: f64,
    end_ms: f64,
    seat: u16,
    v_per_count: f64,
    a_per_count: f64,
) -> Result<TwoNodeFit, String> {
    fit_two_node_pooled(
        &[Hold {
            polls,
            windows,
            t0_ms,
            end_ms,
            seat,
        }],
        v_per_count,
        a_per_count,
    )
}

/// One node's octave as the four pair fits decide it.
struct Octave {
    tau: u16,
    ratio: f64,
    decided: bool,
}

/// The written pair over the pair fits: the pair with the least pooled SSE
/// is read; an octave the SSE ratio leaves undecided ([`OCTAVE_DECIDES`])
/// writes the safe one: the faster rotor (the slower under-reads the first
/// minute), the slowest body a hold of `shortest_s` covers [`BODY_COVER`]
/// times over (the faster under-reads a long hold, and a hold that does
/// not cover an octave cannot certify it). A `pinned` body is the octave
/// given, ratio 1: nothing here decided it. The written pair's place in
/// `fits`, then the rotor and the body.
fn choose(
    fits: &[PairFit],
    shortest_s: f64,
    pinned: Option<&NodeFit>,
) -> Result<(usize, Octave, Octave), String> {
    let sse_of = |taus: [u16; 2]| {
        fits.iter()
            .find(|f| f.taus == taus)
            .map_or(f64::INFINITY, |f| f.sse)
    };
    let best = fits
        .iter()
        .min_by(|a, b| a.sse.total_cmp(&b.sse))
        .ok_or("no octave pair fitted")?;
    let ratio = |taus: [u16; 2]| {
        let other = sse_of(taus);
        if best.sse > 0.0 {
            other / best.sse
        } else if other > 0.0 {
            f64::INFINITY
        } else {
            1.0
        }
    };
    let rotor_ratio = ratio([other_of(&ROTOR_TAUS_S, best.taus[0]), best.taus[1]]);
    let rotor = Octave {
        decided: rotor_ratio >= OCTAVE_DECIDES,
        tau: if rotor_ratio >= OCTAVE_DECIDES {
            best.taus[0]
        } else {
            ROTOR_TAUS_S[0]
        },
        ratio: rotor_ratio,
    };
    let body = match pinned {
        Some(b) => Octave {
            tau: b.tau_s,
            ratio: 1.0,
            decided: b.decided,
        },
        None => {
            let body_ratio = ratio([best.taus[0], other_of(&BODY_TAUS_S, best.taus[1])]);
            Octave {
                decided: body_ratio >= OCTAVE_DECIDES,
                tau: if body_ratio >= OCTAVE_DECIDES {
                    best.taus[1]
                } else {
                    BODY_TAUS_S
                        .iter()
                        .rev()
                        .copied()
                        .find(|&b| shortest_s >= BODY_COVER * b as f64)
                        .unwrap_or(BODY_TAUS_S[0])
                },
                ratio: body_ratio,
            }
        }
    };
    let written = fits
        .iter()
        .position(|f| f.taus == [rotor.tau, body.tau])
        .ok_or("the written pair was not fitted")?;
    Ok((written, rotor, body))
}

/// The four constants from the holds' own copper: per 1 s bin from each
/// hold's blank to its end, the copper's rise over the hold's NTC base
/// less the NTC's own, against the kernel's two heats replayed with the
/// warm input fed back ([`replay_two`]), one R_ref and NTC base per hold
/// and the gains shared by the non-negative solve over every hold's fitted
/// bins, over the four octave pairs of [`ROTOR_TAUS_S`] x [`BODY_TAUS_S`],
/// the written pair chosen on the pooled SSE ([`choose`]). Every hold's
/// blank starts at [`FIT_FROM_MS`]; a hold whose fitted residual is not
/// flat over its first [`FLAT_BLOCK_MS`] ([`Prepared::unflat`]) moves its
/// blank out by that much and the whole fit runs again, until every hold
/// is flat or one is not at [`FIT_FROM_MAX_MS`], which refuses it: the
/// contact had not settled. Each hold moves at most twice, so a pool of
/// one fits at most three times. A hold that fails names its place when
/// there is more than one.
pub fn fit_two_node_pooled(
    holds: &[Hold<'_>],
    v_per_count: f64,
    a_per_count: f64,
) -> Result<TwoNodeFit, String> {
    fit_pool(holds, None, v_per_count, a_per_count)
}

/// The rotor's gain and octave from holds too short to see the body, the
/// body's gain and octave `body` pinned from the servo's long hold: as
/// [`fit_two_node_pooled`] over the pairs [`ROTOR_TAUS_S`] x the body's
/// octave, the replay fed the body's gain and the rotor's gain alone
/// solved on what the body leaves of the copper. The body node written
/// is `body` as given, its SSE ratio 1.
pub fn fit_rotor_pinned(
    holds: &[Hold<'_>],
    body: &NodeFit,
    v_per_count: f64,
    a_per_count: f64,
) -> Result<TwoNodeFit, String> {
    fit_pool(holds, Some(body), v_per_count, a_per_count)
}

/// [`fit_two_node_pooled`] and [`fit_rotor_pinned`] in one body: `pinned`
/// is the body the pairs and the solve are held to, None for both gains
/// free.
fn fit_pool(
    holds: &[Hold<'_>],
    pinned: Option<&NodeFit>,
    v_per_count: f64,
    a_per_count: f64,
) -> Result<TwoNodeFit, String> {
    if holds.is_empty() {
        return Err("no hold to fit".into());
    }
    let r_ohm = r_ohm_at(v_per_count, a_per_count)?;
    let named = |i: usize, why: String| {
        if holds.len() > 1 {
            format!("hold {}: {why}", i + 1)
        } else {
            why
        }
    };
    let mut prepared: Vec<Prepared<'_>> = holds
        .iter()
        .enumerate()
        .map(|(i, h)| prepare(h, &r_ohm).map_err(|why| named(i, why)))
        .collect::<Result<_, _>>()?;
    let shortest_s = prepared
        .iter()
        .map(|h| h.hold_s)
        .fold(f64::INFINITY, f64::min);
    let bodies: &[u16] = match pinned {
        Some(b) => std::slice::from_ref(&b.tau_s),
        None => &BODY_TAUS_S,
    };
    let (written, rotor, body) = loop {
        let mut fits: Vec<PairFit> = ROTOR_TAUS_S
            .iter()
            .flat_map(|&r| bodies.iter().map(move |&b| [r, b]))
            .map(|taus| fit_pair(&prepared, taus, pinned))
            .collect();
        let (at, rotor, body) = choose(&fits, shortest_s, pinned)?;
        let mut moved = false;
        for (i, h) in prepared.iter_mut().enumerate() {
            let Some((m, bound)) = h.unflat(&fits[at].res[i]) else {
                continue;
            };
            if h.from_ms >= FIT_FROM_MAX_MS {
                return Err(named(
                    i,
                    format!(
                        "the fitted residual's first {:.0} s block read {m:+.2} C against \
                         +/-{:.2} C of the hold's own noise, {:.0} s in: the contact had not \
                         settled",
                        FLAT_BLOCK_MS / 1000.0,
                        bound,
                        h.from_ms / 1000.0
                    ),
                ));
            }
            h.blank(h.from_ms + FLAT_BLOCK_MS)
                .map_err(|why| named(i, why))?;
            moved = true;
        }
        if !moved {
            break (fits.swap_remove(at), rotor, body);
        }
    };
    let n: usize = prepared.iter().map(|h| h.all.len() - h.from).sum();
    let residual_sd_c = (prepared
        .iter()
        .zip(&written.res)
        .map(|(h, r)| r[h.from..].iter().map(|e| e * e).sum::<f64>())
        .sum::<f64>()
        / (n - 1) as f64)
        .sqrt();
    let hold_fits = prepared
        .iter()
        .zip(&written.res)
        .zip(&written.warm_ref)
        .map(|((h, res), &w)| hold_fit(h, res, w))
        .collect();
    let node = |n: usize, o: Octave| NodeFit {
        tau_s: o.tau,
        shift: tau_shift(o.tau),
        k: written.k[n],
        rise_q016: written.q[n],
        sse_ratio: o.ratio,
        decided: o.decided,
    };
    Ok(TwoNodeFit {
        rotor: node(0, rotor),
        body: match pinned {
            Some(b) => NodeFit {
                sse_ratio: 1.0,
                ..b.clone()
            },
            None => node(1, body),
        },
        residual_sd_c,
        bins: n,
        passes: written.passes,
        holds: hold_fits,
    })
}

/// One hold's diagnostics from the written pair's residual over its bins,
/// the copper `warm_ref_c` over the NTC at its reference.
fn hold_fit(h: &Prepared<'_>, res: &[f64], warm_ref_c: f64) -> HoldFit {
    let rf = &h.rf;
    let (early, fitted) = res.split_at(h.from);
    let first_c = mean(early.iter().copied()).unwrap_or(0.0);
    let timed: Vec<(f64, f64)> = h.all[h.from..]
        .iter()
        .zip(fitted)
        .map(|(b, r)| (b.t_ms, *r))
        .collect();
    let block = |t: f64| ((t - h.t0_ms) / BLOCK_MS).floor();
    let block_means_c: Vec<f64> = timed
        .chunk_by(|a, b| block(a.0) == block(b.0))
        .map(|c| c.iter().map(|x| x.1).sum::<f64>() / c.len() as f64)
        .collect();
    let settled = &h.all[h.all.len().saturating_sub(SETTLED_BINS)..];
    let m = settled.len() as f64;
    let scale = h.scale(warm_ref_c);
    let settled_rise_c = warm_ref_c
        + settled
            .iter()
            .map(|b| b.copper_c * scale - b.ntc_c)
            .sum::<f64>()
            / m;
    let copper_c =
        rf.t0_c + warm_ref_c + settled.iter().map(|b| b.copper_c * scale).sum::<f64>() / m;
    let estimate_err_c = settled
        .iter()
        .map(|b| b.estimate)
        .sum::<Option<f64>>()
        .map(|s| s / m - copper_c);
    HoldFit {
        r_ref_ohm: rf.r_ref,
        r_ref_start_ms: rf.start_ms,
        r_ref_settle_ms: rf.settle_ms,
        r_ref_dip_c: rf.dip_c,
        r_ref_windows: rf.windows,
        r_ref_slope_c_per_s: rf.slope_c_per_s,
        i_cold_counts: rf.cold.i_counts,
        ceiling_counts: rf.cold.ceiling,
        t0_c: rf.t0_c,
        hold_s: h.hold_s,
        fit_from_ms: h.from_ms,
        first_c,
        block_means_c,
        warm_ref_c,
        settled_rise_c,
        estimate_err_c,
        bins: h.all.len() - h.from,
    }
}

/// The other entry of a two-octave choice.
fn other_of(taus: &[u16; 2], tau: u16) -> u16 {
    if taus[0] == tau { taus[1] } else { taus[0] }
}

/// The reference a live hold reads its copper against once its windows
/// cover the current's floor and the R_ref line: R_ref in the window's own
/// vcounts per ccount and the NTC base T_0, C ([`reference`]).
pub fn hold_reference(
    polls: &[HoldPoll],
    windows: &[ThermWindow],
    t0_ms: f64,
    seat: u16,
) -> Result<(f64, f64), String> {
    let rf = reference(polls, windows, t0_ms, f64::INFINITY, seat, window_r_vpc)?;
    Ok((rf.r_ref, rf.t0_c))
}

/// The copper's temperature, C, from the mean R of the windows at the
/// command from `from_ms` on, against `r_ref` and `t0_c`; None without one.
/// It converts on the NTC's temperature, so a warm start's excess reads
/// low by its rise at the reference over `COPPER_ZERO_R_C + T`: under 2 %
/// for a 5 C warm start, which the session's holds start within.
pub fn copper_c(windows: &[ThermWindow], from_ms: f64, r_ref: f64, t0_c: f64) -> Option<f64> {
    let lo = windows.partition_point(|x| x.w.t_ms < from_ms);
    let r = mean(
        windows[lo..]
            .iter()
            .filter(|x| at_command(x))
            .filter_map(|x| window_r_vpc(&x.w)),
    )?;
    Some(t0_c + (r / r_ref - 1.0) * (COPPER_ZERO_R_C + t0_c))
}

/// The rest check watches the NTC this long before a hold drives.
pub const REST_WINDOW_MS: f64 = 60_000.0;
/// ... and gives up after this long.
pub const REST_MAX_MS: f64 = 300_000.0;
/// ... polling this often.
pub const REST_POLL_MS: f64 = 1000.0;
/// A board at rest drifts under this, C per minute.
pub const REST_DRIFT_C_PER_MIN: f64 = 0.1;

/// The NTC's drift over the last [`REST_WINDOW_MS`] of `samples` (ms, C,
/// ascending), C per minute by least squares; None until the samples
/// span the window, less one poll: the last poll lands on its edge.
pub fn rest_drift(samples: &[(f64, f64)]) -> Option<f64> {
    let (first, last) = (samples.first()?, samples.last()?);
    if last.0 - first.0 < REST_WINDOW_MS - REST_POLL_MS {
        return None;
    }
    let late: Vec<&(f64, f64)> = samples
        .iter()
        .filter(|s| s.0 >= last.0 - REST_WINDOW_MS)
        .collect();
    let m = late.len() as f64;
    let tm = late.iter().map(|s| s.0).sum::<f64>() / m;
    let cm = late.iter().map(|s| s.1).sum::<f64>() / m;
    let sxx: f64 = late.iter().map(|s| (s.0 - tm).powi(2)).sum();
    if sxx <= 0.0 {
        return None;
    }
    let sxy: f64 = late.iter().map(|s| (s.0 - tm) * (s.1 - cm)).sum();
    Some(sxy / sxx * 60_000.0)
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
        assert_eq!(tau_shift(524), 15);
        assert_eq!(tau_shift(u16::MAX), 15);
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

    /// Board D's gains, q016.
    const Q_D: [u16; 2] = [719, 777];

    /// A synthetic hold from 0 to `hold_ms` under `ceiling`, seated from
    /// its first window, whose copper rises over a flat NTC by the gains
    /// `q` on the kernel's heats at (33, 262) s with the warm input fed
    /// back, exactly at every window: a window every [`WIN_MS`] through the
    /// current's floor and one at each later bin's centre, the current
    /// wobbling half a count. The applied duty is `duty(t)` of the command,
    /// the current `scale(t)` of [`I`], the winding `r0` cold times
    /// `r_of(t)`, the brush contact's own part of R.
    struct Synth {
        hold_ms: f64,
        q: [u16; 2],
        duty: fn(f64) -> f64,
        scale: fn(f64) -> f64,
        ceiling: f64,
        r0: f64,
        r_of: fn(f64) -> f64,
    }

    impl Synth {
        fn hold(hold_ms: f64) -> Self {
            Self {
                hold_ms,
                q: Q_D,
                duty: |_| 1.0,
                scale: |_| 1.0,
                ceiling: CEILING,
                r0: 1.0,
                r_of: |_| 1.0,
            }
        }

        fn build(&self) -> (Vec<HoldPoll>, Vec<ThermWindow>) {
            let mut polls = flat_polls(self.hold_ms, self.ceiling);
            for p in polls.iter_mut() {
                p.i_counts = if (0.0..self.hold_ms).contains(&p.t_ms) {
                    I * (self.scale)(p.t_ms)
                } else {
                    0.0
                };
            }
            let dense = DENSE_MS.min(self.hold_ms);
            let bins = (self.hold_ms / BIN_MS) as usize;
            let mut at: Vec<f64> = (0..(dense / WIN_MS) as usize)
                .map(|k| k as f64 * WIN_MS)
                .collect();
            at.extend(
                (0..bins)
                    .map(|j| (j as f64 + 0.5) * BIN_MS)
                    .filter(|&t| t >= dense),
            );
            let h = replay_two(&polls, [tau_shift(33), tau_shift(262)], self.q, &at);
            let windows = at
                .iter()
                .zip(&h)
                .enumerate()
                .map(|(k, (&t, h))| {
                    let x_w = (self.q[0] as f64 * h[0] + self.q[1] as f64 * h[1]) / 65536.0 / 100.0;
                    let wob = if t < dense { wobble(k) } else { 0.0 };
                    let d = (self.duty)(t);
                    ThermWindow {
                        commanded: d == 1.0,
                        ..tw(window_at(
                            t,
                            I * (self.scale)(t) + wob,
                            DUTY * d,
                            self.r0 * (self.r_of)(t),
                            x_w,
                        ))
                    }
                })
                .collect();
            (polls, windows)
        }

        fn fit(&self) -> Result<TwoNodeFit, String> {
            let (v, a) = scales();
            let (polls, windows) = self.build();
            fit_two_node(&polls, &windows, 0.0, f64::INFINITY, SEAT, v, a)
        }

        fn fit_pinned(&self, body: &NodeFit) -> Result<TwoNodeFit, String> {
            let (v, a) = scales();
            let (polls, windows) = self.build();
            fit_rotor_pinned(&[hold(&polls, &windows)], body, v, a)
        }
    }

    /// The body node a long hold wrote at `raw` q016 on 262 s, as the
    /// short holds take it.
    fn pinned_body(raw: u16) -> NodeFit {
        let k = raw as f64 / 65536.0;
        NodeFit {
            tau_s: 262,
            shift: tau_shift(262),
            k,
            rise_q016: enc(k, 65536.0),
            sse_ratio: 1.0,
            decided: true,
        }
    }

    /// A built Synth hold as the pooled fit takes it.
    fn hold<'a>(polls: &'a [HoldPoll], windows: &'a [ThermWindow]) -> Hold<'a> {
        Hold {
            polls,
            windows,
            t0_ms: 0.0,
            end_ms: f64::INFINITY,
            seat: SEAT,
        }
    }

    /// The rotor gain within `pct` percent of board D's.
    fn rotor_near(f: &TwoNodeFit, pct: f64) -> bool {
        (f.rotor.rise_q016.raw as f64 - Q_D[0] as f64).abs() <= Q_D[0] as f64 * pct / 100.0
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
            ..Synth::hold(120_000.0)
        };
        let f = s.fit().unwrap();
        let (v, a) = scales();
        let h = &f.holds[0];
        assert_eq!(h.r_ref_settle_ms, 200.0, "{f:?}");
        assert_eq!(h.fit_from_ms, FIT_FROM_MS);
        // the copper warms through the line: R_ref reads it cold within 0.3 C
        assert!((h.r_ref_ohm / (v / a) - 1.0).abs() < 1e-3, "{f:?}");
        assert!((1000..=1600).contains(&h.r_ref_windows), "{f:?}");
        assert!(rotor_near(&f, 5.0), "{f:?}");
    }

    /// A hold the limiter governs from its first window never reads the
    /// winding at its command: no reference, no rise.
    #[test]
    fn a_hold_never_at_its_command_is_refused() {
        let s = Synth {
            duty: |_| 0.9,
            ..Synth::hold(120_000.0)
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

    /// A hold whose cold current sits 7 % under the ceiling leaves the
    /// limiter no room: refused, named.
    #[test]
    fn a_hold_inside_the_ceiling_margin_is_refused() {
        let s = Synth {
            ceiling: 290.0,
            ..Synth::hold(120_000.0)
        };
        let why = s.fit().unwrap_err();
        assert!(
            why.contains("cold current 270 counts sits 6.9 % under the 290 counts current ceiling"),
            "{why}"
        );
        assert!(why.contains("inside the 10 % margin"), "{why}");
        let ok = Synth {
            ceiling: 301.0,
            ..Synth::hold(120_000.0)
        };
        let f = ok.fit().unwrap();
        let h = &f.holds[0];
        assert!((h.i_cold_counts - 270.0).abs() < 1e-3, "{f:?}");
        assert_eq!(h.ceiling_counts, 301.0);
        assert!((h.margin() - (1.0 - h.i_cold_counts / 301.0)).abs() < 1e-12);
    }

    /// A fold a minute into the hold, as a long tau lands it, moves the
    /// duty well after the reference: it stays at the hold's start.
    #[test]
    fn a_late_fold_leaves_the_reference_at_the_start() {
        let s = Synth {
            duty: |t| if t < 60_000.0 { 1.0 } else { 0.6 },
            scale: |t| if t < 60_000.0 { 1.0 } else { 0.6 },
            ..Synth::hold(240_000.0)
        };
        let f = s.fit().unwrap();
        let (v, a) = scales();
        let h = &f.holds[0];
        assert_eq!(h.r_ref_settle_ms, 0.0, "{f:?}");
        assert!(h.r_ref_start_ms < DIP_MS, "{f:?}");
        // the copper warms through the line: R_ref reads it cold within 0.3 C
        assert!((h.r_ref_ohm / (v / a) - 1.0).abs() < 1e-3, "{f:?}");
        assert!(rotor_near(&f, 5.0), "{f:?}");
    }

    /// A hold the limiter does not govern: its current falls 0.4 %/C as
    /// the copper rises to 15 C, slow at the reference's scale.
    #[test]
    fn a_current_that_drifts_with_the_copper_leaves_the_reference_at_the_start() {
        let s = Synth {
            q: [300, 0],
            scale: |t| 1.0 - 0.004 * 15.0 * (1.0 - (-t / 131_072.0).exp()),
            ..Synth::hold(480_000.0)
        };
        let f = s.fit().unwrap();
        let (v, a) = scales();
        let h = &f.holds[0];
        assert_eq!(h.r_ref_settle_ms, 0.0, "{f:?}");
        assert!(h.r_ref_start_ms < DIP_MS, "{f:?}");
        // the copper warms through the line: R_ref reads it cold within 0.3 C
        assert!((h.r_ref_ohm / (v / a) - 1.0).abs() < 1e-3, "{f:?}");
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
            fit_two_node(polls, windows, 0.0, f64::INFINITY, SEAT, v, a).unwrap_err()
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
    /// The replay is the kernel's loop: a 400-count hold on a 4 s leak
    /// (shift 8) with the rotor gain 1044 settles the heat and the warm
    /// input it feeds back at the kernel's own fixed point, the rise 28.31 C
    /// (`the_estimate_reads_the_ntc_plus_the_rise_on_the_heat`); the body
    /// node on the same leak carries the same heat; no gain, the cold square.
    #[test]
    fn the_replay_lands_on_the_kernels_fixed_point() {
        let polls: Vec<HoldPoll> = (0..2700)
            .map(|n| HoldPoll {
                t_ms: n as f64 * SLOW_PERIOD_MS as f64,
                i_counts: 400.0,
                t_ntc_cc: 2500.0,
                t_winding_cc: 2500.0,
                i_lim_counts: 1200.0,
            })
            .collect();
        let at = [2650.0 * SLOW_PERIOD_MS as f64];
        let [h1, h2] = replay_two(&polls, [8, 8], [1044, 0], &at)[0];
        assert_eq!(h1, h2);
        assert_eq!(rise_cc(h1 as u32, 1044), 2831, "{h1}");
        assert!(h1 > 160_000.0 * 1.1, "{h1}");
        let [c1, c2] = replay_two(&polls, [8, 8], [0, 0], &at)[0];
        assert!(
            c1 == c2 && (160_000.0 - 16.0..=160_000.0).contains(&c1),
            "{c1}"
        );
        assert_eq!(replay_two(&[], [8, 8], [0, 0], &at), [[0.0; 2]]);
    }

    /// Two unknowns held at or over zero: the free solution when it is,
    /// else the better single column.
    #[test]
    fn the_non_negative_solve_clamps_at_zero() {
        let x1: Vec<f64> = (0..50).map(|j| j as f64).collect();
        let x2: Vec<f64> = (0..50).map(|j| (j as f64).sqrt()).collect();
        let y: Vec<f64> = x1.iter().zip(&x2).map(|(a, b)| 2.0 * a + 0.5 * b).collect();
        let k = nnls2(&x1, &x2, &y);
        assert!(
            (k[0] - 2.0).abs() < 1e-9 && (k[1] - 0.5).abs() < 1e-9,
            "{k:?}"
        );
        let y: Vec<f64> = x1.iter().zip(&x2).map(|(a, b)| 2.0 * a - 0.5 * b).collect();
        let k = nnls2(&x1, &x2, &y);
        assert!(k[1] == 0.0 && (1.5..2.0).contains(&k[0]), "{k:?}");
        let y: Vec<f64> = x1.iter().map(|a| -a).collect();
        assert_eq!(nnls2(&x1, &x2, &y), [0.0, 0.0]);
        assert_eq!(nnls2(&[0.0; 3], &[0.0; 3], &[1.0; 3]), [0.0, 0.0]);
    }

    /// Board D's gains on a 960 s hold: both read back within 1 %, the pair
    /// (33, 262) s decided on both octaves, the residual under 0.05 C, the
    /// brush window empty of anything but the model.
    #[test]
    fn recovers_two_gains_and_their_octaves_from_a_long_hold() {
        let f = Synth::hold(960_000.0).fit().unwrap();
        assert!((f.rotor.rise_q016.raw as i32 - 719).abs() <= 7, "{f:?}");
        assert!((f.body.rise_q016.raw as i32 - 777).abs() <= 8, "{f:?}");
        assert_eq!((f.rotor.tau_s, f.rotor.shift), (33, 11));
        assert_eq!((f.body.tau_s, f.body.shift), (262, 14));
        assert!(f.rotor.decided && f.body.decided, "{f:?}");
        assert!(f.rotor.sse_ratio > 2.0 && f.body.sse_ratio > 2.0, "{f:?}");
        assert!(f.residual_sd_c < 0.05, "{f:?}");
        assert_eq!(f.holds.len(), 1);
        let h = &f.holds[0];
        assert_eq!(h.fit_from_ms, FIT_FROM_MS);
        assert!(h.first_c.abs() < 0.1, "{f:?}");
        assert!(h.block_means_c.iter().all(|m| m.abs() < 0.05), "{f:?}");
        assert_eq!(h.block_means_c.len(), 16, "{f:?}");
        assert!((h.hold_s - 960.0).abs() <= 1.0, "{f:?}");
        assert_eq!((f.bins, h.bins), (940, 940));
        assert!(f.passes <= PASSES_MAX && f.passes >= 2, "{f:?}");
        assert!((f.rotor.octave_s() - 32.768).abs() < 1e-9);
        assert!((h.t0_c - T_NTC0).abs() < 1e-9);
        assert!(h.settled_rise_c > 15.0, "{f:?}");
        // the fixed point at the held current reads the settled rise
        assert!((f.plateau_c(270.0) - h.settled_rise_c).abs() < 0.5, "{f:?}");
        // the polls' estimate sat at the NTC: under the copper by its rise
        assert!(
            (h.estimate_err_c.unwrap() + h.settled_rise_c).abs() < 1e-6,
            "{f:?}"
        );
    }

    /// A rotor alone: the body gain reads 0 on either body octave, so the
    /// SSE decides nothing there and the hold's length does: 960 s covers
    /// 524 s and writes it, 480 s covers 262 s alone. The rotor still
    /// decides itself.
    #[test]
    fn an_undecided_body_writes_the_octave_the_hold_covers() {
        let f = Synth {
            q: [719, 0],
            ..Synth::hold(960_000.0)
        }
        .fit()
        .unwrap();
        assert!((f.rotor.rise_q016.raw as i32 - 719).abs() <= 7, "{f:?}");
        assert_eq!(f.body.rise_q016.raw, 0, "{f:?}");
        assert!(f.rotor.decided && !f.body.decided, "{f:?}");
        assert!(f.body.sse_ratio < OCTAVE_DECIDES, "{f:?}");
        assert_eq!((f.body.tau_s, f.body.shift), (524, 15));
        let f = Synth {
            q: [719, 0],
            ..Synth::hold(480_000.0)
        }
        .fit()
        .unwrap();
        assert!(!f.body.decided, "{f:?}");
        assert_eq!(f.body.tau_s, 262);
        assert_eq!(f.rotor.tau_s, 33);
    }

    /// A hold too short to carry 30 fitted bins past the brush window is
    /// refused, named.
    #[test]
    fn a_short_hold_has_too_few_bins_to_fit() {
        let (v, a) = scales();
        let (polls, windows) = Synth::hold(45_000.0).build();
        let why = fit_two_node(&polls, &windows, 0.0, f64::INFINITY, SEAT, v, a).unwrap_err();
        assert!(why.contains("past 20 s, under the 30"), "{why}");
    }

    /// One hold pooled is the single fit, bit for bit.
    #[test]
    fn a_pool_of_one_is_the_single_fit() {
        let (v, a) = scales();
        let (polls, windows) = Synth::hold(480_000.0).build();
        let one = fit_two_node(&polls, &windows, 0.0, f64::INFINITY, SEAT, v, a).unwrap();
        let pooled = fit_two_node_pooled(&[hold(&polls, &windows)], v, a).unwrap();
        assert_eq!(pooled, one);
        assert_eq!(one.holds.len(), 1);
    }

    /// Two holds of one winding in two seats, the second's brush contact 5
    /// % higher: pooled, one pair of gains at board D's, each hold its own
    /// R_ref, the bins every hold's together.
    #[test]
    fn two_seats_pool_to_one_pair_of_gains() {
        let (v, a) = scales();
        let (pa, wa) = Synth::hold(960_000.0).build();
        let (pb, wb) = Synth {
            r0: 1.05,
            ..Synth::hold(960_000.0)
        }
        .build();
        let f = fit_two_node_pooled(&[hold(&pa, &wa), hold(&pb, &wb)], v, a).unwrap();
        // one Q0.16 step is 1.4e-3 of the rotor gain: the bound is a step
        for (n, q) in [(&f.rotor, Q_D[0]), (&f.body, Q_D[1])] {
            assert!(n.rise_q016.raw.abs_diff(q) <= 1, "{n:?} against {q}");
        }
        assert_eq!(f.holds.len(), 2);
        assert!(
            f.holds.iter().all(|h| h.fit_from_ms == FIT_FROM_MS),
            "{f:?}"
        );
        let (ra, rb) = (f.holds[0].r_ref_ohm, f.holds[1].r_ref_ohm);
        assert!((rb / ra - 1.05).abs() < 1e-3, "{ra} {rb}");
        assert_eq!(f.bins, f.holds[0].bins + f.holds[1].bins);
        assert!(f.bins > f.holds[0].bins, "{f:?}");
    }

    /// A rotor alone in a 960 s hold pooled with a 480 s one: the body
    /// stays undecided and writes the octave the SHORTEST hold covers, 262
    /// s, where the 960 s hold alone wrote 524.
    #[test]
    fn the_undecided_body_covers_the_shortest_hold() {
        let (v, a) = scales();
        let long = Synth {
            q: [719, 0],
            ..Synth::hold(960_000.0)
        };
        let short = Synth {
            q: [719, 0],
            ..Synth::hold(480_000.0)
        };
        assert_eq!(long.fit().unwrap().body.tau_s, 524);
        let (pl, wl) = long.build();
        let (ps, ws) = short.build();
        let f = fit_two_node_pooled(&[hold(&pl, &wl), hold(&ps, &ws)], v, a).unwrap();
        assert!(!f.body.decided, "{f:?}");
        assert_eq!(f.body.tau_s, 262);
        assert_eq!(f.rotor.tau_s, 33);
    }

    /// A hold too short to fit fails the pool, named by its place; no hold
    /// at all is its own refusal.
    #[test]
    fn a_short_hold_in_the_pool_is_named() {
        let (v, a) = scales();
        let (pa, wa) = Synth::hold(480_000.0).build();
        let (pb, wb) = Synth::hold(40_000.0).build();
        let why = fit_two_node_pooled(&[hold(&pa, &wa), hold(&pb, &wb)], v, a).unwrap_err();
        assert!(why.starts_with("hold 2: "), "{why}");
        assert!(why.contains("past 20 s, under the 30"), "{why}");
        assert_eq!(
            fit_two_node_pooled(&[], v, a).unwrap_err(),
            "no hold to fit"
        );
    }

    /// A brush contact transient: R up `frac` from `from_ms`, falling back
    /// along a half cosine to its seat by `to_ms`.
    fn transient(t: f64, from_ms: f64, to_ms: f64, frac: f64) -> f64 {
        if t < from_ms || t >= to_ms {
            1.0
        } else {
            1.0 + frac
                * 0.5
                * (1.0 + (std::f64::consts::PI * (t - from_ms) / (to_ms - from_ms)).cos())
        }
    }

    /// Both gains within `pct` percent of board D's.
    fn gains_near(f: &TwoNodeFit, pct: f64) -> bool {
        rotor_near(f, pct)
            && (f.body.rise_q016.raw as f64 - Q_D[1] as f64).abs() <= Q_D[1] as f64 * pct / 100.0
    }

    /// A 480 s hold whose contact adds 1.5 % of R from 5 s and settles by
    /// 15 s: inside the first blank, so the fit reads from 20 s, both gains
    /// within 2 %, and the brush window before the blank reads the
    /// transient.
    #[test]
    fn a_contact_transient_inside_the_blank_is_not_fitted() {
        let f = Synth {
            r_of: |t| transient(t, 5_000.0, 15_000.0, 0.015),
            ..Synth::hold(480_000.0)
        }
        .fit()
        .unwrap();
        assert!(gains_near(&f, 2.0), "{f:?}");
        let h = &f.holds[0];
        assert_eq!(h.fit_from_ms, FIT_FROM_MS, "{f:?}");
        assert!(h.first_c > 0.5, "{f:?}");
    }

    /// The same transient stretched to settle by 35 s runs into the first
    /// fitted block: the fit moves its blank out to 40 s and reads the
    /// rotor within 3 % from there.
    #[test]
    fn an_unsettled_transient_moves_the_blank_out() {
        let f = Synth {
            r_of: |t| transient(t, 5_000.0, 35_000.0, 0.015),
            ..Synth::hold(480_000.0)
        }
        .fit()
        .unwrap();
        assert_eq!(f.holds[0].fit_from_ms, FIT_FROM_MS + FLAT_BLOCK_MS, "{f:?}");
        assert!(rotor_near(&f, 3.0), "{f:?}");
    }

    /// A transient still decaying at 90 s is not flat at any blank: the
    /// fit refuses at the maximum, named.
    #[test]
    fn a_transient_still_moving_at_the_maximum_blank_refuses() {
        let why = Synth {
            r_of: |t| transient(t, 5_000.0, 180_000.0, 0.015),
            ..Synth::hold(480_000.0)
        }
        .fit()
        .unwrap_err();
        assert!(why.contains("the contact had not settled"), "{why}");
        assert!(why.contains("60 s in"), "{why}");
        assert!(!why.starts_with("hold "), "{why}");
    }

    /// Two holds pooled, one clean and one with the 35 s transient: each
    /// hold's blank is its own, the clean one's at 20 s and the other's at
    /// 40 s, and the pooled gains read within 3 %.
    #[test]
    fn the_blank_is_judged_per_hold_in_a_pool() {
        let (v, a) = scales();
        let (pa, wa) = Synth::hold(480_000.0).build();
        let (pb, wb) = Synth {
            r_of: |t| transient(t, 5_000.0, 35_000.0, 0.015),
            ..Synth::hold(480_000.0)
        }
        .build();
        let f = fit_two_node_pooled(&[hold(&pa, &wa), hold(&pb, &wb)], v, a).unwrap();
        assert_eq!(f.holds[0].fit_from_ms, FIT_FROM_MS, "{f:?}");
        assert_eq!(f.holds[1].fit_from_ms, FIT_FROM_MS + FLAT_BLOCK_MS, "{f:?}");
        assert!(gains_near(&f, 3.0), "{f:?}");
        assert_eq!(f.bins, f.holds[0].bins + f.holds[1].bins);
        assert!(f.holds[0].bins > f.holds[1].bins, "{f:?}");
    }

    /// A 120 s hold with the body pinned from the long hold reads the
    /// rotor within 1 % on its own octave, decided, and writes the pin as
    /// its body.
    #[test]
    fn a_short_hold_with_the_body_pinned_reads_the_rotor() {
        let body = pinned_body(Q_D[1]);
        let f = Synth::hold(120_000.0).fit_pinned(&body).unwrap();
        assert!(rotor_near(&f, 1.0), "{f:?}");
        assert_eq!((f.rotor.tau_s, f.rotor.shift), (33, 11));
        assert!(f.rotor.decided, "{f:?}");
        assert_eq!(f.body, body);
        assert_eq!(f.holds.len(), 1);
        assert_eq!(f.holds[0].fit_from_ms, FIT_FROM_MS, "{f:?}");
        assert!(f.residual_sd_c < 0.05, "{f:?}");
    }

    /// The 1.5 % brush transient settling by 35 s on a 120 s hold under the
    /// pinned fit: its tail is most of the short hold's residual, and the
    /// judge still reads the block against the bins after it, moves the
    /// blank to 40 s and reads the rotor within 1 % from there.
    #[test]
    fn a_short_hold_transient_moves_the_blank_under_the_pinned_fit() {
        let f = Synth {
            r_of: |t| transient(t, 5_000.0, 35_000.0, 0.015),
            ..Synth::hold(120_000.0)
        }
        .fit_pinned(&pinned_body(Q_D[1]))
        .unwrap();
        assert_eq!(
            f.holds[0].fit_from_ms,
            FIT_FROM_MS + FLAT_BLOCK_MS,
            "the block read about +0.50 C against the 0.10 C floor: {f:?}"
        );
        assert!(rotor_near(&f, 1.0), "{f:?}");
        assert!(f.holds[0].first_c > 1.0, "{f:?}");
    }

    /// The windows of a hold from `t0_ms` for `hold_ms` in a record whose
    /// copper the gains `q` warm over the whole of `polls`: dense through
    /// the current's floor, then one per bin, as [`Synth::build`] lays
    /// them, the copper read exactly at each from the kernel's replay over
    /// the record from its first poll.
    fn hold_in_record(
        polls: &[HoldPoll],
        t0_ms: f64,
        hold_ms: f64,
        q: [u16; 2],
    ) -> Vec<ThermWindow> {
        let dense = DENSE_MS.min(hold_ms);
        let bins = (hold_ms / BIN_MS) as usize;
        let mut at: Vec<f64> = (0..(dense / WIN_MS) as usize)
            .map(|k| t0_ms + k as f64 * WIN_MS)
            .collect();
        at.extend(
            (0..bins)
                .map(|j| t0_ms + (j as f64 + 0.5) * BIN_MS)
                .filter(|&t| t >= t0_ms + dense),
        );
        let h = replay_two(polls, [tau_shift(33), tau_shift(262)], q, &at);
        at.iter()
            .zip(&h)
            .enumerate()
            .map(|(k, (&t, h))| {
                let x_w = (q[0] as f64 * h[0] + q[1] as f64 * h[1]) / 65536.0 / 100.0;
                let wob = if t < t0_ms + dense { wobble(k) } else { 0.0 };
                tw(window_at(t, I + wob, DUTY, 1.0, x_w))
            })
            .collect()
    }

    /// Two 120 s holds at board D's gains in one record, a 10 s rest of
    /// zero current between them: hold B's reference is read with the
    /// copper still carrying hold A's rise, and the pinned fit of B alone
    /// over the whole record's polls converts its resistance on the
    /// copper's own temperature there, reading the rotor within 1 % and
    /// the warmth at the reference within 0.2 C of the planted.
    #[test]
    fn a_warm_start_reads_the_same_rotor() {
        const A: (f64, f64) = (0.0, 120_000.0);
        const B: (f64, f64) = (130_000.0, 250_000.0);
        let mut polls = flat_polls(B.1, CEILING);
        for p in polls.iter_mut() {
            let held = (A.0..A.1).contains(&p.t_ms) || (B.0..B.1).contains(&p.t_ms);
            p.i_counts = if held { I } else { 0.0 };
        }
        let mut windows = hold_in_record(&polls, A.0, A.1 - A.0, Q_D);
        windows.extend(hold_in_record(&polls, B.0, B.1 - B.0, Q_D));
        let (v, a) = scales();
        let hold_b = Hold {
            polls: &polls,
            windows: &windows,
            t0_ms: B.0,
            end_ms: f64::INFINITY,
            seat: SEAT,
        };
        let f = fit_rotor_pinned(&[hold_b], &pinned_body(Q_D[1]), v, a).unwrap();
        let h = &f.holds[0];
        let at_ref = replay_two(
            &polls,
            [tau_shift(33), tau_shift(262)],
            Q_D,
            &[B.0 + h.r_ref_start_ms],
        )[0];
        let planted_warm =
            (Q_D[0] as f64 * at_ref[0] + Q_D[1] as f64 * at_ref[1]) / 65536.0 / 100.0;
        assert!(
            planted_warm > 4.0,
            "the copper carried {planted_warm:.2} C into hold B"
        );
        let off = (f.rotor.rise_q016.raw as f64 / Q_D[0] as f64 - 1.0) * 100.0;
        assert!(
            rotor_near(&f, 1.0),
            "the rotor read {off:+.1} % off the planted (-5.6 % converted on the NTC's \
             temperature, within 1 % on the copper's own): {f:?}"
        );
        assert!(
            (h.warm_ref_c - planted_warm).abs() < 0.2,
            "warm_ref_c {:.2} C against {planted_warm:.2} C planted",
            h.warm_ref_c
        );
        assert!(h.settled_rise_c > planted_warm, "{h:?}");
    }

    /// The body pinned 20 % high moves the 120 s rotor by the body's share
    /// of the rise there, under 8 %.
    #[test]
    fn a_wrong_body_moves_the_short_rotor_by_its_share() {
        let high = pinned_body((Q_D[1] as f64 * 1.2).round() as u16);
        let f = Synth::hold(120_000.0).fit_pinned(&high).unwrap();
        let moved = (f.rotor.rise_q016.raw as f64 / Q_D[0] as f64 - 1.0) * 100.0;
        assert!(
            moved.abs() < 8.0,
            "the rotor moved {moved:+.1} %, about -6 % on a clean hold: {f:?}"
        );
    }

    /// The free fit's unsettled transient under the pinned fit: the blank
    /// moves out to 40 s there too and the rotor reads within 1 % from it.
    #[test]
    fn a_pinned_fit_keeps_the_blank_check() {
        let f = Synth {
            r_of: |t| transient(t, 5_000.0, 35_000.0, 0.015),
            ..Synth::hold(480_000.0)
        }
        .fit_pinned(&pinned_body(Q_D[1]))
        .unwrap();
        assert_eq!(f.holds[0].fit_from_ms, FIT_FROM_MS + FLAT_BLOCK_MS, "{f:?}");
        assert!(rotor_near(&f, 1.0), "{f:?}");
    }

    /// Five 120 s holds at five seats, the seat's drift planted as the
    /// rotor gain -5, -2.5, 0, +2.5, +5 % about board D's: the pool's
    /// spread reads the mean and the planted sd back, the mean's error as
    /// degrees at the ceiling, and no split between the two stops when
    /// each stop's holds average to the same gain; one stop alone has no
    /// split, and under three holds no spread.
    #[test]
    fn the_pool_spread_reads_the_seats() {
        let body = pinned_body(Q_D[1]);
        let qs: [u16; 5] = [683, 701, 719, 737, 755];
        let singles: Vec<TwoNodeFit> = qs
            .iter()
            .map(|&q| {
                Synth {
                    q: [q, Q_D[1]],
                    ..Synth::hold(120_000.0)
                }
                .fit_pinned(&body)
                .unwrap()
            })
            .collect();
        let s = pool_spread(&singles, CEILING, &[-1, 1, -1, 1, -1]).unwrap();
        assert_eq!(s.n, 5);
        assert!(
            (s.k_mean / (Q_D[0] as f64 / 65536.0) - 1.0).abs() < 0.005,
            "{s:?}"
        );
        let planted: Vec<f64> = qs.iter().map(|&q| q as f64 / 65536.0).collect();
        assert!((s.k_sd / sd(&planted) - 1.0).abs() < 0.2, "{s:?}");
        let at = |k: f64| {
            let mut f = singles[0].clone();
            f.rotor.k = k;
            f.plateau_c(CEILING)
        };
        let se_c = at(s.k_mean + s.k_sd / 5f64.sqrt()) - at(s.k_mean);
        assert!((s.se_c_at_ceiling - se_c).abs() < 1e-9, "{s:?}");
        assert!((0.3..0.6).contains(&s.se_c_at_ceiling), "{s:?}");
        let split = s.split_c.unwrap();
        assert!(
            split.abs() < 0.1,
            "the stops split {split:+.3} C, about 0.000 C on clean holds: {s:?}"
        );
        assert_eq!(
            pool_spread(&singles, CEILING, &[1; 5]).unwrap().split_c,
            None
        );
        assert_eq!(pool_spread(&singles[..2], CEILING, &[-1, 1]), None);
    }

    /// A +0.2 C bump over the hold's first fitted 20 s, the copper the
    /// fit's blank starts on.
    fn bump_c(t: f64) -> f64 {
        if (FIT_FROM_MS..FIT_FROM_MS + FLAT_BLOCK_MS).contains(&t) {
            0.2
        } else {
            0.0
        }
    }

    /// A per-second scatter of sd 0.6 C on the copper reading past the
    /// reference, a hash of the bin index.
    fn scatter_c(t: f64) -> f64 {
        if t < R_REF_WITHIN_MS {
            return 0.0;
        }
        let mut x = (t / BIN_MS) as u64 ^ 0x9e37_79b9_7f4a_7c15;
        x ^= x >> 33;
        x = x.wrapping_mul(0xff51_afd7_ed55_8ccd);
        x ^= x >> 33;
        x = x.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
        x ^= x >> 33;
        0.6 * 3f64.sqrt() * ((x >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0)
    }

    /// The flat judge reads the hold's own scatter: a +0.2 C bump over the
    /// first fitted block sits inside the sigma bound of a hold scattered
    /// 0.6 C per bin and the fit reads from 20 s, while the same bump on a
    /// noiseless hold is over [`FLAT_MIN_C`] and moves the blank to 40 s.
    #[test]
    fn a_noisy_hold_is_judged_by_its_own_scatter() {
        let noisy = Synth {
            r_of: |t| 1.0 + (bump_c(t) + scatter_c(t)) / (COPPER_ZERO_R_C + T_NTC0),
            ..Synth::hold(480_000.0)
        }
        .fit()
        .unwrap();
        assert_eq!(
            noisy.holds[0].fit_from_ms, FIT_FROM_MS,
            "the block read about +0.20 C against a bound of about 0.30 C: {noisy:?}"
        );
        assert!(noisy.residual_sd_c > 0.4, "{noisy:?}");
        assert!(rotor_near(&noisy, 5.0), "{noisy:?}");
        let quiet = Synth {
            r_of: |t| 1.0 + bump_c(t) / (COPPER_ZERO_R_C + T_NTC0),
            ..Synth::hold(480_000.0)
        }
        .fit()
        .unwrap();
        assert_eq!(
            quiet.holds[0].fit_from_ms,
            FIT_FROM_MS + FLAT_BLOCK_MS,
            "the block read about +0.18 C against the 0.10 C floor: {quiet:?}"
        );
        assert!(quiet.holds[0].first_c.abs() < 0.1, "{quiet:?}");
    }

    /// The rest judge: a ramp reads its slope once the samples span the
    /// window, a flat line 0, fewer None, and a long record its last 60 s
    /// alone.
    #[test]
    fn rest_drift_reads_a_ramp() {
        let ramp = |n: usize, c_per_min: f64| -> Vec<(f64, f64)> {
            (0..n)
                .map(|k| (k as f64 * REST_POLL_MS, c_per_min * k as f64 / 60.0))
                .collect()
        };
        let d = rest_drift(&ramp(61, 0.2)).unwrap();
        assert!((d - 0.2).abs() < 1e-9, "{d}");
        assert_eq!(rest_drift(&ramp(61, 0.0)), Some(0.0));
        assert_eq!(rest_drift(&ramp(59, 0.2)), None);
        assert_eq!(rest_drift(&[]), None);
        let bent: Vec<(f64, f64)> = (0..=120)
            .map(|k| {
                let t = k as f64;
                (t * 1000.0, 0.5 * (t - 60.0).max(0.0) / 60.0)
            })
            .collect();
        let d = rest_drift(&bent).unwrap();
        assert!((d - 0.5).abs() < 1e-9, "{d}");
    }

    /// The live hold's reference and copper: R_ref and T_0 as the fit reads
    /// them, the copper over the last second of windows from them.
    #[test]
    fn the_live_hold_reads_its_copper_against_its_own_reference() {
        let (polls, windows) = Synth::hold(120_000.0).build();
        let (r_ref, t0_c) = hold_reference(&polls, &windows, 0.0, SEAT).unwrap();
        assert!((t0_c - T_NTC0).abs() < 1e-9);
        assert!((r_ref - 1.0).abs() < 1e-3, "{r_ref}");
        let f = fit_two_node(&polls, &windows, 0.0, f64::INFINITY, SEAT, 1.0, 1.0).unwrap();
        let h = &f.holds[0];
        assert!((h.r_ref_ohm - r_ref).abs() < 1e-12);
        let c = copper_c(&windows, 119_000.0, r_ref, t0_c).unwrap();
        assert!(
            (c - (T_NTC0 + h.settled_rise_c)).abs() < 0.2,
            "{c} vs {}",
            h.settled_rise_c
        );
        assert_eq!(copper_c(&windows, 1e9, r_ref, t0_c), None);
    }
}
