//! The winding's current rise from a shunt burst: the stage 3 inductance
//! notebook's period-pair fit (oscnb `bursts.period_table`, the notebook's
//! `fit`), ported.
//!
//! The burst's samples are folded onto the PWM period. Each full period
//! after the step gives its window current `i_n` (a line through the
//! window's settled shunt samples, read between the bridge's two edges)
//! and its applied volts `V_n` (the tap difference over the window and
//! over the brake interval, weighted by their durations). With the rotor
//! still, `v = R i + L di/dt`, and over one period from window `n` to
//! window `n + 1`:
//!
//! `L (i_{n+1} - i_n) = T (V_n - R (i_n + i_{n+1}) / 2)`,
//!
//! linear in R and L: one least squares over the burst's pairs gives both,
//! and tau = L / R. [`pooled`] runs the same over several bursts with an
//! intercept, so a constant volts error does not pose as resistance.

use crate::capture::burst::Capture;

use super::{lstsq, mean, median};

/// Timer ticks per ADC clock: the ADC runs at half the 48 MHz timer.
pub const TICKS_PER_ADC_CLK: f64 = 2.0;
/// ADC clocks from a conversion's start to where its sample closes.
pub const APERTURE_CLK: f64 = 13.5;
/// ADC clocks from one sample to the next.
pub const CONVERSION_CLK: f64 = 26.0;
pub const STEP_TICKS: f64 = CONVERSION_CLK * TICKS_PER_ADC_CLK;
pub const TICKS_PER_US: f64 = 48.0;
/// The brake read stops this far before the window, ticks.
pub const BRAKE_GUARD_TICKS: f64 = 10.0;
/// Samples before the step kept out of the zero.
pub const PRE_GUARD: usize = 8;

/// The bridge's slow-decay edges in timer ticks: where the current and the
/// driven tap turn on after the commanded edge, where they turn off after
/// it, and how long the shunt takes to settle from the on edge.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Lags {
    pub on: f64,
    pub tap_on: f64,
    pub off: f64,
    pub settle: f64,
}

/// The conversions a burst is read through.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Scales {
    pub counts_per_a: f64,
    /// Counts of tap difference a volt, and the difference at zero volts.
    pub terminal_counts_per_v: f64,
    pub terminal_offset_counts: f64,
    /// The window's own `V_on / i` over the 100 % one: R and L are divided
    /// by it, tau keeps none of it.
    pub window_scale: f64,
    pub lags: Lags,
}

/// One full period after the step.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Period {
    pub n: i64,
    /// The window current, A.
    pub i_a: f64,
    /// The volts over the whole period, V.
    pub v_v: f64,
    /// The volts inside the window and over the brake interval, V.
    pub v_on_v: f64,
    pub v_off_v: f64,
    /// The shunt between windows, A.
    pub i_off_a: f64,
    /// The window's settled shunt samples.
    pub on_n: usize,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Stream {
    Shunt,
    TapA,
    TapB,
    None,
}

struct Sample {
    stream: Stream,
    code: f64,
    tc: f64,
    n: i64,
    full: bool,
}

/// The PWM period of a counter that crests at `pwm_arr`, s: up and down.
pub fn pwm_period_s(pwm_arr: u16) -> f64 {
    2.0 * pwm_arr as f64 / TICKS_PER_US * 1e-6
}

/// The drive window's half-width in timer ticks for a Q15 duty.
pub fn half_ticks(q15: i16, pwm_arr: u16) -> f64 {
    ((q15.unsigned_abs() as i64 * pwm_arr as i64 + (1 << 14)) >> 15) as f64
}

fn stream_of(k: usize, frame_len: u8, chans: u8) -> Stream {
    if k % 2 == 0 {
        return Stream::Shunt;
    }
    if frame_len == 2 {
        if chans & 1 != 0 {
            Stream::TapA
        } else if chans & 2 != 0 {
            Stream::TapB
        } else {
            Stream::None
        }
    } else if k % 4 == 1 {
        Stream::TapA
    } else {
        Stream::TapB
    }
}

/// Every sample on the grid of full windows centered on the crest.
fn samples(cap: &Capture) -> (Vec<Sample>, f64) {
    let m = &cap.meta;
    let arr = m.pwm_arr as f64;
    let period = 2.0 * arr;
    let half = half_ticks(m.step_q15, m.pwm_arr);
    let phi0 = if m.start_dir == 0 {
        m.start_cnt as f64
    } else {
        period - m.start_cnt as f64
    };
    let tick = |k: usize| phi0 + STEP_TICKS * k as f64 + APERTURE_CLK * TICKS_PER_ADC_CLK;
    // the duty written at the step takes effect at the next trough: the
    // grid starts at the crest window after it
    let trough = period * (tick(m.step_index as usize) / period).ceil();
    let origin = trough + arr - half;
    let n_of = |t: f64| ((t - origin) / period).floor() as i64;
    let last = n_of(tick(cap.samples.len().saturating_sub(1)));
    let out = cap
        .samples
        .iter()
        .enumerate()
        .map(|(k, &code)| {
            let t = tick(k);
            let n = n_of(t);
            Sample {
                stream: stream_of(k, m.frame_len, m.chans),
                code: code as f64,
                tc: ((t - arr).rem_euclid(period) + (period / 2.0).floor()).rem_euclid(period)
                    - (period / 2.0).floor(),
                n,
                full: n < last,
            }
        })
        .collect();
    (out, half)
}

/// The shunt's level before the step: the amplifier's zero for this burst.
pub fn zero(cap: &Capture) -> Option<f64> {
    let pre = (cap.meta.step_index as usize).saturating_sub(PRE_GUARD);
    let v: Vec<f64> = cap
        .samples
        .iter()
        .enumerate()
        .filter(|(k, _)| k % 2 == 0 && *k < pre)
        .map(|(_, &c)| c as f64)
        .collect();
    median(&v)
}

fn line(x: &[f64], y: &[f64]) -> Option<(f64, f64)> {
    let rows: Vec<[f64; 2]> = x.iter().map(|&t| [t, 1.0]).collect();
    lstsq(&rows, y).map(|[m, c]| (m, c))
}

/// One row a full period of the burst. `sign` is the bridge's sign in the
/// wiring's frame (+1 drives tap A); `edge_volt_ticks` is what the two
/// edges hold beyond the two-part mean, 0 where unmeasured.
pub fn periods(
    cap: &Capture,
    sign: f64,
    s: &Scales,
    edge_volt_ticks: f64,
) -> Result<Vec<Period>, String> {
    let (all, half) = samples(cap);
    let zero = zero(cap).ok_or("no shunt samples before the step")?;
    let period = 2.0 * cap.meta.pwm_arr as f64;
    let l = &s.lags;
    let (a, b) = (-half + l.tap_on, half + l.off);
    let on_span = b - a;
    let settled = |tc: f64| tc >= -half + l.settle && tc <= half;
    let clear = |tc: f64| tc >= b + l.settle - l.on || tc <= -half - BRAKE_GUARD_TICKS;
    let mut ns: Vec<i64> = all
        .iter()
        .filter(|x| x.n >= 0 && x.full)
        .map(|x| x.n)
        .collect();
    ns.dedup();
    if ns.is_empty() {
        return Err("no full period after the step".into());
    }
    let mut out = Vec::with_capacity(ns.len());
    for n in ns {
        let g: Vec<&Sample> = all.iter().filter(|x| x.n == n && x.full).collect();
        let pick = |st: Stream, w: &dyn Fn(f64) -> bool| -> Vec<&Sample> {
            g.iter()
                .copied()
                .filter(|x| x.stream == st && w(x.tc))
                .collect()
        };
        let sh = pick(Stream::Shunt, &settled);
        let (tc, y): (Vec<f64>, Vec<f64>) = sh.iter().map(|x| (x.tc, x.code - zero)).unzip();
        let (m, c) = line(&tc, &y).ok_or_else(|| {
            format!(
                "period {n}: {} settled shunt samples, a line needs two",
                sh.len()
            )
        })?;
        let i_on = (m * (a + b) / 2.0 + c) / s.counts_per_a;
        let off: Vec<f64> = pick(Stream::Shunt, &clear)
            .iter()
            .map(|x| x.code - zero)
            .collect();
        let i_off = mean(&off).unwrap_or(f64::NAN) / s.counts_per_a;
        let volts = |w: &dyn Fn(f64) -> bool| -> Result<f64, String> {
            let ta: Vec<f64> = pick(Stream::TapA, w).iter().map(|x| x.code).collect();
            let tb: Vec<f64> = pick(Stream::TapB, w).iter().map(|x| x.code).collect();
            match (mean(&ta), mean(&tb)) {
                (Some(ta), Some(tb)) => {
                    Ok((sign * (ta - tb) - s.terminal_offset_counts) / s.terminal_counts_per_v)
                }
                _ => Err(format!("period {n}: no tap pair to read volts by")),
            }
        };
        let v_on = volts(&settled)?;
        let v_off = volts(&clear)?;
        let v_mean = (v_on * on_span + v_off * (period - on_span) + edge_volt_ticks) / period;
        out.push(Period {
            n,
            i_a: i_on,
            v_v: v_mean,
            v_on_v: v_on,
            v_off_v: v_off,
            i_off_a: i_off,
            on_n: sh.len(),
        });
    }
    Ok(out)
}

/// The period-pair fit's result.
#[derive(Clone, Debug, PartialEq)]
pub struct Rise {
    pub r_ohm: f64,
    pub l_h: f64,
    pub tau_s: f64,
    /// The volts at zero current, V: 0 in a one-burst fit.
    pub v0_v: f64,
    pub pairs: usize,
    /// The volts each pair leaves, V.
    pub resid_v: Vec<f64>,
}

impl Rise {
    pub fn resid_rms_v(&self) -> f64 {
        mean(&self.resid_v.iter().map(|r| r * r).collect::<Vec<_>>())
            .unwrap_or(f64::NAN)
            .sqrt()
    }
}

fn pairs(t: &[Period], period_s: f64) -> impl Iterator<Item = ([f64; 2], f64)> + '_ {
    t.windows(2).map(move |w| {
        let im = (w[0].i_a + w[1].i_a) / 2.0;
        ([im, (w[1].i_a - w[0].i_a) / period_s], w[0].v_v)
    })
}

/// R and L of one burst over every pair, each divided by the window's
/// scale; `period_s` is the PWM period.
pub fn fit(t: &[Period], period_s: f64, window_scale: f64) -> Option<Rise> {
    let (x, y): (Vec<[f64; 2]>, Vec<f64>) = pairs(t, period_s).unzip();
    if x.len() < 2 {
        return None;
    }
    let [r, l] = lstsq(&x, &y)?;
    let resid = x
        .iter()
        .zip(&y)
        .map(|(row, v)| v - (r * row[0] + l * row[1]))
        .collect();
    Some(Rise {
        r_ohm: r / window_scale,
        l_h: l / window_scale,
        tau_s: l / r,
        v0_v: 0.0,
        pairs: x.len(),
        resid_v: resid,
    })
}

/// R, L and an intercept over the pairs of several bursts at once.
pub fn pooled(tables: &[&[Period]], period_s: f64, window_scale: f64) -> Option<Rise> {
    let (x, y): (Vec<[f64; 3]>, Vec<f64>) = tables
        .iter()
        .flat_map(|t| pairs(t, period_s))
        .map(|([im, d], v)| ([im, d, 1.0], v))
        .unzip();
    if x.len() < 4 {
        return None;
    }
    let [r, l, v0] = lstsq(&x, &y)?;
    let resid = x
        .iter()
        .zip(&y)
        .map(|(row, v)| v - (r * row[0] + l * row[1] + v0))
        .collect();
    Some(Rise {
        r_ohm: r / window_scale,
        l_h: l / window_scale,
        tau_s: l / r,
        v0_v: v0,
        pairs: x.len(),
        resid_v: resid,
    })
}
