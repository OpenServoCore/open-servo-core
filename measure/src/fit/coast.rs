//! A spin and its coast: the back-EMF constant from the coast's own EMF
//! and ripple, the stage 3 ladder notebook's coast route (oscnb `coast`
//! and the notebook's `coast_ke`), ported.
//!
//! The coast's rows open with the drive's tail, so the cut, the first row
//! with no drive, starts the EMF: with the bridge open the tap difference
//! over its rest value, signed by the spin's direction, is the winding's
//! EMF. A brush that lifts off its segment opens the path for a row or two
//! and the taps read near the zero there, an open row, bridged by a line
//! between its neighbors. The first floating rows read low, so every read
//! starts [`COAST_FROM_ROWS`] past the cut.
//!
//! Ke is the window's mean EMF over its counted ripple speed, both over
//! the same whole cycles nearest 10 ms, so the rotor's deceleration inside
//! the window cancels. A coast passes the gate when the window loses at
//! most [`DECEL_FRAC_MAX`] of its speed and holds [`GATE_CYCLES`] whole
//! cycles: past that the line sweeps out of the band and the count lands
//! anywhere.

use core::f64::consts::PI;

use super::{interp, mean, median, polyfit, sd};
use crate::capture::rows::TelFrame;
use crate::ripple::{self, order::boxcar};

/// 1 ms past the cut, after the taps settle.
pub const COAST_FROM_ROWS: usize = 20;
/// 10 ms of the early coast, where the seed is read.
pub const SEED_ROWS: usize = 200;
/// 0.1 to 1 ms past the cut: did the rotor turn.
pub const EARLY_ROWS: (usize, usize) = (2, 20);
/// V, an EMF under this: the rotor has about stopped.
pub const TURN_V: f64 = 0.05;
/// About 2 ms, the stretch an EMF row is compared with.
pub const NEIGHBORHOOD_ROWS: usize = 41;
/// A row under this share of its neighborhood is open.
pub const OPEN_FRAC: f64 = 0.3;
/// s, the +/- half width of a deceleration fit.
pub const DECEL_HALF_S: f64 = 0.005;
/// s, a fit center every 0.5 ms.
pub const DECEL_STEP_S: f64 = 0.0005;
/// Rows from [`COAST_FROM_ROWS`], the line E0 projects to the cut.
pub const E0_ROWS: usize = 100;
/// rad/s, the speed bin of a deceleration curve.
pub const DECEL_BIN: f64 = 10.0;
/// ms, the Ke window's target.
pub const KE_WINDOW_MS: f64 = 10.0;
/// The gate: the share of its speed a Ke window may lose.
pub const DECEL_FRAC_MAX: f64 = 0.35;
/// The gate: whole cycles a Ke window holds at least.
pub const GATE_CYCLES: usize = 4;
/// s, the drive's last stretch the tach reads at the cut.
pub const CUT_TACH_S: f64 = 0.02;

/// The conversions a coast is read through.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Scales {
    /// Counts of tap difference a volt.
    pub counts_per_v: f64,
    /// Ripple events a motor revolution.
    pub per_rev: usize,
    /// Rows a second.
    pub fs: f64,
}

/// `(current zero, tap difference zero)`: the medians over the second
/// half of a torque-off baseline. None when a row lacks a channel.
pub fn rest_zeros(baseline: &[TelFrame]) -> Option<(f64, f64)> {
    let rest = &baseline[baseline.len() / 2..];
    let i: Option<Vec<f64>> = rest.iter().map(|f| f.current_raw.map(f64::from)).collect();
    let e: Option<Vec<f64>> = rest
        .iter()
        .map(|f| Some(f64::from(f.vmotor_a?) - f64::from(f.vmotor_b?)))
        .collect();
    Some((median(&i?)?, median(&e?)?))
}

/// The first row with no drive; None when the drive never stops or a row
/// before it carries no duty.
pub fn cut_row(coast: &[TelFrame]) -> Option<usize> {
    for (k, f) in coast.iter().enumerate() {
        if f.duty_counts? == 0 {
            return Some(k);
        }
    }
    None
}

/// The EMF from the cut, V: the tap difference signed by `dir`, over
/// `zero_counts`. None when a row lacks a tap.
pub fn emf_from_cut(
    coast: &[TelFrame],
    dir: i8,
    cut: usize,
    zero_counts: f64,
    counts_per_v: f64,
) -> Option<Vec<f64>> {
    let v_per_count = 1.0 / counts_per_v;
    coast
        .get(cut..)?
        .iter()
        .map(|f| {
            let taps = (i32::from(f.vmotor_a?) - i32::from(f.vmotor_b?)) * i32::from(dir);
            Some((f64::from(taps) - zero_counts) * v_per_count)
        })
        .collect()
}

/// The EMF's mean just past the cut, what says the rotor turned.
pub fn early_emf(emf: &[f64]) -> Option<f64> {
    mean(emf.get(EARLY_ROWS.0..EARLY_ROWS.1)?)
}

/// The EMF's running mean over [`NEIGHBORHOOD_ROWS`], centered.
pub fn local_mean(emf: &[f64]) -> Vec<f64> {
    boxcar(emf, NEIGHBORHOOD_ROWS)
}

/// The rows under [`OPEN_FRAC`] of their neighborhood while the rotor
/// turns, none before [`COAST_FROM_ROWS`].
pub fn open_rows(emf: &[f64]) -> Vec<bool> {
    let r = local_mean(emf);
    emf.iter()
        .zip(&r)
        .enumerate()
        .map(|(k, (e, r))| k >= COAST_FROM_ROWS && *e < OPEN_FRAC * r && *r > TURN_V)
        .collect()
}

/// The EMF with each open row a line between its neighbors.
pub fn bridge(emf: &[f64], bad: &[bool]) -> Vec<f64> {
    let (xp, fp): (Vec<f64>, Vec<f64>) = (0..emf.len())
        .filter(|&k| !bad[k])
        .map(|k| (k as f64, emf[k]))
        .unzip();
    if xp.len() == emf.len() || xp.is_empty() {
        return emf.to_vec();
    }
    let (l, r) = (fp[0], fp[fp.len() - 1]);
    (0..emf.len())
        .map(|k| {
            if bad[k] {
                interp(k as f64, &xp, &fp, l, r)
            } else {
                emf[k]
            }
        })
        .collect()
}

/// The first row from [`COAST_FROM_ROWS`] where the running mean falls
/// under [`TURN_V`].
pub fn stop_row(emf: &[f64]) -> Option<usize> {
    local_mean(emf)
        .iter()
        .enumerate()
        .skip(COAST_FROM_ROWS)
        .find(|(_, m)| **m < TURN_V)
        .map(|(k, _)| k)
}

/// The early coast's ripple line, Hz, the seed of every count on it.
pub fn seed_freq(emf: &[f64], fs: f64) -> Option<f64> {
    let a = COAST_FROM_ROWS.min(emf.len());
    let b = (COAST_FROM_ROWS + SEED_ROWS).min(emf.len());
    ripple::line_freq(&emf[a..b], ripple::BAND_HZ.0, ripple::BAND_HZ.1, fs)
}

/// One deceleration fit: a quadratic of the bridged EMF on time.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DecelFit {
    /// s from the cut.
    pub t: f64,
    /// V and V/s at the center.
    pub emf: f64,
    pub dedt: f64,
    /// rad/s and rad/s^2 through Ke.
    pub w: f64,
    pub alpha: f64,
    pub open_rows: usize,
}

/// One fit every [`DECEL_STEP_S`] from [`COAST_FROM_ROWS`] down to `stop`,
/// each over +/- [`DECEL_HALF_S`]; `ke` in V s/rad.
pub fn coast_decel(emf: &[f64], bad: &[bool], stop: usize, ke: f64, fs: f64) -> Vec<DecelFit> {
    let h = (DECEL_HALF_S * fs) as usize;
    let step = ((DECEL_STEP_S * fs) as usize).max(1);
    let t = |k: usize| k as f64 / fs;
    let mut out = Vec::new();
    let mut j = COAST_FROM_ROWS + h;
    while j + h < stop && j + h < emf.len() {
        let x: Vec<f64> = (j - h..=j + h).map(|k| t(k) - t(j)).collect();
        if let Some(p) = polyfit::<3>(&x, &emf[j - h..=j + h]) {
            out.push(DecelFit {
                t: t(j),
                emf: p[0],
                dedt: p[1],
                w: p[0] / ke,
                alpha: p[1] / ke,
                open_rows: bad[j - h..=j + h].iter().filter(|b| **b).count(),
            });
        }
        j += step;
    }
    out
}

/// A coast's deceleration reduction.
#[derive(Clone, Debug, PartialEq)]
pub struct CoastTable {
    pub fits: Vec<DecelFit>,
    /// V, a line through the early coast projected to the cut.
    pub e0: f64,
    /// rev/s at the cut by E0 over Ke.
    pub w_cut_rev_s: f64,
    /// The row the rotor stopped at, or the coast's end.
    pub stop: usize,
    pub open_rows: usize,
    /// rev/s of the early coast's ripple line.
    pub line_rev_s: Option<f64>,
}

/// A coast's EMF from the cut, open rows bridged, through Ke (V s/rad):
/// its deceleration fits down to the stop, E0 and the stop. None on a
/// coast too short for E0's line.
pub fn coast_table(emf: &[f64], ke: f64, s: &Scales) -> Option<CoastTable> {
    if emf.len() < COAST_FROM_ROWS + E0_ROWS || !emf.iter().all(|v| v.is_finite()) {
        return None;
    }
    let bad = open_rows(emf);
    let e = bridge(emf, &bad);
    let stop = stop_row(&e).unwrap_or(e.len());
    let fits = coast_decel(&e, &bad, stop, ke, s.fs);
    let early = COAST_FROM_ROWS..COAST_FROM_ROWS + E0_ROWS;
    let t: Vec<f64> = early.clone().map(|k| k as f64 / s.fs).collect();
    let [e0, _] = polyfit::<2>(&t, &e[early])?;
    Some(CoastTable {
        fits,
        e0,
        w_cut_rev_s: e0 / ke / (2.0 * PI),
        stop,
        open_rows: bad[..stop].iter().filter(|b| **b).count(),
        line_rev_s: seed_freq(&e, s.fs).map(|f| f / s.per_rev as f64),
    })
}

/// A coast's deceleration against its speed: the median per
/// [`DECEL_BIN`] bin at the bin's mean speed, interpolated, the end values
/// held.
#[derive(Clone, Debug, PartialEq)]
pub struct DecelCurve {
    pub w: Vec<f64>,
    pub alpha: Vec<f64>,
}

impl DecelCurve {
    /// rad/s^2 at `w` rad/s.
    pub fn at(&self, w: f64) -> f64 {
        interp(
            w,
            &self.w,
            &self.alpha,
            self.alpha[0],
            self.alpha[self.alpha.len() - 1],
        )
    }

    /// rad/s, the curve's top bin.
    pub fn top(&self) -> f64 {
        self.w[self.w.len() - 1]
    }
}

/// The curve of a coast's fits; None without one.
pub fn decel_curve(fits: &[DecelFit]) -> Option<DecelCurve> {
    let mut h: Vec<(f64, f64)> = fits.iter().map(|f| (f.w, f.alpha)).collect();
    if h.is_empty() || !h.iter().all(|(w, a)| w.is_finite() && a.is_finite()) {
        return None;
    }
    h.sort_by(|a, b| a.0.total_cmp(&b.0));
    let bin = |w: f64| (w / DECEL_BIN).floor().max(-1.0) as i64;
    let (mut ws, mut al) = (Vec::new(), Vec::new());
    let mut k = 0;
    while k < h.len() {
        let b = bin(h[k].0);
        let e = k + h[k..].iter().take_while(|x| bin(x.0) == b).count();
        let w: Vec<f64> = h[k..e].iter().map(|x| x.0).collect();
        let a: Vec<f64> = h[k..e].iter().map(|x| x.1).collect();
        ws.push(mean(&w)?);
        al.push(median(&a)?);
        k = e;
    }
    Some(DecelCurve { w: ws, alpha: al })
}

/// The Ke window of a turned coast.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct KeWindow {
    /// Rows from the cut, whole cycles of the ripple.
    pub i0: usize,
    pub i1: usize,
    pub cycles: usize,
    /// rev/s over the window.
    pub rev_s: f64,
    /// V, the bridged EMF's mean over the window.
    pub e_win: f64,
    /// V and V/s at the cut: a line through the window's one-ripple
    /// boxcar projected back.
    pub e_cut: f64,
    pub dedt: f64,
    /// V s/rad, the window's EMF over its speed.
    pub ke: f64,
    /// The share of its speed the window loses.
    pub decel_frac: f64,
}

/// One coast's read.
#[derive(Clone, Debug, PartialEq)]
pub struct CoastKe {
    pub dir: i8,
    /// The spin's held duty, ticks: the level the pooled split is by.
    pub level: u16,
    pub cut: usize,
    /// V, 0.1 to 1 ms past the cut.
    pub e_first: f64,
    pub turned: bool,
    pub open_rows: usize,
    /// Hz, the early coast's line.
    pub f_seed: Option<f64>,
    pub stop: Option<usize>,
    pub window: Option<KeWindow>,
    /// rev/s, the drive's ripple tach over its last [`CUT_TACH_S`].
    pub spin_rev_s: Option<f64>,
    pub gated: bool,
}

impl CoastKe {
    /// V s/rad when the coast passed the gate.
    pub fn ke(&self) -> Option<f64> {
        self.window.filter(|_| self.gated).map(|w| w.ke)
    }

    /// V s/rad, E0 over the drive's tach at the cut: a check on
    /// [`CoastKe::ke`] that carries the tach's 20 ms spread.
    pub fn ke_cut(&self) -> Option<f64> {
        let w = self.window?;
        let v = w.e_cut / (2.0 * PI * self.spin_rev_s?);
        v.is_finite().then_some(v)
    }
}

/// V s/rad as mV per rev/s.
pub fn mv_per_rev_s(ke: f64) -> f64 {
    ke * 2.0 * PI * 1e3
}

/// The spin's held duty, ticks: the median of the second half's.
fn held_duty(spin: &[TelFrame]) -> Option<f64> {
    let d: Option<Vec<f64>> = spin[spin.len() / 2..]
        .iter()
        .map(|f| f.duty_counts.map(|d| f64::from(d.unsigned_abs())))
        .collect();
    median(&d?)
}

/// rev/s by the ripple tach over the held drive's last [`CUT_TACH_S`],
/// seeded by the coast's line.
fn spin_rev_s(spin: &[TelFrame], half: f64, f_seed: f64, s: &Scales) -> Option<f64> {
    let held: Option<Vec<f64>> = spin
        .iter()
        .filter(|f| {
            f.duty_counts
                .is_some_and(|d| f64::from(d.unsigned_abs()) == half)
        })
        .map(|f| f.current_raw.map(f64::from))
        .collect();
    let held = held?;
    let x = &held[held.len().saturating_sub((CUT_TACH_S * s.fs) as usize)..];
    let m = mean(x)?;
    let x: Vec<f64> = x.iter().map(|v| v - m).collect();
    let c = ripple::count_cycles(&x, f_seed, ripple::BAND_FRAC, s.fs)?;
    Some(c.hz / s.per_rev as f64)
}

/// The Ke window from [`COAST_FROM_ROWS`] on the bridged EMF, and E0.
fn ke_window(emf: &[f64], f_seed: f64, s: &Scales) -> Option<KeWindow> {
    let w = ripple::read_window(emf, COAST_FROM_ROWS, f_seed, KE_WINDOW_MS, true, s.fs)?;
    if w.cycles == 0 {
        return None;
    }
    let (i0, i1) = (w.i0 as usize, w.i1 as usize);
    if i1 <= i0 + 1 || i1 > emf.len() {
        return None;
    }
    let nbox = ((s.fs / w.hz).round_ties_even() as usize).max(1);
    let smooth = boxcar(emf, nbox);
    let t: Vec<f64> = (i0..i1).map(|k| k as f64 / s.fs).collect();
    let [e_cut, dedt] = polyfit::<2>(&t, &smooth[i0..i1])?;
    let e_win = mean(&emf[i0..i1])?;
    let rev_s = w.hz / s.per_rev as f64;
    let ke = e_win / (2.0 * PI * w.hz / s.per_rev as f64);
    let win_ms = (w.i1 - w.i0) / s.fs * 1e3;
    Some(KeWindow {
        i0,
        i1,
        cycles: w.cycles,
        rev_s,
        e_win,
        e_cut,
        dedt,
        ke,
        decel_frac: -dedt * win_ms / 1e3 / e_win,
    })
}

/// One spin and its coast: the coast's Ke window through the gate, and the
/// drive's tach at the cut. `zero_e` is the rest tap difference. A coast
/// that never cuts, or a row without its channels, is an error.
pub fn coast_ke(
    spin: &[TelFrame],
    coast: &[TelFrame],
    dir: i8,
    zero_e: f64,
    s: &Scales,
) -> Result<CoastKe, String> {
    let cut = cut_row(coast).ok_or("the coast never opens the bridge")?;
    let level = held_duty(spin).ok_or("the spin carries no duty")?;
    let emf =
        emf_from_cut(coast, dir, cut, zero_e, s.counts_per_v).ok_or("a coast row lacks a tap")?;
    let e_first = early_emf(&emf).ok_or("the coast ends inside its first millisecond")?;
    let mut out = CoastKe {
        dir,
        level: level as u16,
        cut,
        e_first,
        turned: e_first >= TURN_V,
        open_rows: 0,
        f_seed: None,
        stop: None,
        window: None,
        spin_rev_s: None,
        gated: false,
    };
    if !out.turned {
        return Ok(out);
    }
    let bad = open_rows(&emf);
    let e = bridge(&emf, &bad);
    out.open_rows = bad.iter().filter(|b| **b).count();
    out.stop = stop_row(&e);
    out.f_seed = seed_freq(&e, s.fs);
    let Some(f0) = out.f_seed else {
        return Ok(out);
    };
    out.window = ke_window(&e, f0, s);
    out.spin_rev_s = spin_rev_s(spin, level, f0, s);
    out.gated = out
        .window
        .is_some_and(|w| w.decel_frac <= DECEL_FRAC_MAX && w.cycles >= GATE_CYCLES);
    Ok(out)
}

/// A mean over coasts with its sd and count.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Pooled {
    pub mean: f64,
    /// Over n - 1; None from one coast.
    pub sd: Option<f64>,
    pub n: usize,
}

impl Pooled {
    pub fn of(v: &[f64]) -> Option<Pooled> {
        Some(Pooled {
            mean: mean(v)?,
            sd: sd(v, 1),
            n: v.len(),
        })
    }
}

/// Ke pooled over the gated coasts, V s/rad, and split by direction and by
/// level.
#[derive(Clone, Debug, PartialEq)]
pub struct KePooled {
    pub all: Pooled,
    pub by_dir: Vec<(i8, Pooled)>,
    pub by_level: Vec<(u16, Pooled)>,
    pub coasts: usize,
}

/// None when no coast passed the gate.
pub fn pooled(coasts: &[CoastKe]) -> Option<KePooled> {
    let gated: Vec<&CoastKe> = coasts.iter().filter(|c| c.ke().is_some()).collect();
    let ke = |f: &dyn Fn(&CoastKe) -> bool| -> Vec<f64> {
        gated
            .iter()
            .filter(|c| f(c))
            .filter_map(|c| c.ke())
            .collect()
    };
    let mut dirs: Vec<i8> = gated.iter().map(|c| c.dir).collect();
    dirs.sort_unstable();
    dirs.dedup();
    let mut levels: Vec<u16> = gated.iter().map(|c| c.level).collect();
    levels.sort_unstable();
    levels.dedup();
    Some(KePooled {
        all: Pooled::of(&ke(&|_| true))?,
        by_dir: dirs
            .into_iter()
            .filter_map(|d| Some((d, Pooled::of(&ke(&|c| c.dir == d))?)))
            .collect(),
        by_level: levels
            .into_iter()
            .filter_map(|l| Some((l, Pooled::of(&ke(&|c| c.level == l))?)))
            .collect(),
        coasts: coasts.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: Scales = Scales {
        counts_per_v: 250.0,
        per_rev: 6,
        fs: 20_000.0,
    };
    const KE: f64 = 1.16e-3;
    const ZERO_E: f64 = 3.0;

    fn row(tick: u64, duty: i16, a: u16, b: u16, raw: u16) -> TelFrame {
        TelFrame {
            tick,
            duty_counts: Some(duty),
            vmotor_a: Some(a),
            vmotor_b: Some(b),
            current_raw: Some(raw),
            ..TelFrame::default()
        }
    }

    /// A spin held at 720 ticks and its coast from `w0` rad/s, slowing at
    /// `decel`, the EMF `KE w` with a 5 % commutation ripple, six a turn.
    fn spin_and_coast(w0: f64, decel: f64) -> (Vec<TelFrame>, Vec<TelFrame>) {
        let spin = (0..1000u64)
            .map(|k| {
                let th = w0 * k as f64 / S.fs;
                row(k, 720, 1500, 300, (1300.0 + 40.0 * (6.0 * th).cos()) as u16)
            })
            .collect();
        let (mut w, mut th) = (w0, 0.0f64);
        let coast = (0..2400u64)
            .map(|k| {
                let emf = KE * w * (1.0 + 0.05 * (6.0 * th).cos());
                let taps = (emf * S.counts_per_v + ZERO_E).round() as u16;
                th += w / S.fs;
                w = (w - decel / S.fs).max(0.0);
                row(
                    1000 + k,
                    if k < 2 { 720 } else { 0 },
                    1000 + taps,
                    1000,
                    1210,
                )
            })
            .collect();
        (spin, coast)
    }

    #[test]
    fn a_planted_coast_gives_its_ke() {
        let (spin, coast) = spin_and_coast(2.0 * PI * 300.0, 1.5e4);
        let c = coast_ke(&spin, &coast, 1, ZERO_E, &S).expect("reads");
        assert_eq!((c.cut, c.level, c.turned, c.gated), (2, 720, true, true));
        let w = c.window.expect("a window");
        assert!((w.ke / KE - 1.0).abs() < 0.005, "{} against {KE}", w.ke);
        assert!((c.spin_rev_s.expect("tach") / 300.0 - 1.0).abs() < 0.01);
        let p = pooled(&[c.clone(), c]).expect("pooled");
        assert_eq!((p.all.n, p.by_dir.len(), p.by_level.len()), (2, 1, 1));
        assert!((mv_per_rev_s(p.all.mean) / (mv_per_rev_s(KE)) - 1.0).abs() < 0.005);
    }

    /// A coast that loses most of its speed inside its window fails the
    /// gate and gives no Ke.
    #[test]
    fn a_coast_that_stops_inside_its_window_is_gated_out() {
        let (spin, coast) = spin_and_coast(2.0 * PI * 30.0, 1.2e4);
        let c = coast_ke(&spin, &coast, 1, ZERO_E, &S).expect("reads");
        assert!(!c.gated && c.ke().is_none());
        assert_eq!(pooled(&[c]), None);
    }

    #[test]
    fn empty_flat_and_missing_rows_read_as_nothing() {
        let (spin, coast) = spin_and_coast(2.0 * PI * 300.0, 1.5e4);
        assert!(coast_ke(&spin, &[], 1, ZERO_E, &S).is_err());
        assert!(coast_ke(&[], &coast, 1, ZERO_E, &S).is_err());
        let never: Vec<TelFrame> = coast
            .iter()
            .map(|f| TelFrame {
                duty_counts: Some(5),
                ..*f
            })
            .collect();
        assert!(coast_ke(&spin, &never, 1, ZERO_E, &S).is_err());
        let mut hole = coast.clone();
        hole[100].vmotor_b = None;
        assert!(coast_ke(&spin, &hole, 1, ZERO_E, &S).is_err());
        let still: Vec<TelFrame> = coast
            .iter()
            .map(|f| TelFrame {
                vmotor_a: Some(1003),
                ..*f
            })
            .collect();
        let c = coast_ke(&spin, &still, 1, ZERO_E, &S).expect("reads");
        assert!(!c.turned && c.window.is_none() && c.ke().is_none());
        assert_eq!(cut_row(&[]), None);
        assert_eq!(early_emf(&[]), None);
        assert_eq!(coast_table(&[], KE, &S), None);
        let mut nan = vec![0.5; 400];
        nan[200] = f64::NAN;
        assert_eq!(coast_table(&nan, KE, &S), None);
        assert_eq!(decel_curve(&[]), None);
        assert_eq!(pooled(&[]), None);
        assert_eq!(rest_zeros(&[]), None);
        assert!(stop_row(&[]).is_none() && seed_freq(&[], S.fs).is_none());
    }
}
