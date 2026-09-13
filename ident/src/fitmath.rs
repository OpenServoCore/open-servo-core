//! Fit primitives the experiment fits compose: least squares (through the
//! origin and affine), robust means, and a sliding local-quadratic
//! derivative extractor. All f64 - Q encoding is gain synthesis' job. Every
//! fn returns None on degenerate input (too few points, no x spread,
//! non-finite data) and never panics or emits NaN.

/// y = slope * x through the origin: slope = sum(x*y) / sum(x^2).
///
/// `r2` uses the affine convention, 1 - SS_res / SS_tot with SS_tot about
/// mean(y) - stricter than the through-origin textbook form (which
/// inflates toward 1), and comparable with [`LinearFit::r2`]. All-equal y
/// makes SS_tot 0: r2 is 1 if the residuals are also 0, else 0.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OriginFit {
    pub slope: f64,
    pub r2: f64,
    pub rms: f64,
    pub n: usize,
}

pub fn origin_ls(xy: &[(f64, f64)]) -> Option<OriginFit> {
    if xy.len() < 2 || !finite_xy(xy) {
        return None;
    }
    let sxx: f64 = xy.iter().map(|(x, _)| x * x).sum();
    let sxy: f64 = xy.iter().map(|(x, y)| x * y).sum();
    if sxx <= 0.0 {
        return None;
    }
    let slope = sxy / sxx;
    let (r2, rms) = fit_quality(xy, |x| slope * x)?;
    Some(OriginFit {
        slope,
        r2,
        rms,
        n: xy.len(),
    })
}

/// y = a + b * x by ordinary least squares.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinearFit {
    pub a: f64,
    pub b: f64,
    pub r2: f64,
    pub rms: f64,
    pub n: usize,
}

pub fn linear_ls(xy: &[(f64, f64)]) -> Option<LinearFit> {
    let n = xy.len();
    if n < 2 || !finite_xy(xy) {
        return None;
    }
    let mx = xy.iter().map(|(x, _)| x).sum::<f64>() / n as f64;
    let my = xy.iter().map(|(_, y)| y).sum::<f64>() / n as f64;
    let sxx: f64 = xy.iter().map(|(x, _)| (x - mx) * (x - mx)).sum();
    let sxy: f64 = xy.iter().map(|(x, y)| (x - mx) * (y - my)).sum();
    if sxx <= 0.0 {
        return None;
    }
    let b = sxy / sxx;
    let a = my - b * mx;
    let (r2, rms) = fit_quality(xy, |x| a + b * x)?;
    Some(LinearFit { a, b, r2, rms, n })
}

pub fn mean(v: &[f64]) -> Option<f64> {
    if v.is_empty() || !v.iter().all(|x| x.is_finite()) {
        return None;
    }
    Some(v.iter().sum::<f64>() / v.len() as f64)
}

/// Linear-interpolated quantile, `q` in [0, 1]. None on empty or
/// non-finite input.
pub fn quantile(v: &[f64], q: f64) -> Option<f64> {
    if v.is_empty() || !v.iter().all(|x| x.is_finite()) || !(0.0..=1.0).contains(&q) {
        return None;
    }
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    let h = q * (s.len() - 1) as f64;
    let lo = h.floor() as usize;
    let hi = h.ceil() as usize;
    Some(s[lo] + (s[hi] - s[lo]) * (h - lo as f64))
}

pub fn median(v: &[f64]) -> Option<f64> {
    quantile(v, 0.5)
}

/// Sample standard deviation (n - 1 divisor).
pub fn stddev(v: &[f64]) -> Option<f64> {
    if v.len() < 2 {
        return None;
    }
    let m = mean(v)?;
    Some((v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (v.len() - 1) as f64).sqrt())
}

/// Mean after dropping the lowest and highest `floor(n * trim_frac)`
/// samples each. `trim_frac` in [0, 0.5); at least one sample must survive.
pub fn trimmed_mean(v: &[f64], trim_frac: f64) -> Option<f64> {
    if v.is_empty() || !v.iter().all(|x| x.is_finite()) || !(0.0..0.5).contains(&trim_frac) {
        return None;
    }
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    let k = (v.len() as f64 * trim_frac) as usize;
    mean(&s[k..v.len() - k])
}

/// Per-sample first and second derivatives; `None` where the window is
/// incomplete (edges) or the local fit is singular.
#[derive(Clone, Debug, Default)]
pub struct QuadDeriv {
    pub dy: Vec<Option<f64>>,
    pub d2y: Vec<Option<f64>>,
}

/// y = a + b * x by Theil-Sen: the median of all pairwise slopes, then the
/// median residual as the intercept. Costs O(n^2) pairs, so it is for the
/// short runs (tens of points) where one settling sample or one dropped
/// conversion would drag a least-squares slope.
pub fn theil_sen(xy: &[(f64, f64)]) -> Option<LinearFit> {
    let n = xy.len();
    if n < 2 || !finite_xy(xy) {
        return None;
    }
    let mut slopes = Vec::with_capacity(n * (n - 1) / 2);
    for (i, (xi, yi)) in xy.iter().enumerate() {
        for (xj, yj) in &xy[i + 1..] {
            if xj != xi {
                slopes.push((yj - yi) / (xj - xi));
            }
        }
    }
    let b = median(&slopes)?;
    let a = median(&xy.iter().map(|(x, y)| y - b * x).collect::<Vec<_>>())?;
    let (r2, rms) = fit_quality(xy, |x| a + b * x)?;
    Some(LinearFit { a, b, r2, rms, n })
}

/// y = a + b*x + c*exp(-x/tau) least squares at a FIXED tau: a ramp read
/// through a first-order lag. `b` is the ramp's own slope with the lag's
/// contribution taken out, so no leading samples have to be discarded to
/// escape it. `x` must start at 0 (the caller centres on the first sample)
/// or the exponential column loses conditioning.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LagFit {
    pub a: f64,
    pub b: f64,
    /// Lag amplitude; negative while the reading is still catching up.
    pub c: f64,
    pub rms: f64,
    pub n: usize,
}

pub fn lag_ls(xy: &[(f64, f64)], tau: f64) -> Option<LagFit> {
    let n = xy.len();
    if n < 4 || tau <= 0.0 || !finite_xy(xy) {
        return None;
    }
    let mut m = [[0.0f64; 4]; 3];
    for (x, y) in xy {
        let p = [1.0, *x, (-x / tau).exp()];
        for (r, &pr) in p.iter().enumerate() {
            for (c, &pc) in p.iter().enumerate() {
                m[r][c] += pr * pc;
            }
            m[r][3] += pr * y;
        }
    }
    let v = solve3(&mut m)?;
    let (a, b, c) = (v[0], v[1], v[2]);
    if !(a.is_finite() && b.is_finite() && c.is_finite()) {
        return None;
    }
    let (_, rms) = fit_quality(xy, |x| a + b * x + c * (-x / tau).exp())?;
    Some(LagFit { a, b, c, rms, n })
}

/// y = x . b least squares over a few columns, by the normal equations
/// with every column scaled to unit RMS first - the callers mix amps,
/// volts, ones and milliseconds in one design matrix. Every row must be as
/// wide as the first. Returns the coefficients and the residual sum of
/// squares.
pub fn lstsq(x: &[Vec<f64>], y: &[f64]) -> Option<(Vec<f64>, f64)> {
    let n = x.len();
    let m = x.first()?.len();
    if n != y.len()
        || n <= m
        || x.iter().any(|r| r.len() != m)
        || !y.iter().chain(x.iter().flatten()).all(|v| v.is_finite())
    {
        return None;
    }
    let mut scale = vec![0.0f64; m];
    for (c, sc) in scale.iter_mut().enumerate() {
        *sc = (x.iter().map(|r| r[c] * r[c]).sum::<f64>() / n as f64).sqrt();
        if *sc <= 0.0 {
            return None;
        }
    }
    let mut a = vec![vec![0.0f64; m]; m];
    let mut b = vec![0.0f64; m];
    for (row, yv) in x.iter().zip(y) {
        for r in 0..m {
            let xr = row[r] / scale[r];
            for c in 0..m {
                a[r][c] += xr * row[c] / scale[c];
            }
            b[r] += xr * yv;
        }
    }
    for col in 0..m {
        let piv = (col..m).max_by(|&p, &q| a[p][col].abs().total_cmp(&a[q][col].abs()))?;
        if a[piv][col].abs() < 1e-12 {
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
    let mut coef = vec![0.0f64; m];
    for r in (0..m).rev() {
        let s: f64 = (r + 1..m).map(|c| a[r][c] * coef[c]).sum();
        coef[r] = (b[r] - s) / a[r][r];
    }
    for (c, sc) in coef.iter_mut().zip(&scale) {
        *c /= sc;
    }
    let rss = x
        .iter()
        .zip(y)
        .map(|(row, yv)| {
            let p: f64 = row.iter().zip(&coef).map(|(a, b)| a * b).sum();
            (yv - p) * (yv - p)
        })
        .sum();
    coef.iter().all(|c| c.is_finite()).then_some((coef, rss))
}

/// Sliding local-quadratic derivative (Savitzky-Golay flavor, nonuniform t
/// allowed): at each i, fit y = c0 + c1*u + c2*u^2 over u = t - t[i] for
/// the 2*half_window + 1 samples around i, then dy = c1, d2y = 2*c2.
///
/// Conditioning: u is centered by construction and scaled by its max
/// magnitude before the 3x3 normal-equation solve, so tick-scale spacings
/// (5e-5 s) do not collapse the u^4 moment into denormals.
pub fn sliding_quadratic_deriv(t: &[f64], y: &[f64], half_window: usize) -> QuadDeriv {
    let n = t.len();
    let mut out = QuadDeriv {
        dy: vec![None; n],
        d2y: vec![None; n],
    };
    if n != y.len() || half_window < 1 {
        return out;
    }
    for i in half_window..n.saturating_sub(half_window) {
        let lo = i - half_window;
        let hi = i + half_window + 1;
        let (dy, d2y) = match quad_fit_at(&t[lo..hi], &y[lo..hi], t[i]) {
            Some(v) => v,
            None => continue,
        };
        if dy.is_finite() && d2y.is_finite() {
            out.dy[i] = Some(dy);
            out.d2y[i] = Some(d2y);
        }
    }
    out
}

/// (dy, d2y) of the local quadratic LS fit around `tc`.
fn quad_fit_at(t: &[f64], y: &[f64], tc: f64) -> Option<(f64, f64)> {
    let s = t.iter().map(|&ti| (ti - tc).abs()).fold(0.0f64, f64::max);
    if s <= 0.0 || !t.iter().all(|x| x.is_finite()) || !y.iter().all(|x| x.is_finite()) {
        return None;
    }
    // normal equations for basis {1, v, v^2}, v = (t - tc) / s
    let mut m = [[0.0f64; 4]; 3]; // augmented [A | b]
    for (&ti, &yi) in t.iter().zip(y) {
        let v = (ti - tc) / s;
        let p = [1.0, v, v * v];
        for (r, &pr) in p.iter().enumerate() {
            for (c, &pc) in p.iter().enumerate() {
                m[r][c] += pr * pc;
            }
            m[r][3] += pr * yi;
        }
    }
    let c = solve3(&mut m)?;
    Some((c[1] / s, 2.0 * c[2] / (s * s)))
}

/// Gaussian elimination with partial pivoting on the 3x4 augmented system.
fn solve3(m: &mut [[f64; 4]; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let piv = (col..3).max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))?;
        if m[piv][col].abs() < 1e-12 {
            return None;
        }
        m.swap(col, piv);
        for row in col + 1..3 {
            let f = m[row][col] / m[col][col];
            for k in col..4 {
                m[row][k] -= f * m[col][k];
            }
        }
    }
    let mut x = [0.0f64; 3];
    for row in (0..3).rev() {
        let mut acc = m[row][3];
        for k in row + 1..3 {
            acc -= m[row][k] * x[k];
        }
        x[row] = acc / m[row][row];
    }
    Some(x)
}

fn finite_xy(xy: &[(f64, f64)]) -> bool {
    xy.iter().all(|(x, y)| x.is_finite() && y.is_finite())
}

/// (r2, residual RMS) of `f` over the points; None if either goes
/// non-finite. SS_tot about mean(y); see [`OriginFit`] for the SS_tot = 0
/// convention.
fn fit_quality(xy: &[(f64, f64)], f: impl Fn(f64) -> f64) -> Option<(f64, f64)> {
    let n = xy.len() as f64;
    let my = xy.iter().map(|(_, y)| y).sum::<f64>() / n;
    let ss_res: f64 = xy.iter().map(|(x, y)| (y - f(*x)).powi(2)).sum();
    let ss_tot: f64 = xy.iter().map(|(_, y)| (y - my).powi(2)).sum();
    let r2 = if ss_tot > 0.0 {
        1.0 - ss_res / ss_tot
    } else if ss_res == 0.0 {
        1.0
    } else {
        0.0
    };
    let rms = (ss_res / n).sqrt();
    (r2.is_finite() && rms.is_finite()).then_some((r2, rms))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic uniform noise in [-half, half] (LCG, no rand dep).
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self, half: f64) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0) * half
        }
    }

    #[test]
    fn origin_ls_recovers_slope() {
        let clean: Vec<_> = (1..20).map(|i| (i as f64, 4.7 * i as f64)).collect();
        let f = origin_ls(&clean).unwrap();
        assert!((f.slope - 4.7).abs() < 1e-12);
        assert!(f.r2 > 0.999999);
        assert!(f.rms < 1e-9);

        let mut lcg = Lcg(7);
        let noisy: Vec<_> = (1..200)
            .map(|i| (i as f64, 4.7 * i as f64 + lcg.next(5.0)))
            .collect();
        let f = origin_ls(&noisy).unwrap();
        assert!((f.slope - 4.7).abs() < 0.01, "slope {}", f.slope);
        assert!(f.r2 > 0.99);

        assert!(origin_ls(&[]).is_none());
        assert!(origin_ls(&[(0.0, 1.0), (0.0, 2.0)]).is_none(), "no x power");
    }

    #[test]
    fn linear_ls_recovers_intercept_and_slope() {
        let clean: Vec<_> = (0..15).map(|i| (i as f64, 3.0 + 0.25 * i as f64)).collect();
        let f = linear_ls(&clean).unwrap();
        assert!((f.a - 3.0).abs() < 1e-12);
        assert!((f.b - 0.25).abs() < 1e-12);
        assert!(f.r2 > 0.999999);

        assert!(
            linear_ls(&[(2.0, 1.0), (2.0, 5.0), (2.0, 9.0)]).is_none(),
            "degenerate x"
        );
        assert!(linear_ls(&[(1.0, f64::NAN), (2.0, 1.0)]).is_none());
    }

    #[test]
    fn quantiles_interpolate_and_median_splits_even_n() {
        let v = [4.0, 1.0, 3.0, 2.0];
        assert_eq!(median(&v), Some(2.5));
        assert_eq!(quantile(&v, 0.0), Some(1.0));
        assert_eq!(quantile(&v, 1.0), Some(4.0));
        assert_eq!(quantile(&v, 0.25), Some(1.75));
        assert_eq!(median(&[7.0]), Some(7.0));
        assert!(median(&[]).is_none());
        assert!(quantile(&[1.0, f64::NAN], 0.5).is_none());
    }

    #[test]
    fn trimmed_mean_kills_outliers() {
        let mut v = vec![10.0; 18];
        v.push(1e6);
        v.push(-2e6);
        assert!(
            (mean(&v).unwrap() - 10.0).abs() > 1.0,
            "plain mean poisoned"
        );
        assert_eq!(trimmed_mean(&v, 0.1), Some(10.0));

        assert!(trimmed_mean(&[], 0.1).is_none());
        assert!(trimmed_mean(&[1.0], 0.5).is_none(), "trim_frac cap");
        assert_eq!(trimmed_mean(&[1.0], 0.0), Some(1.0));
    }

    #[test]
    fn quad_deriv_exact_on_clean_constant_accel() {
        // y = 0.5*a*t^2 + v0*t, tick spacing 1/20100 s
        let (a, v0, dt) = (20_000.0, 500.0, 1.0 / 20_100.0);
        let t: Vec<_> = (0..400).map(|i| i as f64 * dt).collect();
        let y: Vec<_> = t.iter().map(|&t| 0.5 * a * t * t + v0 * t).collect();
        let d = sliding_quadratic_deriv(&t, &y, 5);
        assert!(d.dy[4].is_none() && d.d2y[395].is_none(), "edges");
        for i in 5..395 {
            let dy = d.dy[i].unwrap();
            let d2y = d.d2y[i].unwrap();
            assert!((dy - (a * t[i] + v0)).abs() < 1e-6, "dy at {i}: {dy}");
            assert!((d2y - a).abs() < 1e-4, "d2y at {i}: {d2y}");
        }
    }

    #[test]
    fn quad_deriv_noisy_window_pin() {
        // E4 reality check: pot counts quantized with sigma ~1.5 extra
        // noise at tick spacing. half_window 256 (~25 ms of 20 kHz TEL
        // samples) is what holds noisy d2y RMS under 20% of a mid-range
        // accel - i.e. the raw double derivative needs windows the size of
        // the whole step transient, which is why E4 keeps the
        // exponential-rise fit as plan B and why per-tick TEL data (not
        // 1.25 kHz aggregates) is required at all.
        let (a, v0, dt) = (20_000.0, 0.0, 1.0 / 20_100.0);
        let hw = 256usize;
        let n = 1200;
        let mut lcg = Lcg(42);
        let t: Vec<_> = (0..n).map(|i| i as f64 * dt).collect();
        let y: Vec<_> = t
            .iter()
            .map(|&t| (0.5 * a * t * t + v0 * t + lcg.next(2.6)).round())
            .collect();
        let d = sliding_quadratic_deriv(&t, &y, hw);
        let errs: Vec<_> = (hw..n - hw)
            .filter_map(|i| d.d2y[i].map(|v| (v - a) * (v - a)))
            .collect();
        let rms = (errs.iter().sum::<f64>() / errs.len() as f64).sqrt();
        assert!(rms < 0.2 * a, "d2y rms {rms} vs accel {a}");
    }

    #[test]
    fn synthetic_ladder_recovers_motor_params() {
        // E3-shaped steady-state ladder: v = duty*vbus, electrical
        // v = R*i + Ke*omega, mechanical steady state i = fc + fv*omega
        // -> omega = (v - R*fc) / (Ke + R*fv).
        // scales chosen physical: r*fc must sit well under the drive v
        let (r, ke, fc, fv) = (4.7, 0.012, 0.05, 2.0e-5);
        let vbus = 4.6;
        let ladder: Vec<f64> = (26..=64).step_by(6).map(|p| p as f64 / 100.0).collect();
        let mut pts = Vec::new();
        for sgn in [1.0, -1.0] {
            for &d in &ladder {
                let v = sgn * d * vbus;
                let omega = (v - r * fc * sgn) / (ke + r * fv);
                let i = fc * sgn + fv * omega;
                pts.push((v, i, omega));
            }
        }
        // stalled points (omega = 0): v = R*i
        let stall: Vec<_> = ladder.iter().map(|&d| (d * vbus / r, d * vbus)).collect();
        let r_fit = origin_ls(&stall).unwrap().slope;
        assert!((r_fit - r).abs() < 1e-12);

        let ke_pts: Vec<_> = pts.iter().map(|&(v, i, w)| (w, v - r_fit * i)).collect();
        let ke_fit = origin_ls(&ke_pts).unwrap().slope;
        assert!((ke_fit - ke).abs() < 1e-12, "ke {ke_fit}");

        // friction line per direction; forward here
        let fr_pts: Vec<_> = pts
            .iter()
            .filter(|&&(_, _, w)| w > 0.0)
            .map(|&(_, i, w)| (w, i))
            .collect();
        let f = linear_ls(&fr_pts).unwrap();
        assert!((f.a - fc).abs() < 1e-12, "fc {}", f.a);
        assert!((f.b - fv).abs() < 1e-12, "fv {}", f.b);
    }

    #[test]
    fn lstsq_recovers_mixed_scale_columns() {
        // amps, volts, ones and milliseconds: six decades between columns
        let truth = [0.72, 0.064, -0.013, 0.002];
        let x: Vec<Vec<f64>> = (0..40)
            .map(|k| {
                let k = k as f64;
                vec![0.01 * k, 1.5 + (k % 3.0), 1.0, 0.05 * (k % 7.0)]
            })
            .collect();
        let y: Vec<f64> = x
            .iter()
            .map(|r| r.iter().zip(&truth).map(|(a, b)| a * b).sum())
            .collect();
        let (b, rss) = lstsq(&x, &y).unwrap();
        for (got, want) in b.iter().zip(&truth) {
            assert!((got - want).abs() < 1e-9, "{got} vs {want}");
        }
        assert!(rss < 1e-18);
        // a column of zeros has no scale and no solution
        let flat: Vec<Vec<f64>> = (0..5).map(|k| vec![k as f64, 0.0]).collect();
        assert!(lstsq(&flat, &[0.0; 5]).is_none());
    }
}
