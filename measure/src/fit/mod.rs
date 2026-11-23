//! Estimators over captures, pure: no bus, no files.

pub mod coast;
pub mod rise;
pub mod runup;
pub mod seated;
pub mod window;

/// The median; the mean of the middle two on an even count. None when
/// empty.
pub fn median(v: &[f64]) -> Option<f64> {
    if v.is_empty() {
        return None;
    }
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    let m = s.len() / 2;
    Some(if s.len() % 2 == 0 {
        (s[m - 1] + s[m]) / 2.0
    } else {
        s[m]
    })
}

pub fn mean(v: &[f64]) -> Option<f64> {
    (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
}

/// The standard deviation with `ddof` degrees of freedom taken; None
/// unless more rows than `ddof`.
pub fn sd(v: &[f64], ddof: usize) -> Option<f64> {
    if v.len() <= ddof {
        return None;
    }
    let m = mean(v)?;
    let ss: f64 = v.iter().map(|x| (x - m) * (x - m)).sum();
    Some((ss / (v.len() - ddof) as f64).sqrt())
}

/// numpy's `interp`: `fp` at `x` on the line between the two `xp` around
/// it, `left` and `right` past the ends; `xp` must not fall.
pub fn interp(x: f64, xp: &[f64], fp: &[f64], left: f64, right: f64) -> f64 {
    let n = xp.len().min(fp.len());
    if n == 0 || x.is_nan() {
        return f64::NAN;
    }
    if x < xp[0] {
        return left;
    }
    if x > xp[n - 1] {
        return right;
    }
    if x == xp[n - 1] {
        return fp[n - 1];
    }
    let j = xp[..n].partition_point(|&v| v <= x) - 1;
    if xp[j] == x {
        return fp[j];
    }
    let slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
    let y = slope * (x - xp[j]) + fp[j];
    if y.is_nan() {
        let y = slope * (x - xp[j + 1]) + fp[j + 1];
        if y.is_nan() && fp[j] == fp[j + 1] {
            return fp[j];
        }
        return y;
    }
    y
}

/// The least-squares polynomial of `y` on `x` with `N` coefficients,
/// lowest power first; `x` is scaled to unit range before the normal
/// equations, so the fit keeps its digits. None under `N` rows or on a
/// singular fit.
pub fn polyfit<const N: usize>(x: &[f64], y: &[f64]) -> Option<[f64; N]> {
    if x.len() < N || x.len() != y.len() {
        return None;
    }
    let s = x.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if !(s > 0.0 && s.is_finite()) {
        return None;
    }
    let rows: Vec<[f64; N]> = x
        .iter()
        .map(|&v| {
            let u = v / s;
            let mut r = [1.0; N];
            for k in 1..N {
                r[k] = r[k - 1] * u;
            }
            r
        })
        .collect();
    let mut c = lstsq(&rows, y)?;
    let mut p = 1.0;
    for ck in c.iter_mut() {
        *ck /= p;
        p *= s;
    }
    Some(c)
}

/// A polynomial, lowest power first, at `x`.
pub fn polyval(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, ck| acc * x + ck)
}

/// Least squares of `y` on the columns of `x`, every row `N` wide, by the
/// normal equations; None when they are singular.
pub fn lstsq<const N: usize>(x: &[[f64; N]], y: &[f64]) -> Option<[f64; N]> {
    let mut a = [[0.0; N]; N];
    let mut b = [0.0; N];
    for (row, &yi) in x.iter().zip(y) {
        for i in 0..N {
            b[i] += row[i] * yi;
            for j in 0..N {
                a[i][j] += row[i] * row[j];
            }
        }
    }
    // Gauss-Jordan with partial pivoting
    for c in 0..N {
        let p = (c..N).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-300 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in 0..N {
            if r != c {
                let f = a[r][c] / a[c][c];
                for k in c..N {
                    a[r][k] -= f * a[c][k];
                }
                b[r] -= f * b[c];
            }
        }
    }
    let mut out = [0.0; N];
    for i in 0..N {
        out[i] = b[i] / a[i][i];
    }
    out.iter().all(|v| v.is_finite()).then_some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn median_mean_and_least_squares() {
        assert_eq!(median(&[3.0, 1.0, 2.0]), Some(2.0));
        assert_eq!(median(&[4.0, 1.0, 2.0, 3.0]), Some(2.5));
        assert_eq!(median(&[]), None);
        assert_eq!(mean(&[1.0, 2.0]), Some(1.5));
        let x: Vec<[f64; 2]> = (0..5).map(|i| [i as f64, 1.0]).collect();
        let y: Vec<f64> = (0..5).map(|i| 3.0 * i as f64 - 2.0).collect();
        let [m, c] = lstsq(&x, &y).expect("solves");
        assert!((m - 3.0).abs() < 1e-12 && (c + 2.0).abs() < 1e-12);
        assert_eq!(lstsq(&[[1.0, 1.0], [2.0, 2.0]], &[1.0, 2.0]), None);
    }
}
