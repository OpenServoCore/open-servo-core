//! Estimators over captures, pure: no bus, no files.

pub mod rise;
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
