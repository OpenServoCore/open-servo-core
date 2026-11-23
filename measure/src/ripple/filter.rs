//! The zero-phase band-pass the counts run, scipy's `butter(2, band,
//! "band", output="sos")` and `sosfiltfilt` ported.

use core::f64::consts::PI;

/// One biquad, `[b0, b1, b2, 1, a1, a2]`.
pub type Sos = [f64; 6];

#[derive(Copy, Clone, Debug, PartialEq)]
struct C(f64, f64);

impl C {
    fn add(self, o: C) -> C {
        C(self.0 + o.0, self.1 + o.1)
    }
    fn sub(self, o: C) -> C {
        C(self.0 - o.0, self.1 - o.1)
    }
    fn mul(self, o: C) -> C {
        C(self.0 * o.0 - self.1 * o.1, self.0 * o.1 + self.1 * o.0)
    }
    fn scale(self, k: f64) -> C {
        C(self.0 * k, self.1 * k)
    }
    fn div(self, b: C) -> C {
        if b.0.abs() >= b.1.abs() {
            let rat = b.1 / b.0;
            let scl = 1.0 / (b.0 + b.1 * rat);
            C((self.0 + self.1 * rat) * scl, (self.1 - self.0 * rat) * scl)
        } else {
            let rat = b.0 / b.1;
            let scl = 1.0 / (b.1 + b.0 * rat);
            C((self.0 * rat + self.1) * scl, (self.1 * rat - self.0) * scl)
        }
    }
    fn abs2(self) -> f64 {
        self.0 * self.0 + self.1 * self.1
    }
    /// The principal root.
    fn sqrt(self) -> C {
        let r = self.0.hypot(self.1);
        if r == 0.0 {
            return C(0.0, 0.0);
        }
        if self.0 >= 0.0 {
            let t = ((r + self.0) / 2.0).sqrt();
            C(t, self.1 / (2.0 * t))
        } else {
            let t = ((r - self.0) / 2.0).sqrt();
            C(self.1.abs() / (2.0 * t), t.copysign(self.1))
        }
    }
}

/// The second-order Butterworth band-pass over `lo..hi` Hz at `fs` rows a
/// second, two sections with the gain in the first, laid out as scipy
/// pairs them. None unless `0 < lo < hi < fs / 2`.
pub fn band(lo: f64, hi: f64, fs: f64) -> Option<[Sos; 2]> {
    let (w1, w2) = (2.0 * lo / fs, 2.0 * hi / fs);
    if !(w1 > 0.0 && w1 < w2 && w2 < 1.0) {
        return None;
    }
    // prewarped for the bilinear transform at an internal rate of 2
    let (w1, w2) = (4.0 * (PI * w1 / 2.0).tan(), 4.0 * (PI * w2 / 2.0).tan());
    let wo = (w1 * w2).sqrt();
    let (bw, wo2) = (w2 - w1, wo * wo);
    // the analog prototype's pole, -exp(j pi / 4) conjugated
    let p = C(-(PI / 4.0).cos(), (PI / 4.0).sin()).scale(bw / 2.0);
    let s = p.mul(p).sub(C(wo2, 0.0)).sqrt();
    let fs2 = C(4.0, 0.0);
    let digital = |a: C| fs2.add(a).div(fs2.sub(a));
    let analog = [p.add(s), p.sub(s)];
    let mut den = C(1.0, 0.0);
    for a in analog {
        let c = C(a.0, -a.1);
        den = den.mul(fs2.sub(a)).mul(fs2.sub(c));
    }
    let k = bw * bw * C(16.0, 0.0).div(den).0;
    let mut pairs = analog.map(digital);
    pairs.sort_by(|a, b| a.abs2().total_cmp(&b.abs2()));
    // a double zero at z on the real axis
    let section = |z: f64| [1.0, -2.0 * z, z * z];
    // the pole pair nearest the circle takes the zero pair nearest it
    let near = pairs[1];
    let (z_far, z_near) = if near.sub(C(1.0, 0.0)).abs2() <= near.add(C(1.0, 0.0)).abs2() {
        (-1.0, 1.0)
    } else {
        (1.0, -1.0)
    };
    let [b0, b1, b2] = section(z_far);
    let [c0, c1, c2] = section(z_near);
    let den_of = |z: C| [-2.0 * z.0, z.abs2()];
    let [a1, a2] = den_of(pairs[0]);
    let [d1, d2] = den_of(pairs[1]);
    Some([
        [k * b0, k * b1, k * b2, 1.0, a1, a2],
        [c0, c1, c2, 1.0, d1, d2],
    ])
}

/// Steady-state section states for a unit step, scipy's `sosfilt_zi`.
fn sos_zi(sos: &[Sos]) -> Vec<[f64; 2]> {
    let mut scale = 1.0;
    let mut out = Vec::with_capacity(sos.len());
    for s in sos {
        let [b0, b1, b2, _, a1, a2] = *s;
        // (I - A^T) z = B for the transposed direct form II, 2 x 2 with
        // partial pivoting
        let (m00, m01, m10, m11) = (1.0 + a1, -1.0, a2, 1.0);
        let (r0, r1) = (b1 - a1 * b0, b2 - a2 * b0);
        let (z0, z1) = if m00.abs() >= m10.abs() {
            let l = m10 / m00;
            let z1 = (r1 - l * r0) / (m11 - l * m01);
            ((r0 - m01 * z1) / m00, z1)
        } else {
            let l = m00 / m10;
            let z1 = (r0 - l * r1) / (m01 - l * m11);
            ((r1 - m11 * z1) / m10, z1)
        };
        out.push([scale * z0, scale * z1]);
        scale *= (b0 + b1 + b2) / (1.0 + a1 + a2);
    }
    out
}

/// The cascade over `x`, transposed direct form II from `zi`.
fn sosfilt(sos: &[Sos], x: &[f64], zi: &[[f64; 2]]) -> Vec<f64> {
    let mut z = zi.to_vec();
    x.iter()
        .map(|&v| {
            let mut cur = v;
            for (s, st) in sos.iter().zip(z.iter_mut()) {
                let [b0, b1, b2, _, a1, a2] = *s;
                let y = b0 * cur + st[0];
                st[0] = b1 * cur - a1 * y + st[1];
                st[1] = b2 * cur - a2 * y;
                cur = y;
            }
            cur
        })
        .collect()
}

/// Rows of odd extension at each end, scipy's default for two sections.
pub const PADLEN: usize = 15;

/// Zero-phase filtering, scipy's `sosfiltfilt`: odd extension by
/// [`PADLEN`] rows at each end and step-matched initial states. None
/// unless `x` is longer than the extension.
pub fn filtfilt(sos: &[Sos], x: &[f64]) -> Option<Vec<f64>> {
    let n = x.len();
    if n <= PADLEN {
        return None;
    }
    let mut ext = Vec::with_capacity(n + 2 * PADLEN);
    ext.extend((0..PADLEN).map(|j| 2.0 * x[0] - x[PADLEN - j]));
    ext.extend_from_slice(x);
    ext.extend((0..PADLEN).map(|j| 2.0 * x[n - 1] - x[n - 2 - j]));
    let zi = sos_zi(sos);
    let scaled = |v: f64| -> Vec<[f64; 2]> { zi.iter().map(|z| [z[0] * v, z[1] * v]).collect() };
    let y = sosfilt(sos, &ext, &scaled(ext[0]));
    let rev: Vec<f64> = y.iter().rev().copied().collect();
    let mut y = sosfilt(sos, &rev, &scaled(rev[0]));
    y.reverse();
    Some(y[PADLEN..y.len() - PADLEN].to_vec())
}
