//! The ripple's line, the tallest peak of a Hann-windowed spectrum zero
//! padded [`ZERO_PAD`] x, numpy's `rfft` and `hanning` ported.

use core::f64::consts::PI;

/// The spectrum's zero padding: a line read to 1/8 bin.
pub const ZERO_PAD: usize = 8;

/// numpy's `hanning(n)`, the symmetric window.
pub fn hanning(n: usize) -> Vec<f64> {
    if n <= 1 {
        return vec![1.0; n];
    }
    let m = (n - 1) as f64;
    (0..n)
        .map(|k| 0.5 + 0.5 * (PI * (2.0 * k as f64 + 1.0 - n as f64) / m).cos())
        .collect()
}

/// `(frequencies, amplitudes)` of `x` less its mean, Hann windowed and
/// zero padded; None when `x` is empty or carries a non-finite row.
pub fn spectrum(x: &[f64], fs: f64) -> Option<(Vec<f64>, Vec<f64>)> {
    let n = x.len();
    if n == 0 || !x.iter().all(|v| v.is_finite()) {
        return None;
    }
    let mean = x.iter().sum::<f64>() / n as f64;
    let w = hanning(n);
    let xw: Vec<f64> = x.iter().zip(&w).map(|(v, w)| (v - mean) * w).collect();
    let nfft = ZERO_PAD * n;
    let (re, im) = rfft(&xw, nfft);
    let val = 1.0 / (nfft as f64 * (1.0 / fs));
    let f = (0..re.len()).map(|k| k as f64 * val).collect();
    let amp = re.iter().zip(&im).map(|(r, i)| r.hypot(*i)).collect();
    Some((f, amp))
}

/// The frequency of the tallest line in `lo..=hi` Hz; None when no bin
/// lies in the band.
pub fn line_freq(x: &[f64], lo: f64, hi: f64, fs: f64) -> Option<f64> {
    let (f, amp) = spectrum(x, fs)?;
    let mut best: Option<usize> = None;
    for k in 0..f.len() {
        if f[k] >= lo && f[k] <= hi && best.is_none_or(|b| amp[k] > amp[b]) {
            best = Some(k);
        }
    }
    best.map(|k| f[k])
}

/// Bins `0..=nfft/2` of the DFT of `x` zero padded to `nfft`.
fn rfft(x: &[f64], nfft: usize) -> (Vec<f64>, Vec<f64>) {
    let mut re = vec![0.0; nfft];
    re[..x.len()].copy_from_slice(x);
    let mut im = vec![0.0; nfft];
    if nfft.is_power_of_two() {
        fft_pow2(&mut re, &mut im, false);
    } else {
        bluestein(&mut re, &mut im);
    }
    re.truncate(nfft / 2 + 1);
    im.truncate(nfft / 2 + 1);
    (re, im)
}

/// In-place radix-2 FFT; `inverse` conjugates the twiddles and scales by 1/n.
fn fft_pow2(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    if n < 2 {
        return;
    }
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let sign = if inverse { 1.0 } else { -1.0 };
    let tw: Vec<(f64, f64)> = (0..n / 2)
        .map(|k| {
            let a = sign * 2.0 * PI * k as f64 / n as f64;
            (a.cos(), a.sin())
        })
        .collect();
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let stride = n / len;
        for start in (0..n).step_by(len) {
            for k in 0..half {
                let (wr, wi) = tw[k * stride];
                let (a, b) = (start + k, start + k + half);
                let xr = re[b] * wr - im[b] * wi;
                let xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
        len <<= 1;
    }
    if inverse {
        let s = 1.0 / n as f64;
        for k in 0..n {
            re[k] *= s;
            im[k] *= s;
        }
    }
}

/// In-place DFT of any length as a convolution with a chirp, three
/// power-of-2 FFTs of at least twice the length.
fn bluestein(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let m = (2 * n - 1).next_power_of_two();
    let chirp: Vec<(f64, f64)> = (0..n)
        .map(|k| {
            let q = ((k as u64 * k as u64) % (2 * n as u64)) as f64;
            let a = -PI * q / n as f64;
            (a.cos(), a.sin())
        })
        .collect();
    let (mut ar, mut ai) = (vec![0.0; m], vec![0.0; m]);
    let (mut br, mut bi) = (vec![0.0; m], vec![0.0; m]);
    for k in 0..n {
        let (cr, ci) = chirp[k];
        ar[k] = re[k] * cr - im[k] * ci;
        ai[k] = re[k] * ci + im[k] * cr;
        br[k] = cr;
        bi[k] = -ci;
        if k > 0 {
            br[m - k] = cr;
            bi[m - k] = -ci;
        }
    }
    fft_pow2(&mut ar, &mut ai, false);
    fft_pow2(&mut br, &mut bi, false);
    for k in 0..m {
        let r = ar[k] * br[k] - ai[k] * bi[k];
        let i = ar[k] * bi[k] + ai[k] * br[k];
        ar[k] = r;
        ai[k] = i;
    }
    fft_pow2(&mut ar, &mut ai, true);
    for k in 0..n {
        let (cr, ci) = chirp[k];
        re[k] = ar[k] * cr - ai[k] * ci;
        im[k] = ar[k] * ci + ai[k] * cr;
    }
}
