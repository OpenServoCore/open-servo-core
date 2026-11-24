//! The TEL stream's own load ripples the chip supply once a 16-row frame,
//! and every channel the ADC reads against VDD follows it. A steady channel
//! folded by tick modulo 16 over a whole capture reads that ripple as a
//! 16-point pattern in its own counts, and each channel's share of it is
//! the least-squares gain of the channel's own folded pattern on that one.
//! A row's own reading is too noisy to correct that row by; the folded
//! pattern is not. A pattern a channel carries that does not follow VDD is
//! left in its residual.

use super::rows::TelFrame;

/// Rows a telemetry frame batches: the ripple's period, in ticks.
pub const FOLD: usize = 16;

pub type Pattern = [f64; FOLD];

/// A channel's `(tick, value)` over a capture, the rows that carry it.
pub fn series(rows: &[TelFrame], name: &str) -> Vec<(u64, f64)> {
    rows.iter()
        .filter_map(|f| Some((f.tick, f.channel(name)?)))
        .collect()
}

/// The tick-modulo-16 means, their own mean removed; None unless every
/// phase has a row.
pub fn fold(s: &[(u64, f64)]) -> Option<Pattern> {
    let mut sums = [(0.0, 0usize); FOLD];
    for (t, x) in s {
        let b = &mut sums[(*t % FOLD as u64) as usize];
        b.0 += x;
        b.1 += 1;
    }
    if sums.iter().any(|(_, k)| *k == 0) {
        return None;
    }
    let mut p = sums.map(|(s, k)| s / k as f64);
    let mean = p.iter().sum::<f64>() / FOLD as f64;
    for x in &mut p {
        *x -= mean;
    }
    Some(p)
}

/// The least-squares gain of `channel` on `pattern`; 0 on a flat pattern.
pub fn gain(channel: &Pattern, pattern: &Pattern) -> f64 {
    let pp: f64 = pattern.iter().map(|v| v * v).sum();
    if pp == 0.0 {
        return 0.0;
    }
    channel.iter().zip(pattern).map(|(c, p)| c * p).sum::<f64>() / pp
}

/// `s` with `gain` x `pattern` taken out of every row at its phase.
pub fn compensate(s: &[(u64, f64)], pattern: &Pattern, gain: f64) -> Vec<(u64, f64)> {
    s.iter()
        .map(|&(t, x)| (t, x - gain * pattern[(t % FOLD as u64) as usize]))
        .collect()
}

/// A channel's gain on `pattern` and its rows compensated; gain 0 and the
/// rows as they are when the channel misses a phase.
pub fn fit(s: &[(u64, f64)], pattern: &Pattern) -> (f64, Vec<(u64, f64)>) {
    match fold(s) {
        Some(p) => {
            let g = gain(&p, pattern);
            (g, compensate(s, pattern, g))
        }
        None => (0.0, s.to_vec()),
    }
}

/// A pattern's peak to peak.
pub fn pp(p: &Pattern) -> f64 {
    let hi = p.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let lo = p.iter().copied().fold(f64::INFINITY, f64::min);
    hi - lo
}

/// The supply's frame ripple peak to peak, mV, from the 1.2 V reference's
/// pattern: VDD reads the reference lower as it rises,
/// `dVDD = -VDD x dvref / vref_mean`.
pub fn vdd_ripple_mv_pp(vref: &Pattern, vref_mean: f64, vdd_v: f64) -> f64 {
    1000.0 * vdd_v * pp(vref) / vref_mean
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A VDD frame ripple of `amp` counts on the reference.
    fn wave(k: usize) -> f64 {
        let x = 2.0 * core::f64::consts::PI * k as f64 / FOLD as f64;
        x.cos() + 0.5 * (2.0 * x).sin()
    }

    fn noise(i: u64, k: u64) -> f64 {
        let mut h =
            (i.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ k.wrapping_mul(0xC2B2_AE3D_27D4_EB4F)) >> 11;
        h ^= h >> 17;
        (h % 1000) as f64 / 1000.0 - 0.5
    }

    fn sd(s: &[(u64, f64)]) -> f64 {
        let n = s.len() as f64;
        let m = s.iter().map(|x| x.1).sum::<f64>() / n;
        (s.iter().map(|x| (x.1 - m).powi(2)).sum::<f64>() / n).sqrt()
    }

    fn max_dev(p: &Pattern) -> f64 {
        p.iter().map(|x| x.abs()).fold(0.0, f64::max)
    }

    /// Every channel follows the reference's ripple at its own gain: the
    /// gains come back within 5%, the residuals are flat and quieter.
    #[test]
    fn a_vdd_frame_ripple_comes_out_of_every_channel() {
        let rows = 20_000u64;
        let vref: Vec<(u64, f64)> = (0..rows)
            .map(|i| (i, 1490.0 - 1.5 * wave(i as usize % FOLD) + noise(i, 0)))
            .collect();
        let pv = fold(&vref).expect("every phase");
        for (k, g) in [(1u64, 1.15), (2, 1.59), (3, 0.73), (4, -0.4)] {
            let ch: Vec<(u64, f64)> = (0..rows)
                .map(|i| {
                    (
                        i,
                        2000.0 + g * (vref[i as usize].1 - 1490.0 - noise(i, 0)) + noise(i, k),
                    )
                })
                .collect();
            let before = fold(&ch).expect("phases");
            assert!(max_dev(&before) > 0.5 * g.abs(), "the ripple shows");
            let (got, comp) = fit(&ch, &pv);
            assert!((got - g).abs() <= 0.05 * g.abs(), "gain {got} for {g}");
            assert!(max_dev(&fold(&comp).expect("phases")) < 0.05, "flat");
            assert!(sd(&comp) < sd(&ch), "quieter: {} {}", sd(&comp), sd(&ch));
        }
        let pp = vdd_ripple_mv_pp(&pv, 1490.0, 3.3);
        let want = 1000.0 * 3.3 * 1.5 * (pv_range()) / 1490.0;
        assert!((pp - want).abs() < 0.05 * want, "{pp} {want}");
    }

    fn pv_range() -> f64 {
        let w: Vec<f64> = (0..FOLD).map(wave).collect();
        w.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - w.iter().copied().fold(f64::INFINITY, f64::min)
    }

    /// A frame pattern that does not follow the reference stays in the
    /// residual, whole.
    #[test]
    fn a_pattern_unlike_vdds_stays() {
        let vref: Vec<(u64, f64)> = (0..4096u64).map(|i| (i, wave(i as usize % FOLD))).collect();
        let pv = fold(&vref).expect("phases");
        let other = |k: usize| (2.0 * core::f64::consts::PI * 3.0 * k as f64 / FOLD as f64).sin();
        let ch: Vec<(u64, f64)> = (0..4096u64)
            .map(|i| (i, 3.0 * other(i as usize % FOLD)))
            .collect();
        let (g, comp) = fit(&ch, &pv);
        assert!(g.abs() < 1e-9, "{g}");
        assert!((max_dev(&fold(&comp).expect("phases")) - 3.0).abs() < 1e-6);
        assert_eq!(
            gain(&pv, &[0.0; FOLD]),
            0.0,
            "a flat pattern takes nothing out"
        );
    }
}
