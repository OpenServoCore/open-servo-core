//! The board NTC two ways: the part's nominal beta equation through the
//! board's pull-up divider, the truth, and the fixed-point quadratic the
//! kernel carries (core estimator/thermal.rs `NtcCfg`), designed from the
//! board's parts by the stage 1 rule (notebooks `oscnb/ntc.py`
//! `fit_pinned`): about a reference count, exact at a pin temperature, never
//! further under the truth than a floor over a range, and inside that the
//! least over-read over a low stretch. The notebook solves it as a linear
//! program; here the same program is solved by its structure: the pin
//! fixes the constant, so it is a convex problem in the slope and the
//! curvature alone.

use osc_servo_core::estimator::thermal::{NtcCfg, ntc_cc};

pub const ADC_COUNTS: f64 = 4096.0;
const KELVIN: f64 = 273.15;
const T25_K: f64 = 298.15;
/// The deviation's unit inside the fit, counts: conditioning only.
const FIT_SCALE: f64 = 1000.0;
/// Past every slope `ntc_k1_q88` holds, in fit units.
const B_WIDE: f64 = 2.0 * 32768.0 / 256.0 * FIT_SCALE;

/// The NTC's parts as the servo's read-only CALIB names them.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Parts {
    pub pullup_ohm: f64,
    pub r25_ohm: f64,
    pub beta: f64,
}

impl Parts {
    /// The count the NTC reads at `t_c` by the beta equation, pull-up to VDD.
    pub fn raw_of_c(&self, t_c: f64) -> f64 {
        let r = self.r25_ohm * (self.beta * (1.0 / (t_c + KELVIN) - 1.0 / T25_K)).exp();
        ADC_COUNTS * r / (self.pullup_ohm + r)
    }

    /// The temperature a count means by the beta equation, C.
    pub fn c_of_raw(&self, raw: f64) -> f64 {
        let r = self.pullup_ohm * raw / (ADC_COUNTS - raw);
        1.0 / (1.0 / T25_K + (r / self.r25_ohm).ln() / self.beta) - KELVIN
    }
}

/// The design's choices: stage 1's (`notebooks stages/1-board/ntc-curve`
/// results `design_*`).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Design {
    pub raw_ref: u16,
    /// The true temperature the curve reads exactly: the cutoff.
    pub pin_c: f64,
    pub range_c: (f64, f64),
    /// The most the curve may read under the truth over the range, C (< 0).
    pub floor_c: f64,
    pub low_range_c: (f64, f64),
    pub step_c: f64,
    /// The curvature's field top, Q24 (`ntc_k2_q24` is a u16).
    pub k2_top_q24: f64,
}

pub const STAGE_1: Design = Design {
    raw_ref: 2048,
    pin_c: 90.0,
    range_c: (15.0, 90.0),
    floor_c: -1.0,
    low_range_c: (15.0, 50.0),
    step_c: 0.1,
    k2_top_q24: 65535.0,
};

/// The four CALIB words.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Words {
    pub raw_ref: u16,
    pub t_ref_cc: i16,
    pub k1_q88: i16,
    pub k2_q24: u16,
}

impl Words {
    pub fn cfg(&self) -> NtcCfg {
        NtcCfg {
            raw_ref: self.raw_ref,
            t_ref_cc: self.t_ref_cc,
            k1_q88: self.k1_q88,
            k2_q24: self.k2_q24,
        }
    }

    /// The curve's worst residual against the truth at every whole count
    /// of `range_c`, `(under, over)` in C, as the kernel reads each count.
    pub fn form_error(&self, parts: &Parts, range_c: (f64, f64)) -> (f64, f64) {
        let (hi_raw, lo_raw) = (parts.raw_of_c(range_c.0), parts.raw_of_c(range_c.1));
        let mut worst = (f64::INFINITY, f64::NEG_INFINITY);
        for raw in lo_raw.ceil() as u16..=hi_raw.floor() as u16 {
            if let Some(t) = ntc_cc(raw, &self.cfg()) {
                let e = t as f64 / 100.0 - parts.c_of_raw(raw as f64);
                worst = (worst.0.min(e), worst.1.max(e));
            }
        }
        worst
    }
}

/// `start + k x step` for k up to past `stop`, as numpy's `arange` makes
/// the notebook's grid.
fn grid(lo: f64, hi: f64, step: f64) -> Vec<f64> {
    let n = ((hi + 1e-6 - lo) / step).ceil() as usize;
    (0..n).map(|k| lo + k as f64 * step).collect()
}

/// A grid point: its deviation in fit units and its truth, cC.
struct Point {
    x: f64,
    truth: f64,
}

struct Program {
    pin: Point,
    held: Vec<Point>,
    low: Vec<Point>,
    floor_cc: f64,
    c_top: f64,
}

impl Program {
    fn points(parts: &Parts, d: &Design, lo: f64, hi: f64) -> Vec<Point> {
        grid(lo, hi, d.step_c)
            .into_iter()
            .map(|t| Point {
                x: (d.raw_ref as f64 - parts.raw_of_c(t)) / FIT_SCALE,
                truth: 100.0 * t,
            })
            .collect()
    }

    /// The constant the pin gives the slope `b` and the curvature `c`.
    fn a(&self, b: f64, c: f64) -> f64 {
        self.pin.truth - b * self.pin.x - c * self.pin.x * self.pin.x
    }

    fn residual(&self, p: &Point, b: f64, c: f64) -> f64 {
        self.a(b, c) + b * p.x + c * p.x * p.x - p.truth
    }

    /// The slopes that keep every held point over the floor at `c`.
    fn slopes(&self, c: f64) -> (f64, f64) {
        let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
        for p in &self.held {
            let dx = p.x - self.pin.x;
            let at = self.residual(p, 0.0, c);
            if dx.abs() < 1e-15 {
                if at < -self.floor_cc {
                    return (1.0, 0.0);
                }
                continue;
            }
            let b = (-self.floor_cc - at) / dx;
            if dx > 0.0 {
                lo = lo.max(b);
            } else {
                hi = hi.min(b);
            }
        }
        (lo, hi)
    }

    /// The objective: the largest over-read over the low stretch, never
    /// under 0, cC.
    fn over(&self, b: f64, c: f64) -> f64 {
        self.low
            .iter()
            .map(|p| self.residual(p, b, c))
            .fold(0.0, f64::max)
    }

    /// The best slope at `c` and its objective; None where no slope keeps
    /// the floor.
    fn best_b(&self, c: f64) -> Option<(f64, f64)> {
        let (lo, hi) = self.slopes(c);
        if lo > hi {
            return None;
        }
        // a side the floor leaves open ends where k1 leaves its field
        let (lo, hi) = (lo.max(-B_WIDE), hi.min(B_WIDE));
        let b = golden(lo, hi, |b| self.over(b, c));
        Some((b, self.over(b, c)))
    }
}

/// The minimum of a convex function on `[lo, hi]`, by golden section.
fn golden(mut lo: f64, mut hi: f64, f: impl Fn(f64) -> f64) -> f64 {
    let r = (5f64.sqrt() - 1.0) / 2.0;
    let mut x1 = hi - r * (hi - lo);
    let mut x2 = lo + r * (hi - lo);
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..200 {
        if hi - lo <= 1e-12 * (1.0 + lo.abs().max(hi.abs())) {
            break;
        }
        if f1 <= f2 {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - r * (hi - lo);
            f1 = f(x1);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + r * (hi - lo);
            f2 = f(x2);
        }
    }
    (lo + hi) / 2.0
}

/// The first `c` in `[lo, hi]` where `ok` turns, `ok(lo) != ok(hi)`.
fn edge(mut lo: f64, mut hi: f64, ok: impl Fn(f64) -> bool) -> f64 {
    let want = ok(hi);
    for _ in 0..200 {
        let mid = (lo + hi) / 2.0;
        if ok(mid) == want {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi
}

/// The four words for `parts` under `design`, each rounded to nearest
/// (half to even, as the notebook rounds), or why no quadratic meets it.
pub fn design(parts: &Parts, d: &Design) -> Result<Words, String> {
    let pin = Program::points(parts, d, d.pin_c, d.pin_c).remove(0);
    let p = Program {
        pin,
        held: Program::points(parts, d, d.range_c.0, d.range_c.1),
        low: Program::points(parts, d, d.low_range_c.0, d.low_range_c.1),
        floor_cc: -100.0 * d.floor_c,
        c_top: d.k2_top_q24 / (1u32 << 24) as f64 * FIT_SCALE * FIT_SCALE,
    };
    let refused = || {
        format!(
            "no quadratic about count {} with its curvature inside 0..{} (Q24) reads the beta equation exactly at {} C and stays no more than {} C under it from {} to {} C",
            d.raw_ref, d.k2_top_q24, d.pin_c, -d.floor_c, d.range_c.0, d.range_c.1
        )
    };
    // the floor's slack is convex in c: its least point, then the interval
    // of curvatures it allows
    let slack = |c: f64| {
        let (lo, hi) = p.slopes(c);
        lo - hi
    };
    let tightest = golden(0.0, p.c_top, slack);
    if slack(tightest) > 0.0 {
        return Err(refused());
    }
    let ok = |c: f64| slack(c) <= 0.0;
    let c_lo = if ok(0.0) {
        0.0
    } else {
        edge(0.0, tightest, ok)
    };
    let c_hi = if ok(p.c_top) {
        p.c_top
    } else {
        edge(p.c_top, tightest, ok)
    };
    let c = golden(c_lo, c_hi, |c| {
        p.best_b(c).map_or(f64::INFINITY, |(_, v)| v)
    });
    let (b, _) = p.best_b(c).ok_or_else(refused)?;
    let a = p.a(b, c);
    let words = (
        a.round_ties_even(),
        (b / FIT_SCALE * 256.0).round_ties_even(),
        (c / (FIT_SCALE * FIT_SCALE) * (1u32 << 24) as f64).round_ties_even(),
    );
    let fits = |v: f64, lo: f64, hi: f64| (lo..=hi).contains(&v);
    if !fits(words.0, i16::MIN as f64, i16::MAX as f64)
        || !fits(words.1, i16::MIN as f64, i16::MAX as f64)
        || !fits(words.2, 0.0, u16::MAX as f64)
    {
        return Err(refused());
    }
    Ok(Words {
        raw_ref: d.raw_ref,
        t_ref_cc: words.0 as i16,
        k1_q88: words.1 as i16,
        k2_q24: words.2 as u16,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEV: Parts = Parts {
        pullup_ohm: 10_000.0,
        r25_ohm: 10_000.0,
        beta: 3950.0,
    };

    /// The dev board's parts give stage 1's design words, the board file's
    /// and the set of record's (2048, 2430, 474, 19934), and their form
    /// error is the notebook's (-1.02 to 5.50 C over 15 to 90 C).
    #[test]
    fn the_dev_boards_parts_give_stage_1s_words() {
        let w = design(&DEV, &STAGE_1).expect("designs");
        assert_eq!(
            w,
            Words {
                raw_ref: 2048,
                t_ref_cc: 2430,
                k1_q88: 474,
                k2_q24: 19934
            }
        );
        let (under, over) = w.form_error(&DEV, STAGE_1.range_c);
        assert!(
            (under + 1.0197).abs() < 0.01 && (over - 5.4981).abs() < 0.01,
            "{under} {over}"
        );
        assert!((DEV.c_of_raw(DEV.raw_of_c(37.5)) - 37.5).abs() < 1e-9);
    }

    #[test]
    fn a_design_no_quadratic_meets_is_refused() {
        // pinned mid-range as a straight line, no curve meets a tight floor
        let d = Design {
            pin_c: 50.0,
            floor_c: -0.001,
            k2_top_q24: 0.0,
            ..STAGE_1
        };
        assert!(design(&DEV, &d).is_err());
    }
}
