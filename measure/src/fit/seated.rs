//! The winding's R and L read seated: shunt bursts from rest with the
//! rotor against a stop, each read whole by the period-pair fit with no
//! intercept, `V_n = R i_n + L di_n / T` ([`super::rise::fit`]), as the
//! stage 3 identification notebook reads its bursts at a held seat. The
//! first burst of each group is set aside, and so is a burst whose first
//! window current is more than [`SET_ASIDE_SHARE`] off the median of its
//! group's others (the inductance notebook's rule); the rest pool by
//! medians.

use crate::capture::burst::Capture;
use crate::rules::units::ALPHA_CU;

use super::median;
use super::rise::{self, Rise, Scales};

/// A burst whose first window current is this share off its group's
/// others is set aside.
pub const SET_ASIDE_SHARE: f64 = 0.12;
/// The temperature R is also stated at, C.
pub const R_AT_C: f64 = 25.0;

/// One seated burst and where it was taken.
#[derive(Clone, Debug, PartialEq)]
pub struct Shot {
    /// The bursts of one level at one seating: the first of a group is
    /// set aside.
    pub group: usize,
    /// 0 the low stop, 1 the high.
    pub stop: usize,
    /// 0 the lower duty, 1 the higher.
    pub level: usize,
    /// The board NTC read just before the seating's bursts, C.
    pub ntc_c: f64,
    pub cap: Capture,
}

/// Why a burst is not in the pool.
#[derive(Clone, Debug, PartialEq)]
pub enum Aside {
    First,
    /// Its first window current against its group's others, a share.
    Moved(f64),
    Unread(String),
}

impl Aside {
    pub fn said(&self) -> String {
        match self {
            Aside::First => "the first of its group".into(),
            Aside::Moved(m) => format!(
                "its first window current is {:+.1}% off its group's others, over {:.0}%",
                100.0 * m,
                100.0 * SET_ASIDE_SHARE
            ),
            Aside::Unread(e) => format!("unread: {e}"),
        }
    }
}

/// One burst's read.
#[derive(Clone, Debug, PartialEq)]
pub struct BurstRead {
    pub group: usize,
    pub stop: usize,
    pub level: usize,
    /// Its place in its group, from 0.
    pub k: usize,
    pub ntc_c: f64,
    pub step_q15: i16,
    pub pos: u16,
    pub i_first_a: f64,
    pub move_share: f64,
    pub fit: Option<Rise>,
    pub aside: Option<Aside>,
}

impl BurstRead {
    pub fn kept(&self) -> bool {
        self.aside.is_none() && self.fit.is_some()
    }

    /// R through copper's slope at [`R_AT_C`], from the NTC of its seating.
    pub fn r25_ohm(&self) -> Option<f64> {
        self.fit.as_ref().map(|f| at_25(f.r_ohm, self.ntc_c))
    }
}

/// R at `t_c` stated at 25 C: the inverse of the kernel's copper slope.
pub fn at_25(r_ohm: f64, t_c: f64) -> f64 {
    r_ohm / (1.0 + ALPHA_CU * (t_c - R_AT_C))
}

/// A median and its spread over a set of bursts.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Spread {
    pub median: f64,
    pub n: usize,
    /// The sample sd over the median; NaN under two bursts.
    pub sd_share: f64,
    pub lo: f64,
    pub hi: f64,
}

pub fn spread(v: &[f64]) -> Option<Spread> {
    let m = median(v)?;
    Some(Spread {
        median: m,
        n: v.len(),
        sd_share: sd(v) / m,
        lo: v.iter().copied().fold(f64::INFINITY, f64::min),
        hi: v.iter().copied().fold(f64::NEG_INFINITY, f64::max),
    })
}

fn sd(v: &[f64]) -> f64 {
    if v.len() < 2 {
        return f64::NAN;
    }
    let m = v.iter().sum::<f64>() / v.len() as f64;
    (v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (v.len() - 1) as f64).sqrt()
}

/// Every burst read, and the PWM period they ran at.
#[derive(Clone, Debug, PartialEq)]
pub struct SeatedRead {
    pub bursts: Vec<BurstRead>,
    pub period_s: f64,
}

/// The bridge's sign of a burst in the wiring's frame: its logical step
/// through the `drive_polarity` it ran under.
pub fn wiring_sign(cap: &Capture) -> f64 {
    (cap.meta.step_q15 as f64).signum() * if cap.meta.drive_polarity { 1.0 } else { -1.0 }
}

/// Each shot read by its period table and the period-pair fit, the set
/// aside rule applied by group.
pub fn read(shots: &[Shot], s: &Scales) -> SeatedRead {
    let period_s = shots
        .first()
        .map_or(f64::NAN, |x| rise::pwm_period_s(x.cap.meta.pwm_arr));
    let mut out: Vec<BurstRead> = Vec::with_capacity(shots.len());
    let mut seen: Vec<(usize, usize)> = Vec::new();
    for x in shots {
        let k = match seen.iter_mut().find(|(g, _)| *g == x.group) {
            Some((_, n)) => {
                *n += 1;
                *n - 1
            }
            None => {
                seen.push((x.group, 1));
                0
            }
        };
        let (fit, i_first, aside) = match rise::periods(&x.cap, wiring_sign(&x.cap), s, 0.0) {
            Ok(t) => match rise::fit(&t, period_s, s.window_scale) {
                Some(f) => (Some(f), t[0].i_a, None),
                None => (
                    None,
                    t[0].i_a,
                    Some(Aside::Unread("too few periods to fit".into())),
                ),
            },
            Err(e) => (None, f64::NAN, Some(Aside::Unread(e))),
        };
        out.push(BurstRead {
            group: x.group,
            stop: x.stop,
            level: x.level,
            k,
            ntc_c: x.ntc_c,
            step_q15: x.cap.meta.step_q15,
            pos: x.cap.meta.pos,
            i_first_a: i_first,
            move_share: f64::NAN,
            fit,
            aside: aside.or((k == 0).then_some(Aside::First)),
        });
    }
    for j in 0..out.len() {
        let others: Vec<f64> = out
            .iter()
            .enumerate()
            .filter(|(i, b)| *i != j && b.group == out[j].group && b.i_first_a.is_finite())
            .map(|(_, b)| b.i_first_a)
            .collect();
        let m = median(&others).map_or(f64::NAN, |m| out[j].i_first_a / m - 1.0);
        out[j].move_share = m;
        if out[j].aside.is_none() && m.abs() > SET_ASIDE_SHARE {
            out[j].aside = Some(Aside::Moved(m));
        }
    }
    SeatedRead {
        bursts: out,
        period_s,
    }
}

impl SeatedRead {
    pub fn kept(&self) -> impl Iterator<Item = &BurstRead> {
        self.bursts.iter().filter(|b| b.kept())
    }

    fn of(
        &self,
        keep: impl Fn(&BurstRead) -> bool,
        f: impl Fn(&BurstRead) -> f64,
    ) -> Option<Spread> {
        spread(&self.kept().filter(|b| keep(b)).map(f).collect::<Vec<_>>())
    }

    fn rise_of(b: &BurstRead) -> &Rise {
        // kept bursts carry a fit
        b.fit.as_ref().expect("a kept burst has a fit")
    }

    pub fn r(&self, keep: impl Fn(&BurstRead) -> bool) -> Option<Spread> {
        self.of(keep, |b| Self::rise_of(b).r_ohm)
    }

    pub fn l(&self, keep: impl Fn(&BurstRead) -> bool) -> Option<Spread> {
        self.of(keep, |b| Self::rise_of(b).l_h)
    }

    pub fn tau(&self, keep: impl Fn(&BurstRead) -> bool) -> Option<Spread> {
        self.of(keep, |b| Self::rise_of(b).tau_s)
    }

    pub fn r25(&self, keep: impl Fn(&BurstRead) -> bool) -> Option<Spread> {
        self.of(keep, |b| at_25(Self::rise_of(b).r_ohm, b.ntc_c))
    }

    /// The burst-to-burst sd of R inside the groups, over the pooled
    /// median: the median of each group's sample sd, two kept bursts or
    /// more a group. The levels and the stops do not enter it.
    pub fn r_sd_within_groups(&self) -> Option<f64> {
        let pooled = self.r(|_| true)?.median;
        let mut groups: Vec<usize> = self.kept().map(|b| b.group).collect();
        groups.dedup();
        let sds: Vec<f64> = groups
            .iter()
            .map(|g| {
                sd(&self
                    .kept()
                    .filter(|b| b.group == *g)
                    .map(|b| Self::rise_of(b).r_ohm)
                    .collect::<Vec<_>>())
            })
            .filter(|s| s.is_finite())
            .collect();
        median(&sds).map(|m| m / pooled)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn r_at_25_c_inverts_the_copper_slope() {
        assert_eq!(at_25(4.0, 25.0), 4.0);
        let hot = 4.0 * (1.0 + ALPHA_CU * 10.0);
        assert!((at_25(hot, 35.0) - 4.0).abs() < 1e-12);
        assert!((ALPHA_CU - 0.0039).abs() < 0.0002, "{ALPHA_CU}");
    }

    #[test]
    fn the_spread_is_the_sample_sd_over_the_median() {
        let s = spread(&[1.0, 2.0, 3.0]).expect("three");
        assert_eq!((s.median, s.n, s.lo, s.hi), (2.0, 3, 1.0, 3.0));
        assert!((s.sd_share - 0.5).abs() < 1e-12);
        assert!(spread(&[2.0]).expect("one").sd_share.is_nan());
        assert!(spread(&[]).is_none());
    }
}
