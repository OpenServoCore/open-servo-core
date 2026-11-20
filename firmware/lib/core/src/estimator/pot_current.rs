//! The position reading's current terms. Where the output gear rides on
//! the pot shaft, the pot reading shifts against the torque the output
//! gear carries, and the current reference stands in for that torque. Two terms on the reference, each a tuning
//! parameter in linearized pot counts per amp (CONFIG
//! `pot_steady_counts_per_a`, `pot_fast_counts_per_a`), go onto the
//! linearized sample ahead of the fusion observer:
//!
//! ```text
//! fast += (i_ref - fast) >> FAST_SHIFT     about 20 Hz at the 2 kHz medium rate
//! slow += (i_ref - slow) >> SLOW_SHIFT     about 64 ms
//! reading = pot + k_steady x slow + k_fast x (fast - slow)
//! ```
//!
//! The steady term credits a steady push; the fast term follows a current
//! change and returns to the pot as the slow average catches up. With both
//! terms 0 the reading is the pot's, bit for bit.
//!
//! The configuration take folds each k and the board's amps per count into
//! one gain ([`PotGains`]); the period that lands the reference steps the
//! averages and stores the finished term ([`PotCurrent::step`]), and the
//! next sample only adds it ([`PotCurrent::reading`]).
//!
//! The arithmetic is one i32 at the bounds. Both averages lie between the
//! references they averaged, so under the current band's `|i_ref| <=
//! u16::MAX` they stay under 2^28 at Q12, and the lead `fast - slow` under
//! 2^29. A gain is under 2^31 for k up to `POT_COUNTS_PER_A_MAX` and a
//! reciprocal up to [`AMPS_MAX_Q17`], so each widening product's high word
//! is under 2^27 and 2^28, and their sum under 2^29.

use crate::math::q_mul;
use crate::regions::config::POT_COUNTS_PER_A_MAX;

/// The fast average's corner, 2^-4 per medium period.
pub const FAST_SHIFT: u32 = 4;
/// The slow average's corner, 2^-7 per medium period.
pub const SLOW_SHIFT: u32 = 7;
/// The averages' fraction: the slow one moves at least one LSB a period on
/// any difference of 2^-5 current counts and up.
const AVG_Q: u32 = 12;
/// `KernelTiming::amps_per_count_q17`'s fraction.
pub const AMPS_Q: u32 = 17;
/// The reading's fraction (`phases::pos_q4`).
const POS_Q: u32 = 4;
/// The widening product keeps its high word.
const PROD_SHIFT: u32 = 32;
/// k [counts/A] x amps [Q17] << GAIN_SHIFT x I [Q12] >> 32 lands at Q4.
const GAIN_SHIFT: u32 = PROD_SHIFT + POS_Q - AMPS_Q - AVG_Q;

/// The current band's bound on the reference (`limits::IBand`: a u16
/// magnitude).
const I_REF_MAX: i64 = u16::MAX as i64;
/// The largest reciprocal whose gains stay one i32 at the bound on k: a
/// chain of 16 counts per amp or more.
pub const AMPS_MAX_Q17: u32 =
    (i32::MAX as i64 / ((POT_COUNTS_PER_A_MAX as i64) << GAIN_SHIFT)) as u32;
const GAIN_MAX: i64 = (POT_COUNTS_PER_A_MAX as i64 * AMPS_MAX_Q17 as i64) << GAIN_SHIFT;
const _: () = assert!(GAIN_MAX <= i32::MAX as i64);
const _: () = assert!((I_REF_MAX << AVG_Q) * 2 <= i32::MAX as i64);
const _: () = assert!(
    ((GAIN_MAX * (I_REF_MAX << AVG_Q)) >> PROD_SHIFT) * 3 + u16::MAX as i64 <= i32::MAX as i64
);

/// Amps per current count, Q17, from the board's sense chain: the inverse
/// of `current_counts`' counts per amp, `VDD / (gain x R_shunt x 4096)`,
/// rounded. 118 on the osc-dev-v006 board (1108.5 counts/A), 213 on the
/// arm-B rig (614.4). Const-eval only; a zero chain reads `u32::MAX`,
/// which [`amps_fits`] refuses.
pub const fn amps_per_count_q17(shunt_r_mohm: u16, gain_milli: u16, vdd_mv: u16) -> u32 {
    let num = (vdd_mv as u64 * 1000) << AMPS_Q;
    let den = gain_milli as u64 * shunt_r_mohm as u64 * 4096;
    if den == 0 {
        return u32::MAX;
    }
    let q = (num + den / 2) / den;
    if q > u32::MAX as u64 {
        u32::MAX
    } else {
        q as u32
    }
}

/// Whether a board's reciprocal keeps the gains one i32 ([`AMPS_MAX_Q17`]).
pub const fn amps_fits(amps_q17: u32) -> bool {
    amps_q17 <= AMPS_MAX_Q17
}

/// The two terms' gains: Q4 reading per Q12 current count, 2^-32, so a
/// term is one widening product's high word.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct PotGains {
    pub steady: i32,
    pub fast: i32,
}

impl PotGains {
    /// The take's gains from the stored k and the board's reciprocal, both
    /// held to their bounds, so the products stay one i32 whatever the
    /// table holds.
    pub fn new(k_steady: u16, k_fast: u16, amps_q17: u32) -> Self {
        let amps = amps_q17.min(AMPS_MAX_Q17) as i32;
        let gain = |k: u16| (k.min(POT_COUNTS_PER_A_MAX) as i32 * amps) << GAIN_SHIFT;
        Self {
            steady: gain(k_steady),
            fast: gain(k_fast),
        }
    }
}

/// The two averages of the current reference, current counts at Q12, and
/// the term they last made, Q4.
#[derive(Default)]
pub struct PotCurrent {
    fast: i32,
    slow: i32,
    term_q4: i32,
}

impl PotCurrent {
    pub const fn new() -> Self {
        Self {
            fast: 0,
            slow: 0,
            term_q4: 0,
        }
    }

    /// Both averages and the term at rest; the kernel calls it on the
    /// torque edge, with the reference.
    #[inline(always)]
    pub fn reset(&mut self) {
        self.fast = 0;
        self.slow = 0;
        self.term_q4 = 0;
    }

    /// One medium period on `i_ref`: both averages, then the term the next
    /// sample takes.
    #[inline(always)]
    pub fn step(&mut self, i_ref: i32, g: &PotGains) {
        let i = i_ref << AVG_Q;
        self.fast += (i - self.fast) >> FAST_SHIFT;
        self.slow += (i - self.slow) >> SLOW_SHIFT;
        self.term_q4 = q_mul(g.steady, self.slow, PROD_SHIFT)
            + q_mul(g.fast, self.fast - self.slow, PROD_SHIFT);
    }

    /// `pot_q4` with the stored term on it, saturated into the reading's
    /// range.
    #[inline(always)]
    pub fn reading(&self, pot_q4: u16) -> u16 {
        (pot_q4 as i32 + self.term_q4).max(0).min(u16::MAX as i32) as u16
    }

    /// The term the next sample takes, Q4.
    pub fn term_q4(&self) -> i32 {
        self.term_q4
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const AMPS: u32 = amps_per_count_q17(60, 14_884, 3300);
    const K_MAX: u16 = POT_COUNTS_PER_A_MAX;

    fn run(p: &mut PotCurrent, n: u32, pot: u16, i: i32, g: &PotGains) -> u16 {
        for _ in 0..n {
            p.step(i, g);
        }
        p.reading(pot)
    }

    #[test]
    fn the_reciprocal_follows_the_sense_chain() {
        assert_eq!(amps_per_count_q17(60, 14_884, 3300), 118);
        assert_eq!(amps_per_count_q17(33, 15_000, 3300), 213);
        assert_eq!(amps_per_count_q17(0, 14_884, 3300), u32::MAX);
        assert!(amps_fits(118) && amps_fits(213));
        assert!(!amps_fits(u32::MAX));
    }

    /// The largest reciprocal the bound passes, and the chain it stands for.
    #[test]
    fn the_fit_bound_sits_at_16_counts_per_amp() {
        assert!(amps_fits(AMPS_MAX_Q17) && !amps_fits(AMPS_MAX_Q17 + 1));
        assert_eq!(AMPS_MAX_Q17, 8388);
        assert_eq!((1u32 << AMPS_Q).div_ceil(AMPS_MAX_Q17), 16);
    }

    /// The take holds both inputs to their bounds.
    #[test]
    fn the_gains_hold_k_and_the_reciprocal_to_their_bounds() {
        let g = PotGains::new(u16::MAX, K_MAX, u32::MAX);
        assert_eq!(g.steady, GAIN_MAX as i32);
        assert_eq!(g.fast, GAIN_MAX as i32);
        assert_eq!(PotGains::new(0, 0, AMPS), PotGains::default());
        assert_eq!(
            PotGains::new(440, 0, AMPS).steady,
            (440 * 118) << GAIN_SHIFT
        );
    }

    /// Every swing between the reference's extremes, at both bounds and the
    /// largest reciprocal the bound passes, stays one i32 (a test build
    /// traps on overflow) and saturates at the reading's ends.
    #[test]
    fn the_extremes_stay_one_i32_and_saturate() {
        let lim = u16::MAX as i32;
        for (from, to) in [(lim, -lim), (-lim, lim), (lim, lim), (-lim, -lim)] {
            for ks in [0, K_MAX] {
                for kf in [0, K_MAX] {
                    let g = PotGains::new(ks, kf, AMPS_MAX_Q17);
                    let mut p = PotCurrent::new();
                    run(&mut p, 4096, 0, from, &g);
                    for _ in 0..4096 {
                        p.step(to, &g);
                        p.reading(32768);
                    }
                }
            }
        }
        let g = PotGains::new(K_MAX, 0, AMPS);
        let mut p = PotCurrent::new();
        assert_eq!(run(&mut p, 4096, 4095 << 4, lim, &g), u16::MAX);
        assert_eq!(run(&mut p, 4096, 0, -lim, &g), 0);
    }

    #[test]
    fn the_terms_at_zero_leave_the_reading_alone() {
        let g = PotGains::new(0, 0, AMPS);
        let mut p = PotCurrent::new();
        for (n, i) in [
            0,
            1,
            -1,
            554,
            -554,
            i16::MAX as i32,
            u16::MAX as i32,
            -(u16::MAX as i32),
        ]
        .into_iter()
        .cycle()
        .take(4096)
        .enumerate()
        {
            let pot = (n as u16).wrapping_mul(37);
            p.step(i, &g);
            assert_eq!(p.reading(pot), pot);
        }
    }

    /// Half an amp at 440 counts per amp: 220 counts, within the
    /// reciprocal's rounding and the products' floor.
    #[test]
    fn the_steady_term_settles_at_its_gain_times_the_current() {
        let half_amp = 554;
        let g = PotGains::new(440, 0, AMPS);
        for sign in [1, -1] {
            let mut p = PotCurrent::new();
            let r = run(&mut p, 4096, 2000 << 4, sign * half_amp, &g);
            let got = (r as i32 - (2000 << 4)) * sign;
            assert!((220 * 16 - 16..=220 * 16).contains(&got), "{sign}: {got}");
        }
    }
}
