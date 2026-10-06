//! Winding-resistance thermometer, SLOW rate. Copper's tempco turns the
//! winding's DC resistance into a temperature probe: a gated LMS tracks R
//! from `v_mean`, `i_meas` and the observer's speed, then
//! `t = t0 + k * (R - r0)` maps it through the CALIB anchor. The anchor is
//! never re-measured at boot - a hot reboot would record a warm winding as
//! ambient and bias the estimate low (control-theory "The Winding as a
//! Thermometer").
//!
//! R shows in `v_mean` only at a held operating point. A moving motor puts
//! its back-EMF there, and the pot observer cannot vouch for a motor that
//! turns inside the gear lash (one output count is a dozen motor degrees),
//! so the gate asks the drive itself: the same `(v_mean, i)` pair, within
//! an eighth, for `STEADY_TICKS` consecutive SLOW periods. A reversal
//! regime never holds one (the bench shake heater, flipping every 24 to
//! 70 ms, passed 0 of 45202 polls; a seated hold passed every one). Without
//! a sample the estimate coasts toward the anchor on the can's time
//! constant, so a dormant estimate never latches a derate.

use crate::math::q_mul;

/// Gate thresholds from CONFIG thermal (`rtherm_*`). The gates live inside
/// the estimator, not the kernel: refusing bad samples is part of the
/// estimate itself.
#[derive(Copy, Clone, Default)]
pub struct ThermGates {
    pub i_min_counts: u16,
    /// Bound on the speed whose back-EMF the model subtracts: beyond it the
    /// observer's share of the balance is not trusted.
    pub omega_max_cps: u16,
}

/// CALIB winding anchor: cold resistance `r0_q12` (Q4.12 vcounts/ccount),
/// its ambient `t0_cc` (centi-degC), R-to-T slope `k_r2t_q88` (centi-degC per
/// Q4.12 LSB, Q8.8), LMS step `mu_q016` (Q0.16).
#[derive(Copy, Clone, Default)]
pub struct ThermAnchor {
    pub r0_q12: u16,
    pub t0_cc: i16,
    pub k_r2t_q88: u16,
    pub mu_q016: u16,
}

/// Guard bits under Q4.12 so sub-LSB LMS steps accumulate instead of
/// truncating to zero (bemf `state_qg` convention, more bits because mu is
/// small). R full-scale 16.0 << GUARD stays well inside i32.
const GUARD: u32 = 8;
const R_MAX_QG: i32 = 16 << (12 + GUARD);

/// `e_v` beyond this is garbage input (realistic magnitude < 2^17); the clamp
/// bounds `mu << GUARD * e_v` inside the q_mul contract for any caller values.
const E_MAX: i32 = 1 << 20;

/// A sample is steady when neither `v_mean` nor `i` moved by more than
/// this fraction of itself (a shift) since the previous SLOW tick. A seated
/// hold sits within 2 counts of 225; a reversal moves both by their whole
/// size; a motor spinning up inside the lash under a held current moves
/// `v_mean` by its back-EMF, tens of vcounts per tick.
const STEADY_SHIFT: u32 = 3;
/// Consecutive steady ticks before a sample counts: the operating point has
/// held for two SLOW periods (32 ms), longer than the pot observer's 40 ms
/// settle has left to lag by and longer than any reversal regime holds one.
const STEADY_TICKS: u8 = 2;

/// Per-tick bound on the LMS step as a fraction of `r0` (a shift): 0.1% of
/// R per SLOW tick, 16 C/s of copper. A hard stall at the overcurrent trip
/// (2.2 A into a micro-class winding) heats it at most ~18 C/s and the trip
/// latches first; a locked hold at the limit heats under 1 C/s. So the
/// bound never holds back a real rise, and a tick that slipped through the
/// gate (observer lag at a move's onset or arrival) moves R by at most one
/// step, not by its whole error.
const SLEW_SHIFT: u32 = 10;

/// Coast toward the anchor without a sample: `(r0 - r) >> COAST_SHIFT` per
/// SLOW tick, a 2^13-tick time constant, 131 s at 62.5 Hz. That is the
/// can-to-air mode of the bench MG90 (129 s, notebook 18), the slowest way
/// a winding sheds heat, so the dormant estimate cools no faster than the
/// copper can.
const COAST_SHIFT: u32 = 13;

/// R estimate + cached temperature. `seed` installs the CALIB `r0_q12`
/// (kernel install, and again on a calib rewrite); unseeded `step` is a
/// no-op reporting the anchor temperature.
#[derive(Default)]
pub struct WindingTherm {
    r_qg: i32,
    t_cc: i16,
    seeded: bool,
    /// Previous SLOW tick's accepted-or-not operating point, for the
    /// steadiness test, and how many consecutive ticks it has held.
    v_prev: i32,
    i_prev: i32,
    steady: u8,
}

impl WindingTherm {
    pub const fn new() -> Self {
        Self {
            r_qg: 0,
            t_cc: 0,
            seeded: false,
            v_prev: 0,
            i_prev: 0,
            steady: 0,
        }
    }

    pub fn seed(&mut self, r0_q12: u16) {
        self.r_qg = (r0_q12 as i32) << GUARD;
        self.seeded = true;
    }

    /// One SLOW-tick update. `v_mean` per the bemf RECIP_ARR convention,
    /// `omega_hat_cps` the pot observer's signed speed, `ke_vpc_q` the
    /// motor's forward Ke (Q4.12 vcounts per c/s). Gate: current present
    /// and above `i_min_counts`, speed inside `omega_max_cps`, and the
    /// operating point steady for `STEADY_TICKS`. An accepted sample runs
    /// the signed LMS on `e = v_mean - Ke omega - R i` (sign-data variant:
    /// the update is `e * sgn(i)`, so the step stays mu-controlled
    /// independent of current magnitude), bounded to one slew step; a
    /// refused tick coasts R toward the anchor. `t_cc` is re-derived every
    /// seeded step so an anchor rewrite lands at once. Returns the cached
    /// temperature.
    pub fn step(
        &mut self,
        v_mean: i32,
        i_meas: Option<i32>,
        omega_hat_cps: i32,
        ke_vpc_q: u16,
        gates: &ThermGates,
        anchor: &ThermAnchor,
    ) -> i16 {
        if !self.seeded {
            self.t_cc = anchor.t0_cc;
            return self.t_cc;
        }
        let r0_qg = (anchor.r0_q12 as i32) << GUARD;
        let sample = match i_meas {
            Some(i) => {
                let held = (v_mean.wrapping_sub(self.v_prev)).unsigned_abs()
                    <= v_mean.unsigned_abs() >> STEADY_SHIFT
                    && (i.wrapping_sub(self.i_prev)).unsigned_abs()
                        <= i.unsigned_abs() >> STEADY_SHIFT;
                self.steady = if held {
                    self.steady.saturating_add(1)
                } else {
                    0
                };
                self.v_prev = v_mean;
                self.i_prev = i;
                (self.steady >= STEADY_TICKS
                    && i.unsigned_abs() > gates.i_min_counts as u32
                    && omega_hat_cps.unsigned_abs() <= gates.omega_max_cps as u32)
                    .then_some(i)
            }
            None => {
                self.steady = 0;
                None
            }
        };
        if let Some(i) = sample {
            let bemf = q_mul(ke_vpc_q as i32, omega_hat_cps, 12);
            let r_drop = q_mul(self.r_qg >> GUARD, i, 12);
            let e_v = v_mean
                .saturating_sub(bemf)
                .saturating_sub(r_drop)
                .clamp(-E_MAX, E_MAX);
            let e_signed = if i < 0 { -e_v } else { e_v };
            let slew = r0_qg >> SLEW_SHIFT;
            let step = q_mul((anchor.mu_q016 as i32) << GUARD, e_signed, 16).clamp(-slew, slew);
            self.r_qg = (self.r_qg + step).clamp(0, R_MAX_QG);
        } else {
            self.r_qg += (r0_qg - self.r_qg) >> COAST_SHIFT;
        }
        let dt = q_mul(
            (self.r_qg >> GUARD) - anchor.r0_q12 as i32,
            anchor.k_r2t_q88 as i32,
            8,
        );
        let t = anchor.t0_cc as i32 + dt;
        self.t_cc = t.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        self.t_cc
    }

    /// Saturating cast matching telemetry `est.r_hat_q12`.
    pub fn r_q12(&self) -> u16 {
        (self.r_qg >> GUARD).clamp(0, u16::MAX as i32) as u16
    }

    /// Cached output of the last `step`.
    pub fn t_cc(&self) -> i16 {
        self.t_cc
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GATES: ThermGates = ThermGates {
        i_min_counts: 300,
        omega_max_cps: 400,
    };
    // r0 3.0 Q4.12, 25.00C, 2.00 cc per Q12 LSB, mu ~0.1
    const ANCHOR: ThermAnchor = ThermAnchor {
        r0_q12: 12288,
        t0_cc: 2500,
        k_r2t_q88: 512,
        mu_q016: 6554,
    };
    /// The bench MG90 anchor `osc ident anchor` wrote on 2S: r0 4170 at
    /// 26.50 C, copper's handbook line through it, mu 7670 for a 2 s settle
    /// at the 280 limit; the servo's Ke 350 (0.0854 vcounts per c/s) and
    /// the 167-count sampling floor.
    const BENCH: ThermAnchor = ThermAnchor {
        r0_q12: 4170,
        t0_cc: 2650,
        k_r2t_q88: 1602,
        mu_q016: 7670,
    };
    const BENCH_GATES: ThermGates = ThermGates {
        i_min_counts: 167,
        omega_max_cps: 400,
    };
    const BENCH_KE: u16 = 350;
    /// SLOW ticks per second at the 20 kHz tick.
    const SLOW_HZ: usize = 62;

    /// A still shaft with no back-EMF term.
    fn still(th: &mut WindingTherm, v: i32, i: i32, g: &ThermGates, a: &ThermAnchor) -> i16 {
        th.step(v, Some(i), 0, 0, g, a)
    }

    /// `n` ticks of one operating point.
    fn hold(
        th: &mut WindingTherm,
        n: usize,
        v: i32,
        i: i32,
        g: &ThermGates,
        a: &ThermAnchor,
    ) -> i16 {
        let mut t = 0;
        for _ in 0..n {
            t = still(th, v, i, g, a);
        }
        t
    }

    #[test]
    fn unseeded_step_is_noop() {
        let mut th = WindingTherm::new();
        for _ in 0..4 {
            assert_eq!(still(&mut th, 3000, 4096, &GATES, &ANCHOR), 2500);
        }
        assert_eq!(th.r_q12(), 0);
        assert_eq!(th.t_cc(), 2500);
    }

    #[test]
    fn gate_refuses_bad_samples() {
        let mut th = WindingTherm::new();
        th.seed(ANCHOR.r0_q12);
        // low current, high speed, no current: R held at r0 -> t0, however
        // long the point holds
        assert_eq!(hold(&mut th, 8, 3000, 300, &GATES, &ANCHOR), 2500);
        for _ in 0..8 {
            assert_eq!(th.step(3000, Some(4096), 401, 0, &GATES, &ANCHOR), 2500);
        }
        for _ in 0..8 {
            assert_eq!(th.step(3000, None, 0, 0, &GATES, &ANCHOR), 2500);
        }
        assert_eq!(th.r_q12(), ANCHOR.r0_q12);
    }

    /// The first two ticks of any operating point are refused: a sample
    /// counts once the same `(v, i)` has held for `STEADY_TICKS` periods.
    #[test]
    fn a_sample_needs_a_held_operating_point() {
        let mut th = WindingTherm::new();
        th.seed(ANCHOR.r0_q12);
        let (v, i) = (q_mul(12688, 4096, 12), 4096);
        for _ in 0..STEADY_TICKS {
            still(&mut th, v, i, &GATES, &ANCHOR);
            assert_eq!(th.r_q12(), ANCHOR.r0_q12);
        }
        still(&mut th, v, i, &GATES, &ANCHOR);
        assert!(th.r_q12() > ANCHOR.r0_q12);
        // a step in current re-arms the count: the next two ticks are
        // refused again and R only coasts (a guard bit toward the anchor)
        let r = th.r_q12();
        still(&mut th, v, i / 2, &GATES, &ANCHOR);
        still(&mut th, v, i / 2, &GATES, &ANCHOR);
        assert!(r - th.r_q12() <= 1, "r {} -> {}", r, th.r_q12());
    }

    #[test]
    fn converges_to_warm_r() {
        // R_true = r0 + 400; i = 4096 makes v_mean = R_true exactly, so the
        // LMS equilibrium is exact. t = 2500 + (12688 - 12288) * 512 >> 8
        //   = 2500 + 400 * 2 = 3300 (33.00C).
        let r_true = 12688i32;
        let v_mean = q_mul(r_true, 4096, 12);
        assert_eq!(v_mean, r_true);
        let mut th = WindingTherm::new();
        th.seed(ANCHOR.r0_q12);
        let t = hold(&mut th, 2000, v_mean, 4096, &GATES, &ANCHOR);
        assert_eq!(th.r_q12(), r_true as u16);
        assert_eq!(t, 3300);
        assert_eq!(th.t_cc(), 3300);
    }

    #[test]
    fn negative_current_converges_identically() {
        let r_true = 12688i32;
        let v_mean = q_mul(r_true, -4096, 12);
        assert_eq!(v_mean, -r_true);
        let mut th = WindingTherm::new();
        th.seed(ANCHOR.r0_q12);
        hold(&mut th, 2000, v_mean, -4096, &GATES, &ANCHOR);
        assert_eq!(th.r_q12(), r_true as u16);
        assert_eq!(th.t_cc(), 3300);
    }

    /// Hostile inputs saturate at the R clamps and never wrap; the slew
    /// bound meters the walk there at `r0 >> SLEW_SHIFT` per tick.
    #[test]
    fn hostile_mu_slews_to_the_clamps_no_wrap() {
        let hot = ThermAnchor {
            mu_q016: u16::MAX,
            ..ANCHOR
        };
        let slew = (ANCHOR.r0_q12 >> SLEW_SHIFT) as i32;
        assert_eq!(slew, 12);
        let mut th = WindingTherm::new();
        th.seed(hot.r0_q12);
        hold(&mut th, STEADY_TICKS as usize, i32::MAX, 301, &GATES, &hot);
        for n in 1..=100 {
            still(&mut th, i32::MAX, 301, &GATES, &hot);
            assert_eq!(th.r_q12() as i32, ANCHOR.r0_q12 as i32 + n * slew);
        }
        hold(&mut th, 10_000, i32::MAX, 301, &GATES, &hot);
        assert_eq!(th.r_q12(), u16::MAX);
        assert_eq!(th.t_cc(), i16::MAX);
        hold(&mut th, 10_000, i32::MIN, 301, &GATES, &hot);
        assert_eq!(th.r_q12(), 0);
        // R clamped at 0: t = t0 - r0 * k >> 8 = 2500 - 24576
        assert_eq!(th.t_cc(), -22076);
    }

    /// Fed the R of a winding 40 C over the anchor at the limit's current,
    /// the LMS closes most of the gap in one 2 s settle, settles inside the
    /// one-vcount resolution `v_mean` has at that current (15 LSB of R,
    /// 0.3%), and reads the rise back within a degree.
    #[test]
    fn copper_anchor_reads_the_rise_through_the_handbook_slope() {
        let i = 280;
        // 4170 x (1 + 40 / 260.73) (anchor at 26.5 C, 234.5 + 26.5 = 261)
        let r_hot = 4810i32;
        let v_mean = q_mul(r_hot, i, 12);
        let mut th = WindingTherm::new();
        th.seed(BENCH.r0_q12);
        hold(&mut th, 2 * SLOW_HZ, v_mean, i, &BENCH_GATES, &BENCH);
        let gap = r_hot - th.r_q12() as i32;
        assert!((0..330).contains(&gap), "after one settle the gap is {gap}");
        let t = hold(&mut th, 1000, v_mean, i, &BENCH_GATES, &BENCH);
        assert!(
            (r_hot - th.r_q12() as i32).abs() <= 15,
            "r_hat {} for {r_hot}",
            th.r_q12()
        );
        assert!((t - 6650).abs() <= 100, "reads {t} cc for 66.50 C");
    }

    #[test]
    fn cold_anchor_identity() {
        // R == r0 with a consistent v_mean: e_v = 0, t == t0 exactly
        let mut th = WindingTherm::new();
        th.seed(ANCHOR.r0_q12);
        let v_mean = q_mul(ANCHOR.r0_q12 as i32, 4096, 12);
        for _ in 0..8 {
            assert_eq!(still(&mut th, v_mean, 4096, &GATES, &ANCHOR), 2500);
        }
        assert_eq!(th.r_q12(), ANCHOR.r0_q12);
    }

    /// The bench shake heater (servo-pass/anchor/heat): Position-mode goal
    /// steps the shaft never reaches, the duty reversing every 24 to 70 ms
    /// around +/-150 counts while the motor winds up inside the gear lash
    /// with the pot nearly still. The logged duty x vdiff over i ran 1.4 to
    /// 1.8 x R through each half cycle (back-EMF the pot observer could not
    /// see), and the thermometer that sampled it read +24% of R, 89 C on a
    /// 32 C winding, and derated. Modelled tick by tick at the SLOW rate:
    /// a kick tick as the current reverses against the old motion, then the
    /// drive climbing as the motor spins up, then the flip. The pair never
    /// holds, so nothing is sampled: R coasts at the anchor and the reading
    /// stays the anchor's temperature through 20 s of shaking.
    #[test]
    fn shake_heater_is_refused_and_reads_the_anchor() {
        let mut th = WindingTherm::new();
        th.seed(BENCH.r0_q12);
        // (v_mean, i) per tick of one half cycle, then the sign flips: the
        // kick reads under R i (back-EMF against the drive), the climb
        // +45..+55% over it (R i at 170 counts is 173 vcounts), both over
        // the 167 floor and inside the 400 c/s gate - the old gate took
        // the climb ticks
        let half: [(i32, i32); 3] = [(160, 182), (250, 170), (268, 168)];
        let mut t = 0;
        for n in 0..20 * SLOW_HZ {
            let (v, i) = half[n % 3];
            let s = if (n / 3) % 2 == 0 { 1 } else { -1 };
            t = th.step(s * v, Some(s * i), s * 300, BENCH_KE, &BENCH_GATES, &BENCH);
        }
        assert_eq!(th.r_q12(), BENCH.r0_q12);
        assert_eq!(t, BENCH.t0_cc);
    }

    /// The seated hold the anchor was read in (12% duty at the low stop,
    /// 225 counts, i within +/-2 counts tick to tick): every tick is steady
    /// and the LMS reads the hold's R inside the one-vcount resolution
    /// `v_mean` has at that current (18 LSB of R, 1.1 C), with the 2 s
    /// settle the anchor sized mu for.
    #[test]
    fn seated_hold_samples_every_tick() {
        let mut th = WindingTherm::new();
        th.seed(BENCH.r0_q12);
        // the winding 10 C over the anchor: 4170 x (1 + 10 / 261)
        let r_warm = 4330i32;
        let mut t = 0;
        for n in 0..10 * SLOW_HZ {
            let i = -225 + [0, 1, -1, 2][n % 4];
            let v = q_mul(r_warm, i, 12);
            t = th.step(
                v,
                Some(i),
                [3, -5, 0, 2][n % 4],
                BENCH_KE,
                &BENCH_GATES,
                &BENCH,
            );
        }
        assert!(
            (r_warm - th.r_q12() as i32).abs() <= 18,
            "r_hat {}",
            th.r_q12()
        );
        assert!((t - 3650).abs() <= 115, "reads {t} cc for 36.50 C");
    }

    /// A loaded crawl: the output turning steadily at 300 c/s under 250
    /// counts. The pot observer sees it (the lash is taken up), the model
    /// subtracts its back-EMF (26 vcounts, a tenth of R i) and the LMS lands
    /// on R; unsubtracted it would read +10%, 26 C.
    #[test]
    fn steady_motion_subtracts_the_observed_back_emf() {
        let mut th = WindingTherm::new();
        th.seed(BENCH.r0_q12);
        let (r_true, i, omega) = (4250i32, 250, 300);
        let v = q_mul(r_true, i, 12) + q_mul(BENCH_KE as i32, omega, 12);
        for _ in 0..10 * SLOW_HZ {
            th.step(v, Some(i), omega, BENCH_KE, &BENCH_GATES, &BENCH);
        }
        assert!(
            (r_true - th.r_q12() as i32).abs() <= 10,
            "r_hat {}",
            th.r_q12()
        );
    }

    /// A tick the gate let through with the full back-EMF of a motor at
    /// 1500 c/s in it (130 vcounts on a 230-vcount drive) moves R by one
    /// slew step, 0.1%, a quarter degree - not by the 15% it claims.
    #[test]
    fn a_leaked_tick_moves_r_by_at_most_one_slew_step() {
        let mut th = WindingTherm::new();
        th.seed(BENCH.r0_q12);
        let i = 225;
        let v = q_mul(BENCH.r0_q12 as i32, i, 12);
        hold(&mut th, 8, v, i, &BENCH_GATES, &BENCH);
        assert_eq!(th.r_q12(), BENCH.r0_q12);
        let leak = v + q_mul(BENCH_KE as i32, 1500, 12);
        // the point must look held to be sampled: three ticks of it
        hold(
            &mut th,
            STEADY_TICKS as usize + 1,
            leak,
            i,
            &BENCH_GATES,
            &BENCH,
        );
        let slew = (BENCH.r0_q12 >> SLEW_SHIFT) as i32;
        assert_eq!(slew, 4);
        assert_eq!(th.r_q12() as i32, BENCH.r0_q12 as i32 + slew);
    }

    /// Sampling stops (torque off, or the derate pulled the current under
    /// the floor) with the estimate 24% over the anchor, as the bench left
    /// it at 89 C: the excess coasts toward the anchor with a 2^13-tick
    /// time constant (131 s), so the reading is under the 80 C derate
    /// onset within 30 s and within 2 C of the anchor after five time
    /// constants, instead of holding 89 C until a power cycle. A cold
    /// estimate coasts up toward the anchor the same way.
    #[test]
    fn a_dormant_estimate_coasts_to_the_anchor() {
        let mut th = WindingTherm::new();
        th.seed(BENCH.r0_q12);
        let excess = 1000i32;
        let v = q_mul(BENCH.r0_q12 as i32 + excess, 280, 12);
        hold(&mut th, 20 * SLOW_HZ, v, 280, &BENCH_GATES, &BENCH);
        let r_hot = th.r_q12() as i32;
        assert!(
            (r_hot - BENCH.r0_q12 as i32 - excess).abs() <= 15,
            "r_hat {r_hot}"
        );
        assert!(th.t_cc() > 8800, "starts hot: {}", th.t_cc());
        let coast = |th: &mut WindingTherm, n: usize| {
            let mut t = 0;
            for _ in 0..n {
                t = th.step(0, None, 0, BENCH_KE, &BENCH_GATES, &BENCH);
            }
            t
        };
        assert!(
            coast(&mut th, 30 * SLOW_HZ) < 8000,
            "still derating: {}",
            th.t_cc()
        );
        let left = th.r_q12() as i32 - BENCH.r0_q12 as i32;
        // 30 s is 0.23 time constants: e^-0.23 = 0.80 of the excess
        assert!((770..=815).contains(&left), "excess after 30 s: {left}");
        coast(&mut th, (1 << COAST_SHIFT) - 30 * SLOW_HZ);
        let left = th.r_q12() as i32 - BENCH.r0_q12 as i32;
        // one time constant: e^-1 = 0.37 (the floor rounding shaves a little)
        assert!((330..=375).contains(&left), "excess after one tau: {left}");
        let t = coast(&mut th, 4 << COAST_SHIFT);
        assert!((t - BENCH.t0_cc).abs() <= 200, "reads {t} cc at rest");
        // from below: a cold estimate rises toward the anchor
        th.seed(BENCH.r0_q12 - 500);
        coast(&mut th, 5 << COAST_SHIFT);
        assert!(
            BENCH.r0_q12 as i32 - (th.r_q12() as i32) <= 32,
            "cold left {}",
            th.r_q12()
        );
    }
}
