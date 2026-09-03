//! Winding-resistance thermometer, SLOW rate, telemetry-only in v1 (nothing
//! consumes it yet). Copper's tempco turns the winding's DC resistance into a
//! temperature probe: a gated LMS tracks R from `v_mean` and `i_meas`, then
//! `t = t0 + k * (R - r0)` maps it through the CALIB anchor. The anchor is
//! never re-measured at boot - a hot reboot would record a warm winding as
//! ambient and bias the estimate low (control-theory "The Winding as a
//! Thermometer").

use crate::math::q_mul;

/// Gate thresholds from CONFIG thermal (`rtherm_*`). The gates live inside
/// the estimator, not the kernel: R only shows in `v_mean` when real current
/// flows and the back-EMF term is negligible, so refusing bad samples is part
/// of the estimate itself.
#[derive(Copy, Clone)]
pub struct ThermGates {
    pub i_min_counts: u16,
    pub omega_max_cps: u16,
}

/// CALIB winding anchor: cold resistance `r0_q12` (Q4.12 vcounts/ccount),
/// its ambient `t0_cc` (centi-degC), R-to-T slope `k_r2t_q88` (centi-degC per
/// Q4.12 LSB, Q8.8), LMS step `mu_q016` (Q0.16).
#[derive(Copy, Clone)]
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

/// R estimate + cached temperature. `seed` installs the CALIB `r0_q12`
/// (kernel install, and again on a calib rewrite); unseeded `step` is a
/// no-op reporting the anchor temperature.
#[derive(Default)]
pub struct WindingTherm {
    r_qg: i32,
    t_cc: i16,
    seeded: bool,
}

impl WindingTherm {
    pub const fn new() -> Self {
        Self {
            r_qg: 0,
            t_cc: 0,
            seeded: false,
        }
    }

    pub fn seed(&mut self, r0_q12: u16) {
        self.r_qg = (r0_q12 as i32) << GUARD;
        self.seeded = true;
    }

    /// One SLOW-tick update. `v_mean` per the bemf RECIP_ARR convention,
    /// `omega_hat_abs_cps` from the fusion observer. Gate: current present
    /// and above `i_min_counts`, speed below `omega_max_cps`; a refused
    /// sample holds R. Signed-LMS (sign-data variant): the update is
    /// `e_v * sgn(i)`, not `e_v * i`, so the step size stays mu-controlled
    /// independent of current magnitude. `t_cc` is re-derived from the held
    /// R every seeded step so an anchor rewrite lands without waiting for a
    /// gate pass. Returns the cached temperature.
    pub fn step(
        &mut self,
        v_mean: i32,
        i_meas: Option<i32>,
        omega_hat_abs_cps: u32,
        gates: &ThermGates,
        anchor: &ThermAnchor,
    ) -> i16 {
        if !self.seeded {
            self.t_cc = anchor.t0_cc;
            return self.t_cc;
        }
        if let Some(i) = i_meas
            && i.unsigned_abs() > gates.i_min_counts as u32
            && omega_hat_abs_cps < gates.omega_max_cps as u32
        {
            let r_drop = q_mul(self.r_qg >> GUARD, i, 12);
            let e_v = v_mean.saturating_sub(r_drop).clamp(-E_MAX, E_MAX);
            let e_signed = if i < 0 { -e_v } else { e_v };
            let step = q_mul((anchor.mu_q016 as i32) << GUARD, e_signed, 16);
            self.r_qg = (self.r_qg + step).clamp(0, R_MAX_QG);
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

    #[test]
    fn unseeded_step_is_noop() {
        let mut th = WindingTherm::new();
        for _ in 0..4 {
            assert_eq!(th.step(3000, Some(4096), 0, &GATES, &ANCHOR), 2500);
        }
        assert_eq!(th.r_q12(), 0);
        assert_eq!(th.t_cc(), 2500);
    }

    #[test]
    fn gate_refuses_bad_samples() {
        let mut th = WindingTherm::new();
        th.seed(ANCHOR.r0_q12);
        // low current, high speed, no current: R held at r0 -> t0
        assert_eq!(th.step(3000, Some(300), 0, &GATES, &ANCHOR), 2500);
        assert_eq!(th.step(3000, Some(4096), 400, &GATES, &ANCHOR), 2500);
        assert_eq!(th.step(3000, None, 0, &GATES, &ANCHOR), 2500);
        assert_eq!(th.r_q12(), ANCHOR.r0_q12);
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
        let mut t = 0i16;
        for _ in 0..2000 {
            t = th.step(v_mean, Some(4096), 0, &GATES, &ANCHOR);
        }
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
        for _ in 0..2000 {
            th.step(v_mean, Some(-4096), 0, &GATES, &ANCHOR);
        }
        assert_eq!(th.r_q12(), r_true as u16);
        assert_eq!(th.t_cc(), 3300);
    }

    #[test]
    fn hostile_mu_saturates_no_wrap() {
        let hot = ThermAnchor {
            mu_q016: u16::MAX,
            ..ANCHOR
        };
        let mut th = WindingTherm::new();
        th.seed(hot.r0_q12);
        for _ in 0..100 {
            th.step(i32::MAX, Some(301), 0, &GATES, &hot);
        }
        assert_eq!(th.r_q12(), u16::MAX);
        assert_eq!(th.t_cc(), i16::MAX);
        for _ in 0..100 {
            th.step(i32::MIN, Some(301), 0, &GATES, &hot);
        }
        assert_eq!(th.r_q12(), 0);
        // R clamped at 0: t = t0 - r0 * k >> 8 = 2500 - 24576
        assert_eq!(th.t_cc(), -22076);
    }

    #[test]
    fn cold_anchor_identity() {
        // R == r0 with a consistent v_mean: e_v = 0, t == t0 exactly
        let mut th = WindingTherm::new();
        th.seed(ANCHOR.r0_q12);
        let v_mean = q_mul(ANCHOR.r0_q12 as i32, 4096, 12);
        for _ in 0..8 {
            assert_eq!(th.step(v_mean, Some(4096), 0, &GATES, &ANCHOR), 2500);
        }
        assert_eq!(th.r_q12(), ANCHOR.r0_q12);
    }
}
