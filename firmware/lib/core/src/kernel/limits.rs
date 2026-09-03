//! i_lim composition (control-theory "Limits"): the current clamp is the one
//! choke point every command passes through, so all protection lives here.
//! i_lim = min of four terms - current_limit (gear teeth), thermal derate
//! foldback (copper), stall fold (the losing fight), directional endstop
//! (position walls). `fold` runs at MEDIUM and is compare/min only; the
//! derate ceiling needs a reciprocal, so it is recomputed at SLOW and cached.

use crate::math::q_mul_u;
use crate::regions::config::StallResponse;

/// CONFIG limits + thermal + pos-limits fields the block consumes, loaded
/// fresh by the kernel each step (`CurrentGains` convention).
#[derive(Copy, Clone)]
pub struct LimitCfg {
    pub current_limit_counts: u16,
    pub stall_response: StallResponse,
    pub drive_polarity: bool,
    pub stall_omega_max_cps: u16,
    /// Consecutive pinned-and-slow MEDIUM ticks before the stall trips. The
    /// table field is `stall_time_ms`; the kernel converts ms -> ticks once
    /// at config load, so the per-tick path never multiplies.
    pub stall_time_ticks: u32,
    pub stall_yield_counts: u16,
    pub stall_release_counts: u16,
    pub derate_start_cc: i16,
    pub cutoff_cc: i16,
    pub pos_min_soft_counts: i32,
    pub pos_max_soft_counts: i32,
}

// Q1.30 linear reciprocal seed r0 = 48/17 - (32/17)x for the normalized
// x in [0.5, 1): max relative error 1/17, so two Newton steps land at
// (1/17)^4 ~ 1.2e-5. The power-of-two seed (math::recip_seed_q15) can start
// a factor of 2 off, where r*(2 - r*d) convergence stalls - the default
// 2000cc derate span seeds at x = 1.95 and would need ~8 steps - so the
// one-shot derate uses the linear seed; the seed_q15 path stays for
// estimators that refine across ticks.
const SEED_C1_Q30: u32 = 3_031_741_621; // round(48/17 << 30)
const SEED_C2_Q30: u32 = 2_021_161_080; // round(32/17 << 30)

/// `num / d` without a divide. Normalize d into [2^31, 2^32) (Q0.32), linear
/// seed + two Newton steps `r' = r*(2 - r*d)` in Q1.30, then one widening
/// multiply denormalizes straight into the quotient. Error <= 1.2e-5
/// relative plus truncation (pinned in tests). d == 0 has no quotient;
/// callers gate the degenerate span, num is the no-panic backstop.
fn recip_div_u(num: u32, d: u32) -> u32 {
    if d <= 1 {
        return num;
    }
    let n = d.leading_zeros(); // 1..=30 for d >= 2
    let dn = d << n;
    let mut r = SEED_C1_Q30 - q_mul_u(SEED_C2_Q30, dn, 32);
    r = q_mul_u(r, (2u32 << 30) - q_mul_u(r, dn, 32), 30);
    r = q_mul_u(r, (2u32 << 30) - q_mul_u(r, dn, 32), 30);
    // num/d = num*r >> (62-n), split as (>> 32) then (>> 30-n): identical
    // bits, but the u64 shift stays constant and the variable shift is u32 -
    // a variable 64-bit shift is soft on rv32ec (check-soft-arith.sh bans it)
    let p = num as u64 * r as u64;
    ((p >> 32) as u32) >> (30 - n)
}

/// Stall timer + fold flag + the SLOW-cached derate ceiling. `derate_cache`
/// boots non-binding (u16::MAX) so a fresh state is not zero-current for the
/// first SLOW period; the base min still caps it at current_limit.
pub struct LimitState {
    stall_ticks: u32,
    stalled: bool,
    fault_pending: bool,
    derate_cache: u16,
    i_lim: u16,
}

impl LimitState {
    pub const fn new() -> Self {
        Self {
            stall_ticks: 0,
            stalled: false,
            fault_pending: false,
            derate_cache: u16::MAX,
            i_lim: 0,
        }
    }

    /// SLOW-rate: recompute the cached thermal ceiling. Foldback: full below
    /// derate_start_cc, zero at cutoff_cc, linear between - self-regulating
    /// (less current -> less heating -> equilibrium at "weaker but still
    /// holding"). A degenerate span (cutoff <= start, no table cross-gate)
    /// degrades to a step at cutoff. At 62.5 Hz the reciprocal's ~10
    /// multiplies are noise.
    pub fn update_derate(&mut self, t_winding_cc: i16, cfg: &LimitCfg) {
        let span = cfg.cutoff_cc as i32 - cfg.derate_start_cc as i32;
        self.derate_cache = if t_winding_cc >= cfg.cutoff_cc {
            0
        } else if t_winding_cc < cfg.derate_start_cc || span <= 0 {
            cfg.current_limit_counts
        } else {
            // head in [1, span], both <= 65535: num fits u32 exactly
            let head = (cfg.cutoff_cc as i32 - t_winding_cc as i32) as u32;
            let num = cfg.current_limit_counts as u32 * head;
            recip_div_u(num, span as u32).min(cfg.current_limit_counts as u32) as u16
        };
    }

    /// One MEDIUM-tick composition -> i_lim ccounts for this tick's push.
    /// Compare/min only, no multiplies. `i_ref_pinned` = the caller saw
    /// i_ref sitting at the ceiling (the pin, not a measurement);
    /// `i_sign_positive` is the sign of the command this limit clamps, and
    /// with `drive_polarity` resolves which endstop is "inward".
    ///
    /// Stall: pinned and |omega_hat| < stall_omega_max for stall_time_ticks,
    /// or a tau_d spike (collision). The v1 collision threshold is
    /// tau_d_abs > current_limit_counts - model-unexplained torque exceeding
    /// the commanded ceiling class - bench-tunable through the existing
    /// fields, no new config. Yield folds to stall_yield_counts until
    /// |tau_d| relaxes below stall_release_counts; Fault only pends -
    /// latching is kernel fault policy, and the kernel disables on latch, so
    /// folding here too would double-act.
    pub fn fold(
        &mut self,
        i_ref_pinned: bool,
        omega_hat_abs_cps: u32,
        tau_d_abs_counts: u16,
        theta_hat_counts: i32,
        i_sign_positive: bool,
        cfg: &LimitCfg,
    ) -> u16 {
        // release before the triggers: a relax re-arms a full stall window
        // instead of instantly re-tripping off the stale timer
        if self.stalled && tau_d_abs_counts < cfg.stall_release_counts {
            self.stalled = false;
            self.stall_ticks = 0;
        }
        let mut trip = tau_d_abs_counts > cfg.current_limit_counts;
        if i_ref_pinned && omega_hat_abs_cps < cfg.stall_omega_max_cps as u32 {
            self.stall_ticks = self.stall_ticks.saturating_add(1);
            trip |= self.stall_ticks >= cfg.stall_time_ticks;
        } else {
            self.stall_ticks = 0;
        }
        if trip {
            match cfg.stall_response {
                StallResponse::Yield => self.stalled = true,
                StallResponse::Fault => self.fault_pending = true,
            }
        }
        let mut lim = cfg.current_limit_counts.min(self.derate_cache);
        if self.stalled {
            lim = lim.min(cfg.stall_yield_counts);
        }
        // endstop: inward current at/past a soft limit clamps to 0, retreat
        // stays at the composed limit
        let pushing_up = i_sign_positive == cfg.drive_polarity;
        if (pushing_up && theta_hat_counts >= cfg.pos_max_soft_counts)
            || (!pushing_up && theta_hat_counts <= cfg.pos_min_soft_counts)
        {
            lim = 0;
        }
        self.i_lim = lim;
        lim
    }

    /// Cached output of the last `fold`.
    pub fn i_lim_counts(&self) -> u16 {
        self.i_lim
    }

    pub fn stalled(&self) -> bool {
        self.stalled
    }

    pub fn stall_fault_pending(&self) -> bool {
        self.fault_pending
    }
}

impl Default for LimitState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CFG: LimitCfg = LimitCfg {
        current_limit_counts: 1200,
        stall_response: StallResponse::Yield,
        drive_polarity: true,
        stall_omega_max_cps: 500,
        stall_time_ticks: 10,
        stall_yield_counts: 300,
        stall_release_counts: 150,
        derate_start_cc: 8000,
        cutoff_cc: 10000,
        pos_min_soft_counts: -4000,
        pos_max_soft_counts: 4000,
    };

    /// Mid-range, unpinned, no load: nothing but limit/derate can bind.
    fn free_fold(st: &mut LimitState, cfg: &LimitCfg) -> u16 {
        st.fold(false, 1000, 0, 0, true, cfg)
    }

    #[test]
    fn cold_state_current_limit_binds() {
        let mut st = LimitState::new();
        assert_eq!(free_fold(&mut st, &CFG), 1200);
        assert_eq!(st.i_lim_counts(), 1200);
        assert!(!st.stalled());
        assert!(!st.stall_fault_pending());
    }

    #[test]
    fn derate_endpoints_and_midpoint() {
        let mut st = LimitState::new();
        st.update_derate(7999, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 1200);
        // onset: head == span, quotient = current_limit up to truncation
        st.update_derate(8000, &CFG);
        assert!(free_fold(&mut st, &CFG) >= 1199);
        // midpoint -> half
        st.update_derate(9000, &CFG);
        let mid = free_fold(&mut st, &CFG);
        assert!((599..=600).contains(&mid), "mid={mid}");
        // cutoff and beyond -> zero
        st.update_derate(10000, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 0);
        st.update_derate(12000, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 0);
        // recovery is just the next SLOW recompute
        st.update_derate(2500, &CFG);
        assert_eq!(free_fold(&mut st, &CFG), 1200);
    }

    #[test]
    fn derate_degenerate_span_steps_at_cutoff() {
        let cfg = LimitCfg {
            derate_start_cc: 10000,
            ..CFG
        };
        let mut st = LimitState::new();
        st.update_derate(9999, &cfg);
        assert_eq!(free_fold(&mut st, &cfg), 1200);
        st.update_derate(10000, &cfg);
        assert_eq!(free_fold(&mut st, &cfg), 0);
    }

    #[test]
    fn recip_div_accuracy_pinned() {
        // worst seeds are just under a power of two (x -> 1 from below maps
        // to r near the interval edge); sweep those plus a dense band and
        // the u32-extreme numerator. Bound: 1.2e-5 relative + 2 LSB.
        let spot = [32767u32, 32768, 65534, 65535];
        for d in (1..=4096u32).chain(spot) {
            for num in [1u32, 999, 1_200 * 2_000, 4_294_836_225] {
                let got = recip_div_u(num, d);
                let exact = (num / d) as i64;
                let err = (got as i64 - exact).abs();
                assert!(
                    err <= exact / 65536 + 2,
                    "num={num} d={d} got={got} exact={exact}"
                );
            }
        }
    }

    #[test]
    fn stall_timer_folds_under_yield() {
        let mut st = LimitState::new();
        for _ in 0..9 {
            assert_eq!(st.fold(true, 0, 200, 0, true, &CFG), 1200);
            assert!(!st.stalled());
        }
        assert_eq!(st.fold(true, 0, 200, 0, true, &CFG), 300);
        assert!(st.stalled());
        assert!(!st.stall_fault_pending());
        assert_eq!(st.i_lim_counts(), 300);
    }

    #[test]
    fn movement_resets_timer() {
        let mut st = LimitState::new();
        for _ in 0..9 {
            st.fold(true, 0, 200, 0, true, &CFG);
        }
        // one fast tick clears the count; so does an unpinned one
        st.fold(true, 500, 200, 0, true, &CFG);
        for _ in 0..9 {
            assert_eq!(st.fold(true, 0, 200, 0, true, &CFG), 1200);
        }
        st.fold(false, 0, 200, 0, true, &CFG);
        for _ in 0..9 {
            assert_eq!(st.fold(true, 0, 200, 0, true, &CFG), 1200);
        }
        assert_eq!(st.fold(true, 0, 200, 0, true, &CFG), 300);
    }

    #[test]
    fn release_on_tau_d_relax_restores_and_rearms() {
        let mut st = LimitState::new();
        for _ in 0..10 {
            st.fold(true, 0, 200, 0, true, &CFG);
        }
        assert!(st.stalled());
        // tau_d still above release: stays folded
        assert_eq!(st.fold(false, 1000, 150, 0, true, &CFG), 300);
        // relax below release: restored, and a fresh window is required
        assert_eq!(st.fold(false, 1000, 149, 0, true, &CFG), 1200);
        assert!(!st.stalled());
        for _ in 0..9 {
            assert_eq!(st.fold(true, 0, 200, 0, true, &CFG), 1200);
        }
        assert_eq!(st.fold(true, 0, 200, 0, true, &CFG), 300);
    }

    #[test]
    fn collision_folds_immediately() {
        let mut st = LimitState::new();
        // unpinned and moving: only the tau_d spike path can trip
        assert_eq!(st.fold(false, 2000, 1201, 0, true, &CFG), 300);
        assert!(st.stalled());
        // at the threshold is not a spike
        let mut st = LimitState::new();
        assert_eq!(st.fold(false, 2000, 1200, 0, true, &CFG), 1200);
        assert!(!st.stalled());
    }

    #[test]
    fn fault_mode_pends_without_folding() {
        let cfg = LimitCfg {
            stall_response: StallResponse::Fault,
            ..CFG
        };
        let mut st = LimitState::new();
        for _ in 0..10 {
            assert_eq!(st.fold(true, 0, 200, 0, true, &cfg), 1200);
        }
        assert!(st.stall_fault_pending());
        assert!(!st.stalled());
        // collision pends too, still no fold
        let mut st = LimitState::new();
        assert_eq!(st.fold(false, 2000, 1201, 0, true, &cfg), 1200);
        assert!(st.stall_fault_pending());
        assert!(!st.stalled());
    }

    #[test]
    fn endstop_directional_all_combos() {
        let mut st = LimitState::new();
        // at max: inward (positive command, polarity true) clamps, retreat full
        assert_eq!(st.fold(false, 1000, 0, 4000, true, &CFG), 0);
        assert_eq!(st.fold(false, 1000, 0, 4000, false, &CFG), 1200);
        // mirrored at min
        assert_eq!(st.fold(false, 1000, 0, -4000, false, &CFG), 0);
        assert_eq!(st.fold(false, 1000, 0, -4000, true, &CFG), 1200);
        // past the wall keeps protecting
        assert_eq!(st.fold(false, 1000, 0, 5000, true, &CFG), 0);
        // reversed polarity flips which sign is inward
        let rev = LimitCfg {
            drive_polarity: false,
            ..CFG
        };
        assert_eq!(st.fold(false, 1000, 0, 4000, false, &rev), 0);
        assert_eq!(st.fold(false, 1000, 0, 4000, true, &rev), 1200);
        assert_eq!(st.fold(false, 1000, 0, -4000, true, &rev), 0);
        assert_eq!(st.fold(false, 1000, 0, -4000, false, &rev), 1200);
    }

    #[test]
    fn min_composition_tightest_term_wins() {
        // stalled yield 300 vs derate near cutoff: derate binds
        let mut st = LimitState::new();
        st.update_derate(9990, &CFG); // head 10/2000 -> ~6, way under yield
        for _ in 0..10 {
            st.fold(true, 0, 200, 0, true, &CFG);
        }
        assert!(st.stalled());
        let lim = st.fold(true, 0, 200, 0, true, &CFG);
        assert!((5..=6).contains(&lim), "lim={lim}");
        // endstop zero beats everything
        assert_eq!(st.fold(true, 0, 200, 4000, true, &CFG), 0);
    }
}
