//! i_lim composition (control-theory "Limits"): the current clamp is the one
//! choke point every command passes through, so all protection lives here.
//! i_lim = min of four terms - current_limit (gear teeth), thermal derate
//! foldback (copper), stall fold (the losing fight), directional endstop
//! (position walls). `fold` runs at MEDIUM and is compare/min only; the
//! derate ceiling needs a reciprocal, so it is recomputed at SLOW and cached.

use crate::math::recip_div;
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

/// The signed current band every command clamps into: `lo <= i_ref <= hi`.
/// Symmetric `+-i_lim` away from the walls; a soft-limit wall collapses its
/// inward side to 0 while the outward side keeps the composed limit.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct IBand {
    pub lo: i32,
    pub hi: i32,
}

impl IBand {
    pub fn clamp(&self, i: i32) -> i32 {
        i.clamp(self.lo, self.hi)
    }
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
            recip_div(num, span as u32).min(cfg.current_limit_counts as u32) as u16
        };
    }

    /// One MEDIUM-tick composition -> the signed current band `[lo, hi]`
    /// every command clamps into. Compare/min only, no multiplies.
    /// `i_ref_pinned` = the caller saw i_ref sitting at the ceiling (the
    /// pin, not a measurement).
    ///
    /// The endstop is a DIRECTIONAL bound, not a magnitude fold: at a soft
    /// limit only the inward side collapses to 0 and retreat keeps the
    /// composed limit. Folding a magnitude gated by the command's sign
    /// deadlocks at the wall - the clamp zeroes i_ref, zero reads as an
    /// inward push, and the door never reopens (caught on the bench).
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
        cfg: &LimitCfg,
    ) -> IBand {
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
        self.i_lim = lim;
        // endstop: with drive_polarity true, positive current moves counts
        // up, so the max wall zeroes hi and the min wall zeroes lo; an
        // inverted polarity swaps which side each wall owns
        let mut band = IBand {
            lo: -(lim as i32),
            hi: lim as i32,
        };
        let (at_max, at_min) = (
            theta_hat_counts >= cfg.pos_max_soft_counts,
            theta_hat_counts <= cfg.pos_min_soft_counts,
        );
        if at_max {
            if cfg.drive_polarity {
                band.hi = 0;
            } else {
                band.lo = 0;
            }
        }
        if at_min {
            if cfg.drive_polarity {
                band.lo = 0;
            } else {
                band.hi = 0;
            }
        }
        band
    }

    /// Torque-enable ack: drop the pending fault, unfold, re-arm a full
    /// stall window. The derate cache survives - it is thermal state, not a
    /// latch, and clearing it would hand a hot winding one SLOW period of
    /// full current.
    pub fn ack(&mut self) {
        self.fault_pending = false;
        self.stalled = false;
        self.stall_ticks = 0;
    }

    /// Cached symmetric magnitude of the last `fold` (torque limit, derate,
    /// stall folds - the endstop is positional and lives in the band's
    /// sides, not here). This is what telemetry publishes.
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
        st.fold(false, 1000, 0, 0, cfg).hi as u16
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
    fn stall_timer_folds_under_yield() {
        let mut st = LimitState::new();
        for _ in 0..9 {
            assert_eq!(st.fold(true, 0, 200, 0, &CFG).hi, 1200);
            assert!(!st.stalled());
        }
        assert_eq!(st.fold(true, 0, 200, 0, &CFG).hi, 300);
        assert!(st.stalled());
        assert!(!st.stall_fault_pending());
        assert_eq!(st.i_lim_counts(), 300);
    }

    #[test]
    fn movement_resets_timer() {
        let mut st = LimitState::new();
        for _ in 0..9 {
            st.fold(true, 0, 200, 0, &CFG);
        }
        // one fast tick clears the count; so does an unpinned one
        st.fold(true, 500, 200, 0, &CFG);
        for _ in 0..9 {
            assert_eq!(st.fold(true, 0, 200, 0, &CFG).hi, 1200);
        }
        st.fold(false, 0, 200, 0, &CFG);
        for _ in 0..9 {
            assert_eq!(st.fold(true, 0, 200, 0, &CFG).hi, 1200);
        }
        assert_eq!(st.fold(true, 0, 200, 0, &CFG).hi, 300);
    }

    #[test]
    fn release_on_tau_d_relax_restores_and_rearms() {
        let mut st = LimitState::new();
        for _ in 0..10 {
            st.fold(true, 0, 200, 0, &CFG);
        }
        assert!(st.stalled());
        // tau_d still above release: stays folded
        assert_eq!(st.fold(false, 1000, 150, 0, &CFG).hi, 300);
        // relax below release: restored, and a fresh window is required
        assert_eq!(st.fold(false, 1000, 149, 0, &CFG).hi, 1200);
        assert!(!st.stalled());
        for _ in 0..9 {
            assert_eq!(st.fold(true, 0, 200, 0, &CFG).hi, 1200);
        }
        assert_eq!(st.fold(true, 0, 200, 0, &CFG).hi, 300);
    }

    #[test]
    fn collision_folds_immediately() {
        let mut st = LimitState::new();
        // unpinned and moving: only the tau_d spike path can trip
        assert_eq!(st.fold(false, 2000, 1201, 0, &CFG).hi, 300);
        assert!(st.stalled());
        // at the threshold is not a spike
        let mut st = LimitState::new();
        assert_eq!(st.fold(false, 2000, 1200, 0, &CFG).hi, 1200);
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
            assert_eq!(st.fold(true, 0, 200, 0, &cfg).hi, 1200);
        }
        assert!(st.stall_fault_pending());
        assert!(!st.stalled());
        // collision pends too, still no fold
        let mut st = LimitState::new();
        assert_eq!(st.fold(false, 2000, 1201, 0, &cfg).hi, 1200);
        assert!(st.stall_fault_pending());
        assert!(!st.stalled());
    }

    #[test]
    fn endstop_directional_all_combos() {
        let mut st = LimitState::new();
        // at max: the inward (positive, polarity true) side collapses,
        // retreat keeps the composed limit - one band carries both verdicts
        let b = st.fold(false, 1000, 0, 4000, &CFG);
        assert_eq!((b.lo, b.hi), (-1200, 0));
        // mirrored at min
        let b = st.fold(false, 1000, 0, -4000, &CFG);
        assert_eq!((b.lo, b.hi), (0, 1200));
        // past the wall keeps protecting
        assert_eq!(st.fold(false, 1000, 0, 5000, &CFG).hi, 0);
        // mid-range: symmetric
        let b = st.fold(false, 1000, 0, 0, &CFG);
        assert_eq!((b.lo, b.hi), (-1200, 1200));
        // reversed polarity flips which side each wall owns
        let rev = LimitCfg {
            drive_polarity: false,
            ..CFG
        };
        let b = st.fold(false, 1000, 0, 4000, &rev);
        assert_eq!((b.lo, b.hi), (0, 1200));
        let b = st.fold(false, 1000, 0, -4000, &rev);
        assert_eq!((b.lo, b.hi), (-1200, 0));
        // the deadlock regression: a zeroed command must not re-read as an
        // inward push - the band's retreat side stays open regardless
        let b = st.fold(false, 1000, 0, 4000, &CFG);
        assert_eq!(b.clamp(0), 0);
        assert_eq!(b.clamp(-120), -120);
        assert_eq!(b.clamp(120), 0);
    }

    #[test]
    fn min_composition_tightest_term_wins() {
        // stalled yield 300 vs derate near cutoff: derate binds
        let mut st = LimitState::new();
        st.update_derate(9990, &CFG); // head 10/2000 -> ~6, way under yield
        for _ in 0..10 {
            st.fold(true, 0, 200, 0, &CFG);
        }
        assert!(st.stalled());
        let lim = st.fold(true, 0, 200, 0, &CFG).hi;
        assert!((5..=6).contains(&lim), "lim={lim}");
        // endstop zero beats everything
        assert_eq!(st.fold(true, 0, 200, 4000, &CFG).hi, 0);
    }
}
