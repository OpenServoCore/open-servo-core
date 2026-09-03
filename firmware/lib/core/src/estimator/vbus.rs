//! Supply-voltage estimator. EWMA of the driven-terminal sample (fwd ->
//! vmotor_a, rev -> vmotor_b; the kernel window-selects) plus the cached Q15
//! supply reciprocal the current loop compensates with. The spec sketches an
//! incremental one-Newton-step-per-tick reciprocal; this runs the shared
//! full-accuracy `math::recip_div` every step instead - ~10 multiplies at
//! MEDIUM (2 kHz) is noise, and one reciprocal implementation beats two.

use crate::math::recip_div;

const GUARD: u32 = 3;
const ALPHA_SHIFT: u32 = 3;

/// EWMA alpha 1/8 with 3 guard bits; the first valid sample seeds the state
/// (`BemfObs` convention). A fresh `new()` carries recip 0 - duty 0 through
/// the current loop - until the first sample or an install-time `seed`.
#[derive(Default)]
pub struct VbusEst {
    state_qg: i32,
    initialized: bool,
    recip_q15: u32,
    fresh: bool,
}

impl VbusEst {
    pub const fn new() -> Self {
        Self {
            state_qg: 0,
            initialized: false,
            recip_q15: 0,
            fresh: false,
        }
    }

    /// True iff a valid drive-window sample landed since the last call;
    /// clears on read. Undervolt verdicts gate on this: a held (stale)
    /// estimate is not evidence - a sagged reading frozen by the fault's
    /// own bridge-off otherwise re-latches forever (fault -> no drive ->
    /// no fresh sample -> fault).
    pub fn take_fresh(&mut self) -> bool {
        core::mem::take(&mut self.fresh)
    }

    /// Install-time seed from the first frame or a nominal, so boot does not
    /// ride a zero reciprocal through an EWMA settle.
    pub fn seed(&mut self, vbus_counts: u16, floor: u16) {
        self.state_qg = (vbus_counts as i32) << GUARD;
        self.initialized = true;
        self.update_recip(floor);
    }

    /// One MEDIUM-tick update. `vdrive` = the window-selected driven-terminal
    /// sample; `None` (window under the vmotor validity floor) holds the last
    /// estimate - vbus moves slowly.
    pub fn step(&mut self, vdrive: Option<u16>, undervolt_floor_counts: u16) {
        if let Some(v) = vdrive {
            let x = (v as i32) << GUARD;
            if self.initialized {
                self.state_qg += (x - self.state_qg) >> ALPHA_SHIFT;
            } else {
                self.state_qg = x;
                self.initialized = true;
            }
            self.fresh = true;
        }
        self.update_recip(undervolt_floor_counts);
    }

    /// The floor clamp (>= 1 backstop) bounds the reciprocal: a glitched
    /// near-zero sample cannot blow it up and drive duty to the rail - the
    /// worst case the current loop ever sees is the undervolt floor's own
    /// reciprocal, and undervolt is the fault path's call.
    fn update_recip(&mut self, floor: u16) {
        let d = (self.vbus_counts() as u32).max(floor.max(1) as u32);
        self.recip_q15 = recip_div(32767u32 << 15, d);
    }

    /// Filtered supply, vcounts.
    pub fn vbus_counts(&self) -> u16 {
        (self.state_qg >> GUARD) as u16
    }

    /// Contract: `q_mul(u_vcounts, recip_q15 as i32, 15)` maps u == vbus to
    /// ~32767 - the pair `CurrentLoop::step` expects for `recip_vbus_q15`.
    pub fn recip_q15(&self) -> u32 {
        self.recip_q15
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::q_mul;

    /// Contract residual: 0 means q_mul(vbus, recip, 15) hit 32767 exactly.
    fn recip_err(vbus: u16, recip: u32) -> i32 {
        q_mul(vbus as i32, recip as i32, 15) - 32767
    }

    #[test]
    fn seed_sets_state_and_recip() {
        let mut est = VbusEst::new();
        assert_eq!(est.recip_q15(), 0);
        est.seed(3000, 100);
        assert_eq!(est.vbus_counts(), 3000);
        assert!(recip_err(3000, est.recip_q15()).abs() <= 1);
    }

    #[test]
    fn first_sample_seeds_then_ewma_pins() {
        let mut est = VbusEst::new();
        est.step(Some(800), 1);
        assert_eq!(est.vbus_counts(), 800);
        // alpha 1/8: 800 + 800/8 = 900, then 900 + 700/8 = 987 in Q3
        est.step(Some(1600), 1);
        assert_eq!(est.vbus_counts(), 900);
        est.step(Some(1600), 1);
        assert_eq!(est.vbus_counts(), 987);
    }

    #[test]
    fn none_holds_value_and_recip() {
        let mut est = VbusEst::new();
        est.seed(2500, 1);
        let r = est.recip_q15();
        est.step(None, 1);
        assert_eq!(est.vbus_counts(), 2500);
        assert_eq!(est.recip_q15(), r);
    }

    #[test]
    fn floor_clamps_recip() {
        // vbus sags to 5: the estimate follows but the reciprocal caps at
        // the floor's, not 5's (~200x larger)
        let mut est = VbusEst::new();
        est.seed(5, 1000);
        assert_eq!(est.vbus_counts(), 5);
        assert!(recip_err(1000, est.recip_q15()).abs() <= 1);
        // never-seeded zero vbus with floor 0: the >= 1 backstop pins the
        // recip_div d <= 1 path
        let mut cold = VbusEst::new();
        cold.step(None, 0);
        assert_eq!(cold.recip_q15(), 32767 << 15);
    }

    #[test]
    fn recip_contract_sweep() {
        let mut est = VbusEst::new();
        for vbus in 1000..4000u16 {
            est.seed(vbus, 1);
            let err = recip_err(vbus, est.recip_q15());
            assert!(err.abs() <= 1, "vbus={vbus} err={err}");
        }
    }

    #[test]
    fn recip_tracks_vbus_step_within_ewma_lag() {
        let mut est = VbusEst::new();
        est.seed(4000, 100);
        // tau ~8 steps; 80 steps is >5 tau plus the truncation tail
        for _ in 0..80 {
            est.step(Some(3000), 100);
        }
        let v = est.vbus_counts();
        assert!((v as i32 - 3000).abs() <= 1, "v={v}");
        assert!(recip_err(v, est.recip_q15()).abs() <= 1);
    }
}
