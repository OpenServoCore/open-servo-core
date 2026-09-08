//! Supply-voltage estimator. EWMA of the direct rail tap (`vbus_raw`, every
//! tick, drive or not) rescaled into motor-terminal divider counts - the
//! unit `v_undervolt_counts`, the bemf math and the current loop already
//! speak - plus the cached Q15 supply reciprocal the current loop
//! compensates with. The spec sketches an incremental
//! one-Newton-step-per-tick reciprocal; this runs the shared full-accuracy
//! `math::recip_div` every step instead - ~10 multiplies at MEDIUM (2 kHz)
//! is noise, and one reciprocal implementation beats two.

use crate::math::{q_mul_u, recip_div};

const GUARD: u32 = 3;
const ALPHA_SHIFT: u32 = 3;

/// EWMA alpha 1/8 with 3 guard bits; the first sample seeds the state
/// (`BemfObs` convention). A fresh `new()` carries recip 0 - duty 0 through
/// the current loop - until the first step.
pub struct VbusEst {
    state_qg: i32,
    initialized: bool,
    recip_q15: u32,
    scale_q15: u32,
}

impl VbusEst {
    /// `scale_q15` maps rail-tap counts onto vmotor-tap counts:
    /// `(vbus_top + vbus_bot) * vmotor_bot / (vbus_bot * (vmotor_top +
    /// vmotor_bot))` in Q15, const-evaluated chip-side from the board's two
    /// dividers (`Precomputed`), so core never divides.
    pub const fn new(scale_q15: u32) -> Self {
        Self {
            state_qg: 0,
            initialized: false,
            recip_q15: 0,
            scale_q15,
        }
    }

    /// One MEDIUM-tick update from the tick's rail tap.
    pub fn step(&mut self, vbus_raw: u16, undervolt_floor_counts: u16) {
        let x = (q_mul_u(vbus_raw as u32, self.scale_q15, 15) as i32) << GUARD;
        if self.initialized {
            self.state_qg += (x - self.state_qg) >> ALPHA_SHIFT;
        } else {
            self.state_qg = x;
            self.initialized = true;
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

    const UNITY: u32 = 1 << 15;
    /// osc-dev-v006 board D: 22k/10k rail divider over the 20k/10k terminal
    /// dividers, 32000 x 10000 / (10000 x 30000) = 1.0667.
    const BOARD_D: u32 = 34952;

    /// Contract residual: 0 means q_mul(vbus, recip, 15) hit 32767 exactly.
    fn recip_err(vbus: u16, recip: u32) -> i32 {
        q_mul(vbus as i32, recip as i32, 15) - 32767
    }

    #[test]
    fn first_sample_seeds_then_ewma_pins() {
        let mut est = VbusEst::new(UNITY);
        assert_eq!(est.recip_q15(), 0);
        est.step(800, 1);
        assert_eq!(est.vbus_counts(), 800);
        assert!(recip_err(800, est.recip_q15()).abs() <= 1);
        // alpha 1/8: 800 + 800/8 = 900, then 900 + 700/8 = 987 in Q3
        est.step(1600, 1);
        assert_eq!(est.vbus_counts(), 900);
        est.step(1600, 1);
        assert_eq!(est.vbus_counts(), 987);
    }

    #[test]
    fn scale_maps_rail_tap_onto_terminal_counts() {
        // 2S at 7.4 V: 2313 on the 22k/10k tap reads as 2467 on a 20k/10k tap
        let mut est = VbusEst::new(BOARD_D);
        est.step(2313, 1);
        assert_eq!(est.vbus_counts(), 2467);
        // a 20k/10k rail tap (rev-2A) is the identity
        let mut est = VbusEst::new(UNITY);
        est.step(2313, 1);
        assert_eq!(est.vbus_counts(), 2313);
    }

    #[test]
    fn floor_clamps_recip() {
        // vbus sags to 5: the estimate follows but the reciprocal caps at
        // the floor's, not 5's (~200x larger)
        let mut est = VbusEst::new(UNITY);
        est.step(5, 1000);
        assert_eq!(est.vbus_counts(), 5);
        assert!(recip_err(1000, est.recip_q15()).abs() <= 1);
        // zero rail with floor 0: the >= 1 backstop pins the recip_div
        // d <= 1 path
        let mut cold = VbusEst::new(UNITY);
        cold.step(0, 0);
        assert_eq!(cold.recip_q15(), 32767 << 15);
    }

    #[test]
    fn recip_contract_sweep() {
        for vbus in 1000..4000u16 {
            let mut est = VbusEst::new(UNITY);
            est.step(vbus, 1);
            let err = recip_err(vbus, est.recip_q15());
            assert!(err.abs() <= 1, "vbus={vbus} err={err}");
        }
    }

    #[test]
    fn recip_tracks_vbus_step_within_ewma_lag() {
        let mut est = VbusEst::new(UNITY);
        est.step(4000, 100);
        // tau ~8 steps; 80 steps is >5 tau plus the truncation tail
        for _ in 0..80 {
            est.step(3000, 100);
        }
        let v = est.vbus_counts();
        assert!((v as i32 - 3000).abs() <= 1, "v={v}");
        assert!(recip_err(v, est.recip_q15()).abs() <= 1);
    }
}
