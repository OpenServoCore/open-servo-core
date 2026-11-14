//! Supply-voltage estimator. EWMA of the direct rail tap (`vbus_raw`, every
//! tick, drive or not) rescaled into motor-terminal divider counts - the
//! unit `v_undervolt_counts`, the bemf math and the current loop already
//! speak - plus the cached Q15 supply reciprocal the current loop
//! compensates with. The spec sketches an incremental
//! one-Newton-step-per-tick reciprocal; this runs the shared full-accuracy
//! `math::recip_div` every step instead - ~10 multiplies at MEDIUM (2 kHz)
//! is noise, and one reciprocal implementation beats two.

use crate::adc::ADC_Q4_SHIFT;
use crate::math::{q_mul_u, recip_div};

const GUARD: u32 = 3;
const ALPHA_SHIFT: u32 = 3;
const Q4_FRAC_MASK: u32 = (1 << ADC_Q4_SHIFT) - 1;

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

    /// One MEDIUM-tick update from the tick's rail tap and its wide-code
    /// slot (`Shared::adc_wide_q4`: Q4 counts, 0 off a wide code). The
    /// slot's whole counts join the code ahead of the scale, so the tap
    /// reads as the code that many counts up does, and with no slot
    /// exactly as its code, on any scale; the slot's fraction lands on the
    /// guard bits. The slot's work sits behind its test, off every other
    /// tap's path.
    pub fn step(&mut self, vbus_raw: u16, wide_q4: u16, undervolt_floor_counts: u16) {
        let mut whole = vbus_raw as u32;
        let mut frac_qg = 0;
        if wide_q4 != 0 {
            whole += (wide_q4 >> ADC_Q4_SHIFT) as u32;
            // under one count times a Q15 scale: far inside a u32
            frac_qg =
                ((wide_q4 as u32 & Q4_FRAC_MASK) * self.scale_q15) >> (15 + ADC_Q4_SHIFT - GUARD);
        }
        let x = ((q_mul_u(whole, self.scale_q15, 15) << GUARD) + frac_qg) as i32;
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

    /// Equal rail and terminal divider ratios (osc-dev-v006: 6K4/1K6 both).
    const UNITY: u32 = 1 << 15;
    /// A 22k/10k rail divider over 20k/10k terminal dividers,
    /// 32000 x 10000 / (10000 x 30000) = 1.0667.
    const RAIL_22K_TERM_20K: u32 = 34952;

    /// Contract residual: 0 means q_mul(vbus, recip, 15) hit 32767 exactly.
    fn recip_err(vbus: u16, recip: u32) -> i32 {
        q_mul(vbus as i32, recip as i32, 15) - 32767
    }

    #[test]
    fn first_sample_seeds_then_ewma_pins() {
        let mut est = VbusEst::new(UNITY);
        assert_eq!(est.recip_q15(), 0);
        est.step(800, 0, 1);
        assert_eq!(est.vbus_counts(), 800);
        assert!(recip_err(800, est.recip_q15()).abs() <= 1);
        // alpha 1/8: 800 + 800/8 = 900, then 900 + 700/8 = 987 in Q3
        est.step(1600, 0, 1);
        assert_eq!(est.vbus_counts(), 900);
        est.step(1600, 0, 1);
        assert_eq!(est.vbus_counts(), 987);
    }

    #[test]
    fn scale_maps_rail_tap_onto_terminal_counts() {
        // 2S at 7.4 V: 2313 on the 22k/10k tap reads as 2467 on a 20k/10k tap
        let mut est = VbusEst::new(RAIL_22K_TERM_20K);
        est.step(2313, 0, 1);
        assert_eq!(est.vbus_counts(), 2467);
        // equal ratios are the identity
        let mut est = VbusEst::new(UNITY);
        est.step(2313, 0, 1);
        assert_eq!(est.vbus_counts(), 2313);
    }

    #[test]
    fn floor_clamps_recip() {
        // vbus sags to 5: the estimate follows but the reciprocal caps at
        // the floor's, not 5's (~200x larger)
        let mut est = VbusEst::new(UNITY);
        est.step(5, 0, 1000);
        assert_eq!(est.vbus_counts(), 5);
        assert!(recip_err(1000, est.recip_q15()).abs() <= 1);
        // zero rail with floor 0: the >= 1 backstop pins the recip_div
        // d <= 1 path
        let mut cold = VbusEst::new(UNITY);
        cold.step(0, 0, 0);
        assert_eq!(cold.recip_q15(), 32767 << 15);
        // the largest reciprocal, so the current loop's i32 cast is exact
        assert!(cold.recip_q15() < 1 << 30);
    }

    #[test]
    fn recip_contract_sweep() {
        for vbus in 1000..4000u16 {
            let mut est = VbusEst::new(UNITY);
            est.step(vbus, 0, 1);
            let err = recip_err(vbus, est.recip_q15());
            assert!(err.abs() <= 1, "vbus={vbus} err={err}");
        }
    }

    #[test]
    fn recip_tracks_vbus_step_within_ewma_lag() {
        let mut est = VbusEst::new(UNITY);
        est.step(4000, 0, 100);
        // tau ~8 steps; 80 steps is >5 tau plus the truncation tail
        for _ in 0..80 {
            est.step(3000, 0, 100);
        }
        let v = est.vbus_counts();
        assert!((v as i32 - 3000).abs() <= 1, "v={v}");
        assert!(recip_err(v, est.recip_q15()).abs() <= 1);
    }

    /// The estimate before the tap carried fraction bits: whole counts
    /// scaled, then the guard.
    struct WholeCounts {
        state_qg: i32,
        initialized: bool,
    }
    impl WholeCounts {
        fn step(&mut self, vbus_raw: u16, scale_q15: u32) -> u16 {
            let x = (q_mul_u(vbus_raw as u32, scale_q15, 15) as i32) << GUARD;
            if self.initialized {
                self.state_qg += (x - self.state_qg) >> ALPHA_SHIFT;
            } else {
                self.state_qg = x;
                self.initialized = true;
            }
            (self.state_qg >> GUARD) as u16
        }
    }

    /// A tap with no slot, which is every tap under an all-zero wide-code
    /// map, steps the estimate bit for bit as the whole-count path did:
    /// every code from a fresh estimate, and a wandering run through the
    /// filter, at unity and off it.
    #[test]
    fn a_whole_count_tap_steps_as_its_code() {
        for scale in [UNITY, RAIL_22K_TERM_20K, 30000] {
            for code in 0..=u16::MAX {
                let mut est = VbusEst::new(scale);
                est.step(code, 0, 1);
                let want = q_mul_u(code as u32, scale, 15) as u16;
                assert_eq!(est.vbus_counts(), want, "scale {scale} code {code}");
            }
            let mut est = VbusEst::new(scale);
            let mut was = WholeCounts {
                state_qg: 0,
                initialized: false,
            };
            let (mut rng, mut code) = (12345u32, 2800u16);
            for n in 0..100_000u32 {
                rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
                code = (code as i32 + (rng >> 16) as i32 % 7 - 3).clamp(0, 4095) as u16;
                if n % 5000 == 0 {
                    code = ((rng >> 8) % 4096) as u16;
                }
                est.step(code, 0, 1);
                assert_eq!(
                    est.vbus_counts(),
                    was.step(code, scale),
                    "scale {scale} step {n}"
                );
                assert_eq!(est.state_qg, was.state_qg, "scale {scale} step {n}");
            }
        }
    }

    /// A slot reaches the estimate in Q4: its whole counts read as the
    /// code that many counts up, and its fraction lands on the guard bits,
    /// scaled, where the filter carries it.
    #[test]
    fn a_slot_lands_in_q4() {
        for scale in [UNITY, RAIL_22K_TERM_20K] {
            for (slot, whole) in [(160u32, 10u16), (16, 1), (255, 15), (8, 0), (1, 0)] {
                let frac = slot & 15;
                let mut est = VbusEst::new(scale);
                est.step(2815, slot as u16, 1);
                let mut up = VbusEst::new(scale);
                up.step(2815 + whole, 0, 1);
                // the fraction's share of a count, scaled, on the guard
                let guard = ((frac * scale) >> (15 + ADC_Q4_SHIFT - GUARD)) as i32;
                assert_eq!(
                    est.state_qg,
                    up.state_qg + guard,
                    "scale {scale} slot {slot}"
                );
                assert!(guard < 1 << GUARD || scale > UNITY);
            }
        }
        // half a count is four eighths at unity
        let mut est = VbusEst::new(UNITY);
        est.step(2815, 8, 1);
        assert_eq!(est.state_qg, (2815 << GUARD) + 4);
        assert_eq!(est.vbus_counts(), 2815);
    }
}
