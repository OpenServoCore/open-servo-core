//! Voltage-domain current PI (control-theory "The Current Loop"): PI plus
//! back-EMF decoupling feedforward, supply compensation via the vbus
//! reciprocal, and a voltage-domain clamp at duty_max*vbus whose excess
//! back-calculates into the one integrator. Clamping in volts means the
//! anti-windup term needs no domain crossing and duty <= duty_max by
//! construction, sagging battery included.

use crate::math::q_mul;

/// PI error clamp, ccounts. Physical error is bounded by shunt full scale
/// plus the current limit (|i_meas| <= 4095, |i_ref| <= i_lim <= 4095), so
/// 8192 is never hit in operation; the clamp exists so gain products stay
/// i32-exact: u16 gain * 2^13 <= 2^29.
const E_LIM_CC: i32 = 8192;

/// Back-calculation excess clamp, vcounts. The excess u_cl - u is unbounded
/// (u saturates i32 under hostile inputs); 8192 is already 2x a 12-bit vbus,
/// so clamping only limits the per-tick unwind rate, and the product bound
/// matches E_LIM_CC: kaw * 2^13 <= 2^29.
const AW_LIM_VC: i32 = 8192;

/// CONFIG loop_current gains plus the CALIB forward Ke, loaded fresh each
/// step by the kernel. `ke_q412` is `CalibMotor.ke_vpc_q`: vcounts per c/s,
/// the decoupling feedforward (NOT `recip_ke_q`, which points the other way).
#[derive(Copy, Clone)]
pub struct CurrentGains {
    pub kp_q88: u16,
    pub ki_q412: u16,
    pub kaw_q412: u16,
    pub ke_q412: u16,
    pub duty_max_q15: u16,
}

/// State: one voltage-domain integrator (vcounts Q16.16) and the last duty
/// written, kept for telemetry `duty_applied_q15`.
#[derive(Default)]
pub struct CurrentLoop {
    integ_vq16: i32,
    last_duty_q15: i16,
}

impl CurrentLoop {
    pub const fn new() -> Self {
        Self {
            integ_vq16: 0,
            last_duty_q15: 0,
        }
    }

    /// Zero state; kernel calls on the enable edge so a stale integrator
    /// never kicks a fresh enable.
    pub fn reset(&mut self) {
        self.integ_vq16 = 0;
        self.last_duty_q15 = 0;
    }

    pub fn last_duty_q15(&self) -> i16 {
        self.last_duty_q15
    }

    /// One FAST-tick update -> duty. `i_meas` None (shunt window invalid)
    /// freezes the whole integrator update - ki term and back-calc both -
    /// and the output rides integrator + feedforward until the window
    /// returns. `omega_hat_q16` csQ16 from fusion; `recip_vbus_q15` is the
    /// vbus estimator's reciprocal, contract `(recip * vbus) >> 15
    /// ~= 32767` (estimator::vbus::VbusEst) - the pair must describe the
    /// same vbus or the duty cap is off by their mismatch.
    pub fn step(
        &mut self,
        i_ref: i32,
        i_meas: Option<i32>,
        omega_hat_q16: i32,
        vbus_counts: u16,
        recip_vbus_q15: u32,
        gains: &CurrentGains,
    ) -> i16 {
        let e = match i_meas {
            Some(i) => i_ref.saturating_sub(i).clamp(-E_LIM_CC, E_LIM_CC),
            None => 0,
        };
        // kp * 2^13 <= 2^29, i32-exact before the shift
        let u_pi = q_mul(gains.kp_q88 as i32, e, 8).saturating_add(self.integ_vq16 >> 16);
        // csQ16 * Q4.12 >> 28 -> vcounts; i64 product <= 2^31 * 2^16 = 2^47
        let u_ff = q_mul(omega_hat_q16, gains.ke_q412 as i32, 28);
        let u = u_pi.saturating_add(u_ff);
        // duty_max * vbus <= 2^15 * 2^16 = 2^31, exact in the i64 widen
        let v_max = q_mul(gains.duty_max_q15 as i32, vbus_counts as i32, 15);
        let u_cl = u.clamp(-v_max, v_max);
        if i_meas.is_some() {
            let aw = u_cl.saturating_sub(u).clamp(-AW_LIM_VC, AW_LIM_VC);
            // Q4.12 * ccounts -> vQ16 needs << (16 - 12); the products are
            // i32-exact (<= 2^29 per the E/AW clamps) but << 4 can reach
            // 2^33, so the shift saturates via mul. Integrator clamp:
            // +-(v_max << 16), never charged past what the output can
            // deliver (belt on top of back-calc); v_max <= 65535 so the
            // shift saturates too (12-bit vbus keeps it far below).
            let ki_term = q_mul(gains.ki_q412 as i32, e, 0).saturating_mul(1 << 4);
            let aw_term = q_mul(gains.kaw_q412 as i32, aw, 0).saturating_mul(1 << 4);
            let lim = v_max.saturating_mul(1 << 16);
            self.integ_vq16 = self
                .integ_vq16
                .saturating_add(ki_term)
                .saturating_add(aw_term)
                .clamp(-lim, lim);
        }
        // min guards the i32 cast: a reciprocal with bit 31 set would flip
        // the duty sign; the undervolt-floored vbus keeps real reciprocals
        // far below. |u_cl| * recip <= 2^17 * 2^31 = 2^48, i64-safe.
        let recip = recip_vbus_q15.min(i32::MAX as u32) as i32;
        let duty = q_mul(u_cl, recip, 15).clamp(-(i16::MAX as i32), i16::MAX as i32) as i16;
        self.last_duty_q15 = duty;
        duty
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VBUS: u16 = 3000;
    // floor reciprocal per the recip contract: (RECIP * VBUS) >> 15 = 32766
    const RECIP: u32 = ((32767u64 << 15) / VBUS as u64) as u32;
    const VBUS_SAG: u16 = 2400;
    const RECIP_SAG: u32 = ((32767u64 << 15) / VBUS_SAG as u64) as u32;

    fn recip_for(vbus: u16) -> u32 {
        ((32767u64 << 15) / vbus as u64) as u32
    }

    const KP_1: u16 = 1 << 8; // 1.0 vcounts/ccount

    fn g(kp: u16, ki: u16, kaw: u16, ke: u16, duty_max: u16) -> CurrentGains {
        CurrentGains {
            kp_q88: kp,
            ki_q412: ki,
            kaw_q412: kaw,
            ke_q412: ke,
            duty_max_q15: duty_max,
        }
    }

    /// Static resistive plant, R = 4 vcounts/ccount: applied volts are the
    /// duty fraction of vbus, current is v/R. 20 kHz is far above the
    /// electrical pole so the lag is dropped; steady state is what the
    /// integrator tests need.
    fn plant_i(duty: i16, vbus: u16) -> i32 {
        q_mul(duty as i32, vbus as i32, 15) >> 2
    }

    #[test]
    fn p_only_step_toward_setpoint() {
        // e=500 * kp 1.0 -> u=500 vcounts; duty = 500*RECIP>>15 = 5461
        // (~500/3000 of full scale). Arithmetic shift floors the negative
        // side to -5462.
        let gains = g(KP_1, 0, 0, 0, 32767);
        let mut cur = CurrentLoop::new();
        assert_eq!(cur.step(500, Some(0), 0, VBUS, RECIP, &gains), 5461);
        assert_eq!(cur.last_duty_q15(), 5461);
        let mut cur = CurrentLoop::new();
        assert_eq!(cur.step(-500, Some(0), 0, VBUS, RECIP, &gains), -5462);
    }

    #[test]
    fn integrator_eliminates_steady_state_error() {
        // R=4 plant wants u=2000 for i=500. P-only settles at
        // i = kp*e/R -> e*(1/16): huge standing error; PI closes it.
        let p_only = g(64, 0, 0, 0, 32767);
        let mut cur = CurrentLoop::new();
        let mut i = 0i32;
        for _ in 0..500 {
            i = plant_i(cur.step(500, Some(i), 0, VBUS, RECIP, &p_only), VBUS);
        }
        assert!(i < 100, "p-only i={i}");

        let pi = g(64, 200, 0, 0, 32767);
        let mut cur = CurrentLoop::new();
        let mut i = 0i32;
        for _ in 0..500 {
            i = plant_i(cur.step(500, Some(i), 0, VBUS, RECIP, &pi), VBUS);
        }
        assert!((i - 500).abs() <= 1, "pi i={i}");
    }

    #[test]
    fn invalid_window_holds_integrator_outputs_ff() {
        let gains = g(64, 200, 0, 1 << 12, 32767);
        let mut cur = CurrentLoop::new();
        for _ in 0..50 {
            cur.step(500, Some(0), 0, VBUS, RECIP, &gains);
        }
        // None freezes the integrator: repeated steps identical
        let d1 = cur.step(500, None, 0, VBUS, RECIP, &gains);
        let d2 = cur.step(500, None, 0, VBUS, RECIP, &gains);
        assert_eq!(d1, d2);
        // ff still live: ke 1.0, omega 100 c/s -> +100 vcounts ->
        // +100*RECIP>>15 = +1092 duty on top of the held integrator
        let d3 = cur.step(500, None, 100 << 16, VBUS, RECIP, &gains);
        assert_eq!(d3 - d1, 1092);
        // valid sample moves it again
        let d4 = cur.step(500, Some(0), 0, VBUS, RECIP, &gains);
        assert!(d4 > d1);
    }

    #[test]
    fn back_calc_prevents_windup() {
        // Phase 1: i_ref 4000 unreachable (plant caps at v_max/4 ~ 750);
        // phase 2: drop to reachable 200 (u ~ 800 < v_max 2999) and count
        // ticks pinned near the cap. Back-calc holds the integrator at the
        // saturation equilibrium u = v_max + (ki/kaw)*e, so the release
        // unsaturates on the first tick; without it the integrator rails at
        // the v_max<<16 clamp and must grind back down through ki alone
        // (slow ki here so that grind is visible).
        let run = |kaw: u16| -> u32 {
            let gains = g(64, 50, kaw, 0, 32767);
            let mut cur = CurrentLoop::new();
            let mut i = 0i32;
            let mut duty = 0i16;
            for _ in 0..300 {
                duty = cur.step(4000, Some(i), 0, VBUS, RECIP, &gains);
                i = plant_i(duty, VBUS);
            }
            // v_max = 32767*3000>>15 = 2999 -> duty 2999*RECIP>>15 = 32756
            assert_eq!(duty, 32756, "saturated, kaw={kaw}");
            let mut ticks = 0u32;
            while ticks < 200 {
                duty = cur.step(200, Some(i), 0, VBUS, RECIP, &gains);
                i = plant_i(duty, VBUS);
                if duty < 30000 {
                    break;
                }
                ticks += 1;
            }
            ticks
        };
        let with_aw = run(2048);
        let without_aw = run(0);
        assert!(with_aw <= 3, "with_aw={with_aw}");
        assert!(without_aw > 3 * (with_aw + 1), "without_aw={without_aw}");
    }

    #[test]
    fn vbus_compensation_and_sagging_cap() {
        // Unsaturated: fixed u_ff = 1000 vcounts (ke 1.0, omega 1000 c/s,
        // None so PI stays zero). Sag raises the reciprocal, so the same
        // voltage command costs more duty: 1000/3000 vs 1000/2400 of scale.
        let gains = g(0, 0, 0, 1 << 12, 32767);
        let mut cur = CurrentLoop::new();
        assert_eq!(cur.step(0, None, 1000 << 16, VBUS, RECIP, &gains), 10922);
        let mut cur = CurrentLoop::new();
        assert_eq!(
            cur.step(0, None, 1000 << 16, VBUS_SAG, RECIP_SAG, &gains),
            13652
        );

        // Saturated at duty_max 0.5: available volts scale with vbus, so
        // the R=4 plant settles at v_max/4 - less current from a sagging
        // battery even though duty sits at the cap either way.
        let gains = g(64, 200, 2048, 0, 16384);
        let settle = |vbus: u16, recip: u32| -> (i16, i32) {
            let mut cur = CurrentLoop::new();
            let mut i = 0i32;
            let mut duty = 0i16;
            for _ in 0..300 {
                duty = cur.step(4000, Some(i), 0, vbus, recip, &gains);
                i = plant_i(duty, vbus);
            }
            (duty, i)
        };
        let (duty_full, i_full) = settle(VBUS, RECIP);
        let (duty_sag, i_sag) = settle(VBUS_SAG, RECIP_SAG);
        assert!(duty_full <= 16384 && duty_sag <= 16384);
        // v_max: 16384*3000>>15 = 1499 -> i 374; 16384*2400>>15 = 1199 -> 299
        assert_eq!(i_full, 374);
        assert_eq!(i_sag, 299);
    }

    #[test]
    fn duty_capped_for_any_input() {
        // Floor reciprocal keeps recip*vbus <= 32767<<15, so the voltage
        // clamp maps to duty <= duty_max exactly; sweep hostile corners
        // including i32 extremes and a pre-wound integrator.
        let gains = g(KP_1, 200, 2048, 1 << 12, 20000);
        for vbus in [2400u16, 3000, 4095] {
            let recip = recip_for(vbus);
            for i_ref in [i32::MIN, -8000, 0, 500, 8000, i32::MAX] {
                for omega in [i32::MIN, -(9000 << 16), 0, 9000 << 16, i32::MAX] {
                    for i_meas in [None, Some(-4095), Some(0), Some(4095)] {
                        let mut cur = CurrentLoop::new();
                        for _ in 0..20 {
                            let d = cur.step(i_ref, i_meas, omega, vbus, recip, &gains);
                            assert!(
                                d.unsigned_abs() <= 20000,
                                "d={d} vbus={vbus} i_ref={i_ref} omega={omega}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn reset_zeroes_state() {
        let gains = g(64, 200, 0, 0, 32767);
        let mut cur = CurrentLoop::new();
        for _ in 0..50 {
            cur.step(500, Some(0), 0, VBUS, RECIP, &gains);
        }
        assert_ne!(cur.step(0, Some(0), 0, VBUS, RECIP, &gains), 0);
        cur.reset();
        assert_eq!(cur.last_duty_q15(), 0);
        assert_eq!(cur.step(0, Some(0), 0, VBUS, RECIP, &gains), 0);
    }
}
