//! Voltage-domain current PI (control-theory "The Current Loop"): PI plus
//! back-EMF decoupling feedforward, supply compensation via the vbus
//! reciprocal, and a voltage-domain clamp at duty_max*vbus whose excess
//! back-calculates into the one integrator. Clamping in volts means the
//! anti-windup term needs no domain crossing and duty <= duty_max by
//! construction, sagging battery included. On a tick with no reading the
//! PI does not run: the command is the reference's volts through the
//! winding (control-theory "The Blind Band Under the Floor").

use crate::math::{q_mul, sat_add, sat_clamp, sat_mul};
use crate::regions::config::ConfigLoopCurrent;

/// PI error clamp, ccounts. Physical error is bounded by shunt full scale
/// plus the current limit (|i_meas| <= 4095, |i_ref| <= i_lim <= 4095), so
/// 8192 is never hit in operation; the clamp exists so gain products stay
/// i32-exact: u16 gain * 2^13 <= 2^29.
const E_LIM_CC: i32 = 8192;

/// Back-calculation excess clamp, vcounts. The excess u_cl - u reaches
/// |u| < 2^23 under hostile inputs; 8192 is already 2x a 12-bit vbus,
/// so clamping only limits the per-tick unwind rate, and the product bound
/// matches E_LIM_CC: kaw * 2^13 <= 2^29.
const AW_LIM_VC: i32 = 8192;

/// The Ke decoupling feed-forward, vcounts, on `omega_ff_q16` csQ16: the
/// decoupling reference the kernel picks per mode - the trajectory's
/// omega_star in Velocity/Position, zero otherwise - never a velocity
/// estimate: the back-EMF estimate is the loop's own output read back, the
/// pot observer rails at rest. `ke_vpc_q` is `CalibMotor.ke_vpc_q`, vcounts
/// per c/s (NOT `recip_ke_q`, which points the other way).
pub fn u_ff_vc(omega_ff_q16: i32, ke_vpc_q: u16) -> i32 {
    // csQ16 * Q4.12 >> 28 -> vcounts; i64 product <= 2^31 * 2^16 = 2^47
    q_mul(omega_ff_q16, ke_vpc_q as i32, 28)
}

/// The voltage clamp `duty_max * vbus` and the integrator bound it sets,
/// prepared once per rail estimate.
#[derive(Copy, Clone, Default)]
pub struct RailClamp {
    pub v_max_vc: i32,
    pub integ_lim_vq16: i32,
}

impl RailClamp {
    pub fn new(duty_max_q15: u16, vbus_counts: u16) -> Self {
        // duty_max * vbus <= 2^15 * 2^16 = 2^31, exact in the i64 widen
        let v_max_vc = q_mul(duty_max_q15 as i32, vbus_counts as i32, 15);
        Self {
            v_max_vc,
            // v_max <= 65535 so the shift saturates (12-bit vbus keeps it
            // far below)
            integ_lim_vq16: v_max_vc.saturating_mul(1 << 16),
        }
    }
}

/// State: one voltage-domain integrator (vcounts Q16.16).
#[derive(Default)]
pub struct CurrentLoop {
    integ_vq16: i32,
}

impl CurrentLoop {
    pub const fn new() -> Self {
        Self { integ_vq16: 0 }
    }

    /// Zero state; kernel calls on the enable edge so a stale integrator
    /// never kicks a fresh enable.
    pub fn reset(&mut self) {
        self.integ_vq16 = 0;
    }

    /// The integral at `v_vc` whole vcounts, which the caller holds inside
    /// the integrator's limit.
    pub fn seed(&mut self, v_vc: i32) {
        self.integ_vq16 = v_vc << 16;
    }

    /// One FAST-tick update -> duty. `i_meas` None is a tick with no
    /// reading to take: a window under the shunt floor, or the first one
    /// read after a pulse the bridge dropped. The bridge is a voltage
    /// source under the floor, so the loop commands the reference's volts
    /// through the winding, `r_q12 x i_ref` (Q.12 vcounts per ccount,
    /// `Command::r_loop_q12`), on top of the feed-forward (`u_ff_vc`),
    /// and leaves the integrator there: nothing integrates on a current
    /// nobody read, and the first reading back starts from those volts.
    /// `rail` and `recip_vbus_q15` must describe the same vbus
    /// (estimator::vbus::VbusEst contract `(recip * vbus) >> 15 ~= 32767`)
    /// or the duty cap is off by their mismatch. The plain sums rely on the
    /// kernel's ranges: `i_ref` inside an `IBand` (<= u16::MAX), `i_meas`
    /// from `Fast::measure` (<= 139291), `u_ff_vc` from the free fn
    /// `u_ff_vc` (<= 2^19) and `r_q12` <= 2^20.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        i_ref: i32,
        i_meas: Option<i32>,
        u_ff_vc: i32,
        rail: &RailClamp,
        recip_vbus_q15: u32,
        gains: &ConfigLoopCurrent,
        r_q12: u32,
    ) -> i16 {
        let v_max = rail.v_max_vc;
        let u_cl = if let Some(i) = i_meas {
            let e = sat_clamp(i_ref - i, -E_LIM_CC, E_LIM_CC);
            // kp * 2^13 <= 2^29, i32-exact before the shift; |u_pi| < 2^22,
            // so u, and u_cl - u below, stay under 2^23
            let u_pi = q_mul(gains.i_kp_q88 as i32, e, 8) + (self.integ_vq16 >> 16);
            let u = u_pi + u_ff_vc;
            let u_cl = sat_clamp(u, -v_max, v_max);
            let aw = sat_clamp(u_cl - u, -AW_LIM_VC, AW_LIM_VC);
            // Q4.12 * ccounts -> vQ16 needs << (16 - 12); the products are
            // i32-exact (<= 2^29 per the E/AW clamps) but << 4 can reach
            // 2^33, so the shift saturates via mul. Integrator clamp:
            // +-(v_max << 16), never charged past what the output can
            // deliver (belt on top of back-calc).
            let ki_term = sat_mul(q_mul(gains.i_ki_q412 as i32, e, 0), 1 << 4);
            let aw_term = sat_mul(q_mul(gains.i_kaw_q412 as i32, aw, 0), 1 << 4);
            let lim = rail.integ_lim_vq16;
            self.integ_vq16 = sat_clamp(
                sat_add(sat_add(self.integ_vq16, ki_term), aw_term),
                -lim,
                lim,
            );
            u_cl
        } else {
            core::hint::cold_path();
            // whole vcounts (the product is <= 2^36, under 2^24 after its
            // shift), clamped inside the integrator's limit so the shift
            // back up cannot wrap
            let lim = rail.integ_lim_vq16 >> 16;
            let v = sat_clamp(q_mul(r_q12 as i32, i_ref, 12), -lim, lim);
            self.integ_vq16 = v << 16;
            sat_clamp(v + u_ff_vc, -v_max, v_max)
        };
        // VbusEst floors its divisor at 1, so recip <= 32767 << 15 < 2^30
        // casts exactly; |u_cl| * recip <= 2^17 * 2^30 = 2^47, i64-safe.
        sat_clamp(
            q_mul(u_cl, recip_vbus_q15 as i32, 15),
            -(i16::MAX as i32),
            i16::MAX as i32,
        ) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::window::I_READING_MAX;

    const I_REF_MAX: i32 = u16::MAX as i32;

    const VBUS: u16 = 3000;
    // floor reciprocal per the recip contract: (RECIP * VBUS) >> 15 = 32766
    const RECIP: u32 = ((32767u64 << 15) / VBUS as u64) as u32;
    const VBUS_SAG: u16 = 2400;
    const RECIP_SAG: u32 = ((32767u64 << 15) / VBUS_SAG as u64) as u32;

    /// The kernel's call: the feed-forward and the clamp prepared from the
    /// same gains and vbus the step runs on.
    fn step(
        cur: &mut CurrentLoop,
        i_ref: i32,
        i_meas: Option<i32>,
        omega_ff_q16: i32,
        vbus: u16,
        recip: u32,
        gains: &Gains,
    ) -> i16 {
        let rail = RailClamp::new(gains.lc.duty_max_q15, vbus);
        cur.step(
            i_ref,
            i_meas,
            u_ff_vc(omega_ff_q16, gains.ke),
            &rail,
            recip,
            &gains.lc,
            gains.r,
        )
    }

    fn recip_for(vbus: u16) -> u32 {
        ((32767u64 << 15) / vbus as u64) as u32
    }

    const KP_1: u16 = 1 << 8; // 1.0 vcounts/ccount

    /// The loop's block, the forward Ke the feed-forward takes and the
    /// resistance a tick with no reading commands through (0 from `g`).
    struct Gains {
        lc: ConfigLoopCurrent,
        ke: u16,
        r: u32,
    }

    fn g(kp: u16, ki: u16, kaw: u16, ke: u16, duty_max: u16) -> Gains {
        Gains {
            lc: ConfigLoopCurrent {
                i_kp_q88: kp,
                i_ki_q412: ki,
                i_kaw_q412: kaw,
                duty_max_q15: duty_max,
            },
            ke,
            r: 0,
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
        assert_eq!(step(&mut cur, 500, Some(0), 0, VBUS, RECIP, &gains), 5461);
        let mut cur = CurrentLoop::new();
        assert_eq!(step(&mut cur, -500, Some(0), 0, VBUS, RECIP, &gains), -5462);
    }

    #[test]
    fn integrator_eliminates_steady_state_error() {
        // R=4 plant wants u=2000 for i=500. P-only settles at
        // i = kp*e/R -> e*(1/16): huge standing error; PI closes it.
        let p_only = g(64, 0, 0, 0, 32767);
        let mut cur = CurrentLoop::new();
        let mut i = 0i32;
        for _ in 0..500 {
            i = plant_i(step(&mut cur, 500, Some(i), 0, VBUS, RECIP, &p_only), VBUS);
        }
        assert!(i < 100, "p-only i={i}");

        let pi = g(64, 200, 0, 0, 32767);
        let mut cur = CurrentLoop::new();
        let mut i = 0i32;
        for _ in 0..500 {
            i = plant_i(step(&mut cur, 500, Some(i), 0, VBUS, RECIP, &pi), VBUS);
        }
        assert!((i - 500).abs() <= 1, "pi i={i}");
    }

    /// R = 2.0 vcounts/ccount for the ticks with no reading.
    const R_2: u32 = 2 << 12;

    #[test]
    fn no_reading_commands_the_reference_through_r_on_the_feed_forward() {
        let gains = Gains {
            r: R_2,
            ..g(64, 200, 0, 1 << 12, 32767)
        };
        let mut wound = CurrentLoop::new();
        for _ in 0..50 {
            step(&mut wound, 500, Some(0), 0, VBUS, RECIP, &gains);
        }
        // 2.0 x 500 counts = 1000 vcounts -> 1000*RECIP>>15 = 10922,
        // whatever the integrator held and however many ticks repeat it
        let mut fresh = CurrentLoop::new();
        for cur in [&mut wound, &mut fresh] {
            for _ in 0..3 {
                assert_eq!(step(cur, 500, None, 0, VBUS, RECIP, &gains), 10922);
            }
        }
        // the arithmetic shift floors the negative side
        assert_eq!(step(&mut wound, -500, None, 0, VBUS, RECIP, &gains), -10923);
        // ff rides on top: ke 1.0, omega 100 c/s -> +100 vcounts ->
        // 1100*RECIP>>15 = 12014
        assert_eq!(
            step(&mut wound, 500, None, 100 << 16, VBUS, RECIP, &gains),
            12014
        );
        // the first reading back starts from the reference's volts: on the
        // reference it changes nothing, off it the PI moves from there
        assert_eq!(
            step(&mut wound, 500, Some(500), 0, VBUS, RECIP, &gains),
            10922
        );
        assert!(step(&mut wound, 500, Some(0), 0, VBUS, RECIP, &gains) > 10922);
        // a zero reference leaves nothing standing
        assert_eq!(step(&mut wound, 0, None, 0, VBUS, RECIP, &gains), 0);
        assert_eq!(step(&mut wound, 0, Some(0), 0, VBUS, RECIP, &gains), 0);
    }

    #[test]
    fn no_reading_keeps_the_integrator_inside_its_limit() {
        // the kernel's largest resistance: a u16 kp standing in, Q8.8 to Q.12
        const R_MAX: u32 = (u16::MAX as u32) << 4;
        for r in [1, R_2, R_MAX] {
            let gains = Gains {
                r,
                ..g(KP_1, 200, 2048, 0, u16::MAX)
            };
            for vbus in [1u16, 2400, 4095, u16::MAX] {
                let lim = RailClamp::new(gains.lc.duty_max_q15, vbus).integ_lim_vq16;
                for i_ref in [-I_REF_MAX, -8000, -1, 0, 1, 8000, I_REF_MAX] {
                    let mut cur = CurrentLoop::new();
                    step(&mut cur, i_ref, None, 0, vbus, recip_for(vbus), &gains);
                    let v = cur.integ_vq16;
                    assert!(v.abs() <= lim, "r={r} vbus={vbus} i_ref={i_ref}: {v}");
                    // inside the limit it is the reference's whole volts
                    let want = (r as i64 * i_ref as i64) >> 12;
                    if want.abs() <= (lim >> 16) as i64 {
                        assert_eq!(v as i64, want << 16, "r={r} vbus={vbus} i_ref={i_ref}");
                    } else {
                        assert_eq!(v.signum() as i64, want.signum());
                        assert_eq!(v.abs(), (lim >> 16) << 16);
                    }
                }
            }
        }
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
                duty = step(&mut cur, 4000, Some(i), 0, VBUS, RECIP, &gains);
                i = plant_i(duty, VBUS);
            }
            // v_max = 32767*3000>>15 = 2999 -> duty 2999*RECIP>>15 = 32756
            assert_eq!(duty, 32756, "saturated, kaw={kaw}");
            let mut ticks = 0u32;
            while ticks < 200 {
                duty = step(&mut cur, 200, Some(i), 0, VBUS, RECIP, &gains);
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
        // Unsaturated: fixed u_ff = 1000 vcounts (ke 1.0, omega 1000 c/s;
        // None on a zero reference, so the loop adds nothing). Sag raises
        // the reciprocal, so the same voltage command costs more duty:
        // 1000/3000 vs 1000/2400 of scale.
        let gains = g(0, 0, 0, 1 << 12, 32767);
        let mut cur = CurrentLoop::new();
        assert_eq!(
            step(&mut cur, 0, None, 1000 << 16, VBUS, RECIP, &gains),
            10922
        );
        let mut cur = CurrentLoop::new();
        assert_eq!(
            step(&mut cur, 0, None, 1000 << 16, VBUS_SAG, RECIP_SAG, &gains),
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
                duty = step(&mut cur, 4000, Some(i), 0, vbus, recip, &gains);
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
    fn duty_capped_for_any_legal_input() {
        // Floor reciprocal keeps recip*vbus <= 32767<<15, so the voltage
        // clamp maps to duty <= duty_max exactly; sweep hostile corners
        // including the range extremes and a pre-wound integrator.
        let gains = Gains {
            r: R_2,
            ..g(KP_1, 200, 2048, 1 << 12, 20000)
        };
        for vbus in [2400u16, 3000, 4095] {
            let recip = recip_for(vbus);
            for i_ref in [-I_REF_MAX, -8000, 0, 500, 8000, I_REF_MAX] {
                for omega in [i32::MIN, -(9000 << 16), 0, 9000 << 16, i32::MAX] {
                    for i_meas in [
                        None,
                        Some(-I_READING_MAX),
                        Some(-4095),
                        Some(0),
                        Some(4095),
                        Some(I_READING_MAX),
                    ] {
                        let mut cur = CurrentLoop::new();
                        for _ in 0..20 {
                            let d = step(&mut cur, i_ref, i_meas, omega, vbus, recip, &gains);
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
    fn sums_stay_in_i32_at_legal_extremes() {
        // debug overflow checks trip if any plain add or subtract in `step`
        // leaves i32
        let gains = Gains {
            r: (u16::MAX as u32) << 4,
            ..g(u16::MAX, u16::MAX, u16::MAX, u16::MAX, u16::MAX)
        };
        for vbus in [1u16, 4095, u16::MAX] {
            let recip = recip_for(vbus);
            for i_ref in [-I_REF_MAX, 0, I_REF_MAX] {
                for omega in [i32::MIN, 0, i32::MAX] {
                    for i_meas in [None, Some(-I_READING_MAX), Some(0), Some(I_READING_MAX)] {
                        let mut cur = CurrentLoop::new();
                        for _ in 0..20 {
                            step(&mut cur, i_ref, i_meas, omega, vbus, recip, &gains);
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
            step(&mut cur, 500, Some(0), 0, VBUS, RECIP, &gains);
        }
        assert_ne!(step(&mut cur, 0, Some(0), 0, VBUS, RECIP, &gains), 0);
        cur.reset();
        assert_eq!(step(&mut cur, 0, Some(0), 0, VBUS, RECIP, &gains), 0);
    }
}
