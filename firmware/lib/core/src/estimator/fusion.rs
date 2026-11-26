//! Fixed-gain 3-state fusion observer (control-theory "The Fusion Filter").
//! Predict pushes theta/omega through the mechanical model - b_i_q313 =
//! round(B * 8192) where B bakes Kt*Ts/J as (c/s per medium tick) per
//! ccount (rig-measured B runs ~3.4, so Q3.13 holds it with headroom
//! where Q0.16 saturated), and the shift-0 product << 3 couples current
//! counts into csQ16 - and correct nudges all three states against the
//! measured pot position. The model's drag is the whole friction line on
//! the observer's own speed (Coulomb plus viscous), so the third state is
//! the disturbance torque the model cannot explain, in current counts, and
//! a free shaft reads none in motion; it feeds
//! stall/collision detection and telemetry. Gains are host-synthesized
//! constants (no runtime matrix math on this chip). The measurement is the
//! linearized pot in Q4 (`pos_lut` module), so theta is in linearized
//! counts; at identity `(raw << 4) << 12 == raw << 16`, bit-identical.

use crate::math::{q_mul, sat_add, sat_mul, sat_sub, sat_sym};
use crate::pos_lut::GRID_SHIFT;
use crate::regions::calib::CalibMotor;
use crate::regions::config::ConfigFusion;

/// Q4 linearized counts -> cQ16.
const Q4_TO_Q16: u32 = 16 - GRID_SHIFT;

/// Predict skips the Coulomb term below 1 c/s: at rest omega dithers around
/// zero by sub-count amounts, and a sign-chattering sgn(omega)*fric_fc would
/// inject phantom +-fric_fc drive into every predict.
const FRIC_OMEGA_EPS_CSQ16: i32 = 1 << 16;

/// theta is NOT clamped to the pot span - it must track the measurement
/// freely so e stays honest past the ends. The bound only guards i32: the
/// Q4 word tops at 65535 << 12 < 2^28, so |pos - theta| <= 2^28 + 2^29
/// fits i32.
const THETA_LIM_CQ16: i32 = 1 << 29;

/// Innovation clamp, 128 counts. The worst correct-step product is a Q8.8
/// gain at u16::MAX: 65535 * 2^23 >> 8 = 65535 << 15 < i32::MAX, so no
/// q_mul result wraps for any gain encoding; also bounds glitch response.
const E_LIM_CQ16: i32 = 1 << 23;

/// i16 c/s full scale in csQ16 (SG90 tops ~9000 c/s, 3.6x headroom).
const OMEGA_LIM_CSQ16: i32 = 32767 << 16;

/// Full shunt scale in ccQ16 - a disturbance beyond +-4095 current counts
/// is not resolvable by the model anyway.
const TAU_D_LIM_CCQ16: i32 = 4095 << 16;

/// Model-input clamp, 2x shunt full scale: 65535 * 8192 < 2^31 keeps the
/// shift-0 b_i product i32-exact for any gain encoding; only the << 3 to
/// csQ16 saturates (velocity.rs shift discipline).
const ACCEL_LIM_CC: i32 = 8192;

/// Branchless: the sign of omega outside the dead zone, times the level.
#[inline(always)]
fn fric_c(omega_q16: i32, fric_fc_counts: u16) -> i32 {
    let sign =
        (omega_q16 > FRIC_OMEGA_EPS_CSQ16) as i32 - (omega_q16 < -FRIC_OMEGA_EPS_CSQ16) as i32;
    sign * fric_fc_counts as i32
}

/// States: theta cQ16 (linearized pot counts), omega csQ16, tau_d ccQ16.
#[derive(Default)]
pub struct FusionObs {
    theta_q16: i32,
    omega_q16: i32,
    tau_d_q16: i32,
}

impl FusionObs {
    pub const fn new() -> Self {
        Self {
            theta_q16: 0,
            omega_q16: 0,
            tau_d_q16: 0,
        }
    }

    /// Reset to the measurement. Kernel calls at install and on the
    /// torque-enable edge so stale states never kick a fresh enable.
    pub fn seed(&mut self, pos_q4: u16) {
        self.theta_q16 = Self::seed_theta_q16(pos_q4);
        self.omega_q16 = 0;
        self.tau_d_q16 = 0;
    }

    /// The theta a seed at `pos_q4` takes.
    #[inline(always)]
    pub fn seed_theta_q16(pos_q4: u16) -> i32 {
        (pos_q4 as i32) << Q4_TO_Q16
    }

    /// Keep theta, at rest: the seed without the measurement, for an
    /// estimate the caller trusts over one sample.
    #[inline(always)]
    pub fn rest(&mut self) {
        self.omega_q16 = 0;
        self.tau_d_q16 = 0;
    }

    /// One MEDIUM step: predict over `dt_q32`, correct on this sample.
    /// `i_counts` is the caller's held current: the last window-valid
    /// measurement, 0 once nothing drives - the observer never sees the
    /// validity flag.
    /// The model is CALIB `motor`'s (b_i and the friction line fric_fc,
    /// fric_fv on omega), the corrections CONFIG `fusion`'s.
    pub fn step(
        &mut self,
        i_counts: i32,
        pos_q4: u16,
        dt_q32: u32,
        gains: &ConfigFusion,
        motor: &CalibMotor,
    ) {
        // Predict. b_i is Q3.13 of B (c/s per ccount per period), so the
        // shift-0 product lands in csQ13 (< 2^29, exact); the << 3 to csQ16
        // saturates only beyond omega full scale (ACCEL_LIM_CC keeps the
        // product itself i32-exact). Saturating subs guard a hostile
        // i_counts, everything downstream is clamp-bounded.
        let fric = fric_c(self.omega_q16, motor.fric_fc_counts);
        // The friction line's viscous part on whole c/s, in 32 bits: the
        // omega clamp and a u16 fv keep the product under 2^31 and its sum
        // with tau_d (<= 4095) under 2^16, so the plain ops are exact. The
        // 64-bit q_mul form costs twice the cycles.
        let visc = (motor.fric_fv_q016 as i32 * (self.omega_q16 >> 16)) >> 16;
        let accel = sat_sym(
            sat_sub(sat_sub(i_counts, fric), (self.tau_d_q16 >> 16) + visc),
            ACCEL_LIM_CC,
        );
        let d_omega = q_mul(motor.b_i_q313 as i32, accel, 0);
        self.omega_q16 = sat_sym(
            sat_add(self.omega_q16, sat_mul(d_omega, 1 << 3)),
            OMEGA_LIM_CSQ16,
        );
        // |omega| <= 2^31, dt < 2^23 -> |delta| < 2^22; theta is held
        // inside +-2^29 by every store here and by `seed`, so the plain
        // sum is exact
        self.theta_q16 = sat_sym(
            self.theta_q16 + q_mul(self.omega_q16, dt_q32 as i32, 32),
            THETA_LIM_CQ16,
        );

        // Correct. Plain sub is safe: 2^28 + 2^29 < 2^31 (theta clamp).
        let e = sat_sym(((pos_q4 as i32) << Q4_TO_Q16) - self.theta_q16, E_LIM_CQ16);
        // a Q0.16 gain on |e| <= 2^23 stays under 2^23: exact on theta
        self.theta_q16 = sat_sym(
            self.theta_q16 + q_mul(gains.l1_q016 as i32, e, 16),
            THETA_LIM_CQ16,
        );
        self.omega_q16 = sat_sym(
            sat_add(self.omega_q16, q_mul(gains.l2_q88 as i32, e, 8)),
            OMEGA_LIM_CSQ16,
        );
        // No model, no disturbance: with b_i unset predict explains nothing,
        // so tau_d would integrate plain position error and trip the
        // collision check in a free run.
        self.tau_d_q16 = if motor.b_i_q313 == 0 {
            0
        } else {
            sat_sym(
                sat_sub(self.tau_d_q16, q_mul(gains.l3_q88 as i32, e, 8)),
                TAU_D_LIM_CCQ16,
            )
        };
    }

    pub fn theta_q16(&self) -> i32 {
        self.theta_q16
    }

    pub fn omega_q16(&self) -> i32 {
        self.omega_q16
    }

    /// ccQ16, inside +-4095 counts.
    pub fn tau_d_q16(&self) -> i32 {
        self.tau_d_q16
    }

    /// Whole current counts. The state clamp already bounds to +-4095; the
    /// i16 clamp is the saturating ABI cast.
    pub fn tau_d_counts(&self) -> i16 {
        (self.tau_d_q16 >> 16)
            .max(i16::MIN as i32)
            .min(i16::MAX as i32) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 2 kHz MEDIUM rate, matching the kernel's DT_MED_Q32 derivation.
    const DT: u32 = ((1u64 << 32) / 2000) as u32;

    /// The identity's Q4 word for a raw pot count.
    const fn q4(raw: u16) -> u16 {
        raw << GRID_SHIFT
    }

    // Hand-picked stable set for the 2 kHz discrete observer: b_i = 819 =
    // Q3.13 of B = 0.1 c/s per tick per ccount (low rig-physical range;
    // the rig measures ~3.4), l1 = 0.25, l2 = 4.0 c/s per count, l3 = 8.0
    // cc per count. With the coupling live, the e -> tau_d -> omega ->
    // theta loop gain scales as l3 * B, so l3 must shrink as b_i grows:
    // 64.0 rails the filter at this B (integer-sim verified), 8.0 sits
    // inside the stable region with the disturbance step still settling
    // exactly. If a convergence test oscillates the fix is smaller gains,
    // not more iterations.
    const G: Gains = gains(819, 2048, 0);

    /// The corrections and the model the step reads.
    struct Gains {
        l: ConfigFusion,
        motor: CalibMotor,
    }

    const fn gains(b_i_q313: u16, l3_q88: u16, fric_fc_counts: u16) -> Gains {
        Gains {
            l: ConfigFusion {
                l1_q016: 16384,
                l2_q88: 1024,
                l3_q88,
                ..ConfigFusion::new()
            },
            motor: CalibMotor {
                b_i_q313,
                fric_fc_counts,
                ..CalibMotor::new()
            },
        }
    }

    fn step(f: &mut FusionObs, i: i32, pos_q4: u16, g: &Gains) {
        f.step(i, pos_q4, DT, &g.l, &g.motor);
    }

    #[test]
    fn seed_identity() {
        let mut f = FusionObs::new();
        f.seed(q4(1234));
        assert_eq!(f.theta_q16(), 1234 << 16);
        assert_eq!(f.omega_q16(), 0);
        assert_eq!(f.tau_d_counts(), 0);
    }

    #[test]
    fn rest_keeps_theta_and_clears_the_rest() {
        // torque-off motion against a pinned pot builds omega and tau_d
        let mut f = FusionObs::new();
        f.seed(q4(1990));
        for _ in 0..50 {
            step(&mut f, 300, q4(2000), &G);
        }
        assert_ne!(f.omega_q16(), 0, "precondition");
        assert_ne!(f.tau_d_counts(), 0, "precondition");
        let theta = f.theta_q16();
        assert_ne!(theta, 2000 << 16, "precondition: theta off the pot");
        f.rest();
        assert_eq!(f.theta_q16(), theta);
        assert_eq!(f.omega_q16(), 0);
        assert_eq!(f.tau_d_counts(), 0);
    }

    #[test]
    fn seed_theta_is_the_seed() {
        let mut f = FusionObs::new();
        for pos in [0, 1, q4(2000) + 8, u16::MAX] {
            f.seed(pos);
            assert_eq!(f.theta_q16(), FusionObs::seed_theta_q16(pos));
        }
    }

    /// The Q4 seed at identity is the old `raw << 16` for every raw count,
    /// and a fractional Q4 word lands on the fraction.
    #[test]
    fn seed_q4_matches_raw_shift() {
        let mut f = FusionObs::new();
        for raw in 0..4096u16 {
            f.seed(q4(raw));
            assert_eq!(f.theta_q16(), (raw as i32) << 16);
        }
        f.seed(q4(2000) + 8);
        assert_eq!(f.theta_q16(), (2000 << 16) + (1 << 15));
        step(&mut f, 0, q4(2000) + 8, &G);
        assert_eq!(f.theta_q16(), (2000 << 16) + (1 << 15), "e = 0 holds it");
    }

    #[test]
    fn static_convergence() {
        // Seed 10 counts below the measurement, zero current: theta locks
        // to the measurement, omega and tau_d bleed back to ~0 (the live
        // b_i bleed path settles by ~10000 ticks; 40000 keeps margin).
        // Residuals below the quantization floors (omega under ~2000 q16
        // moves theta by 0 per tick: q_mul(omega, DT, 32) truncates
        // omega/2000 to zero), pinned exact.
        let mut f = FusionObs::new();
        f.seed(q4(1990));
        for _ in 0..40000 {
            step(&mut f, 0, q4(2000), &G);
        }
        assert_eq!(f.theta_q16(), 2000 << 16, "pin");
        assert_eq!(f.omega_q16(), 1964, "pin");
        assert_eq!(f.tau_d_counts(), 0, "pin");
    }

    #[test]
    fn current_couples_at_full_scale() {
        // One step from rest, e = 0 going in: predict alone moves omega by
        // q_mul(b_i, i, 0) << 3 = 819 * 100 * 8 = 655200 q16 (~10 c/s) -
        // the Q3.13 encoding couples whole ccounts into csQ16 through the
        // << 3 (the old shift-16 form moved <= 1 q16 per ccount and left
        // b_i inert). The correct step then subtracts l2*e for the 327-q16
        // theta advance: 655200 - (1024 * 327 >> 8) = 653892.
        let mut f = FusionObs::new();
        f.seed(q4(2000));
        step(&mut f, 100, q4(2000), &G);
        assert_eq!(f.omega_q16(), 653892, "pin");
    }

    #[test]
    fn constant_velocity_tracking() {
        // pos ramps 1 count/tick = 2000 c/s; omega settles onto the ramp
        // rate (measured residual ~0.25%, assert 1%).
        let mut f = FusionObs::new();
        f.seed(q4(0));
        for n in 1..=3000u16 {
            step(&mut f, 0, q4(n), &G);
        }
        let target = 2000i32 << 16;
        let err = (f.omega_q16() - target).abs();
        assert!(err <= target / 100, "omega={} err={}", f.omega_q16(), err);
    }

    #[test]
    fn disturbance_step() {
        // Constant drive current with the measurement pinned: predict keeps
        // pushing theta up, e goes negative, so tau_d -= l3*e rises until
        // tau_d ~= i (a load exactly absorbing the drive), while the
        // correct step keeps theta/omega anchored to the measurement.
        let mut f = FusionObs::new();
        f.seed(q4(2000));
        for _ in 0..20000 {
            step(&mut f, 500, q4(2000), &G);
        }
        // the live bleed path settles tau_d onto the drive exactly
        assert_eq!(f.tau_d_counts(), 500, "pin");
        let theta_err = (f.theta_q16() - (2000 << 16)).abs();
        assert!(theta_err < 1 << 16, "theta={}", f.theta_q16());
        assert!(f.omega_q16().abs() < 1 << 16, "omega={}", f.omega_q16());
    }

    /// The servo of record's observer and friction line: corrections 8640
    /// / 3180 / 84, 0.297 c/s per period per ccount, 43 ccounts and 947
    /// Q0.16 ccounts per c/s.
    const RECORD: Gains = Gains {
        l: ConfigFusion {
            l1_q016: 8640,
            l2_q88: 3180,
            l3_q88: 84,
            ..ConfigFusion::new()
        },
        motor: CalibMotor {
            b_i_q313: 2434,
            fric_fc_counts: 43,
            fric_fv_q016: 947,
            ..CalibMotor::new()
        },
    };
    const RECORD_FC: f64 = 43.0;
    const RECORD_FV: f64 = 947.0 / 65536.0;
    const RECORD_B: f64 = 2434.0 / 8192.0;
    /// 2 counts a period.
    const CRUISE_CPS: f64 = 4000.0;
    /// The friction line's viscous part at that speed, ccounts.
    const CRUISE_VISCOUS: i32 = 58;

    /// The Q4 word of a position in counts.
    fn q4f(x: f64) -> u16 {
        (x * (1 << GRID_SHIFT) as f64).round() as u16
    }

    /// The record's observer on a shaft that cruises 400 ms at
    /// `CRUISE_CPS` on the model's own friction line with `load` ccounts
    /// on it (positive against the travel): the current is the line's at
    /// that speed plus the load, so nothing accelerates. The observer and
    /// the position it ends at.
    fn cruise(load: f64) -> (FusionObs, f64) {
        let mut f = FusionObs::new();
        let mut x = 1000.0;
        f.seed(q4f(x));
        let i = (RECORD_FC + RECORD_FV * CRUISE_CPS + load).round() as i32;
        for _ in 0..800 {
            x += CRUISE_CPS / 2000.0;
            step(&mut f, i, q4f(x), &RECORD);
        }
        (f, x)
    }

    /// A free shaft carries no load while it moves. Cruising on the
    /// model's friction line the estimate reads none; with the Coulomb
    /// term alone in the predict the line's viscous part reads as a load
    /// of its own size, 58 ccounts at 4000 c/s against a friction level
    /// of 43.
    #[test]
    fn a_free_cruise_on_the_friction_line_reads_no_load() {
        let (f, _) = cruise(0.0);
        assert!(
            f.tau_d_counts().abs() <= 2,
            "a free cruise reads a load of {} ccounts",
            f.tau_d_counts()
        );
    }

    /// And none when it stops. Brought to rest along the set's ramp
    /// (10305 Q8.8 c/s a period) on the model's own current, the estimate
    /// is well inside the friction level the moment the shaft stops and
    /// stays there: the hold's park test (`position::unloaded`) reads a
    /// free shaft as free at its arrival. Left in the estimate, the
    /// cruise's viscous friction comes down the ramp only to about the
    /// friction level, and the predict's zero band lets a level inside it
    /// stand on a still shaft: the arrival sits on the edge of the test.
    #[test]
    fn a_free_shaft_reads_no_load_at_the_stop_after_a_cruise() {
        const RAMP: f64 = 10305.0 / 256.0;
        let (mut f, mut x) = cruise(0.0);
        let mut v = CRUISE_CPS;
        while v > 0.0 {
            let next = (v - RAMP).max(0.0);
            x += next / 2000.0;
            // the model's current for this period's speed and its change;
            // nothing drives a stopped shaft
            let i = if next > 0.0 {
                RECORD_FC + RECORD_FV * next + (next - v) / RECORD_B
            } else {
                0.0
            };
            step(&mut f, i.round() as i32, q4f(x), &RECORD);
            v = next;
        }
        let half = RECORD_FC as i16 / 2;
        assert!(
            f.tau_d_counts().abs() <= half,
            "at the stop a free shaft reads a load of {} ccounts",
            f.tau_d_counts()
        );
        for period in 0..120 {
            step(&mut f, 0, q4f(x), &RECORD);
            assert!(
                f.tau_d_counts().abs() <= half,
                "{period} periods at rest a free shaft reads a load of {} ccounts",
                f.tau_d_counts()
            );
        }
    }

    /// A load reads its own size at any speed. One of the viscous part's
    /// size that helps the motion cancels it in an estimate that carries
    /// both, and the loaded shaft reads free while it moves; one against
    /// the motion reads twice its size.
    #[test]
    fn a_load_on_a_cruising_shaft_reads_its_own_size() {
        for load in [-CRUISE_VISCOUS, CRUISE_VISCOUS, 120] {
            let (f, _) = cruise(load as f64);
            assert!(
                (f.tau_d_counts() as i32 - load).abs() <= 2,
                "a load of {load} ccounts on a cruising shaft reads {}",
                f.tau_d_counts()
            );
        }
    }

    /// With the viscous word 0 the predict is the Coulomb model alone, and
    /// the same cruise reads the line's viscous part as a load.
    #[test]
    fn a_zero_viscous_word_reads_the_viscous_part_as_a_load() {
        let g = Gains {
            motor: CalibMotor {
                fric_fv_q016: 0,
                ..RECORD.motor
            },
            ..RECORD
        };
        let mut f = FusionObs::new();
        let mut x = 1000.0;
        f.seed(q4f(x));
        let i = (RECORD_FC + RECORD_FV * CRUISE_CPS).round() as i32;
        for _ in 0..800 {
            x += CRUISE_CPS / 2000.0;
            step(&mut f, i, q4f(x), &g);
        }
        assert_eq!(f.tau_d_counts() as i32, CRUISE_VISCOUS, "pin");
    }

    /// A load on a still shaft reads its size with the viscous word set:
    /// at rest the term is nothing.
    #[test]
    fn a_load_on_a_still_shaft_reads_its_own_size() {
        let mut f = FusionObs::new();
        f.seed(q4(2000));
        for _ in 0..20000 {
            step(&mut f, 200, q4(2000), &RECORD);
        }
        assert!(
            (f.tau_d_counts() - 200).abs() <= 1,
            "a load of 200 ccounts on a still shaft reads {}",
            f.tau_d_counts()
        );
    }

    #[test]
    fn unset_model_holds_tau_d_at_zero() {
        // b_i unset, shaft ramping under drive: the innovation still
        // corrects theta and omega, tau_d never leaves zero.
        let g = gains(0, 2048, 0);
        let mut f = FusionObs::new();
        f.seed(q4(2000));
        for n in 0..20000u16 {
            step(&mut f, 500, q4(2000 + n / 16), &g);
            assert_eq!(f.tau_d_counts(), 0, "step {n}");
        }
        let theta_err = (f.theta_q16() - ((2000 + 19999 / 16) << 16)).abs();
        assert!(theta_err < 4 << 16, "theta={}", f.theta_q16());
    }

    #[test]
    fn friction_zero_band() {
        // At rest with a large Coulomb term configured, nothing moves:
        // fric_c(0) = 0, e = 0, all deltas exactly zero.
        let g = gains(819, 2048, 1000);
        let mut f = FusionObs::new();
        f.seed(q4(2048));
        for _ in 0..100 {
            step(&mut f, 0, q4(2048), &g);
            assert_eq!(f.theta_q16(), 2048 << 16);
            assert_eq!(f.omega_q16(), 0);
            assert_eq!(f.tau_d_counts(), 0);
        }
    }

    #[test]
    fn clamps_under_hostile_gains() {
        // Max-encoded gains, extreme inputs: debug overflow checks are the
        // wrap detector; states must stay inside their clamps.
        let g = Gains {
            l: ConfigFusion {
                l1_q016: u16::MAX,
                l2_q88: u16::MAX,
                l3_q88: u16::MAX,
                ..ConfigFusion::new()
            },
            motor: CalibMotor {
                b_i_q313: u16::MAX,
                fric_fc_counts: u16::MAX,
                fric_fv_q016: u16::MAX,
                ..CalibMotor::new()
            },
        };
        let mut f = FusionObs::new();
        for n in 0..2000 {
            let pos = if n & 1 == 0 { 0 } else { 4095 };
            let i = if n & 2 == 0 { i32::MAX } else { i32::MIN };
            step(&mut f, i, q4(pos), &g);
            assert!(f.theta_q16().abs() <= THETA_LIM_CQ16);
            assert!(f.omega_q16().abs() <= OMEGA_LIM_CSQ16);
            assert!((f.tau_d_q16).abs() <= TAU_D_LIM_CCQ16);
        }
    }
}
