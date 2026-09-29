//! Position P loop (control-theory "The Position Loop"): pure proportional -
//! error times kp gives a velocity command, add the omega* feedforward,
//! clamp to vel_limit. NO integrator ever (one integrator per band; a
//! position integrator over Coulomb friction limit-cycles). The anti-hunt
//! hold predicate tells the kernel to park (Brake) inside the deadband
//! instead of dithering against friction.
//!
//! The deadband is raw counts of the position sensor, where its noise and
//! quantisation live, while the error is linearized: the band is the
//! deadband times the table's local gain at the present position.
//!
//! Hold gates on POSITION rest alone (profile stopped, error inside the
//! deadband) - never on omega_hat. The fused omega is pot-noise driven at
//! rest and the velocity loop amplifies it (bench + integer-sim: a ~3x
//! blow-up when driving), so an omega gate that clears the parked noise
//! floor sits below the driving floor: hold latches from rest but can never
//! re-catch once the loop is shaking. Position is the clean rest signal;
//! the braked winding and friction absorb the residual velocity inside the
//! band.

use super::trajectory::VEL_CAP_CPS;
use crate::math::q_mul;

/// P error clamp, cQ16 (128 counts): 2^16 * 2^23 >> 8 < 2^31 keeps the kp
/// product i32-exact for any gain encoding (velocity.rs E_LIM discipline).
/// Past 128 counts of tracking error any practical kp already commands
/// beyond vel_limit, so the flat region costs nothing; the hold predicate
/// compares the RAW error, never this clamp.
const E_LIM_CQ16: i32 = 1 << 23;
/// Band ceiling in Q4 counts: shifted to cQ16 it is 2^32 - 2^12, past any
/// |error| (2^31), so the product saturates instead of wrapping. The
/// identity's largest band, 65535 x 16, sits under it.
const BAND_SAT_Q4: u32 = (1 << 20) - 1;

/// CONFIG loop_position gain + hold fields, loaded fresh each step by the
/// kernel (`CurrentGains` convention).
#[derive(Copy, Clone)]
pub struct PositionCfg {
    pub kp_q88: u16,
    /// Raw sensor counts; 0 disables the hold predicate entirely.
    pub pos_deadband_counts: u16,
    /// Linearized counts per raw count at the present position, Q4 (16 at
    /// the identity), clamped (`pos_lut::band_gain_q4`).
    pub band_gain_q4: u16,
    pub vel_limit_cps: u16,
}

/// omega_ref for the velocity loop plus the anti-hunt flag (kernel maps
/// hold -> Brake).
#[derive(Copy, Clone)]
pub struct PosOut {
    pub omega_ref_q16: i32,
    pub hold: bool,
}

/// One MEDIUM-tick update; stateless. theta*/omega* come from the
/// trajectory generator, theta_hat/omega_hat from fusion.
pub fn step(
    theta_star_q16: i32,
    omega_star_q16: i32,
    theta_hat_q16: i32,
    cfg: &PositionCfg,
) -> PosOut {
    let e_raw = theta_star_q16.saturating_sub(theta_hat_q16);
    let e = e_raw.clamp(-E_LIM_CQ16, E_LIM_CQ16);
    let v_lim = (cfg.vel_limit_cps.min(VEL_CAP_CPS) as i32) << 16;
    // Q8.8 * cQ16 >> 8 -> csQ16, i32-exact per E_LIM
    let omega_ref = q_mul(cfg.kp_q88 as i32, e, 8)
        .saturating_add(omega_star_q16)
        .clamp(-v_lim, v_lim);
    // raw-magnitude compares as u32: the band reaches 2^32 - 2^12, past i32
    let band = (cfg.pos_deadband_counts as u32 * cfg.band_gain_q4 as u32).min(BAND_SAT_Q4) << 12;
    let hold = cfg.pos_deadband_counts != 0 && e_raw.unsigned_abs() <= band && omega_star_q16 == 0;
    PosOut {
        omega_ref_q16: omega_ref,
        hold,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(kp: u16, db: u16, vl: u16) -> PositionCfg {
        PositionCfg {
            kp_q88: kp,
            pos_deadband_counts: db,
            band_gain_q4: 16,
            vel_limit_cps: vl,
        }
    }

    #[test]
    fn hold_band_scales_by_the_local_gain() {
        // gain 26/16: 12 raw counts span 19.5 linearized
        let cfg = PositionCfg {
            band_gain_q4: 26,
            ..c(1 << 8, 12, 32767)
        };
        assert!(step(39 << 15, 0, 0, &cfg).hold);
        assert!(step(0, 0, 39 << 15, &cfg).hold);
        assert!(!step((39 << 15) + 1, 0, 0, &cfg).hold);
        assert!(!step(0, 0, (39 << 15) + 1, &cfg).hold);
        // the largest deadband at the ceiling saturates past any error
        let cfg = PositionCfg {
            band_gain_q4: 64,
            ..c(1 << 8, 65535, 32767)
        };
        assert!(step(i32::MAX, 0, i32::MIN, &cfg).hold);
        assert!(step(i32::MIN, 0, i32::MAX, &cfg).hold);
    }

    #[test]
    fn p_term_sign_and_scale() {
        // kp 1.0 c/s per count: 100 counts of error -> 100 c/s
        let cfg = c(1 << 8, 0, 32767);
        assert_eq!(step(100 << 16, 0, 0, &cfg).omega_ref_q16, 100 << 16);
        assert_eq!(step(0, 0, 100 << 16, &cfg).omega_ref_q16, -(100 << 16));
        // kp 1/256: 100 counts -> 100/256 c/s = 100 << 8 in csQ16
        let cfg = c(1, 0, 32767);
        assert_eq!(step(100 << 16, 0, 0, &cfg).omega_ref_q16, 100 << 8);
        assert_eq!(step(0, 0, 100 << 16, &cfg).omega_ref_q16, -(100 << 8));
    }

    #[test]
    fn feedforward_adds() {
        // kp 0: pure passthrough of omega*
        let cfg = c(0, 0, 32767);
        assert_eq!(step(100 << 16, 200 << 16, 0, &cfg).omega_ref_q16, 200 << 16);
        // kp term and omega* sum
        let cfg = c(1 << 8, 0, 32767);
        assert_eq!(step(10 << 16, 200 << 16, 0, &cfg).omega_ref_q16, 210 << 16);
    }

    #[test]
    fn vel_limit_and_error_clamp() {
        // 1000 counts of error clamps to E_LIM's 128 first: kp 1.0 -> 128 c/s
        let cfg = c(1 << 8, 0, 32767);
        assert_eq!(step(1000 << 16, 0, 0, &cfg).omega_ref_q16, 128 << 16);
        // vel_limit clamps the sum, both signs
        let cfg = c(1 << 8, 0, 100);
        assert_eq!(step(1000 << 16, 0, 0, &cfg).omega_ref_q16, 100 << 16);
        assert_eq!(step(0, 0, 1000 << 16, &cfg).omega_ref_q16, -(100 << 16));
        // feedforward alone also cannot escape the limit
        assert_eq!(step(0, 500 << 16, 0, &cfg).omega_ref_q16, 100 << 16);
    }

    #[test]
    fn hold_predicate_each_condition() {
        let cfg = c(1 << 8, 5, 32767);
        // both hold conditions true: within deadband and profile at rest
        assert!(step(5 << 16, 0, 0, &cfg).hold);
        // deadband 0 disables hold outright, even at perfect rest
        assert!(!step(0, 0, 0, &c(1 << 8, 0, 32767)).hold);
        // error one cQ16 past the deadband
        assert!(!step((5 << 16) + 1, 0, 0, &cfg).hold);
        // omega* nonzero: profile still moving
        assert!(!step(5 << 16, 1, 0, &cfg).hold);
    }

    #[test]
    fn hold_uses_raw_error_not_clamped() {
        // 6000 counts of error is far past E_LIM's 128 but inside a 65535
        // deadband: the predicate must see the raw magnitude and hold
        let cfg = c(1 << 8, 65535, 32767);
        let out = step(3000 << 16, 0, -(3000 << 16), &cfg);
        assert!(out.hold);
        // while the drive command stays E_LIM/vel_limit-clamped
        assert_eq!(out.omega_ref_q16, 128 << 16);
        // and a small deadband rejects the same error
        assert!(!step(3000 << 16, 0, -(3000 << 16), &c(1 << 8, 5, 32767)).hold);
    }
}
