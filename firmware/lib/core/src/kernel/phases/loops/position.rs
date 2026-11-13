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
//! A shaft resting on the band's edge is released by every dip of sensor
//! noise, each wake too short to move it, so "parked for the dwell" may
//! never come true. The dwell (`HOLD_SETTLE_TICKS`) therefore runs while the
//! error stays within twice the band of a stopped goal: a shaft that has not
//! left that zone for the dwell is at rest there, and from then on the zone
//! is the band. Before the dwell the band is the gate both ways, so a moving
//! shaft is caught and a coast through is redriven as before.
//!
//! Where the play between the motor and the position sensor is wider than
//! the band, a free output can rest outside the band with the motor at the
//! goal, and the loop then throws it from side to side without end. A shaft
//! that has stayed inside the zone for the dwell is parked where it is, so
//! that oscillation ends with the dwell.
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
use crate::regions::config::ConfigLoopPosition;

/// P error clamp, cQ16 (128 counts): 2^16 * 2^23 >> 8 < 2^31 keeps the kp
/// product i32-exact for any gain encoding (velocity.rs E_LIM discipline).
/// Past 128 counts of tracking error any practical kp already commands
/// beyond vel_limit, so the flat region costs nothing; the hold predicate
/// compares the RAW error, never this clamp.
const E_LIM_CQ16: i32 = 1 << 23;

/// Medium ticks the error must stay within twice the band of a stopped goal
/// before the hold settles (64 ms at 2 kHz). A shaft that stays in that
/// zone that long moves slower than its width per dwell: it is nearly
/// stopped, not coasting through.
pub const HOLD_SETTLE_TICKS: u8 = 128;

/// omega_ref for the velocity loop, the anti-hunt flag (kernel maps hold ->
/// Brake) and whether the error is within twice the band of a stopped goal
/// (the kernel counts it toward `HOLD_SETTLE_TICKS`).
#[derive(Copy, Clone)]
pub struct PosOut {
    pub omega_ref_q16: i32,
    pub hold: bool,
    pub near: bool,
}

/// The hold band, cQ16: the deadband's raw counts times the local gain
/// (`band_gain_q4`, Q4). The zone is twice it.
#[inline(always)]
pub fn band_q16(deadband_counts: u16, band_gain_q4: u16) -> u32 {
    // Q4 -> cQ16 is << 12. Raw-magnitude compares as u32: the ceiling and
    // a saturating doubling hold past any |error| (2^31) instead of
    // wrapping; the identity's largest band, 65535 x 16, sits under both.
    (deadband_counts as u32 * band_gain_q4 as u32).min(u32::MAX >> 12) << 12
}

/// One MEDIUM-tick update; stateless. theta*/omega* come from the
/// trajectory generator, theta_hat/omega_hat from fusion. `band_gain_q4` is
/// linearized counts per raw count at the present position, Q4 (16 at the
/// identity), clamped (`pos_lut::band_gain_q4`). `settled`: `near` has held
/// for `HOLD_SETTLE_TICKS`, so the gate is twice the band.
pub fn step(
    theta_star_q16: i32,
    omega_star_q16: i32,
    theta_hat_q16: i32,
    band_gain_q4: u16,
    settled: bool,
    cfg: &ConfigLoopPosition,
) -> PosOut {
    let e_raw = theta_star_q16.saturating_sub(theta_hat_q16);
    let e = e_raw.clamp(-E_LIM_CQ16, E_LIM_CQ16);
    // the table rule bounds the limit; the min keeps the clamp below
    // panic-free on a corrupt one
    let v_lim = (cfg.velocity_limit_cps.min(VEL_CAP_CPS) as i32) << 16;
    // Q8.8 * cQ16 >> 8 -> csQ16, i32-exact per E_LIM
    let omega_ref = q_mul(cfg.p_kp_q88 as i32, e, 8)
        .saturating_add(omega_star_q16)
        .clamp(-v_lim, v_lim);
    let band = band_q16(cfg.pos_deadband_counts, band_gain_q4);
    let zone = band.saturating_add(band);
    let e_abs = e_raw.unsigned_abs();
    let near = cfg.pos_deadband_counts != 0 && omega_star_q16 == 0 && e_abs <= zone;
    PosOut {
        omega_ref_q16: omega_ref,
        // the band sits inside the zone
        hold: near && (settled || e_abs <= band),
        near,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The identity's band gain.
    const ID: u16 = crate::pos_lut::GRID as u16;

    fn c(kp: u16, db: u16, vl: u16) -> ConfigLoopPosition {
        ConfigLoopPosition {
            p_kp_q88: kp,
            pos_deadband_counts: db,
            velocity_limit_cps: vl,
            accel_limit_q88: 0,
        }
    }

    #[test]
    fn band_is_the_deadband_times_the_gain() {
        assert_eq!(band_q16(12, ID), 12 << 16);
        // gain 26/16: 12 raw counts span 19.5 linearized
        assert_eq!(band_q16(12, 26), 39 << 15);
        assert_eq!(band_q16(0, 26), 0);
        assert_eq!(band_q16(12, 0), 0);
        // the ceiling: the largest product stays under 2^32, and so does
        // the identity's largest band
        assert_eq!(band_q16(u16::MAX, u16::MAX), (u32::MAX >> 12) << 12);
        assert_eq!(band_q16(u16::MAX, ID), (u16::MAX as u32) << 16);
    }

    #[test]
    fn hold_band_scales_by_the_local_gain() {
        // gain 26/16: 12 raw counts span 19.5 linearized, twice is 39
        let cfg = c(1 << 8, 12, 32767);
        for (settled, gate) in [(false, 39 << 15), (true, 39 << 16)] {
            assert!(step(gate, 0, 0, 26, settled, &cfg).hold);
            assert!(step(0, 0, gate, 26, settled, &cfg).hold);
            assert!(!step(gate + 1, 0, 0, 26, settled, &cfg).hold);
            assert!(!step(0, 0, gate + 1, 26, settled, &cfg).hold);
        }
        // the largest deadband at the ceiling saturates either gate past
        // any error
        let cfg = c(1 << 8, 65535, 32767);
        for settled in [false, true] {
            assert!(step(i32::MAX, 0, i32::MIN, 64, settled, &cfg).hold);
            assert!(step(i32::MIN, 0, i32::MAX, 64, settled, &cfg).hold);
        }
    }

    #[test]
    fn near_is_two_bands_of_a_stopped_goal() {
        let cfg = c(1 << 8, 12, 32767);
        for settled in [false, true] {
            assert!(step(24 << 16, 0, 0, ID, settled, &cfg).near);
            assert!(step(0, 0, 24 << 16, ID, settled, &cfg).near);
            assert!(!step((24 << 16) + 1, 0, 0, ID, settled, &cfg).near);
            assert!(!step(0, 0, (24 << 16) + 1, ID, settled, &cfg).near);
            // a moving profile is never near, even at zero error
            assert!(!step(0, 1, 0, ID, settled, &cfg).near);
            assert!(!step(0, -1, 0, ID, settled, &cfg).near);
            // deadband 0 never counts
            assert!(!step(0, 0, 0, ID, settled, &c(1 << 8, 0, 32767)).near);
        }
        // the ceiling saturates the zone past any error
        let cfg = c(1 << 8, 65535, 32767);
        assert!(step(i32::MAX, 0, i32::MIN, 64, false, &cfg).near);
        assert!(step(i32::MIN, 0, i32::MAX, 64, false, &cfg).near);
    }

    #[test]
    fn a_settled_park_releases_past_two_bands() {
        let cfg = c(1 << 8, 12, 32767);
        // entry and an un-settled park: the band, not past it
        assert!(step(12 << 16, 0, 0, ID, false, &cfg).hold);
        assert!(step(0, 0, 12 << 16, ID, false, &cfg).hold);
        assert!(!step((12 << 16) + 1, 0, 0, ID, false, &cfg).hold);
        assert!(!step(18 << 16, 0, 0, ID, false, &cfg).hold);
        // settled: twice the band, not past it
        assert!(step(18 << 16, 0, 0, ID, true, &cfg).hold);
        assert!(step(0, 0, 24 << 16, ID, true, &cfg).hold);
        assert!(!step((24 << 16) + 1, 0, 0, ID, true, &cfg).hold);
        assert!(!step(0, 0, (24 << 16) + 1, ID, true, &cfg).hold);
        // a moving profile drops either, even at zero error
        for settled in [false, true] {
            assert!(!step(0, 1, 0, ID, settled, &cfg).hold);
            assert!(!step(0, -1, 0, ID, settled, &cfg).hold);
        }
        // deadband 0 never parks, settled or not
        let off = c(1 << 8, 0, 32767);
        for settled in [false, true] {
            assert!(!step(0, 0, 0, ID, settled, &off).hold);
        }
    }

    #[test]
    fn p_term_sign_and_scale() {
        // kp 1.0 c/s per count: 100 counts of error -> 100 c/s
        let cfg = c(1 << 8, 0, 32767);
        assert_eq!(
            step(100 << 16, 0, 0, ID, false, &cfg).omega_ref_q16,
            100 << 16
        );
        assert_eq!(
            step(0, 0, 100 << 16, ID, false, &cfg).omega_ref_q16,
            -(100 << 16)
        );
        // kp 1/256: 100 counts -> 100/256 c/s = 100 << 8 in csQ16
        let cfg = c(1, 0, 32767);
        assert_eq!(
            step(100 << 16, 0, 0, ID, false, &cfg).omega_ref_q16,
            100 << 8
        );
        assert_eq!(
            step(0, 0, 100 << 16, ID, false, &cfg).omega_ref_q16,
            -(100 << 8)
        );
    }

    #[test]
    fn feedforward_adds() {
        // kp 0: pure passthrough of omega*
        let cfg = c(0, 0, 32767);
        assert_eq!(
            step(100 << 16, 200 << 16, 0, ID, false, &cfg).omega_ref_q16,
            200 << 16
        );
        // kp term and omega* sum
        let cfg = c(1 << 8, 0, 32767);
        assert_eq!(
            step(10 << 16, 200 << 16, 0, ID, false, &cfg).omega_ref_q16,
            210 << 16
        );
    }

    #[test]
    fn vel_limit_and_error_clamp() {
        // 1000 counts of error clamps to E_LIM's 128 first: kp 1.0 -> 128 c/s
        let cfg = c(1 << 8, 0, 32767);
        assert_eq!(
            step(1000 << 16, 0, 0, ID, false, &cfg).omega_ref_q16,
            128 << 16
        );
        // vel_limit clamps the sum, both signs
        let cfg = c(1 << 8, 0, 100);
        assert_eq!(
            step(1000 << 16, 0, 0, ID, false, &cfg).omega_ref_q16,
            100 << 16
        );
        assert_eq!(
            step(0, 0, 1000 << 16, ID, false, &cfg).omega_ref_q16,
            -(100 << 16)
        );
        // feedforward alone also cannot escape the limit
        assert_eq!(
            step(0, 500 << 16, 0, ID, false, &cfg).omega_ref_q16,
            100 << 16
        );
    }

    #[test]
    fn hold_predicate_each_condition() {
        let cfg = c(1 << 8, 5, 32767);
        // both hold conditions true: within deadband and profile at rest
        assert!(step(5 << 16, 0, 0, ID, false, &cfg).hold);
        // deadband 0 disables hold outright, even at perfect rest
        assert!(!step(0, 0, 0, ID, false, &c(1 << 8, 0, 32767)).hold);
        // error one cQ16 past the deadband
        assert!(!step((5 << 16) + 1, 0, 0, ID, false, &cfg).hold);
        // omega* nonzero: profile still moving
        assert!(!step(5 << 16, 1, 0, ID, false, &cfg).hold);
    }

    #[test]
    fn hold_uses_raw_error_not_clamped() {
        // 6000 counts of error is far past E_LIM's 128 but inside a 65535
        // deadband: the predicate must see the raw magnitude and hold
        let cfg = c(1 << 8, 65535, 32767);
        let out = step(3000 << 16, 0, -(3000 << 16), ID, false, &cfg);
        assert!(out.hold);
        // while the drive command stays E_LIM/vel_limit-clamped
        assert_eq!(out.omega_ref_q16, 128 << 16);
        // and a small deadband rejects the same error
        assert!(
            !step(
                3000 << 16,
                0,
                -(3000 << 16),
                ID,
                false,
                &c(1 << 8, 5, 32767)
            )
            .hold
        );
    }
}
