//! Position P loop (control-theory "The Position Loop"): pure proportional -
//! error times kp gives a velocity command, add the omega* feedforward,
//! clamp to the speed ceiling. NO integrator ever (one integrator per band; a
//! position integrator over Coulomb friction limit-cycles). The anti-hunt
//! hold predicate tells the kernel to stop correcting inside the deadband
//! instead of dithering against friction: it parks (Brake) a free shaft and
//! keeps a loaded one's current.
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
//! Only a shaft that holds no load parks. The park drops the current and
//! the velocity integral, so a shaft that needed that current to stay put
//! springs back out of the zone, is driven again from an empty integral and
//! parks again short of the goal: it rests on the zone's far edge on the
//! load's side. The park therefore also needs the observer's disturbance
//! estimate within the friction level (`fric_fc_counts`): a free output at
//! rest carries none and parks as before. With no motor model the estimate
//! is held at 0 and the gate is open.
//!
//! A loaded shaft is held instead, in the region a free one parks in: the
//! velocity loop keeps its integral frozen, so the drive keeps the current
//! it settled on. Left integrating, a still shaft's in-band error winds the
//! current up until it breaks loose of the train's static friction and
//! jumps past the goal, where it sticks again: a stick-slip the integrator
//! drives. Once settled the zone is the gate, so the noise on the estimate
//! at the band's edge never wakes the integral; a held shaft whose load
//! goes away parks where it is.
//!
//! Hold gates on POSITION rest (profile stopped, error inside the
//! deadband) and the load estimate - never on omega_hat. The fused omega is
//! pot-noise driven at rest and the velocity loop amplifies it (bench +
//! integer-sim: a ~3x blow-up when driving), so an omega gate that clears
//! the parked noise floor sits below the driving floor: hold latches from
//! rest but can never re-catch once the loop is shaking. Position is the
//! clean rest signal; the braked winding and friction absorb the residual
//! velocity inside the band.

use super::trajectory::VEL_CAP_CPS;
use crate::math::{q_mul, sat_add, sat_sub, sat_sym};
use crate::regions::config::ConfigLoopPosition;

/// P error clamp, cQ16 (128 counts): 2^16 * 2^23 >> 8 < 2^31 keeps the kp
/// product i32-exact for any gain encoding (velocity.rs E_LIM discipline).
/// Past 128 counts of tracking error any practical kp already commands
/// beyond the ceiling, so the flat region costs nothing; the hold predicate
/// compares the RAW error, never this clamp.
const E_LIM_CQ16: i32 = 1 << 23;

/// Medium ticks the error must stay within twice the band of a stopped goal
/// before the hold settles (64 ms at 2 kHz). A shaft that stays in that
/// zone that long moves slower than its width per dwell: it is nearly
/// stopped, not coasting through.
pub const HOLD_SETTLE_TICKS: u8 = 128;

/// omega_ref for the velocity loop, the anti-hunt flags and whether the
/// error is within twice the band of a stopped goal (the kernel counts it
/// toward `HOLD_SETTLE_TICKS`). Inside the band of a stopped goal, or once
/// settled the zone: `park` for an unloaded shaft (the kernel maps it to
/// Brake), `hold` for a loaded one (it keeps its current, its velocity
/// integral frozen). At most one of the two is set.
#[derive(Copy, Clone)]
pub struct PosOut {
    pub omega_ref_q16: i32,
    pub hold: bool,
    pub park: bool,
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

/// Whether a shaft carries no load a park would drop: the observer's
/// disturbance `tau_d_q16` (ccQ16) within the friction level (ccounts).
#[inline(always)]
pub fn unloaded(tau_d_q16: i32, fric_fc_counts: u16) -> bool {
    // ccounts -> ccQ16 fits u32 for any level
    tau_d_q16.unsigned_abs() <= (fric_fc_counts as u32) << 16
}

/// One MEDIUM-tick update; stateless. theta*/omega* come from the
/// trajectory generator, theta_hat/omega_hat from fusion. `band_gain_q4` is
/// linearized counts per raw count at the present position, Q4 (16 at the
/// identity), clamped (`pos_lut::band_gain_q4`). `settled`: `near` has held
/// for `HOLD_SETTLE_TICKS`, so the gate is twice the band. `unloaded`:
/// [`unloaded`] on this period's estimate.
pub fn step(
    theta_star_q16: i32,
    omega_star_q16: i32,
    theta_hat_q16: i32,
    band_gain_q4: u16,
    settled: bool,
    unloaded: bool,
    cfg: &ConfigLoopPosition,
) -> PosOut {
    let e_raw = sat_sub(theta_star_q16, theta_hat_q16);
    let e = sat_sym(e_raw, E_LIM_CQ16);
    // the speed ceiling, not the move speed: a shaft behind its profile is
    // asked for more, up to the ceiling. The table rule bounds it; the min
    // keeps the shift below from turning v_lim negative on a corrupt one
    let v_lim = (cfg.speed_ceiling_cps.min(VEL_CAP_CPS) as i32) << 16;
    // Q8.8 * cQ16 >> 8 -> csQ16, i32-exact per E_LIM
    let omega_ref = sat_sym(
        sat_add(q_mul(cfg.p_kp_q88 as i32, e, 8), omega_star_q16),
        v_lim,
    );
    // a moving profile skips the band's multiply
    if cfg.pos_deadband_counts == 0 || omega_star_q16 != 0 {
        return PosOut {
            omega_ref_q16: omega_ref,
            hold: false,
            park: false,
            near: false,
        };
    }
    let band = band_q16(cfg.pos_deadband_counts, band_gain_q4);
    let zone = band.saturating_add(band);
    let e_abs = e_raw.unsigned_abs();
    let near = e_abs <= zone;
    // the band sits inside the zone
    let hit = near && (e_abs <= band || settled);
    PosOut {
        omega_ref_q16: omega_ref,
        hold: hit && !unloaded,
        park: hit && unloaded,
        near,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The identity's band gain.
    const ID: u16 = crate::pos_lut::GRID as u16;

    /// The step on an unloaded shaft.
    fn step(
        theta_star_q16: i32,
        omega_star_q16: i32,
        theta_hat_q16: i32,
        band_gain_q4: u16,
        settled: bool,
        cfg: &ConfigLoopPosition,
    ) -> PosOut {
        super::step(
            theta_star_q16,
            omega_star_q16,
            theta_hat_q16,
            band_gain_q4,
            settled,
            true,
            cfg,
        )
    }

    /// The block as the take leaves it: the move speed at the ceiling.
    fn c(kp: u16, db: u16, ceiling: u16) -> ConfigLoopPosition {
        ConfigLoopPosition {
            p_kp_q88: kp,
            pos_deadband_counts: db,
            speed_ceiling_cps: ceiling,
            velocity_limit_cps: ceiling,
            accel_limit_q88: 0,
            _rsvd_align: 0,
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
            assert!(step(gate, 0, 0, 26, settled, &cfg).park);
            assert!(step(0, 0, gate, 26, settled, &cfg).park);
            assert!(!step(gate + 1, 0, 0, 26, settled, &cfg).park);
            assert!(!step(0, 0, gate + 1, 26, settled, &cfg).park);
        }
        // the largest deadband at the ceiling saturates either gate past
        // any error
        let cfg = c(1 << 8, 65535, 32767);
        for settled in [false, true] {
            assert!(step(i32::MAX, 0, i32::MIN, 64, settled, &cfg).park);
            assert!(step(i32::MIN, 0, i32::MAX, 64, settled, &cfg).park);
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
        assert!(step(12 << 16, 0, 0, ID, false, &cfg).park);
        assert!(step(0, 0, 12 << 16, ID, false, &cfg).park);
        assert!(!step((12 << 16) + 1, 0, 0, ID, false, &cfg).park);
        assert!(!step(18 << 16, 0, 0, ID, false, &cfg).park);
        // settled: twice the band, not past it
        assert!(step(18 << 16, 0, 0, ID, true, &cfg).park);
        assert!(step(0, 0, 24 << 16, ID, true, &cfg).park);
        assert!(!step((24 << 16) + 1, 0, 0, ID, true, &cfg).park);
        assert!(!step(0, 0, (24 << 16) + 1, ID, true, &cfg).park);
        // a moving profile drops either, even at zero error
        for settled in [false, true] {
            assert!(!step(0, 1, 0, ID, settled, &cfg).park);
            assert!(!step(0, -1, 0, ID, settled, &cfg).park);
        }
        // deadband 0 never parks, settled or not
        let off = c(1 << 8, 0, 32767);
        for settled in [false, true] {
            assert!(!step(0, 0, 0, ID, settled, &off).park);
        }
    }

    #[test]
    fn unloaded_is_the_disturbance_within_the_friction_level() {
        let level = 43 << 16;
        for tau in [0, 1, -1, level, -level] {
            assert!(unloaded(tau, 43), "tau {tau}");
        }
        for tau in [level + 1, -level - 1, i32::MAX, i32::MIN] {
            assert!(!unloaded(tau, 43), "tau {tau}");
        }
        // the largest level spans every disturbance, a level of 0 only none
        assert!(unloaded(i32::MIN, u16::MAX));
        assert!(unloaded(0, 0));
        assert!(!unloaded(1, 0));
        assert!(!unloaded(-1, 0));
    }

    #[test]
    fn a_loaded_shaft_is_held_but_never_parks() {
        let cfg = c(1 << 8, 12, 32767);
        let step = |e: i32, settled: bool, unloaded: bool| {
            super::step(e, 0, 0, ID, settled, unloaded, &cfg)
        };
        // the free shaft's park region holds a loaded one: the band, and
        // once settled the zone
        for (e, settled) in [
            (0, false),
            (12 << 16, false),
            (-(12 << 16), false),
            (0, true),
            ((12 << 16) + 1, true),
            (24 << 16, true),
            (-(24 << 16), true),
        ] {
            let free = step(e, settled, true);
            let loaded = step(e, settled, false);
            assert!(free.park && !free.hold, "e {e}, settled {settled}");
            assert!(loaded.hold && !loaded.park, "e {e}, settled {settled}");
            // the dwell counts on position alone, and the loop's output is
            // the free shaft's
            assert!(loaded.near);
            assert_eq!(loaded.omega_ref_q16, free.omega_ref_q16);
        }
        // past the band before the dwell, past the zone, and on a moving
        // profile, neither
        for unloaded in [false, true] {
            for (e, omega_star, settled) in [
                ((12 << 16) + 1, 0, false),
                (25 << 16, 0, true),
                (0, 1, true),
            ] {
                let out = super::step(e, omega_star, 0, ID, settled, unloaded, &cfg);
                assert!(!out.hold && !out.park, "e {e}, unloaded {unloaded}");
            }
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
    fn ceiling_and_error_clamp() {
        // 1000 counts of error clamps to E_LIM's 128 first: kp 1.0 -> 128 c/s
        let cfg = c(1 << 8, 0, 32767);
        assert_eq!(
            step(1000 << 16, 0, 0, ID, false, &cfg).omega_ref_q16,
            128 << 16
        );
        // the ceiling clamps the sum, both signs
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
        // the move speed under the ceiling bounds nothing here
        let cfg = ConfigLoopPosition {
            velocity_limit_cps: 30,
            ..c(1 << 8, 0, 100)
        };
        assert_eq!(
            step(1000 << 16, 0, 0, ID, false, &cfg).omega_ref_q16,
            100 << 16
        );
    }

    #[test]
    fn hold_predicate_each_condition() {
        let cfg = c(1 << 8, 5, 32767);
        // both hold conditions true: within deadband and profile at rest
        assert!(step(5 << 16, 0, 0, ID, false, &cfg).park);
        // deadband 0 disables hold outright, even at perfect rest
        assert!(!step(0, 0, 0, ID, false, &c(1 << 8, 0, 32767)).park);
        // error one cQ16 past the deadband
        assert!(!step((5 << 16) + 1, 0, 0, ID, false, &cfg).park);
        // omega* nonzero: profile still moving
        assert!(!step(5 << 16, 1, 0, ID, false, &cfg).park);
    }

    #[test]
    fn hold_uses_raw_error_not_clamped() {
        // 6000 counts of error is far past E_LIM's 128 but inside a 65535
        // deadband: the predicate must see the raw magnitude and hold
        let cfg = c(1 << 8, 65535, 32767);
        let out = step(3000 << 16, 0, -(3000 << 16), ID, false, &cfg);
        assert!(out.park);
        // while the drive command stays E_LIM/ceiling-clamped
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
            .park
        );
    }
}
