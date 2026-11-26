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
//! A shaft the play throws out of the band or the zone at every push is
//! caught and lost without end: the push the loop keeps up on it is what
//! throws it. A hold lost [`HOLD_THROWN`] times running is thrown ([`Thrown`]):
//! once the raw pot sample is at rest inside its reach it parks there, if
//! the load estimate is within the friction level, the plain park's own
//! test: a thrown shaft the estimate calls loaded is the plain hold, so its
//! current, and the stall and position error timers on it, are as without
//! the rule. The reach starts at the zone and grows a band for every second
//! further loss, to [`HOLD_REACH_BANDS`], so a hold with a resting place
//! inside the zone ends there and only the wide throws are taken farther
//! out. That park is kept to a band past its reach and while the estimate
//! stays within twice the friction level; lost, the count clears, as it
//! does on a loss after the dwell, a moving profile, the run edge and a
//! mode change. Under the count every output is the plain hold's.
//!
//! Hold gates on POSITION rest (profile stopped, error inside the
//! deadband, or the thrown hold's rest on the raw sample) and the load
//! estimate - never on omega_hat. The fused omega is
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
/// integral frozen). At most one of the two is set. On a stopped goal the
/// band and the raw error's magnitude it was judged on, else 0.
#[derive(Copy, Clone)]
pub struct PosOut {
    pub omega_ref_q16: i32,
    pub hold: bool,
    pub park: bool,
    pub near: bool,
    pub band: u32,
    pub e_abs: u32,
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
            band: 0,
            e_abs: 0,
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
        band,
        e_abs,
    }
}

/// Catches lost running from which a hold is thrown. The landing itself
/// loses up to two.
pub const HOLD_THROWN: u8 = 3;

/// Medium periods the raw pot sample stays within [`HOLD_STILL_COUNTS`]
/// for the shaft to be at rest (6 ms at 2 kHz).
pub const HOLD_REST_TICKS: u8 = 12;

/// Raw counts the sample may move and still be at rest: a throw moves 2 to
/// 5 a period.
pub const HOLD_STILL_COUNTS: u16 = 2;

/// The thrown hold's widest reach, bands.
pub const HOLD_REACH_BANDS: u8 = 5;

/// The thrown hold: the catches the loop lost running, the rest on the raw
/// sample, and whether the hold parked on them.
pub struct Thrown {
    lost: u8,
    still: u8,
    anchor: u16,
    at_rest: bool,
}

impl Thrown {
    pub const fn new() -> Self {
        Self {
            lost: 0,
            still: 0,
            anchor: 0,
            at_rest: false,
        }
    }

    #[inline(always)]
    pub fn clear(&mut self) {
        self.lost = 0;
        self.still = 0;
        self.at_rest = false;
    }

    #[cfg(test)]
    pub fn lost(&self) -> u8 {
        self.lost
    }

    #[cfg(test)]
    pub fn at_rest(&self) -> bool {
        self.at_rest
    }

    /// Lost `lost` times running, not at rest yet.
    #[cfg(test)]
    pub fn lost_times(lost: u8) -> Self {
        Self {
            lost,
            ..Self::new()
        }
    }

    /// Parked at rest on `raw` by the rule on a count of `lost`.
    #[cfg(test)]
    pub fn parked_at(raw: u16, lost: u8) -> Self {
        Self {
            lost,
            still: HOLD_REST_TICKS,
            anchor: raw,
            at_rest: true,
        }
    }

    /// One period of a profile resting on its goal, after [`step`]:
    /// whether the hold parks where `step` did not. `raw` is the tick's
    /// pot sample, `e_abs` and `band` `step`'s. `lost`: a catch ended, the
    /// last period parked or held and this one's `step` does neither;
    /// `settled`: the dwell `step` ran on was over. `tau_abs` is the load
    /// estimate's magnitude and `fric` the friction level, both ccQ16: the
    /// park is taken only on a shaft [`unloaded`] calls free, and kept
    /// only while the estimate stays inside twice that level.
    #[inline(always)]
    #[allow(clippy::too_many_arguments)]
    pub fn parks(
        &mut self,
        raw: u16,
        e_abs: u32,
        band: u32,
        lost: bool,
        settled: bool,
        tau_abs: u32,
        fric: u32,
    ) -> bool {
        if raw.abs_diff(self.anchor) > HOLD_STILL_COUNTS {
            self.anchor = raw;
            self.still = 0;
        } else {
            self.still += (self.still < HOLD_REST_TICKS) as u8;
        }
        if self.at_rest {
            // an undriven shaft at rest that leaves, or one the estimate
            // shows a load on, is the plain hold's again. Twice the level
            // the park was taken under: the estimate's noise on a free
            // shaft never ends it, a load does before it moves the shaft
            if e_abs <= self.reach(band).saturating_add(band) && tau_abs >> 1 <= fric {
                return true;
            }
            self.clear();
            return false;
        }
        if lost {
            // a shaft that had come to rest was moved by something else
            self.lost = if settled {
                0
            } else {
                self.lost.saturating_add(1)
            };
        }
        if self.lost < HOLD_THROWN || self.still < HOLD_REST_TICKS {
            return false;
        }
        // the plain hold's load test: the rule widens where a free shaft
        // parks, never which shaft is free
        if tau_abs > fric {
            return false;
        }
        self.at_rest = e_abs <= self.reach(band);
        self.at_rest
    }

    /// The zone, a band more for every second loss past the throw.
    #[inline(always)]
    fn reach(&self, band: u32) -> u32 {
        let bands = (2 + (self.lost.saturating_sub(HOLD_THROWN) >> 1)).min(HOLD_REACH_BANDS);
        band.saturating_mul(bands as u32)
    }
}

impl Default for Thrown {
    fn default() -> Self {
        Self::new()
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

    /// The identity's band at the deadband of 4.
    const B: u32 = 4 << 16;

    /// A friction level of 43 counts, ccQ16.
    const FRIC: u32 = 43 << 16;

    /// The periods of a rest: the anchor's and [`HOLD_REST_TICKS`] on it.
    const REST: u32 = HOLD_REST_TICKS as u32 + 1;

    /// `n` uncaught periods at rest on `raw`, `e_abs` off the goal: whether
    /// the last one parks.
    fn rest(t: &mut Thrown, raw: u16, e_abs: u32, n: u32) -> bool {
        let mut parks = false;
        for _ in 0..n {
            parks = t.parks(raw, e_abs, B, false, false, 0, FRIC);
        }
        parks
    }

    /// `n` catches lost before the dwell by a shaft in flight past `raw`,
    /// each a caught period and the next.
    fn lose(t: &mut Thrown, raw: u16, n: u8) {
        for _ in 0..n {
            assert!(!t.parks(raw + 10, 0, B, false, false, 0, FRIC));
            assert!(!t.parks(raw + 20, 3 * B, B, true, false, 0, FRIC));
        }
    }

    #[test]
    fn a_hold_lost_twice_is_the_plain_hold() {
        let mut t = Thrown::new();
        lose(&mut t, 2000, 2);
        assert_eq!(t.lost(), 2);
        // at rest in the zone, past the band: the plain hold drives
        assert!(!rest(&mut t, 2000, 2 * B, 100));
        lose(&mut t, 2000, 1);
        assert_eq!(t.lost(), 3);
        assert!(!rest(&mut t, 2000, 2 * B, REST - 1));
        assert!(rest(&mut t, 2000, 2 * B, 1));
        assert!(t.at_rest());
    }

    #[test]
    fn a_thrown_hold_parks_after_the_rest_and_inside_the_zone_only() {
        let mut t = Thrown::new();
        lose(&mut t, 2000, HOLD_THROWN);
        assert!(!rest(&mut t, 2000, 2 * B, REST - 1));
        assert!(rest(&mut t, 2000, 2 * B, 1));
        // a sample past the still counts restarts the rest
        let mut t = Thrown::new();
        lose(&mut t, 2000, HOLD_THROWN);
        assert!(!rest(&mut t, 2000, 2 * B, REST - 1));
        assert!(!rest(&mut t, 2003, 2 * B, REST - 1));
        assert!(rest(&mut t, 2003, 2 * B, 1));
        let mut t = Thrown::new();
        lose(&mut t, 2000, HOLD_THROWN);
        assert!(!rest(&mut t, 2100, 2 * B + 1, 100));
        assert_eq!(t.lost(), HOLD_THROWN);
    }

    #[test]
    fn the_reach_grows_a_band_every_second_loss_to_five() {
        for lost in HOLD_THROWN..=40 {
            let bands = (2 + (lost - HOLD_THROWN) as u32 / 2).min(5);
            let mut t = Thrown::new();
            lose(&mut t, 2000, lost);
            assert!(!rest(&mut t, 2000, bands * B + 1, 50), "lost {lost}");
            assert_eq!(t.lost(), lost);
            assert!(rest(&mut t, 2000, bands * B, 1), "lost {lost}");
        }
        // the count saturates and the reach stays at the cap
        let mut t = Thrown::new();
        lose(&mut t, 2000, 255);
        lose(&mut t, 2000, 3);
        assert_eq!(t.lost(), 255);
        assert!(!rest(&mut t, 2000, 5 * B + 1, 50));
        assert!(rest(&mut t, 2000, 5 * B, 1));
    }

    #[test]
    fn the_park_is_kept_a_band_past_its_reach_and_once_lost_clears() {
        for (lost, reach) in [(3, 2), (5, 3), (9, 5)] {
            let mut t = Thrown::new();
            lose(&mut t, 2000, lost);
            assert!(rest(&mut t, 2000, reach * B, REST));
            // moving or not, the park holds to the kept band, lost or not
            for (raw, lost) in [(2010, true), (1990, false)] {
                assert!(t.parks(raw, (reach + 1) * B, B, lost, false, 0, FRIC));
            }
            assert!(!t.parks(2000, (reach + 1) * B + 1, B, true, false, 0, FRIC));
            assert_eq!((t.lost(), t.at_rest()), (0, false), "lost {lost}");
            // the hold is the plain one again: never parked twice
            assert!(!rest(&mut t, 2000, B + 1, 100));
            lose(&mut t, 2000, 2);
            assert!(!rest(&mut t, 2000, B + 1, 100));
        }
    }

    #[test]
    fn a_thrown_hold_parks_only_inside_the_friction_level() {
        // thrown, at rest in the zone, a count over the level: the plain
        // hold's loaded shaft, never parked, and the count stands
        let mut t = Thrown::new();
        lose(&mut t, 2000, HOLD_THROWN);
        for n in 0..1000 {
            assert!(
                !t.parks(2000, 2 * B, B, false, false, FRIC + 1, FRIC),
                "period {n}"
            );
        }
        assert_eq!((t.lost(), t.at_rest()), (HOLD_THROWN, false));
        // the estimate back inside the level: the rest already stands
        assert!(t.parks(2000, 2 * B, B, false, false, FRIC, FRIC));
        // no friction level set: only a zero estimate is a free shaft
        let mut t = Thrown::new();
        lose(&mut t, 2000, HOLD_THROWN);
        for _ in 0..REST {
            assert!(!t.parks(2000, B, B, false, false, 1, 0));
        }
        assert!(t.parks(2000, B, B, false, false, 0, 0));
    }

    #[test]
    fn the_park_is_kept_inside_twice_the_friction_level_and_a_load_ends_it() {
        let mut t = Thrown::new();
        lose(&mut t, 2000, HOLD_THROWN);
        assert!(rest(&mut t, 2000, 2 * B, REST));
        // the estimate's noise past the level it was taken under
        for tau in [FRIC + 1, 2 * FRIC, 2 * FRIC + 1] {
            assert!(t.parks(2000, 2 * B, B, false, false, tau, FRIC), "{tau}");
        }
        assert!(!t.parks(2000, 2 * B, B, false, false, 2 * FRIC + 2, FRIC));
        assert_eq!((t.lost(), t.at_rest()), (0, false));
        // the largest estimate and the largest level wrap nothing
        let mut t = Thrown::parked_at(2000, HOLD_THROWN);
        assert!(!t.parks(2000, 0, B, false, false, u32::MAX, 43 << 16));
        let mut t = Thrown::parked_at(2000, HOLD_THROWN);
        assert!(t.parks(2000, 0, B, false, false, u32::MAX, u32::MAX >> 1));
    }

    #[test]
    fn a_loss_after_the_dwell_restarts_the_count() {
        let mut t = Thrown::new();
        lose(&mut t, 2000, 2);
        assert!(!t.parks(2000, 2 * B, B, false, true, 0, FRIC));
        assert!(!t.parks(2000, 3 * B, B, true, true, 0, FRIC));
        assert_eq!(t.lost(), 0);
        lose(&mut t, 2000, 2);
        assert!(!rest(&mut t, 2000, B + 1, 100));
    }

    #[test]
    fn a_creeping_shaft_with_pot_noise_is_never_at_rest() {
        let mut t = Thrown::new();
        lose(&mut t, 2000, 20);
        // a count every second period under 2 counts of noise
        for n in 0..2_000u16 {
            let raw = 1000 + n / 2 + 2 * (n & 1);
            assert!(!t.parks(raw, B, B, false, false, 0, FRIC), "period {n}");
        }
        // the same noise on a still shaft is a rest
        let mut parks = false;
        for n in 0..REST as u16 {
            parks = t.parks(3000 + 2 * (n & 1), B, B, false, false, 0, FRIC);
        }
        assert!(parks);
    }

    #[test]
    fn clear_drops_the_count_the_rest_and_the_park() {
        let mut t = Thrown::parked_at(2000, 9);
        t.clear();
        assert_eq!((t.lost(), t.at_rest()), (0, false));
        assert!(!rest(&mut t, 2000, B, 100));
    }
}
