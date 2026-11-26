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
//! error stays within twice the band of a resting goal: a shaft that has
//! not left that zone for the dwell is at rest there and parks there. A free
//! one is then kept to three bands (keep): the dwell can settle a park on
//! the zone's edge, where the estimate's noise would release it into a push
//! on a still shaft. Before the dwell the band is the gate both ways, so a
//! moving shaft is caught and a coast through is redriven as before. The
//! hold is judged from the period after the profile lands.
//!
//! Where the play between the motor and the position sensor is wider than
//! the zone, a free output rests at either end of it, outside the zone, and
//! every push throws it to the other end: the dwell never runs. A throw is a
//! park left on the far side of the goal from the side the loop last drove
//! from, counted once, in the period the park is left; the first is a coast
//! through, the second (`HOLD_THROWS`) settles the hold where it is, kept to
//! five bands (loose). A near-side exit and a swing past loose clear the
//! count. Both reaches hold an unloaded shaft only, and cost accuracy: a hold
//! the throws settle rests up to five bands off the goal. With no velocity
//! integral the loop cannot throw the shaft, and a hold resting outside the
//! zone does not park.
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
//! Hold gates on POSITION rest (profile at rest on the goal, error inside
//! the reach) and the load estimate - never on omega_hat. The fused omega is
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

/// Parks left on the far side of the goal, running, that settle a hold
/// where it is. The first is a coast through the band, which the loop
/// redrives; the second is the way back, a shaft thrown from one end of
/// the play to the other.
pub const HOLD_THROWS: u8 = 2;

/// omega_ref for the velocity loop, the anti-hunt flags, whether the error
/// is within the hold's reach of a goal the profile rests on (the kernel
/// counts it toward `HOLD_SETTLE_TICKS`) and the hold's record of the play.
/// Inside the band, or once settled or thrown the reach: `park` for an
/// unloaded shaft (the kernel maps it to Brake), `hold` for a loaded one (it
/// keeps its current, its velocity integral frozen). At most one of the two
/// is set.
#[derive(Copy, Clone)]
pub struct PosOut {
    pub omega_ref_q16: i32,
    pub hold: bool,
    pub park: bool,
    pub near: bool,
    pub play: Play,
}

/// The hold's record of the play: the side of the goal the estimate was on
/// in the last period the loop did not start parked (`above`: past the
/// goal upward), and the parks left on the far side of it since, up to
/// `HOLD_THROWS`.
#[derive(Copy, Clone, Default, PartialEq, Eq, Debug)]
pub struct Play {
    pub above: bool,
    pub throws: u8,
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

/// One MEDIUM-tick update. theta*/omega* come from the trajectory
/// generator, theta_hat from fusion. `rested`: the profile was at rest on
/// the goal when the period began (`TrajGen::step_position`); the period it
/// lands is judged as moving. `band_gain_q4` is linearized counts per raw
/// count at the present position, Q4 (16 at the identity), clamped
/// (`pos_lut::band_gain_q4`). `settled`: `near` has held for
/// `HOLD_SETTLE_TICKS`. `unloaded`: [`unloaded`] on this period's estimate.
/// `parked`: the last period parked. `play`: the last period's record.
#[allow(clippy::too_many_arguments)]
pub fn step(
    rested: bool,
    theta_star_q16: i32,
    omega_star_q16: i32,
    theta_hat_q16: i32,
    band_gain_q4: u16,
    settled: bool,
    unloaded: bool,
    parked: bool,
    play: Play,
    cfg: &ConfigLoopPosition,
) -> PosOut {
    let e_raw = sat_sub(theta_star_q16, theta_hat_q16);
    let e = sat_sym(e_raw, E_LIM_CQ16);
    // the speed ceiling, not the move speed: a shaft behind its profile is
    // asked for more, up to the ceiling. The table rule bounds it; the min
    // keeps the shift below from turning v_lim negative on a corrupt one
    let v_lim = (cfg.speed_ceiling_cps.min(VEL_CAP_CPS) as i32) << 16;
    let mut play = play;
    let v = judge_hold(
        rested && cfg.pos_deadband_counts != 0,
        e_raw,
        band_gain_q4,
        settled,
        unloaded,
        parked,
        &mut play,
        cfg,
    );
    let hit = v & HIT != 0;
    PosOut {
        // Q8.8 * cQ16 >> 8 -> csQ16, i32-exact per E_LIM
        omega_ref_q16: sat_sym(
            sat_add(q_mul(cfg.p_kp_q88 as i32, e, 8), omega_star_q16),
            v_lim,
        ),
        hold: hit && !unloaded,
        park: hit && unloaded,
        near: v & NEAR != 0,
        play,
    }
}

/// `judge_hold`'s verdict bits: within the reach (`PosOut::near`), and
/// inside the hold, a park or a hold by its load. A hit is near.
const NEAR: u8 = 1;
const HIT: u8 = 2;

/// `step`'s hold on `e_raw`, `play` updated. One byte out: a struct of
/// flags is carried through the early returns as a packed word, shifted
/// apart again at the merge. `judged`: the profile rests on the goal and
/// the deadband is not 0.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn judge_hold(
    judged: bool,
    e_raw: i32,
    band_gain_q4: u16,
    settled: bool,
    unloaded: bool,
    parked: bool,
    play: &mut Play,
    cfg: &ConfigLoopPosition,
) -> u8 {
    // a moving profile skips the band's multiply
    if !judged {
        *play = Play::default();
        return 0;
    }
    let band = band_q16(cfg.pos_deadband_counts, band_gain_q4);
    let e_abs = e_raw.unsigned_abs();
    let above = e_raw < 0;
    if !parked {
        play.above = above;
    }
    // the band sits inside every reach
    if e_abs <= band {
        return NEAR | HIT;
    }
    // past the band, which is then under 2^31: each reach compares as the
    // excess over the band against a multiple of it, exact with no
    // saturation
    let over = e_abs - band;
    // past loose, 5 bands: a swing that wide is not a throw across the play
    if (over + 3) >> 2 > band {
        play.throws = 0;
        return 0;
    }
    // thrown: kept to loose, inside it here
    if unloaded && play.throws >= HOLD_THROWS {
        return NEAR | HIT;
    }
    // settled: kept to keep, 3 bands, or under a load the zone, 2. Before
    // the dwell the zone only counts it
    let near = if settled {
        if over <= band << unloaded as u32 {
            return NEAR | HIT;
        }
        false
    } else {
        over <= band
    };
    // a throw counts once, in the period the park is left: counted on
    // every parked period past the band, a load pushing a settled park out
    // of it reads as thrown and sags to loose
    // (`a_load_under_the_limit_sags_and_is_pulled_back`). At HOLD_THROWS
    // the count stays
    if parked && play.throws < HOLD_THROWS {
        if above == play.above {
            play.throws = 0;
        } else {
            play.throws += 1;
            // the second throw settles the hold on the spot
            if unloaded && play.throws >= HOLD_THROWS {
                return NEAR | HIT;
            }
        }
    }
    near as u8
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
            omega_star_q16 == 0,
            theta_star_q16,
            omega_star_q16,
            theta_hat_q16,
            band_gain_q4,
            settled,
            true,
            false,
            Play::default(),
            cfg,
        )
    }

    /// The step on a resting profile at the goal 0, the estimate at `-e`.
    fn judged(
        e: i32,
        settled: bool,
        unloaded: bool,
        parked: bool,
        play: Play,
        cfg: &ConfigLoopPosition,
    ) -> PosOut {
        super::step(true, e, 0, 0, ID, settled, unloaded, parked, play, cfg)
    }

    /// A record the play has thrown: the last drive from below the goal.
    const THROWN: Play = Play {
        above: false,
        throws: HOLD_THROWS,
    };

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
        // gain 26/16: 12 raw counts span 19.5 linearized; a settled free
        // shaft is kept to three bands, 58.5
        let cfg = c(1 << 8, 12, 32767);
        for (settled, gate) in [(false, 39 << 15), (true, 117 << 15)] {
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
    fn near_is_two_bands_of_a_resting_goal_and_three_once_a_free_hold_settles() {
        let cfg = c(1 << 8, 12, 32767);
        for (settled, unloaded, reach) in [
            (false, true, 24),
            (false, false, 24),
            (true, false, 24),
            (true, true, 36),
        ] {
            let near = |e: i32| judged(e, settled, unloaded, false, Play::default(), &cfg).near;
            assert!(near(reach << 16), "settled {settled}, unloaded {unloaded}");
            assert!(near(-(reach << 16)));
            assert!(!near((reach << 16) + 1));
            assert!(!near(-(reach << 16) - 1));
        }
        for settled in [false, true] {
            // a moving profile is never near, even at zero error
            assert!(!step(0, 1, 0, ID, settled, &cfg).near);
            assert!(!step(0, -1, 0, ID, settled, &cfg).near);
            // nor is the period it lands, at rest on the goal
            let out = super::step(false, 0, 0, 0, ID, settled, true, false, THROWN, &cfg);
            assert!(!out.near && !out.park && !out.hold);
            // deadband 0 never counts
            assert!(!step(0, 0, 0, ID, settled, &c(1 << 8, 0, 32767)).near);
        }
        // the ceiling saturates the zone past any error
        let cfg = c(1 << 8, 65535, 32767);
        assert!(step(i32::MAX, 0, i32::MIN, 64, false, &cfg).near);
        assert!(step(i32::MIN, 0, i32::MAX, 64, false, &cfg).near);
    }

    #[test]
    fn a_settled_park_releases_past_three_bands() {
        let cfg = c(1 << 8, 12, 32767);
        // entry and an un-settled park: the band, not past it
        assert!(step(12 << 16, 0, 0, ID, false, &cfg).park);
        assert!(step(0, 0, 12 << 16, ID, false, &cfg).park);
        assert!(!step((12 << 16) + 1, 0, 0, ID, false, &cfg).park);
        assert!(!step(18 << 16, 0, 0, ID, false, &cfg).park);
        // settled: three bands, not past them
        assert!(step(30 << 16, 0, 0, ID, true, &cfg).park);
        assert!(step(0, 0, 36 << 16, ID, true, &cfg).park);
        assert!(!step((36 << 16) + 1, 0, 0, ID, true, &cfg).park);
        assert!(!step(0, 0, (36 << 16) + 1, ID, true, &cfg).park);
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
            judged(e, settled, unloaded, false, Play::default(), &cfg)
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
        // profile, neither: a settled free shaft is kept to three bands, a
        // loaded one still released past two
        for (e, settled) in [
            ((12 << 16) + 1, false),
            (25 << 16, true),
            (-(36 << 16), true),
        ] {
            let out = step(e, settled, false);
            assert!(!out.hold && !out.park, "e {e}");
        }
        assert!(step(25 << 16, true, true).park);
        assert!(!step((36 << 16) + 1, true, true).park);
        for unloaded in [false, true] {
            let out = super::step(
                false,
                0,
                1,
                0,
                ID,
                true,
                unloaded,
                false,
                Play::default(),
                &cfg,
            );
            assert!(!out.hold && !out.park, "unloaded {unloaded}");
        }
    }

    #[test]
    fn a_throw_is_a_park_left_on_the_far_side() {
        let cfg = c(1 << 8, 12, 32767);
        let from_below = |throws| Play {
            above: false,
            throws,
        };
        // driven last from below: a park left above the goal counts in the
        // period it is left, up to HOLD_THROWS, and the second settles the
        // hold where it is
        let first = judged(-(13 << 16), false, true, true, from_below(0), &cfg);
        assert_eq!(first.play, from_below(1));
        assert!(!first.park);
        let second = judged(-(13 << 16), false, true, true, from_below(1), &cfg);
        assert_eq!(second.play, from_below(HOLD_THROWS));
        assert!(second.park);
        let more = judged(-(60 << 16), false, true, true, THROWN, &cfg);
        assert_eq!(more.play, THROWN);
        // inside the band, or driving, nothing counts; a drive moves the
        // side to where the estimate is
        for (e, parked, play) in [
            (-(12 << 16), true, from_below(1)),
            (-(13 << 16), false, from_below(1)),
        ] {
            let out = judged(e, false, true, parked, play, &cfg);
            let side = if parked { play.above } else { true };
            assert_eq!(
                out.play,
                Play {
                    above: side,
                    throws: 1
                },
                "e {e}"
            );
        }
    }

    #[test]
    fn a_settled_park_held_past_the_band_counts_no_throw_until_it_is_left() {
        // settled and parked two bands above the goal, on the far side of
        // the last drive: kept, period after period, with nothing counted
        let cfg = c(1 << 8, 12, 32767);
        let mut play = Play::default();
        for _ in 0..1000 {
            let out = judged(-(24 << 16), true, true, true, play, &cfg);
            assert!(out.park && out.near);
            play = out.play;
        }
        assert_eq!(play, Play::default());
        // a load there is held, and the park that ends counts nothing
        let out = judged(-(24 << 16), true, false, true, play, &cfg);
        assert!(out.hold && !out.park);
        // left past the keep: one throw, in that period
        let out = judged(-(37 << 16), true, true, true, play, &cfg);
        assert!(!out.park && !out.near);
        assert_eq!(out.play.throws, 1);
    }

    #[test]
    fn a_near_side_exit_clears_the_throws() {
        let cfg = c(1 << 8, 12, 32767);
        // driven last from below, the park left below the goal again
        for e in [13 << 16, 60 << 16] {
            let out = judged(
                e,
                false,
                true,
                true,
                Play {
                    above: false,
                    throws: 1,
                },
                &cfg,
            );
            assert_eq!(out.play, Play::default(), "e {e}");
            assert!(!out.park);
        }
        // a settled hold stays settled on the spot
        assert_eq!(
            judged(13 << 16, false, true, true, THROWN, &cfg).play,
            THROWN
        );
        assert!(judged(60 << 16, false, true, true, THROWN, &cfg).park);
    }

    #[test]
    fn a_swing_past_loose_clears_the_throws() {
        let cfg = c(1 << 8, 12, 32767);
        // loose is five bands, 60: past it, parked or driving, either side
        for e in [(60 << 16) + 1, -(60 << 16) - 1, 200 << 16, i32::MIN + 1] {
            for parked in [false, true] {
                let out = judged(e, true, true, parked, THROWN, &cfg);
                assert_eq!(out.play.throws, 0, "e {e}, parked {parked}");
                assert!(!out.park && !out.hold && !out.near);
            }
        }
        // a rattle that swings past loose between two far-side exits is
        // not a throw across the play: the second exit counts one, again
        let play = judged(-(20 << 16), false, true, true, Play::default(), &cfg).play;
        assert_eq!(play.throws, 1);
        let play = judged(-(80 << 16), false, true, false, play, &cfg).play;
        let play = judged(40 << 16, false, true, false, play, &cfg).play;
        assert_eq!(play, Play::default());
        let play = judged(5 << 16, false, true, false, play, &cfg).play;
        let out = judged(-(20 << 16), false, true, true, play, &cfg);
        assert_eq!(out.play.throws, 1);
        assert!(!out.park);
    }

    #[test]
    fn a_thrown_park_is_kept_to_five_bands_of_the_local_gain() {
        // loose is five bands, settled or not: 60 at the identity, 97.5 at
        // gain 26/16
        let cfg = c(1 << 8, 12, 32767);
        for (gain, gate) in [(ID, 60 << 16), (26, 195 << 15)] {
            for settled in [false, true] {
                let thrown =
                    |e: i32| super::step(true, e, 0, 0, gain, settled, true, true, THROWN, &cfg);
                assert!(thrown(gate).park && thrown(gate).near, "gain {gain}");
                assert!(thrown(-gate).park);
                assert!(!thrown(gate + 1).park && !thrown(gate + 1).near);
                assert!(!thrown(-gate - 1).park);
            }
        }
        // the keep scales with the gain as the band does: 58.5 at 26/16
        let keep = |e: i32| super::step(true, e, 0, 0, 26, true, true, true, Play::default(), &cfg);
        assert!(keep(117 << 15).park && !keep((117 << 15) + 1).park);
    }

    #[test]
    fn a_load_on_a_thrown_park_drives_or_holds() {
        // a load, the record thrown: never parked, held only where a loaded
        // shaft is, the band and once settled the zone
        let cfg = c(1 << 8, 12, 32767);
        for settled in [false, true] {
            for raw in -60..=60 {
                let e = raw << 16;
                let out = judged(e, settled, false, true, THROWN, &cfg);
                let held = raw.abs() <= 12 || (settled && raw.abs() <= 24);
                assert!(!out.park, "e {raw}, settled {settled}");
                assert_eq!(out.hold, held, "e {raw}, settled {settled}");
            }
        }
    }

    #[test]
    fn a_moving_profile_and_deadband_0_clear_the_record() {
        let cfg = c(1 << 8, 12, 32767);
        for parked in [false, true] {
            let moving = super::step(false, 30 << 16, 0, 0, ID, true, true, parked, THROWN, &cfg);
            assert_eq!(moving.play, Play::default());
            let off = judged(30 << 16, true, true, parked, THROWN, &c(1 << 8, 0, 32767));
            assert_eq!(off.play, Play::default());
            assert!(!off.park && !off.near);
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
