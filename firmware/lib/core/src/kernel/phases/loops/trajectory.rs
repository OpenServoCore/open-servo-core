//! Trapezoid trajectory generator (control-theory "The Trajectory
//! Generator"): shapes goal steps into accel/vel-limited profiles and emits
//! theta*/omega*/alpha* - the references plus the feedforward pair. alpha*
//! is the ACTUALLY-APPLIED delta-omega each tick, so the velocity loop's J
//! term follows what the profile really did, clamps and terminal snap
//! included. The decel decision is a discrete per-tick test done
//! multiply-only (wide_ge cross-compare, no divide, no v^2/2a quotient).

use crate::math::{q_mul, wide_ge};
use crate::regions::config::{ConfigLoopPosition, ConfigPosSoft};

/// theta_ref guard, matching fusion's THETA_LIM: pot tops 4095<<16 < 2^28
/// and goals clamp to soft limits, so 2^29 only guards i32 arithmetic
/// (goal - theta_ref stays exact).
const THETA_LIM_CQ16: i32 = 1 << 29;

/// Whole-count goal bound implied by THETA_LIM; the soft limits' table
/// rule.
pub const GOAL_LIM_COUNTS: i32 = THETA_LIM_CQ16 >> 16;

/// omega_ref cap in c/s: i16 full scale (fusion OMEGA_LIM convention); also
/// keeps v_up + a inside u32 for the decel compare. The table rule on
/// `velocity_limit_cps`.
pub const VEL_CAP_CPS: u16 = 32767;

/// Terminal snap floor: below one accel step of omega and within one whole
/// count, land exactly instead of creeping forever. Assumes one accel step
/// travels under a count per step - true for Q8.8 accel (<= 256 c/s per
/// tick) at the 2 kHz MEDIUM rate.
const SNAP_D_CQ16: u32 = 1 << 16;

/// Velocity-mode wall ramp slope: 2^4 = 16 c/s of inward allowance per
/// count of distance to a soft limit. Matches the decel the default limits
/// can deliver (accel/vel = 30000/1500 = 20 /s); a hotter config just lags
/// the ramp through the accel slew.
const WALL_RAMP_SHIFT: u32 = 4;

/// The profile reads CONFIG `loop_position` and `pos_soft` as stored: the
/// table rules hold the velocity limit under [`VEL_CAP_CPS`] and the soft
/// limits ordered inside [`GOAL_LIM_COUNTS`]. The cap and the goal bound
/// apply again at the use, constant compares, so a corrupt table stays
/// bounded and panic-free.
fn vel_cap(lp: &ConfigLoopPosition) -> i32 {
    lp.velocity_limit_cps.min(VEL_CAP_CPS) as i32
}

/// State: theta_ref cQ16, omega_ref csQ16, alpha_last csQ16 per tick (the
/// applied delta, emitted as alpha*).
#[derive(Default)]
pub struct TrajGen {
    theta_ref_q16: i32,
    omega_ref_q16: i32,
    alpha_last_q16: i32,
}

impl TrajGen {
    pub const fn new() -> Self {
        Self {
            theta_ref_q16: 0,
            omega_ref_q16: 0,
            alpha_last_q16: 0,
        }
    }

    /// Enable edge / mode change: reference = estimate, at rest - bumpless.
    pub fn reseed(&mut self, theta_hat_q16: i32) {
        self.theta_ref_q16 = theta_hat_q16.clamp(-THETA_LIM_CQ16, THETA_LIM_CQ16);
        self.omega_ref_q16 = 0;
        self.alpha_last_q16 = 0;
    }

    /// One MEDIUM step over `dt_q32` toward `goal_counts` clamped to the soft
    /// limits.
    ///
    /// Decel decision: a decel chain shedding a per tick from v travels
    /// dt*sum(v - k*a) = (v^2 - a*v)*dt/(2a) - exact when a divides v, else
    /// the closed form undershoots by < a*dt/8, sub-count for real configs.
    /// Cruising one more tick at v_up then stopping therefore needs
    ///   v_up*dt + (v_up^2 - a*v_up)*dt/(2a) = v_up*(v_up + a)*dt/(2a) <= |d|
    /// Multiply-only: with vt = q_mul(v_up, dt, 32) (per-tick travel, cQ16,
    /// the same floored product the integrator applies), decelerate when
    /// (v_up + a) * vt >= 2a * |d|. Scales: (v_up + a) csQ16 x vt cQ16 vs
    /// 2a csQ16-per-tick x |d| cQ16 - both u64 products carry physical
    /// value x 2^32, since vt already absorbed dt (c/s x s = c). Choosing
    /// v_up by this test every tick is inductively overshoot-free: a safe
    /// v_up leaves the whole decel tail inside |d| - vt, and each decel tick
    /// preserves the remaining-travel identity D(v) = v_next*dt + D(v_next).
    pub fn step_position(
        &mut self,
        goal_counts: i32,
        dt_q32: u32,
        lp: &ConfigLoopPosition,
        soft: &ConfigPosSoft,
    ) {
        let a = (lp.accel_limit_q88 as i32) << 8;
        let v_lim = vel_cap(lp) << 16;
        // max().min(): the order is the table's, clamp would keep a panic
        // path
        let goal = goal_counts
            .max(soft.pos_min_soft_counts)
            .min(soft.pos_max_soft_counts)
            .clamp(-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS)
            << 16;
        // both operands within +-2^29 -> exact
        let d = goal - self.theta_ref_q16;
        let prev = self.omega_ref_q16;
        // landed and at rest: the full step lands again with alpha 0
        if d == 0 && prev == 0 {
            self.alpha_last_q16 = 0;
            return;
        }
        let d_abs = d.unsigned_abs();
        let dir_pos = d >= 0;
        // magnitudes along the goal direction; v < 0 means moving away
        let v = if dir_pos { prev } else { -prev };
        let v_next = if v < 0 {
            // moving away: slew toward the goal through zero; the decel test
            // is meaningless until the sign flips
            (v + a).min(v_lim)
        } else {
            let v_up = v + (v_lim - v).clamp(-a, a);
            let vt = q_mul(v_up, dt_q32 as i32, 32) as u32;
            if wide_ge(v_up as u32 + a as u32, vt, 2 * (a as u32), d_abs) {
                (v - a).max(0)
            } else {
                v_up
            }
        };
        let omega = if dir_pos { v_next } else { -v_next };
        let step_mag = q_mul(if omega < 0 { -omega } else { omega }, dt_q32 as i32, 32);
        let toward = if omega > 0 {
            d >= 0
        } else if omega < 0 {
            d <= 0
        } else {
            true
        };
        let land = (toward && step_mag as u32 >= d_abs)
            || (omega.unsigned_abs() <= a as u32 && d_abs < SNAP_D_CQ16);
        if land {
            self.theta_ref_q16 = goal;
            self.omega_ref_q16 = 0;
            self.alpha_last_q16 = prev.saturating_neg();
        } else {
            let travel = if omega >= 0 { step_mag } else { -step_mag };
            // |theta| <= 2^29 and |travel| <= 2^30 -> the sum is exact
            self.theta_ref_q16 =
                (self.theta_ref_q16 + travel).clamp(-THETA_LIM_CQ16, THETA_LIM_CQ16);
            self.omega_ref_q16 = omega;
            self.alpha_last_q16 = omega.saturating_sub(prev);
        }
    }

    /// Velocity-mode entry: slew omega_ref toward the clamped goal by +-a
    /// per tick. theta_ref holds: nothing reads it in this mode, and every
    /// entry into position mode reseeds the profile.
    ///
    /// Soft-limit wall ramp: the goal's inward component is capped at
    /// WALL_RAMP c/s per count of distance to the wall, reaching 0 at the
    /// wall (and staying 0 past it - momentum only ever coasts against the
    /// endstop's current cut, quadratic in speed on the bench). A linear
    /// ramp converges without the limit cycle a binary at-the-wall cut
    /// produces: the cut flips at one count and the velocity PI's brake
    /// bounce re-arms it every swing. Retreat is never capped. The accel
    /// slew below stays the physical enforcer when a config's accel/vel
    /// ratio disagrees with the ramp slope - the ramp is a reference shape.
    pub fn step_velocity(
        &mut self,
        goal_velocity_cps: i32,
        theta_hat_q16: i32,
        lp: &ConfigLoopPosition,
        soft: &ConfigPosSoft,
    ) {
        let a = (lp.accel_limit_q88 as i32) << 8;
        let v_cap = vel_cap(lp);
        let th = theta_hat_q16 >> 16;
        // distance pre-clamped so the shift cannot overflow
        let cap = (VEL_CAP_CPS as i32) >> WALL_RAMP_SHIFT;
        let goal = goal_velocity_cps.clamp(-v_cap, v_cap);
        // only the wall the goal points at caps it: both allowances are >= 0
        let goal = if goal >= 0 {
            goal.min(soft.pos_max_soft_counts.saturating_sub(th).clamp(0, cap) << WALL_RAMP_SHIFT)
        } else {
            goal.max(
                -(th.saturating_sub(soft.pos_min_soft_counts).clamp(0, cap) << WALL_RAMP_SHIFT),
            )
        } << 16;
        let prev = self.omega_ref_q16;
        // goal - prev can span 2^32; saturation past +-a is clamped away
        let omega = prev + goal.saturating_sub(prev).clamp(-a, a);
        self.omega_ref_q16 = omega;
        self.alpha_last_q16 = omega - prev;
    }

    pub fn theta_star_q16(&self) -> i32 {
        self.theta_ref_q16
    }

    pub fn omega_star_q16(&self) -> i32 {
        self.omega_ref_q16
    }

    pub fn alpha_star_q16(&self) -> i32 {
        self.alpha_last_q16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 2 kHz MEDIUM rate, matching the kernel's DT_MED_Q32 derivation.
    const DT: u32 = ((1u64 << 32) / 2000) as u32;
    // 50 c/s per tick in csQ16 (= 100000 c/s^2 continuous at 2 kHz)
    const A: i32 = 50 << 16;

    /// The two CONFIG blocks the profile reads.
    struct Cfg {
        lp: ConfigLoopPosition,
        soft: ConfigPosSoft,
    }

    fn cfg_with(vel: u16, accel_q88: u16, min: i32, max: i32) -> Cfg {
        Cfg {
            lp: ConfigLoopPosition {
                velocity_limit_cps: vel,
                accel_limit_q88: accel_q88,
                ..ConfigLoopPosition::new()
            },
            soft: ConfigPosSoft {
                pos_min_soft_counts: min,
                pos_max_soft_counts: max,
            },
        }
    }

    fn cfg(vel: u16, accel_q88: u16) -> Cfg {
        cfg_with(vel, accel_q88, -100_000, 100_000)
    }

    fn step_pos(t: &mut TrajGen, goal: i32, dt: u32, c: &Cfg) {
        t.step_position(goal, dt, &c.lp, &c.soft);
    }

    fn step_vel(t: &mut TrajGen, goal: i32, theta_hat_q16: i32, c: &Cfg) {
        t.step_velocity(goal, theta_hat_q16, &c.lp, &c.soft);
    }

    fn run_to_land(t: &mut TrajGen, goal: i32, target_q16: i32, c: &Cfg, max: u32) -> u32 {
        for k in 1..=max {
            step_pos(t, goal, DT, c);
            if t.omega_star_q16() == 0 && t.theta_star_q16() == target_q16 {
                return k;
            }
        }
        panic!(
            "no landing: theta={} omega={}",
            t.theta_star_q16(),
            t.omega_star_q16()
        );
    }

    #[test]
    fn reseed_bumpless() {
        let c = cfg(1000, 50 << 8);
        let mut t = TrajGen::new();
        for _ in 0..30 {
            step_pos(&mut t, 500, DT, &c);
        }
        assert_ne!(t.omega_star_q16(), 0);
        t.reseed(123 << 16);
        assert_eq!(t.theta_star_q16(), 123 << 16);
        assert_eq!(t.omega_star_q16(), 0);
        assert_eq!(t.alpha_star_q16(), 0);
        // extreme estimate clamps to the arithmetic guard
        t.reseed(i32::MAX);
        assert_eq!(t.theta_star_q16(), THETA_LIM_CQ16);
    }

    #[test]
    fn full_profile_plateau_and_exact_landing() {
        // 2000-count step at vel 1000 c/s, accel 50 c/s/tick: 20-tick ramp,
        // long plateau, decel, creep, exact landing - and theta_ref is
        // monotone with no overshoot at EVERY tick.
        let c = cfg(1000, 50 << 8);
        let goal_q = 2000 << 16;
        let mut t = TrajGen::new();
        t.reseed(0);
        let mut peak = 0i32;
        let mut plateau = 0u32;
        let mut landed = 0u32;
        let mut prev_o = 0i32;
        let mut prev_th = 0i32;
        for k in 1..=4400u32 {
            step_pos(&mut t, 2000, DT, &c);
            let o = t.omega_star_q16();
            let th = t.theta_star_q16();
            // alpha* IS the applied delta-omega, snap ticks included
            assert_eq!(t.alpha_star_q16(), o - prev_o);
            // 2a only on the landing tick: the snap zeroes the creep value a
            // plus at most one accel step
            assert!((o - prev_o).abs() <= 2 * A, "tick {k}: alpha past accel");
            assert!((0..=1000 << 16).contains(&o), "tick {k}: omega={o}");
            assert!(th <= goal_q, "tick {k}: overshoot theta={th}");
            assert!(th >= prev_th, "tick {k}: non-monotone theta");
            peak = peak.max(o);
            if o == 1000 << 16 {
                plateau += 1;
            }
            prev_o = o;
            prev_th = th;
            if o == 0 && th == goal_q {
                landed = k;
                break;
            }
        }
        assert!(landed > 0, "never landed");
        assert_eq!(peak, 1000 << 16, "plateau below vel_limit");
        assert!(plateau > 3500, "plateau={plateau}");
        // landed state is a fixed point
        for _ in 0..5 {
            step_pos(&mut t, 2000, DT, &c);
            assert_eq!(t.theta_star_q16(), goal_q);
            assert_eq!(t.omega_star_q16(), 0);
            assert_eq!(t.alpha_star_q16(), 0);
        }
    }

    #[test]
    fn decel_distance_pinned() {
        // Hand example: v = 1000 c/s, a = 50 c/s per tick at 2 kHz.
        // Continuous stop distance s = v^2*dt/(2a) = 1000*1000*0.0005/100
        // = 5 counts; the discrete decision adds this tick's travel:
        // v*(v + a)*dt/(2a) = 5.25 counts.
        let c = cfg(1000, 50 << 8);
        // per-tick travel at 1000 c/s is half a count = 32768 cQ16, minus 1
        // from the dt_med_q32 floor (2147483 vs 2147483.648)
        let vt = q_mul(1000 << 16, DT as i32, 32);
        assert_eq!(vt, 32767);
        // exact decision boundary: decel iff (1050<<16)*vt >= (100<<16)*d,
        // i.e. d <= floor(10.5 * 32767) = 344053 cQ16; the ideal 5.25
        // counts = 344064 sits 11 cQ16 above, the vt floor times 10.5
        let omega_after = |d_q16: i32| {
            let mut t = TrajGen::new();
            t.theta_ref_q16 = -d_q16;
            t.omega_ref_q16 = 1000 << 16;
            step_pos(&mut t, 0, DT, &c);
            (t.omega_star_q16(), t.alpha_star_q16())
        };
        // 6 counts out: cruise holds, alpha 0
        assert_eq!(omega_after(6 << 16), (1000 << 16, 0));
        // 5 counts out (= the continuous s): inside the decision distance,
        // shed exactly one accel step
        assert_eq!(omega_after(5 << 16), (950 << 16, -A));
        // the boundary itself, exact to the cQ16
        assert_eq!(omega_after(344054).0, 1000 << 16);
        assert_eq!(omega_after(344053).0, 950 << 16);

        // ride the 5-count case to standstill: never past the goal, lands
        // exactly (decel travel 4.75 counts, creep covers the rest)
        let mut t = TrajGen::new();
        t.theta_ref_q16 = -(5 << 16);
        t.omega_ref_q16 = 1000 << 16;
        for _ in 0..100 {
            step_pos(&mut t, 0, DT, &c);
            assert!(t.theta_star_q16() <= 0, "overshoot {}", t.theta_star_q16());
            if t.omega_star_q16() == 0 && t.theta_star_q16() == 0 {
                return;
            }
        }
        panic!("no landing");
    }

    #[test]
    fn short_move_triangle() {
        // 4 counts: peak sqrt(2 * 100000 * 2) ~ 632 c/s, under the limit
        let c = cfg(1000, 50 << 8);
        let goal_q = 4 << 16;
        let mut t = TrajGen::new();
        t.reseed(0);
        let mut peak = 0i32;
        for _ in 0..200 {
            step_pos(&mut t, 4, DT, &c);
            peak = peak.max(t.omega_star_q16());
            assert!(t.theta_star_q16() <= goal_q);
            if t.omega_star_q16() == 0 && t.theta_star_q16() == goal_q {
                assert!(peak > 0 && peak < 1000 << 16, "peak={peak}");
                return;
            }
        }
        panic!("no landing");
    }

    /// The step as it ran before the stationary return, on the raw CONFIG
    /// fields.
    fn step_position_full(
        t: &mut TrajGen,
        goal_counts: i32,
        dt_q32: u32,
        raw: (u16, u16, i32, i32),
    ) {
        let (vel, accel, min, max) = raw;
        let a = (accel as i32) << 8;
        let v_lim = (vel.min(VEL_CAP_CPS) as i32) << 16;
        let goal = goal_counts
            .max(min)
            .min(max)
            .clamp(-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS)
            << 16;
        let d = goal - t.theta_ref_q16;
        let d_abs = d.unsigned_abs();
        let dir_pos = d >= 0;
        let prev = t.omega_ref_q16;
        let v = if dir_pos { prev } else { -prev };
        let v_next = if v < 0 {
            (v + a).min(v_lim)
        } else {
            let v_up = v + (v_lim - v).clamp(-a, a);
            let vt = q_mul(v_up, dt_q32 as i32, 32) as u32;
            if wide_ge(v_up as u32 + a as u32, vt, 2 * (a as u32), d_abs) {
                (v - a).max(0)
            } else {
                v_up
            }
        };
        let omega = if dir_pos { v_next } else { -v_next };
        let step_mag = q_mul(if omega < 0 { -omega } else { omega }, dt_q32 as i32, 32);
        let toward = if omega > 0 {
            d >= 0
        } else if omega < 0 {
            d <= 0
        } else {
            true
        };
        let land = (toward && step_mag as u32 >= d_abs)
            || (omega.unsigned_abs() <= a as u32 && d_abs < SNAP_D_CQ16);
        if land {
            t.theta_ref_q16 = goal;
            t.omega_ref_q16 = 0;
            t.alpha_last_q16 = prev.saturating_neg();
        } else {
            let travel = if omega >= 0 { step_mag } else { -step_mag };
            t.theta_ref_q16 = (t.theta_ref_q16 + travel).clamp(-THETA_LIM_CQ16, THETA_LIM_CQ16);
            t.omega_ref_q16 = omega;
            t.alpha_last_q16 = omega.saturating_sub(prev);
        }
    }

    #[test]
    fn stationary_return_matches_the_full_step() {
        let state = |t: &TrajGen| (t.theta_ref_q16, t.omega_ref_q16, t.alpha_last_q16);
        let mut seed = 1u32;
        let mut next = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            seed as i32
        };
        for (vel, accel) in [(0u16, 0u16), (1, 1), (1000, 50 << 8), (u16::MAX, u16::MAX)] {
            for (min, max) in [(-300, 500), (100, -100), (i32::MIN, i32::MAX)] {
                let raw = (vel, accel, min, max);
                let c = cfg_with(vel, accel, min, max);
                for dt in [0, DT, u32::MAX] {
                    // on the goal: at rest with alpha still the landing
                    // tick's, and passing through it
                    for goal in [i32::MIN, -301, 0, 77, 501, i32::MAX] {
                        let on_goal = goal
                            .max(min)
                            .min(max)
                            .clamp(-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS)
                            << 16;
                        for omega in [0, A, -A, 1000 << 16] {
                            let mut t = TrajGen {
                                theta_ref_q16: on_goal,
                                omega_ref_q16: omega,
                                alpha_last_q16: -A,
                            };
                            let mut r = TrajGen { ..t };
                            step_pos(&mut t, goal, dt, &c);
                            step_position_full(&mut r, goal, dt, raw);
                            assert_eq!(state(&t), state(&r), "goal {goal} omega {omega} dt {dt}");
                            if omega == 0 {
                                assert_eq!(state(&t), (on_goal, 0, 0));
                            }
                        }
                    }
                    // any reachable state: theta inside the guard, omega
                    // inside the i16 c/s cap
                    for _ in 0..200 {
                        let mut t = TrajGen {
                            theta_ref_q16: next() >> 2,
                            omega_ref_q16: (next() % 32768) << 16,
                            alpha_last_q16: next(),
                        };
                        let mut r = TrajGen { ..t };
                        let goal = next() >> (next() as u32 & 31);
                        step_pos(&mut t, goal, dt, &c);
                        step_position_full(&mut r, goal, dt, raw);
                        assert_eq!(state(&t), state(&r), "goal {goal} dt {dt}");
                    }
                }
            }
        }
    }

    #[test]
    fn goal_clamped_to_soft_limits() {
        let c = cfg_with(1000, 50 << 8, -300, 500);
        let mut t = TrajGen::new();
        t.reseed(0);
        run_to_land(&mut t, 100_000, 500 << 16, &c, 2000);
        run_to_land(&mut t, -100_000, -(300 << 16), &c, 4000);
    }

    #[test]
    fn velocity_mode_slews_and_clamps() {
        let c = cfg(1000, 50 << 8);
        let mut t = TrajGen::new();
        t.reseed(0);
        for k in 1..=10i32 {
            step_vel(&mut t, 500, 0, &c);
            assert_eq!(t.omega_star_q16(), (50 * k) << 16);
            assert_eq!(t.alpha_star_q16(), A);
        }
        step_vel(&mut t, 500, 0, &c);
        assert_eq!(t.omega_star_q16(), 500 << 16);
        assert_eq!(t.alpha_star_q16(), 0);
        // goal past vel_limit clamps at the limit
        for _ in 0..100 {
            step_vel(&mut t, 5000, 0, &c);
        }
        assert_eq!(t.omega_star_q16(), 1000 << 16);
        // reversal: exactly one accel step per tick through zero
        let mut prev = t.omega_star_q16();
        for _ in 0..40 {
            step_vel(&mut t, -5000, 0, &c);
            assert_eq!(prev - t.omega_star_q16(), A);
            prev = t.omega_star_q16();
        }
        assert_eq!(prev, -(1000 << 16));
    }

    #[test]
    fn velocity_wall_ramp_caps_inward_goal() {
        let c = cfg_with(1000, 50 << 8, 0, 1000);
        let run = |theta: i32, goal: i32| {
            let mut t = TrajGen::new();
            for _ in 0..100 {
                step_vel(&mut t, goal, theta << 16, &c);
            }
            t.omega_star_q16() >> 16
        };
        // far from both walls: the vel limit binds
        assert_eq!(run(500, 5000), 1000);
        // 50 counts out: 50 << WALL_RAMP_SHIFT binds
        assert_eq!(run(950, 5000), 800);
        // at and past the wall: inward goal pinned to 0
        assert_eq!(run(1000, 5000), 0);
        assert_eq!(run(1100, 5000), 0);
        // retreat is never capped, even from past the wall
        assert_eq!(run(1100, -700), -700);
        // mirrored at the min wall
        assert_eq!(run(25, -5000), -400);
        assert_eq!(run(0, -5000), 0);
        assert_eq!(run(500, -5000), -1000);
    }

    #[test]
    fn velocity_caps_by_the_wall_the_goal_points_at() {
        // the goal clamped by both wall allowances, each wall as stored
        let both = |goal: i32, th: i32, raw: (u16, i32, i32)| {
            let (vel, lo, hi) = raw;
            let v_cap = vel.min(VEL_CAP_CPS) as i32;
            let cap = (VEL_CAP_CPS as i32) >> WALL_RAMP_SHIFT;
            let up = hi.saturating_sub(th).clamp(0, cap) << WALL_RAMP_SHIFT;
            let dn = th.saturating_sub(lo).clamp(0, cap) << WALL_RAMP_SHIFT;
            goal.clamp(-v_cap, v_cap).clamp(-dn, up) << 16
        };
        // one step from rest slews by at most a
        let a = (u16::MAX as i32) << 8;
        for (vel, min, max) in [
            (1000u16, 0, 1000),
            (u16::MAX, 100, -100),
            (0, i32::MIN, i32::MAX),
        ] {
            let c = cfg_with(vel, u16::MAX, min, max);
            for th in [-9000, -1, 0, 25, 950, 1000, 1100, 9000] {
                for goal in [i32::MIN, -5000, -1, 0, 1, 700, 5000, i32::MAX] {
                    let mut t = TrajGen::new();
                    step_vel(&mut t, goal, th << 16, &c);
                    assert_eq!(
                        t.omega_star_q16(),
                        both(goal, th, (vel, min, max)).clamp(-a, a),
                        "goal {goal} th {th}"
                    );
                }
            }
        }
    }

    #[test]
    fn reversal_decelerates_through_zero() {
        let c = cfg(1000, 50 << 8);
        let mut t = TrajGen::new();
        t.reseed(0);
        for _ in 0..200 {
            step_pos(&mut t, 2000, DT, &c);
        }
        assert_eq!(t.omega_star_q16(), 1000 << 16);
        // flip the goal mid-cruise: omega walks down by at most a per tick
        // (2a on the one landing tick, where the snap zeroes creep + step)
        // and theta by at most one tick of travel - no teleport anywhere
        let mut prev_o = t.omega_star_q16();
        let mut prev_th = t.theta_star_q16();
        let mut seen_negative = false;
        for _ in 0..6000 {
            step_pos(&mut t, -2000, DT, &c);
            let o = t.omega_star_q16();
            let th = t.theta_star_q16();
            assert!(
                (o - prev_o).abs() <= 2 * A,
                "omega jump {} -> {}",
                prev_o,
                o
            );
            assert!(
                (th - prev_th).abs() <= 32767,
                "theta jump {} -> {}",
                prev_th,
                th
            );
            seen_negative |= o < 0;
            prev_o = o;
            prev_th = th;
            if o == 0 && th == -(2000 << 16) {
                assert!(seen_negative);
                return;
            }
        }
        panic!("no landing");
    }

    #[test]
    fn hostile_inputs_no_panic_bounded() {
        // extreme goals/seeds/configs (inverted soft limits included): no
        // panic, omega inside the i16 c/s cap, theta inside the guard
        for accel in [0u16, 1, u16::MAX] {
            for vel in [0u16, 1, u16::MAX] {
                for (lo, hi) in [(i32::MIN, i32::MAX), (100, -100)] {
                    let c = cfg_with(vel, accel, lo, hi);
                    for goal in [i32::MIN, -1, 0, 1, i32::MAX] {
                        for seed in [i32::MIN, 0, i32::MAX] {
                            let mut t = TrajGen::new();
                            t.reseed(seed);
                            for _ in 0..50 {
                                step_pos(&mut t, goal, DT, &c);
                                assert!(t.omega_star_q16().unsigned_abs() <= 32767u32 << 16);
                                assert!(t.theta_star_q16().unsigned_abs() <= 1 << 29);
                                step_vel(&mut t, goal, seed, &c);
                                assert!(t.omega_star_q16().unsigned_abs() <= 32767u32 << 16);
                                assert!(t.theta_star_q16().unsigned_abs() <= 1 << 29);
                            }
                        }
                    }
                }
            }
        }
    }
}
