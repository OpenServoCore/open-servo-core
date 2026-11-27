use super::fast::Command;
use super::loops::faults::FaultLatch;
use super::loops::position::{self, HOLD_SETTLE_TICKS};
#[cfg(feature = "loop-margin")]
use super::loops::trajectory;
use super::{Life, Medium};
use crate::kernel::config::KernelConfig;
#[cfg(feature = "loop-margin")]
use crate::regions::config::ConfigLoopPosition;
use crate::regions::control::Mode;
#[cfg(feature = "loop-margin")]
use crate::regions::control::{INJECT_ACCEL_MAX_Q88, INJECT_HALF_PERIODS_MAX};

impl Medium {
    /// TRAJECTORY: the profile and the position loop. One out-of-line body
    /// with the profile step inlined, called straight from
    /// `Kernel::on_tick`: one call where the step's frame and dispatch
    /// would wrap a second, and `Medium::step` keeps its cold arms in
    /// branch reach (v006-performance sec 3). `raw_pos` is the tick's pot
    /// sample, the thrown hold's rest. Returns the step's "the TX lane
    /// holds": always.
    #[inline(never)]
    #[rustc_align(4)]
    pub(in crate::kernel) fn trajectory(
        &mut self,
        cfg: &KernelConfig,
        faults: &FaultLatch,
        cmd: &mut Command,
        raw_pos: u16,
    ) -> bool {
        if self.life == Life::Off {
            return true;
        }
        let (lp, soft) = (&cfg.loop_position, &cfg.pos_soft);
        let life = &self.ctl.life;
        let theta_hat = self.fusion.theta_q16();
        match life.mode {
            Mode::Position => {
                self.traj
                    .step_position(life.goal_position, self.dt_med_q32, lp, soft);
                let settled = self.near_ticks == HOLD_SETTLE_TICKS;
                let tau_abs = self.fusion.tau_d_q16().unsigned_abs();
                // ccounts -> ccQ16 fits u32 for any level
                let fric = (cfg.motor.fric_fc_counts as u32) << 16;
                let mut out = position::step(
                    self.traj.theta_star_q16(),
                    self.traj.omega_star_q16(),
                    theta_hat,
                    self.band_gain_q4,
                    settled,
                    tau_abs <= fric,
                    lp,
                );
                // only on a profile at rest before and after the step: a band
                // is judged only on a stopped one, and the applied acceleration
                // is 0 only if no speed was there before. The landing period,
                // the dearest path, takes the clear alone
                if out.band == 0 || self.traj.alpha_star_q16() != 0 {
                    self.thrown.clear();
                } else if self.thrown.parks(
                    raw_pos,
                    out.e_abs,
                    out.band,
                    self.life != Life::Run && !(out.park | out.hold),
                    settled,
                    tau_abs,
                    fric,
                ) {
                    out.park = true;
                    out.hold = false;
                }
                self.omega_ref_q16 = out.omega_ref_q16;
                self.life = Life::of_hold(out.park, out.hold);
                self.near_ticks = if out.near {
                    self.near_ticks + (self.near_ticks < HOLD_SETTLE_TICKS) as u8
                } else {
                    0
                };
            }
            Mode::Velocity => {
                self.traj
                    .step_velocity(life.goal_velocity, theta_hat, lp, soft);
                self.omega_ref_q16 = self.traj.omega_star_q16();
                #[cfg(feature = "loop-margin")]
                if self.inj_on {
                    self.inject(lp);
                }
                self.life = Life::Run;
            }
            Mode::Current | Mode::OpenLoop => self.life = Life::Run,
        }
        cmd.drive = self.drive(cfg, faults);
        true
    }
}

#[cfg(feature = "loop-margin")]
impl Medium {
    /// The loop-margin stimulus (kernel sec 2.7): the triangle summed into
    /// the velocity loop's reference, inside the speed ceiling that bounds
    /// the reference it rides. Its phase steps only on a period it runs, so
    /// a gated period holds it.
    #[inline(always)]
    fn inject(&mut self, lp: &ConfigLoopPosition) {
        let inj = self.ctl.inject;
        let n = inj.inject_half_periods.min(INJECT_HALF_PERIODS_MAX) as i32;
        let a = inj.inject_accel_q88.min(INJECT_ACCEL_MAX_Q88) as i32;
        let p = self.inj_p as i32 + 1;
        let p = if p >= n + n { 0 } else { p };
        self.inj_p = p as u16;
        // |triangle| <= 256 and a < 2^15: the product stays under 2^23, so
        // the Q8.8 -> csQ16 shift stays inside i32
        let d = (a * ((n >> 1) - (p - n).abs())) << 8;
        let v_lim = trajectory::ceiling_cap(lp) << 16;
        self.omega_ref_q16 = self.omega_ref_q16.saturating_add(d).max(-v_lim).min(v_lim);
    }
}
