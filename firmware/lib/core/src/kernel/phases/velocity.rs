use super::Medium;
use super::fast::Command;
use crate::kernel::config::KernelConfig;
use crate::regions::control::Mode;

impl Medium {
    /// VELOCITY: the current reference, which lands in the command with
    /// the band and the base duty.
    pub(in crate::kernel) fn velocity(&mut self, cfg: &KernelConfig, cmd: &mut Command) {
        if self.run {
            let life = &self.ctl.life;
            match life.mode {
                // Parked: nothing drives, so the velocity loop must stop too.
                // omega_hat is the pot observer at rest (no window,
                // +-hundreds c/s of noise); left running, the PI integrates
                // that phantom error until i_ref pins at the current limit,
                // and pinned + slow false-trips the stall detector (bench:
                // CODE_STALL a few seconds into a clean hold). Zero the
                // command and drain the integrator so it never winds and
                // resumes bumplessly.
                Mode::Position if self.hold => {
                    self.i_ref_cc = 0;
                    self.vel.reset();
                }
                Mode::Velocity | Mode::Position => {
                    self.i_ref_cc = self.vel.step(
                        self.omega_ref_q16,
                        self.omega_hat,
                        self.traj.alpha_star_q16(),
                        self.traj.omega_star_q16(),
                        self.i_band,
                        &cfg.loop_velocity,
                        &cfg.motor,
                    );
                }
                // directional band: an endstop blocks only inward goals;
                // retreat clamps against the composed limit
                Mode::Current => self.i_ref_cc = self.i_band.clamp(life.goal_current as i32),
                Mode::OpenLoop => self.i_ref_cc = 0,
            }
        }
        cmd.i_ref_cc = self.i_ref_cc;
        cmd.band = self.i_band;
        cmd.ol_base_q15 = self.ol_base_q15;
    }
}
