use super::fast::Command;
use super::{Life, Medium};
use crate::kernel::config::KernelConfig;
use crate::regions::control::Mode;

impl Medium {
    /// VELOCITY: the current reference, which lands in the command with
    /// the band and the base duty.
    pub(in crate::kernel) fn velocity(&mut self, cfg: &KernelConfig, cmd: &mut Command) {
        if self.life != Life::Off {
            let life = &self.ctl.life;
            match life.mode {
                Mode::Position if self.life == Life::Parked => self.parked(),
                Mode::Velocity | Mode::Position => {
                    self.i_ref_cc = self.vel.step(
                        self.omega_ref_q16,
                        self.omega_hat,
                        self.traj.alpha_star_q16(),
                        self.traj.omega_star_q16(),
                        self.i_band,
                        &cfg.loop_velocity,
                        &cfg.motor,
                        self.life == Life::Held,
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
