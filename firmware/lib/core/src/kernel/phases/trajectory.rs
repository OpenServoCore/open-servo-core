use super::Medium;
use super::fast::Command;
use super::loops::faults::FaultLatch;
use super::loops::position;
use crate::kernel::config::KernelConfig;
use crate::regions::control::Mode;

impl Medium {
    /// TRAJECTORY: the profile and the position loop.
    pub(in crate::kernel) fn trajectory(
        &mut self,
        cfg: &KernelConfig,
        faults: &FaultLatch,
        cmd: &mut Command,
    ) {
        if !self.run {
            return;
        }
        let (lp, soft) = (&cfg.loop_position, &cfg.pos_soft);
        let life = &self.ctl.life;
        let theta_hat = self.fusion.theta_q16();
        match life.mode {
            Mode::Position => {
                self.traj
                    .step_position(life.goal_position, self.dt_med_q32, lp, soft);
                let out = position::step(
                    self.traj.theta_star_q16(),
                    self.traj.omega_star_q16(),
                    theta_hat,
                    self.band_gain_q4,
                    lp,
                );
                self.omega_ref_q16 = out.omega_ref_q16;
                self.hold = out.hold;
            }
            Mode::Velocity => {
                self.traj
                    .step_velocity(life.goal_velocity, theta_hat, lp, soft);
                self.omega_ref_q16 = self.traj.omega_star_q16();
                self.hold = false;
            }
            Mode::Current | Mode::OpenLoop => self.hold = false,
        }
        cmd.drive = self.drive(cfg, faults);
    }
}
