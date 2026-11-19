//! The control path's lifecycle: one state (`Life`) and the cells each
//! event zeroes or reseeds. Every cell not named keeps stepping as before;
//! everything that measures the shaft (the observer, the back-EMF sums,
//! the blend inside a run, the rail and winding estimates) keeps through
//! every edge. Faulted is Off beside the latch, which is its own machine
//! (raised by the detectors, cleared by the enable edge). The soft-wall
//! Brake and the OpenLoop brakes are outputs, decided every tick, not
//! states.
//!
//! | Event | Lands at | Cells |
//! | --- | --- | --- |
//! | boot | first tick | all at rest; the shunt zero at the install's rest reading, the observer at the first sample |
//! | enable edge (`torque_enable` 0 -> 1) | CONTROL (`admit`) | latch, OC count, detectors, the limits' stall, yield and pending: acknowledged |
//! | run on | CONTROL | current integral, reference, velocity integral, blend, position loop output: 0; limiter at the floor; observer at the sample, or at rest on an in-band goal; profile at rest at the goal or the estimate; Run |
//! | run off | CONTROL | current integral, reference, velocity integral, blend, position loop output: 0; limiter at the floor; Off |
//! | mode change under run | CONTROL | limiter at the floor; Run. Between Velocity and Position: profile at the estimate, its speed kept. Otherwise: profile at rest at the estimate, velocity integral 0. From OpenLoop: current integral at the applied volts, reference at the held current inside the band, the dropped-pulse gate on OpenLoop's last width |
//! | park in | TRAJECTORY | Parked; every parked period at VELOCITY: reference, velocity integral, blend 0 |
//! | park out | TRAJECTORY | Run, or Held under a load |
//! | held in | TRAJECTORY | Held (loaded, where a free shaft parks); every held period at VELOCITY: the velocity loop steps with its integral frozen |
//! | held out | TRAJECTORY | Run, or Parked once the load is gone |
//! | fault | any tick | the latch; FAST drives Off that tick, run off follows at CONTROL |
//! | OpenLoop sign change | FAST (`fast.rs`) | limiter at the floor, held current 0 |
//!
//! The first closed tick after any tick that drove nothing (Off, either
//! Brake, an OpenLoop brake) has no reading: a zero width voids the window
//! and the current loop commands `r x i_ref` with its integral there. So
//! only the entry from OpenLoop reseeds the current loop: there the window
//! is live, and on a reference at the current that flows the integral's
//! steady state is the volts the bridge was applying, less the
//! feed-forward (0 at rest and in Current mode).

use super::fast::{Command, Fast};
use super::loops::position;
use super::loops::trajectory::TrajGen;
use super::{Medium, pos_q4};
use crate::Shared;
use crate::estimator::FusionObs;
use crate::kernel::config::KernelConfig;
use crate::regions::control::Mode;

#[derive(Copy, Clone, PartialEq, Eq)]
pub(in crate::kernel) enum Life {
    Off,
    Run,
    Parked,
    Held,
}

impl Life {
    /// A Position period's state from the hold's flags, which are
    /// exclusive (`position::PosOut`). Counted, not chosen: a select chain
    /// lays TRAJECTORY's blocks out past its budget.
    #[inline(always)]
    pub(super) fn of_hold(park: bool, hold: bool) -> Self {
        match (park | hold) as u8 + hold as u8 {
            0 => Life::Run,
            1 => Life::Parked,
            _ => Life::Held,
        }
    }
}

impl Medium {
    /// Torque on or off at CONTROL. A position goal inside the band of the
    /// estimate, with this sample agreeing within the zone, keeps the
    /// estimate's position and starts the profile on the goal: the host
    /// read the goal from that estimate, one sample sits farther off the
    /// shaft, and a profile from it to the goal drives a shaft already
    /// there. Otherwise fusion seeds at the sample and the profile at it:
    /// torque-off tau_d is built on a zero current, so a hand-moved shaft
    /// leaves the estimate stale and rails tau_d, and every enable would
    /// re-latch STALL via the collision check.
    #[inline(always)]
    pub(super) fn run_edge(
        &mut self,
        run: bool,
        raw_pos: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        fast: &mut Fast,
    ) {
        let life = &self.ctl.life;
        if run {
            let pos = pos_q4(shared, self.ctl.lut_live, raw_pos);
            let theta_hat = self.fusion.theta_q16();
            let goal = TrajGen::goal_q16(life.goal_position, &cfg.pos_soft);
            let db = cfg.loop_position.pos_deadband_counts;
            let band = position::band_q16(db, self.band_gain_q4);
            if life.mode == Mode::Position
                && db != 0
                && goal.abs_diff(theta_hat) <= band
                && FusionObs::seed_theta_q16(pos).abs_diff(theta_hat) <= band.saturating_add(band)
            {
                self.fusion.rest();
                self.traj.reseed(goal);
            } else {
                self.fusion.seed(pos);
                self.traj.reseed(self.fusion.theta_q16());
            }
        }
        fast.reset_current_loop();
        self.vel.reset();
        self.omega_blend.reset();
        fast.reset_limiter(cfg.ol_floor_q15);
        self.i_ref_cc = 0;
        self.omega_ref_q16 = 0;
        self.near_ticks = 0;
        self.life = if run { Life::Run } else { Life::Off };
    }

    /// A mode change under torque at CONTROL. The profile's speed is the
    /// reference the shaft follows, so a switch between the profile modes
    /// keeps it; from Current or OpenLoop there is none (the observer's is
    /// pot noise at rest). The velocity integral is the loop's answer to a
    /// speed error nothing in the new mode has stated.
    #[inline(always)]
    pub(super) fn mode_edge(&mut self, cfg: &KernelConfig, fast: &mut Fast, cmd: &Command) {
        let profiled = |m: Mode| matches!(m, Mode::Velocity | Mode::Position);
        let theta_hat = self.fusion.theta_q16();
        if profiled(self.ctl.life.mode) && profiled(self.mode_prev) {
            self.traj.rebase(theta_hat);
        } else {
            self.traj.reseed(theta_hat);
            self.vel.reset();
        }
        if self.mode_prev == Mode::OpenLoop {
            self.i_ref_cc = self.i_band.clamp(fast.i_meas_last() as i32);
            fast.enter_current_loop(cmd);
        }
        fast.reset_limiter(cfg.ol_floor_q15);
        self.life = Life::Run;
        self.near_ticks = 0;
    }

    /// Every parked period at VELOCITY: nothing drives, so the velocity
    /// loop stops too. omega_hat is the blend, and with no window its lead
    /// runs out (the reset here empties it), so at rest it reads the pot
    /// observer (+-hundreds c/s of noise); left running, the PI integrates
    /// that phantom error until i_ref pins at the current limit, and
    /// pinned + slow false-trips the stall detector (bench: CODE_STALL a
    /// few seconds into a clean hold).
    #[inline(always)]
    pub(super) fn parked(&mut self) {
        self.i_ref_cc = 0;
        self.vel.reset();
        self.omega_blend.reset();
    }
}
