use super::Medium;
use super::fast::Command;
use super::loops::faults::{self, FaultLatch};
use crate::Frame;
use crate::kernel::config::KernelConfig;
use crate::regions::control::Mode;

impl Medium {
    /// RAIL: the rail estimate on this tick's tap, the sensor screen and
    /// the position error timer.
    pub(in crate::kernel) fn rail(
        &mut self,
        frame: Frame<'_>,
        cfg: &KernelConfig,
        faults: &mut FaultLatch,
        cmd: &mut Command,
    ) {
        let fault_cfg = &cfg.fault_cfg;
        self.vbus
            .step(frame.vbus_raw(), cfg.thermal.v_undervolt_counts);
        cmd.vbus_counts = self.vbus.vbus_counts();
        cmd.vbus_recip_q15 = self.vbus.recip_q15();
        cmd.prepare_rail(cfg);
        // raw-pot sanity screen runs in every mode, torque-off included
        if self.det.sensor_sample(
            frame.pos(),
            fault_cfg.sensor_delta_max,
            fault_cfg.sensor_bad_count,
        ) {
            faults.raise(faults::BIT_SENSOR, faults::CODE_SENSOR);
        }
        // tracking-error persistence: only meaningful with a live profile
        let pos_err_over = self.run
            && self.ctl.life.mode == Mode::Position
            && self
                .traj
                .theta_star_q16()
                .saturating_sub(self.fusion.theta_q16())
                .unsigned_abs()
                > (fault_cfg.pos_error_counts as u32) << 16;
        if self
            .det
            .pos_err_sample(pos_err_over, fault_cfg.pos_error_time_ticks)
        {
            faults.raise(faults::BIT_POSITION_ERROR, faults::CODE_POSITION_ERROR);
        }
    }
}
