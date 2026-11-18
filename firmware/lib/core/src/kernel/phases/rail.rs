use super::fast::{Command, Fast};
use super::loops::faults::{self, FaultLatch};
use super::{Life, Medium};
use crate::Shared;
use crate::kernel::config::KernelConfig;
use crate::regions::control::Mode;

impl Medium {
    /// RAIL: the rail estimate on this tick's tap, a wide code read as the
    /// middle of its band, its fold and the window term on it; the sensor
    /// screen on the raw pot and the position error timer.
    pub(in crate::kernel) fn rail(
        &mut self,
        shared: &Shared,
        cfg: &KernelConfig,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
    ) {
        let frame = shared.frame();
        let fault_cfg = &cfg.fault_cfg;
        let vbus_raw = frame.vbus_raw();
        self.vbus.step(
            vbus_raw,
            shared.adc_wide_q4(vbus_raw),
            cfg.thermal.v_undervolt_counts,
        );
        cmd.vbus_counts = self.vbus.vbus_counts();
        cmd.vbus_recip_q15 = self.vbus.recip_q15();
        let vbus_eff = cmd.prepare_rail(cfg);
        self.limits.update_rail(
            cmd.vbus_counts,
            cfg.thermal.v_undervolt_counts,
            fault_cfg.rail_fold_q8,
        );
        // raw-pot sanity screen runs in every mode, torque-off included
        if self.det.sensor_sample(
            frame.pos(),
            fault_cfg.sensor_delta_max,
            fault_cfg.sensor_bad_count,
        ) {
            faults.raise(faults::BIT_SENSOR, faults::CODE_SENSOR);
        }
        // tracking-error persistence: only meaningful with a live profile
        let pos_err_over = self.life != Life::Off
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
        // after the frame's reads, which `budget::FRAME_DONE` bounds
        fast.prepare_window(cfg.g_arr, vbus_eff);
    }
}
