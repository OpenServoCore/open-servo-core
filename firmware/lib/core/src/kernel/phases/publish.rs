use super::Medium;
use super::fast::Fast;
use crate::estimator::bemf;
use crate::kernel::faults::FaultLatch;
use crate::{RegionStorageRaw, Shared};

impl Medium {
    /// PUBLISH: the estimates, mode and limits; the raw frame and the
    /// tick's conclusions are in the table every tick (`TelemetryTick`).
    pub(in crate::kernel) fn publish(&self, shared: &Shared, faults: &FaultLatch, fast: &Fast) {
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (`Kernel` doc); volatile
        // per-field stores.
        unsafe {
            (&raw mut (*p).telemetry.refs.current_bias_counts).write_volatile(fast.bias_counts());
            let e = &raw mut (*p).telemetry.estimates;
            (&raw mut (*e).theta_hat_q16).write_volatile(self.fusion.theta_q16());
            (&raw mut (*e).omega_hat_cps).write_volatile(self.omega_hat);
            (&raw mut (*e).tau_d_counts).write_volatile(self.fusion.tau_d_counts());
            (&raw mut (*e).i_lim_counts).write_volatile(self.limits.i_lim_counts());
            (&raw mut (*e).t_winding_cc).write_volatile(self.thermal.t_cc());
            (&raw mut (*e).vbus_counts).write_volatile(self.vbus.vbus_counts());
            (&raw mut (*e).omega_bemf_cps)
                .write_volatile(bemf::omega_cps_i16(self.omega_bemf.unwrap_or(0)));
            (&raw mut (*e).r_hat_q12).write_volatile(self.thermal.r_q12());
            let md = &raw mut (*p).telemetry.mode;
            (&raw mut (*md).mode_active).write_volatile(self.ctl.life.mode as u8);
            (&raw mut (*md).fault_code).write_volatile(faults.code());
            (&raw mut (*md).omega_hat_src).write_volatile(self.omega_sw.source() as u8);
            (&raw mut (*p).telemetry.common.fault_flags).write_volatile(faults.mask());
            (&raw mut (*p).telemetry.limits.limit_flags).write_volatile(self.limit_flags);
            let th = &raw mut (*p).telemetry_ext.therm;
            (&raw mut (*th).t_ntc_cc).write_volatile(self.thermal.t_ntc_cc());
            (&raw mut (*th).therm_flags).write_volatile(self.thermal.flags());
        }
    }
}
