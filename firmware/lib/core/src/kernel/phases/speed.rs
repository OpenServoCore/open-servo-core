use super::Medium;
use super::fast::Fast;
use crate::estimator::bemf;
use crate::kernel::config::KernelConfig;

impl Medium {
    /// SPEED: the back-EMF half this tick's sample ends closes, and the
    /// boxcar over it and the half before is the one the next period's
    /// OBSERVER and PUBLISH read. The brush term's weight is set by the
    /// speed this period's VELOCITY closed on.
    pub(in crate::kernel) fn speed(&mut self, cfg: &KernelConfig, fast: &mut Fast) {
        let xw_q12 = bemf::brush_xw_q12(self.omega_hat, cfg.brush_rc_q12, cfg.brush_lo_cps);
        self.omega_bemf = fast.close_bemf_half(
            self.r_hat_q12,
            xw_q12,
            cfg.motor.recip_ke_q,
            self.recip_arr_q24,
        );
    }
}
