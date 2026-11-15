use super::Medium;
use super::fast::Fast;
use crate::Shared;
use crate::kernel::config::KernelConfig;

impl Medium {
    /// OBSERVER, on this tick's sample.
    pub(in crate::kernel) fn observe(
        &mut self,
        raw_pos: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        fast: &Fast,
    ) {
        // the band gain is the interval this sample linearized in
        let (pos, band_gain_q4) = super::pos_q4_band_gain(shared, self.ctl.lut_live, raw_pos);
        self.fusion.step(
            fast.i_meas_last() as i32,
            pos,
            self.dt_med_q32,
            &cfg.fusion,
            &cfg.motor,
        );
        // Only the velocity loop reads the blend. The observer's omega
        // keeps the rest-shaped consumer (the stall verdict): it is always
        // there and reads small at rest, where the boxcar has no window at
        // all.
        self.omega_hat = self.omega_blend.step(
            self.omega_bemf,
            fast.bemf.flipped(),
            self.fusion.omega_q16(),
        );
        self.band_gain_q4 = band_gain_q4;
    }
}
