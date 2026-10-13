use super::Medium;
use super::fast::Fast;
use crate::Shared;
use crate::kernel::config::KernelConfig;
use crate::pos_lut;

impl Medium {
    /// OBSERVER, on this tick's sample.
    pub(in crate::kernel) fn observe(
        &mut self,
        raw_pos: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        fast: &mut Fast,
    ) {
        // the band gain is the interval this sample linearized in
        let (pos, band_gain_q4) = if self.ctl.lut_live {
            shared.pos_lut_q4_band_gain(raw_pos)
        } else {
            (pos_lut::identity_q4(raw_pos), pos_lut::GRID as u16)
        };
        let m = &cfg.motor;
        self.omega_bemf = fast.close_bemf_half(m.r_q12, m.recip_ke_q, self.recip_arr_q24);
        self.fusion.step(
            fast.i_meas_last() as i32,
            pos,
            self.dt_med_q32,
            &cfg.fusion,
            m,
        );
        // The observer's omega keeps the rest-shaped consumers (stall
        // verdict, thermometer gate): it is always there and reads small at
        // rest, where the boxcar has no window at all.
        self.omega_hat = self.omega_sw.step(self.omega_bemf, self.fusion.omega_q16());
        self.band_gain_q4 = band_gain_q4;
    }
}
