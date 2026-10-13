use super::Medium;
use super::fast::Measured;
use super::loops::faults::{self, FaultLatch};
use crate::estimator::{bemf, thermal};
use crate::kernel::config::KernelConfig;
use crate::kernel::{DECIM_SLOW, phase};
use crate::math::q_mul;

impl Medium {
    /// SLOW, every DECIM_SLOW periods, in two stages a period apart, so
    /// neither sets the period's longest tick: on the due tick the permit
    /// lease, undervoltage and the thermometer's step on this tick's
    /// window; on the next period's SLOW phase its temperature and
    /// overtemperature. Whether a stage ran.
    pub(in crate::kernel) fn slow(
        &mut self,
        m: &Measured,
        cfg: &KernelConfig,
        faults: &mut FaultLatch,
    ) -> bool {
        self.slow_periods += 1;
        if self.slow_periods < DECIM_SLOW {
            if self.slow_periods == 1 {
                self.slow_finish(cfg, faults);
                return true;
            } else if self.slow_periods == DECIM_SLOW - 1 {
                self.thermal.request(thermal::stage::NTC);
            }
            return false;
        }
        self.slow_periods = 0;
        self.slow_due(m, cfg, faults);
        true
    }

    fn slow_due(&mut self, m: &Measured, cfg: &KernelConfig, faults: &mut FaultLatch) {
        self.permit_ticks = self.permit_ticks.saturating_sub(1);
        // v_mean needs the taps' window valid (bemf RECIP_ARR contract)
        let v_mean = m.vdiff.map(|vdiff| {
            q_mul(
                m.ticks as i32 * vdiff,
                self.recip_arr_q24 as i32,
                bemf::RECIP_ARR_SHIFT,
            )
        });
        self.thermal.step(
            &thermal::Sample {
                v_mean,
                i_meas: m.i_meas,
                omega_hat_cps: self.fusion.omega_q16() >> 16,
                ke_vpc_q: cfg.motor.ke_vpc_q,
                pos_counts: self.fusion.theta_q16() >> 16,
            },
            &cfg.therm,
        );
        // the rail tap samples drive or not, so a held sag never outlives the
        // sag itself: ack clears once the rail is back
        if self.vbus.vbus_counts() < cfg.thermal.v_undervolt_counts {
            faults.raise(faults::BIT_UNDER_VOLT, faults::CODE_UNDER_VOLT);
        }
    }

    fn slow_finish(&mut self, cfg: &KernelConfig, faults: &mut FaultLatch) {
        let t_cc = self.thermal.finish(&cfg.therm);
        // the fold reads the derate from the next period's LIMITS on, so it
        // waits for SPARE_B; UNSET_CC sits under every threshold: no
        // derate, no cutoff
        self.thermal.request(thermal::stage::DERATE);
        if t_cc >= cfg.thermal.cutoff_cc {
            faults.raise(faults::BIT_OVER_TEMP, faults::CODE_OVER_TEMP);
        }
    }

    /// SPARE_A and SPARE_B, while the thermometer has work; whether
    /// a stage ran.
    pub(in crate::kernel) fn spare(&mut self, k: u8, ntc_raw: u16, cfg: &KernelConfig) -> bool {
        if k == phase::SPARE_A {
            let due = self.thermal.pending() & thermal::stage::INSTALL != 0;
            self.thermal.install();
            return due;
        }
        if self.thermal.take(thermal::stage::DERATE) {
            self.limits
                .update_derate(self.thermal.t_cc(), &cfg.limits, &cfg.thermal);
        }
        self.thermal.side(ntc_raw, &cfg.therm);
        true
    }
}
