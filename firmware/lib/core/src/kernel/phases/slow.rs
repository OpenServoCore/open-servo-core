use super::Medium;
use super::loops::faults::{self, FaultLatch};
use crate::kernel::DECIM_SLOW;
use crate::kernel::config::KernelConfig;

impl Medium {
    /// SLOW, every DECIM_SLOW periods, in two stages a period apart, so
    /// neither sets the period's longest tick: on the due tick the permit
    /// lease, undervoltage and the two heats on the held current `i_held`
    /// warmed by the last estimate; on the next period's SLOW phase the
    /// winding estimate from the NTC and the heats, its derate and
    /// overtemperature.
    /// Whether a stage ran.
    pub(in crate::kernel) fn slow(
        &mut self,
        ntc_raw: u16,
        i_held: i16,
        cfg: &KernelConfig,
        faults: &mut FaultLatch,
    ) -> bool {
        self.slow_periods += 1;
        if self.slow_periods < DECIM_SLOW {
            if self.slow_periods == 1 {
                self.estimate(ntc_raw, cfg, faults);
                return true;
            }
            return false;
        }
        self.slow_periods = 0;
        self.slow_due(i_held, cfg, faults);
        true
    }

    fn slow_due(&mut self, i_held: i16, cfg: &KernelConfig, faults: &mut FaultLatch) {
        self.permit_ticks = self.permit_ticks.saturating_sub(1);
        self.limits
            .heat(i_held, self.thermal.warm_q16(), cfg.tau_shifts());
        // the rail tap samples drive or not, so a held sag never outlives the
        // sag itself: ack clears once the rail is back
        if self.vbus.vbus_counts() < cfg.thermal.v_undervolt_counts {
            faults.raise(faults::BIT_UNDER_VOLT, faults::CODE_UNDER_VOLT);
        }
    }

    /// The winding estimate on the NTC and the heats, its derate and the
    /// overtemperature fault; the seed runs it too, so a failed NTC faults
    /// before the first drive.
    pub(super) fn estimate(&mut self, ntc_raw: u16, cfg: &KernelConfig, faults: &mut FaultLatch) {
        let t_cc = self
            .thermal
            .update(ntc_raw, self.limits.heats(), &cfg.therm);
        // UNSET_CC sits under every threshold: no derate, no cutoff;
        // NTC_FAULT_CC over every one
        self.limits.update_derate(t_cc, &cfg.limits, &cfg.thermal);
        if t_cc >= cfg.thermal.cutoff_cc {
            faults.raise(faults::BIT_OVER_TEMP, faults::CODE_OVER_TEMP);
        }
    }
}
