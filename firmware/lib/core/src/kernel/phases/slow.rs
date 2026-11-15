use super::Medium;
use super::fast::{Command, Fast};
use super::loops::faults::{self, FaultLatch};
use crate::kernel::DECIM_SLOW;
use crate::kernel::config::KernelConfig;
use crate::{RegionStorageRaw, Shared};

impl Medium {
    /// SLOW, every DECIM_SLOW periods, in two stages a period apart, so
    /// neither sets the period's longest tick: on the due tick the permit
    /// lease, undervoltage and the two heats on the cycle's mean of the
    /// held current squared (LIMITS sums it) warmed by the last estimate;
    /// on the next period's SLOW phase the winding estimate from the NTC
    /// and the heats, its cutoff, the resistance at its temperature, and
    /// the bias tracker's aging step. Whether a stage ran.
    pub(in crate::kernel) fn slow(
        &mut self,
        ntc_raw: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
    ) -> bool {
        self.slow_periods += 1;
        if self.slow_periods < DECIM_SLOW {
            if self.slow_periods == 1 {
                self.estimate(ntc_raw, cfg, shared, faults, cmd);
                fast.age_bias();
                return true;
            }
            return false;
        }
        self.slow_periods = 0;
        self.slow_due(cfg, faults);
        true
    }

    fn slow_due(&mut self, cfg: &KernelConfig, faults: &mut FaultLatch) {
        self.permit_ticks = self.permit_ticks.saturating_sub(1);
        let mean_sq = self.limits.take_heat_mean();
        self.limits
            .heat(mean_sq, self.thermal.warm_q16(), cfg.tau_shifts());
        // the rail tap samples drive or not, so a held sag never outlives the
        // sag itself: ack clears once the rail is back
        if self.vbus.vbus_counts() < cfg.thermal.v_undervolt_counts {
            faults.raise(faults::BIT_UNDER_VOLT, faults::CODE_UNDER_VOLT);
        }
    }

    /// The winding estimate on the NTC and the heats, its cutoff and the
    /// resistance at its temperature; the seed runs it too, so a failed
    /// NTC faults before the first drive. The NTC reads a wide code as the
    /// middle of its band, whole counts.
    pub(super) fn estimate(
        &mut self,
        ntc_raw: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        cmd: &mut Command,
    ) {
        // at most `ADC_WIDE_MID_MAX` over a 12-bit code
        let ntc = shared.adc_mid(ntc_raw) as u16;
        self.thermal.update(ntc, self.limits.heats(), &cfg.therm);
        self.cutoff(cfg, faults);
        self.r_hat(cfg, shared, cmd);
    }

    /// The one resistance the control path reads, the stored `r_q12` at
    /// the estimate's temperature (`WindingTherm::r_hat_q12`): the back-EMF
    /// boxcar's R, and through the command the current loop's on a tick
    /// with no reading, where an unidentified winding keeps the take's
    /// stand-in unscaled. Published here, where it changes: the estimate
    /// runs it, and the configuration take, so it is never stale against a
    /// new `r_q12`. Out of line: one copy behind both, which keep their
    /// shape.
    #[inline(never)]
    #[rustc_align(4)]
    pub(in crate::kernel) fn r_hat(
        &mut self,
        cfg: &KernelConfig,
        shared: &Shared,
        cmd: &mut Command,
    ) {
        let r_hat = self.thermal.r_hat_q12(cfg.motor.r_q12);
        self.r_hat_q12 = r_hat;
        cmd.r_loop_q12 = if cfg.motor.r_q12 == 0 {
            cfg.r_loop_q12
        } else {
            r_hat as u32
        };
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (`Kernel` doc); volatile
        // store.
        unsafe {
            (&raw mut (*p).telemetry.estimates.r_hat_q12).write_volatile(r_hat);
        }
    }

    /// The overtemperature fault on the last estimate: UNSET_CC sits under
    /// the cutoff, NTC_FAULT_CC over every one. The enable edge runs it
    /// too, so an ack over the cutoff never drives.
    pub(super) fn cutoff(&self, cfg: &KernelConfig, faults: &mut FaultLatch) {
        if self.thermal.t_cc() >= cfg.thermal.cutoff_cc {
            faults.raise(faults::BIT_OVER_TEMP, faults::CODE_OVER_TEMP);
        }
    }
}
