//! The experiment image's medium half (kernel sec 2.7): CONTROL, the bus
//! dead-man, the bias tracker's aging and PUBLISH with the tracker's
//! feed. No loop, no estimator, no limit: the fast half drives the
//! OpenLoop goal as written, and the overcurrent trip and the dead-man are
//! the only raise points.

use super::deadman::Deadman;
use super::fast::{Command, Drive, Fast};
use super::loops::faults::FaultLatch;
use super::loops::limits::flag;
use crate::kernel::config::KernelConfig;
use crate::kernel::{DECIM_SLOW, KernelTiming};
use crate::regions::control::{ControlLifecycle, Mode};
use crate::{RegionStorageRaw, Shared};

#[cfg(test)]
mod tests;

pub(in crate::kernel) struct Medium {
    /// CONTROL as the CONTROL phase read it.
    life: ControlLifecycle,
    te_prev: bool,
    deadman: Deadman,
    /// Medium periods toward the next SLOW period.
    pub(in crate::kernel) slow_periods: u8,
}

impl Medium {
    pub fn new(_: &KernelTiming) -> Self {
        Self {
            life: ControlLifecycle::default(),
            te_prev: false,
            deadman: Deadman::new(),
            slow_periods: 0,
        }
    }

    /// CONTROL: the read, the enable edge and the drive kind.
    pub(in crate::kernel) fn control(
        &mut self,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
    ) {
        let p = shared.table.region_ptr();
        self.deadman.see(shared);
        // SAFETY: a read of a transport-owned region - a raw-pointer
        // volatile block copy, no `&T` formed, an aligned repr(C) block
        // inside the static table (single-writer contract on `Kernel`).
        self.life = unsafe { (&raw const (*p).control.lifecycle).read_volatile() };
        let life = &self.life;
        // torque_enable 0->1 is the fault ack, and the dead-man starts from
        // the field: a still-present trip re-latches through the detector
        let enable_edge = life.torque_enable && !self.te_prev;
        if enable_edge {
            faults.clear();
            fast.ack();
            self.deadman.start(life.bus_deadman_ticks);
        }
        self.te_prev = life.torque_enable;
        cmd.drive = self.drive(cfg, faults);
    }

    /// The drive kind the fast half runs: off, or OpenLoop at the clamped
    /// goal. The mode byte is accepted and ignored.
    fn drive(&self, cfg: &KernelConfig, faults: &FaultLatch) -> Drive {
        let life = &self.life;
        if !life.torque_enable || faults.mask() != 0 {
            return Drive::Off;
        }
        let max = cfg.loop_current.duty_max_q15 as i32;
        Drive::OpenLoop {
            goal_q15: (life.goal_duty as i32).max(-max).min(max),
        }
    }

    /// RAIL: the dead-man's period (`Deadman::step`). Out of line, so the
    /// tick body keeps its shape.
    #[inline(never)]
    #[rustc_align(4)]
    pub(in crate::kernel) fn deadman(&mut self, streaming: bool, faults: &mut FaultLatch) {
        let life = &self.life;
        self.deadman.step(
            life.torque_enable,
            life.bus_deadman_ticks,
            streaming,
            faults,
        );
    }

    /// SLOW, every DECIM_SLOW periods: the bias tracker's aging step, the
    /// image's whole slow block. It holds no TX lane.
    #[inline(always)]
    pub(in crate::kernel) fn slow(&mut self, fast: &mut Fast) {
        self.slow_periods += 1;
        if self.slow_periods >= DECIM_SLOW {
            self.slow_periods = 0;
            fast.age_bias();
        }
    }

    /// PUBLISH: the bias, the mode, the fault and the permit bit of
    /// `limit_flags`; the estimates, the other limits and the thermal
    /// fields keep their boot zero, nothing computes them.
    pub(in crate::kernel) fn publish(&self, shared: &Shared, faults: &FaultLatch, fast: &Fast) {
        let p = shared.table.region_ptr();
        // the dead-man stands in for the stall permit lease, so its loaded
        // countdown is the grant the burst arm reads for a seat outside the
        // soft limits (kernel sec 2.7)
        let permit = self.life.torque_enable && self.deadman.loaded();
        // SAFETY: sole-telemetry-writer contract (`Kernel` doc); volatile
        // per-field stores.
        unsafe {
            (&raw mut (*p).telemetry.refs.current_bias_counts).write_volatile(fast.bias_counts());
            (&raw mut (*p).telemetry.limits.limit_flags)
                .write_volatile(permit as u8 * flag::PERMIT);
            let md = &raw mut (*p).telemetry.mode;
            (&raw mut (*md).mode_active).write_volatile(Mode::OpenLoop as u8);
            (&raw mut (*md).fault_code).write_volatile(faults.code());
            (&raw mut (*p).telemetry.common.fault_flags).write_volatile(faults.mask());
        }
    }
}
