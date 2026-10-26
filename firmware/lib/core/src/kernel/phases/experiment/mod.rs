//! The experiment image's medium half (kernel sec 2.7): CONTROL, the bus
//! dead-man and PUBLISH. No loop, no estimator, no limit: the fast half
//! drives the OpenLoop goal as written, and the overcurrent trip and the
//! dead-man are the only raise points.

use super::fast::{Command, Drive, Fast};
use super::loops::faults::{self, FaultLatch};
use crate::kernel::config::KernelConfig;
use crate::kernel::{DECIM_MED, KernelTiming};
use crate::regions::control::{ControlLifecycle, Mode};
use crate::{RegionStorageRaw, Shared};

#[cfg(test)]
mod tests;

/// The bus dead-man countdown of `bus_deadman_ticks`, stepped once per
/// medium period on the RAIL phase.
struct Deadman {
    /// Ticks left; 0 while off, lapsed or torque off.
    ticks: u16,
    /// `Shared::bus_gen` at the last reload.
    bus_gen: u8,
}

pub(in crate::kernel) struct Medium {
    /// CONTROL as the CONTROL phase read it.
    life: ControlLifecycle,
    te_prev: bool,
    deadman: Deadman,
}

impl Medium {
    pub fn new(_: &KernelTiming) -> Self {
        Self {
            life: ControlLifecycle::default(),
            te_prev: false,
            deadman: Deadman {
                ticks: 0,
                bus_gen: 0,
            },
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
            self.deadman.ticks = life.bus_deadman_ticks;
            self.deadman.bus_gen = shared.bus_gen();
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

    /// RAIL: the dead-man's period. An instruction the bus dispatched since
    /// the last period reloads the countdown from the field; a streaming
    /// burst holds it; else it counts the period down, and at zero with
    /// the field nonzero the fault latches. Out of line, so the tick body
    /// keeps its shape.
    #[inline(never)]
    #[rustc_align(4)]
    pub(in crate::kernel) fn deadman(
        &mut self,
        shared: &Shared,
        streaming: bool,
        faults: &mut FaultLatch,
    ) {
        let life = &self.life;
        if !life.torque_enable {
            self.deadman.ticks = 0;
            return;
        }
        let bus_gen = shared.bus_gen();
        if bus_gen != self.deadman.bus_gen {
            self.deadman.bus_gen = bus_gen;
            self.deadman.ticks = life.bus_deadman_ticks;
        } else if !streaming {
            self.deadman.ticks = self.deadman.ticks.saturating_sub(DECIM_MED as u16);
            if self.deadman.ticks == 0 && life.bus_deadman_ticks != 0 {
                faults.raise(faults::BIT_DEADMAN, faults::CODE_DEADMAN);
            }
        }
    }

    /// PUBLISH: the bias, the mode and the fault; the estimates, limits
    /// and thermal fields keep their boot zero, nothing computes them.
    pub(in crate::kernel) fn publish(&self, shared: &Shared, faults: &FaultLatch, fast: &Fast) {
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (`Kernel` doc); volatile
        // per-field stores.
        unsafe {
            (&raw mut (*p).telemetry.refs.current_bias_counts).write_volatile(fast.bias_counts());
            let md = &raw mut (*p).telemetry.mode;
            (&raw mut (*md).mode_active).write_volatile(Mode::OpenLoop as u8);
            (&raw mut (*md).fault_code).write_volatile(faults.code());
            (&raw mut (*p).telemetry.common.fault_flags).write_volatile(faults.mask());
        }
    }
}
