use super::Medium;
use super::fast::{Command, Drive, Fast};
use super::lifecycle::Life;
use super::loops::current;
use super::loops::faults::{self, FaultLatch};
#[cfg(feature = "loop-margin")]
use super::loops::limits;
use crate::kernel::config::{KernelConfig, belt};
use crate::pos_lut;
#[cfg(feature = "loop-margin")]
use crate::regions::control::ControlInject;
use crate::regions::control::{ControlLifecycle, Mode};
use crate::{RegionStorageRaw, Shared};

/// One read of CONTROL.
#[derive(Copy, Clone, Default)]
pub struct Control {
    pub life: ControlLifecycle,
    /// `pos_lut_state` reads LIVE: the position table applies.
    pub lut_live: bool,
    #[cfg(feature = "loop-margin")]
    pub inject: ControlInject,
}

impl Control {
    pub fn read(shared: &Shared) -> Self {
        let p = shared.table.region_ptr();
        // SAFETY: reads of transport-owned regions - raw-pointer volatile
        // block copies, no `&T` formed, aligned repr(C) blocks inside the
        // static table (single-writer contract on `Kernel`).
        let (life, pos_lut_state) = unsafe {
            (
                (&raw const (*p).control.lifecycle).read_volatile(),
                (&raw const (*p).control.pos_lut.pos_lut_state).read_volatile(),
            )
        };
        Self {
            life,
            lut_live: pos_lut_state == pos_lut::state::LIVE,
            // SAFETY: as above.
            #[cfg(feature = "loop-margin")]
            inject: unsafe { (&raw const (*p).control.inject).read_volatile() },
        }
    }
}

#[cfg(feature = "loop-margin")]
impl Medium {
    /// The loop-margin stimulus's gate for this period's TRAJECTORY, on the
    /// last LIMITS flags. Either field 0 closes it, so an unasked stimulus
    /// leaves the Velocity arm the production image's.
    #[inline(always)]
    fn gate_inject(&mut self, streaming: bool) {
        let inj = &self.ctl.inject;
        self.inj_on = streaming
            && self.deadman.loaded()
            && self.limit_flags & !limits::flag::PERMIT == 0
            && inj.inject_accel_q88 != 0
            && inj.inject_half_periods != 0;
    }
}

impl Medium {
    /// CONTROL: the read, the dead-man's period on it, the edges, and the
    /// drive kind they imply. `streaming`: a TEL burst streams this tick.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::kernel) fn control(
        &mut self,
        raw_pos: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
        streaming: bool,
    ) {
        self.deadman.see(shared);
        self.ctl = Control::read(shared);
        // ahead of the edge: torque off has cleared the countdown by the
        // edge, so the edge's start counts from the next period
        let life = &self.ctl.life;
        self.deadman.step(
            life.torque_enable,
            life.bus_deadman_ticks,
            streaming,
            faults,
        );
        self.admit(cfg, shared, faults, fast);
        #[cfg(feature = "loop-margin")]
        self.gate_inject(streaming);
        self.edges(raw_pos, cfg, shared, faults, fast, cmd);
        cmd.drive = self.drive(cfg, faults);
        // the resets of an edge reach the fast half with the drive they
        // change
        cmd.i_ref_cc = self.i_ref_cc;
    }

    /// The torque enable edge, the data-state entry check and the belt.
    fn admit(
        &mut self,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
    ) {
        let life = &self.ctl.life;
        // torque_enable 0->1 is the fault ack: latch, detectors, and the
        // limits pend all clear; a still-present condition re-latches
        // through the normal detectors, an estimate still at its cutoff
        // here, ahead of the drive. The dead-man starts from the field.
        let enable_edge = life.torque_enable && !self.te_prev;
        if enable_edge {
            faults.clear();
            self.det.reset();
            fast.ack();
            self.limits.ack();
            self.cutoff(cfg, faults);
            self.deadman.start(life.bus_deadman_ticks);
        }
        self.te_prev = life.torque_enable;

        // Data-state entry check (data_state module): at the enable edge and
        // at a mode change under torque, a named reason refuses closed loop
        // for this run. A reason that appears mid-run (a live edit) waits
        // for the next entry; only the belt below stops a running loop.
        if life.torque_enable && (enable_edge || life.mode != self.mode_prev) {
            let p = shared.table.region_ptr();
            // SAFETY: same volatile read contract as `Control::read`; boot
            // and the dispatcher own this byte, the kernel only reads it.
            let data = unsafe { (&raw const (*p).telemetry.mode.data_flags).read_volatile() };
            if !crate::data_state::allows(life.mode, data) {
                faults.raise(faults::BIT_DATA, faults::CODE_DATA);
            }
        }
        // Physics belt: a closed loop on a zero Ke runs open and one on an
        // unset limit has none fit for its motor, whatever the flags say -
        // a live write of 0 into a running loop stops it here.
        if life.torque_enable && belt::refuses(cfg.belt, life.mode) {
            faults.raise(faults::BIT_DATA, faults::CODE_DATA);
        }
    }

    /// The run and mode edges (`lifecycle`).
    fn edges(
        &mut self,
        raw_pos: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &FaultLatch,
        fast: &mut Fast,
        cmd: &Command,
    ) {
        let run = self.ctl.life.torque_enable && faults.mask() == 0;
        if run != (self.life != Life::Off) {
            self.run_edge(run, raw_pos, cfg, shared, fast);
        }
        if self.ctl.life.mode != self.mode_prev {
            if run {
                self.mode_edge(cfg, fast, cmd);
            }
            self.mode_prev = self.ctl.life.mode;
        }
    }

    /// The drive kind the fast half runs: off, the hold's park, OpenLoop
    /// at its clamped goal, or the current loop.
    pub(in crate::kernel) fn drive(&self, cfg: &KernelConfig, faults: &FaultLatch) -> Drive {
        let life = &self.ctl.life;
        if !life.torque_enable || faults.mask() != 0 {
            return Drive::Off;
        }
        match life.mode {
            Mode::OpenLoop => {
                let max = cfg.loop_current.duty_max_q15 as i32;
                Drive::OpenLoop {
                    goal_q15: (life.goal_duty as i32).max(-max).min(max),
                }
            }
            _ if self.life == Life::Parked => Drive::Brake,
            // Ke decoupling rides the profile, not an estimate
            // (`current::u_ff_vc`); Current mode has no profile
            Mode::Velocity | Mode::Position => Drive::Closed {
                u_ff_vc: current::u_ff_vc(self.traj.omega_star_q16(), cfg.motor.ke_vpc_q),
            },
            Mode::Current => Drive::Closed { u_ff_vc: 0 },
        }
    }
}
