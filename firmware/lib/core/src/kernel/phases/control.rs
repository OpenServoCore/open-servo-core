use super::fast::{Command, Drive, Fast};
use super::loops::current;
use super::loops::faults::{self, FaultLatch};
use super::{Medium, pos_q4};
use crate::kernel::config::KernelConfig;
use crate::pos_lut;
use crate::regions::control::{ControlLifecycle, Mode};
use crate::{RegionStorageRaw, Shared};

/// One read of CONTROL.
#[derive(Copy, Clone, Default)]
pub struct Control {
    pub life: ControlLifecycle,
    /// `pos_lut_state` reads LIVE: the position table applies.
    pub lut_live: bool,
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
        }
    }
}

impl Medium {
    /// CONTROL: the read, the edges, and the drive kind they imply.
    pub(in crate::kernel) fn control(
        &mut self,
        raw_pos: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
    ) {
        self.ctl = Control::read(shared);
        self.admit(cfg, shared, faults, fast);
        self.edges(raw_pos, cfg, shared, faults, fast);
        cmd.drive = self.drive(cfg, faults);
        // the resets of an edge reach the fast half with the drive they
        // change
        cmd.i_ref_cc = self.i_ref_cc;
    }

    /// The torque enable edge, the data-state entry check and the Ke belt.
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
        // through the normal detectors.
        let enable_edge = life.torque_enable && !self.te_prev;
        if enable_edge {
            faults.clear();
            self.det.reset();
            fast.ack();
            self.limits.ack();
        }
        self.te_prev = life.torque_enable;

        // Data-state entry check (data_state module): at the enable edge and
        // at a mode change under torque, a named reason refuses closed loop
        // for this run. A reason that appears mid-run (a live edit) waits
        // for the next entry; only the Ke belt below stops a running loop.
        if life.torque_enable && (enable_edge || life.mode != self.mode_prev) {
            let p = shared.table.region_ptr();
            // SAFETY: same volatile read contract as `Control::read`; boot
            // and the dispatcher own this byte, the kernel only reads it.
            let data = unsafe { (&raw const (*p).telemetry.mode.data_flags).read_volatile() };
            if !crate::data_state::allows(life.mode, data) {
                faults.raise(faults::BIT_DATA, faults::CODE_DATA);
            }
        }
        // Physics belt: a closed loop on a zero Ke runs open (the boxcar
        // yields nothing, the current loop decouples nothing), whatever the
        // flags say - a live write of 0 into a running loop stops it here.
        if life.torque_enable
            && matches!(life.mode, Mode::Velocity | Mode::Position)
            && cfg.ke_unset
        {
            faults.raise(faults::BIT_DATA, faults::CODE_DATA);
        }
    }

    /// The run and mode edges.
    fn edges(
        &mut self,
        raw_pos: u16,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &FaultLatch,
        fast: &mut Fast,
    ) {
        let life = &self.ctl.life;
        let run = life.torque_enable && faults.mask() == 0;
        if run != self.run_prev {
            // both edges zero the loop chain; the enable edge additionally
            // reseeds fusion at the measurement (torque-off tau_d is built
            // on a zero current - a hand-moved shaft rails it and every
            // enable would re-latch STALL via the collision check) and the
            // profile at the fresh estimate - bumpless
            if run {
                self.fusion.seed(pos_q4(shared, self.ctl.lut_live, raw_pos));
                self.traj.reseed(self.fusion.theta_q16());
            }
            fast.reset_current_loop();
            self.vel.reset();
            fast.reset_limiter(cfg.ol_floor_q15);
            self.i_ref_cc = 0;
            self.omega_ref_q16 = 0;
            self.hold = false;
            self.run_prev = run;
        }
        if run && life.mode != self.mode_prev {
            // mode change mid-run: reference = estimate, at rest
            self.traj.reseed(self.fusion.theta_q16());
            fast.reset_limiter(cfg.ol_floor_q15);
            self.hold = false;
        }
        self.mode_prev = life.mode;
        self.run = run;
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
                    goal_q15: (life.goal_duty as i32).clamp(-max, max),
                }
            }
            _ if self.hold => Drive::Brake,
            // Ke decoupling rides the profile, not an estimate
            // (`current::u_ff_vc`); Current mode has no profile
            Mode::Velocity | Mode::Position => Drive::Closed {
                u_ff_vc: current::u_ff_vc(self.traj.omega_star_q16(), cfg.motor.ke_vpc_q),
            },
            Mode::Current => Drive::Closed { u_ff_vc: 0 },
        }
    }
}
