use super::fast::Fast;
use super::loops::faults::{self, FaultLatch};
use super::loops::limits;
use super::{Medium, pos_q4};
use crate::Shared;
use crate::kernel::config::KernelConfig;
use crate::math::q_mul_u;
use crate::pos_lut::GRID_SHIFT;
use crate::regions::control::Mode;

/// Stall permit lease in SLOW ticks: 63 x 16 ms = 1.008 s from the grant.
const PERMIT_LEASE_TICKS: u8 = 63;

impl Medium {
    /// LIMITS: the band, the OpenLoop base duty, the stall verdict.
    pub(in crate::kernel) fn limit(
        &mut self,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
    ) {
        let life = &self.ctl.life;
        // pinned = last command sat at a nonzero ceiling, in OpenLoop the
        // duty ceiling held under the goal
        let prev_lim = self.i_band.lim();
        let pinned = if life.mode == Mode::OpenLoop {
            fast.take_pinned()
        } else {
            prev_lim != 0 && self.i_ref_cc.unsigned_abs() >= prev_lim as u32
        };
        let permit_gen = shared.permit_gen();
        if permit_gen != self.permit_gen {
            self.permit_gen = permit_gen;
            self.permit_ticks = PERMIT_LEASE_TICKS;
        }
        if !life.torque_enable {
            self.permit_ticks = 0;
        }
        let permit = life.stall_permit && self.permit_ticks != 0;
        // the goal caps the closed loops' demand; in Current mode it is the
        // reference, and OpenLoop's knob is its duty (a reference left
        // behind by a Current-mode run must not cap a tool's drive)
        let goal_abs = if matches!(life.mode, Mode::Velocity | Mode::Position) {
            life.goal_current.unsigned_abs()
        } else {
            0
        };
        let limits::Fold { band, terms } = self.limits.fold(
            &limits::FoldIn {
                i_held_abs: fast.i_meas_last().unsigned_abs() as u32,
                omega_hat_abs_cps: self.fusion.omega_q16().unsigned_abs() >> 16,
                tau_d_abs_counts: self.fusion.tau_d_counts().unsigned_abs(),
                theta_hat_counts: self.fusion.theta_q16() >> 16,
                stall_permit: permit,
                goal_abs,
            },
            &cfg.limits,
            &cfg.pos_soft,
        );
        self.i_band = band;
        // the stall-safe duty lim x R / vbus: winding R alone, so it errs low
        // by the bridge and shunt; unset R keeps it at the floor
        let r_q12 = cfg.motor.r_q12;
        self.ol_base_q15 = if r_q12 == 0 {
            cfg.ol_floor_q15
        } else {
            let lim = if life.goal_duty >= 0 {
                band.hi
            } else {
                band.lo
            };
            // Both factors fit u16, so the product fits u32.
            let v = (lim.unsigned_abs() * r_q12 as u32) >> 12;
            q_mul_u(v, self.vbus.recip_q15(), 15).min(cfg.ol_floor_q15 as u32) as u16
        };
        if self.run && self.limits.stall_fault_pending() {
            faults.raise(faults::BIT_STALL, faults::CODE_STALL);
        }
        // one closed side makes the band asymmetric; a zero limit closes
        // both and is no endstop
        self.limit_flags = (pinned as u8 * limits::flag::CEILING)
            | (self.limits.stalled() as u8 * limits::flag::YIELD)
            | ((band.lo + band.hi != 0) as u8 * limits::flag::ENDSTOP)
            | (permit as u8 * limits::flag::PERMIT)
            | terms;
    }

    /// The observer's position, linearized Q4 counts: filtered, so the pot's
    /// noise times a steep table's gain does not end legs.
    #[inline(always)]
    pub(in crate::kernel) fn theta_q4(&self) -> i32 {
        self.fusion.theta_q16() >> (16 - GRID_SHIFT)
    }

    /// The drive direction check's stage (`DriveCheck::stage`): a leg's
    /// end, its verdict or an anchor, booked on top of the SPARE_B row
    /// (`budget::DRIVE_STAGE`). The closed loops' direction is their
    /// velocity command: on a reversed motor the duty flips sign every few
    /// periods as the velocity source swaps between the back-EMF and the
    /// pot, which disagree.
    #[inline(never)]
    #[rustc_align(4)]
    pub(in crate::kernel) fn drive_stage(
        &mut self,
        shared: &Shared,
        duty_q15: i16,
        faults: &mut FaultLatch,
    ) {
        let closed = matches!(self.ctl.life.mode, Mode::Velocity | Mode::Position);
        let omega_ref = self.omega_ref_q16;
        let band = self.i_band;
        let lut_live = self.ctl.lut_live;
        if self.drive.stage(
            self.theta_q4(),
            || pos_q4(shared, lut_live, shared.frame().pos()) as i32,
            || if closed { omega_ref } else { duty_q15 as i32 },
            closed,
            [band.lo == 0, band.hi == 0],
        ) {
            faults.raise(faults::BIT_DATA, faults::CODE_DRIVE_REVERSED);
        }
    }
}
