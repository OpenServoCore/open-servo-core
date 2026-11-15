use super::fast::Fast;
use super::loops::faults::{self, FaultLatch};
use super::loops::limits;
use super::{Life, Medium};
use crate::Shared;
use crate::kernel::config::KernelConfig;
use crate::math::q_mul_u;
use crate::regions::control::Mode;

/// Stall permit lease in SLOW ticks: 63 x 16 ms = 1.008 s from the grant.
const PERMIT_LEASE_TICKS: u8 = 63;

impl Medium {
    /// LIMITS: the band, the OpenLoop base duty, the stall verdict, and
    /// the held current's square into the heat's SLOW-cycle sum.
    pub(in crate::kernel) fn limit(
        &mut self,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
    ) {
        let life = &self.ctl.life;
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
        // Current mode's band folds on the blend, the speed the velocity
        // loop closes on, at that loop's own gain
        let speed_fold = if life.mode == Mode::Current {
            limits::speed_fold(
                cfg.loop_position.speed_ceiling_cps,
                self.omega_hat,
                cfg.loop_velocity.v_kp_q88,
            )
        } else {
            u16::MAX
        };
        let i_held_abs = fast.i_meas_last().unsigned_abs();
        self.limits.heat_sample(i_held_abs);
        let limits::Fold { band, lim, terms } = self.limits.fold(
            &limits::FoldIn {
                i_held_abs: i_held_abs as u32,
                omega_hat_abs_cps: self.fusion.omega_q16().unsigned_abs() >> 16,
                tau_d_abs_counts: self.fusion.tau_d_counts().unsigned_abs(),
                theta_hat_counts: self.fusion.theta_q16() >> 16,
                stall_permit: permit,
                goal_abs,
                speed_fold,
                omega_q16: self.omega_hat,
            },
            &cfg.limits,
            &cfg.pos_soft,
        );
        self.i_band = band;
        // the stall-safe duty lim x R / vbus: winding R alone, so it errs low
        // by the bridge and shunt; unset R keeps it at the floor. The stored
        // `r_q12`, not the one at the estimate's temperature the loops read:
        // this is a ceiling, which must err low, and a cold resistance does
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
        if self.life != Life::Off && self.limits.stall_fault_pending() {
            faults.raise(faults::BIT_STALL, faults::CODE_STALL);
        }
        // pinned = last command sat at a nonzero ceiling, in OpenLoop the
        // duty ceiling held under the goal
        let prev_lim = self.i_lim;
        let pinned = if life.mode == Mode::OpenLoop {
            fast.take_pinned()
        } else {
            prev_lim != 0 && self.i_ref_cc.unsigned_abs() >= prev_lim as u32
        };
        self.i_lim = lim;
        self.limit_flags = (pinned as u8 * limits::flag::CEILING)
            | (self.limits.stalled() as u8 * limits::flag::YIELD)
            | (permit as u8 * limits::flag::PERMIT)
            | cfg.unset_flag
            | terms;
    }
}
