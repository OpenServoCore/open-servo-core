use super::Medium;
use super::fast::Fast;
use crate::Shared;
use crate::kernel::config::KernelConfig;
use crate::kernel::faults::{self, FaultLatch};
use crate::kernel::limits;
use crate::math::q_mul_u;
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
        let prev_lim = self.limits.i_lim_counts();
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
        let band = self.limits.fold(
            pinned,
            self.fusion.omega_q16().unsigned_abs() >> 16,
            self.fusion.tau_d_counts().unsigned_abs(),
            self.fusion.theta_q16() >> 16,
            permit,
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
            let v = q_mul_u(lim.unsigned_abs(), r_q12 as u32, 12);
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
            | (permit as u8 * limits::flag::PERMIT);
    }
}
