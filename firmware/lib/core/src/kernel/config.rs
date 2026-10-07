//! The kernel's own copy of CONFIG and CALIB, prebuilt into the shapes its
//! consumers take. Configuration changes only on a host write, so the kernel
//! rebuilds this at a medium boundary when `Shared::config_gen` has moved,
//! and on its first tick; no tick copies a table block.

use super::{CurrentGains, KernelTiming, LimitCfg, PositionCfg, TrajCfg, VelocityGains};
use crate::estimator::{FusionGains, ThermAnchor, ThermGates, window};
use crate::math::q_mul_u;
use crate::regions::config::DecaySelect;
use crate::traits::DecayMode;
use crate::{RegionStorageRaw, Shared};

/// What the per-tick path reads; the medium path reads it too.
#[derive(Copy, Clone, Default)]
pub struct FastConfig {
    pub i_window_min_ticks: u16,
    pub v_window_min_ticks: u16,
    pub oc_trip_counts: u16,
    pub oc_trip_ticks: u8,
    pub drive_polarity: bool,
    /// `recip_ke_q == 0 || ke_vpc_q == 0`: the physics belt's verdict.
    pub ke_unset: bool,
    /// `duty_max_q15` as the OpenLoop clamp, at most `i16::MAX`.
    pub ol_duty_max_q15: u16,
    pub ol_decay: DecayMode,
    pub ol_zero_brake: bool,
    /// Smallest duty with a valid shunt window: where the OpenLoop ceiling
    /// restarts, published as `window_floor_q15`.
    pub ol_floor_q15: u16,
    pub v_undervolt_counts: u16,
    pub current: CurrentGains,
}

/// What only the medium and slow paths read.
#[derive(Copy, Clone, Default)]
pub struct MediumConfig {
    pub fusion: FusionGains,
    pub r_q12: u16,
    pub l_tick_q412: u16,
    pub recip_ke_q: u16,
    pub traj: TrajCfg,
    pub position: PositionCfg,
    pub velocity: VelocityGains,
    pub limits: LimitCfg,
    pub sensor_delta_max: u16,
    pub sensor_bad_count: u8,
    pub pos_error_counts: u16,
    pub pos_error_time_ticks: u32,
    pub therm_gates: ThermGates,
    pub therm_anchor: ThermAnchor,
}

#[derive(Copy, Clone, Default)]
pub struct KernelConfig {
    pub fast: FastConfig,
    pub medium: MediumConfig,
}

impl KernelConfig {
    pub(super) fn load(shared: &Shared, timing: &KernelTiming) -> Self {
        let p = shared.table.region_ptr();
        // SAFETY: reads of transport-owned regions - raw-pointer volatile
        // block copies, no `&T` formed, aligned repr(C) blocks inside the
        // static table (single-writer contract on `Kernel`).
        let (
            loop_cur,
            loop_vel,
            loop_pos,
            lim,
            therm,
            fus,
            fault,
            pos_lim,
            sense,
            winding,
            motor,
            motor_ext,
        ) = unsafe {
            (
                (&raw const (*p).config.loop_current).read_volatile(),
                (&raw const (*p).config.loop_velocity).read_volatile(),
                (&raw const (*p).config.loop_position).read_volatile(),
                (&raw const (*p).config.limits).read_volatile(),
                (&raw const (*p).config.thermal).read_volatile(),
                (&raw const (*p).config.fusion).read_volatile(),
                (&raw const (*p).config.fault_cfg).read_volatile(),
                (&raw const (*p).config.pos_limits).read_volatile(),
                (&raw const (*p).calib.sense).read_volatile(),
                (&raw const (*p).calib.winding).read_volatile(),
                (&raw const (*p).calib.motor).read_volatile(),
                (&raw const (*p).calib.motor_ext).read_volatile(),
            )
        };
        let ms_to_ticks = |ms: u16| q_mul_u(ms as u32, timing.ticks_per_ms_q16, 16);
        Self {
            fast: FastConfig {
                i_window_min_ticks: sense.i_window_min_ticks,
                v_window_min_ticks: sense.v_window_min_ticks,
                oc_trip_counts: lim.oc_trip_counts,
                oc_trip_ticks: lim.oc_trip_ticks,
                drive_polarity: lim.drive_polarity,
                ke_unset: motor.recip_ke_q == 0 || motor.ke_vpc_q == 0,
                ol_duty_max_q15: loop_cur.duty_max_q15.min(i16::MAX as u16),
                ol_decay: match lim.openloop_decay {
                    DecaySelect::Slow => DecayMode::Slow,
                    DecaySelect::Fast => DecayMode::Fast,
                },
                ol_zero_brake: lim.openloop_zero_brake,
                ol_floor_q15: window::floor_duty(
                    sense.i_window_min_ticks,
                    timing.pwm_arr,
                    timing.recip_arr_q24,
                ),
                v_undervolt_counts: therm.v_undervolt_counts,
                current: CurrentGains {
                    kp_q88: loop_cur.i_kp_q88,
                    ki_q412: loop_cur.i_ki_q412,
                    kaw_q412: loop_cur.i_kaw_q412,
                    ke_q412: motor.ke_vpc_q,
                    duty_max_q15: loop_cur.duty_max_q15,
                },
            },
            medium: MediumConfig {
                fusion: FusionGains {
                    b_i_q313: motor.b_i_q313,
                    l1_q016: fus.l1_q016,
                    l2_q88: fus.l2_q88,
                    l3_q88: fus.l3_q88,
                    fric_fc_counts: motor.fric_fc_counts,
                },
                r_q12: motor.r_q12,
                l_tick_q412: motor_ext.l_tick_q412,
                recip_ke_q: motor.recip_ke_q,
                traj: TrajCfg {
                    vel_limit_cps: loop_pos.velocity_limit_cps,
                    accel_limit_q88: loop_pos.accel_limit_q88,
                    pos_min_soft_counts: pos_lim.pos_min_soft_counts,
                    pos_max_soft_counts: pos_lim.pos_max_soft_counts,
                },
                position: PositionCfg {
                    kp_q88: loop_pos.p_kp_q88,
                    pos_deadband_counts: loop_pos.pos_deadband_counts,
                    vel_limit_cps: loop_pos.velocity_limit_cps,
                },
                velocity: VelocityGains {
                    kp_q88: loop_vel.v_kp_q88,
                    ki_q412: loop_vel.v_ki_q412,
                    kaw_q412: loop_vel.v_kaw_q412,
                    j_ff_q88: loop_vel.j_ff_q88,
                    fric_fc_counts: motor.fric_fc_counts,
                    fric_fv_q016: motor.fric_fv_q016,
                },
                limits: LimitCfg {
                    current_limit_counts: lim.current_limit_counts,
                    stall_response: lim.stall_response,
                    stall_omega_max_cps: lim.stall_omega_max_cps,
                    stall_time_ticks: ms_to_ticks(lim.stall_time_ms),
                    stall_yield_counts: lim.stall_yield_counts,
                    stall_release_counts: lim.stall_release_counts,
                    stall_tau_trip_counts: lim.stall_tau_trip_counts,
                    derate_start_cc: therm.derate_start_cc,
                    cutoff_cc: therm.cutoff_cc,
                    pos_min_soft_counts: pos_lim.pos_min_soft_counts,
                    pos_max_soft_counts: pos_lim.pos_max_soft_counts,
                },
                sensor_delta_max: fault.sensor_delta_max,
                sensor_bad_count: fault.sensor_bad_count,
                pos_error_counts: fault.pos_error_counts,
                pos_error_time_ticks: ms_to_ticks(fault.pos_error_time_ms),
                therm_gates: ThermGates {
                    i_min_counts: therm.rtherm_i_min_counts,
                    omega_max_cps: therm.rtherm_omega_max_cps,
                },
                therm_anchor: ThermAnchor {
                    r0_q12: winding.r0_q12,
                    t0_cc: winding.t0_cc,
                    k_r2t_q88: winding.k_r2t_q88,
                    mu_q016: winding.mu_q016,
                },
            },
        }
    }
}
