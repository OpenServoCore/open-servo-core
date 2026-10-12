//! The kernel's own copy of CONFIG and CALIB, prebuilt into the shapes its
//! consumers take. Configuration changes only on a host write, so the kernel
//! rebuilds this in two halves on the free phases after `Shared::config_gen`
//! has moved, and whole on its first tick; no other tick copies a table
//! block.

use super::{CurrentGains, KernelTiming, LimitCfg, PositionCfg, TrajCfg, VelocityGains};
use crate::estimator::{FusionGains, NtcCfg, ThermCfg, window};
use crate::math::q_mul_u;
use crate::regions::config::DecaySelect;
use crate::regions::config::{DEFAULT_RTHERM_COLD_BAND_Q016, DEFAULT_RTHERM_SEAT_BAND_COUNTS};
use crate::traits::DecayMode;
use crate::{RegionStorageRaw, Shared};

/// One field of the table at `p` (`region_ptr`).
macro_rules! rd {
    ($p:ident . $($f:ident).+) => {
        // SAFETY: reads of transport-owned regions - raw-pointer volatile
        // field reads, no `&T` formed, aligned repr(C) fields inside the
        // static table (single-writer contract on `Kernel`).
        unsafe { (&raw const (*$p).$($f).+).read_volatile() }
    };
}

/// What the per-tick path reads; the medium path reads it too.
#[derive(Copy, Clone, Default)]
pub struct FastConfig {
    pub i_window_min_ticks: u16,
    /// The terminal floor per decay (`window::v_floors`), indexed by
    /// `DecayMode as usize`.
    pub v_floor_ticks: [u16; 2],
    pub oc_trip_counts: u16,
    pub oc_trip_ticks: u8,
    pub drive_polarity: bool,
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
    pub recip_ke_q: u16,
    /// `recip_ke_q == 0 || ke_vpc_q == 0`: the physics belt's verdict, from
    /// the same read as `recip_ke_q`.
    pub ke_unset: bool,
    pub traj: TrajCfg,
    pub position: PositionCfg,
    pub velocity: VelocityGains,
    pub limits: LimitCfg,
    pub sensor_delta_max: u16,
    pub sensor_bad_count: u8,
    pub pos_error_counts: u16,
    pub pos_error_time_ticks: u32,
    pub therm: ThermCfg,
}

#[derive(Copy, Clone, Default)]
pub struct KernelConfig {
    pub fast: FastConfig,
    pub medium: MediumConfig,
}

impl FastConfig {
    /// As `MediumConfig::load`.
    pub(super) fn load(&mut self, shared: &Shared, timing: &KernelTiming) {
        let p = shared.table.region_ptr();
        let i_window_min_ticks = rd!(p.calib.sense.i_window_min_ticks);
        self.i_window_min_ticks = i_window_min_ticks;
        self.ol_floor_q15 =
            window::floor_duty(i_window_min_ticks, timing.pwm_arr, timing.recip_arr_q24);
        self.v_floor_ticks = window::v_floors(
            rd!(p.calib.sense.v_window_min_ticks),
            timing.v_trough_min_ticks,
        );
        self.oc_trip_counts = rd!(p.config.limits.oc_trip_counts);
        self.oc_trip_ticks = rd!(p.config.limits.oc_trip_ticks);
        self.drive_polarity = rd!(p.config.limits.drive_polarity);
        self.ol_decay = match rd!(p.config.limits.openloop_decay) {
            DecaySelect::Slow => DecayMode::Slow,
            DecaySelect::Fast => DecayMode::Fast,
        };
        self.ol_zero_brake = rd!(p.config.limits.openloop_zero_brake);
        self.v_undervolt_counts = rd!(p.config.thermal.v_undervolt_counts);
        let duty_max_q15 = rd!(p.config.loop_current.duty_max_q15);
        self.ol_duty_max_q15 = duty_max_q15.min(i16::MAX as u16);
        self.current = CurrentGains {
            kp_q88: rd!(p.config.loop_current.i_kp_q88),
            ki_q412: rd!(p.config.loop_current.i_ki_q412),
            kaw_q412: rd!(p.config.loop_current.i_kaw_q412),
            ke_q412: rd!(p.calib.motor.ke_vpc_q),
            duty_max_q15,
        };
    }
}

impl MediumConfig {
    /// Field by field into place, each read where it is stored: a block
    /// copy holds the whole half live across the build.
    pub(super) fn load(&mut self, shared: &Shared, timing: &KernelTiming) {
        let p = shared.table.region_ptr();
        // a CONFIG image from before the bands existed reads them as zero
        let or_default = |v: u16, d: u16| if v == 0 { d } else { v };
        let ms_to_ticks = |ms: u16| q_mul_u(ms as u32, timing.ticks_per_ms_q16, 16);
        let fric_fc_counts = rd!(p.calib.motor.fric_fc_counts);
        self.fusion = FusionGains {
            b_i_q313: rd!(p.calib.motor.b_i_q313),
            l1_q016: rd!(p.config.fusion.l1_q016),
            l2_q88: rd!(p.config.fusion.l2_q88),
            l3_q88: rd!(p.config.fusion.l3_q88),
            fric_fc_counts,
        };
        self.velocity = VelocityGains {
            kp_q88: rd!(p.config.loop_velocity.v_kp_q88),
            ki_q412: rd!(p.config.loop_velocity.v_ki_q412),
            kaw_q412: rd!(p.config.loop_velocity.v_kaw_q412),
            j_ff_q88: rd!(p.config.loop_velocity.j_ff_q88),
            fric_fc_counts,
            fric_fv_q016: rd!(p.calib.motor.fric_fv_q016),
        };
        self.r_q12 = rd!(p.calib.motor.r_q12);
        self.recip_ke_q = rd!(p.calib.motor.recip_ke_q);
        self.ke_unset = self.recip_ke_q == 0 || rd!(p.calib.motor.ke_vpc_q) == 0;
        let vel_limit_cps = rd!(p.config.loop_position.velocity_limit_cps);
        let pos_min_soft_counts = rd!(p.config.pos_limits.pos_min_soft_counts);
        let pos_max_soft_counts = rd!(p.config.pos_limits.pos_max_soft_counts);
        self.traj = TrajCfg::new(
            vel_limit_cps,
            rd!(p.config.loop_position.accel_limit_q88),
            pos_min_soft_counts,
            pos_max_soft_counts,
        );
        self.position = PositionCfg {
            kp_q88: rd!(p.config.loop_position.p_kp_q88),
            pos_deadband_counts: rd!(p.config.loop_position.pos_deadband_counts),
            vel_limit_cps,
        };
        let current_limit_counts = rd!(p.config.limits.current_limit_counts);
        self.limits = LimitCfg {
            current_limit_counts,
            stall_response: rd!(p.config.limits.stall_response),
            stall_omega_max_cps: rd!(p.config.limits.stall_omega_max_cps),
            stall_time_ticks: ms_to_ticks(rd!(p.config.limits.stall_time_ms)),
            stall_yield_counts: rd!(p.config.limits.stall_yield_counts),
            stall_release_counts: rd!(p.config.limits.stall_release_counts),
            stall_tau_trip_counts: rd!(p.config.limits.stall_tau_trip_counts),
            derate_start_cc: rd!(p.config.thermal.derate_start_cc),
            cutoff_cc: rd!(p.config.thermal.cutoff_cc),
            pos_min_soft_counts,
            pos_max_soft_counts,
        };
        self.sensor_delta_max = rd!(p.config.fault_cfg.sensor_delta_max);
        self.sensor_bad_count = rd!(p.config.fault_cfg.sensor_bad_count);
        self.pos_error_counts = rd!(p.config.fault_cfg.pos_error_counts);
        self.pos_error_time_ticks = ms_to_ticks(rd!(p.config.fault_cfg.pos_error_time_ms));
        self.therm = ThermCfg {
            alpha_q24: rd!(p.calib.thermal.th_alpha_q24),
            g_q016: rd!(p.calib.thermal.th_g_q016),
            mu_q016: rd!(p.calib.thermal.th_mu_q016),
            r_cold_q12: rd!(p.calib.winding.r0_q12),
            i_min_counts: rd!(p.config.thermal.rtherm_i_min_counts),
            i_lim_counts: current_limit_counts,
            seat_band_counts: or_default(
                rd!(p.config.thermal_ext.rtherm_seat_band_counts),
                DEFAULT_RTHERM_SEAT_BAND_COUNTS,
            ),
            cold_band_q016: or_default(
                rd!(p.config.thermal_ext.rtherm_cold_band_q016),
                DEFAULT_RTHERM_COLD_BAND_Q016,
            ),
            ntc: NtcCfg {
                raw_ref: rd!(p.calib.thermal.ntc_raw_ref),
                t_ref_cc: rd!(p.calib.thermal.ntc_t_ref_cc),
                k1_q88: rd!(p.calib.thermal.ntc_k1_q88),
                k2_q24: rd!(p.calib.thermal.ntc_k2_q24),
            },
        };
    }
}
