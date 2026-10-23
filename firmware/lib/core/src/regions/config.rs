use control_table::{Block, Enum, Section};

use crate::kernel::phases::loops::trajectory::{GOAL_LIM_COUNTS, VEL_CAP_CPS};
use crate::traits::DecayMode;

// Wire vocabulary, hoisted to the foundation crate so the host column can
// name it (grid law 2). The table field below stays a raw index because the
// `Enum` derive impls a control-table trait -- orphan-rule-bound to whichever
// crate defines the type.
pub use osc_protocol::wire::{BaudRate, DEFAULT_RESPONSE_DEADLINE_US};

/// Stall policy: Fault latches STALL; Yield folds the current limit.
/// `repr(u8)`; constructing from an unlisted discriminant is UB, so
/// validators MUST gate writes to `StallResponse::ALLOWED`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum StallResponse {
    #[default]
    Fault = 0,
    Yield = 1,
}

/// protocol sec 5.4 CONFIG-COMMON block: identity RO at the region front,
/// comms RW at 0x010, reserved bytes between and behind. Model-specific
/// config starts at 0x020.
#[repr(C)]
#[derive(Copy, Clone, Block)]
#[ct_block(hooks = crate::regions::hooks::ControlTableHookEvents)]
pub struct ConfigCommon {
    #[ct_field(access = ro)]
    pub model_number: u16,
    #[ct_field(access = ro)]
    pub firmware_version: u16,
    #[ct_field(access = ro)]
    pub capability_flags: u32,
    #[ct_field(access = ro)]
    pub hardware_revision: u8,
    #[ct_field(skip)]
    pub _rsvd_identity: u8,
    #[ct_field(access = ro)]
    pub tel_row_bytes_max: u8,
    #[ct_field(access = ro)]
    pub tel_chunks_max: u8,
    #[ct_field(skip)]
    pub _rsvd_tel: [u8; 4],
    #[ct_field(within = (1u8, 249u8), hook = on_id_write)]
    pub id: u8,
    // `BaudRate` index; le gate = the enum ceiling. Zero (0.5M, the rescue
    // floor) keeps the const image all-zero so SHARED lands in .bss; the
    // operational default is `ConfigDefaults.baud`, seeded at boot.
    #[ct_field(le = 3u8, hook = on_baud_rate_idx_write)]
    pub baud_rate_idx: u8,
    #[ct_field(hook = on_response_deadline_us_write)]
    pub response_deadline_us: u16,
    #[ct_field(skip)]
    pub _rsvd_tail: [u8; 12],
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigPosPhys {
    #[ct_field(lt = &addr::pos_phys::POS_MAX_PHYS_COUNTS)]
    pub pos_min_phys_counts: i32,
    #[ct_field(gt = &addr::pos_phys::POS_MIN_PHYS_COUNTS)]
    pub pos_max_phys_counts: i32,
}

/// Inside the physical stops and the trajectory's goal range, so the kernel
/// takes them as stored.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigPosSoft {
    #[ct_field(
        within = (&addr::pos_phys::POS_MIN_PHYS_COUNTS, &addr::pos_phys::POS_MAX_PHYS_COUNTS),
        lt = &addr::pos_soft::POS_MAX_SOFT_COUNTS,
        within = (-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS),
    )]
    pub pos_min_soft_counts: i32,
    #[ct_field(
        within = (&addr::pos_phys::POS_MIN_PHYS_COUNTS, &addr::pos_phys::POS_MAX_PHYS_COUNTS),
        gt = &addr::pos_soft::POS_MIN_SOFT_COUNTS,
        within = (-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS),
    )]
    pub pos_max_soft_counts: i32,
}

/// Current PI gains, voltage-domain clamp + back-calc anti-windup.
/// Gains carry i_/v_/p_ prefixes: descriptor field names are table-flat.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLoopCurrent {
    pub i_kp_q88: u16,
    pub i_ki_q412: u16,
    pub i_kaw_q412: u16,
    #[ct_field(le = 32767u16)]
    pub duty_max_q15: u16,
}

// Full-scale duty by default; gains stay zero so the loop boots inert.
pub const DEFAULT_DUTY_MAX_Q15: u16 = 32767;

/// Velocity PI gains + inertia feedforward.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLoopVelocity {
    pub v_kp_q88: u16,
    pub v_ki_q412: u16,
    pub v_kaw_q412: u16,
    pub j_ff_q88: u16,
}

/// Position P gain, anti-hunt hold deadband, trajectory limits.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLoopPosition {
    pub p_kp_q88: u16,
    /// Raw counts of the position sensor, scaled by the position table's
    /// local gain into the linearized error. 0 = hold disabled. Hold brakes
    /// on position rest alone (position.rs).
    pub pos_deadband_counts: u16,
    #[ct_field(le = VEL_CAP_CPS)]
    pub velocity_limit_cps: u16,
    /// c/s per medium tick.
    pub accel_limit_q88: u16,
}

// Trajectory limits are clamps, not gains: at 0 the profile pins omega_ref
// to 0 and velocity/position modes are inert even with live gains, so they
// get real defaults (bench-validated on the SG90 rig: 1500 c/s, 15 c/s per
// medium tick = 0 -> 1500 in 50 ms).
pub const DEFAULT_VELOCITY_LIMIT_CPS: u16 = 1500;
pub const DEFAULT_ACCEL_LIMIT_Q88: u16 = 3840;

// Anti-hunt hold: at 0 the park never engages and the position loop chases
// pot noise at rest (audible shake on the SG90 rig). Modest pot-noise-scale
// backstop; identification refines it from the measured sigma_theta.
pub const DEFAULT_POS_DEADBAND_COUNTS: u16 = 12;

/// The hard peak cap, the continuous rating and its time constant, the
/// stall policy, the overcurrent trip window.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLimits {
    /// The hard peak cap: the motor's nameplate stall current, which the
    /// train is rated to deliver. Nothing composes over it.
    pub current_limit_counts: u16,
    pub stall_response: StallResponse,
    /// false = the wiring is reversed: the kernel negates its motor output so
    /// positive duty and current still increase position counts.
    pub drive_polarity: bool,
    /// Kernel ticks (`CalibSense.tick_hz`).
    pub stall_time_ticks: u32,
    pub stall_omega_max_cps: u16,
    /// The continuous current: what the winding holds at its maximum
    /// temperature. The I2t budget folds the ceiling toward it, the press
    /// detector triggers over it and folds to it. 0 reads as the cap: no
    /// budget, the press at the cap.
    pub i_cont_counts: u16,
    /// The press fold lets go once the held current reads under this.
    pub stall_release_counts: u16,
    /// Collision trip: |tau_d| above this is model-unexplained torque;
    /// `u16::MAX` trips nothing.
    pub stall_tau_trip_counts: u16,
    pub oc_trip_counts: u16,
    pub oc_trip_ticks: u8,
    pub openloop_decay: DecayMode,
    /// true = Brake at zero commanded duty in OpenLoop (default coast).
    pub openloop_zero_brake: bool,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    /// The winding's thermal time constant: the I2t budget's leak. The
    /// kernel runs it at the nearest power of two of its 16 ms period
    /// (`kernel::phases::loops::limits::tau_shift`), 16 ms to 65.5 s.
    pub thermal_tau_ms: u16,
}

// Boot limits until the host supplies nameplate and thermal values.
// The maxon A-max 12 data sheet gives 59.5 K/W winding to ambient and
// a 5.03 s winding constant (source linked in control-theory). At a 60 K
// rise, an assumed 5 ohm winding would hold about 0.45 A in free air.
// The 250 mA default leaves unmeasured enclosure margin; the host must
// replace it with the actual motor's rating. The leak uses the octave
// below that reference constant, so a burst spends its budget sooner.
// Release stays below the fold so a held press does not unfold.
pub const DEFAULT_CURRENT_LIMIT_MA: u16 = 500;
pub const DEFAULT_I_CONT_MA: u16 = 250;
pub const DEFAULT_THERMAL_TAU_MS: u16 = 4096;
pub const DEFAULT_DRIVE_POLARITY: bool = true;
pub const DEFAULT_STALL_OMEGA_MAX_CPS: u16 = 500;
pub const DEFAULT_STALL_TIME_TICKS: u32 = ms_ticks(500);
pub const DEFAULT_STALL_RELEASE_MA: u16 = 90;
/// The collision trip boots off: identification sets it from the measured
/// tau_d noise floor. A copy of the limit tripped the first free run under
/// a low limit.
pub const STALL_TAU_TRIP_OFF: u16 = u16::MAX;
// Copper and shunt guard, not a gear-train limit: 2.2 A sits above any
// micro-class stall on 2S (2.1 A) and below where the osc-dev-v006 chain
// saturates (~2.3 A over its 1.10 V reference on 60 mOhm, the amplifier
// topping out at VDD - 0.16 V), so the trip stays measurable.
pub const DEFAULT_OC_TRIP_MA: u16 = 2200;
pub const DEFAULT_OC_TRIP_TICKS: u8 = 8;
// Applied volts (duty x rail) a shunt-burst step may drive. A burst runs
// past the current limit on purpose, so what bounds its load on the teeth is
// the impulse the rotor takes in one half window: class-absolute, not a
// multiple of the user's limit. Reaches the chip in vcounts through
// `vmotor_counts`.
pub const BURST_MAX_MV: u16 = 3200;

/// Thermal derate/cutoff on the winding estimate, undervolt floor. The
/// reserved word held the retired resistance thermometer's current gate.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigThermal {
    pub derate_start_cc: i16,
    #[ct_field(gt = &addr::thermal::RECOVER_CC)]
    pub cutoff_cc: i16,
    #[ct_field(lt = &addr::thermal::CUTOFF_CC)]
    pub recover_cc: i16,
    pub v_undervolt_counts: u16,
    #[ct_field(skip)]
    pub _rsvd: u16,
}

// 80C derate onset / 100C cutoff / 90C recover.
pub const DEFAULT_DERATE_START_CC: i16 = 8000;
pub const DEFAULT_CUTOFF_CC: i16 = 10000;
pub const DEFAULT_RECOVER_CC: i16 = 9000;
// True brown-out floor, not a battery-health line: the default must sit
// below every legal supply, and a USB-fed 5 V bench bus idles at ~4.3 V.
// Pack-health thresholds are per-product tuning, written by the host.
// Reaches the chip in vcounts through `vmotor_counts`.
pub const DEFAULT_V_UNDERVOLT_MV: u16 = 2900;

/// Fusion observer correction gains.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigFusion {
    pub l1_q016: u16,
    pub l2_q88: u16,
    pub l3_q88: u16,
}

// Observer corrections are not loop gains: at 0 the observer is an open
// integrator on the shunt current, theta_hat rails within a second of
// drive and the endstop band folds on the railed estimate, zeroing
// outbound open-loop duty (bench: recentre seeks crawled after a config
// image reset). The fusion.rs test set, stable against b_i up to ~8192;
// identification refines them from the fitted B.
pub const DEFAULT_L1_Q016: u16 = 16384;
pub const DEFAULT_L2_Q88: u16 = 1024;
pub const DEFAULT_L3_Q88: u16 = 128;

/// Fault thresholds: position-error window, sensor-delta screen, and the
/// rail fold ahead of the undervolt fault.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigFaultCfg {
    /// Kernel ticks (`CalibSense.tick_hz`).
    pub pos_error_time_ticks: u32,
    pub pos_error_counts: u16,
    pub sensor_delta_max: u16,
    pub sensor_bad_count: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    /// The rail fold: the ceiling is this many counts per vcount the rail
    /// sits over `v_undervolt_counts`, Q8, so it reaches the cap
    /// `cap / slope` vcounts above the floor and zero at the floor, where
    /// the fault latches, and a sagging source folds the torque instead of
    /// browning out. The slope is the source admittance a stall must not
    /// sag the rail through; 0 = off.
    pub rail_fold_q8: u16,
}

// The source a stall may sag the rail through: about 0.8 ohm, what a USB
// lead and its connectors add, so the 500 mA class cap folds over the
// 400 mV above the floor; a pack's floor and source are per-product
// tuning, written by the host. Reaches the chip through `rail_fold_q8`.
pub const DEFAULT_RAIL_SOURCE_MOHM: u32 = 800;

// Wide screens so healthy motion never trips; tighten per application.
pub const DEFAULT_POS_ERROR_COUNTS: u16 = 400;
pub const DEFAULT_POS_ERROR_TIME_TICKS: u32 = ms_ticks(500);
pub const DEFAULT_SENSOR_DELTA_MAX: u16 = 256;
pub const DEFAULT_SENSOR_BAD_COUNT: u8 = 4;

/// Config section: always writable (normal field validation applies), volatile
/// until `MGMT SAVE` persists it -- SAVE is the only torque-gated operation
/// (osc-native sec 9.4 separates write from persistence).
#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CONFIG_BASE_ADDR,
    size = crate::regions::CONFIG_REGION_SIZE,
    hooks = crate::regions::hooks::ControlTableHookEvents,
)]
pub struct ConfigRegs {
    pub common: ConfigCommon,
    pub pos_phys: ConfigPosPhys,
    pub pos_soft: ConfigPosSoft,
    pub loop_current: ConfigLoopCurrent,
    pub loop_velocity: ConfigLoopVelocity,
    pub loop_position: ConfigLoopPosition,
    pub limits: ConfigLimits,
    pub thermal: ConfigThermal,
    /// Held the retired resistance thermometer's seat rules.
    #[ct_section(skip)]
    pub _rsvd_thermal_ext: [u8; 4],
    pub fusion: ConfigFusion,
    pub fault_cfg: ConfigFaultCfg,
}

/// Boot-time seed for `ControlTable.config`; stamped pre-IRQ, then host-owned.
#[derive(Copy, Clone, Debug, Default)]
pub struct ConfigDefaults {
    pub pos_min_phys_counts: i32,
    pub pos_max_phys_counts: i32,
    pub id: u8,
    pub baud: BaudRate,
    pub response_deadline_us: u16,
}

/// Count-domain seeds for the current thresholds: the `DEFAULT_*_MA` policy
/// scaled through the board's sense chain (`CalibSense` inputs). Const-eval
/// only; the chip has no divide.
#[derive(Copy, Clone, Debug)]
pub struct CurrentDefaults {
    pub current_limit_counts: u16,
    pub i_cont_counts: u16,
    pub stall_release_counts: u16,
    pub oc_trip_counts: u16,
}

impl CurrentDefaults {
    pub const fn from_sense(shunt_r_mohm: u16, gain_milli: u16, vdd_mv: u16) -> Self {
        Self {
            current_limit_counts: current_counts(
                DEFAULT_CURRENT_LIMIT_MA,
                shunt_r_mohm,
                gain_milli,
                vdd_mv,
            ),
            i_cont_counts: current_counts(DEFAULT_I_CONT_MA, shunt_r_mohm, gain_milli, vdd_mv),
            stall_release_counts: current_counts(
                DEFAULT_STALL_RELEASE_MA,
                shunt_r_mohm,
                gain_milli,
                vdd_mv,
            ),
            oc_trip_counts: current_counts(DEFAULT_OC_TRIP_MA, shunt_r_mohm, gain_milli, vdd_mv),
        }
    }
}

const ADC_COUNTS_PER_VDD: u64 = 4096;

/// Kernel ticks in `ms` at `kernel::TICK_HZ`, the rate the chip asserts.
pub const fn ms_ticks(ms: u32) -> u32 {
    ms * (crate::kernel::TICK_HZ / 1000)
}

const _: () = assert!(crate::kernel::TICK_HZ.is_multiple_of(1000));

/// counts = mA x gain x R_shunt x 4096 / VDD, rounded: the chain puts
/// gain x R_shunt volts per amp onto a VDD-referenced 12-bit ADC: 614.4
/// counts/A on the arm-B rig (33 mOhm, G 15.000, 3300 mV), 1108.5 on the
/// osc-dev-v006 board (60 mOhm, G 14.884), where the 500 mA class limit is
/// 554.
pub const fn current_counts(ma: u16, shunt_r_mohm: u16, gain_milli: u16, vdd_mv: u16) -> u16 {
    let num = ma as u64 * gain_milli as u64 * shunt_r_mohm as u64 * ADC_COUNTS_PER_VDD;
    let den = vdd_mv as u64 * 1_000_000;
    let counts = (num + den / 2) / den;
    if counts > u16::MAX as u64 {
        u16::MAX
    } else {
        counts as u16
    }
}

/// The rail fold's slope for a source of `z_mohm`, counts per vcount in
/// Q8: the shunt chain's counts per amp over the terminal tap's vcounts
/// per volt, over the source. 1430 on the osc-dev-v006 board (1108.5
/// counts/A, 248.3 vcounts/V) at 800 mOhm. Const-eval only.
pub const fn rail_fold_q8(
    z_mohm: u32,
    shunt_r_mohm: u16,
    gain_milli: u16,
    vdd_mv: u16,
    div_top_ohm: u32,
    div_bot_ohm: u32,
) -> u16 {
    let counts_per_amp = current_counts(1000, shunt_r_mohm, gain_milli, vdd_mv) as u64;
    let vcounts_per_volt = vmotor_counts(1000, div_top_ohm, div_bot_ohm, vdd_mv) as u64;
    let den = vcounts_per_volt * z_mohm as u64;
    if den == 0 {
        return 0;
    }
    let num = counts_per_amp * 1000 * 256;
    let slope = (num + den / 2) / den;
    if slope > u16::MAX as u64 {
        u16::MAX
    } else {
        slope as u16
    }
}

/// vcounts = mV x 4096 x bot / ((top + bot) x VDD), rounded: the rail as the
/// motor-terminal tap reads it, the unit `vbus_counts` publishes. 794 for
/// the 3200 mV burst cap on the osc-dev-v006 6K4/1K6 taps (4.028 mV/count).
pub const fn vmotor_counts(mv: u16, div_top_ohm: u32, div_bot_ohm: u32, vdd_mv: u16) -> u16 {
    let num = mv as u64 * ADC_COUNTS_PER_VDD * div_bot_ohm as u64;
    let den = (div_top_ohm as u64 + div_bot_ohm as u64) * vdd_mv as u64;
    let counts = (num + den / 2) / den;
    if counts > u16::MAX as u64 {
        u16::MAX
    } else {
        counts as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn burst_cap_in_vcounts_follows_the_terminal_divider() {
        assert_eq!(vmotor_counts(BURST_MAX_MV, 6_400, 1_600, 3300), 794);
        assert_eq!(vmotor_counts(0, 6_400, 1_600, 3300), 0);
        assert_eq!(vmotor_counts(u16::MAX, 0, 1, 1), u16::MAX);
    }

    #[test]
    fn rail_fold_slope_follows_both_chains() {
        assert_eq!(
            rail_fold_q8(DEFAULT_RAIL_SOURCE_MOHM, 60, 14_884, 3300, 6_400, 1_600),
            1430
        );
        // 500 mA x 0.8 ohm: the cap, 554 counts, 99 vcounts (400 mV) over
        // the floor
        assert_eq!((554u32 << 8) / 1430, 99);
        assert_eq!(rail_fold_q8(0, 60, 14_884, 3300, 6_400, 1_600), 0);
        assert_eq!(rail_fold_q8(1, 60, 14_884, 3300, 6_400, 1_600), u16::MAX);
    }

    #[test]
    fn undervolt_floor_in_vcounts_follows_the_terminal_divider() {
        assert_eq!(
            vmotor_counts(DEFAULT_V_UNDERVOLT_MV, 6_400, 1_600, 3300),
            720
        );
        assert_eq!(
            vmotor_counts(DEFAULT_V_UNDERVOLT_MV, 20_000, 10_000, 3300),
            1200
        );
    }

    #[test]
    fn bias_nominal_is_vdd_through_the_bias_divider_at_any_vdd() {
        assert_eq!(vmotor_counts(3300, 430, 100, 3300), 773);
        assert_eq!(vmotor_counts(3280, 430, 100, 3280), 773);
    }

    #[test]
    fn arm_b_rig_reproduces_the_count_seeds() {
        let d = CurrentDefaults::from_sense(33, 15_000, 3300);
        assert_eq!(d.current_limit_counts, 307);
        assert_eq!(d.i_cont_counts, 154);
        assert_eq!(d.stall_release_counts, 55);
        assert_eq!(d.oc_trip_counts, 1352);
    }

    #[test]
    fn current_counts_scales_with_the_shunt_and_saturates() {
        assert_eq!(current_counts(500, 60, 14_884, 3300), 554);
        assert_eq!(current_counts(0, 60, 14_884, 3300), 0);
        assert_eq!(current_counts(u16::MAX, u16::MAX, u16::MAX, 1), u16::MAX);
    }
}
