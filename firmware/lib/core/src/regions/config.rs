use control_table::{Block, Enum, Section};

use crate::kernel::phases::loops::trajectory::{GOAL_LIM_COUNTS, VEL_CAP_CPS};
use crate::regions::calib::CalibSense;

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

/// An application's narrower travel, inside the physical stops and the
/// trajectory's goal range, so the kernel takes them as stored; or 0 / 0,
/// off: the kernel's take puts the physical stops in their place. Both
/// sides go in one write, since each is checked against the other as
/// stored.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigPosSoft {
    #[ct_field(
        within = (&addr::pos_phys::POS_MIN_PHYS_COUNTS, &addr::pos_phys::POS_MAX_PHYS_COUNTS),
        lt = &addr::pos_soft::POS_MAX_SOFT_COUNTS,
        within = (-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS),
        off_pair,
    )]
    pub pos_min_soft_counts: i32,
    #[ct_field(
        within = (&addr::pos_phys::POS_MIN_PHYS_COUNTS, &addr::pos_phys::POS_MAX_PHYS_COUNTS),
        gt = &addr::pos_soft::POS_MIN_SOFT_COUNTS,
        within = (-GOAL_LIM_COUNTS, GOAL_LIM_COUNTS),
        off_pair,
    )]
    pub pos_max_soft_counts: i32,
}

impl ConfigPosSoft {
    /// The travel these limits hold to: as stored, or `phys`, the physical
    /// stops, while they are 0 / 0, off.
    #[inline(always)]
    pub fn or_stops(self, phys: &ConfigPosPhys) -> Self {
        if self.pos_min_soft_counts | self.pos_max_soft_counts == 0 {
            Self {
                pos_min_soft_counts: phys.pos_min_phys_counts,
                pos_max_soft_counts: phys.pos_max_phys_counts,
            }
        } else {
            self
        }
    }
}

// `off_pair` finds each side's partner at `addr ^ 4`.
const _: () = assert!(
    addr::pos_soft::POS_MIN_SOFT_COUNTS.is_multiple_of(8)
        && addr::pos_soft::POS_MAX_SOFT_COUNTS == addr::pos_soft::POS_MIN_SOFT_COUNTS + 4
);

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

/// Position P gain, anti-hunt hold deadband, the speed ceiling, the move
/// speed and the profile's acceleration.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLoopPosition {
    pub p_kp_q88: u16,
    /// Raw counts of the position sensor, scaled by the position table's
    /// local gain into the linearized error. 0 = hold disabled. Hold brakes
    /// on position rest a shaft whose load estimate sits within
    /// `fric_fc_counts`, and keeps a loaded one's current with its velocity
    /// integral frozen (position.rs).
    pub pos_deadband_counts: u16,
    /// The hard speed limit calibration writes: the position loop's output
    /// and the Velocity goal clamp at it, and Current mode's band folds
    /// under it. 0 = unset: every closed mode is refused. It sits
    /// ahead of the move speed, so an install in table order lands it
    /// before the rule that reads it.
    #[ct_field(le = VEL_CAP_CPS)]
    pub speed_ceiling_cps: u16,
    /// The move speed, the Position profile's cruise; 0 = the ceiling.
    #[ct_field(le = &addr::loop_position::SPEED_CEILING_CPS)]
    pub velocity_limit_cps: u16,
    /// c/s per medium tick.
    pub accel_limit_q88: u16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
}

// The profile's acceleration is a clamp, not a gain: at 0 the profile never
// leaves rest, so it gets a real default (bench-validated on the SG90 rig:
// 15 c/s per medium tick, 0 -> 1500 c/s in 50 ms).
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
    /// train is rated to deliver. Nothing composes over it. 0 = unset:
    /// every closed mode is refused, and OpenLoop runs under
    /// `oc_trip_counts`.
    pub current_limit_counts: u16,
    pub stall_response: StallResponse,
    /// false = the wiring is reversed: the kernel negates its motor output so
    /// positive duty and current still increase position counts.
    pub drive_polarity: bool,
    /// Kernel ticks (`CalibSense.tick_hz`).
    pub stall_time_ticks: u32,
    pub stall_omega_max_cps: u16,
    /// The continuous current: what the winding holds at its maximum
    /// temperature. The press detector triggers over it and folds to it. 0
    /// reads as the cap: the press at the cap.
    pub i_cont_counts: u16,
    /// The press fold lets go once the held current reads under this.
    pub stall_release_counts: u16,
    /// Collision trip: |tau_d| above this is model-unexplained torque;
    /// `u16::MAX` trips nothing.
    pub stall_tau_trip_counts: u16,
    pub oc_trip_counts: u16,
    pub oc_trip_ticks: u8,
    /// true = Brake at zero commanded duty in OpenLoop (default coast).
    pub openloop_zero_brake: bool,
    /// The rotor heat node's time constant in seconds. The kernel runs it
    /// at the nearest power of two of its 16 ms period
    /// (`kernel::phases::loops::limits::tau_shift`), capped at the longest
    /// leak, 524 s; 0 runs one period (16 ms) and 1 s runs 64 (1.02 s).
    pub thermal_tau_s: u16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
}

// The press detector's rest speed and dwell, in the kernel's position
// counts and ticks: no motor or supply fact, so the core's.
pub const DEFAULT_STALL_OMEGA_MAX_CPS: u16 = 500;
pub const DEFAULT_STALL_TIME_TICKS: u32 = ms_ticks(500);
/// The collision trip boots off: identification sets it from the measured
/// tau_d noise floor. A copy of the limit tripped the first free run under
/// a low limit.
pub const STALL_TAU_TRIP_OFF: u16 = u16::MAX;
// Applied volts (duty x rail) a shunt-burst step may drive. A burst runs
// past the current limit on purpose, so what bounds its load on the teeth is
// the impulse the rotor takes in one half window: class-absolute, not a
// multiple of the user's limit. Reaches the chip in vcounts through
// `vmotor_counts`.
pub const BURST_MAX_MV: u16 = 3200;

/// The winding estimate's cutoff, the undervolt floor, and the body
/// node's leak: `thermal_tau2_s` in seconds to the nearest octave of the
/// SLOW period like `thermal_tau_s`, 0 = no body node.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigThermal {
    /// The one thermal limit: the overtemperature fault latches while the
    /// winding estimate reads this or more, and the host's ack relatches
    /// until it reads less.
    pub cutoff_cc: i16,
    pub v_undervolt_counts: u16,
    pub thermal_tau2_s: u16,
}

/// Fusion observer correction gains, and the position reading's two
/// current terms ahead of it (`estimator::pot_current`): tuning parameters
/// for the load at hand, in linearized pot counts per amp of the current
/// reference, 0 = off.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigFusion {
    pub l1_q016: u16,
    pub l2_q88: u16,
    pub l3_q88: u16,
    /// Credits a steady push: counts per amp of the current's slow average.
    #[ct_field(le = POT_COUNTS_PER_A_MAX)]
    pub pot_steady_counts_per_a: u16,
    /// Follows a current change: counts per amp of the fast average's lead
    /// over the slow one.
    #[ct_field(le = POT_COUNTS_PER_A_MAX)]
    pub pot_fast_counts_per_a: u16,
}

/// The bound on both current terms: the observer's arithmetic stays one
/// i32 under it (`estimator::pot_current`).
pub const POT_COUNTS_PER_A_MAX: u16 = 2000;

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

/// A board's protection defaults in physical units: what its motor, its
/// supply and its sense chain stand, seeded into CONFIG at boot and
/// host-owned after. Plain data with no core fallback, so every board
/// states each field; `ProtectionSeed` converts them.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ProtectionDefaults {
    /// The continuous current, `i_cont_counts`.
    pub i_cont_ma: u16,
    /// `stall_release_counts`, under `i_cont_ma` so a held press stays
    /// folded.
    pub stall_release_ma: u16,
    pub oc_trip_ma: u16,
    /// The overcurrent trip window, `oc_trip_ticks`.
    pub oc_trip_us: u16,
    pub thermal_tau_s: u16,
    /// Deg C x100.
    pub cutoff_cc: i16,
    pub v_undervolt_mv: u16,
    /// The source a stall may sag the rail through, `rail_fold_q8`.
    pub rail_source_mohm: u32,
    /// false = the motor leads are wired reversed.
    pub drive_polarity: bool,
}

/// `ProtectionDefaults` in the table's units, through the board's shunt
/// chain and motor-terminal divider. Const-eval only; the chip has no
/// divide.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ProtectionSeed {
    pub i_cont_counts: u16,
    pub stall_release_counts: u16,
    pub oc_trip_counts: u16,
    pub oc_trip_ticks: u8,
    pub thermal_tau_s: u16,
    pub cutoff_cc: i16,
    pub v_undervolt_counts: u16,
    pub rail_fold_q8: u16,
    pub drive_polarity: bool,
}

impl ProtectionSeed {
    pub const fn from_chain(
        p: &ProtectionDefaults,
        shunt_r_mohm: u16,
        gain_milli: u16,
        vdd_mv: u16,
        div_top_ohm: u32,
        div_bot_ohm: u32,
    ) -> Self {
        Self {
            i_cont_counts: current_counts(p.i_cont_ma, shunt_r_mohm, gain_milli, vdd_mv),
            stall_release_counts: current_counts(
                p.stall_release_ma,
                shunt_r_mohm,
                gain_milli,
                vdd_mv,
            ),
            oc_trip_counts: current_counts(p.oc_trip_ma, shunt_r_mohm, gain_milli, vdd_mv),
            oc_trip_ticks: us_ticks_u8(p.oc_trip_us),
            thermal_tau_s: p.thermal_tau_s,
            cutoff_cc: p.cutoff_cc,
            v_undervolt_counts: vmotor_counts(p.v_undervolt_mv, div_top_ohm, div_bot_ohm, vdd_mv),
            rail_fold_q8: rail_fold_q8(
                p.rail_source_mohm,
                shunt_r_mohm,
                gain_milli,
                vdd_mv,
                div_top_ohm,
                div_bot_ohm,
            ),
            drive_polarity: p.drive_polarity,
        }
    }

    /// The same through the chains as CALIB carries them.
    pub const fn from_sense(p: &ProtectionDefaults, sense: &CalibSense) -> Self {
        Self::from_chain(
            p,
            sense.shunt_r_mohm,
            sense.gain_milli,
            sense.vdd_mv,
            sense.vmotor_div_top as u32,
            sense.vmotor_div_bot as u32,
        )
    }
}

/// The osc-dev-v006 board's values, for the core's own tests.
#[cfg(test)]
pub(crate) const TEST_PROTECTION: ProtectionDefaults = ProtectionDefaults {
    i_cont_ma: 250,
    stall_release_ma: 90,
    oc_trip_ma: 2200,
    oc_trip_us: 400,
    thermal_tau_s: 4,
    cutoff_cc: 9000,
    v_undervolt_mv: 2900,
    rail_source_mohm: 800,
    drive_polarity: true,
};

const ADC_COUNTS_PER_VDD: u64 = 4096;

/// Kernel ticks in `ms` at `kernel::TICK_HZ`, the rate the chip asserts.
pub const fn ms_ticks(ms: u32) -> u32 {
    ms * (crate::kernel::TICK_HZ / 1000)
}

const _: () = assert!(crate::kernel::TICK_HZ.is_multiple_of(1000));

/// Kernel ticks in `us`, rounded, saturating at the u8 field.
pub const fn us_ticks_u8(us: u16) -> u8 {
    let ticks = (us as u64 * crate::kernel::TICK_HZ as u64 + 500_000) / 1_000_000;
    if ticks > u8::MAX as u64 {
        u8::MAX
    } else {
        ticks as u8
    }
}

/// counts = mA x gain x R_shunt x 4096 / VDD, rounded: the chain puts
/// gain x R_shunt volts per amp onto a VDD-referenced 12-bit ADC: 614.4
/// counts/A on the arm-B rig (33 mOhm, G 15.000, 3300 mV), 1108.5 on the
/// osc-dev-v006 board (60 mOhm, G 14.884), where 500 mA is 554.
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
        assert_eq!(rail_fold_q8(800, 60, 14_884, 3300, 6_400, 1_600), 1430);
        // 500 mA x 0.8 ohm: 554 counts, 99 vcounts (400 mV) over the floor
        assert_eq!((554u32 << 8) / 1430, 99);
        assert_eq!(rail_fold_q8(0, 60, 14_884, 3300, 6_400, 1_600), 0);
        assert_eq!(rail_fold_q8(1, 60, 14_884, 3300, 6_400, 1_600), u16::MAX);
    }

    #[test]
    fn undervolt_floor_in_vcounts_follows_the_terminal_divider() {
        assert_eq!(vmotor_counts(2900, 6_400, 1_600, 3300), 720);
        assert_eq!(vmotor_counts(2900, 20_000, 10_000, 3300), 1200);
    }

    #[test]
    fn bias_nominal_is_vdd_through_the_bias_divider_at_any_vdd() {
        assert_eq!(vmotor_counts(3300, 430, 100, 3300), 773);
        assert_eq!(vmotor_counts(3280, 430, 100, 3280), 773);
    }

    #[test]
    fn arm_b_rig_reproduces_the_count_seeds() {
        let d = ProtectionSeed::from_chain(&TEST_PROTECTION, 33, 15_000, 3300, 20_000, 10_000);
        assert_eq!(d.i_cont_counts, 154);
        assert_eq!(d.stall_release_counts, 55);
        assert_eq!(d.oc_trip_counts, 1352);
        assert_eq!(d.v_undervolt_counts, 1200);
    }

    #[test]
    fn trip_window_rounds_to_ticks_and_saturates() {
        assert_eq!(us_ticks_u8(400), 8);
        assert_eq!(us_ticks_u8(0), 0);
        assert_eq!(us_ticks_u8(74), 1);
        assert_eq!(us_ticks_u8(u16::MAX), u8::MAX);
    }

    /// The soft limits as one write over stops at 100..=3000: 0 / 0 is off
    /// and taken; every other pair, one side 0 among them, as before.
    #[test]
    fn soft_limits_take_0_0_as_off_and_every_other_pair_by_the_rules() {
        use control_table::{Error, RegionStorage, RegisterFile, ValidationKind};
        let t = crate::ControlTableCell::new();
        t.with_mut(|t| {
            t.config.pos_phys.pos_min_phys_counts = 100;
            t.config.pos_phys.pos_max_phys_counts = 3000;
        });
        let write = |lo: i32, hi: i32| {
            let mut b = [0u8; 8];
            b[..4].copy_from_slice(&lo.to_le_bytes());
            b[4..].copy_from_slice(&hi.to_le_bytes());
            t.write(addr::pos_soft::POS_MIN_SOFT_COUNTS, &b)
        };
        let refused = Err(Error::ValidationError(ValidationKind::Compare));
        assert_eq!(write(0, 0), Ok(()), "off");
        assert_eq!(write(0, 500), refused, "0 is under the low stop");
        assert_eq!(write(500, 0), refused);
        assert_eq!(write(200, 2800), Ok(()));
        assert_eq!(write(2800, 200), refused);
        assert_eq!(write(50, 2800), refused);
        assert_eq!(write(200, 3100), refused);
        // stops around 0: (0, 500) is a narrower travel and passes as before
        t.with_mut(|t| t.config.pos_phys.pos_min_phys_counts = -100);
        assert_eq!(write(0, 500), Ok(()));
        assert_eq!(write(-50, 0), Ok(()));
        assert_eq!(write(0, 0), Ok(()), "off");
        // one side to 0 alone is checked against the stored other side
        assert_eq!(write(200, 2800), Ok(()));
        assert_eq!(
            t.write(addr::pos_soft::POS_MAX_SOFT_COUNTS, &0i32.to_le_bytes()),
            refused
        );
    }

    #[test]
    fn current_counts_scales_with_the_shunt_and_saturates() {
        assert_eq!(current_counts(500, 60, 14_884, 3300), 554);
        assert_eq!(current_counts(0, 60, 14_884, 3300), 0);
        assert_eq!(current_counts(u16::MAX, u16::MAX, u16::MAX, 1), u16::MAX);
    }
}
