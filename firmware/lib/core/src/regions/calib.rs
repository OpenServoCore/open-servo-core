use control_table::{Block, Section};

/// The pot's mechanical stops in raw ADC counts: the ends of the angle map
/// (`CalibKinematics`) and the domain a position table is validated against.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibPot {
    pub raw_min: u16,
    pub raw_max: u16,
}

/// Sense-chain primitives the host converts raw counts with (protocol sec
/// 5.5): divider legs in ohms, amplifier gain x1000. `vdd_mv` is the
/// host-measured VDD at the chip pin -- the v006 ADC reference is VDD itself.
/// `tick_hz` is stamped at install from the same chip constant that programs
/// TIM1; the window floors are board data in TIM1 ticks.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibSense {
    #[ct_field(access = ro)]
    pub shunt_r_mohm: u16,
    #[ct_field(access = ro)]
    pub gain_milli: u16,
    #[ct_field(access = ro)]
    pub vmotor_div_top: u16,
    #[ct_field(access = ro)]
    pub vmotor_div_bot: u16,
    pub vdd_mv: u16,
    #[ct_field(access = ro)]
    pub tick_hz: u16,
    #[ct_field(access = ro)]
    pub i_window_min_ticks: u16,
    #[ct_field(access = ro)]
    pub v_window_min_ticks: u16,
}

/// The winding's cold resistance, firmware-shaped: `r0_q12` is the kernel's
/// own R (vcounts per ccount, Q4.12) at a seated hold, reduced to 25.00 C
/// (`estimator::thermal::R_COLD_REF_CC`) through copper and the board NTC.
/// The hot-reboot floor, the cold-R health check and the excess bound read
/// it; 0 = no thermometer.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibWinding {
    pub r0_q12: u16,
}

/// Motor identification, firmware-shaped: bemf subtract R (Q4.12),
/// reciprocal Ke (c/s per vcount Q6.10, estimator::bemf convention), fusion
/// current gain (bakes Ts/J, Q3.13: rig B runs ~3.4 so Q0.16 saturated), the
/// friction model (Coulomb ccounts, viscous Q0.16), and forward Ke (vcounts
/// per c/s Q4.12, the current loop's bemf decoupling feedforward). Forward
/// and reciprocal Ke are both host-written: the chip has no divide to derive
/// one from the other. SG90 scale ~0.28 vcounts per c/s (~1150 stored); the
/// u16 caps at 16, 57x headroom.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibMotor {
    pub r_q12: u16,
    pub recip_ke_q: u16,
    pub b_i_q313: u16,
    pub fric_fc_counts: u16,
    pub fric_fv_q016: u16,
    pub ke_vpc_q: u16,
}

/// The identification record firmware never reads: Ke as the host states
/// it and the breakaway friction, ccounts.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibMotorExt {
    pub ke_uvs_per_rad: u16,
    pub fric_breakaway_counts: u16,
}

/// Count<->angle scale and gearing, pure host-facing metadata: firmware never
/// reads it, the ISR stays in counts. Angles are centi-degrees at the pot
/// stops raw_min/raw_max (which equal pos_min/max_phys); `gear_ratio_centi`
/// is motor revs per output rev x100. All-zero = unset.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibKinematics {
    pub angle_min_cdeg: i16,
    pub angle_max_cdeg: i16,
    pub gear_ratio_centi: u16,
}

/// Supply-sense and thermistor board data (protocol sec 5.5): the direct
/// rail divider legs, the NTC pull-up / R25 / beta, the nominal
/// motor-terminal divider bias (the boot-measured value publishes in
/// telemetry), and the VDD divider that makes that bias: terminal leg
/// current moves the bias node under drive, and a host reading one terminal
/// absolutely corrects for it from these legs. RO: install re-stamps every
/// board fact after the overlay.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibSenseExt {
    #[ct_field(access = ro)]
    pub vbus_div_top_ohm: u16,
    #[ct_field(access = ro)]
    pub vbus_div_bot_ohm: u16,
    #[ct_field(access = ro)]
    pub ntc_pullup_ohm: u16,
    #[ct_field(access = ro)]
    pub ntc_r25_ohm: u16,
    #[ct_field(access = ro)]
    pub ntc_beta: u16,
    #[ct_field(access = ro)]
    pub vmotor_bias_nom_counts: u16,
    /// Supply connector -> VSYS drop across the input Schottky at rest, mV,
    /// so a host can estimate the pack voltage from `vbus_raw`.
    #[ct_field(access = ro)]
    pub rail_drop_mv: u16,
    #[ct_field(access = ro)]
    pub vmotor_bias_top_ohm: u16,
    #[ct_field(access = ro)]
    pub vmotor_bias_bot_ohm: u16,
}

/// Winding thermometer model, host-written per motor family (MG90 on the
/// dev board in open air: tau 128 s, 63 C/W over the board NTC): `th_alpha_q24`
/// the carry step per SLOW tick (dt/tau, Q0.24), `th_g_q016` the steady
/// winding excess over the NTC per vcount-ccount of `i (v - Ke omega)`
/// (centi-C, Q0.16), `th_mu_q016` the same-seat LMS step; then the board
/// NTC's counts-to-centi-C quadratic about its reference point, derived by
/// the host from `CalibSenseExt`'s beta constants (`estimator::thermal::
/// NtcCfg`). Zero alpha, g, k1 or r0 = no thermometer (`t_winding_cc` reads
/// its sentinel, nothing derates).
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibThermal {
    pub th_alpha_q24: u16,
    pub th_g_q016: u16,
    pub th_mu_q016: u16,
    pub ntc_raw_ref: u16,
    pub ntc_t_ref_cc: i16,
    pub ntc_k1_q88: i16,
    pub ntc_k2_q24: u16,
}

/// TIM1's period in compare counts, stamped at install from the constant
/// that programs it: the full scale of TEL `duty` (sec 5.6) and of the
/// window floors.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibPwm {
    #[ct_field(access = ro)]
    pub pwm_period_ticks: u16,
}

/// The plant stamp (`stamp` module): the host's CRC over the identified and
/// calibrated set plus the effective position table, 0 = never stamped.
/// Firmware recomputes it at every checkpoint; a mismatch is
/// `STAMP_MISMATCH`. Sits right after the host-written blocks so one WRITE
/// can carry the whole identified set and its stamp.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibStamp {
    pub plant_stamp: u16,
}

/// Calibration section: always writable (normal field validation applies),
/// volatile until persisted -- persistence is SAVE's job, not a write gate.
#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CALIB_BASE_ADDR,
    size = crate::regions::CALIB_REGION_SIZE,
)]
pub struct CalibRegs {
    pub motor: CalibMotor,
    pub winding: CalibWinding,
    pub thermal: CalibThermal,
    pub motor_ext: CalibMotorExt,
    pub pot: CalibPot,
    pub kinematics: CalibKinematics,
    pub stamp: CalibStamp,
    pub sense: CalibSense,
    pub sense_ext: CalibSenseExt,
    pub pwm: CalibPwm,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 176],
}
