use control_table::{Block, Section};

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryConverted {
    #[ct_field(access = ro)]
    pub present_position: i32,
    #[ct_field(access = ro)]
    pub present_velocity: i32,
    #[ct_field(access = ro)]
    pub present_current: i16,
    #[ct_field(access = ro)]
    pub present_temp: i16,
    #[ct_field(access = ro)]
    pub present_vbus_mv: u16,
    #[ct_field(skip)]
    pub _rsvd_tail: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryIntermediaries {
    #[ct_field(access = ro)]
    pub vbus_filt_mv: u16,
    #[ct_field(access = ro)]
    pub t_winding_dc: i16,
    #[ct_field(access = ro)]
    pub pwm_duty_actual: i16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
    #[ct_field(access = ro)]
    pub pid_error_last: i32,
    #[ct_field(access = ro)]
    pub pid_output_last: i32,
    #[ct_field(access = ro)]
    pub internal_goal: i32,
    #[ct_field(access = ro)]
    pub sample_tick: u32,
}

/// protocol sec 5.4 TELEMETRY-COMMON block, at the region front. Counters:
/// host can Write zero to clear (bench instrumentation); the chip publishes
/// deltas via raw pointer, bypassing the regmap, so a concurrent host clear +
/// publish may drop one update -- acceptable for bench. `trim_steps`
/// (sec 9.3) is the trim loop's applied total in signed chip trim steps from
/// the factory default (positive = slowed), volatile by design -- every boot
/// re-converges from live traffic.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryCommon {
    /// The sec 5.3 alarm register -- ALERT's read target.
    #[ct_field(access = ro)]
    pub fault_flags: u8,
    /// Bit 0 = config-dirty, modified-since-save (sec 9.4): set when a
    /// committed write lands in CONFIG or PROFILE, cleared by a successful
    /// SAVE; boot state is clean. Bits 1-7 reserved.
    #[ct_field(access = ro)]
    pub status_flags: u8,
    #[ct_field(access = ro)]
    pub trim_steps: i8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    #[ct_field(access = rw)]
    pub crc_fail_count: u32,
    #[ct_field(access = rw)]
    pub framing_drop_count: u32,
    #[ct_field(skip)]
    pub _rsvd_tail: [u8; 20],
}

/// Model-specific mode + fault detail: sec 5.4 keeps only the alarm byte
/// common; which faults exist and what the codes mean vary by node.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryMode {
    #[ct_field(access = ro)]
    pub mode_active: u8,
    #[ct_field(access = ro)]
    pub fault_code: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryRaw {
    #[ct_field(access = ro)]
    pub raw_pos: u16,
    #[ct_field(access = ro)]
    pub raw_current: u16,
    #[ct_field(access = ro)]
    pub raw_temp: u16,
    #[ct_field(access = ro)]
    pub raw_vbus: u16,
    #[ct_field(access = ro)]
    pub raw_vmotor_a: u16,
    #[ct_field(access = ro)]
    pub raw_vmotor_b: u16,
    #[ct_field(access = ro)]
    pub raw_enc_a: u16,
    #[ct_field(access = ro)]
    pub raw_enc_b: u16,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(base = crate::regions::TELEMETRY_BASE_ADDR, size = crate::regions::TELEMETRY_REGION_SIZE)]
pub struct TelemetryRegs {
    pub common: TelemetryCommon,
    pub mode: TelemetryMode,
    pub converted: TelemetryConverted,
    pub intermediaries: TelemetryIntermediaries,
    pub raw: TelemetryRaw,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 36],
}
