use control_table::{Block, Section};

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

/// Estimator outputs, published at the medium boundary (`sample_tick` at
/// every fast tick). Counts-domain throughout; conversion is the host's job.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryEstimates {
    /// Fused position, cQ16 (pot counts x 2^16).
    #[ct_field(access = ro)]
    pub theta_hat_q16: i32,
    /// Fused velocity, raw csQ16 ((counts/s) x 2^16) -- published unshifted.
    #[ct_field(access = ro)]
    pub omega_hat_cps: i32,
    /// Disturbance-torque estimate, current counts.
    #[ct_field(access = ro)]
    pub tau_d_counts: i16,
    /// Effective current ceiling after limit folds.
    #[ct_field(access = ro)]
    pub i_lim_counts: u16,
    /// Winding temperature, centi-C.
    #[ct_field(access = ro)]
    pub t_winding_cc: i16,
    #[ct_field(access = ro)]
    pub vbus_counts: u16,
    /// Post-clamp post-gate duty actually written to the bridge.
    #[ct_field(access = ro)]
    pub duty_applied_q15: i16,
    /// Telemetry-only bemf observer, whole c/s.
    #[ct_field(access = ro)]
    pub omega_bemf_cps: i16,
    /// Winding-R LMS estimate, vcounts/ccount Q4.12.
    #[ct_field(access = ro)]
    pub r_hat_q12: u16,
    /// Window-selected, bias-subtracted, signed current sample.
    #[ct_field(access = ro)]
    pub i_hat_counts: i16,
    #[ct_field(access = ro)]
    pub sample_tick: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetrySensors {
    #[ct_field(access = ro)]
    pub pos: u16,
    #[ct_field(access = ro)]
    pub current: u16,
    #[ct_field(access = ro)]
    pub vcal: u16,
    #[ct_field(access = ro)]
    pub vcal_lpf: u16,
    #[ct_field(access = ro)]
    pub vmotor_a: u16,
    #[ct_field(access = ro)]
    pub vmotor_b: u16,
    #[ct_field(access = ro)]
    pub enc_a: u16,
    #[ct_field(access = ro)]
    pub enc_b: u16,
    #[ct_field(access = ro)]
    pub current_trough: u16,
    /// Boot-measured zero-current sense-chain output, raw ADC counts.
    #[ct_field(access = ro)]
    pub current_bias_counts: u16,
}

/// Identification aggregates on their own fast-tick /16 window (mean =
/// sum>>4); `agg_seq` increments per window so the host pairs a consistent
/// set.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryIdent {
    #[ct_field(access = ro)]
    pub i_mean_counts: u16,
    #[ct_field(access = ro)]
    pub i_min_counts: u16,
    #[ct_field(access = ro)]
    pub i_max_counts: u16,
    #[ct_field(access = ro)]
    pub vdiff_mean: i16,
    #[ct_field(access = ro)]
    pub duty_mean_q15: i16,
    #[ct_field(access = ro)]
    pub agg_seq: u16,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(base = crate::regions::TELEMETRY_BASE_ADDR, size = crate::regions::TELEMETRY_REGION_SIZE)]
pub struct TelemetryRegs {
    pub common: TelemetryCommon,
    pub mode: TelemetryMode,
    pub estimates: TelemetryEstimates,
    pub sensors: TelemetrySensors,
    pub ident: TelemetryIdent,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 32],
}
