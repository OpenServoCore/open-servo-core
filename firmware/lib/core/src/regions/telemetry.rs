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
    /// Velocity-loop feedback source behind `omega_hat_cps`: 0 = the pot
    /// observer's omega, 1 = the back-EMF boxcar (estimator::OmegaSource).
    #[ct_field(access = ro)]
    pub omega_hat_src: u8,
    /// Why closed loop is refused, one bit per reason (`data_state` module
    /// consts); 0 = the persisted images and the identified set are this
    /// servo's own. Written by boot, SAVE and the HIGH dispatcher's commits
    /// (a covered write marks STAMP_MISMATCH, a stamp write and a LUT COMMIT
    /// checkpoint), read by the kernel's entry check - the one
    /// TELEMETRY-MODE byte the kernel does not own.
    #[ct_field(access = ro)]
    pub data_flags: u8,
}

/// Estimator outputs, published at the medium boundary (`sample_tick` at
/// every fast tick). Counts-domain throughout; conversion is the host's job.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryEstimates {
    /// Fused position, cQ16 (pot counts x 2^16).
    #[ct_field(access = ro)]
    pub theta_hat_q16: i32,
    /// Velocity-loop feedback, raw csQ16 ((counts/s) x 2^16) - published
    /// unshifted; `omega_hat_src` names which estimate it is.
    #[ct_field(access = ro)]
    pub omega_hat_cps: i32,
    /// Disturbance-torque estimate, current counts.
    #[ct_field(access = ro)]
    pub tau_d_counts: i16,
    /// Effective current ceiling after limit folds.
    #[ct_field(access = ro)]
    pub i_lim_counts: u16,
    /// Winding temperature, centi-C; `i16::MIN` while the thermometer has
    /// no model or no NTC (`therm_flags` bit 0).
    #[ct_field(access = ro)]
    pub t_winding_cc: i16,
    #[ct_field(access = ro)]
    pub vbus_counts: u16,
    /// Post-clamp post-gate duty actually written to the bridge.
    #[ct_field(access = ro)]
    pub duty_applied_q15: i16,
    /// Back-EMF boxcar, whole c/s; 0 while a sub-floor window sits inside
    /// the boxcar (`omega_hat_src` then reads 0 within a millisecond).
    #[ct_field(access = ro)]
    pub omega_bemf_cps: i16,
    /// Winding-R LMS estimate, vcounts/ccount Q4.12.
    #[ct_field(access = ro)]
    pub r_hat_q12: u16,
    /// Window-selected, bias-subtracted, settle-gained, signed current
    /// sample (`window::i_from_frame`).
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
    /// Zero-current sense-chain output in use, raw ADC counts, driver
    /// awake: the boot rest measurement, then tracked from every settled
    /// tick the shunt carries no current - torque off, a brake or coast, a
    /// Slow-decay brake trough (`estimator::bias`).
    #[ct_field(access = ro)]
    pub current_bias_counts: u16,
    /// Direct supply divider tap, raw ADC counts (the rail `vbus_counts`
    /// derives from).
    #[ct_field(access = ro)]
    pub vbus_raw: u16,
    /// NTC divider tap, raw ADC counts.
    #[ct_field(access = ro)]
    pub ntc_raw: u16,
    /// Boot-measured motor-terminal divider bias (terminals high-impedance),
    /// raw ADC counts; the terminal taps read this with the rail at 0.
    #[ct_field(access = ro)]
    pub vmotor_bias_counts: u16,
}

/// Identification aggregates on their own fast-tick /16 window (mean =
/// sum>>4, arithmetic); `agg_seq` increments per window so the host pairs a
/// consistent set. Current fields are SIGNED bias-subtracted counts - the
/// fitter's domain. Ticks with an invalid window contribute the last valid
/// current/vdiff sample, not zero, while the drive pushes; with nothing
/// driving the current reads 0 (kernel/ident.rs doc). Published only while
/// CONTROL `ident_agg` is set; off, the block holds its last window.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryIdent {
    #[ct_field(access = ro)]
    pub i_mean_counts: i16,
    #[ct_field(access = ro)]
    pub i_min_counts: i16,
    #[ct_field(access = ro)]
    pub i_max_counts: i16,
    /// Drive-window `va - vb` differential mean.
    #[ct_field(access = ro)]
    pub vdiff_mean: i16,
    /// Mean of the per-tick commanded duty (always defined, 0 while off).
    #[ct_field(access = ro)]
    pub duty_mean_q15: i16,
    #[ct_field(access = ro)]
    pub agg_seq: u16,
}

/// Which limit governs the command, published at the medium boundary: one
/// bit per reason (`kernel::limits::flag`), 0 = nothing holds it back.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryLimits {
    #[ct_field(access = ro)]
    pub limit_flags: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    /// Smallest duty whose drive window the shunt reads
    /// (`window::floor_duty`), Q15: where the OpenLoop ceiling restarts and
    /// its blind band begins. Published at the kernel's first tick, then
    /// whenever a CONFIG or CALIB write reaches the kernel. 0 until the first
    /// tick.
    #[ct_field(access = ro)]
    pub window_floor_q15: u16,
}

/// The terminal taps' floor beside `TelemetryLimits::window_floor_q15`,
/// filling the reserved tail so a servo without it reads 0.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryLimitsExt {
    /// Smallest duty whose drive window the terminal taps read
    /// (`window::floor_duty` of `v_window_min_ticks`), Q15: a drive whose
    /// fit needs `va - vb` runs at or above the higher of the two floors.
    /// Published with `window_floor_q15`.
    #[ct_field(access = ro)]
    pub window_v_floor_q15: u16,
}

/// Servo health, written by the chip side (the tick interrupt, the main
/// loop), never by the kernel. The `rw` counters follow the
/// `TelemetryCommon` clear contract: the host writes zero to clear.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryHealth {
    /// Mean share of the kernel period the tick interrupt took over the
    /// last 4096 ticks (0.2 s at 20 kHz), Q15: 32768 = the whole period.
    #[ct_field(access = ro)]
    pub tick_load_mean_q15: u16,
    /// Kernel ticks whose interrupt took longer than one period, transport
    /// preemption included; updated every 16 ticks. A few per bus
    /// transaction are normal, hundreds per second mean the kernel overruns
    /// by itself. Wraps.
    #[ct_field(access = rw)]
    pub tick_over_count: u16,
    /// Kernel ticks that never ran: the previous tick was still running or
    /// interrupts were held off; updated every 16 ticks. Wraps.
    #[ct_field(access = rw)]
    pub tick_lost_count: u16,
    /// TEL stream rows dropped because both stream buffers were waiting for
    /// the wire. Wraps.
    #[ct_field(access = rw)]
    pub tel_drop_count: u16,
    /// Smallest free stack seen since boot, bytes.
    #[ct_field(access = ro)]
    pub stack_free_min: u16,
}

/// Winding thermometer state beside `t_winding_cc`: the board NTC as the
/// kernel converts it (centi-C, the carry's base) and the estimator's
/// flags (`estimator::thermal::flag`: unset, tracking a seat, cold-R
/// recalibrate, hot-boot floor).
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryTherm {
    #[ct_field(access = ro)]
    pub t_ntc_cc: i16,
    #[ct_field(access = ro)]
    pub therm_flags: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
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
    pub limits: TelemetryLimits,
    pub health: TelemetryHealth,
    pub limits_ext: TelemetryLimitsExt,
    pub therm: TelemetryTherm,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 6],
}
