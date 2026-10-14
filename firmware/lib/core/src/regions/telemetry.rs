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
    /// servo's own. Written by boot, SAVE and the bus dispatcher's commits
    /// (a covered write marks STAMP_MISMATCH, a stamp write and a LUT COMMIT
    /// checkpoint), read by the kernel's entry check - the one
    /// TELEMETRY-MODE byte the kernel does not own.
    #[ct_field(access = ro)]
    pub data_flags: u8,
}

/// One control tick (protocol sec 5.6). The chip's ADC DMA lands both scans
/// of each PWM period at the front: the trough scan and the crest scan's
/// shunt and taps in `_scan`, the crest's `pos` to `ntc_raw` on their
/// fields. The DMA writes the taps in sample order, so no published name
/// lies in its way: the tick ISR copies the crest's shunt and the trough's
/// out, writes each tap under its terminal's name and stamps `sample_tick`;
/// the kernel writes its conclusions on the window the scan measured, and
/// the TEL row is copied. The next scans land mid-tick and late in it
/// (`budget::NEXT_TROUGH`, `budget::NEXT_CREST`), after every read of this
/// one, so inside the tick every field is this tick's. Counts-domain
/// throughout.
#[repr(C)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Block)]
pub struct TelemetryTick {
    #[ct_field(skip)]
    pub _scan: [u16; 10],
    #[ct_field(access = ro)]
    pub pos: u16,
    #[ct_field(access = ro)]
    pub vcal: u16,
    /// Direct supply divider tap (the rail `vbus_counts` derives from).
    #[ct_field(access = ro)]
    pub vbus_raw: u16,
    /// NTC divider tap.
    #[ct_field(access = ro)]
    pub ntc_raw: u16,
    /// The shunt at the counter peak.
    #[ct_field(access = ro)]
    pub current_raw: u16,
    #[ct_field(access = ro)]
    pub vmotor_a: u16,
    #[ct_field(access = ro)]
    pub vmotor_b: u16,
    /// The PWM compare width the window measured was driven at, negative
    /// in reverse; full scale is CALIB `pwm_period_ticks`.
    #[ct_field(access = ro)]
    pub duty_ticks: i16,
    #[ct_field(access = ro)]
    pub current_trough: u16,
    /// Window-selected, bias-subtracted, settle-gained, signed current
    /// (`window::i_from_frame`), held through windows the shunt cannot read,
    /// 0 from the first tick nothing drives.
    #[ct_field(access = ro)]
    pub i_tick_counts: i16,
    /// Drive-window `va - vb`, logical sign, held through windows the
    /// terminal taps cannot read.
    #[ct_field(access = ro)]
    pub vdiff_counts: i16,
    #[ct_field(access = ro)]
    pub vmotor_a_trough: u16,
    #[ct_field(access = ro)]
    pub vmotor_b_trough: u16,
    /// [`tick_flag`] bits: the shunt window was read, a fault is
    /// latched, the terminal window was read.
    #[ct_field(access = ro)]
    pub tick_flags: u16,
    #[ct_field(access = ro)]
    pub sample_tick: u32,
}

/// `TelemetryTick::tick_flags` bits, written every tick.
pub mod tick_flag {
    /// The shunt read the window `duty_ticks` drove: `i_tick_counts` is
    /// fresh, not held.
    pub const I_VALID: u16 = 1 << 0;
    /// A fault is latched.
    pub const FAULT: u16 = 1 << 1;
    /// The terminal taps read the window: `vdiff_counts` is fresh.
    pub const V_VALID: u16 = 1 << 2;
}

/// Estimator outputs, published at the medium boundary. Counts-domain
/// throughout; conversion is the host's job.
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
    /// Back-EMF boxcar, whole c/s; 0 while a sub-floor window sits inside
    /// the boxcar (`omega_hat_src` then reads 0 within a millisecond).
    #[ct_field(access = ro)]
    pub omega_bemf_cps: i16,
    /// Winding-R LMS estimate, vcounts/ccount Q4.12.
    #[ct_field(access = ro)]
    pub r_hat_q12: u16,
}

/// The references the raw frame is read against.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryRefs {
    /// Zero-current sense-chain output in use, raw ADC counts, driver
    /// awake: the boot rest measurement, then tracked from every settled
    /// tick the shunt carries no current - torque off, a brake or coast, a
    /// Slow-decay brake trough (`estimator::bias`). Published at the medium
    /// boundary.
    #[ct_field(access = ro)]
    pub current_bias_counts: u16,
    /// Boot-measured motor-terminal divider bias (terminals high-impedance),
    /// raw ADC counts; the terminal taps read this with the rail at 0.
    #[ct_field(access = ro)]
    pub vmotor_bias_counts: u16,
    #[ct_field(access = ro)]
    pub enc_a: u16,
    #[ct_field(access = ro)]
    pub enc_b: u16,
}

/// Which limit governs the command, published at the medium boundary: one
/// bit per reason (`kernel::phases::loops::limits::flag`), 0 = nothing holds it back.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryLimits {
    #[ct_field(access = ro)]
    pub limit_flags: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    /// Smallest duty whose drive window the shunt reads
    /// (`window::floor_duty`), Q15: where the OpenLoop ceiling restarts and
    /// its blind band begins. Published once, at the kernel's first tick:
    /// its input is read-only. 0 until then.
    #[ct_field(access = ro)]
    pub window_floor_q15: u16,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(base = crate::regions::TELEMETRY_BASE_ADDR, size = crate::regions::TELEMETRY_REGION_SIZE)]
pub struct TelemetryRegs {
    pub common: TelemetryCommon,
    pub mode: TelemetryMode,
    pub tick: TelemetryTick,
    pub estimates: TelemetryEstimates,
    pub refs: TelemetryRefs,
    #[ct_section(skip)]
    pub _rsvd_ident: [u8; 8],
    pub limits: TelemetryLimits,
}
