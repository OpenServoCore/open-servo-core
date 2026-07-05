use control_table::{FlatBlock, Section};

#[repr(C)]
#[derive(Copy, Clone, FlatBlock)]
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
#[derive(Copy, Clone, FlatBlock)]
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

/// `fault_flags` writable-RO carve-out: host writing 0x00 clears non-latched bits.
#[repr(C)]
#[derive(Copy, Clone, FlatBlock)]
pub struct TelemetryFault {
    #[ct_field(access = ro)]
    pub mode_active: u8,
    #[ct_field(access = ro)]
    pub fault_flags: u8,
    #[ct_field(access = ro)]
    pub fault_code: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
}

#[repr(C)]
#[derive(Copy, Clone, FlatBlock)]
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

/// Host can Write zero to any field to clear it (bench instrumentation).
/// Chip-side `report_fault` increments via raw pointer, bypassing the regmap.
/// Concurrent host clear + ISR increment may drop one update — acceptable for bench.
#[repr(C)]
#[derive(Copy, Clone, FlatBlock)]
pub struct TelemetryDxlLink {
    #[ct_field(access = rw)]
    pub illegal_transition: u32,
    #[ct_field(access = rw)]
    pub unexpected_byte_count: u32,
    #[ct_field(access = rw)]
    pub previous_slot_timeout: u32,
    #[ct_field(access = rw)]
    pub slot_timing_miss: u32,
    #[ct_field(access = rw)]
    pub crc_patch_deadline_miss: u32,
    #[ct_field(access = rw)]
    pub dma_overrun: u32,
    #[ct_field(access = rw)]
    pub parity_error: u32,
    #[ct_field(access = rw)]
    pub framing_error: u32,
    #[ct_field(access = rw)]
    pub noise_error: u32,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(base = crate::regions::TELEMETRY_BASE_ADDR, size = crate::regions::TELEMETRY_REGION_SIZE)]
pub struct TelemetryRegs {
    pub converted: TelemetryConverted,
    pub intermediaries: TelemetryIntermediaries,
    pub fault: TelemetryFault,
    pub raw: TelemetryRaw,
    pub link: TelemetryDxlLink,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 32],
}
