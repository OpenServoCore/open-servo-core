use control_table::{Block, Region};

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

/// `fault_flags` writable-RO carve-out: host writing 0x00 clears non-latched bits.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct TelemetryFault {
    #[ct_field(access = ro)]
    pub mode_active: u8,
    #[ct_field(access = ro)]
    pub fault_flags: u8,
    #[ct_field(access = ro)]
    pub fault_code: u8,
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
#[derive(Copy, Clone, Region)]
#[ct_region(addr = crate::regions::TELEMETRY_BASE_ADDR, size = crate::regions::TELEMETRY_REGION_SIZE)]
pub struct TelemetryRegs {
    pub converted: TelemetryConverted,
    pub intermediaries: TelemetryIntermediaries,
    pub fault: TelemetryFault,
    pub raw: TelemetryRaw,
}

impl TelemetryConverted {
    pub const fn const_new() -> Self {
        Self {
            present_position: 0,
            present_velocity: 0,
            present_current: 0,
            present_temp: 0,
            present_vbus_mv: 0,
        }
    }
}

impl TelemetryIntermediaries {
    pub const fn const_new() -> Self {
        Self {
            vbus_filt_mv: 0,
            t_winding_dc: 0,
            pwm_duty_actual: 0,
            _rsvd_align: 0,
            pid_error_last: 0,
            pid_output_last: 0,
            internal_goal: 0,
            sample_tick: 0,
        }
    }
}

impl TelemetryFault {
    pub const fn const_new() -> Self {
        Self {
            mode_active: 0,
            fault_flags: 0,
            fault_code: 0,
        }
    }
}

impl TelemetryRaw {
    pub const fn const_new() -> Self {
        Self {
            raw_pos: 0,
            raw_current: 0,
            raw_temp: 0,
            raw_vbus: 0,
            raw_vmotor_a: 0,
            raw_vmotor_b: 0,
            raw_enc_a: 0,
            raw_enc_b: 0,
        }
    }
}

impl TelemetryRegs {
    pub const fn const_new() -> Self {
        Self {
            converted: TelemetryConverted::const_new(),
            intermediaries: TelemetryIntermediaries::const_new(),
            fault: TelemetryFault::const_new(),
            raw: TelemetryRaw::const_new(),
        }
    }
}
