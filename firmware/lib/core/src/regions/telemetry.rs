use crate::regions::TELEMETRY_BLOCK_SIZE;
use crate::regmap::{Access, FieldDesc};
use core::mem::offset_of;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct TelemetryConverted {
    pub present_position: i32,
    pub present_velocity: i32,
    pub present_current: i16,
    pub present_temp: i16,
    pub present_vbus_mv: u16,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct TelemetryIntermediaries {
    pub vbus_filt_mv: u16,
    pub t_winding_dc: i16,
    pub pwm_duty_actual: i16,
    pub _rsvd_align: u16,
    pub pid_error_last: i32,
    pub pid_output_last: i32,
    pub internal_goal: i32,
    pub sample_tick: u32,
}

/// `fault_flags` writable-RO carve-out: host writing 0x00 clears non-latched bits.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TelemetryFault {
    pub mode_active: u8,
    pub fault_flags: u8,
    pub fault_code: u8,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct TelemetryRaw {
    pub raw_pos: u16,
    pub raw_current: u16,
    pub raw_temp: u16,
    pub raw_vbus: u16,
    pub raw_vmotor_a: u16,
    pub raw_vmotor_b: u16,
    pub raw_enc_a: u16,
    pub raw_enc_b: u16,
}

#[derive(Copy, Clone)]
#[repr(C)]
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

/// All fields RO from host; `fault_flags` clear-byte carve-out enforced one layer up.
const CONVERTED_ADDR: u16 = 0;
const CONVERTED_STRUCT: u16 = offset_of!(TelemetryRegs, converted) as u16;
const INTERM_ADDR: u16 = TELEMETRY_BLOCK_SIZE as u16;
const INTERM_STRUCT: u16 = offset_of!(TelemetryRegs, intermediaries) as u16;
const FAULT_ADDR: u16 = 2 * TELEMETRY_BLOCK_SIZE as u16;
const FAULT_STRUCT: u16 = offset_of!(TelemetryRegs, fault) as u16;
const RAW_ADDR: u16 = 3 * TELEMETRY_BLOCK_SIZE as u16;
const RAW_STRUCT: u16 = offset_of!(TelemetryRegs, raw) as u16;

pub const TELEMETRY_FIELDS: &[FieldDesc] = &[
    // TelemetryConverted
    FieldDesc {
        addr_offset: CONVERTED_ADDR + offset_of!(TelemetryConverted, present_position) as u16,
        size: 4,
        struct_offset: CONVERTED_STRUCT + offset_of!(TelemetryConverted, present_position) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: CONVERTED_ADDR + offset_of!(TelemetryConverted, present_velocity) as u16,
        size: 4,
        struct_offset: CONVERTED_STRUCT + offset_of!(TelemetryConverted, present_velocity) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: CONVERTED_ADDR + offset_of!(TelemetryConverted, present_current) as u16,
        size: 2,
        struct_offset: CONVERTED_STRUCT + offset_of!(TelemetryConverted, present_current) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: CONVERTED_ADDR + offset_of!(TelemetryConverted, present_temp) as u16,
        size: 2,
        struct_offset: CONVERTED_STRUCT + offset_of!(TelemetryConverted, present_temp) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: CONVERTED_ADDR + offset_of!(TelemetryConverted, present_vbus_mv) as u16,
        size: 2,
        struct_offset: CONVERTED_STRUCT + offset_of!(TelemetryConverted, present_vbus_mv) as u16,
        access: Access::Ro,
    },
    // TelemetryIntermediaries (skip _rsvd_align at +6..8)
    FieldDesc {
        addr_offset: INTERM_ADDR + offset_of!(TelemetryIntermediaries, vbus_filt_mv) as u16,
        size: 2,
        struct_offset: INTERM_STRUCT + offset_of!(TelemetryIntermediaries, vbus_filt_mv) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: INTERM_ADDR + offset_of!(TelemetryIntermediaries, t_winding_dc) as u16,
        size: 2,
        struct_offset: INTERM_STRUCT + offset_of!(TelemetryIntermediaries, t_winding_dc) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: INTERM_ADDR + offset_of!(TelemetryIntermediaries, pwm_duty_actual) as u16,
        size: 2,
        struct_offset: INTERM_STRUCT + offset_of!(TelemetryIntermediaries, pwm_duty_actual) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: INTERM_ADDR + offset_of!(TelemetryIntermediaries, pid_error_last) as u16,
        size: 4,
        struct_offset: INTERM_STRUCT + offset_of!(TelemetryIntermediaries, pid_error_last) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: INTERM_ADDR + offset_of!(TelemetryIntermediaries, pid_output_last) as u16,
        size: 4,
        struct_offset: INTERM_STRUCT + offset_of!(TelemetryIntermediaries, pid_output_last) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: INTERM_ADDR + offset_of!(TelemetryIntermediaries, internal_goal) as u16,
        size: 4,
        struct_offset: INTERM_STRUCT + offset_of!(TelemetryIntermediaries, internal_goal) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: INTERM_ADDR + offset_of!(TelemetryIntermediaries, sample_tick) as u16,
        size: 4,
        struct_offset: INTERM_STRUCT + offset_of!(TelemetryIntermediaries, sample_tick) as u16,
        access: Access::Ro,
    },
    // TelemetryFault
    FieldDesc {
        addr_offset: FAULT_ADDR + offset_of!(TelemetryFault, mode_active) as u16,
        size: 1,
        struct_offset: FAULT_STRUCT + offset_of!(TelemetryFault, mode_active) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: FAULT_ADDR + offset_of!(TelemetryFault, fault_flags) as u16,
        size: 1,
        struct_offset: FAULT_STRUCT + offset_of!(TelemetryFault, fault_flags) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: FAULT_ADDR + offset_of!(TelemetryFault, fault_code) as u16,
        size: 1,
        struct_offset: FAULT_STRUCT + offset_of!(TelemetryFault, fault_code) as u16,
        access: Access::Ro,
    },
    // TelemetryRaw
    FieldDesc {
        addr_offset: RAW_ADDR + offset_of!(TelemetryRaw, raw_pos) as u16,
        size: 2,
        struct_offset: RAW_STRUCT + offset_of!(TelemetryRaw, raw_pos) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: RAW_ADDR + offset_of!(TelemetryRaw, raw_current) as u16,
        size: 2,
        struct_offset: RAW_STRUCT + offset_of!(TelemetryRaw, raw_current) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: RAW_ADDR + offset_of!(TelemetryRaw, raw_temp) as u16,
        size: 2,
        struct_offset: RAW_STRUCT + offset_of!(TelemetryRaw, raw_temp) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: RAW_ADDR + offset_of!(TelemetryRaw, raw_vbus) as u16,
        size: 2,
        struct_offset: RAW_STRUCT + offset_of!(TelemetryRaw, raw_vbus) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: RAW_ADDR + offset_of!(TelemetryRaw, raw_vmotor_a) as u16,
        size: 2,
        struct_offset: RAW_STRUCT + offset_of!(TelemetryRaw, raw_vmotor_a) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: RAW_ADDR + offset_of!(TelemetryRaw, raw_vmotor_b) as u16,
        size: 2,
        struct_offset: RAW_STRUCT + offset_of!(TelemetryRaw, raw_vmotor_b) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: RAW_ADDR + offset_of!(TelemetryRaw, raw_enc_a) as u16,
        size: 2,
        struct_offset: RAW_STRUCT + offset_of!(TelemetryRaw, raw_enc_a) as u16,
        access: Access::Ro,
    },
    FieldDesc {
        addr_offset: RAW_ADDR + offset_of!(TelemetryRaw, raw_enc_b) as u16,
        size: 2,
        struct_offset: RAW_STRUCT + offset_of!(TelemetryRaw, raw_enc_b) as u16,
        access: Access::Ro,
    },
];

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::TELEMETRY_BLOCK_SIZE;
    use core::mem::size_of;

    #[test]
    fn leaf_blocks_fit_block() {
        assert!(size_of::<TelemetryConverted>() <= TELEMETRY_BLOCK_SIZE);
        assert!(size_of::<TelemetryIntermediaries>() <= TELEMETRY_BLOCK_SIZE);
        assert!(size_of::<TelemetryFault>() <= TELEMETRY_BLOCK_SIZE);
        assert!(size_of::<TelemetryRaw>() <= TELEMETRY_BLOCK_SIZE);
    }
}
