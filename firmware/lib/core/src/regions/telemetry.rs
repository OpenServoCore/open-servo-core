//! TELEMETRY region — RO from the host's perspective; written by the kernel.

use crate::regions::TELEMETRY_BLOCK_SIZE;
use crate::regmap::{Access, BlockDesc};
use core::mem::{offset_of, size_of};

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

/// `fault_flags` has a writable-RO carve-out: host writing 0x00 clears
/// non-latched bits.
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

/// Protocol-address slot map for TELEMETRY. All blocks RO from host.
/// `fault_flags` clear-byte carve-out is enforced one layer up, not here.
pub const TELEMETRY_BLOCKS: &[BlockDesc] = &[
    BlockDesc {
        addr_offset: 0 * TELEMETRY_BLOCK_SIZE as u16,
        size: size_of::<TelemetryConverted>() as u16,
        struct_offset: offset_of!(TelemetryRegs, converted) as u16,
        access: Access::Ro,
    },
    BlockDesc {
        addr_offset: 1 * TELEMETRY_BLOCK_SIZE as u16,
        size: size_of::<TelemetryIntermediaries>() as u16,
        struct_offset: offset_of!(TelemetryRegs, intermediaries) as u16,
        access: Access::Ro,
    },
    BlockDesc {
        addr_offset: 2 * TELEMETRY_BLOCK_SIZE as u16,
        size: size_of::<TelemetryFault>() as u16,
        struct_offset: offset_of!(TelemetryRegs, fault) as u16,
        access: Access::Ro,
    },
    BlockDesc {
        addr_offset: 3 * TELEMETRY_BLOCK_SIZE as u16,
        size: size_of::<TelemetryRaw>() as u16,
        struct_offset: offset_of!(TelemetryRegs, raw) as u16,
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
        assert!(size_of::<TelemetryConverted>()      <= TELEMETRY_BLOCK_SIZE);
        assert!(size_of::<TelemetryIntermediaries>() <= TELEMETRY_BLOCK_SIZE);
        assert!(size_of::<TelemetryFault>()          <= TELEMETRY_BLOCK_SIZE);
        assert!(size_of::<TelemetryRaw>()            <= TELEMETRY_BLOCK_SIZE);
    }
}
