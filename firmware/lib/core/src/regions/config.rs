use crate::page::PageHeader;
use crate::regions::CONFIG_BLOCK_SIZE;
use crate::regmap::{Access, BlockDesc};
use core::mem::{offset_of, size_of};

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigIdentity {
    pub model_number: u16,
    pub firmware_version: u16,
    pub hardware_revision: u32,
    pub capability_flags: u32,
}

/// Writes gated on torque (FAULT_CFG_LOCK).
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigComms {
    pub id: u8,
    pub baud_rate_idx: u8,
    pub return_delay_us: u16,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigLimits {
    pub pos: ConfigPosLimits,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigPosLimits {
    pub pos_min_phys_urad: i32,
    pub pos_max_phys_urad: i32,
    pub pos_min_soft_urad: i32,
    pub pos_max_soft_urad: i32,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigSafety {
    pub stall: ConfigStall,
    pub thermal: ConfigThermal,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigStall {
    pub stall_response: u8,
    pub _rsvd_align: u8,
    pub stall_effort_threshold: i16,
    pub stall_motion_threshold_urad: u32,
    pub stall_time_threshold_ms: u16,
    pub comply_release_window_ms: u16,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigThermal {
    pub motor_thermal_k_q88: u16,
    pub motor_thermal_tau_ms: u16,
    pub winding_cutoff_dc: i16,
    pub winding_recover_dc: i16,
    pub v_undervolt_mv: u16,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigControl {
    pub position: ConfigControlPosition,
}

/// `vdd_mv` is the DMM-measured VDD-at-chip-pin baked in per board.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigCalibration {
    pub vdd_mv: u16,
    pub _rsvd_align: u16,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigControlPosition {
    pub pid_kp_q88: u16,
    pub pid_ki_q88: u16,
    pub pid_kd_q88: u16,
    pub _rsvd_align: u16,
    pub pid_i_limit: i32,
    pub pos_deadband_urad: u32,
    pub pwm_deadband_pct: u8,
    pub v_comp_enable: u8,
    pub max_effort: i16,
    pub v_nominal_mv: u16,
}

/// Compact in-RAM mirror; on-flash page is full 512 B with reserved gaps.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ConfigRegs {
    pub identity: ConfigIdentity,
    pub comms: ConfigComms,
    pub limits: ConfigLimits,
    pub safety: ConfigSafety,
    pub control: ConfigControl,
    pub calibration: ConfigCalibration,
    /// Host-RO; writes into header range return AccessError.
    pub header: PageHeader,
}

impl ConfigIdentity {
    pub const fn const_new() -> Self {
        Self {
            model_number: 0,
            firmware_version: 0,
            hardware_revision: 0,
            capability_flags: 0,
        }
    }
}

impl ConfigComms {
    pub const fn const_new() -> Self {
        Self {
            id: 0,
            baud_rate_idx: 0,
            return_delay_us: 0,
        }
    }
}

impl ConfigPosLimits {
    pub const fn const_new() -> Self {
        Self {
            pos_min_phys_urad: 0,
            pos_max_phys_urad: 0,
            pos_min_soft_urad: 0,
            pos_max_soft_urad: 0,
        }
    }
}

impl ConfigLimits {
    pub const fn const_new() -> Self {
        Self {
            pos: ConfigPosLimits::const_new(),
        }
    }
}

impl ConfigStall {
    pub const fn const_new() -> Self {
        Self {
            stall_response: 0,
            _rsvd_align: 0,
            stall_effort_threshold: 0,
            stall_motion_threshold_urad: 0,
            stall_time_threshold_ms: 0,
            comply_release_window_ms: 0,
        }
    }
}

impl ConfigThermal {
    pub const fn const_new() -> Self {
        Self {
            motor_thermal_k_q88: 0,
            motor_thermal_tau_ms: 0,
            winding_cutoff_dc: 0,
            winding_recover_dc: 0,
            v_undervolt_mv: 0,
        }
    }
}

impl ConfigSafety {
    pub const fn const_new() -> Self {
        Self {
            stall: ConfigStall::const_new(),
            thermal: ConfigThermal::const_new(),
        }
    }
}

impl ConfigControlPosition {
    pub const fn const_new() -> Self {
        Self {
            pid_kp_q88: 0,
            pid_ki_q88: 0,
            pid_kd_q88: 0,
            _rsvd_align: 0,
            pid_i_limit: 0,
            pos_deadband_urad: 0,
            pwm_deadband_pct: 0,
            v_comp_enable: 0,
            max_effort: 0,
            v_nominal_mv: 0,
        }
    }
}

impl ConfigControl {
    pub const fn const_new() -> Self {
        Self {
            position: ConfigControlPosition::const_new(),
        }
    }
}

impl ConfigCalibration {
    pub const fn const_new() -> Self {
        Self {
            vdd_mv: 0,
            _rsvd_align: 0,
        }
    }
}

/// Block N starts at `CONFIG_BASE_ADDR + N * CONFIG_BLOCK_SIZE`;
/// unlisted indices are reserved and return AccessError.
pub const CONFIG_BLOCKS: &[BlockDesc] = &[
    BlockDesc {
        addr_offset: 0,
        size: size_of::<ConfigIdentity>() as u16,
        struct_offset: offset_of!(ConfigRegs, identity) as u16,
        access: Access::Ro,
    },
    BlockDesc {
        addr_offset: CONFIG_BLOCK_SIZE as u16,
        size: size_of::<ConfigComms>() as u16,
        struct_offset: offset_of!(ConfigRegs, comms) as u16,
        access: Access::Rw,
    },
    BlockDesc {
        addr_offset: 2 * CONFIG_BLOCK_SIZE as u16,
        size: size_of::<ConfigPosLimits>() as u16,
        struct_offset: (offset_of!(ConfigRegs, limits) + offset_of!(ConfigLimits, pos)) as u16,
        access: Access::Rw,
    },
    BlockDesc {
        addr_offset: 3 * CONFIG_BLOCK_SIZE as u16,
        size: size_of::<ConfigStall>() as u16,
        struct_offset: (offset_of!(ConfigRegs, safety) + offset_of!(ConfigSafety, stall)) as u16,
        access: Access::Rw,
    },
    BlockDesc {
        addr_offset: 4 * CONFIG_BLOCK_SIZE as u16,
        size: size_of::<ConfigThermal>() as u16,
        struct_offset: (offset_of!(ConfigRegs, safety) + offset_of!(ConfigSafety, thermal)) as u16,
        access: Access::Rw,
    },
    BlockDesc {
        addr_offset: 5 * CONFIG_BLOCK_SIZE as u16,
        size: size_of::<ConfigControlPosition>() as u16,
        struct_offset: (offset_of!(ConfigRegs, control) + offset_of!(ConfigControl, position))
            as u16,
        access: Access::Rw,
    },
    BlockDesc {
        addr_offset: 6 * CONFIG_BLOCK_SIZE as u16,
        size: size_of::<ConfigCalibration>() as u16,
        struct_offset: offset_of!(ConfigRegs, calibration) as u16,
        access: Access::Rw,
    },
    BlockDesc {
        addr_offset: 15 * CONFIG_BLOCK_SIZE as u16,
        size: size_of::<PageHeader>() as u16,
        struct_offset: offset_of!(ConfigRegs, header) as u16,
        access: Access::Ro,
    },
];

impl ConfigRegs {
    pub const fn const_new() -> Self {
        Self {
            identity: ConfigIdentity::const_new(),
            comms: ConfigComms::const_new(),
            limits: ConfigLimits::const_new(),
            safety: ConfigSafety::const_new(),
            control: ConfigControl::const_new(),
            calibration: ConfigCalibration::const_new(),
            header: PageHeader::const_erased(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::CONFIG_BLOCK_SIZE;
    use core::mem::size_of;

    #[test]
    fn leaf_blocks_fit_block() {
        assert!(size_of::<ConfigIdentity>() <= CONFIG_BLOCK_SIZE);
        assert!(size_of::<ConfigComms>() <= CONFIG_BLOCK_SIZE);
        assert!(size_of::<ConfigPosLimits>() <= CONFIG_BLOCK_SIZE);
        assert!(size_of::<ConfigStall>() <= CONFIG_BLOCK_SIZE);
        assert!(size_of::<ConfigThermal>() <= CONFIG_BLOCK_SIZE);
        assert!(size_of::<ConfigControlPosition>() <= CONFIG_BLOCK_SIZE);
        assert!(size_of::<ConfigCalibration>() <= CONFIG_BLOCK_SIZE);
    }
}
