use crate::page::PageHeader;
use crate::regions::locks;
use crate::regions::{CONFIG_BASE_ADDR, CONFIG_BLOCK_SIZE, CONFIG_REGION_SIZE};
use crate::regmap::{
    Access, BOOL_ALLOWED, BlockDesc, CompareOp, CrossField, FieldDesc, FieldValidator, RegionDesc,
};
use core::mem::{offset_of, size_of};

/// DXL X-series baud rate indices. V006 USART caps at 3 Mbps; indices 6–7
/// (4 Mbps, 4.5 Mbps) are absent and host writes of them get rejected.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum BaudRate {
    B9600 = 0,
    B57600 = 1,
    B115200 = 2,
    #[default]
    B1000000 = 3,
    B2000000 = 4,
    B3000000 = 5,
}

impl BaudRate {
    pub const ALLOWED: &[u8] = &[
        BaudRate::B9600 as u8,
        BaudRate::B57600 as u8,
        BaudRate::B115200 as u8,
        BaudRate::B1000000 as u8,
        BaudRate::B2000000 as u8,
        BaudRate::B3000000 as u8,
    ];

    pub const fn as_idx(self) -> u8 {
        self as u8
    }

    pub const fn as_hz(self) -> u32 {
        match self {
            BaudRate::B9600 => 9_600,
            BaudRate::B57600 => 57_600,
            BaudRate::B115200 => 115_200,
            BaudRate::B1000000 => 1_000_000,
            BaudRate::B2000000 => 2_000_000,
            BaudRate::B3000000 => 3_000_000,
        }
    }

    pub const fn from_idx(idx: u8) -> Option<Self> {
        match idx {
            0 => Some(BaudRate::B9600),
            1 => Some(BaudRate::B57600),
            2 => Some(BaudRate::B115200),
            3 => Some(BaudRate::B1000000),
            4 => Some(BaudRate::B2000000),
            5 => Some(BaudRate::B3000000),
            _ => None,
        }
    }
}

/// Stall detector policy. `repr(u8)`; constructing from an unlisted discriminant is UB,
/// so validators MUST gate writes to `StallResponse::ALLOWED`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum StallResponse {
    #[default]
    Disable = 0,
    Comply = 1,
}

impl StallResponse {
    pub const ALLOWED: &[u8] = &[StallResponse::Disable as u8, StallResponse::Comply as u8];
}

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
    pub baud_rate_idx: BaudRate,
    pub return_delay_2us: u8,
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
    pub stall_response: StallResponse,
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
    pub winding_cutoff_cc: i16,
    pub winding_recover_cc: i16,
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
    pub v_comp_enable: bool,
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
            baud_rate_idx: BaudRate::B9600,
            return_delay_2us: 0,
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
            stall_response: StallResponse::Disable,
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
            winding_cutoff_cc: 0,
            winding_recover_cc: 0,
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
            v_comp_enable: false,
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
/// gaps (incl. `_rsvd_align` padding) and unlisted indices return AccessError.
const IDENTITY_ADDR: u16 = CONFIG_BASE_ADDR;
const IDENTITY_STRUCT: u16 = offset_of!(ConfigRegs, identity) as u16;
const COMMS_ADDR: u16 = CONFIG_BASE_ADDR + CONFIG_BLOCK_SIZE as u16;
const COMMS_STRUCT: u16 = offset_of!(ConfigRegs, comms) as u16;
const POS_LIMITS_ADDR: u16 = CONFIG_BASE_ADDR + 2 * CONFIG_BLOCK_SIZE as u16;
const POS_LIMITS_STRUCT: u16 =
    (offset_of!(ConfigRegs, limits) + offset_of!(ConfigLimits, pos)) as u16;
const STALL_ADDR: u16 = CONFIG_BASE_ADDR + 3 * CONFIG_BLOCK_SIZE as u16;
const STALL_STRUCT: u16 = (offset_of!(ConfigRegs, safety) + offset_of!(ConfigSafety, stall)) as u16;
const THERMAL_ADDR: u16 = CONFIG_BASE_ADDR + 4 * CONFIG_BLOCK_SIZE as u16;
const THERMAL_STRUCT: u16 =
    (offset_of!(ConfigRegs, safety) + offset_of!(ConfigSafety, thermal)) as u16;
const CTRL_POS_ADDR: u16 = CONFIG_BASE_ADDR + 5 * CONFIG_BLOCK_SIZE as u16;
const CTRL_POS_STRUCT: u16 =
    (offset_of!(ConfigRegs, control) + offset_of!(ConfigControl, position)) as u16;
const CALIBRATION_ADDR: u16 = CONFIG_BASE_ADDR + 6 * CONFIG_BLOCK_SIZE as u16;
const CALIBRATION_STRUCT: u16 = offset_of!(ConfigRegs, calibration) as u16;
const HEADER_ADDR: u16 = CONFIG_BASE_ADDR + 15 * CONFIG_BLOCK_SIZE as u16;
const HEADER_STRUCT: u16 = offset_of!(ConfigRegs, header) as u16;

// *_ADDR consts break the const-eval cycle that `&FIELD_X` in cross-field validators would form.
pub const POS_MIN_PHYS_URAD_ADDR: u16 =
    POS_LIMITS_ADDR + offset_of!(ConfigPosLimits, pos_min_phys_urad) as u16;
pub const POS_MAX_PHYS_URAD_ADDR: u16 =
    POS_LIMITS_ADDR + offset_of!(ConfigPosLimits, pos_max_phys_urad) as u16;
pub const POS_MIN_SOFT_URAD_ADDR: u16 =
    POS_LIMITS_ADDR + offset_of!(ConfigPosLimits, pos_min_soft_urad) as u16;
pub const POS_MAX_SOFT_URAD_ADDR: u16 =
    POS_LIMITS_ADDR + offset_of!(ConfigPosLimits, pos_max_soft_urad) as u16;
pub const WINDING_CUTOFF_CC_ADDR: u16 =
    THERMAL_ADDR + offset_of!(ConfigThermal, winding_cutoff_cc) as u16;
pub const WINDING_RECOVER_CC_ADDR: u16 =
    THERMAL_ADDR + offset_of!(ConfigThermal, winding_recover_cc) as u16;
pub const MAX_EFFORT_ADDR: u16 =
    CTRL_POS_ADDR + offset_of!(ConfigControlPosition, max_effort) as u16;

// ConfigIdentity (RO)
pub const FIELD_MODEL_NUMBER: FieldDesc = FieldDesc {
    addr: IDENTITY_ADDR + offset_of!(ConfigIdentity, model_number) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigIdentity, model_number) as u16,
    access: Access::Ro,
    validators: &[],
};
pub const FIELD_FIRMWARE_VERSION: FieldDesc = FieldDesc {
    addr: IDENTITY_ADDR + offset_of!(ConfigIdentity, firmware_version) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigIdentity, firmware_version) as u16,
    access: Access::Ro,
    validators: &[],
};
pub const FIELD_HARDWARE_REVISION: FieldDesc = FieldDesc {
    addr: IDENTITY_ADDR + offset_of!(ConfigIdentity, hardware_revision) as u16,
    size: 4,
    struct_offset: offset_of!(ConfigIdentity, hardware_revision) as u16,
    access: Access::Ro,
    validators: &[],
};
pub const FIELD_CAPABILITY_FLAGS: FieldDesc = FieldDesc {
    addr: IDENTITY_ADDR + offset_of!(ConfigIdentity, capability_flags) as u16,
    size: 4,
    struct_offset: offset_of!(ConfigIdentity, capability_flags) as u16,
    access: Access::Ro,
    validators: &[],
};

// ConfigComms
pub const FIELD_ID: FieldDesc = FieldDesc {
    addr: COMMS_ADDR + offset_of!(ConfigComms, id) as u16,
    size: 1,
    struct_offset: offset_of!(ConfigComms, id) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::RangeU8 { lo: 0, hi: 252 }],
};
pub const FIELD_BAUD_RATE_IDX: FieldDesc = FieldDesc {
    addr: COMMS_ADDR + offset_of!(ConfigComms, baud_rate_idx) as u16,
    size: 1,
    struct_offset: offset_of!(ConfigComms, baud_rate_idx) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::EnumU8 {
        allowed: BaudRate::ALLOWED,
    }],
};
pub const FIELD_RETURN_DELAY_2US: FieldDesc = FieldDesc {
    addr: COMMS_ADDR + offset_of!(ConfigComms, return_delay_2us) as u16,
    size: 1,
    struct_offset: offset_of!(ConfigComms, return_delay_2us) as u16,
    access: Access::Rw,
    validators: &[],
};

// ConfigPosLimits
pub const FIELD_POS_MIN_PHYS_URAD: FieldDesc = FieldDesc {
    addr: POS_MIN_PHYS_URAD_ADDR,
    size: 4,
    struct_offset: offset_of!(ConfigPosLimits, pos_min_phys_urad) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::Cross(CrossField::CompareI32 {
        op: CompareOp::Lt,
        other_addr: POS_MAX_PHYS_URAD_ADDR,
    })],
};
pub const FIELD_POS_MAX_PHYS_URAD: FieldDesc = FieldDesc {
    addr: POS_MAX_PHYS_URAD_ADDR,
    size: 4,
    struct_offset: offset_of!(ConfigPosLimits, pos_max_phys_urad) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::Cross(CrossField::CompareI32 {
        op: CompareOp::Gt,
        other_addr: POS_MIN_PHYS_URAD_ADDR,
    })],
};
pub const FIELD_POS_MIN_SOFT_URAD: FieldDesc = FieldDesc {
    addr: POS_MIN_SOFT_URAD_ADDR,
    size: 4,
    struct_offset: offset_of!(ConfigPosLimits, pos_min_soft_urad) as u16,
    access: Access::Rw,
    validators: &[
        FieldValidator::Cross(CrossField::WithinI32 {
            lo_addr: POS_MIN_PHYS_URAD_ADDR,
            hi_addr: POS_MAX_PHYS_URAD_ADDR,
        }),
        FieldValidator::Cross(CrossField::CompareI32 {
            op: CompareOp::Lt,
            other_addr: POS_MAX_SOFT_URAD_ADDR,
        }),
    ],
};
pub const FIELD_POS_MAX_SOFT_URAD: FieldDesc = FieldDesc {
    addr: POS_MAX_SOFT_URAD_ADDR,
    size: 4,
    struct_offset: offset_of!(ConfigPosLimits, pos_max_soft_urad) as u16,
    access: Access::Rw,
    validators: &[
        FieldValidator::Cross(CrossField::WithinI32 {
            lo_addr: POS_MIN_PHYS_URAD_ADDR,
            hi_addr: POS_MAX_PHYS_URAD_ADDR,
        }),
        FieldValidator::Cross(CrossField::CompareI32 {
            op: CompareOp::Gt,
            other_addr: POS_MIN_SOFT_URAD_ADDR,
        }),
    ],
};

// ConfigStall (skip _rsvd_align at +1)
pub const FIELD_STALL_RESPONSE: FieldDesc = FieldDesc {
    addr: STALL_ADDR + offset_of!(ConfigStall, stall_response) as u16,
    size: 1,
    struct_offset: offset_of!(ConfigStall, stall_response) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::EnumU8 {
        allowed: StallResponse::ALLOWED,
    }],
};
pub const FIELD_STALL_EFFORT_THRESHOLD: FieldDesc = FieldDesc {
    addr: STALL_ADDR + offset_of!(ConfigStall, stall_effort_threshold) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigStall, stall_effort_threshold) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_STALL_MOTION_THRESHOLD_URAD: FieldDesc = FieldDesc {
    addr: STALL_ADDR + offset_of!(ConfigStall, stall_motion_threshold_urad) as u16,
    size: 4,
    struct_offset: offset_of!(ConfigStall, stall_motion_threshold_urad) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_STALL_TIME_THRESHOLD_MS: FieldDesc = FieldDesc {
    addr: STALL_ADDR + offset_of!(ConfigStall, stall_time_threshold_ms) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigStall, stall_time_threshold_ms) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_COMPLY_RELEASE_WINDOW_MS: FieldDesc = FieldDesc {
    addr: STALL_ADDR + offset_of!(ConfigStall, comply_release_window_ms) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigStall, comply_release_window_ms) as u16,
    access: Access::Rw,
    validators: &[],
};

// ConfigThermal
pub const FIELD_MOTOR_THERMAL_K_Q88: FieldDesc = FieldDesc {
    addr: THERMAL_ADDR + offset_of!(ConfigThermal, motor_thermal_k_q88) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigThermal, motor_thermal_k_q88) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_MOTOR_THERMAL_TAU_MS: FieldDesc = FieldDesc {
    addr: THERMAL_ADDR + offset_of!(ConfigThermal, motor_thermal_tau_ms) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigThermal, motor_thermal_tau_ms) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_WINDING_CUTOFF_CC: FieldDesc = FieldDesc {
    addr: WINDING_CUTOFF_CC_ADDR,
    size: 2,
    struct_offset: offset_of!(ConfigThermal, winding_cutoff_cc) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::Cross(CrossField::CompareI16 {
        op: CompareOp::Gt,
        other_addr: WINDING_RECOVER_CC_ADDR,
    })],
};
pub const FIELD_WINDING_RECOVER_CC: FieldDesc = FieldDesc {
    addr: WINDING_RECOVER_CC_ADDR,
    size: 2,
    struct_offset: offset_of!(ConfigThermal, winding_recover_cc) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::Cross(CrossField::CompareI16 {
        op: CompareOp::Lt,
        other_addr: WINDING_CUTOFF_CC_ADDR,
    })],
};
pub const FIELD_V_UNDERVOLT_MV: FieldDesc = FieldDesc {
    addr: THERMAL_ADDR + offset_of!(ConfigThermal, v_undervolt_mv) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigThermal, v_undervolt_mv) as u16,
    access: Access::Rw,
    validators: &[],
};

// ConfigControlPosition (skip _rsvd_align at +6..8)
pub const FIELD_PID_KP_Q88: FieldDesc = FieldDesc {
    addr: CTRL_POS_ADDR + offset_of!(ConfigControlPosition, pid_kp_q88) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigControlPosition, pid_kp_q88) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_PID_KI_Q88: FieldDesc = FieldDesc {
    addr: CTRL_POS_ADDR + offset_of!(ConfigControlPosition, pid_ki_q88) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigControlPosition, pid_ki_q88) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_PID_KD_Q88: FieldDesc = FieldDesc {
    addr: CTRL_POS_ADDR + offset_of!(ConfigControlPosition, pid_kd_q88) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigControlPosition, pid_kd_q88) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_PID_I_LIMIT: FieldDesc = FieldDesc {
    addr: CTRL_POS_ADDR + offset_of!(ConfigControlPosition, pid_i_limit) as u16,
    size: 4,
    struct_offset: offset_of!(ConfigControlPosition, pid_i_limit) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_POS_DEADBAND_URAD: FieldDesc = FieldDesc {
    addr: CTRL_POS_ADDR + offset_of!(ConfigControlPosition, pos_deadband_urad) as u16,
    size: 4,
    struct_offset: offset_of!(ConfigControlPosition, pos_deadband_urad) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_PWM_DEADBAND_PCT: FieldDesc = FieldDesc {
    addr: CTRL_POS_ADDR + offset_of!(ConfigControlPosition, pwm_deadband_pct) as u16,
    size: 1,
    struct_offset: offset_of!(ConfigControlPosition, pwm_deadband_pct) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::RangeU8 { lo: 0, hi: 50 }],
};
pub const FIELD_V_COMP_ENABLE: FieldDesc = FieldDesc {
    addr: CTRL_POS_ADDR + offset_of!(ConfigControlPosition, v_comp_enable) as u16,
    size: 1,
    struct_offset: offset_of!(ConfigControlPosition, v_comp_enable) as u16,
    access: Access::Rw,
    validators: &[FieldValidator::EnumU8 {
        allowed: BOOL_ALLOWED,
    }],
};
pub const FIELD_MAX_EFFORT: FieldDesc = FieldDesc {
    addr: MAX_EFFORT_ADDR,
    size: 2,
    struct_offset: offset_of!(ConfigControlPosition, max_effort) as u16,
    access: Access::Rw,
    validators: &[],
};
pub const FIELD_V_NOMINAL_MV: FieldDesc = FieldDesc {
    addr: CTRL_POS_ADDR + offset_of!(ConfigControlPosition, v_nominal_mv) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigControlPosition, v_nominal_mv) as u16,
    access: Access::Rw,
    validators: &[],
};

// ConfigCalibration (skip _rsvd_align at +2..4)
pub const FIELD_VDD_MV: FieldDesc = FieldDesc {
    addr: CALIBRATION_ADDR + offset_of!(ConfigCalibration, vdd_mv) as u16,
    size: 2,
    struct_offset: offset_of!(ConfigCalibration, vdd_mv) as u16,
    access: Access::Rw,
    validators: &[],
};

// PageHeader (RO, host-opaque)
pub const FIELD_HEADER: FieldDesc = FieldDesc {
    addr: HEADER_ADDR,
    size: size_of::<PageHeader>() as u16,
    struct_offset: 0,
    access: Access::Ro,
    validators: &[],
};

pub const BLOCK_IDENTITY: BlockDesc = BlockDesc {
    addr: IDENTITY_ADDR,
    size: size_of::<ConfigIdentity>() as u16,
    struct_offset: IDENTITY_STRUCT,
    fields: &[
        FIELD_MODEL_NUMBER,
        FIELD_FIRMWARE_VERSION,
        FIELD_HARDWARE_REVISION,
        FIELD_CAPABILITY_FLAGS,
    ],
    validators: &[],
};

pub const BLOCK_COMMS: BlockDesc = BlockDesc {
    addr: COMMS_ADDR,
    size: size_of::<ConfigComms>() as u16,
    struct_offset: COMMS_STRUCT,
    fields: &[FIELD_ID, FIELD_BAUD_RATE_IDX, FIELD_RETURN_DELAY_2US],
    validators: &[],
};

pub const BLOCK_POS_LIMITS: BlockDesc = BlockDesc {
    addr: POS_LIMITS_ADDR,
    size: size_of::<ConfigPosLimits>() as u16,
    struct_offset: POS_LIMITS_STRUCT,
    fields: &[
        FIELD_POS_MIN_PHYS_URAD,
        FIELD_POS_MAX_PHYS_URAD,
        FIELD_POS_MIN_SOFT_URAD,
        FIELD_POS_MAX_SOFT_URAD,
    ],
    validators: &[],
};

pub const BLOCK_STALL: BlockDesc = BlockDesc {
    addr: STALL_ADDR,
    size: size_of::<ConfigStall>() as u16,
    struct_offset: STALL_STRUCT,
    fields: &[
        FIELD_STALL_RESPONSE,
        FIELD_STALL_EFFORT_THRESHOLD,
        FIELD_STALL_MOTION_THRESHOLD_URAD,
        FIELD_STALL_TIME_THRESHOLD_MS,
        FIELD_COMPLY_RELEASE_WINDOW_MS,
    ],
    validators: &[],
};

pub const BLOCK_THERMAL: BlockDesc = BlockDesc {
    addr: THERMAL_ADDR,
    size: size_of::<ConfigThermal>() as u16,
    struct_offset: THERMAL_STRUCT,
    fields: &[
        FIELD_MOTOR_THERMAL_K_Q88,
        FIELD_MOTOR_THERMAL_TAU_MS,
        FIELD_WINDING_CUTOFF_CC,
        FIELD_WINDING_RECOVER_CC,
        FIELD_V_UNDERVOLT_MV,
    ],
    validators: &[],
};

pub const BLOCK_CTRL_POS: BlockDesc = BlockDesc {
    addr: CTRL_POS_ADDR,
    size: size_of::<ConfigControlPosition>() as u16,
    struct_offset: CTRL_POS_STRUCT,
    fields: &[
        FIELD_PID_KP_Q88,
        FIELD_PID_KI_Q88,
        FIELD_PID_KD_Q88,
        FIELD_PID_I_LIMIT,
        FIELD_POS_DEADBAND_URAD,
        FIELD_PWM_DEADBAND_PCT,
        FIELD_V_COMP_ENABLE,
        FIELD_MAX_EFFORT,
        FIELD_V_NOMINAL_MV,
    ],
    validators: &[],
};

pub const BLOCK_CALIBRATION: BlockDesc = BlockDesc {
    addr: CALIBRATION_ADDR,
    size: size_of::<ConfigCalibration>() as u16,
    struct_offset: CALIBRATION_STRUCT,
    fields: &[FIELD_VDD_MV],
    validators: &[],
};

pub const BLOCK_HEADER: BlockDesc = BlockDesc {
    addr: HEADER_ADDR,
    size: size_of::<PageHeader>() as u16,
    struct_offset: HEADER_STRUCT,
    fields: &[FIELD_HEADER],
    validators: &[],
};

pub const CONFIG_REGION: RegionDesc = RegionDesc {
    addr: CONFIG_BASE_ADDR,
    size: CONFIG_REGION_SIZE as u16,
    blocks: &[
        BLOCK_IDENTITY,
        BLOCK_COMMS,
        BLOCK_POS_LIMITS,
        BLOCK_STALL,
        BLOCK_THERMAL,
        BLOCK_CTRL_POS,
        BLOCK_CALIBRATION,
        BLOCK_HEADER,
    ],
    validators: &[locks::torque_locked],
};

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
