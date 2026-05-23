use crate::page::PageHeader;
use crate::regions::locks;
use control_table::{Block, Enum, Region};

/// DXL X-series baud rate indices. V006 USART caps at 3 Mbps; indices 6–7
/// (4 Mbps, 4.5 Mbps) are absent and host writes of them get rejected.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
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
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum StallResponse {
    #[default]
    Disable = 0,
    Comply = 1,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigIdentity {
    #[ct_field(access = ro)]
    pub model_number: u16,
    #[ct_field(access = ro)]
    pub firmware_version: u16,
    #[ct_field(access = ro)]
    pub hardware_revision: u32,
    #[ct_field(access = ro)]
    pub capability_flags: u32,
}

/// Writes gated on torque (FAULT_CFG_LOCK).
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigComms {
    #[ct_field(le = 252u8)]
    pub id: u8,
    pub baud_rate_idx: BaudRate,
    pub return_delay_2us: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigPosLimits {
    #[ct_field(lt = &addr::pos_limits::POS_MAX_PHYS_URAD)]
    pub pos_min_phys_urad: i32,
    #[ct_field(gt = &addr::pos_limits::POS_MIN_PHYS_URAD)]
    pub pos_max_phys_urad: i32,
    #[ct_field(
        ge = &addr::pos_limits::POS_MIN_PHYS_URAD,
        le = &addr::pos_limits::POS_MAX_PHYS_URAD,
        lt = &addr::pos_limits::POS_MAX_SOFT_URAD,
    )]
    pub pos_min_soft_urad: i32,
    #[ct_field(
        ge = &addr::pos_limits::POS_MIN_PHYS_URAD,
        le = &addr::pos_limits::POS_MAX_PHYS_URAD,
        gt = &addr::pos_limits::POS_MIN_SOFT_URAD,
    )]
    pub pos_max_soft_urad: i32,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigStall {
    pub stall_response: StallResponse,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    pub stall_effort_threshold: i16,
    pub stall_motion_threshold_urad: u32,
    pub stall_time_threshold_ms: u16,
    pub comply_release_window_ms: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigThermal {
    pub motor_thermal_k_q88: u16,
    pub motor_thermal_tau_ms: u16,
    #[ct_field(gt = &addr::thermal::WINDING_RECOVER_CC)]
    pub winding_cutoff_cc: i16,
    #[ct_field(lt = &addr::thermal::WINDING_CUTOFF_CC)]
    pub winding_recover_cc: i16,
    pub v_undervolt_mv: u16,
}

/// `vdd_mv` is the DMM-measured VDD-at-chip-pin baked in per board.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigCalibration {
    pub vdd_mv: u16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigControlPosition {
    pub pid_kp_q88: u16,
    pub pid_ki_q88: u16,
    pub pid_kd_q88: u16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
    pub pid_i_limit: i32,
    pub pos_deadband_urad: u32,
    #[ct_field(le = 50u8)]
    pub pwm_deadband_pct: u8,
    pub v_comp_enable: bool,
    pub max_effort: i16,
    pub v_nominal_mv: u16,
}

/// Compact in-RAM mirror; on-flash page is full 512 B with reserved gaps.
#[repr(C)]
#[derive(Copy, Clone, Region)]
#[ct_region(
    addr = crate::regions::CONFIG_BASE_ADDR,
    size = crate::regions::CONFIG_REGION_SIZE,
    validators = [locks::torque_locked],
)]
pub struct ConfigRegs {
    pub identity: ConfigIdentity,
    pub comms: ConfigComms,
    pub pos_limits: ConfigPosLimits,
    pub stall: ConfigStall,
    pub thermal: ConfigThermal,
    pub ctrl_pos: ConfigControlPosition,
    pub calibration: ConfigCalibration,
    /// Persistence-layer metadata; not exposed via the protocol.
    #[ct_region(skip)]
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

impl ConfigCalibration {
    pub const fn const_new() -> Self {
        Self {
            vdd_mv: 0,
            _rsvd_align: 0,
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

impl ConfigRegs {
    pub const fn const_new() -> Self {
        Self {
            identity: ConfigIdentity::const_new(),
            comms: ConfigComms::const_new(),
            pos_limits: ConfigPosLimits::const_new(),
            stall: ConfigStall::const_new(),
            thermal: ConfigThermal::const_new(),
            ctrl_pos: ConfigControlPosition::const_new(),
            calibration: ConfigCalibration::const_new(),
            header: PageHeader::const_erased(),
        }
    }
}
