use control_table::{Block, Enum, Section};

/// osc-native operational baud (sec 2). Recovery is the rescue break's job, not a
/// crawl-speed fallback, so only the four operational rates exist.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum BaudRate {
    // Default = the sec 9.1 rescue floor, the always-reachable rate -- and the
    // zero value, which keeps the table's const image all-zero so SHARED
    // lands in .bss (no 1 KB flash init image). The operational default is
    // the board's `ConfigDefaults.baud`, seeded at boot before the bus runs.
    #[default]
    B500000 = 0,
    B1000000 = 1,
    B2000000 = 2,
    B3000000 = 3,
}

/// osc-native sec 7 default: chain reclaim + host timeout, not a reply-time floor.
pub const DEFAULT_RESPONSE_DEADLINE_US: u16 = 60;

impl BaudRate {
    pub const fn as_idx(self) -> u8 {
        self as u8
    }

    pub const fn as_hz(self) -> u32 {
        match self {
            BaudRate::B500000 => 500_000,
            BaudRate::B1000000 => 1_000_000,
            BaudRate::B2000000 => 2_000_000,
            BaudRate::B3000000 => 3_000_000,
        }
    }

    pub const fn from_idx(idx: u8) -> Option<Self> {
        match idx {
            0 => Some(BaudRate::B500000),
            1 => Some(BaudRate::B1000000),
            2 => Some(BaudRate::B2000000),
            3 => Some(BaudRate::B3000000),
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
    pub firmware_version: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    #[ct_field(access = ro)]
    pub hardware_revision: u32,
    #[ct_field(access = ro)]
    pub capability_flags: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
#[ct_block(hooks = crate::regions::hooks::ControlTableHookEvents)]
pub struct ConfigComms {
    #[ct_field(ge = 1u8, le = 249u8, hook = on_id_write)]
    pub id: u8,
    #[ct_field(hook = on_baud_rate_idx_write)]
    pub baud_rate_idx: BaudRate,
    #[ct_field(hook = on_response_deadline_us_write)]
    pub response_deadline_us: u16,
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
    #[ct_field(skip)]
    pub _rsvd_tail: u16,
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
    #[ct_field(skip)]
    pub _rsvd_tail: u16,
}

/// Config section: always writable (normal field validation applies), volatile
/// until `MGMT SAVE` persists it -- SAVE is the only torque-gated operation
/// (osc-native sec 9.4 separates write from persistence).
#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CONFIG_BASE_ADDR,
    size = crate::regions::CONFIG_REGION_SIZE,
    hooks = crate::regions::hooks::ControlTableHookEvents,
)]
pub struct ConfigRegs {
    pub identity: ConfigIdentity,
    pub comms: ConfigComms,
    pub pos_limits: ConfigPosLimits,
    pub stall: ConfigStall,
    pub thermal: ConfigThermal,
    pub ctrl_pos: ConfigControlPosition,
    pub calibration: ConfigCalibration,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 44],
}

/// Boot-time seed for `ControlTable.config`; stamped pre-IRQ, then host-owned.
#[derive(Copy, Clone, Debug, Default)]
pub struct ConfigDefaults {
    pub pos_min_phys_urad: i32,
    pub pos_max_phys_urad: i32,
    /// VDD-at-chip-pin in mV; the v006 ADC reference is VDD itself.
    pub vdd_mv: u16,
    pub id: u8,
    pub baud: BaudRate,
    pub response_deadline_us: u16,
}
