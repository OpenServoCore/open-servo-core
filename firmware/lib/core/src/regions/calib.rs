use control_table::{Block, Section};

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct PotLutBlock {
    pub raw_min: u16,
    pub raw_max: u16,
    pub lut: [i32; 55],
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct BemfCalibBlock {
    pub ke_uvs_per_rad: u16,
    pub r_motor_mohm: u16,
    pub calib_v_bus_mv: u16,
    pub calib_i_ma: u16,
}

/// Calibration section. Torque-gated writes fail with `Access` while
/// `lifecycle.torque_enable` is set.
#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CALIB_BASE_ADDR,
    size = crate::regions::CALIB_REGION_SIZE,
    write_locked_by = super::control::addr::lifecycle::TORQUE_ENABLE,
)]
pub struct CalibRegs {
    pub pot_lut: PotLutBlock,
    pub bemf: BemfCalibBlock,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 24],
}
