use crate::page::PageHeader;
use crate::regions::locks;
use control_table::{Block, Region};

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct PotLutBlock {
    pub raw_min: u16,
    pub raw_max: u16,
    pub lut: [i32; 55],
    #[ct_field(skip)]
    pub header: PageHeader,
}

/// In-RAM 40 B; flash-side block is 256 B with header at flash offset 0xE0.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct BemfCalibBlock {
    pub ke_uvs_per_rad: u16,
    pub r_motor_mohm: u16,
    pub calib_v_bus_mv: u16,
    pub calib_i_ma: u16,
    #[ct_field(skip)]
    pub header: PageHeader,
}

#[repr(C)]
#[derive(Copy, Clone, Region)]
#[ct_region(
    addr = crate::regions::CALIB_BASE_ADDR,
    size = crate::regions::CALIB_REGION_SIZE,
    validators = [locks::torque_locked],
)]
pub struct CalibRegs {
    pub pot_lut: PotLutBlock,
    pub bemf: BemfCalibBlock,
}

impl PotLutBlock {
    pub const fn const_new() -> Self {
        Self {
            raw_min: 0,
            raw_max: 0,
            lut: [0; 55],
            header: PageHeader::const_erased(),
        }
    }
}

impl BemfCalibBlock {
    pub const fn const_new() -> Self {
        Self {
            ke_uvs_per_rad: 0,
            r_motor_mohm: 0,
            calib_v_bus_mv: 0,
            calib_i_ma: 0,
            header: PageHeader::const_erased(),
        }
    }
}

impl CalibRegs {
    pub const fn const_new() -> Self {
        Self {
            pot_lut: PotLutBlock::const_new(),
            bemf: BemfCalibBlock::const_new(),
        }
    }
}
