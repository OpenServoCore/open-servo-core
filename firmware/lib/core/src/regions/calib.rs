use crate::page::PageHeader;
use crate::regions::{CALIB_BASE_ADDR, CALIB_BLOCK_SIZE};
use crate::regmap::{Access, FieldDesc};
use core::mem::{offset_of, size_of};

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PotLutBlock {
    pub raw_min: u16,
    pub raw_max: u16,
    pub lut: [i32; 55],
    pub header: PageHeader,
}

/// In-RAM 40 B; flash-side block is 256 B with header at flash offset 0xE0.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct BemfCalibBlock {
    pub ke_uvs_per_rad: u16,
    pub r_motor_mohm: u16,
    pub calib_v_bus_mv: u16,
    pub calib_i_ma: u16,
    pub header: PageHeader,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct CalibRegs {
    pub pot_lut: PotLutBlock,
    pub bemf: BemfCalibBlock,
}

// Trailing PageHeader is persistence-layer only; not in field tables.
const POT_LUT_ADDR: u16 = CALIB_BASE_ADDR;
const POT_LUT_STRUCT: u16 = offset_of!(CalibRegs, pot_lut) as u16;
const BEMF_ADDR: u16 = CALIB_BASE_ADDR + CALIB_BLOCK_SIZE as u16;
const BEMF_STRUCT: u16 = offset_of!(CalibRegs, bemf) as u16;

// PotLutBlock body (raw_min, raw_max, lut)
pub static FIELD_RAW_MIN: FieldDesc = FieldDesc {
    addr: POT_LUT_ADDR + offset_of!(PotLutBlock, raw_min) as u16,
    size: 2,
    struct_offset: POT_LUT_STRUCT + offset_of!(PotLutBlock, raw_min) as u16,
    access: Access::Rw,
    validators: &[],
};
pub static FIELD_RAW_MAX: FieldDesc = FieldDesc {
    addr: POT_LUT_ADDR + offset_of!(PotLutBlock, raw_max) as u16,
    size: 2,
    struct_offset: POT_LUT_STRUCT + offset_of!(PotLutBlock, raw_max) as u16,
    access: Access::Rw,
    validators: &[],
};
pub static FIELD_LUT: FieldDesc = FieldDesc {
    addr: POT_LUT_ADDR + offset_of!(PotLutBlock, lut) as u16,
    size: size_of::<[i32; 55]>() as u16,
    struct_offset: POT_LUT_STRUCT + offset_of!(PotLutBlock, lut) as u16,
    access: Access::Rw,
    validators: &[],
};

// BemfCalibBlock body (4 × u16)
pub static FIELD_KE_UVS_PER_RAD: FieldDesc = FieldDesc {
    addr: BEMF_ADDR + offset_of!(BemfCalibBlock, ke_uvs_per_rad) as u16,
    size: 2,
    struct_offset: BEMF_STRUCT + offset_of!(BemfCalibBlock, ke_uvs_per_rad) as u16,
    access: Access::Rw,
    validators: &[],
};
pub static FIELD_R_MOTOR_MOHM: FieldDesc = FieldDesc {
    addr: BEMF_ADDR + offset_of!(BemfCalibBlock, r_motor_mohm) as u16,
    size: 2,
    struct_offset: BEMF_STRUCT + offset_of!(BemfCalibBlock, r_motor_mohm) as u16,
    access: Access::Rw,
    validators: &[],
};
pub static FIELD_CALIB_V_BUS_MV: FieldDesc = FieldDesc {
    addr: BEMF_ADDR + offset_of!(BemfCalibBlock, calib_v_bus_mv) as u16,
    size: 2,
    struct_offset: BEMF_STRUCT + offset_of!(BemfCalibBlock, calib_v_bus_mv) as u16,
    access: Access::Rw,
    validators: &[],
};
pub static FIELD_CALIB_I_MA: FieldDesc = FieldDesc {
    addr: BEMF_ADDR + offset_of!(BemfCalibBlock, calib_i_ma) as u16,
    size: 2,
    struct_offset: BEMF_STRUCT + offset_of!(BemfCalibBlock, calib_i_ma) as u16,
    access: Access::Rw,
    validators: &[],
};

pub const CALIB_FIELDS: &[FieldDesc] = &[
    FIELD_RAW_MIN,
    FIELD_RAW_MAX,
    FIELD_LUT,
    FIELD_KE_UVS_PER_RAD,
    FIELD_R_MOTOR_MOHM,
    FIELD_CALIB_V_BUS_MV,
    FIELD_CALIB_I_MA,
];

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::CALIB_BLOCK_SIZE;
    use core::mem::size_of;

    #[test]
    fn pot_lut_exact_block() {
        assert_eq!(size_of::<PotLutBlock>(), CALIB_BLOCK_SIZE);
    }

    #[test]
    fn bemf_fits_block() {
        assert!(size_of::<BemfCalibBlock>() <= CALIB_BLOCK_SIZE);
    }
}
