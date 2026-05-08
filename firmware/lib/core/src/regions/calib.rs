//! CALIB region — single-copy persistent calibration with per-block CRC.

use crate::page::PageHeader;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PotLutBlock {
    pub raw_min: u16,
    pub raw_max: u16,
    pub lut: [i32; 55],
    pub header: PageHeader,
}

/// Compact (40 B); flash-side block is 256 B with header at flash offset 0xE0.
/// Save/load bridges layouts via the per-block descriptor.
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
