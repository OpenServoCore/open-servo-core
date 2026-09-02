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

/// Sense-chain primitives the host converts raw counts with (protocol sec
/// 5.5): divider legs in ohms, amplifier gain x1000. `vdd_mv` is the
/// host-measured VDD at the chip pin -- the v006 ADC reference is VDD itself.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibSense {
    #[ct_field(access = ro)]
    pub shunt_r_mohm: u16,
    #[ct_field(access = ro)]
    pub gain_milli: u16,
    #[ct_field(access = ro)]
    pub vmotor_div_top: u16,
    #[ct_field(access = ro)]
    pub vmotor_div_bot: u16,
    pub vdd_mv: u16,
}

/// Winding-resistance thermometry anchor: `r0_mohm` is the cold winding
/// resistance the calibration routine measures, `t0_cc` the ambient the user
/// entered for it (centi-degC).
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibWinding {
    pub r0_mohm: u16,
    pub t0_cc: i16,
}

/// Calibration section: always writable (normal field validation applies),
/// volatile until persisted -- persistence is SAVE's job, not a write gate.
#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CALIB_BASE_ADDR,
    size = crate::regions::CALIB_REGION_SIZE,
)]
pub struct CalibRegs {
    pub pot_lut: PotLutBlock,
    pub bemf: BemfCalibBlock,
    pub sense: CalibSense,
    pub winding: CalibWinding,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 10],
}
