//! A real calibrated servo's table, dumped off an SG90 on the dev board, so
//! the simulated fleet reads back like hardware instead of "not calibrated".

pub const RAW_MIN: u16 = 5;
pub const RAW_MAX: u16 = 4095;
pub const LUT_CORR: [i16; 55] = [
    0, 0, 421, 364, 345, 323, 317, 306, 286, 272, 279, 276, 271, 262, 267, 246, 265, 261, 250, 239,
    239, 236, 233, 255, 245, 241, 233, 226, 222, 222, 223, 235, 228, 212, 226, 213, 193, 209, 206,
    194, 192, 196, 164, 157, 153, 172, 148, 123, 119, 122, 101, 95, 91, 46, 0,
];
pub const ANGLE_MIN_CDEG: i16 = 0;
pub const ANGLE_MAX_CDEG: i16 = 20200;
pub const GEAR_RATIO_CENTI: u16 = 25464;
pub const POS_MIN_PHYS_COUNTS: i32 = 5;
pub const POS_MAX_PHYS_COUNTS: i32 = 4095;
pub const POS_MIN_SOFT_COUNTS: i32 = 228;
pub const POS_MAX_SOFT_COUNTS: i32 = 3872;
pub const DRIVE_POLARITY: bool = true;
pub const VBUS_DIV_TOP_OHM: u16 = 15000;
pub const VMOTOR_BIAS_NOM_COUNTS: u16 = 779;
pub const SHUNT_R_MOHM: u16 = 60;
pub const VMOTOR_DIV_TOP: u16 = 6800;
pub const VMOTOR_DIV_BOT: u16 = 3300;
