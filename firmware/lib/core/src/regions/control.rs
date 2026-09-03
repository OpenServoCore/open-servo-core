use crate::regions::config;
use control_table::{Block, Enum, Section};

/// Control mode. `repr(u8)` so the byte-level commit path round-trips
/// cleanly; validators MUST gate writes to `Mode::ALLOWED` because constructing a
/// `Mode` from an unlisted discriminant is UB.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum Mode {
    #[default]
    OpenLoop = 0,
    Current = 1,
    Velocity = 2,
    Position = 3,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum BootMode {
    #[default]
    App = 0,
    Bootloader = 1,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ControlLifecycle {
    pub torque_enable: bool,
    pub mode: Mode,
    // abs-le duty_max rule lands with the loop-gain block
    pub goal_duty: i16,
    #[ct_field(
        ge = &config::addr::pos_limits::POS_MIN_SOFT_COUNTS,
        le = &config::addr::pos_limits::POS_MAX_SOFT_COUNTS,
    )]
    pub goal_position: i32,
    pub goal_velocity: i32,
    // abs-le current_limit_counts rule lands with the loop-gain block
    pub goal_current: i16,
    #[ct_field(skip)]
    pub _rsvd_tail: [u8; 2],
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ControlSystem {
    pub boot_mode: BootMode,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(base = crate::regions::CONTROL_BASE_ADDR, size = crate::regions::CONTROL_REGION_SIZE)]
pub struct ControlRegs {
    pub lifecycle: ControlLifecycle,
    pub system: ControlSystem,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 111],
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn region_fits_declared_size() {
        assert_eq!(
            size_of::<ControlRegs>(),
            crate::regions::CONTROL_REGION_SIZE as usize
        );
    }
}
