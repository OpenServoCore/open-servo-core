use crate::regions::config;
use control_table::{Enum, FlatBlock, Section};

/// Position controller mode. `repr(u8)` so the byte-level commit path round-trips
/// cleanly; validators MUST gate writes to `Mode::ALLOWED` because constructing a
/// `Mode` from an unlisted discriminant is UB.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum Mode {
    #[default]
    OpenLoop = 0,
    PositionPid = 1,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum BootMode {
    #[default]
    App = 0,
    Bootloader = 1,
}

#[repr(C)]
#[derive(Copy, Clone, FlatBlock)]
pub struct ControlLifecycle {
    pub torque_enable: bool,
    pub mode: Mode,
    #[ct_field(skip)]
    pub _rsvd_align: [u8; 2],
    #[ct_field(
        ge = &config::addr::pos_limits::POS_MIN_SOFT_URAD,
        le = &config::addr::pos_limits::POS_MAX_SOFT_URAD,
    )]
    pub goal_position: i32,
    pub goal_velocity: i32,
    #[ct_field(le = &config::addr::ctrl_pos::MAX_EFFORT, abs)]
    pub goal_effort: i16,
    #[ct_field(skip)]
    pub _rsvd_tail: [u8; 2],
}

#[repr(C)]
#[derive(Copy, Clone, FlatBlock)]
pub struct ControlStreaming {
    pub stream_enable: bool,
    pub stream_decimation: u8,
    #[ct_field(ge = 1u16)]
    pub stream_duration_ms: u16,
    pub stream_field_mask: u32,
    #[ct_field(access = ro)]
    pub stream_dropped: u32,
}

#[repr(C)]
#[derive(Copy, Clone, FlatBlock)]
pub struct ControlSystem {
    pub boot_mode: BootMode,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(base = crate::regions::CONTROL_BASE_ADDR, size = crate::regions::CONTROL_REGION_SIZE)]
pub struct ControlRegs {
    pub lifecycle: ControlLifecycle,
    pub streaming: ControlStreaming,
    pub system: ControlSystem,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 99],
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
