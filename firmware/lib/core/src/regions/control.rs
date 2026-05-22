use crate::regions::{CONTROL_BASE_ADDR, CONTROL_BLOCK_SIZE};
use crate::regmap::{Access, BOOL_ALLOWED, FieldDesc, Validator};
use core::mem::offset_of;

/// Position controller mode. `repr(u8)` so the byte-level commit path round-trips
/// cleanly; validators MUST gate writes to `Mode::ALLOWED` because constructing a
/// `Mode` from an unlisted discriminant is UB.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Mode {
    #[default]
    OpenLoop = 0,
    PositionPid = 1,
}

impl Mode {
    pub const ALLOWED: &[u8] = &[Mode::OpenLoop as u8, Mode::PositionPid as u8];
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ControlLifecycle {
    pub torque_enable: bool,
    pub mode: Mode,
    pub _rsvd_align: [u8; 2],
    pub goal_position: i32,
    pub goal_velocity: i32,
    pub goal_effort: i16,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ControlStreaming {
    pub stream_enable: bool,
    pub stream_decimation: u8,
    pub stream_duration_ms: u16,
    pub stream_field_mask: u32,
    pub stream_dropped: u32,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ControlRegs {
    pub lifecycle: ControlLifecycle,
    pub streaming: ControlStreaming,
}

impl ControlLifecycle {
    pub const fn const_new() -> Self {
        Self {
            torque_enable: false,
            mode: Mode::OpenLoop,
            _rsvd_align: [0; 2],
            goal_position: 0,
            goal_velocity: 0,
            goal_effort: 0,
        }
    }
}

impl ControlStreaming {
    pub const fn const_new() -> Self {
        Self {
            stream_enable: false,
            stream_decimation: 0,
            stream_duration_ms: 0,
            stream_field_mask: 0,
            stream_dropped: 0,
        }
    }
}

const LIFECYCLE_ADDR: u16 = CONTROL_BASE_ADDR;
const LIFECYCLE_STRUCT: u16 = offset_of!(ControlRegs, lifecycle) as u16;
const STREAMING_ADDR: u16 = CONTROL_BASE_ADDR + CONTROL_BLOCK_SIZE as u16;
const STREAMING_STRUCT: u16 = offset_of!(ControlRegs, streaming) as u16;

pub const CONTROL_FIELDS: &[FieldDesc] = &[
    // ControlLifecycle (skip _rsvd_align at +2..4 and trailing pad at +14..16)
    FieldDesc {
        addr: LIFECYCLE_ADDR + offset_of!(ControlLifecycle, torque_enable) as u16,
        size: 1,
        struct_offset: LIFECYCLE_STRUCT + offset_of!(ControlLifecycle, torque_enable) as u16,
        access: Access::Rw,
        validators: &[Validator::EnumU8 {
            allowed: BOOL_ALLOWED,
        }],
    },
    FieldDesc {
        addr: LIFECYCLE_ADDR + offset_of!(ControlLifecycle, mode) as u16,
        size: 1,
        struct_offset: LIFECYCLE_STRUCT + offset_of!(ControlLifecycle, mode) as u16,
        access: Access::Rw,
        validators: &[Validator::EnumU8 {
            allowed: Mode::ALLOWED,
        }],
    },
    FieldDesc {
        addr: LIFECYCLE_ADDR + offset_of!(ControlLifecycle, goal_position) as u16,
        size: 4,
        struct_offset: LIFECYCLE_STRUCT + offset_of!(ControlLifecycle, goal_position) as u16,
        access: Access::Rw,
        validators: &[],
    },
    FieldDesc {
        addr: LIFECYCLE_ADDR + offset_of!(ControlLifecycle, goal_velocity) as u16,
        size: 4,
        struct_offset: LIFECYCLE_STRUCT + offset_of!(ControlLifecycle, goal_velocity) as u16,
        access: Access::Rw,
        validators: &[],
    },
    FieldDesc {
        addr: LIFECYCLE_ADDR + offset_of!(ControlLifecycle, goal_effort) as u16,
        size: 2,
        struct_offset: LIFECYCLE_STRUCT + offset_of!(ControlLifecycle, goal_effort) as u16,
        access: Access::Rw,
        validators: &[],
    },
    // ControlStreaming
    FieldDesc {
        addr: STREAMING_ADDR + offset_of!(ControlStreaming, stream_enable) as u16,
        size: 1,
        struct_offset: STREAMING_STRUCT + offset_of!(ControlStreaming, stream_enable) as u16,
        access: Access::Rw,
        validators: &[Validator::EnumU8 {
            allowed: BOOL_ALLOWED,
        }],
    },
    FieldDesc {
        addr: STREAMING_ADDR + offset_of!(ControlStreaming, stream_decimation) as u16,
        size: 1,
        struct_offset: STREAMING_STRUCT + offset_of!(ControlStreaming, stream_decimation) as u16,
        access: Access::Rw,
        validators: &[],
    },
    FieldDesc {
        addr: STREAMING_ADDR + offset_of!(ControlStreaming, stream_duration_ms) as u16,
        size: 2,
        struct_offset: STREAMING_STRUCT + offset_of!(ControlStreaming, stream_duration_ms) as u16,
        access: Access::Rw,
        validators: &[Validator::RangeU16 {
            lo: 1,
            hi: u16::MAX,
        }],
    },
    FieldDesc {
        addr: STREAMING_ADDR + offset_of!(ControlStreaming, stream_field_mask) as u16,
        size: 4,
        struct_offset: STREAMING_STRUCT + offset_of!(ControlStreaming, stream_field_mask) as u16,
        access: Access::Rw,
        validators: &[],
    },
    FieldDesc {
        addr: STREAMING_ADDR + offset_of!(ControlStreaming, stream_dropped) as u16,
        size: 4,
        struct_offset: STREAMING_STRUCT + offset_of!(ControlStreaming, stream_dropped) as u16,
        access: Access::Ro,
        validators: &[],
    },
];

impl ControlRegs {
    pub const fn const_new() -> Self {
        Self {
            lifecycle: ControlLifecycle::const_new(),
            streaming: ControlStreaming::const_new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::CONTROL_BLOCK_SIZE;
    use core::mem::size_of;

    #[test]
    fn leaf_blocks_fit_block() {
        assert!(size_of::<ControlLifecycle>() <= CONTROL_BLOCK_SIZE);
        assert!(size_of::<ControlStreaming>() <= CONTROL_BLOCK_SIZE);
    }
}
