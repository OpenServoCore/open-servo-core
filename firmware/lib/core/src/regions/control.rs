//! CONTROL region — RW volatile runtime commands from the host.

use crate::regions::CONTROL_BLOCK_SIZE;
use crate::regmap::{Access, BlockDesc};
use core::mem::{offset_of, size_of};

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ControlLifecycle {
    pub torque_enable: u8,
    pub mode: u8,
    pub _rsvd_align: [u8; 2],
    pub goal_position: i32,
    pub goal_velocity: i32,
    pub goal_effort: i16,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ControlStreaming {
    pub stream_enable: u8,
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
            torque_enable: 0,
            mode: 0,
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
            stream_enable: 0,
            stream_decimation: 0,
            stream_duration_ms: 0,
            stream_field_mask: 0,
            stream_dropped: 0,
        }
    }
}

/// Protocol-address slot map for CONTROL. Both blocks are host-RW.
pub const CONTROL_BLOCKS: &[BlockDesc] = &[
    BlockDesc {
        addr_offset: 0 * CONTROL_BLOCK_SIZE as u16,
        size: size_of::<ControlLifecycle>() as u16,
        struct_offset: offset_of!(ControlRegs, lifecycle) as u16,
        access: Access::Rw,
    },
    BlockDesc {
        addr_offset: 1 * CONTROL_BLOCK_SIZE as u16,
        size: size_of::<ControlStreaming>() as u16,
        struct_offset: offset_of!(ControlRegs, streaming) as u16,
        access: Access::Rw,
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
