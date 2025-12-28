//! Shadow table field offsets for ServoKernel.
//!
//! These offsets define the byte layout within shadow table regions.
//! The layout is kernel-specific (not part of the API contract).

use open_servo_kernel_api::shadow::FieldDesc;

/// Control region field offsets (relative to CTRL_START = 0x80).
///
/// Host writes here; kernel reads and commits to live state.
pub mod ctrl {
    use super::FieldDesc;

    /// Engaged flag (u8: 0=disengaged, 1=engaged).
    pub const ENGAGED: FieldDesc = FieldDesc::new(0x80, 1);

    /// Operating mode (u8: 0=Position, 1=OpenLoop).
    pub const MODE: FieldDesc = FieldDesc::new(0x81, 1);

    /// Position setpoint (i32 LE, centi-degrees).
    pub const GOAL_POS: FieldDesc = FieldDesc::new(0x84, 4);

    /// Open-loop effort (i16 LE, raw effort).
    pub const OPEN_LOOP_EFFORT: FieldDesc = FieldDesc::new(0x88, 2);
}

/// Telemetry region field offsets (relative to TELEM_START = 0x00).
///
/// Kernel writes here; host reads.
pub mod telem {
    /// Current position (i32 LE, centi-degrees).
    pub const POS_CDEG32: u16 = 0x00;

    /// Last commanded effort (i16 LE, raw).
    pub const EFFORT_RAW: u16 = 0x04;

    /// Engaged state mirror (u8).
    pub const ENGAGED: u16 = 0x06;

    /// Operating mode mirror (u8).
    pub const MODE: u16 = 0x07;

    /// Fault mask (u32 LE, bitfield).
    pub const FAULT_MASK: u16 = 0x08;

    /// Gate reason (u8).
    pub const GATE_REASON: u16 = 0x0C;
}
