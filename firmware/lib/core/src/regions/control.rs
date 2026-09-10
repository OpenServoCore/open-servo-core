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
#[ct_block(hooks = crate::regions::hooks::ControlTableHookEvents)]
pub struct ControlLifecycle {
    pub torque_enable: bool,
    /// Let the motor stall on purpose: drops the stall trip and the endstop
    /// band, and NOTHING else - the current limit and the thermal derate
    /// still compose. Identification pushes into a hard stop to measure R and
    /// L, which is precisely what those two guards exist to prevent, and no
    /// soft-limit value can express it because a stop can sit AT the position
    /// rail. Lives in the control region, so it is RAM only: never saved, and
    /// a reboot clears it. A tool that dies mid-run cannot leave a servo
    /// unguarded.
    pub stall_permit: bool,
    /// TEL sample layout, one bit per field (`tel` module). `bits` rejects
    /// reserved bits; `max_ones` caps the field count at the wire budget.
    #[ct_field(bits = crate::tel::MASK_ALL, max_ones = crate::tel::FIELDS_MAX)]
    pub tel_mask: u16,
    pub mode: Mode,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    #[ct_field(le = &config::addr::loop_current::DUTY_MAX_Q15, abs)]
    pub goal_duty: i16,
    /// Phys-validated, soft-clamped: garbage outside the rails rejects; an
    /// out-of-soft goal runs to the soft wall (trajectory clamp) instead of
    /// bouncing the write.
    #[ct_field(
        ge = &config::addr::pos_limits::POS_MIN_PHYS_COUNTS,
        le = &config::addr::pos_limits::POS_MAX_PHYS_COUNTS,
    )]
    pub goal_position: i32,
    pub goal_velocity: i32,
    #[ct_field(le = &config::addr::limits::CURRENT_LIMIT_COUNTS, abs)]
    pub goal_current: i16,
    /// TEL burst arm: a committed nonzero write streams that many samples
    /// (one per fast tick, batched 16 per `Stream` frame), then stops and
    /// releases the line; 0 is disarmed. Composes with HOLD/COMMIT so a
    /// goal write and the arm apply in the same instant.
    #[ct_field(hook = on_tel_count_write)]
    pub tel_count: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ControlSystem {
    pub boot_mode: BootMode,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
}

/// Shunt-burst request. `arm` is level, not an edge: 1 asks for a capture, 0
/// releases the result back to Idle, so a host that dies mid-run leaves a
/// servo that only has to be told 0. `page` selects which slice of the
/// capture the BURST section publishes.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ControlBurst {
    #[ct_field(le = &config::addr::loop_current::DUTY_MAX_Q15, abs)]
    pub duty_q15: i16,
    pub arm: u8,
    pub page: u8,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CONTROL_BASE_ADDR,
    size = crate::regions::CONTROL_REGION_SIZE,
    hooks = crate::regions::hooks::ControlTableHookEvents,
)]
pub struct ControlRegs {
    pub lifecycle: ControlLifecycle,
    pub system: ControlSystem,
    pub burst: ControlBurst,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 102],
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
