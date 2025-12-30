//! RAM register definitions (0x40-0xFB / 64-251).
//!
//! These registers are volatile and writable when torque is enabled.

use crate::spec::{Access, Encoding, RangeSpec, RegSpec};

// ============================================================================
// RAM Register Addresses (from REGISTER_MAP.md)
// ============================================================================

pub mod addr {
    // Control registers
    pub const TORQUE_ENABLE: u16 = 64;
    pub const LED: u16 = 65;
    pub const STATUS_RETURN_LEVEL: u16 = 68;
    pub const REGISTERED_INSTRUCTION: u16 = 69;
    pub const HARDWARE_ERROR_STATUS: u16 = 70;

    // PID gains
    pub const VELOCITY_I_GAIN: u16 = 76;
    pub const VELOCITY_P_GAIN: u16 = 78;
    pub const POSITION_D_GAIN: u16 = 80;
    pub const POSITION_I_GAIN: u16 = 82;
    pub const POSITION_P_GAIN: u16 = 84;
    pub const FEEDFORWARD_2ND_GAIN: u16 = 88;
    pub const FEEDFORWARD_1ST_GAIN: u16 = 90;

    // Goal/profile
    pub const BUS_WATCHDOG: u16 = 98;
    pub const GOAL_PWM: u16 = 100;
    pub const GOAL_CURRENT: u16 = 102;
    pub const GOAL_VELOCITY: u16 = 104;
    pub const PROFILE_ACCELERATION: u16 = 108;
    pub const PROFILE_VELOCITY: u16 = 112;
    pub const GOAL_POSITION: u16 = 116;

    // Telemetry
    pub const REALTIME_TICK: u16 = 120;
    pub const MOVING: u16 = 122;
    pub const MOVING_STATUS: u16 = 123;
    pub const PRESENT_PWM: u16 = 124;
    pub const PRESENT_CURRENT: u16 = 126;
    pub const PRESENT_VELOCITY: u16 = 128;
    pub const PRESENT_POSITION: u16 = 132;
    pub const VELOCITY_TRAJECTORY: u16 = 136;
    pub const POSITION_TRAJECTORY: u16 = 140;
    pub const PRESENT_INPUT_VOLTAGE: u16 = 144;
    pub const PRESENT_TEMPERATURE: u16 = 146;
    pub const BACKUP_READY: u16 = 147;

    // Indirect access
    pub const INDIRECT_ADDRESS_BASE: u16 = 168;
    pub const INDIRECT_DATA_BASE: u16 = 224;
}

// ============================================================================
// Register Specifications
// ============================================================================

/// All RAM region fields (excluding indirect ranges).
pub const RAM_FIELDS: &[RegSpec] = &[
    // Control registers
    RegSpec::new("torque_enable", addr::TORQUE_ENABLE, Encoding::Bool, Access::Rw),
    RegSpec::new("led", addr::LED, Encoding::U8, Access::Rw),
    RegSpec::new("status_return_level", addr::STATUS_RETURN_LEVEL, Encoding::U8, Access::Rw),
    RegSpec::new("registered_instruction", addr::REGISTERED_INSTRUCTION, Encoding::U8, Access::R),
    RegSpec::new("hardware_error_status", addr::HARDWARE_ERROR_STATUS, Encoding::U8, Access::R),

    // PID gains
    RegSpec::new("velocity_i_gain", addr::VELOCITY_I_GAIN, Encoding::U16Le, Access::Rw),
    RegSpec::new("velocity_p_gain", addr::VELOCITY_P_GAIN, Encoding::U16Le, Access::Rw),
    RegSpec::new("position_d_gain", addr::POSITION_D_GAIN, Encoding::U16Le, Access::Rw),
    RegSpec::new("position_i_gain", addr::POSITION_I_GAIN, Encoding::U16Le, Access::Rw),
    RegSpec::new("position_p_gain", addr::POSITION_P_GAIN, Encoding::U16Le, Access::Rw),
    RegSpec::new("feedforward_2nd_gain", addr::FEEDFORWARD_2ND_GAIN, Encoding::U16Le, Access::Rw),
    RegSpec::new("feedforward_1st_gain", addr::FEEDFORWARD_1ST_GAIN, Encoding::U16Le, Access::Rw),

    // Goal/profile
    RegSpec::new("bus_watchdog", addr::BUS_WATCHDOG, Encoding::U8, Access::Rw),
    RegSpec::new("goal_pwm", addr::GOAL_PWM, Encoding::I16Le, Access::Rw),
    RegSpec::new("goal_current", addr::GOAL_CURRENT, Encoding::I16Le, Access::Rw),
    RegSpec::new("goal_velocity", addr::GOAL_VELOCITY, Encoding::I32Le, Access::Rw),
    RegSpec::new("profile_acceleration", addr::PROFILE_ACCELERATION, Encoding::U32Le, Access::Rw),
    RegSpec::new("profile_velocity", addr::PROFILE_VELOCITY, Encoding::U32Le, Access::Rw),
    RegSpec::new("goal_position", addr::GOAL_POSITION, Encoding::I32Le, Access::Rw),

    // Telemetry
    RegSpec::new("realtime_tick", addr::REALTIME_TICK, Encoding::U16Le, Access::R),
    RegSpec::new("moving", addr::MOVING, Encoding::Bool, Access::R),
    RegSpec::new("moving_status", addr::MOVING_STATUS, Encoding::U8, Access::R),
    RegSpec::new("present_pwm", addr::PRESENT_PWM, Encoding::I16Le, Access::R),
    RegSpec::new("present_current", addr::PRESENT_CURRENT, Encoding::I16Le, Access::R),
    RegSpec::new("present_velocity", addr::PRESENT_VELOCITY, Encoding::I32Le, Access::R),
    RegSpec::new("present_position", addr::PRESENT_POSITION, Encoding::I32Le, Access::R),
    RegSpec::new("velocity_trajectory", addr::VELOCITY_TRAJECTORY, Encoding::I32Le, Access::R),
    RegSpec::new("position_trajectory", addr::POSITION_TRAJECTORY, Encoding::I32Le, Access::R),
    RegSpec::new("present_input_voltage", addr::PRESENT_INPUT_VOLTAGE, Encoding::U16Le, Access::R),
    RegSpec::new("present_temperature", addr::PRESENT_TEMPERATURE, Encoding::U8, Access::R),
    RegSpec::new("backup_ready", addr::BACKUP_READY, Encoding::U8, Access::R),
];

// ============================================================================
// Indirect Access Ranges
// ============================================================================

/// Indirect Address registers (28 x u16).
pub const INDIRECT_ADDRESS: RangeSpec = RangeSpec::new(
    "indirect_addr",
    addr::INDIRECT_ADDRESS_BASE,
    28,
    Encoding::U16Le,
    Access::Rw,
);

/// Indirect Data registers (28 x u8).
pub const INDIRECT_DATA: RangeSpec = RangeSpec::new(
    "indirect_data",
    addr::INDIRECT_DATA_BASE,
    28,
    Encoding::U8,
    Access::Rw,
);

// ============================================================================
// Lookup
// ============================================================================

/// Lookup field by name (case-insensitive prefix match).
pub fn find(name: &str) -> Option<&'static RegSpec> {
    for field in RAM_FIELDS {
        if starts_with_ignore_case(field.name, name) {
            return Some(field);
        }
    }
    None
}

/// Check if `haystack` starts with `needle` (ASCII case-insensitive).
fn starts_with_ignore_case(haystack: &str, needle: &str) -> bool {
    if needle.len() > haystack.len() {
        return false;
    }
    haystack
        .as_bytes()
        .iter()
        .zip(needle.as_bytes().iter())
        .all(|(&h, &n)| h.to_ascii_lowercase() == n.to_ascii_lowercase())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_exact() {
        let spec = find("torque_enable").unwrap();
        assert_eq!(spec.address, 64);
        assert_eq!(spec.len(), 1);
    }

    #[test]
    fn test_find_prefix() {
        let spec = find("present_pos").unwrap();
        assert_eq!(spec.name, "present_position");
        assert_eq!(spec.address, 132);
    }

    #[test]
    fn test_indirect_address_range() {
        assert_eq!(INDIRECT_ADDRESS.address_of(0), 168);
        assert_eq!(INDIRECT_ADDRESS.address_of(1), 170);
        assert_eq!(INDIRECT_ADDRESS.address_of(27), 168 + 27 * 2);
        assert_eq!(INDIRECT_ADDRESS.entry_len(), 2);
    }

    #[test]
    fn test_indirect_data_range() {
        assert_eq!(INDIRECT_DATA.address_of(0), 224);
        assert_eq!(INDIRECT_DATA.address_of(1), 225);
        assert_eq!(INDIRECT_DATA.address_of(27), 224 + 27);
        assert_eq!(INDIRECT_DATA.entry_len(), 1);
    }

    #[test]
    fn test_ram_access() {
        // Control fields are Rw
        let spec = find("torque_enable").unwrap();
        assert_eq!(spec.access, Access::Rw);

        // Telemetry fields are R
        let spec = find("present_position").unwrap();
        assert_eq!(spec.access, Access::R);
    }
}
