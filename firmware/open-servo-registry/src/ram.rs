//! RAM register definitions (0x40-0xFB / 64-251).
//!
//! These registers are volatile and writable when torque is enabled.

use crate::spec::{Access, Encoding, RangeSpec, RegSpec};
use open_servo_macros::RegMap;

// ============================================================================
// Register Map Definition
// ============================================================================

#[derive(RegMap)]
#[regmap(base = 64, fields = "RAM_FIELDS")]
#[allow(dead_code)]
struct RamRegs {
    // Control registers (64-70)
    #[reg(RW)]
    torque_enable: bool, // 64
    #[reg(RW)]
    led: u8, // 65
    _reserved1: [u8; 2], // 66-67
    #[reg(RW)]
    status_return_level: u8, // 68
    #[reg(RO)]
    registered_instruction: u8, // 69
    #[reg(RO)]
    hardware_error_status: u8, // 70

    // Reserved (71-75)
    _reserved2: [u8; 5], // 71-75

    // PID gains (76-91)
    #[reg(RW)]
    velocity_i_gain: u16, // 76
    #[reg(RW)]
    velocity_p_gain: u16, // 78
    #[reg(RW)]
    position_d_gain: u16, // 80
    #[reg(RW)]
    position_i_gain: u16, // 82
    #[reg(RW)]
    position_p_gain: u16, // 84
    _reserved3: [u8; 2], // 86-87
    #[reg(RW)]
    feedforward_2nd_gain: u16, // 88
    #[reg(RW)]
    feedforward_1st_gain: u16, // 90

    // Reserved (92-97)
    _reserved4: [u8; 6], // 92-97

    // Goal/profile (98-119)
    #[reg(RW)]
    bus_watchdog: u8, // 98
    _reserved5: [u8; 1], // 99
    #[reg(RW)]
    goal_pwm: i16, // 100
    #[reg(RW)]
    goal_current: i16, // 102
    #[reg(RW)]
    goal_velocity: i32, // 104
    #[reg(RW)]
    profile_acceleration: u32, // 108
    #[reg(RW)]
    profile_velocity: u32, // 112
    #[reg(RW)]
    goal_position: i32, // 116

    // Telemetry (120-147)
    #[reg(RO)]
    realtime_tick: u16, // 120
    #[reg(RO)]
    moving: bool, // 122
    #[reg(RO)]
    moving_status: u8, // 123
    #[reg(RO)]
    present_pwm: i16, // 124
    #[reg(RO)]
    present_current: i16, // 126
    #[reg(RO)]
    present_velocity: i32, // 128
    #[reg(RO)]
    present_position: i32, // 132
    #[reg(RO)]
    velocity_trajectory: i32, // 136
    #[reg(RO)]
    position_trajectory: i32, // 140
    #[reg(RO)]
    present_input_voltage: u16, // 144
    #[reg(RO)]
    present_temperature: u8, // 146
    #[reg(RO)]
    backup_ready: u8, // 147

    // Reserved (148-167)
    _reserved6: [u8; 20], // 148-167

                          // Indirect access ranges handled separately below
}

// ============================================================================
// Indirect Access Ranges
// ============================================================================

/// Indirect Address registers (28 x u16) starting at 168.
pub const INDIRECT_ADDRESS: RangeSpec =
    RangeSpec::new("indirect_addr", 168, 28, Encoding::U16Le, Access::RW);

/// Indirect Data registers (28 x u8) starting at 224.
pub const INDIRECT_DATA: RangeSpec =
    RangeSpec::new("indirect_data", 224, 28, Encoding::U8, Access::RW);

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
        let spec = find("torque_enable").unwrap();
        assert_eq!(spec.access, Access::RW);

        let spec = find("present_position").unwrap();
        assert_eq!(spec.access, Access::RO);
    }

    #[test]
    fn test_addresses_computed() {
        assert_eq!(addr::TORQUE_ENABLE, 64);
        assert_eq!(addr::LED, 65);
        assert_eq!(addr::STATUS_RETURN_LEVEL, 68);
        assert_eq!(addr::VELOCITY_I_GAIN, 76);
        assert_eq!(addr::GOAL_PWM, 100);
        assert_eq!(addr::GOAL_POSITION, 116);
        assert_eq!(addr::PRESENT_POSITION, 132);
        assert_eq!(addr::PRESENT_TEMPERATURE, 146);
    }
}
