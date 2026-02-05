//! DXL (Dynamixel-compatible) register definitions (0x00-0xFB / 0-251).
//!
//! This is the compatibility layer for Dynamixel Protocol 2.0 hosts.
//! Registers are either:
//! - Standalone (`#[reg(RO|RW)]`) - native DXL registers
//! - EEPROM (`#[reg(RW, eeprom)]`) - persisted to flash, locked when torque enabled
//! - Facade aliases (`#[reg(facade)]`) - translated to/from vendor registers
//! - Indirect access (`#[indirect_address(n)]` / `#[indirect_data(n)]`)
//!
//! Note: RO fields in the EEPROM region (model_number, etc.) come from BoardConfig
//! and are NOT persisted - they're written once at boot from board defaults.

use crate::spec::RegSpec;
use open_servo_macros::RegMap;

// ============================================================================
// Register Map Definition
// ============================================================================

#[derive(RegMap)]
#[regmap(base = 0, fields = "DXL_FIELDS")]
#[allow(dead_code)]
struct DxlRegs {
    // ========================================================================
    // EEPROM Region (0-63) - persisted, requires Torque Enable = 0 to write
    // ========================================================================
    #[reg(RO)]
    model_number: u16, // 0 - from BoardConfig
    #[reg(RO)]
    model_information: u32, // 2 - from BoardConfig
    #[reg(RO)]
    firmware_version: u8, // 6 - from BoardConfig
    #[reg(RW, eeprom)]
    id: u8, // 7
    #[reg(RW, eeprom)]
    baud_rate: u8, // 8
    #[reg(RW, eeprom)]
    return_delay_time: u8, // 9
    #[reg(RW, eeprom)]
    drive_mode: u8, // 10
    #[reg(RW, eeprom)]
    operating_mode: u8, // 11
    #[reg(RW, eeprom)]
    secondary_id: u8, // 12
    #[reg(RO)]
    protocol_type: u8, // 13 - from BoardConfig
    _reserved1: [u8; 6], // 14-19
    #[reg(RW, eeprom)]
    homing_offset: i32, // 20
    #[reg(RW, eeprom)]
    moving_threshold: u32, // 24
    _reserved2: [u8; 3], // 28-30
    #[reg(RW, eeprom)]
    temperature_limit: u8, // 31
    #[reg(RW, eeprom)]
    max_voltage_limit: u16, // 32
    #[reg(RW, eeprom)]
    min_voltage_limit: u16, // 34
    #[reg(RW, eeprom)]
    pwm_limit: u16, // 36
    #[reg(RW, eeprom)]
    current_limit: u16, // 38
    _reserved3: [u8; 4], // 40-43
    #[reg(RW, eeprom)]
    velocity_limit: u32, // 44
    #[reg(RW, eeprom)]
    max_position_limit: i32, // 48
    #[reg(RW, eeprom)]
    min_position_limit: i32, // 52
    _reserved4: [u8; 4], // 56-59
    #[reg(RW, eeprom)]
    startup_configuration: u8, // 60
    _reserved5: [u8; 1], // 61
    #[reg(RW, eeprom)]
    pwm_slope: u8, // 62
    _reserved6: [u8; 1], // 63

    // ========================================================================
    // RAM Region (64-167) - volatile, writable when torque enabled
    // ========================================================================

    // Control registers (64-70)
    #[reg(facade)]
    torque_enable: bool, // 64 - facade alias to vendor
    #[reg(RW)]
    led: u8, // 65
    _reserved7: [u8; 2], // 66-67
    #[reg(RW)]
    status_return_level: u8, // 68
    #[reg(RO)]
    registered_instruction: u8, // 69
    #[reg(RO)]
    hardware_error_status: u8, // 70

    // Reserved (71-75)
    _reserved8: [u8; 5], // 71-75

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
    _reserved9: [u8; 2], // 86-87
    #[reg(RW)]
    feedforward_2nd_gain: u16, // 88
    #[reg(RW)]
    feedforward_1st_gain: u16, // 90

    // Reserved (92-97)
    _reserved10: [u8; 6], // 92-97

    // Goal/profile (98-119)
    #[reg(RW)]
    bus_watchdog: u8, // 98
    _reserved11: [u8; 1], // 99
    #[reg(facade)]
    goal_pwm: i16, // 100 - facade alias to vendor
    #[reg(RW)]
    goal_current: i16, // 102
    #[reg(RW)]
    goal_velocity: i32, // 104
    #[reg(RW)]
    profile_acceleration: u32, // 108
    #[reg(RW)]
    profile_velocity: u32, // 112
    #[reg(facade)]
    goal_position: i32, // 116 - facade alias to vendor

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
    #[reg(facade)]
    present_position: i32, // 132 - facade alias to vendor
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
    _reserved12: [u8; 20], // 148-167

    // ========================================================================
    // Indirect Access Region (168-251)
    // ========================================================================
    #[indirect_address(1)]
    indirect_address_1: [u16; 28], // 168-223

    #[indirect_data(1)]
    indirect_data_1: [u8; 28], // 224-251
}

// ============================================================================
// Lookup
// ============================================================================

/// Lookup field by name (case-insensitive prefix match).
pub fn find(name: &str) -> Option<&'static RegSpec> {
    for field in DXL_FIELDS {
        if starts_with_ignore_case(field.name, name) {
            return Some(field);
        }
    }
    None
}

/// Lookup field by address.
pub fn find_by_address(address: u16) -> Option<&'static RegSpec> {
    for field in DXL_FIELDS {
        if field.address == address {
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
    use crate::spec::Access;

    #[test]
    fn test_find_exact() {
        let spec = find("model_number").unwrap();
        assert_eq!(spec.address, 0);
        assert_eq!(spec.len(), 2);
    }

    #[test]
    fn test_find_prefix() {
        let spec = find("torque").unwrap();
        assert_eq!(spec.name, "torque_enable");
        assert_eq!(spec.address, 64);
    }

    #[test]
    fn test_eeprom_fields() {
        // Writable EEPROM field - persisted to flash
        let spec = find("id").unwrap();
        assert_eq!(spec.access, Access::RW);
        assert!(spec.is_eeprom());

        // Read-only field in EEPROM region - NOT persisted (comes from BoardConfig)
        let spec = find("model_number").unwrap();
        assert_eq!(spec.access, Access::RO);
        assert!(!spec.is_eeprom()); // RO fields are not persisted

        // RAM field (not EEPROM)
        let spec = find("led").unwrap();
        assert_eq!(spec.access, Access::RW);
        assert!(!spec.is_eeprom());
    }

    #[test]
    fn test_facade_access() {
        let spec = find("goal_position").unwrap();
        assert_eq!(spec.access, Access::Facade);
        assert_eq!(spec.address, 116);

        let spec = find("present_position").unwrap();
        assert_eq!(spec.access, Access::Facade);
        assert_eq!(spec.address, 132);

        let spec = find("torque_enable").unwrap();
        assert_eq!(spec.access, Access::Facade);
        assert_eq!(spec.address, 64);

        let spec = find("goal_pwm").unwrap();
        assert_eq!(spec.access, Access::Facade);
        assert_eq!(spec.address, 100);
    }

    #[test]
    fn test_addresses_computed() {
        // EEPROM
        assert_eq!(addr::MODEL_NUMBER, 0);
        assert_eq!(addr::ID, 7);
        assert_eq!(addr::HOMING_OFFSET, 20);

        // RAM
        assert_eq!(addr::TORQUE_ENABLE, 64);
        assert_eq!(addr::GOAL_PWM, 100);
        assert_eq!(addr::GOAL_POSITION, 116);
        assert_eq!(addr::PRESENT_POSITION, 132);

        // Indirect
        assert_eq!(addr::INDIRECT_ADDRESS_1, 168);
        assert_eq!(addr::INDIRECT_DATA_1, 224);
    }

    #[test]
    fn test_indirect_banks() {
        assert_eq!(INDIRECT_BANKS.len(), 1);
        let bank = &INDIRECT_BANKS[0];
        assert_eq!(bank.bank, 1);
        assert_eq!(bank.addr_base, 168);
        assert_eq!(bank.data_base, 224);
        assert_eq!(bank.slots, 28);
    }

    #[test]
    fn test_find_by_address() {
        let spec = find_by_address(116).unwrap();
        assert_eq!(spec.name, "goal_position");

        let spec = find_by_address(0).unwrap();
        assert_eq!(spec.name, "model_number");
    }
}
