//! EEPROM register definitions (0x00-0x3F / 0-63).
//!
//! These registers are persisted and require Torque Enable = 0 to write.

use crate::spec::RegSpec;
use open_servo_macros::RegMap;

// ============================================================================
// Register Map Definition
// ============================================================================

#[derive(RegMap)]
#[regmap(base = 0, fields = "EEPROM_FIELDS")]
#[allow(dead_code)]
struct EepromRegs {
    #[reg(RO)]
    model_number: u16, // 0
    #[reg(RO)]
    model_information: u32, // 2
    #[reg(RO)]
    firmware_version: u8, // 6
    #[reg(RWE)]
    id: u8, // 7
    #[reg(RWE)]
    baud_rate: u8, // 8
    #[reg(RWE)]
    return_delay_time: u8, // 9
    #[reg(RWE)]
    drive_mode: u8, // 10
    #[reg(RWE)]
    operating_mode: u8, // 11
    #[reg(RWE)]
    secondary_id: u8, // 12
    #[reg(RO)]
    protocol_type: u8, // 13
    _reserved1: [u8; 6], // 14-19
    #[reg(RWE)]
    homing_offset: i32, // 20
    #[reg(RWE)]
    moving_threshold: u32, // 24
    _reserved2: [u8; 3], // 28-30
    #[reg(RWE)]
    temperature_limit: u8, // 31
    #[reg(RWE)]
    max_voltage_limit: u16, // 32
    #[reg(RWE)]
    min_voltage_limit: u16, // 34
    #[reg(RWE)]
    pwm_limit: u16, // 36
    #[reg(RWE)]
    current_limit: u16, // 38
    _reserved3: [u8; 4], // 40-43
    #[reg(RWE)]
    velocity_limit: u32, // 44
    #[reg(RWE)]
    max_position_limit: i32, // 48
    #[reg(RWE)]
    min_position_limit: i32, // 52
    _reserved4: [u8; 4], // 56-59
    #[reg(RWE)]
    startup_configuration: u8, // 60
    _reserved5: [u8; 1], // 61
    #[reg(RWE)]
    pwm_slope: u8, // 62
}

// ============================================================================
// Lookup
// ============================================================================

/// Lookup field by name (case-insensitive prefix match).
pub fn find(name: &str) -> Option<&'static RegSpec> {
    for field in EEPROM_FIELDS {
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
    use crate::spec::Access;

    #[test]
    fn test_find_exact() {
        let spec = find("model_number").unwrap();
        assert_eq!(spec.address, 0);
        assert_eq!(spec.len(), 2);
    }

    #[test]
    fn test_find_prefix() {
        let spec = find("baud").unwrap();
        assert_eq!(spec.name, "baud_rate");
        assert_eq!(spec.address, 8);
    }

    #[test]
    fn test_eeprom_access() {
        let spec = find("id").unwrap();
        assert_eq!(spec.access, Access::RWE);

        let spec = find("model_number").unwrap();
        assert_eq!(spec.access, Access::RO);
    }

    #[test]
    fn test_addresses_computed() {
        assert_eq!(addr::MODEL_NUMBER, 0);
        assert_eq!(addr::MODEL_INFORMATION, 2);
        assert_eq!(addr::FIRMWARE_VERSION, 6);
        assert_eq!(addr::ID, 7);
        assert_eq!(addr::HOMING_OFFSET, 20);
        assert_eq!(addr::TEMPERATURE_LIMIT, 31);
        assert_eq!(addr::VELOCITY_LIMIT, 44);
        assert_eq!(addr::STARTUP_CONFIGURATION, 60);
        assert_eq!(addr::PWM_SLOPE, 62);
    }

    #[test]
    fn test_reserved_fields() {
        // Find a reserved field
        let reserved = EEPROM_FIELDS
            .iter()
            .find(|f| f.name == "_reserved1")
            .unwrap();
        assert_eq!(reserved.address, 14);
        assert_eq!(reserved.len(), 6);
        assert_eq!(reserved.access, Access::Reserved);
    }
}
