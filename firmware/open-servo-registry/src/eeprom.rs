//! EEPROM register definitions (0x00-0x3F / 0-63).
//!
//! These registers are persisted and require Torque Enable = 0 to write.

use crate::spec::{Access, Encoding, RegSpec};

// ============================================================================
// EEPROM Register Addresses (from REGISTER_MAP.md)
// ============================================================================

pub mod addr {
    pub const MODEL_NUMBER: u16 = 0;
    pub const MODEL_INFORMATION: u16 = 2;
    pub const FIRMWARE_VERSION: u16 = 6;
    pub const ID: u16 = 7;
    pub const BAUD_RATE: u16 = 8;
    pub const RETURN_DELAY_TIME: u16 = 9;
    pub const DRIVE_MODE: u16 = 10;
    pub const OPERATING_MODE: u16 = 11;
    pub const SECONDARY_ID: u16 = 12;
    pub const PROTOCOL_TYPE: u16 = 13;
    pub const HOMING_OFFSET: u16 = 20;
    pub const MOVING_THRESHOLD: u16 = 24;
    pub const TEMPERATURE_LIMIT: u16 = 31;
    pub const MAX_VOLTAGE_LIMIT: u16 = 32;
    pub const MIN_VOLTAGE_LIMIT: u16 = 34;
    pub const PWM_LIMIT: u16 = 36;
    pub const CURRENT_LIMIT: u16 = 38;
    pub const VELOCITY_LIMIT: u16 = 44;
    pub const MAX_POSITION_LIMIT: u16 = 48;
    pub const MIN_POSITION_LIMIT: u16 = 52;
    pub const STARTUP_CONFIGURATION: u16 = 60;
    pub const PWM_SLOPE: u16 = 62;
}

// ============================================================================
// Register Specifications
// ============================================================================

/// All EEPROM region fields.
pub const EEPROM_FIELDS: &[RegSpec] = &[
    RegSpec::new("model_number", addr::MODEL_NUMBER, Encoding::U16Le, Access::R),
    RegSpec::new("model_information", addr::MODEL_INFORMATION, Encoding::U32Le, Access::R),
    RegSpec::new("firmware_version", addr::FIRMWARE_VERSION, Encoding::U8, Access::R),
    RegSpec::new("id", addr::ID, Encoding::U8, Access::RwEepromLocked),
    RegSpec::new("baud_rate", addr::BAUD_RATE, Encoding::U8, Access::RwEepromLocked),
    RegSpec::new("return_delay_time", addr::RETURN_DELAY_TIME, Encoding::U8, Access::RwEepromLocked),
    RegSpec::new("drive_mode", addr::DRIVE_MODE, Encoding::U8, Access::RwEepromLocked),
    RegSpec::new("operating_mode", addr::OPERATING_MODE, Encoding::U8, Access::RwEepromLocked),
    RegSpec::new("secondary_id", addr::SECONDARY_ID, Encoding::U8, Access::RwEepromLocked),
    RegSpec::new("protocol_type", addr::PROTOCOL_TYPE, Encoding::U8, Access::R),
    RegSpec::new("homing_offset", addr::HOMING_OFFSET, Encoding::I32Le, Access::RwEepromLocked),
    RegSpec::new("moving_threshold", addr::MOVING_THRESHOLD, Encoding::U32Le, Access::RwEepromLocked),
    RegSpec::new("temperature_limit", addr::TEMPERATURE_LIMIT, Encoding::U8, Access::RwEepromLocked),
    RegSpec::new("max_voltage_limit", addr::MAX_VOLTAGE_LIMIT, Encoding::U16Le, Access::RwEepromLocked),
    RegSpec::new("min_voltage_limit", addr::MIN_VOLTAGE_LIMIT, Encoding::U16Le, Access::RwEepromLocked),
    RegSpec::new("pwm_limit", addr::PWM_LIMIT, Encoding::U16Le, Access::RwEepromLocked),
    RegSpec::new("current_limit", addr::CURRENT_LIMIT, Encoding::U16Le, Access::RwEepromLocked),
    RegSpec::new("velocity_limit", addr::VELOCITY_LIMIT, Encoding::U32Le, Access::RwEepromLocked),
    RegSpec::new("max_position_limit", addr::MAX_POSITION_LIMIT, Encoding::I32Le, Access::RwEepromLocked),
    RegSpec::new("min_position_limit", addr::MIN_POSITION_LIMIT, Encoding::I32Le, Access::RwEepromLocked),
    RegSpec::new("startup_configuration", addr::STARTUP_CONFIGURATION, Encoding::U8, Access::RwEepromLocked),
    RegSpec::new("pwm_slope", addr::PWM_SLOPE, Encoding::U8, Access::RwEepromLocked),
];

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
        // Most EEPROM fields are RwEepromLocked
        let spec = find("id").unwrap();
        assert_eq!(spec.access, Access::RwEepromLocked);

        // But some are read-only
        let spec = find("model_number").unwrap();
        assert_eq!(spec.access, Access::R);
    }
}
