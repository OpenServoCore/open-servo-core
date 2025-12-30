//! Register map definitions for servo kernels.
//!
//! This crate provides field specifications and lookup functions for
//! services (REPL, Dynamixel bridge, debug tools) to access shadow table fields.
//!
//! # Register Layout
//!
//! | Range | Size | Region | Description |
//! |-------|------|--------|-------------|
//! | 0x000-0x03F | 64 | EEPROM | Persisted, locked when torque on |
//! | 0x040-0x0FB | 188 | RAM | Volatile, writable |
//! | 0x0FC-0x1FF | 260 | Reserved | Future use |
//! | 0x200-0x3FF | 512 | Vendor | High-res kernel registers |
//!
//! # Usage
//!
//! ```ignore
//! use open_servo_registry::{find, RegSpec, Encoding};
//!
//! // Lookup by name prefix (searches all regions)
//! if let Some(spec) = find("goal_pos") {
//!     println!("{}: addr={:#x}, len={}", spec.name, spec.address, spec.len());
//! }
//!
//! // Region-specific lookup
//! use open_servo_registry::vendor;
//! if let Some(spec) = vendor::find("goal_pos_cdeg") {
//!     println!("Vendor: {}", spec.name);
//! }
//! ```

#![no_std]

pub mod eeprom;
pub mod ram;
pub mod spec;
pub mod vendor;

// Re-export main types
pub use spec::{Access, Encoding, RangeSpec, RegSpec};

// Re-export vendor module items for kernel use
pub use vendor::{ctrl, telem, VENDOR_FIELDS};

/// Maximum protocol address (1024-byte address space).
pub const MAX_ADDRESS: u16 = 1024;

/// EEPROM region start address.
pub const EEPROM_START: u16 = 0x000;

/// RAM region start address.
pub const RAM_START: u16 = 0x040;

/// Vendor region start address.
pub const VENDOR_START: u16 = 0x200;

// ============================================================================
// Unified Lookup
// ============================================================================

/// Lookup field by name (case-insensitive prefix match) across all regions.
///
/// Search order: eeprom, ram, vendor (standard Dynamixel first for compatibility).
pub fn find(name: &str) -> Option<&'static RegSpec> {
    eeprom::find(name)
        .or_else(|| ram::find(name))
        .or_else(|| vendor::find(name))
}

/// Lookup field by address across all regions.
pub fn find_by_address(address: u16) -> Option<&'static RegSpec> {
    // Check vendor region
    for field in vendor::VENDOR_FIELDS {
        if field.address == address {
            return Some(field);
        }
    }
    // Check RAM region
    for field in ram::RAM_FIELDS {
        if field.address == address {
            return Some(field);
        }
    }
    // Check EEPROM region
    for field in eeprom::EEPROM_FIELDS {
        if field.address == address {
            return Some(field);
        }
    }
    None
}

/// Get all fields from all regions.
pub fn all_fields() -> impl Iterator<Item = &'static RegSpec> {
    eeprom::EEPROM_FIELDS
        .iter()
        .chain(ram::RAM_FIELDS.iter())
        .chain(vendor::VENDOR_FIELDS.iter())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_vendor() {
        let spec = find("goal_pos_cdeg").unwrap();
        assert_eq!(spec.address, 512);
    }

    #[test]
    fn test_find_ram() {
        let spec = find("torque_enable").unwrap();
        assert_eq!(spec.address, 64);
    }

    #[test]
    fn test_find_eeprom() {
        let spec = find("model_number").unwrap();
        assert_eq!(spec.address, 0);
    }

    #[test]
    fn test_find_by_address() {
        let spec = find_by_address(512).unwrap();
        assert_eq!(spec.name, "goal_pos_cdeg");

        let spec = find_by_address(64).unwrap();
        assert_eq!(spec.name, "torque_enable");

        let spec = find_by_address(0).unwrap();
        assert_eq!(spec.name, "model_number");
    }

    #[test]
    fn test_all_fields_count() {
        let count = all_fields().count();
        // Should have all EEPROM + RAM + Vendor fields
        let expected =
            eeprom::EEPROM_FIELDS.len() + ram::RAM_FIELDS.len() + vendor::VENDOR_FIELDS.len();
        assert_eq!(count, expected);
    }
}
