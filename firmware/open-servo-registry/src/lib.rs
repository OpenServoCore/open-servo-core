//! Register map definitions for servo kernels.
//!
//! This crate provides field specifications and lookup functions for
//! services (REPL, Dynamixel bridge, debug tools) to access shadow table fields.
//!
//! # Register Layout
//!
//! | Range | Size | Region | Description |
//! |-------|------|--------|-------------|
//! | 0x000-0x0FB | 252 | DXL | Dynamixel Protocol 2.0 compat layer |
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
//!
//! // Typed register access
//! use open_servo_registry::{Reg, RW, ViewWrite};
//! const GOAL_POS: Reg<RW, i32> = Reg::new(512);
//! GOAL_POS.write(&mut view, 1000)?;
//! ```

#![no_std]

pub mod dxl;
pub mod facade;
pub mod reg;
pub mod spec;
pub mod vendor;
pub mod view;

// Re-export main types
pub use spec::{Access, Encoding, IndirectBankSpec, MapEntry, RangeSpec, RegSpec, UnitEncoding};

// Re-export typed register access
pub use reg::{Facade, Reg, RO, RW, RWE, WO};
pub use view::{ViewRead, ViewWrite};

// Re-export vendor module items for kernel use
pub use vendor::{ctrl, telem, VENDOR_FIELDS};

// Re-export DXL module items
pub use dxl::{DXL_FIELDS, INDIRECT_BANKS};

/// Maximum protocol address (1024-byte address space).
pub const MAX_ADDRESS: u16 = 1024;

/// DXL region start address.
pub const DXL_START: u16 = 0x000;

/// DXL region end address (exclusive).
pub const DXL_END: u16 = 0x0FC;

/// Vendor region start address.
pub const VENDOR_START: u16 = 0x200;

// ============================================================================
// Unified Lookup
// ============================================================================

/// Lookup field by name (case-insensitive prefix match) across all regions.
///
/// Search order: dxl, vendor (standard Dynamixel first for compatibility).
pub fn find(name: &str) -> Option<&'static RegSpec> {
    dxl::find(name).or_else(|| vendor::find(name))
}

/// Lookup field by address across all regions.
pub fn find_by_address(address: u16) -> Option<&'static RegSpec> {
    // Check vendor region
    for field in vendor::VENDOR_FIELDS {
        if field.address == address {
            return Some(field);
        }
    }
    // Check DXL region
    for field in dxl::DXL_FIELDS {
        if field.address == address {
            return Some(field);
        }
    }
    None
}

/// Get all fields from all regions.
pub fn all_fields() -> impl Iterator<Item = &'static RegSpec> {
    dxl::DXL_FIELDS.iter().chain(vendor::VENDOR_FIELDS.iter())
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
    fn test_find_dxl() {
        let spec = find("torque_enable").unwrap();
        assert_eq!(spec.address, 64);
    }

    #[test]
    fn test_find_dxl_eeprom() {
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
        // Should have all DXL + Vendor fields
        let expected = dxl::DXL_FIELDS.len() + vendor::VENDOR_FIELDS.len();
        assert_eq!(count, expected);
    }
}
