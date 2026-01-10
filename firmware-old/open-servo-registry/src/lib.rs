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
//! # EEPROM Address Convention
//!
//! EEPROM fields must be placed in one of two regions for O(1) bitmap checks:
//! - **DXL EEPROM**: addresses 0-63
//! - **Vendor EEPROM**: addresses 512-575
//!
//! This allows `touches_eeprom()` to use two 64-bit bitmaps (16 bytes total)
//! instead of iterating through all EEPROM fields.
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
//!
//! // Initialize registry (call once at startup)
//! open_servo_registry::init();
//! ```

#![no_std]

pub mod dxl;
pub mod facade;
pub mod policy;
pub mod reg;
pub mod spec;
pub mod vendor;

use core::cell::RefCell;
use critical_section::Mutex;
use heapless::FnvIndexMap;

// Re-export main types
pub use spec::{Access, Encoding, IndirectBankSpec, MapEntry, RangeSpec, RegSpec, UnitEncoding};

// Re-export typed register access
pub use reg::{Facade, Reg, ViewRead, ViewWrite, RO, RW, WO};

// Re-export shadow policies for embedded-shadow integration
pub use policy::{ServoAccessPolicy, ServoPersistPolicy};

// Re-export vendor module items for kernel use
pub use vendor::{ctrl, telem, VENDOR_FIELDS};

// Re-export DXL module items
pub use dxl::{DXL_FIELDS, DXL_FIELDS_EEPROM, INDIRECT_BANKS};

// Re-export EEPROM bitmaps and hash map builder
pub use dxl::{
    build_eeprom_hash_map, EEPROM_BITMAP_DXL, EEPROM_BITMAP_VENDOR, EEPROM_CAPACITY, EEPROM_COUNT,
};

/// All EEPROM fields for persistence (const version).
///
/// Currently only DXL region has EEPROM fields. If vendor EEPROM fields
/// are added in the future, this will need to be updated.
pub const EEPROM_FIELDS: &[RegSpec] = dxl::DXL_FIELDS_EEPROM;

/// Maximum protocol address (1024-byte address space).
pub const MAX_ADDRESS: u16 = 1024;

/// DXL region start address.
pub const DXL_START: u16 = 0x000;

/// DXL region end address (exclusive).
pub const DXL_END: u16 = 0x0FC;

/// Vendor region start address.
pub const VENDOR_START: u16 = 0x200;

/// EEPROM bitmap width (64 bits = 64 addresses per bitmap).
const EEPROM_BITMAP_WIDTH: u16 = 64;

/// DXL EEPROM region: 0..64
const DXL_EEPROM_START: u16 = 0;
const DXL_EEPROM_END: u16 = DXL_EEPROM_START + EEPROM_BITMAP_WIDTH;

/// Vendor EEPROM region: 512..576
const VENDOR_EEPROM_START: u16 = VENDOR_START;
const VENDOR_EEPROM_END: u16 = VENDOR_EEPROM_START + EEPROM_BITMAP_WIDTH;

// ============================================================================
// Static Storage for Hash Map
// ============================================================================

/// Type alias for the EEPROM hash map.
pub type EepromHashMap = FnvIndexMap<u32, usize, { dxl::EEPROM_CAPACITY }>;

/// Static storage for the EEPROM hash map (initialized by `init()`).
static EEPROM_HASH_MAP: Mutex<RefCell<Option<EepromHashMap>>> = Mutex::new(RefCell::new(None));

/// Initialize the registry.
///
/// Must be called once at system startup before using `eeprom_idx_by_hash()`.
/// Safe to call multiple times (subsequent calls are no-ops).
pub fn init() {
    critical_section::with(|cs| {
        let mut map_ref = EEPROM_HASH_MAP.borrow_ref_mut(cs);
        if map_ref.is_none() {
            *map_ref = Some(dxl::build_eeprom_hash_map());
        }
    });
}

/// Lookup EEPROM field index by name_hash. O(1) after init.
///
/// Returns the index into `EEPROM_FIELDS` for the given name hash.
/// Returns `None` if the hash is not found or registry not initialized.
pub fn eeprom_idx_by_hash(hash: u32) -> Option<usize> {
    critical_section::with(|cs| {
        EEPROM_HASH_MAP
            .borrow_ref(cs)
            .as_ref()
            .and_then(|map| map.get(&hash).copied())
    })
}

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

/// Get all EEPROM fields for persistence.
///
/// Returns fields that should be persisted to flash.
/// These are fields with `eeprom=true` (locked when torque enabled).
pub fn eeprom_fields() -> impl Iterator<Item = &'static RegSpec> {
    all_fields().filter(|f| f.is_eeprom())
}

/// Check if an address range overlaps any EEPROM field. O(1).
///
/// Uses bitmap lookup for addresses in valid EEPROM ranges:
/// - DXL EEPROM: 0-63
/// - Vendor EEPROM: 512-575
///
/// Used by ShadowStorage to determine if a write should trigger persistence.
#[inline]
pub fn touches_eeprom(offset: u16, len: u16) -> bool {
    let end = offset.saturating_add(len);

    // Check DXL EEPROM region
    if offset < DXL_EEPROM_END {
        let mask = range_mask(offset as u64, end.min(DXL_EEPROM_END) as u64);
        if (EEPROM_BITMAP_DXL & mask) != 0 {
            return true;
        }
    }

    // Check vendor EEPROM region
    if end > VENDOR_EEPROM_START && offset < VENDOR_EEPROM_END {
        let start = offset.saturating_sub(VENDOR_EEPROM_START) as u64;
        let e = end
            .saturating_sub(VENDOR_EEPROM_START)
            .min(EEPROM_BITMAP_WIDTH as u16) as u64;
        let mask = range_mask(start, e);
        if (EEPROM_BITMAP_VENDOR & mask) != 0 {
            return true;
        }
    }

    false
}

/// Create a bitmask for range [start, end) within a 64-bit bitmap.
#[inline]
const fn range_mask(start: u64, end: u64) -> u64 {
    if end >= EEPROM_BITMAP_WIDTH as u64 {
        !0u64 << start
    } else if end <= start {
        0
    } else {
        ((1u64 << end) - 1) & !((1u64 << start) - 1)
    }
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

    #[test]
    fn test_eeprom_fields() {
        let count = eeprom_fields().count();
        // Should have some EEPROM fields
        assert!(count > 0);

        // All returned fields should have eeprom=true
        for field in eeprom_fields() {
            assert!(field.is_eeprom());
        }

        // id field at addr 7 should be EEPROM
        let id_field = eeprom_fields().find(|f| f.name == "id");
        assert!(id_field.is_some());
    }

    #[test]
    fn test_eeprom_fields_const() {
        // EEPROM_FIELDS should match the iterator version
        assert_eq!(EEPROM_FIELDS.len(), eeprom_fields().count());

        // All const fields should have eeprom=true
        for field in EEPROM_FIELDS {
            assert!(field.is_eeprom());
        }

        // Check specific known EEPROM fields
        let has_id = EEPROM_FIELDS.iter().any(|f| f.name == "id");
        let has_baud = EEPROM_FIELDS.iter().any(|f| f.name == "baud_rate");
        assert!(has_id, "id should be in EEPROM_FIELDS");
        assert!(has_baud, "baud_rate should be in EEPROM_FIELDS");

        // Each field should have a non-zero name_hash
        for field in EEPROM_FIELDS {
            assert_ne!(
                field.name_hash, 0,
                "name_hash should be computed for {}",
                field.name
            );
        }
    }

    #[test]
    fn test_touches_eeprom() {
        // Write to id field (addr 7, len 1) should touch EEPROM
        assert!(touches_eeprom(7, 1));

        // Write to baud_rate field (addr 8, len 1) should touch EEPROM
        assert!(touches_eeprom(8, 1));

        // Write spanning multiple EEPROM fields
        assert!(touches_eeprom(7, 4));

        // Write to RAM region (torque_enable at 64) should NOT touch EEPROM
        assert!(!touches_eeprom(64, 1));

        // Write to vendor region should NOT touch EEPROM (no vendor EEPROM fields)
        assert!(!touches_eeprom(512, 4));
    }

    #[test]
    fn test_name_hash_exists() {
        // Verify name_hash is populated
        let spec = find("id").unwrap();
        assert_ne!(spec.name_hash, 0);

        // Different names should have different hashes
        let id_hash = find("id").unwrap().name_hash;
        let baud_hash = find("baud_rate").unwrap().name_hash;
        assert_ne!(id_hash, baud_hash);
    }

    #[test]
    fn test_eeprom_bitmap_dxl() {
        // Verify bitmap has bits set for known EEPROM addresses
        // id is at addr 7
        assert_ne!(
            EEPROM_BITMAP_DXL & (1 << 7),
            0,
            "bit 7 should be set for id"
        );
        // baud_rate is at addr 8
        assert_ne!(
            EEPROM_BITMAP_DXL & (1 << 8),
            0,
            "bit 8 should be set for baud_rate"
        );

        // torque_enable is at addr 64 - not in bitmap (not EEPROM)
        // Address 64 is outside DXL bitmap range anyway
        assert_eq!(EEPROM_BITMAP_DXL & (1 << 63), 0, "bit 63 should not be set");
    }

    #[test]
    fn test_eeprom_bitmap_vendor() {
        // Currently no vendor EEPROM fields
        assert_eq!(EEPROM_BITMAP_VENDOR, 0, "no vendor EEPROM fields expected");
    }

    #[test]
    fn test_range_mask() {
        // Single bit
        assert_eq!(range_mask(7, 8), 1 << 7);

        // Range of bits
        assert_eq!(range_mask(0, 4), 0b1111);
        assert_eq!(range_mask(4, 8), 0b11110000);

        // Empty range
        assert_eq!(range_mask(5, 5), 0);
        assert_eq!(range_mask(10, 5), 0);

        // Full range
        assert_eq!(range_mask(0, 64), !0u64);
    }

    #[test]
    fn test_init_and_hash_lookup() {
        // Initialize registry
        init();

        // Lookup by hash should work
        let id_hash = find("id").unwrap().name_hash;
        let idx = eeprom_idx_by_hash(id_hash);
        assert!(idx.is_some(), "should find id by hash");

        // The index should point to the correct field
        let idx = idx.unwrap();
        assert!(idx < EEPROM_FIELDS.len());
        assert_eq!(EEPROM_FIELDS[idx].name, "id");

        // Unknown hash should return None
        assert!(eeprom_idx_by_hash(0xDEADBEEF).is_none());
    }

    #[test]
    fn test_eeprom_capacity() {
        // Capacity should be >= count and power of 2
        assert!(EEPROM_CAPACITY >= EEPROM_COUNT);
        assert!(EEPROM_CAPACITY.is_power_of_two());
    }
}
