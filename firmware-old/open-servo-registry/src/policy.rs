//! Shadow table policies for embedded-shadow integration.
//!
//! Provides access control and persistence policies using the register
//! map's EEPROM bitmaps for O(1) checks.

use embedded_shadow::policy::{AccessPolicy, PersistPolicy};

use crate::{touches_eeprom, EEPROM_FIELDS};

/// Access policy for servo shadow table.
///
/// Currently allows all reads and writes. Access control for EEPROM fields
/// (torque-enable check) is enforced at a higher level in the protocol handler.
pub struct ServoAccessPolicy;

impl AccessPolicy for ServoAccessPolicy {
    #[inline]
    fn can_read(&self, _addr: u16, _len: usize) -> bool {
        // All addresses are readable
        true
    }

    #[inline]
    fn can_write(&self, _addr: u16, _len: usize) -> bool {
        // All addresses are writable at this layer.
        // EEPROM lock (torque-enable check) is enforced by protocol handler.
        true
    }
}

/// Persistence policy for servo shadow table.
///
/// Uses EEPROM bitmaps for O(1) detection and pushes field name hashes
/// as persist keys for each overlapping EEPROM field.
pub struct ServoPersistPolicy;

impl PersistPolicy<u32> for ServoPersistPolicy {
    fn push_persist_keys_for_range<F>(&self, addr: u16, len: usize, mut push_key: F) -> bool
    where
        F: FnMut(u32),
    {
        // O(1) bitmap check first
        if !touches_eeprom(addr, len as u16) {
            return false;
        }

        // Push name_hash for each overlapping EEPROM field
        let end = addr.saturating_add(len as u16);
        let mut any = false;

        for field in EEPROM_FIELDS {
            let field_end = field.address.saturating_add(field.len() as u16);
            // Check for overlap: [addr, end) ∩ [field.address, field_end) ≠ ∅
            if addr < field_end && field.address < end {
                push_key(field.name_hash);
                any = true;
            }
        }

        any
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn access_policy_allows_all() {
        let policy = ServoAccessPolicy;

        // All reads allowed
        assert!(policy.can_read(0, 1));
        assert!(policy.can_read(512, 4));
        assert!(policy.can_read(1000, 24));

        // All writes allowed
        assert!(policy.can_write(0, 1));
        assert!(policy.can_write(512, 4));
        assert!(policy.can_write(64, 1));
    }

    #[test]
    fn persist_policy_detects_eeprom() {
        let policy = ServoPersistPolicy;

        // Write to id field (addr 7) should trigger persist
        let mut keys = heapless::Vec::<u32, 8>::new();
        let result =
            policy.push_persist_keys_for_range(7, 1, |k| {
                let _ = keys.push(k);
            });
        assert!(result, "id field should be EEPROM");
        assert!(!keys.is_empty(), "should push at least one key");

        // Write to RAM (torque_enable at 64) should NOT trigger persist
        keys.clear();
        let result =
            policy.push_persist_keys_for_range(64, 1, |k| {
                let _ = keys.push(k);
            });
        assert!(!result, "torque_enable should not be EEPROM");
        assert!(keys.is_empty(), "should not push any keys");

        // Write to vendor region (no EEPROM fields) should NOT trigger persist
        keys.clear();
        let result = policy.push_persist_keys_for_range(512, 4, |k| {
            let _ = keys.push(k);
        });
        assert!(!result, "vendor region has no EEPROM fields");
        assert!(keys.is_empty());
    }

    #[test]
    fn persist_policy_pushes_correct_hash() {
        use crate::find;

        let policy = ServoPersistPolicy;
        let id_spec = find("id").unwrap();

        // Write exactly to id field
        let mut keys = heapless::Vec::<u32, 8>::new();
        policy.push_persist_keys_for_range(id_spec.address, id_spec.len() as usize, |k| {
            let _ = keys.push(k);
        });

        assert!(keys.contains(&id_spec.name_hash), "should push id's name_hash");
    }

    #[test]
    fn persist_policy_spanning_multiple_fields() {
        let policy = ServoPersistPolicy;

        // Write spanning multiple EEPROM fields (addr 7-10 covers id and baud_rate)
        let mut keys = heapless::Vec::<u32, 8>::new();
        policy.push_persist_keys_for_range(7, 4, |k| {
            let _ = keys.push(k);
        });

        // Should have multiple keys
        assert!(keys.len() >= 2, "should push keys for multiple fields");
    }
}
