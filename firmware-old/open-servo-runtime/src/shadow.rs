//! Shadow storage for servo runtime using embedded-shadow.
//!
//! This module provides:
//! - `ServoPersistTrigger`: Persist trigger that signals the persist service
//! - `ServoShadowStorage`: Type alias for the concrete shadow storage
//! - `create_storage`: Factory function for creating shadow storage

use embedded_shadow::{
    persist::PersistTrigger,
    policy::{AllowAllPolicy, NoPersistPolicy},
    storage::ShadowStorage,
};
use heapless::Vec;
use open_servo_hw::v2::PersistW;
use open_servo_registry::{ServoAccessPolicy, ServoPersistPolicy};

/// No-op persist trigger for testing.
pub struct NoPersistTrigger;

impl<PK> PersistTrigger<PK> for NoPersistTrigger {
    fn push_key(&mut self, _key: PK) {}
    fn request_persist(&mut self) {}
}

/// Shadow table size (1024 bytes, Dynamixel Protocol 2.0 compatible).
pub const SHADOW_TABLE_SIZE: usize = 1024;

/// Block size for dirty tracking (64 bytes per block).
pub const SHADOW_BLOCK_SIZE: usize = 64;

/// Block count (1024 / 64 = 16 blocks).
pub const SHADOW_BLOCK_COUNT: usize = SHADOW_TABLE_SIZE / SHADOW_BLOCK_SIZE;

/// Persist trigger that signals the persist service when EEPROM is written.
pub struct ServoPersistTrigger<PW: PersistW> {
    signal: PW,
    pending: Vec<u32, 8>,
}

impl<PW: PersistW> ServoPersistTrigger<PW> {
    /// Create a new persist trigger with the given signal.
    pub fn new(signal: PW) -> Self {
        Self {
            signal,
            pending: Vec::new(),
        }
    }

    /// Get the pending persist keys.
    pub fn pending_keys(&self) -> &[u32] {
        &self.pending
    }

    /// Check if there are pending keys.
    pub fn has_pending(&self) -> bool {
        !self.pending.is_empty()
    }
}

impl<PW: PersistW> embedded_shadow::persist::PersistTrigger<u32> for ServoPersistTrigger<PW> {
    fn push_key(&mut self, key: u32) {
        // Deduplicate keys
        if !self.pending.contains(&key) {
            let _ = self.pending.push(key);
        }
    }

    fn request_persist(&mut self) {
        if !self.pending.is_empty() {
            self.signal.signal(());
            self.pending.clear();
        }
    }
}

/// Type alias for the servo shadow storage with persist support.
pub type ServoShadowStorage<PW> = ShadowStorage<
    SHADOW_TABLE_SIZE,
    SHADOW_BLOCK_SIZE,
    SHADOW_BLOCK_COUNT,
    ServoAccessPolicy,
    ServoPersistPolicy,
    ServoPersistTrigger<PW>,
    u32,
>;

/// Type alias for shadow storage without persist support (for testing).
pub type TestShadowStorage = ShadowStorage<
    SHADOW_TABLE_SIZE,
    SHADOW_BLOCK_SIZE,
    SHADOW_BLOCK_COUNT,
    AllowAllPolicy,
    NoPersistPolicy,
    NoPersistTrigger,
    (),
>;

/// Create shadow storage with persist support.
pub fn create_storage<PW: PersistW>(signal: PW) -> ServoShadowStorage<PW> {
    ShadowStorage::new(
        ServoAccessPolicy,
        ServoPersistPolicy,
        ServoPersistTrigger::new(signal),
    )
}

/// Create shadow storage for testing (no persist).
pub fn create_test_storage() -> TestShadowStorage {
    ShadowStorage::new(AllowAllPolicy {}, NoPersistPolicy {}, NoPersistTrigger)
}

// Re-export signal traits for convenience.
pub use open_servo_hw::v2::{PersistR, ResetLevel, ResetR, ResetW, SignalReader};

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;
    use open_servo_hw::v2::SignalWriter;

    struct MockPersistSignal(Cell<bool>);

    impl MockPersistSignal {
        const fn new() -> Self {
            Self(Cell::new(false))
        }

        fn was_signaled(&self) -> bool {
            self.0.get()
        }
    }

    impl SignalWriter<()> for MockPersistSignal {
        fn signal(&self, _: ()) {
            self.0.set(true);
        }
    }

    #[test]
    fn test_create_storage() {
        let storage = create_storage(MockPersistSignal::new());

        // Should be able to read/write via views
        storage.host_shadow().with_view(|view| {
            view.write_range(0x50, &[1, 2, 3, 4]).unwrap();
        });

        storage.host_shadow().with_view(|view| {
            let mut buf = [0u8; 4];
            view.read_range(0x50, &mut buf).unwrap();
            assert_eq!(buf, [1, 2, 3, 4]);
        });
    }

    #[test]
    fn test_create_test_storage() {
        let storage = create_test_storage();

        storage.host_shadow().with_view(|view| {
            view.write_range(0x50, &[1, 2, 3, 4]).unwrap();
        });

        // SAFETY: Test context, no ISR contention.
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view| {
                assert!(view.any_dirty());
                let mut buf = [0u8; 4];
                view.read_range(0x50, &mut buf).unwrap();
                assert_eq!(buf, [1, 2, 3, 4]);
            });
        }
    }

    #[test]
    fn test_persist_trigger_deduplicates() {
        let mut trigger = ServoPersistTrigger::new(MockPersistSignal::new());

        trigger.push_key(123);
        trigger.push_key(456);
        trigger.push_key(123); // Duplicate

        assert_eq!(trigger.pending_keys(), &[123, 456]);
    }

    #[test]
    fn test_persist_trigger_signals_and_clears() {
        let signal = MockPersistSignal::new();
        let mut trigger = ServoPersistTrigger::new(signal);

        trigger.push_key(123);
        assert!(trigger.has_pending());

        // Can't easily access signal after move, but we can check pending is cleared
        trigger.request_persist();
        assert!(!trigger.has_pending());
    }
}
