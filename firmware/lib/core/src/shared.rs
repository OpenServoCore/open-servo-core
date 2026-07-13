use core::cell::SyncUnsafeCell;

use osc_protocol::wire::UID_LEN;

use crate::ControlTableCell;
use crate::persist::ConfigStore;

#[repr(C)]
pub struct Shared {
    pub table: ControlTableCell,
    /// The factory UID, silicon ID zero-padded to the 16-byte wire field
    /// (osc-native sec 9.2) -- internal identity, not a table register; MGMT ENUM
    /// is its only wire reader.
    uid: SyncUnsafeCell<[u8; UID_LEN]>,
    /// The sec 9.4 persistence store; MGMT SAVE/FACTORY are its only callers
    /// (cold path -- `dyn` costs nothing that matters here).
    store: SyncUnsafeCell<Option<&'static dyn ConfigStore>>,
}

#[allow(clippy::new_without_default)]
impl Shared {
    pub const fn new() -> Self {
        Self {
            table: ControlTableCell::new(),
            uid: SyncUnsafeCell::new([0; UID_LEN]),
            store: SyncUnsafeCell::new(None),
        }
    }

    /// Seed the ESIG-derived UID. Bringup-only, pre-IRQ; sole writer (the
    /// `seed_config_defaults` contract).
    pub fn seed_uid(&self, uid: [u8; UID_LEN]) {
        // SAFETY: see fn doc -- no reader exists before IRQs enable.
        unsafe { *self.uid.get() = uid };
    }

    /// Stable-address borrow: ENUM replies stream straight from it (sec 4.2).
    pub fn uid(&self) -> &[u8; UID_LEN] {
        // SAFETY: written only by `seed_uid` pre-IRQ; read-only afterward.
        unsafe { &*self.uid.get() }
    }

    /// Seed the persistence store. Bringup-only, pre-IRQ; sole writer (the
    /// `seed_config_defaults` contract).
    pub fn seed_store(&self, store: &'static dyn ConfigStore) {
        // SAFETY: see fn doc -- no reader exists before IRQs enable.
        unsafe { *self.store.get() = Some(store) };
    }

    /// The sec 9.4 store; `None` until seeded (dispatch answers `hardware`).
    pub fn store(&self) -> Option<&'static dyn ConfigStore> {
        // SAFETY: written only by `seed_store` pre-IRQ; read-only afterward.
        unsafe { *self.store.get() }
    }
}
