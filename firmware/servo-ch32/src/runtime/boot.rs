//! Tinyboot boot confirmation. A freshly flashed image boots in Validating
//! and the bootloader spends one trial per power-on; finishing bringup
//! marks it Idle so the servo never falls back into the bootloader.

use tinyboot_ch32::app::traits::{BootMetaStore as _, BootState};
use tinyboot_ch32::platform::BootMetaStore;

/// Tinyboot's `App::confirm` without the protocol frame it never uses:
/// Validating -> Idle on a new image's first boot, else one META read.
pub fn confirm() {
    let mut meta = BootMetaStore::default();
    if meta.boot_state() == BootState::Validating {
        let _ = meta.refresh(meta.app_checksum(), BootState::Idle, meta.app_size());
    }
}
