//! Tinyboot boot confirmation. A freshly flashed image boots in Validating
//! and the bootloader spends one trial per power-on; finishing bringup
//! marks it Idle so the servo never falls back into the bootloader.

use tinyboot_ch32::app::{BootCtl, new_app};

/// Validating -> Idle on a new image's first boot, else one META read.
pub fn confirm() {
    new_app(BootCtl::new()).confirm();
}
