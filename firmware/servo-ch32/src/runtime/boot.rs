//! Tinyboot glue. A freshly flashed image boots in Validating and the
//! bootloader spends one trial per power-on; finishing bringup marks it
//! Idle so the servo never falls back into the bootloader. The bootloader
//! runs at the baud stored in META, kept at the bus's (protocol sec 9.6).

use osc_servo_core::BaudRate;
use tinyboot_ch32::app::{BaudRate as BootBaud, BootCtl, new_app};

/// Validating -> Idle on a new image's first boot, then `baud` into META.
/// Each is a flash write only when it changes, else one META read.
pub fn confirm(baud: BaudRate) {
    let mut app = new_app(BootCtl::new());
    app.confirm();
    app.set_baud(boot_baud(baud));
}

/// The bootloader comes up at `baud` from the next reset. Call before
/// `flash::set_boot_mode`: tinyboot's EOP clear writes STATR whole,
/// BOOT_MODE included.
pub fn store_baud(baud: BaudRate) {
    new_app(BootCtl::new()).set_baud(boot_baud(baud));
}

const fn boot_baud(baud: BaudRate) -> BootBaud {
    match baud {
        BaudRate::B500000 => BootBaud::B500000,
        BaudRate::B1000000 => BootBaud::B1000000,
        BaudRate::B2000000 => BootBaud::B2000000,
        BaudRate::B3000000 => BootBaud::B3000000,
    }
}
