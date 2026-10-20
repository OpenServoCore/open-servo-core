//! Tinyboot glue. A freshly flashed image boots in Validating and the
//! bootloader spends one trial per power-on; finishing bringup marks it
//! Idle so the servo never falls back into the bootloader.

use osc_servo_core::{BaudRate, Reboot};
use tinyboot_ch32::app::traits::RunMode;
use tinyboot_ch32::app::{BaudRate as BootBaud, BootCtl, new_app};

/// Validating -> Idle on a new image's first boot, else one META read.
pub fn confirm() {
    new_app(BootCtl::new()).confirm();
}

/// Honor a staged REBOOT (protocol sec 9.5): back into the app, or into
/// the bootloader at the rate the host named (protocol sec 9.6), stored in
/// META only when it changed.
pub fn reboot(reboot: Reboot) -> ! {
    let (mode, baud) = match reboot {
        Reboot::App => (RunMode::HandOff, None),
        Reboot::Bootloader(baud) => (RunMode::Service, Some(boot_baud(baud))),
    };
    new_app(BootCtl::new()).reset(mode, baud)
}

const fn boot_baud(baud: BaudRate) -> BootBaud {
    match baud {
        BaudRate::B500000 => BootBaud::B500000,
        BaudRate::B1000000 => BootBaud::B1000000,
        BaudRate::B2000000 => BootBaud::B2000000,
        BaudRate::B3000000 => BootBaud::B3000000,
    }
}
