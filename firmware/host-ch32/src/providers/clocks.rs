//! System provider: clock tree + peripheral gates. HSE-fail policy is the
//! caller's -- this just reports it and leaves the chip on the loader's
//! clock, where the LED still blinks.

use crate::hal::rcc;

pub struct Clocks;

impl Clocks {
    /// Returns false when the crystal never came ready; the USBHS PLL is
    /// only configured on success (it has no meaning without the crystal).
    pub fn init() -> bool {
        rcc::enable_peripherals();
        if !rcc::init_sysclk() {
            return false;
        }
        rcc::init_usbhs_pll();
        true
    }
}
