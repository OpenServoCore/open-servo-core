//! GPIO adapters — bind driver interfaces to a specific `Pin` and forward
//! to `hal::gpio` register access.

use crate::drivers::traits::DigitalOut;
use crate::hal::Pin;
use crate::hal::gpio::{self, PinMode};
use crate::types::Level;

/// Push-pull output bound to one pin. Configures the pin and drives its
/// initial level at construction; `set` thereafter is one BSHR/BCR write.
pub struct Output {
    pin: Pin,
}

impl Output {
    pub fn new(pin: Pin, initial: Level) -> Self {
        gpio::configure(pin, PinMode::OUTPUT_PUSH_PULL);
        gpio::set_level(pin, initial);
        Self { pin }
    }
}

impl DigitalOut for Output {
    #[inline(always)]
    fn set(&mut self, level: Level) {
        gpio::set_level(self.pin, level);
    }
}
