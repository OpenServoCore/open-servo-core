//! Digital-output provider -- binds `DigitalOut` to a single GPIO pin.

use osc_servo_drivers::Level;
use osc_servo_drivers::traits;

use crate::hal::Pin;
use crate::hal::gpio::{self, PinMode};

/// Push-pull output bound to one pin. Configures the pin and drives its
/// initial level at construction; `set` thereafter is one BSHR/BCR write.
pub struct DigitalOut {
    pin: Pin,
}

impl DigitalOut {
    pub fn new(pin: Pin, initial: Level) -> Self {
        gpio::configure(pin, PinMode::OUTPUT_PUSH_PULL);
        gpio::set_level(pin, initial);
        Self { pin }
    }
}

impl traits::DigitalOut for DigitalOut {
    #[inline(always)]
    fn set(&mut self, level: Level) {
        gpio::set_level(self.pin, level);
    }
}
