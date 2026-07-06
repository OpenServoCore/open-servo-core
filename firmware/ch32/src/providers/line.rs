//! Line-sense provider (osc-native §9.1) — raw level of the bus pin for
//! rescue-break confirmation. An ordinary break has risen by FE-ISR entry
//! [F5]; a still-low line ~100 µs later is a rescue pulse.

use osc_drivers::traits::bus;

use crate::cfg::chip;
use crate::hal::gpio;

/// Production binding to the PC0 input level.
pub struct LineSense;

impl bus::LineSense for LineSense {
    #[inline(always)]
    fn is_low(&self) -> bool {
        gpio::is_low(chip::BUS_USART_MAPPING.tx_pin())
    }
}
