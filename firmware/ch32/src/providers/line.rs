//! Line-sense provider (osc-native §9.1) — raw level of the bus wire for
//! rescue-break confirmation. An ordinary break has risen by FE-ISR entry
//! [F5]; a still-low line ~100 µs later is a rescue pulse. On the buffered
//! wire the level arrives through the receive buffer, which is muted only
//! during own TX — and rescue sensing runs on the RX path (FE entry +
//! SysTick rechecks), never mid-TX.

use ch32_metapac::USART1;
use osc_drivers::traits::bus;

use crate::cfg::chip;
use crate::hal::{gpio, usart};

/// Production binding to the bus line pin's input level.
pub struct LineSense;

impl bus::LineSense for LineSense {
    #[inline(always)]
    fn is_low(&self) -> bool {
        gpio::is_low(chip::BUS_LINE_PIN)
    }

    /// §6 A4 storm throttle: EIE gates the error-IRQ level pend. Flags keep
    /// latching and DMAR keeps ringing bytes while it is off.
    #[inline(always)]
    fn set_fault_wake(&mut self, on: bool) {
        usart::set_err_irq(USART1, on);
    }
}
