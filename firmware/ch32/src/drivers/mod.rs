pub mod output_pin;
pub mod stat_led;

use crate::board::BoardWiring;
use crate::hal::gpio::Level;

/// Install every static driver instance in this crate from the board
/// wiring. Single entry point so bringup doesn't need to know each driver's
/// per-instance install API.
///
/// SAFETY: must be called once during bringup, before any IRQ is unmasked.
pub unsafe fn install_drivers(w: &BoardWiring) {
    // SAFETY: per fn contract; caller holds pre-IRQ sole-writer discipline.
    unsafe { stat_led::StatLed::install(w.stat_led) };
    unsafe { output_pin::OutputPin::install_dbg(w.dbg, Level::Low) };
}
