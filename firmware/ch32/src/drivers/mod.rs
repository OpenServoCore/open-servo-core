pub mod output_pin;

use crate::board::BoardWiring;
use crate::hal::gpio::Level;

/// Install every static driver instance in this crate from the board
/// wiring. Single entry point so bringup doesn't need to know each driver's
/// per-instance install API.
///
/// SAFETY: must be called once during bringup, before any IRQ is unmasked.
pub unsafe fn install_drivers(w: &BoardWiring) {
    // STAT LED defaults solid ON to signal "chip alive"; main loop blinks it
    // on slave-TX activity via `crate::stat_led::poll`.
    // SAFETY: per fn contract; caller holds pre-IRQ sole-writer discipline.
    unsafe { output_pin::OutputPin::install_stat_led(w.stat_led, Level::High) };
    unsafe { output_pin::OutputPin::install_dbg(w.dbg, Level::Low) };
}
