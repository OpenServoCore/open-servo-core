//! Activity LED policy — solid ON when idle, ~10 Hz blink during slave TX.
//!
//! Schema: main loop polls every `SAMPLE_PERIOD_US`. Each tick reads
//! `DXL_TX_COUNT`; if it changed since the previous sample, toggle the LED;
//! otherwise drive it ON (default state). Pin writes go through
//! [`OutputPin::stat_led`]; this module is the *policy*, the driver is the
//! *mechanism*.

use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;

use crate::drivers::output_pin::OutputPin;
use crate::dxl::statics::DXL_TX_COUNT;
use crate::hal::gpio::Level;
use crate::hal::systick::{self, TICKS_PER_US};

/// 50 ms between samples — ~10 Hz blink during sustained TX, comfortable for
/// the eye and well above typical DXL polling rates so single-shot replies
/// still produce a visible flicker.
const SAMPLE_PERIOD_US: u32 = 50_000;

/// Active level used to drive the LED ON. Schematic has the LED cathode to
/// the pin (sink to MCU), so logic-high lights it; both polarities work
/// since the field is just the "ON" level — keep as `High` until a board
/// proves otherwise.
const ON_LEVEL: Level = Level::High;

const OFF_LEVEL: Level = match ON_LEVEL {
    Level::High => Level::Low,
    Level::Low => Level::High,
};

struct PollState {
    last_sample_tick: u32,
    last_tx_count: u32,
    led_on: bool,
}

static POLL_STATE: SyncUnsafeCell<PollState> = SyncUnsafeCell::new(PollState {
    last_sample_tick: 0,
    last_tx_count: 0,
    led_on: true,
});

/// Called once per main-loop iteration. Cheap early-return between samples.
pub(crate) fn poll() {
    let now = systick::ticks();
    // SAFETY: main-loop sole writer; no ISR aliases POLL_STATE.
    let state = unsafe { &mut *POLL_STATE.get() };
    if now.wrapping_sub(state.last_sample_tick) < SAMPLE_PERIOD_US * TICKS_PER_US {
        return;
    }
    state.last_sample_tick = now;

    let tx_count = DXL_TX_COUNT.load(Ordering::Relaxed);
    let active = tx_count != state.last_tx_count;
    state.last_tx_count = tx_count;

    let target_on = if active { !state.led_on } else { true };
    if target_on != state.led_on {
        let level = if target_on { ON_LEVEL } else { OFF_LEVEL };
        // SAFETY: STAT_LED is main-loop-only; see `OutputPin::stat_led`.
        unsafe { OutputPin::stat_led() }.set(level);
        state.led_on = target_on;
    }
}
