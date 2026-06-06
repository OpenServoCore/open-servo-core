//! Activity LED driver — solid ON when idle, ~10 Hz blink during slave TX.
//!
//! Schema: main loop polls every `SAMPLE_PERIOD_US`. Each tick reads
//! `DXL_TX_COUNT`; if it changed since the previous sample, toggle the LED;
//! otherwise drive it ON (default state). Sustained traffic → visible blink
//! at the sample rate; idle → solid ON. Cheap: one atomic load and at most
//! one GPIO write per sample.

use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;

use crate::dxl::statics::DXL_TX_COUNT;
use crate::hal::gpio::{self, Level};
use crate::hal::systick::{self, TICKS_PER_US};
use crate::hal::Pin;

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

/// Pin handle installed at bring-up. `None` until [`install`] runs; [`poll`]
/// no-ops in that window.
static STAT_LED_PIN: SyncUnsafeCell<Option<Pin>> = SyncUnsafeCell::new(None);

/// Main-loop poll state. Single-threaded (only `poll` writes it), so plain
/// fields are fine; wrapped in `SyncUnsafeCell` to satisfy the static.
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

/// Seed the pin handle. Bringup also drives the pin to [`ON_LEVEL`] so the
/// LED is solid ON before the main loop takes over.
pub(crate) fn install(pin: Pin) {
    // SAFETY: called once during bring-up, pre-IRQ, sole writer.
    unsafe {
        *STAT_LED_PIN.get() = Some(pin);
    }
}

/// Called once per main-loop iteration. Cheap early-return between samples.
pub(crate) fn poll() {
    let pin = match unsafe { *STAT_LED_PIN.get() } {
        Some(p) => p,
        None => return,
    };
    let now = systick::ticks();
    // SAFETY: main-loop-only writer; no ISR touches `POLL_STATE`.
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
        gpio::set_level(pin, if target_on { ON_LEVEL } else { OFF_LEVEL });
        state.led_on = target_on;
    }
}
