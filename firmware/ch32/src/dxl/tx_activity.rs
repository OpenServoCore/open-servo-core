//! TX-activity indicator policy — drives the stat LED based on observed
//! servo-TX activity. Sampled at a slow cadence so the resulting blink is
//! visible to a human.
//!
//! Policy, not a driver: composes the chip-side TX counter with the LED's
//! pattern API. Will fold into the future `DxlTx` driver once that lands.

use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;

use crate::drivers::Drivers;
use crate::drivers::led::Pattern;
use crate::dxl::statics::DXL_TX_COUNT;
use crate::hal::systick::{self, TICKS_PER_US};

/// 50 ms between samples — ~10 Hz blink while active, comfortable for the
/// eye and well above typical DXL polling rates so single-shot replies still
/// produce a visible flicker.
const SAMPLE_PERIOD_US: u32 = 50_000;

/// Active blink period (full cycle). 100 ms = 10 Hz.
const ACTIVE_BLINK_PERIOD_US: u32 = 100_000;

struct State {
    last_sample_tick: u32,
    last_tx_count: u32,
}

static STATE: SyncUnsafeCell<State> = SyncUnsafeCell::new(State {
    last_sample_tick: 0,
    last_tx_count: 0,
});

/// Called once per main-loop iteration. Cheap early-return between samples;
/// updates the StatLed pattern when activity state changes.
pub(crate) fn poll() {
    let now = systick::ticks();
    // SAFETY: main-loop sole writer.
    let state = unsafe { &mut *STATE.get() };
    if now.wrapping_sub(state.last_sample_tick) < SAMPLE_PERIOD_US * TICKS_PER_US {
        return;
    }
    state.last_sample_tick = now;

    let tx_count = DXL_TX_COUNT.load(Ordering::Relaxed);
    let active = tx_count != state.last_tx_count;
    state.last_tx_count = tx_count;

    let pattern = if active {
        Pattern::Blink {
            period_us: ACTIVE_BLINK_PERIOD_US,
        }
    } else {
        Pattern::SolidOn
    };
    // SAFETY: stat_led is main-loop accessor; see `Drivers::stat_led`.
    unsafe { Drivers::stat_led() }.set_pattern(pattern);
}
