//! Decoupled 5 Hz telemetry pump.
//!
//! The DMA TC ISR (20 kHz) calls [`record_frame`] to publish the latest
//! `SampleFrame` into a static slot. The main loop calls [`pump`] in a tight
//! spin — it reads the latest tick, prints once per 4000 samples (= 5 Hz at
//! 20 kHz), and otherwise returns. No CS, no separate timer: occasional torn
//! reads on a debug stream are acceptable in exchange for keeping TIM2/TIM3
//! free for encoders and DXL turnaround respectively.

use core::cell::SyncUnsafeCell;

use osc_core::SampleFrame;

use crate::statics::read_sample_tick;

/// Latest frame published by the kernel ISR. `None` until the first tick.
static TELEMETRY_FRAME: SyncUnsafeCell<Option<SampleFrame>> = SyncUnsafeCell::new(None);

/// Last `sample_tick` value at which `pump` printed. Used to detect the next
/// 4000-tick boundary. Only the main loop writes; ISR never touches it.
static LAST_PRINT_TICK: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// 20 kHz tick rate / 5 Hz print rate. Wrapping subtraction handles u32 rollover.
const PRINT_INTERVAL_TICKS: u32 = 4_000;

/// Called from the DMA TC ISR. Bare write — torn reads vs the main loop are tolerated.
pub fn record_frame(frame: &SampleFrame) {
    unsafe {
        *TELEMETRY_FRAME.get() = Some(*frame);
    }
}

/// Call from the main loop. Cheap path is one load + one subtract + one compare;
/// the print fires once per PRINT_INTERVAL_TICKS.
pub fn pump() {
    let now = read_sample_tick();
    let last = unsafe { *LAST_PRINT_TICK.get() };
    if now.wrapping_sub(last) < PRINT_INTERVAL_TICKS {
        return;
    }
    unsafe { *LAST_PRINT_TICK.get() = now };

    let Some(f) = (unsafe { *TELEMETRY_FRAME.get() }) else {
        return;
    };

    crate::log::info!(
        "t={=u32} pos={=i32}urad temp={=i16}cC vbus={=i16}mV vmot={=i16}mV cur_pk={=i16}mA cur_tr={=i16}mA",
        f.tick,
        f.pos.0,
        f.temp.0,
        f.vbus.0,
        f.vmotor.0,
        f.current.0,
        f.current_post_trough.0,
    );
}
