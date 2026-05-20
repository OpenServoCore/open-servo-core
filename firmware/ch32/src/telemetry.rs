//! Decoupled 5 Hz telemetry pump. The DMA TC ISR (20 kHz) publishes the latest
//! `SampleFrame` into a static slot; the main loop prints once per 4000 ticks.
//! No CS, no separate timer — occasional torn reads on a debug stream are
//! tolerated in exchange for keeping TIM2/TIM3 free.

use core::cell::SyncUnsafeCell;

use osc_core::SampleFrame;

use crate::statics::read_sample_tick;

static TELEMETRY_FRAME: SyncUnsafeCell<Option<SampleFrame>> = SyncUnsafeCell::new(None);
static LAST_PRINT_TICK: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

const PRINT_INTERVAL_TICKS: u32 = 4_000;

pub fn record_frame(frame: &SampleFrame) {
    unsafe {
        *TELEMETRY_FRAME.get() = Some(*frame);
    }
}

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
