//! Decoupled 5 Hz telemetry pump. The DMA TC ISR (20 kHz) publishes the latest
//! `Sample` into a static slot; the main loop prints once per 4000 ticks.
//! No CS, no separate timer — occasional torn reads on a debug stream are
//! tolerated in exchange for keeping TIM2/TIM3 free.

use core::cell::SyncUnsafeCell;

use osc_core::Sample;

use crate::statics::read_sample_tick;

static TELEMETRY_SAMPLE: SyncUnsafeCell<Option<Sample>> = SyncUnsafeCell::new(None);
static LAST_PRINT_TICK: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

const PRINT_INTERVAL_TICKS: u32 = 4_000;

pub fn record_sample(sample: &Sample) {
    unsafe {
        *TELEMETRY_SAMPLE.get() = Some(*sample);
    }
}

pub fn pump() {
    let now = read_sample_tick();
    let last = unsafe { *LAST_PRINT_TICK.get() };
    if now.wrapping_sub(last) < PRINT_INTERVAL_TICKS {
        return;
    }
    unsafe { *LAST_PRINT_TICK.get() = now };

    let Some(s) = (unsafe { *TELEMETRY_SAMPLE.get() }) else {
        return;
    };

    crate::log::info!(
        "t={=u32} pos={=i32}urad temp={=i16}cC vbus={=i16}mV vmot={=i16}mV cur_pk={=i16}mA cur_tr={=i16}mA",
        s.tick,
        s.pos.0,
        s.temp.0,
        s.vbus.0,
        s.vmotor.0,
        s.current.0,
        s.current_post_trough.0,
    );
}
