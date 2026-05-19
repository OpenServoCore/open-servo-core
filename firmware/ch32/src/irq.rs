use ch32_metapac::DMA1;
use osc_core::FrameInputs;

use crate::statics::{KERNEL, SHARED};

#[qingke_rt::interrupt]
fn DMA1_CHANNEL1() {
    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        // Bump tick first so `build_sample_frame` reads the value that
        // matches *this* scan rather than the previous one.
        let tick = &raw mut (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        tick.write(tick.read().wrapping_add(1));

        if let Some(kernel) = (*KERNEL.get()).as_mut() {
            // Snapshot SHARED → FrameInputs once per tick so the board
            // never holds a cross-domain reference into the control table.
            let inputs = FrameInputs::snapshot(&SHARED);
            let frame = kernel.board.build_sample_frame(&inputs);
            kernel.on_tick(frame, &SHARED);
        }
    }
}
