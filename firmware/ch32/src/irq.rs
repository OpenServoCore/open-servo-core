use ch32_metapac::DMA1;
use osc_core::{Board, FrameInputs};

use crate::statics::{KERNEL, SHARED};

#[qingke_rt::interrupt]
fn DMA1_CHANNEL1() {
    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        // Volatile pair: load-bearing against optimizer hoisting in the pump.
        let tick = &raw mut (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        tick.write_volatile(tick.read_volatile().wrapping_add(1));

        if let Some(kernel) = (*KERNEL.get()).as_mut() {
            kernel.board.dbg_high();
            let inputs = FrameInputs::snapshot(&SHARED);
            let frame = kernel.board.sample(&inputs);
            #[cfg(feature = "defmt")]
            crate::telemetry::record_frame(&frame);
            kernel.on_tick(frame, &SHARED);
            kernel.board.dbg_low();
        }
    }
}
