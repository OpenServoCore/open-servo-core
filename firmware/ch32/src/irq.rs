use ch32_metapac::DMA1;
use osc_core::FrameInputs;

use crate::statics::{KERNEL, SHARED};

#[qingke_rt::interrupt]
fn DMA1_CHANNEL1() {
    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        // Bump tick before build_sample_frame so it matches *this* scan.
        // Volatile pair so the main-loop pump can't hoist its load out of the
        // spin (and so the bump itself can't be elided by the optimizer).
        let tick = &raw mut (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        tick.write_volatile(tick.read_volatile().wrapping_add(1));

        if let Some(kernel) = (*KERNEL.get()).as_mut() {
            // Scope: pulse period = ISR rate, pulse width = ISR runtime.
            kernel.board.dbg_high();
            let inputs = FrameInputs::snapshot(&SHARED);
            let frame = kernel.board.build_sample_frame(&inputs);
            #[cfg(feature = "defmt")]
            crate::telemetry::record_frame(&frame);
            kernel.on_tick(frame, &SHARED);
            kernel.board.dbg_low();
        }
    }
}
