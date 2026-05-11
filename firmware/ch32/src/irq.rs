use ch32_metapac::DMA1;

use crate::statics::SHARED;

#[qingke_rt::interrupt]
fn DMA1_CHANNEL1() {
    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        let tick = &raw mut (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        tick.write(tick.read().wrapping_add(1));
    }
}
