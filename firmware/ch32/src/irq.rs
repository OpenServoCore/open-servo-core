use ch32_metapac::{DMA1, USART1};
use core::sync::atomic::Ordering;
use osc_core::{Board, FrameInputs};

use crate::hal::{dma, usart};
use crate::statics::{DXL_RX_BUF_LEN, KERNEL, SHARED};

/// ADC DMA TC handler body — wire into the vector table via [`crate::install_isrs!`].
pub fn on_adc_dma_tc() {
    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        // Volatile pair: load-bearing against optimizer hoisting in the pump.
        let tick = &raw mut (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        tick.write_volatile(tick.read_volatile().wrapping_add(1));

        // SAFETY: PFIC unmasks DMA1_CHANNEL1 only after install_kernel writes KERNEL.
        let kernel = (*KERNEL.get()).assume_init_mut();
        kernel.board.dbg_high();
        let inputs = FrameInputs::snapshot(&SHARED);
        let frame = kernel.board.sample(&inputs);
        #[cfg(feature = "defmt")]
        crate::telemetry::record_frame(&frame);
        kernel.on_tick(frame, &SHARED);
        kernel.board.dbg_low();
    }
}

pub fn on_usart1() {
    if usart::is_idle(USART1) {
        usart::clear_idle(USART1);
        let remaining = dma::remaining(dma::Channel::CH5);
        let write_pos = (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining);
        SHARED
            .stream
            .dxl_rx_write_pos
            .store(write_pos, Ordering::Release);
    }
}

/// Wires osc-ch32 ISR bodies into the vector table. Caller must depend on `qingke-rt`.
#[macro_export]
macro_rules! install_isrs {
    () => {
        #[::qingke_rt::interrupt]
        fn DMA1_CHANNEL1() {
            $crate::irq::on_adc_dma_tc();
        }

        #[::qingke_rt::interrupt]
        fn USART1() {
            $crate::irq::on_usart1();
        }
    };
}
