use ch32_metapac::{DMA1, USART1};
use core::sync::atomic::Ordering;
use osc_core::{Board, FrameInputs};

use crate::hal::{dma, gpio, pfic, systick, usart};
use crate::statics::{
    DXL_REBOOT_PENDING, DXL_RX_BUF_LEN, DXL_RX_IDLE_TICK, DXL_RX_WRITE_POS, DXL_TX_BUF, DXL_TX_EN,
    KERNEL, SHARED,
};

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
        let idle_tick = systick::ticks();
        let remaining = dma::remaining(dma::Channel::CH5);
        let write_pos = (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining);
        DXL_RX_WRITE_POS.store(write_pos, Ordering::Release);
        DXL_RX_IDLE_TICK.store(idle_tick, Ordering::Release);
    }

    if usart::is_tc(USART1) {
        usart::set_tc_irq(USART1, false);
        usart::clear_tc(USART1);
        usart::set_dma_tx(USART1, false);
        dma::disable(dma::Channel::CH4);
        if let Some(t) = unsafe { *DXL_TX_EN.get() } {
            gpio::set_level(t.pin, t.idle_level());
        }
        // SAFETY: TC IRQ runs strictly after start_dxl_tx and before any next
        // writer touches DXL_TX_BUF — no concurrent access.
        let buf = unsafe { &mut *DXL_TX_BUF.get() };
        buf.clear();
        if DXL_REBOOT_PENDING.load(Ordering::Acquire) {
            pfic::software_reset();
        }
    }
}

pub fn on_systick_match() {
    systick::clear_match();
    systick::set_irq(false);
    dma::enable(dma::Channel::CH4);
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

        #[::qingke_rt::interrupt(core)]
        fn SysTick() {
            $crate::irq::on_systick_match();
        }
    };
}
