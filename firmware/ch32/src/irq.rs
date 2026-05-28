use ch32_metapac::{DMA1, USART1};
use core::sync::atomic::Ordering;
use osc_core::{FrameInputs, KernelIo, Sensors};

use crate::dxl_fast;
use crate::hal::{dma, gpio, pfic, systick, usart};
use crate::idle_ring;
use crate::statics::{
    DXL_BAUD_PENDING_BRR, DXL_REBOOT_PENDING, DXL_RX_BUF_LEN, DXL_RX_WRITE_POS, DXL_TX_BUF,
    DXL_TX_EN, KERNEL, SHARED,
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
        kernel.io.dbg.scope_high();
        let inputs = FrameInputs::snapshot(&SHARED);
        let frame = {
            let (sensors, _motor) = kernel.io.parts();
            sensors.sample(&inputs)
        };
        #[cfg(feature = "defmt")]
        crate::telemetry::record_frame(&frame);
        kernel.on_tick(frame, &SHARED);
        kernel.io.dbg.scope_low();
        kernel.io.dbg.pulse_tick();
    }
}

pub fn on_usart1() {
    on_usart1_idle();
    on_usart1_tc();
    // RXNE is self-gated by dxl_fast's stage; cheap when no snoop is active.
    // STATR.RXNE always reads 0 in-ISR (DMA wins the clear race), so calling
    // unconditionally on every USART1 entry is the only reliable trigger.
    dxl_fast::on_rxne();
}

fn on_usart1_idle() {
    if !usart::is_idle(USART1) {
        return;
    }
    usart::clear_idle(USART1);
    let idle_tick = systick::ticks();
    let remaining = dma::remaining(dma::Channel::CH5);
    let write_pos = (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining);
    let prev = DXL_RX_WRITE_POS.load(Ordering::Relaxed);
    let delta = write_pos.wrapping_sub(prev) % (DXL_RX_BUF_LEN as u16);
    DXL_RX_WRITE_POS.store(write_pos, Ordering::Release);
    idle_ring::record(delta, idle_tick);
}

fn on_usart1_tc() {
    if !usart::is_tc(USART1) {
        return;
    }
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
    dxl_fast::cancel();
    let pending_brr = DXL_BAUD_PENDING_BRR.swap(0, Ordering::AcqRel);
    if pending_brr != 0 {
        usart::set_baud(USART1, pending_brr);
    }
    if DXL_REBOOT_PENDING.load(Ordering::Acquire) {
        pfic::software_reset();
    }
}

#[inline(always)]
pub fn on_systick_match() {
    dxl_fast::on_systick();
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
