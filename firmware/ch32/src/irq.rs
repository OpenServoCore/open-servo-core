use ch32_metapac::{DMA1, USART1};
use core::sync::atomic::Ordering;
use osc_core::{FrameInputs, KernelIo, Sensors};

use crate::dxl_fast;
use crate::hal::rcc;
use crate::hal::{dma, gpio, pfic, systick, usart};
use crate::idle_ring;
use crate::statics::{
    CLOCK_FINE_TRIM_NO_PENDING, CLOCK_TRIM_NO_PENDING, DXL_BAUD_PENDING_BRR, DXL_CHAR_TIME_TICKS,
    DXL_CLOCK_FINE_TRIM_PENDING, DXL_CLOCK_TRIM_PENDING, DXL_REBOOT_PENDING, DXL_RX_BUF_LEN,
    DXL_RX_WRITE_POS, DXL_TX_BUF, DXL_TX_EN, KERNEL, SHARED, recompute_fire_advance_fine_ticks,
    store_baud_derived,
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
        let inputs = FrameInputs::snapshot(&SHARED);
        let frame = {
            let (sensors, _motor) = kernel.io.parts();
            sensors.sample(&inputs)
        };
        #[cfg(feature = "defmt")]
        crate::telemetry::record_frame(&frame);
        kernel.on_tick(frame, &SHARED);
        kernel.io.dbg.pulse_tick();
    }
}

pub fn on_usart1() {
    on_usart1_rx_errors();
    on_usart1_idle();
    // RXNE entries happen only when dxl_fast armed the tail-tier — the
    // handler self-gates via STATE, so IDLE/TC-only wakes pay one branch
    // each. STATR.RXNE always reads 0 in DMA-RX mode (V006 quirk: DMA
    // wins the clear race), so there's no flag to check here.
    dxl_fast::on_rxne();
    on_usart1_tc();
}

fn on_usart1_rx_errors() {
    let errs = usart::rx_errors(USART1);
    if !(errs.ore || errs.pe || errs.fe || errs.ne) {
        return;
    }
    if errs.ore {
        dxl_fast::report_dma_overrun();
    }
    if errs.pe {
        dxl_fast::report_parity_error();
    }
    if errs.fe {
        dxl_fast::report_framing_error();
    }
    if errs.ne {
        dxl_fast::report_noise_error();
    }
    // SR-then-DR clear is the only V006 path. Called only from on_usart1
    // entry — post-IDLE or post-TC, both packet boundaries — so DMA has
    // already drained DR and the extra DR read can't steal a pending byte.
    usart::clear_rx_errors(USART1);
}

fn on_usart1_idle() {
    if !usart::is_idle(USART1) {
        return;
    }
    // USART IDLE asserts 1 char-time after the wire returns to idle; backdate
    // to recover the request end tick the dispatcher's fire_us math expects.
    let request_end_tick =
        systick::ticks().wrapping_sub(DXL_CHAR_TIME_TICKS.load(Ordering::Relaxed));
    usart::clear_idle(USART1);
    let remaining = dma::remaining(dma::Channel::CH5);
    let write_pos = (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining);
    let prev = DXL_RX_WRITE_POS.load(Ordering::Relaxed);
    let mask = (DXL_RX_BUF_LEN as u16).wrapping_sub(1);
    let delta = write_pos.wrapping_sub(prev) & mask;
    DXL_RX_WRITE_POS.store(write_pos, Ordering::Release);
    idle_ring::record(delta, request_end_tick);
}

fn on_usart1_tc() {
    if !usart::is_tc(USART1) {
        return;
    }
    // Guard against spurious mid-stream TC: a per-byte TC oscillation can
    // fire ISR before CH4 has DMA'd all bytes. Acting on it would cut the
    // stream short. NDTR>0 means TX still in progress — clear flag, keep
    // TCIE on, wait for real end.
    if dma::remaining(dma::Channel::CH4) != 0 {
        usart::clear_tc(USART1);
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
        store_baud_derived(pending_brr);
    }
    let pending_trim = DXL_CLOCK_TRIM_PENDING.swap(CLOCK_TRIM_NO_PENDING, Ordering::AcqRel);
    if pending_trim != CLOCK_TRIM_NO_PENDING {
        rcc::apply_clock_trim_delta(pending_trim as i8);
    }
    let pending_fine =
        DXL_CLOCK_FINE_TRIM_PENDING.swap(CLOCK_FINE_TRIM_NO_PENDING, Ordering::AcqRel);
    if pending_fine != CLOCK_FINE_TRIM_NO_PENDING {
        recompute_fire_advance_fine_ticks(pending_fine as i16);
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
