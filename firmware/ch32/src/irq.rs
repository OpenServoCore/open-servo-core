use ch32_metapac::{DMA1, USART1};
use core::sync::atomic::Ordering;
use osc_core::{FrameInputs, KernelIo, Sensors};
use portable_atomic::{AtomicU16, AtomicU32};

use crate::dxl_fast;
use crate::hal::{dma, gpio, pfic, systick, usart};
use crate::idle_ring;
use crate::statics::{
    DXL_BAUD_PENDING_BRR, DXL_CHAR_TIME_TICKS, DXL_REBOOT_PENDING, DXL_RX_BUF_LEN,
    DXL_RX_BYTE_COUNT, DXL_RX_STAMP_FIRST, DXL_RX_STAMP_LAST, DXL_RX_WRITE_POS, DXL_TX_BUF,
    DXL_TX_EN, KERNEL, SHARED, store_baud_derived,
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

/// Per-packet RX byte stamps. RXNE IRQ is armed at bring-up and re-armed at
/// every IDLE; it fires on byte 1 of the next packet, stamps the tick, latches
/// the ring boundary, and disables itself. Bytes 2..N arrive silently via DMA;
/// LAST recovers from the IDLE backdate, COUNT from NDTR delta against the
/// boundary. One trap per RX packet — sustained ~40% CPU per-byte cost at 3
/// Mbaud is the alternative.
static RX_RUN_FIRST: AtomicU32 = AtomicU32::new(0);
static RX_RUN_START_POS: AtomicU16 = AtomicU16::new(0);
static RX_RUN_BOUNDARY_POS: AtomicU16 = AtomicU16::new(0);

pub fn on_usart1() {
    on_usart1_first_byte_stamp();
    on_usart1_rx_errors();
    on_usart1_idle();
    on_usart1_tc();
}

fn on_usart1_first_byte_stamp() {
    if !usart::is_rxne_irq_enabled(USART1) {
        return;
    }
    let remaining = dma::remaining(dma::Channel::CH5);
    let pos = (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining);
    let boundary = RX_RUN_BOUNDARY_POS.load(Ordering::Relaxed);
    if pos == boundary {
        return;
    }
    let now = systick::ticks();
    RX_RUN_FIRST.store(now, Ordering::Relaxed);
    RX_RUN_START_POS.store(boundary, Ordering::Relaxed);
    usart::set_rxne_irq(USART1, false);
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

    // Publish RX byte-stamp snapshot. START_POS was latched on byte 1's RXNE;
    // LAST falls out of the IDLE backdate (end-of-byte-N tick).
    let start = RX_RUN_START_POS.load(Ordering::Relaxed);
    let count = write_pos.wrapping_sub(start) & mask;
    let first = RX_RUN_FIRST.load(Ordering::Relaxed);
    DXL_RX_STAMP_FIRST.store(first, Ordering::Relaxed);
    DXL_RX_STAMP_LAST.store(request_end_tick, Ordering::Relaxed);
    DXL_RX_BYTE_COUNT.store(count, Ordering::Release);

    // Re-arm for next packet: latch boundary at current write_pos, re-enable
    // the one-shot RXNE IRQ so the next byte 1 trap fires.
    RX_RUN_BOUNDARY_POS.store(write_pos, Ordering::Relaxed);
    usart::set_rxne_irq(USART1, true);
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
        store_baud_derived(pending_brr);
    }
    if DXL_REBOOT_PENDING.load(Ordering::Acquire) {
        pfic::software_reset();
    }
}

#[inline(always)]
pub fn on_systick_match() {
    dxl_fast::on_systick();
}

pub fn on_dma1_ch5_tc() {
    dxl_fast::on_dma1_ch5_tc();
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
        fn DMA1_CHANNEL5() {
            $crate::irq::on_dma1_ch5_tc();
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
