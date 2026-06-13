use ch32_metapac::{DMA1, USART1};
use osc_core::{BootMode, ControlIo, ConversionVariables, Sensors};

use crate::hal::{dma, flash, pfic, usart};
use crate::legacy::statics::{KERNEL, SHARED};
use crate::runtime::Drivers;

/// ADC DMA TC handler body — wire into the vector table via [`crate::install_isrs!`].
pub fn on_adc_dma_tc() {
    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        // Volatile pair: load-bearing against optimizer hoisting in the pump.
        let tick = &raw mut (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        tick.write_volatile(tick.read_volatile().wrapping_add(1));

        // SAFETY: PFIC unmasks DMA1_CHANNEL1 only after install_kernel writes KERNEL.
        let kernel = (*KERNEL.get()).assume_init_mut();
        let vars = ConversionVariables::snapshot(&SHARED);
        let sample = {
            let (sensors, _motor) = kernel.io.parts();
            sensors.sample(&vars)
        };
        #[cfg(feature = "defmt")]
        crate::legacy::telemetry::record_sample(&sample);
        kernel.on_tick(sample, &SHARED);
    }
}

pub fn on_usart1() {
    on_usart1_rx_errors();
    on_usart1_idle();
    on_usart1_tc();
}

/// DMA1_CH7 HT/TC handler — dispatches into `DxlUart`, which routes the
/// `ticks_per_bit` lookup from its `clock` sub-driver into the RX
/// classifier walk.
///
/// SAFETY: driver is installed before this vector is unmasked, and
/// DMA1_CH7 shares PFIC HIGH with USART1 so no concurrent `&mut` into the
/// driver is possible.
pub fn on_dma1_ch7() {
    unsafe { Drivers::dxl_uart() }.on_rx_edge_advance();
}

fn on_usart1_rx_errors() {
    let errs = usart::rx_errors(USART1);
    if !(errs.ore || errs.pe || errs.fe || errs.ne) {
        return;
    }
    // M2 (#33): RX-error counters previously routed through `legacy::dxl::state`
    // — gone with the legacy drain. M3+ will surface them via the driver's
    // telemetry surface (TBD). For now the log line is the only visible
    // record of an RX-error edge.
    crate::log::info!(
        "rxerr: ore={} pe={} fe={} ne={}",
        errs.ore,
        errs.pe,
        errs.fe,
        errs.ne,
    );
    // SR-then-DR clear is the only V006 path. Called only from on_usart1
    // entry — post-IDLE or post-TC, both packet boundaries — so DMA has
    // already drained DR and the extra DR read can't steal a pending byte.
    usart::clear_rx_errors(USART1);
}

fn on_usart1_idle() {
    if !usart::is_idle(USART1) {
        return;
    }
    usart::clear_idle(USART1);
    // Backstop the RX classifier: for packets shorter than half the ET ring
    // the HT/TC ISR never fires, so IDLE is the only chance to walk those
    // edges. `on_rx_idle` drains the tail and invalidates the anchor so the
    // next packet's first edge re-seeds. IDLE is *only* a signal here — no
    // tick is derived from it per [[no_idle_timing]].
    // SAFETY: see `on_dma1_ch7`.
    unsafe { Drivers::dxl_uart() }.on_rx_idle();
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
    usart::clear_tc(USART1);
    // SAFETY: see `on_dma1_ch7`.
    let pending_reboot = unsafe { Drivers::dxl_uart() }.on_tx_complete();
    if let Some(mode) = pending_reboot {
        flash::set_boot_mode(matches!(mode, BootMode::Bootloader));
        pfic::software_reset();
    }
}

/// TIM2 CC3 compare-match — TX-start deadline reached. Routes into the
/// driver's `on_tx_start`, which delegates to the scheduler provider to
/// activate the wire driver.
///
/// Hot path: no pending-flag check (CC3IE is the only IRQ enabled on TIM2
/// — nothing else can fire this vector) and no flag clear (the scheduler
/// provider masks CC3IE in `handle_start`; CC3IF stays set but harmless,
/// and the next `schedule()` clears it before re-arming CC3IE).
///
/// SAFETY: see `on_dma1_ch7` — TIM2 shares PFIC HIGH with USART1 / DMA1_CH7
/// so no concurrent `&mut` into the driver is possible.
pub fn on_tim2_cc3() {
    unsafe { Drivers::dxl_uart() }.on_tx_start();
}

/// Wires osc-ch32 ISR bodies into the vector table. Caller must depend on `qingke-rt`.
#[macro_export]
macro_rules! install_isrs {
    () => {
        #[::qingke_rt::interrupt]
        fn DMA1_CHANNEL1() {
            $crate::runtime::isr::on_adc_dma_tc();
        }

        #[::qingke_rt::interrupt]
        fn USART1() {
            $crate::runtime::isr::on_usart1();
        }

        #[::qingke_rt::interrupt]
        fn DMA1_CHANNEL7() {
            $crate::runtime::isr::on_dma1_ch7();
        }

        #[::qingke_rt::interrupt]
        fn TIM2() {
            $crate::runtime::isr::on_tim2_cc3();
        }
    };
}
