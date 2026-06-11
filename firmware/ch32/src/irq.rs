use ch32_metapac::{DMA1, USART1};
use core::sync::atomic::Ordering;
use osc_core::{FrameInputs, KernelIo, Sensors};

use crate::runtime::Drivers;
use crate::dxl;
use crate::dxl::statics::{
    CLOCK_FINE_TRIM_NO_PENDING, CLOCK_TRIM_NO_PENDING, DXL_BAUD_PENDING_BRR, DXL_CHAR_TIME_TICKS,
    DXL_CLOCK_FINE_TRIM_PENDING, DXL_CLOCK_TRIM_PENDING, DXL_REBOOT_PENDING, DXL_RX_BUF_LEN,
    DXL_RX_EXTI_FIRES, DXL_RX_FIRST_TICK, DXL_RX_FIRST_VALID, DXL_RX_LAST_TICK, DXL_RX_PIN,
    DXL_RX_WRITE_POS, DXL_TX_BUF, DXL_TX_COUNT, DXL_TX_EN, recompute_fire_advance_fine_ticks,
    store_baud_derived,
};
use crate::hal::rcc;
use crate::hal::{dma, exti, gpio, pfic, systick, usart};
use crate::idle_anchor;
use crate::statics::{KERNEL, SHARED};

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
    }
}

pub fn on_usart1() {
    on_usart1_rx_errors();
    on_usart1_idle();
    on_usart1_tc();
}

/// DMA1_CH7 HT/TC handler — dispatches into `DxlRx`, which drains its own
/// adapter, computes the head, and walks newly-captured edges through the
/// window classifier. `ticks_per_bit` is sourced from `DxlClock` (the
/// canonical wire-rate owner).
///
/// SAFETY: both drivers are installed before this vector is unmasked, and
/// DMA1_CH7 shares PFIC HIGH with USART1 so no concurrent `&mut` into
/// either driver is possible.
pub fn on_dma1_ch7() {
    let ticks_per_bit = unsafe { Drivers::dxl_clock() }.ticks_per_bit();
    unsafe { Drivers::dxl_rx() }.on_dma_event(ticks_per_bit);
}

/// EXTI7_0 handler body — covers lines 0..7 on V006's shared vector. Only
/// the DXL RX pin's line is armed; check pending + stamp + disarm.
pub fn on_exti() {
    // SAFETY: bring-up sets `DXL_RX_PIN` before unmasking PFIC EXTI7_0.
    let pin = match unsafe { *DXL_RX_PIN.get() } {
        Some(p) => p,
        None => return,
    };
    if !exti::is_pending(pin) {
        return;
    }
    let tick = systick::ticks();
    // Truly-first-byte stamp: only the first on_exti since the IDLE
    // handler cleared VALID writes FIRST_TICK. LAST_TICK + FIRES expose
    // any extra fires (glitch retriggers) for the bench snoop log.
    if !DXL_RX_FIRST_VALID.load(Ordering::Acquire) {
        DXL_RX_FIRST_TICK.store(tick, Ordering::Release);
        DXL_RX_FIRST_VALID.store(true, Ordering::Release);
    }
    DXL_RX_LAST_TICK.store(tick, Ordering::Release);
    DXL_RX_EXTI_FIRES.fetch_add(1, Ordering::AcqRel);
    exti::set_irq(pin, false);
    exti::clear_pending(pin);
}

fn on_usart1_rx_errors() {
    let errs = usart::rx_errors(USART1);
    if !(errs.ore || errs.pe || errs.fe || errs.ne) {
        return;
    }
    if errs.ore {
        dxl::report_dma_overrun();
    }
    if errs.pe {
        dxl::report_parity_error();
    }
    if errs.fe {
        dxl::report_framing_error();
    }
    if errs.ne {
        dxl::report_noise_error();
    }
    // Per-fire log so a bench `snoop:` trace can align rx errors with
    // bad first-tick samples; clear_rx_errors below also clears IDLE
    // (SR-then-DR side effect), so if this fires at IDLE time the
    // anchor for that packet is silently skipped.
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
    // Consume + snapshot the EXTI-stamped first-byte tick into the anchor
    // before re-arming. Reading FIRST_VALID via swap clears it: any later
    // EXTI fire (for the *next* packet) sets it back to true on its own
    // FIRST_TICK store; the snoop snapshot here is frozen against that.
    let first_valid = DXL_RX_FIRST_VALID.swap(false, Ordering::AcqRel);
    let first_tick = DXL_RX_FIRST_TICK.load(Ordering::Acquire);
    let last_tick = DXL_RX_LAST_TICK.load(Ordering::Acquire);
    let exti_fires = DXL_RX_EXTI_FIRES.swap(0, Ordering::AcqRel);
    idle_anchor::record(
        delta,
        request_end_tick,
        first_tick,
        first_valid,
        last_tick,
        exti_fires,
    );
    // Re-arm EXTI on the RX pin so the next packet's first-byte falling
    // edge stamps DXL_RX_FIRST_TICK. Clear pending before unmask: the
    // just-completed packet's stream of byte-start edges latched the
    // pending bit while the mask was off; an unmask without clear would
    // fire IRQ immediately and stamp on the wrong moment.
    // SAFETY: see on_exti.
    if let Some(p) = unsafe { *DXL_RX_PIN.get() } {
        exti::clear_pending(p);
        exti::set_irq(p, true);
    }
    // Backstop the DxlRx classifier: for packets shorter than half the ET
    // ring, the HT/TC ISR never fires, so IDLE is the only chance to walk
    // those edges. `on_idle` drains the tail and invalidates the anchor so
    // the next packet's first edge re-seeds.
    // SAFETY: see on_dma1_ch7.
    let ticks_per_bit = unsafe { Drivers::dxl_clock() }.ticks_per_bit();
    unsafe { Drivers::dxl_rx() }.on_idle(ticks_per_bit);
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
    dxl::cancel();
    DXL_TX_COUNT.fetch_add(1, Ordering::Relaxed);
    // dxl::cancel masks the EXTI stamp line (chain-mode safety); re-arm so
    // the next inbound packet's first-byte edge still stamps. IDLE re-arm
    // covers the no-reply path; this covers the reply-then-RX path.
    // SAFETY: see on_exti.
    if let Some(p) = unsafe { *DXL_RX_PIN.get() } {
        exti::clear_pending(p);
        exti::set_irq(p, true);
    }
    apply_pending_after_tc();
}

/// Drain queued config writes (baud / clock_trim / fine_trim / reboot) after
/// the reply has fully drained the wire. Runs at the tail of `on_usart1_tc`
/// — off the wire-fire jitter path, so individual handlers can take their
/// time. Each slot is consume-on-swap so a back-to-back reply that re-queues
/// the same knob doesn't apply twice.
#[inline(always)]
fn apply_pending_after_tc() {
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
    dxl::on_systick();
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

        #[::qingke_rt::interrupt]
        fn EXTI7_0() {
            $crate::irq::on_exti();
        }

        #[::qingke_rt::interrupt]
        fn DMA1_CHANNEL7() {
            $crate::irq::on_dma1_ch7();
        }

        #[::qingke_rt::interrupt(core)]
        fn SysTick() {
            $crate::irq::on_systick_match();
        }
    };
}
