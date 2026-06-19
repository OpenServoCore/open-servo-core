use ch32_metapac::{DMA1, USART1};
use osc_core::{BootMode, ControlIo, ConversionVariables, Sensors};

use crate::hal::{dma, flash, pfic, systick, usart};
use crate::runtime::Drivers;
use crate::runtime::statics::{KERNEL, SHARED};

/// Configures PFIC priorities and unmasks every DXL-transport + ADC IRQ.
/// Called once during bringup, after the drivers and statics are installed.
pub fn install_irqs() {
    pfic::set_priority(pfic::Interrupt::USART1, pfic::Priority::High);
    // DMA1_CH7 carries TIM2_CH4 edge timestamps; the HT/TC ISR walks them
    // through the RX classifier and IDLE drains the tail. Share HIGH
    // with USART1 so on_dma1_ch7 and on_usart1_idle serialize (same prio →
    // no preemption) — classifier state is mutated from both paths.
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL7, pfic::Priority::High);
    // TIM2 CC3 IRQ kicks the wire-driver activate sequence; shares HIGH with
    // USART1 + DMA1_CH7 so the DXL transport's three IRQ sources serialize.
    pfic::set_priority(pfic::Interrupt::TIM2, pfic::Priority::High);
    // SysTick CMP fires the Fast Last periodic-walk fold body; shares HIGH
    // with USART1 / DMA1_CH7 / TIM2 so all four DXL ISR sources serialize.
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL1, pfic::Priority::Low);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL7);
    pfic::enable(pfic::Interrupt::TIM2);
    pfic::enable_systick();
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
    crate::log::info!("ISRs live");
}

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
/// classifier walk. `now` (TIM2.CNT) is captured at ISR entry — the
/// driver stashes it on the codec RX half so the parser's Crc handler
/// can pick a fallback `packet_end_tick` if the classifier was
/// unanchored.
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
    // edges. `on_rx_idle` drains the tail; anchor invalidation lives at
    // the parser's Crc / Resync event (deterministic packet boundary),
    // not here. The driver sources its fallback wake-capture from the
    // WireClock provider — only consumed at Crc when the classifier was
    // unanchored, so IDLE-derived timing per [[no_idle_timing]] never
    // enters the anchored path.
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

/// SysTick CMP-match — a long-horizon scheduling deadline arrived. Two
/// consumers share the CMP: the TX-scheduler handoff (multi-wrap-distance
/// arms that direct TIM2 CC3 can't span) and the Fast Last periodic-walk
/// fold body. The driver's `on_schedule_due` demuxes; if the TX scheduler
/// armed the match it consumes, otherwise the fold body runs. CNTIF must
/// be cleared at entry: the final fold body returns without re-arming and
/// a stale-but-latched CNTIF would re-fire the IRQ the moment we return.
///
/// SAFETY: see `on_dma1_ch7` — SysTick shares PFIC HIGH with USART1 /
/// DMA1_CH7 / TIM2 so no concurrent `&mut` into the driver is possible.
pub fn on_systick() {
    systick::clear_match();
    unsafe { Drivers::dxl_uart() }.on_schedule_due();
}

/// Wires osc-ch32 ISR bodies into the vector table. Caller must depend on
/// `qingke-rt`. Only the TIM2 vector lands in `.highcode` (RAM) — its
/// CC3 post-fire fold races CH4's DMA prefetch with ~10 byte-times slack
/// at 3 Mbaud GUARD=1, so the body must skip the flash-fetch path. The
/// other DXL/ADC ISRs opt out via `#[interrupt(lowcode)]` (upstream
/// qingke-rt API): their bodies land in `.text.{NAME}` (flash) since
/// per-grid-step or per-packet-end slack absorbs any flash-fetch jitter.
#[macro_export]
macro_rules! install_isrs {
    () => {
        #[::qingke_rt::interrupt(lowcode)]
        fn DMA1_CHANNEL1() {
            $crate::runtime::isr::on_adc_dma_tc();
        }

        #[::qingke_rt::interrupt(lowcode)]
        fn USART1() {
            $crate::runtime::isr::on_usart1();
        }

        #[::qingke_rt::interrupt(lowcode)]
        fn DMA1_CHANNEL7() {
            $crate::runtime::isr::on_dma1_ch7();
        }

        #[::qingke_rt::interrupt]
        fn TIM2() {
            $crate::runtime::isr::on_tim2_cc3();
        }

        #[::qingke_rt::interrupt(core, lowcode)]
        fn SysTick() {
            $crate::runtime::isr::on_systick();
        }
    };
}
