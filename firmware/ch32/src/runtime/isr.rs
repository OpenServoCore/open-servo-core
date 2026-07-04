use ch32_metapac::{DMA1, USART1};
use osc_core::{BootMode, ControlIo, ConversionVariables, Sensors};

use crate::hal::{dma, flash, pfic, systick, usart};
use crate::runtime::Drivers;
use crate::runtime::statics::{KERNEL, SERVICES, SHARED};

/// Configures PFIC priorities and unmasks every DXL-transport + ADC IRQ.
/// Called once during bringup, after the drivers and statics are installed.
pub fn install_irqs() {
    pfic::set_priority(pfic::Interrupt::USART1, pfic::Priority::High);
    // DMA1_CH5 carries USART1 RX bytes; the HT/TC ISR publishes the byte-
    // ring head + drives the parser drain (see `on_dma1_ch5`). Shares HIGH
    // with the other DXL ISRs so parser-state mutations serialize.
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL5, pfic::Priority::High);
    // DMA1_CH7's TC fires once per TX kickoff (one-transfer MEM→PER write
    // of the CH4 enable word); the body restores CH4→IC + CH7→edge-ring.
    // Shares HIGH so the restore serializes with the arm/cancel sites.
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL7, pfic::Priority::High);
    // SysTick CMP fires the Fast Last periodic-walk fold body; shares HIGH
    // so all DXL ISR sources serialize.
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL1, pfic::Priority::Low);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL5);
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL7);
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
    on_usart1_status_start();
}

/// DMA1_CH5 HT/TC handler — byte-ring publish plus parser drain. `on_rx_advance`
/// clears the channel's HT/TC flags via the `RxDma` provider and refreshes
/// the codec's `write_seq` from NDTR; `services.poll` then drives the
/// streaming parser over the freshly-published bytes. Per
/// `dxl-streaming-rx.md` §3 / §4.4 / §5.2, parser drains on three
/// triggers (USART1 IDLE, DMA1_CH5 HT, DMA1_CH5 TC); same-handler
/// drain is what makes the §5.1 chain-slot start rule (observe
/// predecessor skip-exhaust → start TX inline) hold under low-RDT
/// timing.
///
/// SAFETY: driver installed before this vector unmasks, and DMA1_CH5
/// shares PFIC HIGH with USART1 / DMA1_CH7 / SysTick so no concurrent `&mut`
/// into the driver is possible.
pub fn on_dma1_ch5() {
    unsafe { Drivers::dxl_uart() }.on_rx_advance();
    // SAFETY: see fn doc.
    let services = unsafe { (*SERVICES.get()).assume_init_mut() };
    services.poll(&SHARED);
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

/// DMA1_CH7 TC — the TX kickoff's one-word transfer completed, meaning the
/// hardware just enabled CH4 at the CC4 compare match and TX is streaming.
/// Restore the channel pair to RX duty (CH4→IC, CH7→edge ring): our own
/// TX produces no RX edges (no self-echo), so the ring comes back live and
/// silent well before the bus turns around. Pure chip-side — no driver
/// routing; the driver observes the restart through
/// `EdgeDma::take_ring_restart` on its next edge publish.
pub fn on_dma1_ch7() {
    crate::providers::tx_kickoff::on_kickoff_complete();
}

fn on_usart1_idle() {
    if !usart::is_idle(USART1) {
        return;
    }
    usart::clear_idle(USART1);
    // Backstop the RX classifier: for packets shorter than half the ET ring
    // the HT/TC ISR never fires, so IDLE is the only chance to walk those
    // edges. `on_rx_idle` refreshes the codec's edge-ring view (`(now,
    // Idle)` ISR-capture stash for the Crc fallback path); the parser
    // drain itself runs via `services.poll` below — IDLE is one of the
    // three parser-drive triggers per `dxl-streaming-rx.md` §3 / §4.4.
    // Anchor invalidation lives at the parser's Crc / Resync event
    // (deterministic packet boundary), not here. IDLE-derived timing
    // per [[no_idle_timing]] never enters the anchored path — the
    // fallback is only consumed at Crc when the classifier was
    // unanchored.
    // SAFETY: see `on_dma1_ch5`.
    unsafe { Drivers::dxl_uart() }.on_rx_idle();
    // SAFETY: see `on_dma1_ch5`.
    let services = unsafe { (*SERVICES.get()).assume_init_mut() };
    services.poll(&SHARED);
}

fn on_usart1_tc() {
    // TCIE gates arbitration: the peripheral would not have raised this vector
    // for TC unless the scheduler armed TCIE=1. The shared USART1 vector fans
    // in RX-errors / IDLE / TC — a foreign source can enter here with TC=1
    // stale from silicon reset (STATR reset = 0xC0). Gate on TCIE so a stale
    // TC bit at boot does not walk into `on_tx_complete` and disarm the first
    // reply's SysTick handoff.
    if !usart::is_tcie(USART1) || !usart::is_tc(USART1) {
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
    // SAFETY: see `on_dma1_ch5`.
    let pending_reboot = unsafe { Drivers::dxl_uart() }.on_tx_complete();
    if let Some(mode) = pending_reboot {
        flash::set_boot_mode(matches!(mode, BootMode::Bootloader));
        pfic::software_reset();
    }
}

/// Per-byte status-start wake for a deferred FAST slot k > 0 (task
/// #142). Gated on CTLR1.RXNEIE — the watch window the RxDma provider
/// opens at `send_slot` and the driver closes on resolution — NOT on
/// `STATR.RXNE`, which always reads 0 in DMA-RX mode (DMA wins the clear
/// race; see the provider doc). While the window is open every USART1
/// vector entry routes one wake; spurious entries (residual-drain race,
/// IDLE sharing the vector) are cursor-qualified inside
/// `on_status_start`, so no flag inspection happens here. Runs after the
/// existing IDLE/TC bodies — additive per [[isr-ordering]].
fn on_usart1_status_start() {
    if !usart::is_rxneie(USART1) {
        return;
    }
    // SAFETY: see `on_dma1_ch5`.
    unsafe { Drivers::dxl_uart() }.on_status_start();
}

/// SysTick CMP-match — a long-horizon scheduling deadline arrived. Two
/// consumers share the CMP: the TX-scheduler handoff (multi-wrap-distance
/// arms that a direct TIM2 compare can't span) and the Fast Last periodic-walk
/// fold body. The driver's `on_schedule_due` demuxes; if the TX scheduler
/// armed the match it consumes, otherwise the fold body runs. CNTIF must
/// be cleared at entry: the final fold body returns without re-arming and
/// a stale-but-latched CNTIF would re-fire the IRQ the moment we return.
///
/// SAFETY: see `on_dma1_ch5` — SysTick shares PFIC HIGH with USART1 /
/// DMA1_CH5 / DMA1_CH7 so no concurrent `&mut` into the driver is possible.
pub fn on_systick() {
    systick::clear_match();
    unsafe { Drivers::dxl_uart() }.on_schedule_due();
}

/// Wires osc-ch32 ISR bodies into the vector table. Caller must depend on
/// `qingke-rt`. Every ISR opts out of `.highcode` via
/// `#[interrupt(lowcode)]` (upstream qingke-rt API): nothing CPU-driven
/// sits on the wire deadline anymore — the TX start is a pure-hardware
/// CC4→DMA kickoff — and per-grid-step / per-packet-end slack absorbs any
/// flash-fetch jitter in the remaining bodies.
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
        fn DMA1_CHANNEL5() {
            $crate::runtime::isr::on_dma1_ch5();
        }

        #[::qingke_rt::interrupt(lowcode)]
        fn DMA1_CHANNEL7() {
            $crate::runtime::isr::on_dma1_ch7();
        }

        #[::qingke_rt::interrupt(core, lowcode)]
        fn SysTick() {
            $crate::runtime::isr::on_systick();
        }
    };
}
