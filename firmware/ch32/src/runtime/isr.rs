use ch32_metapac::{DMA1, USART1};
use osc_core::{ControlIo, ConversionVariables, RegionStorageRaw, Sensors};

use crate::hal::{pfic, systick, usart};
use crate::runtime::Drivers;
use crate::runtime::statics::{KERNEL, SESSION, SHARED};

/// Configures PFIC priorities and unmasks the transport + ADC IRQs. Called
/// once during bringup, after the drivers and statics are installed.
///
/// The transport vectors (USART1 for break/TC, SysTick for the framer
/// deadlines) share PFIC HIGH so all `&mut` access into the `ServoBus`
/// composite serializes. The ADC DMA channel sits at LOW. DMA1_CH5 (RX ring)
/// runs silent circular — no HT/TC IRQ — and CH4/CH3 raise none either.
pub fn install_irqs() {
    pfic::set_priority(pfic::Interrupt::USART1, pfic::Priority::High);
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL1, pfic::Priority::Low);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable_systick();
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
    crate::log::info!("ISRs live");
}

/// ADC DMA TC handler body — wire into the vector table via [`crate::install_isrs!`].
pub fn on_adc_dma_tc() {
    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        // Volatile pair: load-bearing against optimizer hoisting in the pump.
        let tick = &raw mut (*SHARED.table.region_ptr())
            .telemetry
            .intermediaries
            .sample_tick;
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

/// USART1 vector — break detection (RX framing error) and TX arm completion.
///
/// SAFETY: the bus driver is installed before this vector unmasks, and USART1
/// shares PFIC HIGH with SysTick, so no concurrent `&mut` into the composite
/// is possible. Statement ordering is load-bearing: the break handoff runs
/// off the RX-error read, then the TC branch does release work first.
pub fn on_usart1() {
    // (a) RX errors: an FE marks a break (or mid-frame garble) → the framer
    // anchors on the just-ringed 0x00 (F2: the DMA write beats the ISR).
    let errs = usart::rx_errors(USART1);
    if errs.fe || errs.ore || errs.pe || errs.ne {
        if errs.fe {
            // SAFETY: see fn doc.
            unsafe { Drivers::bus() }.on_break();
        }
        // SR-then-DR is the only V006 error clear. DMA already drained DR for
        // the ring byte, so this read cannot steal a payload byte (spike-
        // validated: NDTR unchanged across the DR read).
        usart::clear_rx_errors(USART1);
    }

    // (b) TC: an armed TX arm drained (shifter empty). TCIE gates arbitration
    // — the shared vector fans in RX-errors + TC, and a foreign source could
    // enter with a stale reset-value TC. Gate on TCIE so it can't walk into
    // on_tx_complete before the first reply is armed.
    if usart::is_tcie(USART1) && usart::is_tc(USART1) {
        usart::clear_tc(USART1);
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_tx_complete();
    }
}

/// SysTick CMP-match — one or more framer/chain/rescue deadlines are due.
/// CNTIF is cleared first: a final deadline body returns without re-arming
/// and a stale-but-latched CNTIF would re-fire the IRQ the moment we return.
/// The dispatcher is built per-call from the shared table + the session.
///
/// SAFETY: SysTick shares PFIC HIGH with USART1, so no concurrent `&mut` into
/// the composite (or the session) is possible.
pub fn on_systick() {
    systick::clear_match();
    // SAFETY: see fn doc — SESSION is installed before this vector unmasks.
    let session = unsafe { (*SESSION.get()).assume_init_mut() };
    let mut dispatcher = session.dispatcher(&SHARED);
    // SAFETY: see fn doc.
    unsafe { Drivers::bus() }.on_deadline(&mut dispatcher);
}

/// Wires osc-ch32 ISR bodies into the vector table. Caller must depend on
/// `qingke-rt`. Every ISR opts out of `.highcode` via `#[interrupt(lowcode)]`:
/// the break makes reply timing non-critical (no hardware-timed kickoff on the
/// wire deadline), and the framer's per-deadline slack absorbs any flash-fetch
/// jitter in the remaining bodies.
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

        #[::qingke_rt::interrupt(core, lowcode)]
        fn SysTick() {
            $crate::runtime::isr::on_systick();
        }
    };
}
