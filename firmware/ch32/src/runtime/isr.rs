use ch32_metapac::{DMA1, USART1};
use osc_core::traits::{Dispatch, Dispatched, Reply, Request, RequestCtx};
use osc_core::{ControlIo, ConversionVariables, RegionStorageRaw, Sensors};

use crate::hal::{pfic, usart};
use crate::runtime::Drivers;
use crate::runtime::statics::{KERNEL, SESSION, SHARED};

/// Configures PFIC priorities and unmasks the transport + ADC IRQs. Called
/// once during bringup, after the drivers and statics are installed.
///
/// The transport vectors (USART1 for break/TC, SysTick for the framer
/// deadlines) share PFIC HIGH so all `&mut` access into the `ServoBus`
/// composite serializes -- dispatch runs inline on these vectors. LOW holds
/// only the motor kernel (DMA1_CH1 = 22), which HIGH preempts and which runs
/// in the wire gaps between frames. DMA1_CH5 (RX ring) runs silent circular --
/// no HT/TC IRQ -- and CH4/CH3 raise none either.
pub fn install_irqs() {
    pfic::set_priority(pfic::Interrupt::USART1, pfic::Priority::High);
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL1, pfic::Priority::Low);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable_systick();
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
    crate::log::info!("ISRs live");
}

/// HIGH-side dispatcher: materializes the `SESSION` borrow inside each
/// `Dispatch` method instead of holding one across the whole ISR body.
///
/// SAFETY (the SESSION exclusivity invariant): `SESSION` is touched only by
/// the HIGH transport ISRs (USART1 + SysTick), which share PFIC HIGH and so
/// never preempt each other -- dispatch at the covered checkpoint / fast path
/// and the verdict commit/revert all run to completion within one HIGH body.
/// No other class reaches the session.
struct HighDispatcher;

impl HighDispatcher {
    #[inline(always)]
    fn with<R>(&mut self, f: impl FnOnce(&mut osc_core::Dispatcher<'_>) -> R) -> R {
        // SAFETY: see type doc -- HIGH-exclusive, no concurrent borrow.
        let session = unsafe { (*SESSION.get()).assume_init_mut() };
        f(&mut session.dispatcher(&SHARED))
    }
}

impl Dispatch for HighDispatcher {
    fn dispatch<R: Reply>(
        &mut self,
        req: Request<'_>,
        ctx: RequestCtx,
        reply: &mut R,
    ) -> Dispatched {
        self.with(|d| d.dispatch(req, ctx, reply))
    }

    fn commit<R: Reply>(&mut self, reply: &mut R) {
        self.with(|d| d.commit(reply))
    }

    fn revert(&mut self) {
        self.with(|d| d.revert())
    }
}

/// ADC DMA TC handler body -- wire into the vector table via [`crate::install_isrs!`].
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

/// USART1 vector -- break detection (LBD) and TX arm completion.
///
/// SAFETY: the bus driver is installed before this vector unmasks, and USART1
/// shares PFIC HIGH with SysTick, so no concurrent `&mut` into the composite
/// is possible. Statement ordering is load-bearing: one STATR image is taken
/// at entry, the break handler runs first, then the TC branch.
pub fn on_usart1() {
    crate::log::trace!("usart1 isr");
    // One STATR image; the two enabled sources (LBDIE, TCIE) branch off it.
    // This path never reads DATAR: a CPU DATAR read while a byte is
    // mid-reception kills the byte in the shifter -- no flags, no ring
    // entry, every later anchor shifts (measured; the DMA ladder
    // only protects the byte already in RDR).
    let sr = usart::statr(USART1);

    // (a) LBD: a genuine >=10-bit dominant span -- a break, and the ONLY RX
    // wake (protocol sec 3.4: garble never interrupts; FE/NE/ORE have no enable
    // and latch silently). Cleared here by the flag-selective constant write,
    // so the level pend is retired before the handler runs -- a break
    // landing mid-body re-pends the vector, nothing is lost, and no flag
    // can storm. on_break is idempotent (transport sec 5: position from ring data).
    if sr.lbd() {
        usart::clear_lbd(USART1);
        // The break handler resolves complete frames from ring data in
        // place (transport sec 5), so it carries the (lazy) HIGH dispatcher
        // like the deadline body.
        let mut dispatcher = HighDispatcher;
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_break(&mut dispatcher);
    }

    // (b) TC: an armed TX arm drained (shifter empty). TCIE gates arbitration
    // -- the shared vector fans in LBD + TC, and a break entry could land
    // with a stale reset-value TC. Gate on TCIE so it can't walk into
    // on_tx_complete before the first reply is armed.
    //
    // TC is NOT cleared here -- `TxWire::send` clears it per-arm once the
    // next arm's first byte is in flight, and the final arm's release drops
    // TCIE, leaving TC=1 as the natural idle state (STATR reset 0xC0).
    if sr.tc() && usart::is_tcie(USART1) {
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_tx_complete();
    }

    // Trailing on purpose (statement order above is jitter-tuned): any
    // wire event marks the bus as talking for the main loop's LED policy.
    crate::runtime::registry::BUS_ACTIVITY.store(true, portable_atomic::Ordering::Relaxed);
}

/// SysTick compare -- one or more framer/chain/rescue deadlines are due, or a
/// `pend_systick` late-arm wake. CNTIF is cleared first: a final deadline body
/// returns without re-arming and a stale-but-latched flag would re-fire
/// the IRQ the moment we return.
///
/// SAFETY: SysTick shares PFIC HIGH with USART1, so no concurrent `&mut`
/// into the composite is possible; SESSION access goes through the lazy
/// [`HighDispatcher`] under its exclusivity invariant.
pub fn on_deadline_irq() {
    crate::log::trace!("deadline isr");
    crate::hal::systick::clear_match();
    let mut dispatcher = HighDispatcher;
    // SAFETY: see fn doc.
    unsafe { Drivers::bus() }.on_deadline(&mut dispatcher);
}

/// Wires osc-ch32 ISR bodies into the vector table via the stock
/// `#[qingke_rt::interrupt]` trampolines (save-ra + HPE hardware stacking,
/// INTSYSCR=0x3 from qingke-rt's startup).
///
/// The "HPE corrupts t0/t2" concern is DEBUNKED (bringup `hpe_matrix`):
/// stock trampolines survived ~8M IRQ crossings across
/// single/nested/tail-chain/critical-section/100 kHz-storm legs with zero
/// corruption. The corruptor was `wlink write-mem`, which resumes the hart
/// with its scratch registers leaked into the running context -- t0 = the
/// poked address, t2 = the poked value (canary-captured verbatim). Debug
/// pokes perturb t0/t2; the runtime is sound.
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

        #[::qingke_rt::interrupt(core)]
        fn SysTick() {
            $crate::runtime::isr::on_deadline_irq();
        }
    };
}
