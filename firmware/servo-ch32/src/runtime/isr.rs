use core::cell::SyncUnsafeCell;

use ch32_metapac::{DMA1, USART1};
use osc_servo_core::traits::{Dispatch, Dispatched, Reply, Request, RequestCtx};
use osc_servo_core::{ControlIo, RegionStorageRaw, Sensors};

use crate::hal::{pfic, systick, usart};
use crate::providers::break_wake::BreakWake;
use crate::runtime::Drivers;
use crate::runtime::statics::{KERNEL, SESSION, SHARED, TEL_CHANNEL};
use crate::runtime::tick_load::TickLoad;

/// Touched only from the DMA1 CH1 vector, which never preempts itself.
static TICK_LOAD: SyncUnsafeCell<TickLoad> = SyncUnsafeCell::new(TickLoad::new());

/// Configures PFIC priorities and unmasks the transport + ADC IRQs. Called
/// once during bringup, after the drivers and statics are installed.
///
/// The transport vectors (TIM2 for the break wake, USART1 for TC, SysTick
/// for the framer deadlines) share PFIC HIGH so all `&mut` access into the
/// `ServoBus` composite serializes -- dispatch runs inline on these vectors.
/// LOW holds only the motor kernel (DMA1_CH1 = 22), which HIGH preempts and
/// which runs in the wire gaps between frames. DMA1_CH5 (RX ring) runs
/// silent circular -- no HT/TC IRQ -- and CH3/CH4/CH6/CH7 raise none either.
pub fn install_irqs() {
    pfic::set_priority(pfic::Interrupt::TIM2, pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::USART1, pfic::Priority::High);
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL1, pfic::Priority::Low);
    pfic::enable(pfic::Interrupt::TIM2);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable_systick();
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
    crate::log::info!("ISRs live");
}

/// HIGH-side dispatcher: materializes the `SESSION` borrow inside each
/// `Dispatch` method instead of holding one across the whole ISR body.
///
/// SAFETY (the SESSION exclusivity invariant): `SESSION` is touched only by
/// the HIGH transport ISRs (TIM2 + USART1 + SysTick), which share PFIC HIGH
/// and so never preempt each other -- dispatch at the covered checkpoint /
/// fast path and the verdict commit/revert all run to completion within one
/// HIGH body.
/// No other class reaches the session.
struct HighDispatcher;

impl HighDispatcher {
    #[inline(always)]
    fn with<R>(&mut self, f: impl FnOnce(&mut osc_servo_core::Dispatcher<'_>) -> R) -> R {
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
    let entry = systick::ticks();
    // SAFETY: see TICK_LOAD.
    let load = unsafe { &mut *TICK_LOAD.get() };
    // A shunt burst time-shares DMA1 CH1, so its HT and TC arrive on this
    // vector; the buffer then holds raw shunt codes, not a 7-slot scan, and
    // nothing below may run against it.
    if crate::control::burst::capturing() {
        load.skip();
        crate::control::burst::on_dma_event(&SHARED);
        return;
    }
    // Ahead of the body on purpose: the scan-geometry witness reads TIM1's
    // counting phase, which is only meaningful this close to the TC.
    crate::control::burst::witness_scan_tc(&SHARED);

    DMA1.ifcr().write(|w| w.set_tcif(0, true));

    unsafe {
        // Volatile pair: load-bearing against optimizer hoisting in the pump.
        let tick = &raw mut (*SHARED.table.region_ptr()).telemetry.estimates.sample_tick;
        tick.write_volatile(tick.read_volatile().wrapping_add(1));

        // SAFETY: PFIC unmasks DMA1_CHANNEL1 only after install_kernel writes KERNEL.
        let kernel = (*KERNEL.get()).assume_init_mut();
        let frame = {
            let (sensors, _motor) = kernel.io.parts();
            sensors.frame()
        };
        kernel.on_tick(frame, &SHARED);
    }

    // Trailing on purpose: the burst handshake must never displace a kernel
    // tick, and a launch wants the scan TC's slack ahead of the next trigger.
    crate::control::burst::poll_arm(&SHARED);

    if let Some(w) = load.tick(entry, systick::ticks()) {
        // SAFETY: table storage is 'static; the health block's tick fields
        // have this vector as their only chip-side writer. The counters are
        // stored only on a change, so a host clear races a store only when
        // one is due. KERNEL as above.
        unsafe {
            let h = &raw mut (*SHARED.table.region_ptr()).telemetry.health;
            if let Some(mean) = w.mean_q15 {
                (&raw mut (*h).tick_load_mean_q15).write_volatile(mean);
            }
            if w.over != 0 {
                let over = &raw mut (*h).tick_over_count;
                over.write_volatile(over.read_volatile().wrapping_add(w.over));
            }
            if w.lost != 0 {
                let lost = &raw mut (*h).tick_lost_count;
                lost.write_volatile(lost.read_volatile().wrapping_add(w.lost));
                (*KERNEL.get()).assume_init_mut().lost_ticks(w.lost);
            }
        }
    }

    // TEL burst (protocol sec 5.6): a six-field frame drains in ~690 us of
    // the 800 us its successor takes to fill, so a batch must stage within
    // a tick of banking or of the wire freeing; the main loop, starved by a
    // driving tick, staged up to 300 us late and the kernel dropped rows.
    // After the load stamp: staging is transport work, not the kernel's.
    // ISRs masked: `bus()` is HIGH-owned.
    if TEL_CHANNEL.active() {
        // SAFETY: bus installed in bringup; ISRs masked by the CS.
        critical_section::with(|_| unsafe { Drivers::bus() }.poll_tel());
    }
}

/// TIM2 vector -- the break wake (`providers::break_wake`): an overflow
/// after 9.25 bit-times of continuous low is a break, unless it is the
/// same low again after a park.
///
/// SAFETY: the bus driver is installed before this vector unmasks, and TIM2
/// shares PFIC HIGH with USART1 and SysTick, so no concurrent `&mut` into
/// the composite is possible.
pub fn on_tim2() {
    crate::log::trace!("tim2 isr");
    let entry = crate::probe::stamp();
    if BreakWake::service() {
        // The break handler resolves complete frames from ring data in
        // place (transport sec 5), so it carries the (lazy) HIGH dispatcher
        // like the deadline body.
        let mut dispatcher = HighDispatcher;
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_break(&mut dispatcher);
    }
    crate::probe::high_probe(|p| p.tim2.exit(entry));
}

/// USART1 vector -- TX arm completion. TCIE is the one enabled source.
///
/// SAFETY: the bus driver is installed before this vector unmasks, and USART1
/// shares PFIC HIGH with TIM2 and SysTick, so no concurrent `&mut` into the
/// composite is possible.
pub fn on_usart1() {
    crate::log::trace!("usart1 isr");
    let entry = crate::probe::stamp();
    // This path never reads DATAR: a CPU DATAR read while a byte is
    // mid-reception kills the byte in the shifter -- no flags, no ring
    // entry, every later anchor shifts (measured; the DMA ladder only
    // protects the byte already in RDR).
    //
    // TC: an armed TX arm drained (shifter empty). Gate on TCIE so a stale
    // reset-value TC can't walk into on_tx_complete before the first reply
    // is armed.
    //
    // TC is NOT cleared here -- `TxWire::send` clears it per-arm once the
    // next arm's first byte is in flight, and the final arm's release drops
    // TCIE, leaving TC=1 as the natural idle state (STATR reset 0xC0).
    if usart::is_tc(USART1) && usart::is_tcie(USART1) {
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_tx_complete();
    }
    crate::probe::high_probe(|p| p.usart1.exit(entry));
}

/// SysTick compare -- one or more framer/chain/rescue deadlines are due, or a
/// `pend_systick` late-arm wake. CNTIF is cleared first: a final deadline body
/// returns without re-arming and a stale-but-latched flag would re-fire
/// the IRQ the moment we return.
///
/// SAFETY: SysTick shares PFIC HIGH with TIM2 and USART1, so no concurrent
/// `&mut` into the composite is possible; SESSION access goes through the lazy
/// [`HighDispatcher`] under its exclusivity invariant.
pub fn on_deadline_irq() {
    crate::log::trace!("deadline isr");
    let entry = crate::probe::stamp();
    crate::hal::systick::clear_match();
    let mut dispatcher = HighDispatcher;
    // SAFETY: see fn doc.
    unsafe { Drivers::bus() }.on_deadline(&mut dispatcher);
    crate::probe::high_probe(|p| p.systick.exit(entry));
}

/// Wires osc-servo-ch32 ISR bodies into the vector table via the stock
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
        fn TIM2() {
            $crate::runtime::isr::on_tim2();
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
