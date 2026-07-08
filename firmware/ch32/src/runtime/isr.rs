use ch32_metapac::{DMA1, USART1};
use osc_core::traits::{Dispatch, Reply, Request, RequestCtx, Speculated};
use osc_core::{ControlIo, ConversionVariables, RegionStorageRaw, Sensors};

use crate::hal::{pfic, usart};
use crate::runtime::Drivers;
use crate::runtime::statics::{KERNEL, SESSION, SHARED};

/// Configures PFIC priorities and unmasks the transport + ADC IRQs. Called
/// once during bringup, after the drivers and statics are installed.
///
/// The transport vectors (USART1 for break/TC, SysTick for the framer
/// deadlines) share PFIC HIGH so all `&mut` access into the `ServoBus`
/// composite serializes. LOW holds the kernel (DMA1_CH1 = 22) and the two
/// dispatch-consumer wakes around it — Software (14) for live requests,
/// I2C1_EV (30) for backlog frames — same-class arbitration is
/// lowest-vector-first, which IS the interleave policy (A3(b)). DMA1_CH5
/// (RX ring) runs silent circular — no HT/TC IRQ — and CH4/CH3 raise none
/// either. Bound on the FE→DMA settle spin: the RXNE→ring DMA write is ~10
/// AHB cycles; 64 spins is orders of magnitude past it while staying sub-µs.
const FE_DMA_SETTLE_SPINS: u32 = 64;

/// RX-error flag discipline. FE/ORE/NE/PE are RO, cleared only by the
/// SR-read-then-DR-read sequence — and a CPU DR read mid-stream races the
/// RX-DMA drain: landing between a byte's completion and its ~10-cycle
/// drain steals the byte from the ring (bench 2026-07-08: burst writes
/// lost single bytes to the FE clear, structurally correlated with
/// HIGH-body FE lag; write-0 to STATR is disproven — the flags storm).
/// So the error vector never reads DR. Its entry STATR read arms the
/// sequence and the stream's own next drain completes it in hardware;
/// EIE is masked meanwhile so the still-set flag doesn't storm, and
/// [`maintain_rx_flags`] (every HIGH wake) unmasks once the flags read
/// clean — or CPU-clears in a provably quiet window: our reply streaming
/// (host contractually silent + HDSEL no self-echo), or the cursor idle
/// past [`RX_FLAG_IDLE_CLEAR_TICKS`] (no completion for over a byte-time;
/// the residual — a resuming host's first byte completing within the
/// read's ~2 cycles — merges into the dead-transmitter giveup contract).
/// Break delivery is not framing truth (A2): an FE masked through this
/// window surfaces as ring data and resolves on the fast path.
struct RxFlagPark {
    masked: bool,
    seen_cursor: u16,
    idle_since: u32,
}

static RX_FLAG_PARK: core::cell::SyncUnsafeCell<RxFlagPark> =
    core::cell::SyncUnsafeCell::new(RxFlagPark {
        masked: false,
        seen_cursor: 0,
        idle_since: 0,
    });

/// Two byte-times at the 500k rescue floor, with margin: any byte in
/// flight when the window opened has long completed (and drained) by the
/// time it fires.
const RX_FLAG_IDLE_CLEAR_TICKS: u32 = 2400;

fn rx_cursor() -> u16 {
    512 - crate::hal::dma::remaining(crate::hal::dma::Channel::CH5)
}

/// Park a still-set error flag: mask EIE (no storm) and let the stream's
/// next DMA drain complete the clear armed by the caller's STATR read.
/// HIGH-serialized with `maintain_rx_flags` (USART1 + SysTick share the
/// class).
fn park_rx_flags() {
    // SAFETY: HIGH-exclusive state, see above.
    let p = unsafe { &mut *RX_FLAG_PARK.get() };
    usart::set_eie(USART1, false);
    p.masked = true;
    p.seen_cursor = rx_cursor();
    p.idle_since = crate::hal::systick::ticks();
}

/// Unmask parked RX-error flags once the stream (or a quiet-window CPU
/// clear) has cleared them. Runs at every HIGH transport wake.
fn maintain_rx_flags() {
    // SAFETY: HIGH-exclusive state, see above.
    let p = unsafe { &mut *RX_FLAG_PARK.get() };
    if !p.masked {
        return;
    }
    // This STATR read doubles as the SR half for any flag set since the
    // last one — the next drain then clears it without CPU help.
    let errs = usart::rx_errors(USART1);
    if !(errs.fe || errs.ore || errs.pe || errs.ne) {
        usart::set_eie(USART1, true);
        p.masked = false;
        return;
    }
    let cursor = rx_cursor();
    let now = crate::hal::systick::ticks();
    if cursor != p.seen_cursor {
        // Stream alive: its next drain completes the armed clear.
        p.seen_cursor = cursor;
        p.idle_since = now;
        return;
    }
    if usart::is_tcie(USART1) || now.wrapping_sub(p.idle_since) >= RX_FLAG_IDLE_CLEAR_TICKS {
        // Provably quiet wire — the DR read cannot meet an undrained byte.
        usart::clear_rx_errors(USART1);
        usart::set_eie(USART1, true);
        p.masked = false;
    }
}

pub fn install_irqs() {
    pfic::set_priority(pfic::Interrupt::USART1, pfic::Priority::High);
    pfic::set_systick_priority(pfic::Priority::High);
    pfic::set_priority(pfic::Interrupt::DMA1_CHANNEL1, pfic::Priority::Low);
    pfic::set_software_priority(pfic::Priority::Low);
    pfic::set_priority(pfic::Interrupt::I2C1_EV, pfic::Priority::Low);
    pfic::enable(pfic::Interrupt::USART1);
    pfic::enable_systick();
    pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
    pfic::enable_software();
    pfic::enable(pfic::Interrupt::I2C1_EV);
    crate::log::info!("ISRs live");
}

/// HIGH-side dispatcher: materializes the `SESSION` borrow inside each
/// `Dispatch` method instead of holding one across the whole ISR body.
///
/// SAFETY (the SESSION exclusivity invariant): the session's other user is
/// the LOW dispatch consumer, live only while the handoff slot holds a
/// claimed job. `ServoBus` reaches these methods only on speculation paths
/// (covered checkpoint, CRC verify), and the handoff backpressure holds the
/// framer — so no speculation can begin or resolve — whenever the slot is
/// occupied. The two borrows are therefore temporally exclusive even though
/// the vectors preempt.
struct SpecDispatcher;

impl SpecDispatcher {
    #[inline(always)]
    fn with<R>(&mut self, f: impl FnOnce(&mut osc_core::Dispatcher<'_>) -> R) -> R {
        // SAFETY: see type doc — the LOW consumer holds no live borrow on
        // any path that reaches here.
        let session = unsafe { (*SESSION.get()).assume_init_mut() };
        f(&mut session.dispatcher(&SHARED))
    }
}

impl Dispatch for SpecDispatcher {
    fn dispatch<R: Reply>(&mut self, req: Request<'_>, ctx: RequestCtx, reply: &mut R) {
        self.with(|d| d.dispatch(req, ctx, reply))
    }

    fn dispatch_speculative<R: Reply>(
        &mut self,
        req: Request<'_>,
        ctx: RequestCtx,
        reply: &mut R,
    ) -> Speculated {
        self.with(|d| d.dispatch_speculative(req, ctx, reply))
    }

    fn commit_speculation<R: Reply>(&mut self, reply: &mut R) {
        self.with(|d| d.commit_speculation(reply))
    }

    fn revert_speculation(&mut self) {
        self.with(|d| d.revert_speculation())
    }
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
    crate::log::trace!("usart1 isr");
    maintain_rx_flags();
    // (a) RX errors: a break (or mid-frame garble) → the framer anchors on
    // the just-ringed 0x00 (F2: the DMA write beats the ISR). The IRQ is
    // the break signal, NOT the flags: with RX-DMA, the hardware's SR→DR
    // clear sequence can complete between the IRQ pend and this read (the
    // DMA's DR drain is the DR half), leaving STATR clean — bench: gating
    // on FE went deaf on every other exchange, entry seen with STATR=0xC0.
    // This vector has exactly two sources (EIE errors, gated TC), so any
    // non-TC entry is an RX error whether or not its flags survived.
    let errs = usart::rx_errors(USART1);
    let any_err = errs.fe || errs.ore || errs.pe || errs.ne;
    let tc = usart::is_tcie(USART1) && usart::is_tc(USART1);
    if any_err || !tc {
        // CPU DR-read steal from a hardware drain-stall overrun.
        // The ERR interrupt can beat the DMA drain: at entry the break's
        // 0x00 may still sit in DR (RXNE set), so the ring cursor hasn't
        // counted it yet — and the framer's anchor would land one byte
        // early (bench-observed: every header read as [stale, 00, ID, LEN]
        // → BadId). Wait, bounded, for the DMA to take the byte before the
        // driver samples the cursor.
        let mut settle = FE_DMA_SETTLE_SPINS;
        while usart::is_rxne(USART1) && settle > 0 {
            settle -= 1;
            core::hint::spin_loop();
        }
        if any_err {
            // NEVER clear here: the SR-then-DR sequence's DR read races the
            // RX-DMA drain, and mid-stream it steals in-flight bytes from
            // the ring (bench 2026-07-08: a burst write's LEN/CRC byte lost
            // to the FE clear, structurally aligned by HIGH-body FE lag —
            // zero counters, stale replies). Park the flag instead: the
            // entry STATR read above armed the sequence, so the stream's
            // own next drain completes the clear in hardware;
            // `maintain_rx_flags` unmasks at the next wake, or CPU-clears
            // only in a provably quiet window.
            park_rx_flags();
        }
        // A2: the break handler resolves complete frames from ring data in
        // place, so it carries the (lazy) speculation dispatcher like the
        // deadline body.
        let mut dispatcher = SpecDispatcher;
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_break(&mut dispatcher);
    }

    // (b) TC: an armed TX arm drained (shifter empty). TCIE gates arbitration
    // — the shared vector fans in RX-errors + TC, and a foreign source could
    // enter with a stale reset-value TC. Gate on TCIE so it can't walk into
    // on_tx_complete before the first reply is armed.
    //
    // TC is NOT cleared here — `TxWire::send` clears it per-arm once the
    // next arm's first byte is in flight, and the final arm's release drops
    // TCIE, leaving TC=1 as the natural idle state (STATR reset 0xC0).
    if tc {
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_tx_complete();
    }
}

/// SysTick compare — one or more framer/chain/rescue deadlines are due, a
/// `pend_systick` late-arm wake, or the consumer's reply-ready pend (the
/// adoption path, A3(b)). CNTIF is cleared first: a final deadline body
/// returns without re-arming and a stale-but-latched flag would re-fire
/// the IRQ the moment we return.
///
/// SAFETY: SysTick shares PFIC HIGH with USART1, so no concurrent `&mut`
/// into the composite is possible; SESSION access goes through the lazy
/// [`SpecDispatcher`] under its exclusivity invariant.
pub fn on_deadline_irq() {
    crate::log::trace!("deadline isr");
    crate::hal::systick::clear_match();
    maintain_rx_flags();
    let mut dispatcher = SpecDispatcher;
    // SAFETY: see fn doc.
    unsafe { Drivers::bus() }.on_deadline(&mut dispatcher);
}

/// LOW dispatch-consumer body, shared by both wake vectors (Software = live
/// lane, I2C1_EV = backlog lane): decode + dispatch the outstanding job and
/// record its reply for HIGH adoption. A spurious wake (both lanes pended,
/// or a pend racing adoption) finds no claimable job and no-ops.
///
/// SAFETY: SESSION `&mut` at LOW is exclusive with the HIGH side by the
/// [`SpecDispatcher`] invariant — while this body holds the claimed job,
/// the handoff backpressure keeps every HIGH speculation path unreachable.
/// The consumer cell itself is exclusive to these two vectors (same class,
/// no mutual preemption; the kernel never touches it).
pub fn on_dispatch_job() {
    crate::log::trace!("dispatch isr");
    // SAFETY: see fn doc — SESSION is installed before these vectors unmask.
    let session = unsafe { (*SESSION.get()).assume_init_mut() };
    let mut dispatcher = session.dispatcher(&SHARED);
    // SAFETY: see fn doc.
    unsafe { Drivers::consumer() }.process(&mut dispatcher);
}

/// Wires osc-ch32 ISR bodies into the vector table via the stock
/// `#[qingke_rt::interrupt]` trampolines (save-ra + HPE hardware stacking,
/// INTSYSCR=0x3 from qingke-rt's startup).
///
/// The "HPE corrupts t0/t2" episode is DEBUNKED (bringup `hpe_matrix`,
/// 2026-07-06): stock trampolines survived ~8M IRQ crossings across
/// single/nested/tail-chain/critical-section/100 kHz-storm legs with zero
/// corruption. The corruptor was `wlink write-mem`, which resumes the hart
/// with its scratch registers leaked into the running context — t0 = the
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

        #[::qingke_rt::interrupt(core)]
        fn Software() {
            $crate::runtime::isr::on_dispatch_job();
        }

        #[::qingke_rt::interrupt]
        fn I2C1_EV() {
            $crate::runtime::isr::on_dispatch_job();
        }
    };
}
