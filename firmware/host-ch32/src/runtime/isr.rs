//! Vector bodies + IRQ bring-up. Two priority levels: the USART3 vector
//! alone at the top, where it marks breaks within a byte-time of their end
//! (the mark is the DMA count at entry, so lateness would misplace it) and
//! hands TX completion down; SysTick and the core software interrupt share
//! the engine level, never preempt each other, and are the only vectors
//! that touch the engine. The main loop holds the engine level off by PFIC
//! threshold (`Drivers::with_bus`), never by masking USART3. TC latency
//! bounds when the receiver is re-enabled for the reply (the 12 us reply
//! gap at 3M is the budget), SysTick entry latency bounds CAL ruler-mark
//! jitter. The USB stack stays polled - it has no timing coupling.

use ch32_metapac::USART3;

use crate::hal::{pfic, systick, usart};
use crate::providers::ring::RxRing;
use crate::runtime::Drivers;

/// Break marking preempts everything else.
const BREAK_LEVEL: u8 = 0x00;
/// The engine's level: the IPRIOR preemption bit set.
pub const ENGINE_LEVEL: u8 = 0x80;

/// Unmask the transport IRQs. Called once during bringup, after
/// `Drivers::install`.
pub fn install_irqs() {
    pfic::enable_preemption();
    pfic::set_priority(pfic::Interrupt::USART3 as usize, BREAK_LEVEL);
    pfic::set_priority(pfic::SYSTICK_IRQ, ENGINE_LEVEL);
    pfic::set_priority(pfic::SOFTWARE_IRQ, ENGINE_LEVEL);
    pfic::enable(pfic::Interrupt::USART3);
    pfic::enable_core(pfic::SYSTICK_IRQ);
    pfic::enable_core(pfic::SOFTWARE_IRQ);
}

/// USART3 vector - a break detected, or the TX arm drained. Never touches
/// the engine: it preempts engine bodies.
pub fn on_usart3() {
    let s = usart::statr(USART3);
    if s.lbd() {
        RxRing::mark_break();
        usart::clear_lbd(USART3);
    }
    // TCIE gates arbitration: TC idles latched, so a break entry (or a
    // spurious one) must not complete an arm that does not exist. TCIE
    // drops here so the latched TC cannot re-enter before the engine runs.
    if s.tc() && usart::is_tcie(USART3) {
        usart::set_tc_irq(USART3, false);
        pfic::pend_software();
    }
}

/// Core software interrupt - the TX arm drained (pended by `on_usart3`).
///
/// SAFETY: the bus is installed before this vector unmasks; it shares the
/// engine level with SysTick, so no concurrent `&mut` exists.
pub fn on_software() {
    // The engine re-arms (TCIE back on) or releases inside.
    // SAFETY: see fn doc.
    unsafe { Drivers::bus() }.on_tx_complete();
}

/// SysTick compare -- an engine deadline is due (train pacing, rescue
/// hold, pacing gap) or a `pend_systick` late-arm wake. CNTIF is cleared
/// first: a body that returns without re-arming must not re-fire on the
/// stale latch.
///
/// SAFETY: shares the engine level with the software interrupt; see
/// `on_software`.
pub fn on_systick() {
    systick::clear_match();
    // SAFETY: see fn doc.
    unsafe { Drivers::bus() }.on_deadline();
}

/// Wires the osc-host-ch32 ISR bodies into the vector table via the stock
/// `#[qingke_rt::interrupt]` trampolines.
#[macro_export]
macro_rules! install_isrs {
    () => {
        #[::qingke_rt::interrupt]
        fn USART3() {
            $crate::runtime::isr::on_usart3();
        }

        #[::qingke_rt::interrupt(core)]
        fn SysTick() {
            $crate::runtime::isr::on_systick();
        }

        #[::qingke_rt::interrupt(core)]
        fn Software() {
            $crate::runtime::isr::on_software();
        }
    };
}
