//! Vector bodies + IRQ bring-up. Two transport vectors, both one-line
//! dispatchers into the engine: TC latency bounds when the receiver is
//! re-enabled for the reply (the 12 us reply gap at 3M is the budget),
//! SysTick entry latency bounds CAL ruler-mark jitter. The USB stack stays
//! polled -- it has no timing coupling.

use ch32_metapac::USART3;

use crate::hal::{pfic, systick, usart};
use crate::runtime::Drivers;

/// Unmask the transport IRQs. Called once during bringup, after
/// `Drivers::install`. Both vectors keep the reset preemption class, so
/// neither preempts the other and every `&mut` into the engine serializes.
pub fn install_irqs() {
    pfic::enable(pfic::Interrupt::USART3);
    pfic::enable_systick();
}

/// USART3 vector -- TX arm drained (TC is the only enabled source; the
/// receive side never interrupts, the framer walks the ring from `poll`).
///
/// SAFETY: the bus is installed before this vector unmasks; USART3 shares
/// the preemption class with SysTick, so no concurrent `&mut` exists.
pub fn on_usart3() {
    // TCIE gates arbitration: a spurious entry with the reset-value TC
    // latched must not walk into on_tx_complete before an arm exists.
    if usart::statr(USART3).tc() && usart::is_tcie(USART3) {
        // The engine re-arms or releases inside; TCIE is dropped by the
        // provider's release, leaving TC=1 as the natural idle state.
        // SAFETY: see fn doc.
        unsafe { Drivers::bus() }.on_tx_complete();
    }
}

/// SysTick compare -- an engine deadline is due (train pacing, rescue
/// hold, pacing gap) or a `pend_systick` late-arm wake. CNTIF is cleared
/// first: a body that returns without re-arming must not re-fire on the
/// stale latch.
///
/// SAFETY: shares the preemption class with USART3; see `on_usart3`.
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
    };
}
