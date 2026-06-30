//! PFIC priority assignment. The three walker-driving IRQs — USART3
//! (IDLE drain), DMA1_CHANNEL6 (IC ring HT/TC; trailing high-half
//! writer), and DMA1_CHANNEL3 (RX ring HT/TC) — share `PRIO_WALKER` so
//! they cannot preempt one another mid-walk; `capture::walk()` runs
//! single-threaded across all three. USB sits at `PRIO_USB` so it
//! can't delay a wire-side stamp.
//!
//! Per TIMING.md §3.2 the walker is fully event-driven (no TIM2 cadence),
//! so TIM2's PFIC slot is unconfigured (= reset default, IRQ disabled at
//! init time too).
//!
//! qingke-rt's V4 init writes INTSYSCR=0x3 (HWSTKEN + INESTEN). With
//! default PMTCFG, IPRIOR bit 7 is the preempt-class select (0 = high,
//! 1 = low) and bits 6:4 are subpriority within the class.

use ch32_hal::pac::Interrupt;
use qingke::pfic;

const PRIO_WALKER: u8 = 0x00;
const PRIO_USB: u8 = 0x80;

pub fn set_priorities() {
    // SAFETY: writes one byte per IRQ to PFIC_IPRIOR; called once at
    // boot before USB enumeration and bus traffic can race.
    unsafe {
        pfic::set_priority(Interrupt::USART3 as u8, PRIO_WALKER);
        pfic::set_priority(Interrupt::DMA1_CHANNEL6 as u8, PRIO_WALKER);
        pfic::set_priority(Interrupt::DMA1_CHANNEL3 as u8, PRIO_WALKER);
        pfic::set_priority(Interrupt::USB_LP_CAN1_RX0 as u8, PRIO_USB);

        // qingke-rt sets WFITOWFE=1; undo it so `wfi` wakes on pending IRQs.
        pfic::wfi_to_wfe(false);
    }
}
