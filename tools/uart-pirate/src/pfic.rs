//! PFIC priority assignment. USART3 is the one wire-side vector (TC
//! release, break-boundary capture, IDLE bookkeeping); it sits at
//! `PRIO_WIRE` so a boundary stamp is never delayed by USB service.
//! USB sits in the low preempt class below it.
//!
//! qingke-rt's V4 init writes INTSYSCR=0x3 (HWSTKEN + INESTEN). With
//! default PMTCFG, IPRIOR bit 7 is the preempt-class select (0 = high,
//! 1 = low) and bits 6:4 are subpriority within the class.

use ch32_metapac::Interrupt;
use qingke::pfic;

const PRIO_WIRE: u8 = 0x00;
const PRIO_USB: u8 = 0x80;

pub fn set_priorities() {
    // SAFETY: writes one byte per IRQ to PFIC_IPRIOR; called once at
    // boot before USB enumeration and bus traffic can race.
    unsafe {
        pfic::set_priority(Interrupt::USART3 as u8, PRIO_WIRE);
        pfic::set_priority(Interrupt::USB_LP_CAN1_RX0 as u8, PRIO_USB);

        // qingke-rt sets WFITOWFE=1; undo it so `wfi` wakes on pending IRQs.
        pfic::wfi_to_wfe(false);
    }
}
