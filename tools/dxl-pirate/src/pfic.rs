//! PFIC priority assignment. Time-critical stamping IRQs (USART1 TC, USART3
//! IDLE/RXNE) sit at preempt class 0 so an in-flight USB IRQ can't add
//! 5-20 µs of jitter to T_first / T_last on the cal path.
//!
//! qingke-rt's V4 init writes INTSYSCR=0x3 (HWSTKEN + INESTEN), so preemption
//! is supported. With default PMTCFG, IPRIOR bit 7 is the preempt-class
//! select (0 = high, 1 = low) and bits 6:4 are subpriority within the class.
//! USART3 takes subprio 0 to win simultaneous-pending races against USART1.

use ch32_hal::pac::Interrupt;
use qingke::pfic;

const PRIO_USART3: u8 = 0x00;
const PRIO_USART1: u8 = 0x10;
const PRIO_USB: u8 = 0x80;

pub fn set_priorities() {
    // SAFETY: writes one byte per IRQ to PFIC_IPRIOR; called once at boot
    // before USB enumeration and bus traffic can race.
    unsafe {
        pfic::set_priority(Interrupt::USART3 as u8, PRIO_USART3);
        pfic::set_priority(Interrupt::USART1 as u8, PRIO_USART1);
        pfic::set_priority(Interrupt::USB_LP_CAN1_RX0 as u8, PRIO_USB);
    }
}
