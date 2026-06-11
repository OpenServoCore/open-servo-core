//! USART adapters — bind `UsartBaud` to specific USART instances.

use ch32_metapac::USART1 as USART1_REGS;
use osc_drivers::traits::UsartBaud;

use crate::hal::clocks::PCLK_HZ;
use crate::hal::usart;

/// Production binding to USART1's BRR register.
pub struct Usart1;

impl UsartBaud for Usart1 {
    const CLOCK_HZ: u32 = PCLK_HZ;

    #[inline(always)]
    fn set_baud(&mut self, brr: u32) {
        usart::set_baud(USART1_REGS, brr);
    }
}
