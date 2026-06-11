//! USART-baud provider — binds `UsartBaud` to a specific USART instance.

use ch32_metapac::USART1 as USART1_REGS;
use osc_drivers::traits;

use crate::hal::clocks::PCLK_HZ;
use crate::hal::usart;

/// Production binding to USART1's BRR register.
pub struct UsartBaud;

impl traits::UsartBaud for UsartBaud {
    const CLOCK_HZ: u32 = PCLK_HZ;

    #[inline(always)]
    fn set_baud(&mut self, brr: u32) {
        usart::set_baud(USART1_REGS, brr);
    }
}
