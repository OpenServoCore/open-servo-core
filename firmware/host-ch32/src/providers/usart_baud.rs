//! USART-baud provider -- binds `UsartBaud` to USART3's BRR. Owns the
//! `BaudRate` -> BRR map; every divisor is integer and crystal-exact at
//! 144 MHz, and all sit above the V305's USARTDIV >= 1 floor (BRR >= 16).

use ch32_metapac::USART3;
use osc_protocol::wire::BaudRate;

use crate::hal::rcc::PCLK1_HZ;
use crate::hal::usart;

/// Production binding to USART3's BRR register.
pub struct UsartBaud;

impl osc_host::traits::UsartBaud for UsartBaud {
    #[inline]
    fn apply(&mut self, baud: BaudRate) {
        usart::set_baud(USART3, brr_for(baud));
    }

    fn apply_raw(&mut self, bps: u32) {
        // Rounded divisor, floored at the USARTDIV >= 1 silicon minimum
        // (BRR >= 16 at 16x oversampling).
        usart::set_baud(USART3, ((PCLK1_HZ + bps / 2) / bps).max(16));
    }
}

pub const fn brr_for(baud: BaudRate) -> u32 {
    PCLK1_HZ / baud.as_hz()
}
