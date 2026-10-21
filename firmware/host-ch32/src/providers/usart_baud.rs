//! USART-baud provider -- binds `UsartBaud` to USART3's BRR. Owns the
//! `BaudRate` -> BRR map; every divisor is integer and crystal-exact at
//! 144 MHz, and all sit above the V305's USARTDIV >= 1 floor (BRR >= 16).

use ch32_metapac::USART3;
use osc_host::link::record::TB_RATES;
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
        usart::set_baud(USART3, raw_brr(bps));
    }
}

pub const fn brr_for(baud: BaudRate) -> u32 {
    PCLK1_HZ / baud.as_hz()
}

/// Rounded divisor, floored at the USARTDIV >= 1 silicon minimum (BRR >= 16
/// at 16x oversampling).
pub const fn raw_brr(bps: u32) -> u32 {
    let brr = (PCLK1_HZ + bps / 2) / bps;
    if brr < 16 { 16 } else { brr }
}

// Every tinyboot rate within 2% (link sec 5): a clock change that breaks one
// fails the build.
const _: () = {
    let mut i = 0;
    while i < TB_RATES.len() {
        let bps = TB_RATES[i];
        let err = (PCLK1_HZ / raw_brr(bps)).abs_diff(bps) as u64;
        assert!(
            err * 50 <= bps as u64,
            "a tinyboot rate is off by more than 2%"
        );
        i += 1;
    }
};
