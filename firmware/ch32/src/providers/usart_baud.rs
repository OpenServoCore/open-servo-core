//! USART-baud provider (transport sec 2 / protocol sec 9.1) -- binds `UsartBaud` to USART1's
//! BRR register. Owns the `BaudRate` -> BRR map; the driver hands a rate and
//! stays unaware of the divisor.

use ch32_metapac::USART1;
use osc_core::BaudRate;
use osc_drivers::traits::bus;

use crate::hal::clocks::PCLK_HZ;
use crate::hal::usart;

/// Production binding to USART1's BRR register.
pub struct UsartBaud;

impl bus::UsartBaud for UsartBaud {
    #[inline(always)]
    fn apply(&mut self, baud: BaudRate) {
        usart::set_baud(USART1, brr_for(baud));
    }
}

/// Round-to-nearest BRR divisor at PCLK for the four operational rates. Each
/// arm folds to a literal via `const {}` — the board build targets +zmmul
/// (no hardware divide), so no runtime division is emitted. Source of truth
/// for both runtime `apply` and the `cfg::Precomputed` seed.
pub const fn brr_for(baud: BaudRate) -> u32 {
    const fn compute(baud_hz: u32) -> u32 {
        (PCLK_HZ + baud_hz / 2) / baud_hz
    }
    match baud {
        BaudRate::B500000 => const { compute(500_000) },
        BaudRate::B1000000 => const { compute(1_000_000) },
        BaudRate::B2000000 => const { compute(2_000_000) },
        BaudRate::B3000000 => const { compute(3_000_000) },
    }
}
