//! USART-baud provider — binds `UsartBaud` to USART1's BRR register. Owns
//! the BaudRate → BRR map; the driver hands a `BaudRate` and stays unaware
//! of the divisor math.

use ch32_metapac::USART1 as USART1_REGS;
use osc_core::BaudRate;
use osc_drivers::traits::dxl;

use crate::hal::clocks::PCLK_HZ;
use crate::hal::usart;

/// Production binding to USART1's BRR register.
pub struct UsartBaud;

impl dxl::UsartBaud for UsartBaud {
    const CLOCK_HZ: u32 = PCLK_HZ;

    #[inline(always)]
    fn apply_baud(&mut self, baud: BaudRate) {
        usart::set_baud(USART1_REGS, brr_for(baud));
    }
}

/// Round-to-nearest USART_BRR divisor at PCLK for the chip's six supported
/// DXL bauds. Each arm folds to a literal — RV32EC has no hardware divide.
/// Source of truth for both runtime `apply_baud` and the const-fn
/// precompute path in `cfg::Precomputed::compute`.
pub const fn brr_for(baud: BaudRate) -> u32 {
    const fn compute(baud_hz: u32) -> u32 {
        (PCLK_HZ + baud_hz / 2) / baud_hz
    }
    match baud {
        BaudRate::B9600 => const { compute(BaudRate::B9600.as_hz()) },
        BaudRate::B57600 => const { compute(BaudRate::B57600.as_hz()) },
        BaudRate::B115200 => const { compute(BaudRate::B115200.as_hz()) },
        BaudRate::B1000000 => const { compute(BaudRate::B1000000.as_hz()) },
        BaudRate::B2000000 => const { compute(BaudRate::B2000000.as_hz()) },
        BaudRate::B3000000 => const { compute(BaudRate::B3000000.as_hz()) },
    }
}
