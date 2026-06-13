//! USART-baud provider — binds `UsartBaud` to USART1 and the TIM2_CH4
//! input-capture filter that tracks it. Owns the BaudRate → BRR map and
//! the BaudRate → IC4F map; the driver hands a `BaudRate` and stays
//! unaware of either.

use ch32_metapac::USART1 as USART1_REGS;
use ch32_metapac::timer::vals::FilterValue;
use osc_core::BaudRate;
use osc_drivers::traits;

use crate::hal::clocks::PCLK_HZ;
use crate::hal::{timer, usart};

/// Production binding to USART1's BRR register and TIM2_CH4's IC4F field.
pub struct UsartBaud;

impl traits::UsartBaud for UsartBaud {
    const CLOCK_HZ: u32 = PCLK_HZ;

    #[inline(always)]
    fn apply_baud(&mut self, baud: BaudRate) {
        usart::set_baud(USART1_REGS, brr_for(baud));
        timer::set_tim2_ch4_icf(filter_for(baud));
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

/// TIM2_CH4 input-capture filter picker. Rule
/// (`docs/dxl-hw-timed-transport.md` §8.1): pick the largest min-pulse
/// strictly under the baud's bit-time at `fDTS = HCLK = 48 MHz` (CKD=0,
/// pinned by `hal::timer::init_tim2_ch4_ic_capture`). Maximum glitch
/// immunity inside that bound. Exhaustive match — adding a `BaudRate`
/// variant in osc-core forces a filter pick here. Source of truth for
/// both runtime `apply_baud` and bringup's `init_tim2_ch4_ic_capture`.
pub const fn filter_for(baud: BaudRate) -> FilterValue {
    match baud {
        // 250 ns min pulse < 333 ns bit time.
        BaudRate::B3000000 => FilterValue::FDTS_DIV2_N6,
        // 333 ns min pulse < 500 ns bit time.
        BaudRate::B2000000 => FilterValue::FDTS_DIV2_N8,
        // 667 ns min pulse < 1000 ns bit time.
        BaudRate::B1000000 => FilterValue::FDTS_DIV4_N8,
        // 5333 ns min pulse — covers every sub-1M baud (bit times start at
        // 8681 ns for B115200 and grow from there).
        BaudRate::B115200 | BaudRate::B57600 | BaudRate::B9600 => FilterValue::FDTS_DIV32_N8,
    }
}
