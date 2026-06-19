//! Wire-clock provider — surfaces `WireClock` from SysTick's 32-bit
//! free-running counter at HCLK. TIM2 (PSC=0, ARR=0xFFFF) shares the same
//! HCLK source as SysTick on V006, so the low 16 bits of `now()` equal the
//! TIM2 CCR4 IC stamps the chip-side ISR layer feeds into the edge ring at
//! the same instant (per the [`WireClock`] contract). The driver-side
//! classifier lifts u16 IC stamps to u32 using current `now()` readings;
//! no explicit lift method is exposed here.
//!
//! [`WireClock`]: osc_drivers::traits::dxl::WireClock

use osc_drivers::traits::dxl;

use crate::hal::systick;

pub struct WireClock;

impl dxl::WireClock for WireClock {
    #[inline(always)]
    fn now(&self) -> u32 {
        systick::ticks()
    }
}
