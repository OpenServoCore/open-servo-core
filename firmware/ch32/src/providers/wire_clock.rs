//! Wire-clock provider — supplies `WireClock` from TIM2's 16-bit free-running
//! counter. Same counter the DXL ISR layer stamps into the edge ring via
//! TIM2_CH4 IC capture; this accessor surfaces the bare CNT so the driver can
//! read `now` at poll time without the chip-side caller having to know which
//! peripheral feeds the wire clock.

use osc_drivers::traits::dxl;

use crate::hal::timer;

pub struct WireClock;

impl dxl::WireClock for WireClock {
    #[inline(always)]
    fn now(&self) -> u16 {
        timer::tim2_cnt()
    }
}
