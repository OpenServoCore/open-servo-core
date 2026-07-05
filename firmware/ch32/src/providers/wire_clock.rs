//! Wire-clock provider — surfaces `WireClock` from SysTick's 32-bit
//! free-running counter at HCLK.
//!
//! TIM2 (PSC=0, ARR=0xFFFF) and SysTick are both clocked from HCLK, so
//! once their CNT registers are zeroed at the same instant they stay
//! phase-aligned forever. That alignment lets the TX scheduler truncate a
//! u32 `now()`-domain deadline into a 16-bit TIM2 CCR4/CCR2 compare: the
//! low 16 bits of `now()` equal TIM2.CNT.
//!
//! The alignment is established at boot in
//! [`crate::hal::timer::init_tim2_ch4_oc_kickoff`], which calls
//! [`crate::hal::systick::reset_cnt`] in the register-write immediately
//! preceding `TIM2.CEN=true`. After that point **no production code may
//! reset CNT on either timer** — doing so silently breaks the mapping and
//! reproduces the wedge bug class. `delay_ms` reinitializes SysTick on
//! every call and is therefore restricted to pre-boot use (see the doc on
//! `runtime::init::bringup`'s SysTick init ordering).
//!
//! [`WireClock`]: osc_drivers::traits::dxl::WireClock

use osc_drivers::traits::dxl;

use crate::hal::systick;

pub struct WireClock;

impl dxl::WireClock for WireClock {
    const PACKET_END_ENTRY_COMP_TICKS: u32 = crate::measurements::PACKET_END_ENTRY_COMP_TICKS;

    #[inline(always)]
    fn now(&self) -> u32 {
        systick::ticks()
    }
}
