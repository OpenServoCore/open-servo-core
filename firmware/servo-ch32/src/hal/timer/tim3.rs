//! TIM3 free-running on the timer clock: the counter the break stamps latch
//! (transport sec 8). The streamlined block (RM ch. 13) has no prescaler
//! and no interrupt, which is all a DMA stamp source needs; ATRLR resets to
//! 0xFFFF, so the count wraps every 65536 ticks.

use ch32_metapac::TIM3;

/// The counter's address, the source of the break-stamp DMA.
#[inline]
pub fn counter_addr() -> u32 {
    TIM3.cnt().as_ptr() as u32
}

pub fn start() {
    TIM3.ctlr().write(|w| w.set_cen(true));
}
