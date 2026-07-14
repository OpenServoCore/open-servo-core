//! TIM2 wire-edge capture -- the instrument's stopwatch (linke-edgecap
//! spike, silicon-proven). Partial remap 2 routes CH3 onto PB10 (the bus
//! pin, input path live under USART3 AF); IC3 takes TI3 falling, IC4
//! cross-selects TI3 rising; each capture DMA-drains to its ring with
//! zero CPU. Armed once at bringup and never re-armed: re-arming mangles
//! captures (spike), and the boundary quirks that remain (break-edge
//! phantoms, the swallowed break fall) are the PC decoder's contract.
//!
//! Tick domain: PSC divides PCLK1 144 MHz to 18 MHz = the SysTick domain,
//! and CNT is aligned to the SysTick count at init -- both count HCLK/8,
//! so the offset stays constant forever and u16 captures unwrap directly
//! against engine ticks.

use ch32_metapac::timer::vals::CcmrInputCcs;
use ch32_metapac::{AFIO, RCC, TIM2};

use super::systick;

/// PCLK1 144 MHz -> the 18 MHz SysTick tick domain.
const PSC_TO_18MHZ: u16 = 7;

pub fn init() {
    RCC.apb1pcenr().modify(|w| w.set_tim2en(true));
    RCC.apb2pcenr().modify(|w| w.set_afioen(true));

    // Partial remap 2: TIM2 CH3/CH4 onto PB10/PB11 (only TI3/PB10 used).
    // The RMW writes PCFR1's write-only SWCFG bits as 000 = the full-debug
    // reset default; SWD stays alive (PIN RULES).
    AFIO.pcfr1().modify(|w| w.set_tim2_rm(0b10));

    TIM2.psc().write_value(PSC_TO_18MHZ);
    TIM2.atrlr().write_value(0xFFFF);
    TIM2.swevgr().write(|w| w.set_ug(true)); // latch PSC now

    TIM2.chctlr_input(1).modify(|w| {
        // metapac value naming: 01 = own TI (IC3 <- TI3), 10 = the 3<->4
        // swap (IC4 <- TI3). No filter, no capture prescaler: every edge.
        w.set_ccs(0, CcmrInputCcs::TI4);
        w.set_ccs(1, CcmrInputCcs::TI3);
    });
    TIM2.ccer().modify(|w| {
        w.set_cce(2, true);
        w.set_ccp(2, true); // IC3: falling
        w.set_cce(3, true);
        w.set_ccp(3, false); // IC4: rising
    });
    TIM2.dmaintenr().modify(|w| {
        w.set_ccde(2, true);
        w.set_ccde(3, true);
    });

    // Align CNT to the SysTick domain just before counting starts: a few
    // ticks of enable latency shift ALL captures by one constant, which
    // delta measurements cancel and unwrap tolerance absorbs.
    TIM2.cnt().write_value(systick::ticks() as u16);
    TIM2.ctlr1().modify(|w| w.set_cen(true));
}

/// CHCVR3 address (falling captures) for the DMA1_CH1 arm.
pub fn fall_capture_addr() -> u32 {
    TIM2.chcvr(2).as_ptr() as u32
}

/// CHCVR4 address (rising captures) for the DMA1_CH7 arm.
pub fn rise_capture_addr() -> u32 {
    TIM2.chcvr(3).as_ptr() as u32
}
