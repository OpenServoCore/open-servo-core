//! TX scheduler — TIM2_CH3 OC fires the wire-driver activate IRQ, TIM2_CH2 OC
//! drives PC2 (TX_EN) in hardware. Per `docs/dxl-hw-timed-transport.md` §5:
//! the driver hands a `deadline_tick` already in TIM2 ticks; CCR3 back-dates
//! by `TX_START_ENTRY_TICKS` to absorb the ISR+DMA+USART path so the wire
//! bit lands at or after deadline. CCR2 = deadline_tick verbatim — TX_EN
//! rises ~1-2 ticks late by OC latency, biasing toward "late not early" so
//! we never encroach on the previous slot's RDT.

use ch32_metapac::USART1 as USART1_REGS;
use osc_drivers::traits::{DxlTxScheduler as DxlTxSchedulerTrait, SendKind};

use crate::hal::clocks::HCLK_HZ;
use crate::hal::{dma, timer, usart};
use crate::measurements::{SCHEDULE_WRAP_GUARD_TICKS, TX_START_ENTRY_TICKS};

pub struct DxlTxScheduler;

impl DxlTxSchedulerTrait for DxlTxScheduler {
    const TICKS_PER_US: u16 = (HCLK_HZ / 1_000_000) as u16;

    fn schedule(&mut self, deadline_tick: u16, byte_count: u16, _kind: SendKind) {
        let ccr3 = deadline_tick.wrapping_sub(TX_START_ENTRY_TICKS);
        let ccr2 = deadline_tick;

        // DMA channel must be disabled before NDTR is written.
        dma::disable(dma::Channel::CH4);
        dma::set_count(dma::Channel::CH4, byte_count);
        usart::clear_tc(USART1_REGS);
        usart::set_dma_tx(USART1_REGS, true);
        usart::set_tc_irq(USART1_REGS, true);

        timer::set_tim2_ccr3(ccr3);
        timer::set_tim2_ccr2(ccr2);
        timer::tim2_ch2_active_on_match();
        timer::clear_tim2_cc3_flag();
        timer::enable_tim2_cc3_irq(true);

        // §5.4 set-and-recheck: if CNT just passed CCR3, the next CC3IF is a
        // full wrap (~1.365 ms) away. Detect modular underflow → start-now.
        let cnt = timer::tim2_cnt();
        let remaining = ccr3.wrapping_sub(cnt);
        if remaining > SCHEDULE_WRAP_GUARD_TICKS {
            start_now();
        }
    }

    fn cancel(&mut self) {
        timer::enable_tim2_cc3_irq(false);
        timer::tim2_ch2_force_inactive();
        usart::set_tc_irq(USART1_REGS, false);
        usart::set_dma_tx(USART1_REGS, false);
        dma::disable(dma::Channel::CH4);
        timer::clear_tim2_cc3_flag();
    }

    fn handle_start(&mut self) {
        // DMA enable first — every tick here delays the wire bit. CC3IE
        // mask follows: stale CC3IF stays set, the next `schedule` clears it
        // before re-enabling CC3IE.
        dma::enable(dma::Channel::CH4);
        timer::enable_tim2_cc3_irq(false);
    }

    fn handle_tx_complete(&mut self) {
        timer::tim2_ch2_force_inactive();
        usart::set_tc_irq(USART1_REGS, false);
        usart::set_dma_tx(USART1_REGS, false);
        dma::disable(dma::Channel::CH4);
    }
}

/// §5.4 start-now: CCR3 wrapped past CNT, so the CC3 IRQ is ~1.365 ms away.
/// Drive PC2 high immediately and kick DMA from here. The CC3 IRQ is masked
/// so the late wrap-around match doesn't double-fire.
#[inline]
fn start_now() {
    timer::tim2_ch2_force_active();
    timer::enable_tim2_cc3_irq(false);
    dma::enable(dma::Channel::CH4);
    timer::clear_tim2_cc3_flag();
}
