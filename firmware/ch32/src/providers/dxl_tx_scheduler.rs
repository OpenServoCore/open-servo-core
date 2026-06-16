//! TX scheduler — TIM2_CH3 OC fires the wire-driver activate IRQ, TIM2_CH2 OC
//! drives PC2 (TX_EN) in hardware. Per `docs/dxl-hw-timed-transport.md` §5:
//! the driver hands a `deadline_tick` already in TIM2 ticks; CCR3 back-dates
//! by `TX_START_ENTRY_TICKS` to absorb the ISR+DMA+USART path so the wire
//! bit lands at or after deadline. CCR2 = deadline_tick verbatim — TX_EN
//! rises ~1-2 ticks late by OC latency, biasing toward "late not early" so
//! we never encroach on the previous slot's RDT.
//!
//! Wire-driver state (activate at CC3, release at TC, sequence-driven
//! `start_now` for Plain chain k > 0) lives in the sibling
//! [`crate::providers::dxl_tx_bus::DxlTxBus`] per the `TxBus` trait split.

use ch32_metapac::USART1 as USART1_REGS;
use osc_drivers::traits::dxl::{SendKind, TxScheduler as TxSchedulerTrait};

use crate::hal::clocks::HCLK_HZ;
use crate::hal::{dma, timer, usart};
use crate::measurements::{SCHEDULE_WRAP_GUARD_TICKS, TX_START_ENTRY_TICKS};

pub struct DxlTxScheduler;

impl TxSchedulerTrait for DxlTxScheduler {
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
        // full wrap (~1.365 ms) away. Detect modular underflow → start-now:
        // force TX_EN active, mask CC3 IRQ so the late wrap-around match
        // doesn't double-fire, kick DMA, clear the stale CC3 flag.
        let cnt = timer::tim2_cnt();
        let remaining = ccr3.wrapping_sub(cnt);
        if remaining > SCHEDULE_WRAP_GUARD_TICKS {
            timer::tim2_ch2_force_active();
            timer::enable_tim2_cc3_irq(false);
            dma::enable(dma::Channel::CH4);
            timer::clear_tim2_cc3_flag();
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
}
