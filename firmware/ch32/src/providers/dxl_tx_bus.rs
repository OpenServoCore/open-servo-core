//! TX bus driver — chip-side control of the half-duplex DXL bus during
//! transmission. Owns the wire-driver state machine: TX_EN gating (via
//! TIM2_CH2 OC on PC2), TX DMA enable/disable on DMA1_CH4, and the
//! sequence-driven `start_now` path used by the Plain Sync / Bulk Read
//! chain reply at slot k > 0 per `docs/dxl-streaming-rx.md` §5.2.
//!
//! Sibling of [`crate::providers::dxl_tx_scheduler::DxlTxScheduler`]; the
//! scheduler decides *when* the wire bit fires (or arms the wrap-fallback
//! path), and this provider is what `DxlUart::on_tx_start` /
//! `on_tx_complete` / `poll`-SkipComplete drive to actually move bytes.

use ch32_metapac::USART1 as USART1_REGS;
use osc_drivers::traits::dxl::TxBus as TxBusTrait;

use crate::hal::{dma, timer, usart};

pub struct DxlTxBus;

impl TxBusTrait for DxlTxBus {
    fn start_now(&mut self, byte_count: u16) {
        // Same DMA + USART prelude as `DxlTxScheduler::schedule` — the
        // chain k > 0 path skips the scheduler entirely, so the
        // peripheral setup that schedule normally stages has to land
        // here too.
        dma::disable(dma::Channel::CH4);
        dma::set_count(dma::Channel::CH4, byte_count);
        usart::clear_tc(USART1_REGS);
        usart::set_dma_tx(USART1_REGS, true);
        usart::set_tc_irq(USART1_REGS, true);

        // Bypass CCR3: drive TX_EN active inline, mask CC3 IRQ so any
        // stale arm doesn't double-fire, kick DMA, clear the CC3 flag.
        timer::tim2_ch2_force_active();
        timer::enable_tim2_cc3_irq(false);
        dma::enable(dma::Channel::CH4);
        timer::clear_tim2_cc3_flag();
    }

    fn handle_start(&mut self) {
        // DMA enable first — every tick here delays the wire bit. CC3IE
        // mask follows: stale CC3IF stays set, the next `schedule`
        // clears it before re-enabling CC3IE.
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
