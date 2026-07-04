//! TX bus driver — chip-side control of the half-duplex DXL bus during
//! transmission. Owns the wire-driver state machine: TX_EN gating (via
//! TIM2_CH2 OC on PC2), TX DMA enable/disable on DMA1_CH4, and the
//! sequence-driven `start_now` path used by the Plain Sync / Bulk Read
//! chain reply at slot k > 0 per `docs/dxl-streaming-rx.md` §5.2.
//!
//! Sibling of [`crate::providers::dxl_tx_scheduler::DxlTxScheduler`]; the
//! scheduler decides *when* the wire bit fires (the hardware kickoff owns
//! the deadline path with no driver involvement), and this provider is
//! what `poll`-SkipComplete / `on_tx_complete` drive to actually move
//! bytes.

use ch32_metapac::USART1 as USART1_REGS;
use osc_drivers::traits::dxl::TxBus as TxBusTrait;

use crate::hal::{dma, timer, usart};

pub struct DxlTxBus;

impl TxBusTrait for DxlTxBus {
    fn start_now(&mut self, byte_count: u16) {
        // Same DMA + USART prelude as the scheduler's arm — the chain
        // k > 0 path skips the scheduler entirely, so the peripheral
        // setup that schedule normally stages has to land here too.
        dma::disable(dma::Channel::CH4);
        dma::set_count(dma::Channel::CH4, byte_count);
        usart::clear_tc(USART1_REGS);
        usart::set_dma_tx(USART1_REGS, true);
        usart::set_tc_irq(USART1_REGS, true);

        // No compare involved: drive TX_EN active inline and kick DMA.
        timer::tim2_ch2_force_active();
        dma::enable(dma::Channel::CH4);
    }

    fn release_bus(&mut self) {
        timer::tim2_ch2_force_inactive();
        usart::set_tc_irq(USART1_REGS, false);
        usart::set_dma_tx(USART1_REGS, false);
        dma::disable(dma::Channel::CH4);
    }
}
