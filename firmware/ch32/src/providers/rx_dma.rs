//! RX byte-ring DMA provider — binds `RxDma` to DMA1_CH5 (USART1 RX →
//! byte ring). The channel itself is configured + enabled in `runtime/init`;
//! this provider surfaces `remaining()` (NDTR) for the Fast Last fold body's
//! intra-loop refresh per doc §6 and `read_and_ack()` for the byte-ring
//! publish ISR (doc §9).

use ch32_metapac::USART1;
use osc_drivers::traits::dxl::{self, DmaFlags};

use crate::hal::{dma, usart};

/// Production binding to DMA1_CH5 (USART1 RX → byte ring).
pub struct RxDma;

impl dxl::RxDma for RxDma {
    #[inline(always)]
    fn remaining(&self) -> u16 {
        dma::remaining(dma::Channel::CH5)
    }

    #[inline(always)]
    fn read_and_ack(&mut self) -> DmaFlags {
        let ht = dma::is_ht_flag(dma::Channel::CH5);
        let tc = dma::is_tc_flag(dma::Channel::CH5);
        if ht {
            dma::clear_ht_flag(dma::Channel::CH5);
        }
        if tc {
            dma::clear_tc_flag(dma::Channel::CH5);
        }
        DmaFlags { ht, tc }
    }

    /// USART1 RXNEIE on. In DMA-RX mode `STATR.RXNE` always reads 0 (DMA
    /// wins the clear race), so the ISR can't gate on the flag — the
    /// vector treats every non-IDLE entry inside this window as a byte
    /// wake and the driver cursor-qualifies it. The window also admits
    /// spurious entries from the DMA-RDR drain race; same driver-side
    /// qualification absorbs them.
    #[inline(always)]
    fn watch_status_start(&mut self) {
        usart::set_rxne_irq(USART1, true);
    }

    #[inline(always)]
    fn unwatch_status_start(&mut self) {
        usart::set_rxne_irq(USART1, false);
    }
}
