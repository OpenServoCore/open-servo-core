//! DMA adapters — bind `DmaRing` to specific DMA channels.

use crate::drivers::traits::{DmaFlags, DmaRing};
use crate::hal::dma;

/// Production binding to DMA1_CH7 (TIM2_CH4 input-capture → ET ring).
pub struct Ch7;

impl DmaRing for Ch7 {
    #[inline(always)]
    fn read_and_ack(&mut self) -> DmaFlags {
        let ht = dma::is_ht_flag(dma::Channel::CH7);
        let tc = dma::is_tc_flag(dma::Channel::CH7);
        if ht {
            dma::clear_ht_flag(dma::Channel::CH7);
        }
        if tc {
            dma::clear_tc_flag(dma::Channel::CH7);
        }
        DmaFlags { ht, tc }
    }

    #[inline(always)]
    fn remaining(&self) -> u16 {
        dma::remaining(dma::Channel::CH7)
    }
}
