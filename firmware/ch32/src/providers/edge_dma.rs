//! Edge-DMA provider — binds `EdgeDma` to DMA1_CH7 (TIM2_CH4 input-capture
//! → ET ring destination).

use osc_drivers::traits::dxl::{self, DmaFlags};

use crate::hal::dma;

/// Production binding to DMA1_CH7 (TIM2_CH4 input-capture → ET ring).
pub struct EdgeDma;

impl dxl::EdgeDma for EdgeDma {
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

    #[inline(always)]
    fn pause(&mut self) {
        dma::set_htie(dma::Channel::CH7, false);
        dma::set_tcie(dma::Channel::CH7, false);
        dma::clear_ht_flag(dma::Channel::CH7);
        dma::clear_tc_flag(dma::Channel::CH7);
    }

    #[inline(always)]
    fn resume(&mut self) {
        dma::clear_ht_flag(dma::Channel::CH7);
        dma::clear_tc_flag(dma::Channel::CH7);
        dma::set_htie(dma::Channel::CH7, true);
        dma::set_tcie(dma::Channel::CH7, true);
    }
}
