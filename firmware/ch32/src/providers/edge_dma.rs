//! Edge-DMA provider — binds `EdgeDma` to DMA1_CH7 (TIM2_CH4 input-capture
//! → ET ring destination).

use osc_drivers::traits::dxl;

use crate::hal::dma;

/// Production binding to DMA1_CH7 (TIM2_CH4 input-capture → ET ring).
pub struct EdgeDma;

impl dxl::EdgeDma for EdgeDma {
    #[inline(always)]
    fn remaining(&self) -> u16 {
        dma::remaining(dma::Channel::CH7)
    }
}
