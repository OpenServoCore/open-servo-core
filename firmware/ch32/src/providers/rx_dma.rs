//! RX byte-ring DMA provider — binds `RxDma` to DMA1_CH5 (USART1 RX →
//! byte ring). The channel itself is configured + enabled in `runtime/init`;
//! this provider only surfaces `remaining()` (NDTR) for the Fast Last fold
//! body's intra-loop refresh per doc §6.

use osc_drivers::traits::dxl;

use crate::hal::dma;

/// Production binding to DMA1_CH5 (USART1 RX → byte ring).
pub struct RxDma;

impl dxl::RxDma for RxDma {
    #[inline(always)]
    fn remaining(&self) -> u16 {
        dma::remaining(dma::Channel::CH5)
    }
}
