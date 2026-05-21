use core::sync::atomic::Ordering;

use ch32_metapac::USART1;
use heapless::Vec;
use osc_core::{DxlIo, RxSnapshot};

use crate::hal::{dma, gpio, usart};
use crate::statics::{DXL_RX_BUF, DXL_RX_WRITE_POS, DXL_TX_BUF, DXL_TX_BUF_LEN, DXL_TX_EN};

pub struct Ch32DxlIo;

impl Ch32DxlIo {
    pub const fn new() -> Self {
        Self
    }
}

impl Default for Ch32DxlIo {
    fn default() -> Self {
        Self::new()
    }
}

impl DxlIo for Ch32DxlIo {
    type TxBuf = Vec<u8, DXL_TX_BUF_LEN>;

    fn rx_snapshot(&self) -> RxSnapshot<'_> {
        let write_pos = DXL_RX_WRITE_POS.load(Ordering::Acquire);
        // SAFETY: DMA writes DXL_RX_BUF circularly; we only read indices
        // below the IDLE-published write_pos.
        let ring = unsafe { &*DXL_RX_BUF.get() };
        RxSnapshot::new(ring, write_pos)
    }

    fn tx_buf(&mut self) -> &mut Self::TxBuf {
        // SAFETY: &mut self proves sole-writer; USART1 TC ISR only clears
        // after a start_tx cycle this struct initiated.
        unsafe { &mut *DXL_TX_BUF.get() }
    }

    fn start_tx(&mut self) {
        let len = self.tx_buf().len();
        if len == 0 {
            return;
        }
        if let Some(t) = unsafe { *DXL_TX_EN.get() } {
            gpio::set_level(t.pin, t.tx_level);
        }
        dma::set_count(dma::Channel::CH4, len as u16);
        usart::clear_tc(USART1);
        usart::set_dma_tx(USART1, true);
        usart::set_tc_irq(USART1, true);
        dma::enable(dma::Channel::CH4);
    }
}
