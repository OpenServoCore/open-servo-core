use dxl_protocol::prelude::WriteBuf;

use crate::{BootMode, RxSnapshot};

pub trait DxlIo {
    type TxBuf: WriteBuf;

    fn rx_snapshot(&self) -> RxSnapshot<'_>;
    fn tx_buf(&mut self) -> &mut Self::TxBuf;
    fn start_tx(&mut self);
    fn start_tx_after(&mut self, delay_us: u32);
    fn request_reboot(&mut self, mode: BootMode);
}
