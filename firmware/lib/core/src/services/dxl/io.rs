use dxl_protocol::prelude::WriteBuf;

use crate::{BootMode, RxSnapshot};

pub trait DxlIo {
    type TxBuf: WriteBuf;

    fn rx_snapshot(&self) -> RxSnapshot<'_>;
    fn tx_buf(&mut self) -> &mut Self::TxBuf;
    fn start_tx(&mut self);
    fn start_tx_after(&mut self, idle_tick: u32, delay_us: u32);
    /// SysTick value at the trailing IDLE of the packet whose final byte sits
    /// at cumulative byte offset `parsed_end`. Returns `None` when no stamp
    /// matches — the packet is still mid-burst, or its stamp was evicted by a
    /// later IDLE under producer drop-oldest. Callers must skip slot-timed
    /// replies when this returns `None`.
    fn idle_for(&self, parsed_end: u32) -> Option<u32>;
    /// Last-slot fire path for Fast Sync/Bulk Read: snoop CRC from
    /// `frame_end..`, fire TX at `idle_tick + fire_us`, patch the CRC
    /// placeholder. `tx_buf` must already hold payload + 2-byte placeholder
    /// from `FastSlot::Last` / `Only`. `switch_us == fire_us` signals `Only`
    /// (no preceding slaves to snoop).
    fn start_fast_tx_after(&mut self, idle_tick: u32, switch_us: u32, fire_us: u32, frame_end: u32);
    fn request_reboot(&mut self, mode: BootMode);
}
