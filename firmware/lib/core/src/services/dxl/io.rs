use dxl_protocol::prelude::WriteBuf;

use crate::{BootMode, RxSnapshot};

/// Inter-slave CRC snoop window for Fast Sync/Bulk Read `Last` slots.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SnoopWindow {
    /// Offset from `idle_tick` at which the snoop opens (slot N-2 start).
    pub open_us: u32,
    /// Cumulative RX-ring byte offset where the snoop CRC begins.
    pub rx_start: u32,
}

pub trait DxlIo {
    type TxBuf: WriteBuf;

    fn rx_snapshot(&self) -> RxSnapshot<'_>;
    fn tx_buf(&mut self) -> &mut Self::TxBuf;
    fn start_tx(&mut self);
    fn start_tx_after(&mut self, idle_tick: u32, delay_us: u32);
    /// SysTick value at the trailing IDLE of the packet whose final byte sits
    /// at cumulative byte offset `parsed_end`. `None` when no stamp matches —
    /// packet still mid-burst or stamp evicted by a later IDLE. Callers must
    /// skip slot-timed replies when this returns `None`.
    fn idle_for(&self, parsed_end: u32) -> Option<u32>;
    /// Fire path for Fast Sync/Bulk Read. `tx_buf` must hold payload (and,
    /// when `snoop` is `Some`, a trailing 2-byte CRC placeholder). `None` =
    /// `Only` slot: no predecessors to snoop, fire directly.
    fn start_fast_tx_after(&mut self, idle_tick: u32, fire_us: u32, snoop: Option<SnoopWindow>);
    fn request_reboot(&mut self, mode: BootMode);
}
