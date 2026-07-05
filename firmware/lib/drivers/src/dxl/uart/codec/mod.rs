//! Bytes ↔ packets. Composite over two field-projection halves — `CodecRx`
//! (flat frame classifier + RX byte ring + drift bookkeeping) and `CodecTx`
//! (encoder + TX byte ring) — joined under `Codec`. The split lets the
//! parent driver hand a `&mut CodecTx` reply handle to the dispatcher while
//! a frame-verdict borrow lives in `CodecRx`.

mod codec_rx;
mod codec_tx;
mod framer;
mod packet_end;
mod poll_event;
mod skip;
mod span;

pub use codec_rx::CodecRx;
pub use codec_tx::CodecTx;
pub(crate) use framer::HELD_FRAME_MAX;
pub use poll_event::{FrameAction, FrameVerdict, PacketEnd};

use dxl_protocol::CrcUmts;

pub struct Codec<CRC: CrcUmts, const RX_BUF_LEN: usize, const TX_BUF_LEN: usize> {
    pub(super) rx: CodecRx<CRC, RX_BUF_LEN>,
    pub(super) tx: CodecTx<CRC, TX_BUF_LEN>,
}

impl<CRC: CrcUmts, const RX_BUF_LEN: usize, const TX_BUF_LEN: usize>
    Codec<CRC, RX_BUF_LEN, TX_BUF_LEN>
{
    pub fn new() -> Self {
        Self {
            rx: CodecRx::new(),
            tx: CodecTx::new(),
        }
    }

    // -- events -------------------------------------------------------------

    /// Line-IDLE drain-ISR entry. Returns a drift span when this stamp pairs
    /// with the previous same-flavor stamp over a contiguous Instruction
    /// burst — see [`CodecRx::on_idle`].
    pub fn on_idle(&mut self, now: u32, byte_ticks: u32) -> Option<(u32, u32)> {
        self.rx.on_idle(now, byte_ticks)
    }

    pub fn on_rx_progress(&mut self, remaining: u16) {
        self.rx.on_rx_progress(remaining);
    }

    /// Byte-ring HT/TC drain-ISR entry. Returns a drift span on a paired
    /// contiguous burst — see [`CodecRx::on_byte_batch_wake`].
    pub fn on_byte_batch_wake(&mut self, now: u32, byte_ticks: u32) -> Option<(u32, u32)> {
        self.rx.on_byte_batch_wake(now, byte_ticks)
    }

    // -- commands -----------------------------------------------------------

    pub fn cancel_skip(&mut self) {
        self.rx.cancel_skip();
    }

    /// Disjoint mutable borrow of the RX and TX halves. The parent driver's
    /// `poll` uses this to hand the dispatcher a `&mut CodecTx` reply handle
    /// while a frame-verdict borrow lives in `&mut CodecRx`.
    pub fn split_mut(&mut self) -> (&mut CodecRx<CRC, RX_BUF_LEN>, &mut CodecTx<CRC, TX_BUF_LEN>) {
        (&mut self.rx, &mut self.tx)
    }

    // -- accessors ------------------------------------------------------------

    pub fn instruction_count(&self) -> u32 {
        self.rx.instruction_count()
    }

    pub fn rx_buf_addr(&self) -> usize {
        self.rx.rx_buf_addr()
    }

    pub fn tx_buf_addr(&self) -> usize {
        self.tx.tx_buf_addr()
    }

    pub fn tx_len(&self) -> u16 {
        self.tx.tx_len()
    }
}

impl<CRC: CrcUmts, const RX_BUF_LEN: usize, const TX_BUF_LEN: usize> Default
    for Codec<CRC, RX_BUF_LEN, TX_BUF_LEN>
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
impl<CRC: CrcUmts, const RX_BUF_LEN: usize, const TX_BUF_LEN: usize>
    Codec<CRC, RX_BUF_LEN, TX_BUF_LEN>
{
    // Test-only forwarders — production reaches both halves through
    // `split_mut`; only this crate's tests drive the halves through the
    // whole `Codec`.

    pub(crate) fn send_slot(
        &mut self,
        slot: &dxl_protocol::Slot<'_>,
        position: dxl_protocol::SlotPosition,
    ) -> Result<(), dxl_protocol::WriteError> {
        self.tx.send_slot(slot, position)
    }

    pub(crate) fn own_reply_bytes(&self) -> &[u8] {
        self.tx.own_reply_bytes()
    }

    pub(crate) fn stage_rx_bytes_for_test(&mut self, at: u16, bytes: &[u8]) {
        self.rx.stage_rx_bytes_for_test(at, bytes);
    }
}
