//! Bytes ↔ packets. Composite over two field-projection halves — `CodecRx`
//! (streaming parser + RX byte ring + edge capture + drift bookkeeping) and
//! `CodecTx` (encoder + TX byte ring) — joined under `Codec`. The split lets
//! the parent driver hand a `&mut CodecTx` reply handle to the dispatcher
//! while a parser-event borrow lives in `CodecRx`.

mod anchor;
mod codec_rx;
mod codec_tx;
mod edge_capture;
mod edge_parser;
mod poll_event;
mod skip;
mod span;

pub use codec_rx::CodecRx;
#[cfg(test)]
pub(crate) use codec_rx::TAIL_BYTES_FOR_ANCHOR;
pub use codec_tx::CodecTx;
pub use edge_capture::EdgeCapture;
pub use edge_parser::edge_buf_len;
pub use poll_event::{PacketEnd, PollAction, PollEvent};

use dxl_protocol::CrcUmts;

use crate::traits::dxl::EdgeDma;

pub struct Codec<
    R: EdgeDma,
    CRC: CrcUmts,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> {
    pub(super) rx: CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>,
    pub(super) tx: CodecTx<CRC, TX_BUF_LEN>,
}

impl<
    R: EdgeDma,
    CRC: CrcUmts,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> Codec<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    pub fn new(ring: R) -> Self {
        Self {
            rx: CodecRx::new(ring),
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

    /// Forward to [`CodecRx::on_baud_change`]. The `rx_edge_comp_ticks`
    /// plumbing now feeds only the edge parser's tail-anchor/walk, which
    /// lost its live callers with the drift-span swap — kept until the
    /// edge subsystem is removed in the deletion chunk.
    pub fn on_baud_change(&mut self, rx_edge_comp_ticks: u16) {
        self.rx.on_baud_change(rx_edge_comp_ticks);
    }

    // -- commands -----------------------------------------------------------

    pub fn cancel_skip(&mut self) {
        self.rx.cancel_skip();
    }

    /// Disjoint mutable borrow of the RX and TX halves. The parent driver's
    /// `poll` uses this to hand the dispatcher a `&mut CodecTx` reply handle
    /// while a parser event borrow lives in `&mut CodecRx`.
    pub fn split_mut(
        &mut self,
    ) -> (
        &mut CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>,
        &mut CodecTx<CRC, TX_BUF_LEN>,
    ) {
        (&mut self.rx, &mut self.tx)
    }

    // -- accessors ------------------------------------------------------------

    pub fn instruction_count(&self) -> u32 {
        self.rx.instruction_count()
    }

    pub fn edges_addr(&self) -> usize {
        self.rx.edges_addr()
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

#[cfg(test)]
impl<
    R: EdgeDma,
    CRC: CrcUmts,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> Codec<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
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

    pub(crate) fn stage_tail_signature_for_test(
        &mut self,
        tail_bytes: &[u8],
        ticks_per_bit: u16,
        anchor_tick: u16,
    ) -> u16 {
        self.rx
            .stage_tail_signature_for_test(tail_bytes, ticks_per_bit, anchor_tick)
    }
}
