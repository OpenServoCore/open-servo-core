//! TX half — encoder + TX byte ring. Splits off from
//! [`CodecRx`](super::codec_rx::CodecRx) under [`Codec`](super::Codec) so
//! the parent driver's split-borrow `poll` can hand the dispatcher's reply
//! handle a `&mut CodecTx` alongside a parser event stream that borrows
//! the disjoint RX half.

use core::marker::PhantomData;

use dxl_protocol::types::{Id, Slot, Status, StatusError};
use dxl_protocol::wire::CRC_BYTES;
use dxl_protocol::{
    Chunk, CrcUmts, SlotPosition, WriteError, encode_slot, encode_slot_chunked, encode_status,
    encode_status_chunked,
};

pub struct CodecTx<CRC: CrcUmts, const TX_BUF_LEN: usize> {
    /// DMA1_CH4 source for transmitted bytes. Single-shot DMA per reply:
    /// the encoder methods ([`Self::send_status`], [`Self::send_slot`])
    /// fill it from offset 0; bringup hands [`Self::tx_buf_addr`] to
    /// `dma::configure(CH4, ...)` once. Producer (encoder) and consumer
    /// (DMA shift-out) phases are exclusive — the composite holds the only
    /// `&mut CodecTx` and stops writing once it routes to the scheduler —
    /// so no `SyncUnsafeCell`.
    tx_buf: [u8; TX_BUF_LEN],
    /// Valid byte count of the most-recent encoded frame — the DMA transfer
    /// count. Bytes past it in `tx_buf` are stale and never shifted out.
    tx_len: usize,
    _crc: PhantomData<CRC>,
}

impl<CRC: CrcUmts, const TX_BUF_LEN: usize> CodecTx<CRC, TX_BUF_LEN> {
    pub(super) fn new() -> Self {
        Self {
            tx_buf: [0; TX_BUF_LEN],
            tx_len: 0,
            _crc: PhantomData,
        }
    }
}

#[cfg(test)]
impl<CRC: CrcUmts, const TX_BUF_LEN: usize> CodecTx<CRC, TX_BUF_LEN> {
    /// Standalone TX half for `ReplyHandle` tests — production constructs
    /// only through `Codec::new`.
    pub(crate) fn new_for_test() -> Self {
        Self::new()
    }

    /// The trailing CRC slot of the encoded TX buffer — fold tests assert
    /// the placeholder → patched transition without raw-pointer reads.
    pub(crate) fn trailing_crc_slot_for_test(&self) -> [u8; CRC_BYTES] {
        self.tx_buf[self.tx_len - CRC_BYTES..self.tx_len]
            .try_into()
            .unwrap()
    }
}

// -- commands -----------------------------------------------------------

impl<CRC: CrcUmts, const TX_BUF_LEN: usize> CodecTx<CRC, TX_BUF_LEN> {
    /// Encode a Status reply into the TX buffer from offset 0, recording the
    /// frame length in [`Self::tx_len`] for the DMA transfer count. The fused
    /// [`encode_status`] returns `WriteError::Overflow` if the frame exceeds
    /// `TX_BUF_LEN`; `tx_len` is left untouched on error.
    pub fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        let n = match status {
            Status::Empty { id, error } => encode_status::<CRC>(&mut self.tx_buf, id, error, &[])?,
            Status::Ping { id, error, status } => {
                let m = status.model.to_le_bytes();
                let payload = [m[0], m[1], status.fw_version];
                encode_status::<CRC>(&mut self.tx_buf, id, error, &payload)?
            }
            Status::Read { id, error, data } => {
                encode_status::<CRC>(&mut self.tx_buf, id, error, data)?
            }
            Status::Raw { id, error, payload }
            | Status::FastSyncRead { id, error, payload }
            | Status::FastBulkRead { id, error, payload } => {
                encode_status::<CRC>(&mut self.tx_buf, id, error, payload)?
            }
        };
        self.tx_len = n;
        Ok(())
    }

    /// Encode one Fast slot reply into the TX buffer from offset 0. The fused
    /// [`encode_slot`] dispatches on `position` (First/Successor) and writes
    /// the header, payload, and (locally-computed or caller-supplied) CRC.
    pub fn send_slot(&mut self, slot: &Slot<'_>, position: SlotPosition) -> Result<(), WriteError> {
        self.tx_len = encode_slot::<CRC>(&mut self.tx_buf, slot, position)?;
        Ok(())
    }

    /// Streamed counterpart of [`Self::send_status`] for `Status::Read`
    /// replies: the dispatcher hands an iterator of [`Chunk`]s sourced
    /// directly from a control-table read; the stuffer consumes them in
    /// place of a scratch buffer.
    pub fn send_status_read_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        self.tx_len = encode_status_chunked::<CRC, _>(&mut self.tx_buf, id, error, chunks)?;
        Ok(())
    }

    /// Streamed counterpart of [`Self::send_slot`]: slot body bytes come
    /// from a chunk iterator. Slot bodies are unstuffed, so each
    /// `Chunk::Slice` / `Chunk::Zero` run copies straight into the buffer.
    pub fn send_slot_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        position: SlotPosition,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        self.tx_len = encode_slot_chunked::<CRC, _>(&mut self.tx_buf, id, error, position, chunks)?;
        Ok(())
    }

    /// Overwrite the trailing [`CRC_BYTES`] of the encoded TX buffer with
    /// `crc` in little-endian. The Fast Last chain-CRC fold path calls this
    /// once the predecessor wire bytes have been folded and our own reply
    /// bytes are mixed in: the encoder emitted a placeholder CRC at
    /// `send_slot(Last)` time and this patches it before DMA1_CH4's read
    /// cursor reaches the trailing slot (doc §10.6). No-op when `tx_buf`
    /// hasn't been encoded yet.
    pub fn patch_crc(&mut self, crc: u16) {
        let n = self.tx_len;
        if n < CRC_BYTES {
            return;
        }
        self.tx_buf[n - CRC_BYTES..n].copy_from_slice(&crc.to_le_bytes());
    }
}

// -- accessors ----------------------------------------------------------

impl<CRC: CrcUmts, const TX_BUF_LEN: usize> CodecTx<CRC, TX_BUF_LEN> {
    /// Stable peripheral-memory address for DMA1_CH4's source buffer.
    /// Bringup hands this to `dma::configure(CH4, ...)` once; per-reply
    /// scheduling reads [`Self::tx_len`] for the transfer count.
    pub fn tx_buf_addr(&self) -> usize {
        self.tx_buf.as_ptr() as usize
    }

    /// Length in bytes of the most-recent encoded packet — the DMA1_CH4
    /// transfer count for the next reply. Zero until the first send.
    pub fn tx_len(&self) -> u16 {
        self.tx_len as u16
    }

    /// Slice of our own reply bytes excluding the trailing [`CRC_BYTES`]
    /// slot — the bytes the Fast Last fold mixes into the chain CRC
    /// before patching. Empty when no reply has been encoded yet.
    pub fn own_reply_bytes(&self) -> &[u8] {
        let n = self.tx_len;
        if n < CRC_BYTES {
            return &[];
        }
        &self.tx_buf[..n - CRC_BYTES]
    }
}

/// The Fast Last fold engine finalizes into the TX half without importing it
/// (driver-pattern §5.1). Both methods delegate to the inherent forms above.
impl<CRC: CrcUmts, const TX_BUF_LEN: usize> crate::dxl::uart::fast_last::CrcPatchSink
    for CodecTx<CRC, TX_BUF_LEN>
{
    fn own_reply_bytes(&self) -> &[u8] {
        CodecTx::own_reply_bytes(self)
    }

    fn patch_crc(&mut self, crc: u16) {
        CodecTx::patch_crc(self, crc)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dxl_protocol::SoftwareCrcUmts;

    /// `DXL_TX_MAX_BYTES` per `osc-core::services::dxl::limits` — chip-side
    /// registry uses the same value.
    const TX_BUF_LEN: usize = 140;
    const TEST_ID: u8 = 0x07;

    fn make() -> CodecTx<SoftwareCrcUmts, TX_BUF_LEN> {
        CodecTx::new()
    }

    /// Read the encoded buffer back through the DMA-facing address/len
    /// contract — the same view the DMA shift-out sees.
    ///
    /// SAFETY: `tx_buf` is initialized up to `tx_len` bytes by a prior
    /// `send_*` call; read-only slice for assertion.
    fn wire_view(tx: &CodecTx<SoftwareCrcUmts, TX_BUF_LEN>) -> &[u8] {
        unsafe { core::slice::from_raw_parts(tx.tx_buf_addr() as *const u8, tx.tx_len() as usize) }
    }

    #[test]
    fn send_status_writes_wire_bytes_into_tx_buf() {
        let mut tx = make();
        tx.send_status(Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        })
        .expect("encode fits");

        let mut expected = [0u8; TX_BUF_LEN];
        let n =
            encode_status::<SoftwareCrcUmts>(&mut expected, Id::new(TEST_ID), StatusError::OK, &[])
                .unwrap();
        assert!(tx.tx_len() > 0);
        assert_eq!(tx.tx_len() as usize, n);
        assert_eq!(wire_view(&tx), &expected[..n]);
    }

    #[test]
    fn send_status_overwrites_previous_contents() {
        let mut tx = make();
        tx.send_status(Status::Ping {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            status: dxl_protocol::types::PingStatus {
                model: 0x0123,
                fw_version: 0x45,
            },
        })
        .unwrap();
        let first_len = tx.tx_len();
        assert!(first_len > 0);

        tx.send_status(Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        })
        .unwrap();
        let second_len = tx.tx_len();
        assert!(second_len < first_len);
        assert_eq!(&wire_view(&tx)[0..4], &[0xFF, 0xFF, 0xFD, 0x00]);
    }

    #[test]
    fn tx_buf_addr_is_stable() {
        let tx = make();
        let a = tx.tx_buf_addr();
        let b = tx.tx_buf_addr();
        assert_eq!(a, b);
        assert_ne!(a, 0);
    }

    #[test]
    fn send_slot_only_writes_header_plus_body_plus_crc() {
        let mut tx = make();
        let payload = [0x11_u8, 0x22, 0x33];
        let slot = Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data: &payload,
        };
        tx.send_slot(&slot, SlotPosition::First { packet_length: 8 })
            .expect("encode fits");

        assert_eq!(&wire_view(&tx)[0..4], &[0xFF, 0xFF, 0xFD, 0x00]);

        let mut expected = [0u8; TX_BUF_LEN];
        let n = encode_slot::<SoftwareCrcUmts>(
            &mut expected,
            &slot,
            SlotPosition::First { packet_length: 8 },
        )
        .unwrap();
        assert_eq!(wire_view(&tx), &expected[..n]);
    }

    #[test]
    fn send_slot_last_writes_caller_supplied_crc() {
        let mut tx = make();
        let payload = [0xAA_u8, 0xBB];
        let slot = Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data: &payload,
        };
        tx.send_slot(&slot, SlotPosition::Successor { crc: 0xDEAD })
            .expect("encode fits");

        let wire = wire_view(&tx);
        assert_eq!(&wire[wire.len() - 2..], &[0xAD, 0xDE]);
    }

    #[test]
    fn patch_crc_overwrites_last_two_bytes_le() {
        let mut tx = make();
        let payload = [0x11_u8, 0x22, 0x33];
        let slot = Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data: &payload,
        };
        tx.send_slot(&slot, SlotPosition::Successor { crc: 0x0000 })
            .expect("encode fits");

        tx.patch_crc(0xBEEF);
        let wire = wire_view(&tx);
        assert_eq!(&wire[wire.len() - 2..], &[0xEF, 0xBE]);
    }

    #[test]
    fn patch_crc_noop_when_tx_buf_empty() {
        let mut tx = make();
        assert_eq!(tx.tx_len(), 0);
        tx.patch_crc(0xDEAD);
        assert_eq!(tx.tx_len(), 0);
    }

    #[test]
    fn own_reply_bytes_excludes_trailing_crc_slot() {
        let mut tx = make();
        let payload = [0xAA_u8, 0xBB];
        let slot = Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data: &payload,
        };
        tx.send_slot(&slot, SlotPosition::Successor { crc: 0xDEAD })
            .expect("encode fits");

        let len = tx.tx_len() as usize;
        let bytes = tx.own_reply_bytes();
        let wire = wire_view(&tx);
        assert_eq!(bytes.len(), len - 2);
        assert_eq!(bytes, &wire[..len - 2]);
    }

    #[test]
    fn own_reply_bytes_empty_when_tx_buf_empty() {
        let tx = make();
        assert!(tx.own_reply_bytes().is_empty());
    }
}
