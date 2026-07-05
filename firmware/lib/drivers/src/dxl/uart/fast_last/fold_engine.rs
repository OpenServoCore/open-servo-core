//! Chain-CRC finalizer for Fast Sync / Bulk Read successor replies.
//!
//! Half of the [`FastLast`] sub-composite (§4.3): the CRC half. Under the
//! official per-block layout, the predecessor window's last two wire bytes
//! ARE the running chain-CRC state (each block ends with the cumulative
//! packet CRC — the e-manual: "CRC values are used for internal calculation
//! in DYNAMIXEL to confirm packet integrity between DYNAMIXELs"). So this
//! engine never streams the window through a fold: it waits for that
//! checkpoint, seeds the state from it, extends over the checkpoint's own
//! two bytes (they're stream bytes for OUR block's CRC) and our reply
//! bytes, and patches the TX buffer's trailing slot before DMA1_CH4 reads
//! it. O(own reply) work per exchange, regardless of chain length.
//!
//! Finalize never reaches into a sibling sub-driver: [`CrcPatchSink`] —
//! implemented by the codec's TX half, passed in by the composite —
//! exposes our reply bytes for the final mix and patches the trailing CRC
//! slot (driver-pattern §5.1: consumer-owned interface).
//!
//! [`FastLast`]: super::FastLast

use dxl_protocol::crc16_umts_continue;
use dxl_protocol::wire::CRC_BYTES;

use super::crc_patch_sink::CrcPatchSink;

/// Checkpoint-finalize state for one Fast successor reply.
pub struct FoldEngine {
    /// Wire cursor of the predecessor window's first byte (the Status
    /// packet's leading `0xFF`) — the composite-captured
    /// `next_status_pos` at the chain instruction's parse-complete.
    start_cursor: u32,
    /// Total predecessor wire bytes before our block — the window ends at
    /// `start_cursor + predecessor_bytes`, with the checkpoint (the
    /// predecessor's cumulative CRC) in its last [`CRC_BYTES`].
    predecessor_bytes: u32,
    active: bool,
}

impl FoldEngine {
    pub fn new() -> Self {
        Self {
            start_cursor: 0,
            predecessor_bytes: 0,
            active: false,
        }
    }

    // -- events -----------------------------------------------------------------

    /// Finalize from the wire checkpoint: seed the chain state from the
    /// predecessor's trailing CRC bytes, extend over those two bytes and
    /// our own reply bytes, and patch the trailing slot. The caller
    /// (composite pickup closure) is responsible for having read
    /// `checkpoint` from the window's last [`CRC_BYTES`] wire bytes.
    ///
    /// The checkpoint is inherited untrusted — per the format's design,
    /// per-block validation is the host's job; a corrupted upstream block
    /// fails the host's chain check with or without our patch.
    // RAM placement is load-bearing — see the note on `FsmScheduler::on_step`.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    pub fn finalize_from_checkpoint(
        &mut self,
        checkpoint: [u8; CRC_BYTES],
        sink: &mut impl CrcPatchSink,
    ) {
        if !self.active {
            return;
        }
        let mut state = u16::from_le_bytes(checkpoint);
        state = crc16_umts_continue(state, &checkpoint);
        state = crc16_umts_continue(state, sink.own_reply_bytes());
        sink.patch_crc(state);
        self.active = false;
    }

    // -- commands ---------------------------------------------------------------

    /// Arm for one successor reply. `start_cursor` is the composite-captured
    /// `PollEvent::Event::next_status_pos` at parse-complete;
    /// `predecessor_bytes` is `FastSlotInfo::bytes_before`. Idempotent.
    pub fn start(&mut self, start_cursor: u32, predecessor_bytes: u32) {
        self.start_cursor = start_cursor;
        self.predecessor_bytes = predecessor_bytes;
        self.active = true;
    }

    /// Drop the armed state. Idempotent. Called from
    /// [`DxlUart::on_tx_complete`] regardless of whether finalize ran —
    /// finalize already cleared `active`, so this is a no-op in the
    /// successful path.
    ///
    /// [`DxlUart::on_tx_complete`]: super::super::DxlUart::on_tx_complete
    pub fn cancel(&mut self) {
        self.active = false;
    }

    // -- accessors --------------------------------------------------------------

    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Wire cursor one past the predecessor window — the checkpoint's end.
    /// The composite's pickup reads the window's last [`CRC_BYTES`] wire
    /// bytes once the ring has published through this cursor, and advances
    /// consumption to it so the parser resumes at our block's slot.
    pub fn window_end_cursor(&self) -> u32 {
        self.start_cursor.wrapping_add(self.predecessor_bytes)
    }
}

impl Default for FoldEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::codec::Codec;
    use dxl_protocol::types::{Id, Slot, StatusError};
    use dxl_protocol::{CrcUmts, SlotPosition, SoftwareCrcUmts};

    const RX_BUF_LEN: usize = 64;
    const TX_BUF_LEN: usize = 140;

    type TestCodec = Codec<SoftwareCrcUmts, RX_BUF_LEN, TX_BUF_LEN>;

    fn make_codec_with_successor_reply() -> TestCodec {
        let mut c: TestCodec = Codec::new();
        let payload = [0xAA_u8, 0xBB];
        let slot = Slot {
            id: Id::new(0x07),
            error: StatusError::OK,
            data: &payload,
        };
        // Placeholder CRC; the checkpoint finalize patches it.
        c.send_slot(&slot, SlotPosition::Successor { crc: 0x0000 })
            .expect("encode fits");
        c
    }

    /// Read the last `n` bytes of the encoded reply as an array.
    ///
    /// SAFETY: `tx_buf` is initialized up to `tx_len` bytes by the earlier
    /// `send_slot` call in `make_codec_with_successor_reply`.
    fn tx_trailing<const N: usize>(codec: &TestCodec) -> [u8; N] {
        let len = codec.tx_len() as usize;
        let s = unsafe { core::slice::from_raw_parts(codec.tx_buf_addr() as *const u8, len) };
        let mut out = [0u8; N];
        out.copy_from_slice(&s[len - N..]);
        out
    }

    #[test]
    fn finalize_matches_a_full_fold_over_the_wire() {
        // The checkpoint shortcut must be byte-for-byte equivalent to
        // folding the entire predecessor window: build a wire-faithful
        // window (bytes + their cumulative CRC), finalize from the
        // checkpoint alone, and compare against the host-side full fold.
        let mut codec = make_codec_with_successor_reply();
        let window_body = [
            0xFF_u8, 0xFF, 0xFD, 0x00, 0xFE, 0x0B, 0x00, 0x55, 0x00, 0x32,
        ];
        let checkpoint = crc16_umts_continue(0, &window_body).to_le_bytes();

        let mut fl = FoldEngine::new();
        fl.start(0, (window_body.len() + CRC_BYTES) as u32);
        fl.finalize_from_checkpoint(checkpoint, &mut codec.tx);

        assert!(!fl.is_active(), "finalize clears active");
        let patched = tx_trailing::<2>(&codec);
        let mut expected = SoftwareCrcUmts::new();
        expected.update(&window_body);
        expected.update(&checkpoint);
        expected.update(codec.own_reply_bytes());
        assert_eq!(u16::from_le_bytes(patched), expected.finalize());
    }

    #[test]
    fn window_end_cursor_is_start_plus_predecessor_bytes() {
        let mut fl = FoldEngine::new();
        fl.start(20, 14);
        assert_eq!(fl.window_end_cursor(), 34);
    }

    #[test]
    fn cancel_clears_without_patching() {
        let mut codec = make_codec_with_successor_reply();
        let trailing_before: [u8; 2] = tx_trailing(&codec);

        let mut fl = FoldEngine::new();
        fl.start(0, 14);
        fl.cancel();
        assert!(!fl.is_active());

        // Post-cancel finalize is a no-op.
        fl.finalize_from_checkpoint([0x12, 0x34], &mut codec.tx);
        assert_eq!(tx_trailing::<2>(&codec), trailing_before);
    }

    #[test]
    fn second_finalize_is_a_no_op() {
        let mut codec = make_codec_with_successor_reply();
        let mut fl = FoldEngine::new();
        fl.start(0, 14);
        fl.finalize_from_checkpoint([0x12, 0x34], &mut codec.tx);
        let first: [u8; 2] = tx_trailing(&codec);

        fl.finalize_from_checkpoint([0x56, 0x78], &mut codec.tx);
        assert_eq!(tx_trailing::<2>(&codec), first, "already finalized");
    }
}
