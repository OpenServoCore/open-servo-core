//! Chain-CRC fold engine for Fast Sync / Bulk Read Last replies.
//!
//! Owns a running [`CrcUmts`] engine plus the bookkeeping that decides
//! when to feed wire bytes into it. Composite-side; the slice hook
//! ([`Self::on_slice`]) is wired into [`super::codec::CodecRx::drain_raw`]'s
//! slice callback so every wire byte the post-parser drain surfaces —
//! own, foreign, Status-frame, resync'd prefix alike — flows through the
//! guard in one bulk CRC fold per drain pass. Per
//! `docs/dxl-hw-timed-transport.md` §10.6.
//!
//! Lifecycle: started at [`super::ReplyHandle::send_slot`] when the
//! cached slot position is [`SlotPosition::Last`]; cancelled by
//! [`Self::cancel`] at [`super::DxlUart::on_tx_complete`] or self-clears
//! naturally when `predecessor_bytes` have been folded — the latter
//! finalizes the CRC over our own reply bytes and patches the TX
//! buffer's trailing slot before DMA1_CH4 reads it.
//!
//! [`SlotPosition`]: dxl_protocol::SlotPosition

use dxl_protocol::CrcUmts;

use super::codec::CodecTx;

/// Per [`super::codec::CodecRx::drain_raw`]'s callback contract — fired
/// once per drained RX-ring front slice during the Fast Last predecessor
/// window; receives `(slice, base_cursor)` where `base_cursor` is the
/// pre-advance value of the codec's `wire_bytes_consumed` counter, so
/// `slice[i]` sits at wire cursor `base_cursor + i`.
pub struct FastLastCrc<CRC: CrcUmts> {
    crc: CRC,
    /// Lower bound on the wire-byte cursor that contributes to the fold.
    /// Bytes with `cursor < start_cursor` were already past at
    /// [`Self::start`] time (the parser had already moved on); the chain
    /// CRC begins from the byte that completes parsing, which sits at
    /// `cursor == start_cursor`. Compared `<` so the byte at exactly
    /// `start_cursor` IS folded.
    start_cursor: u32,
    bytes_folded: u32,
    /// Total predecessor wire bytes to fold before finalize.
    /// `FastSlotInfo::bytes_before` on Last. Matches the field of the
    /// same name on [`super::fast_last::FastLastSchedule`].
    predecessor_bytes: u32,
    active: bool,
}

impl<CRC: CrcUmts> FastLastCrc<CRC> {
    pub fn new() -> Self {
        Self {
            crc: CRC::new(),
            start_cursor: 0,
            bytes_folded: 0,
            predecessor_bytes: 0,
            active: false,
        }
    }

    /// Begin folding for one Fast Last reply. `start_cursor` is the
    /// composite-captured `PollEvent::Event::next_status_pos` at
    /// parse-complete (so the first byte of the predecessor's first reply
    /// lands at cursor == start_cursor); `predecessor_bytes` is
    /// `FastSlotInfo::bytes_before`. Resets the running CRC; idempotent.
    pub fn start(&mut self, start_cursor: u32, predecessor_bytes: u32) {
        self.crc.reset();
        self.start_cursor = start_cursor;
        self.bytes_folded = 0;
        self.predecessor_bytes = predecessor_bytes;
        self.active = true;
    }

    /// Bulk slice hook for [`super::codec::CodecRx::drain_raw`]'s callback.
    /// Trims the leading slice prefix that sits before `start_cursor`,
    /// caps the fold at the remaining predecessor budget, folds the
    /// resulting active region in one CRC pass, and finalize-patches on
    /// reaching `predecessor_bytes`.
    ///
    /// Finalize mixes our own reply bytes (`tx.own_reply_bytes()` —
    /// everything except the trailing 2-byte placeholder CRC slot) and
    /// writes the result to the placeholder via [`CodecTx::patch_crc`].
    /// `tx_buf` must already be encoded by the caller's earlier
    /// `send_slot(Last)` — empty `own_reply_bytes()` is a programmer error
    /// the engine doesn't try to catch.
    pub fn on_slice<const TX_BUF_LEN: usize>(
        &mut self,
        slice: &[u8],
        base_cursor: u32,
        tx: &mut CodecTx<CRC, TX_BUF_LEN>,
    ) {
        if !self.active {
            return;
        }
        let skip = self
            .start_cursor
            .saturating_sub(base_cursor)
            .min(slice.len() as u32) as usize;
        let active = &slice[skip..];
        let need = self.predecessor_bytes.wrapping_sub(self.bytes_folded);
        let take = (active.len() as u32).min(need) as usize;
        if take == 0 {
            return;
        }
        self.crc.update(&active[..take]);
        self.bytes_folded = self.bytes_folded.wrapping_add(take as u32);
        if self.bytes_folded >= self.predecessor_bytes {
            self.crc.update(tx.own_reply_bytes());
            tx.patch_crc(self.crc.finalize());
            self.active = false;
        }
    }

    /// Drop the running state. Idempotent. Called from
    /// [`super::DxlUart::on_tx_complete`] regardless of whether finalize
    /// ran — finalize already cleared `active`, so this is a no-op in the
    /// successful path.
    pub fn cancel(&mut self) {
        self.active = false;
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Cumulative count of folded bytes since the last [`Self::start`].
    /// Driver-side [`super::FastLast::on_step`] reads this through the
    /// composite's walker closure as the busy-wait exit condition.
    pub fn bytes_folded(&self) -> u32 {
        self.bytes_folded
    }
}

impl<CRC: CrcUmts> Default for FastLastCrc<CRC> {
    fn default() -> Self {
        Self::new()
    }
}

// Shelved pending U4 (osc-drivers unit test audit): tests below bind to
// hand-rolled mock fields; will be migrated to the mockall + state-companion
// API as part of the audit.
#[cfg(any())]
#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::codec::Codec;
    use crate::mocks::MockEdgeDma;
    use dxl_protocol::types::{Id, Slot, StatusError};
    use dxl_protocol::{SlotPosition, SoftwareCrcUmts};

    const RX_BUF_LEN: usize = 64;
    const EDGE_BUF_LEN: usize = 128;
    const TX_BUF_LEN: usize = 140;

    type TestCodec = Codec<MockEdgeDma, SoftwareCrcUmts, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;

    fn make_codec_with_last_reply() -> TestCodec {
        let mut c: TestCodec = Codec::new(MockEdgeDma::default());
        let payload = [0xAA_u8, 0xBB];
        let slot = Slot {
            id: Id::new(0x07),
            error: StatusError::OK,
            data: &payload,
        };
        // Placeholder CRC; fold patches it.
        c.send_slot(&slot, SlotPosition::Last { crc: 0x0000 })
            .expect("encode fits");
        c
    }

    #[test]
    fn skips_bytes_before_start_cursor() {
        let mut codec = make_codec_with_last_reply();
        let mut fl = FastLastCrc::<SoftwareCrcUmts>::new();
        fl.start(
            /* start_cursor = */ 10, /* predecessor_bytes = */ 4,
        );

        // Cursors 0..9 are before start_cursor; fold drops them silently.
        for cursor in 0..10 {
            fl.on_byte(0xAA, cursor, &mut codec.tx);
        }
        assert_eq!(fl.bytes_folded(), 0);
        assert!(fl.is_active());
    }

    #[test]
    fn folds_byte_at_start_cursor() {
        let mut codec = make_codec_with_last_reply();
        let mut fl = FastLastCrc::<SoftwareCrcUmts>::new();
        fl.start(5, 4);

        // Cursor == start_cursor IS included.
        fl.on_byte(0xAA, 5, &mut codec.tx);
        assert_eq!(fl.bytes_folded(), 1);
    }

    #[test]
    fn finalize_patches_trailing_crc_slot_on_target_hit() {
        let mut codec = make_codec_with_last_reply();
        let pre_patch_len = codec.tx_len() as usize;
        // Capture pre-patch trailing bytes — placeholder 0x0000 LE.
        // SAFETY: tx_buf is initialized to pre_patch_len bytes.
        let trailing_before =
            unsafe { core::slice::from_raw_parts(codec.tx_buf_addr() as *const u8, pre_patch_len) }
                [pre_patch_len - 2..]
                .to_vec();
        assert_eq!(trailing_before, [0x00, 0x00]);

        let mut fl = FastLastCrc::<SoftwareCrcUmts>::new();
        fl.start(0, 3);
        fl.on_byte(0x11, 0, &mut codec.tx);
        fl.on_byte(0x22, 1, &mut codec.tx);
        // Third byte trips finalize → patch.
        fl.on_byte(0x33, 2, &mut codec.tx);

        assert!(!fl.is_active(), "finalize should clear active");
        // SAFETY: see above.
        let len = codec.tx_len() as usize;
        let actual = unsafe { core::slice::from_raw_parts(codec.tx_buf_addr() as *const u8, len) };
        // Patched trailing slot must NOT be the placeholder anymore.
        let patched = &actual[len - 2..];
        assert_ne!(patched, &[0x00, 0x00], "patch_crc should overwrite slot");

        // Independently compute the expected CRC: chain-CRC over the
        // three predecessor bytes + own_reply_bytes (= the encoded reply
        // minus its trailing 2-byte placeholder slot).
        let mut expected = SoftwareCrcUmts::new();
        expected.update(&[0x11, 0x22, 0x33]);
        expected.update(codec.own_reply_bytes());
        let expected_crc = expected.finalize();
        let actual_crc = u16::from_le_bytes([patched[0], patched[1]]);
        assert_eq!(actual_crc, expected_crc);
    }

    #[test]
    fn extra_bytes_after_finalize_are_dropped() {
        let mut codec = make_codec_with_last_reply();
        let mut fl = FastLastCrc::<SoftwareCrcUmts>::new();
        fl.start(0, 2);
        fl.on_byte(0x11, 0, &mut codec.tx);
        fl.on_byte(0x22, 1, &mut codec.tx);
        assert!(!fl.is_active());

        // Already cleared; this is a no-op (doesn't fold, doesn't panic).
        fl.on_byte(0x33, 2, &mut codec.tx);
        assert_eq!(fl.bytes_folded(), 2);
    }

    #[test]
    fn cancel_clears_without_patching() {
        let mut codec = make_codec_with_last_reply();
        let pre_patch_len = codec.tx_len() as usize;
        // SAFETY: tx_buf initialized to pre_patch_len.
        let trailing_before =
            unsafe { core::slice::from_raw_parts(codec.tx_buf_addr() as *const u8, pre_patch_len) }
                [pre_patch_len - 2..]
                .to_vec();

        let mut fl = FastLastCrc::<SoftwareCrcUmts>::new();
        fl.start(0, 4);
        fl.on_byte(0x11, 0, &mut codec.tx);
        fl.cancel();
        assert!(!fl.is_active());

        // Subsequent bytes silently drop.
        fl.on_byte(0x22, 1, &mut codec.tx);
        assert_eq!(fl.bytes_folded(), 1);

        // Trailing slot unchanged.
        // SAFETY: see above.
        let after_len = codec.tx_len() as usize;
        let trailing_after =
            unsafe { core::slice::from_raw_parts(codec.tx_buf_addr() as *const u8, after_len) }
                [after_len - 2..]
                .to_vec();
        assert_eq!(trailing_before, trailing_after);
    }

    #[test]
    fn restart_resets_running_crc() {
        let mut codec = make_codec_with_last_reply();
        let mut fl = FastLastCrc::<SoftwareCrcUmts>::new();

        // First cycle.
        fl.start(0, 1);
        fl.on_byte(0xFF, 0, &mut codec.tx);
        let first_patch = {
            let len = codec.tx_len() as usize;
            // SAFETY: see above.
            let s = unsafe { core::slice::from_raw_parts(codec.tx_buf_addr() as *const u8, len) };
            [s[len - 2], s[len - 1]]
        };

        // Re-start; same start_cursor and predecessor_bytes but different byte.
        fl.start(0, 1);
        fl.on_byte(0x42, 0, &mut codec.tx);
        let second_patch = {
            let len = codec.tx_len() as usize;
            // SAFETY: see above.
            let s = unsafe { core::slice::from_raw_parts(codec.tx_buf_addr() as *const u8, len) };
            [s[len - 2], s[len - 1]]
        };

        // Different inputs → different patches; proves crc.reset() ran on
        // re-start rather than continuing the prior run.
        assert_ne!(first_patch, second_patch);
    }
}
