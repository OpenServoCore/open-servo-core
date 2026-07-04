//! Chain-CRC fold engine for Fast Sync / Bulk Read Last replies.
//!
//! Half of the [`FastLast`] sub-composite (§4.3): the fold half. Owns the
//! running CRC-16/UMTS state plus the bookkeeping that decides when to feed
//! wire bytes into it. The slice hook ([`Self::on_slice`]) is wired into
//! [`CodecRx::drain_raw`]'s slice callback so every wire byte the
//! post-parser drain surfaces — own, foreign, Status-frame, resync'd prefix
//! alike — flows through the guard in one bulk CRC fold per drain pass. Per
//! `docs/dxl-hw-timed-transport.md` §10.6.
//!
//! Lifecycle: started at [`ReplyHandle::send_slot`] when the cached slot
//! position is [`SlotPosition::Last`]; cancelled by [`Self::cancel`] at
//! [`DxlUart::on_tx_complete`] or self-clears naturally when
//! `predecessor_bytes` have been folded — the latter finalizes the CRC over
//! our own reply bytes and patches the TX buffer's trailing slot before
//! DMA1_CH4 reads it.
//!
//! Finalize never reaches into a sibling sub-driver: the fold engine folds
//! the predecessor bytes but never learns *where* our own reply lives.
//! [`CrcPatchSink`] — implemented by the codec's TX half, passed in by the
//! composite — exposes our reply bytes for the final mix and patches the
//! trailing CRC slot (driver-pattern §5.1: consumer-owned interface).
//!
//! [`FastLast`]: super::FastLast
//! [`CodecRx::drain_raw`]: super::super::codec::CodecRx::drain_raw
//! [`ReplyHandle::send_slot`]: super::super::ReplyHandle::send_slot
//! [`DxlUart::on_tx_complete`]: super::super::DxlUart::on_tx_complete
//! [`SlotPosition`]: dxl_protocol::SlotPosition

use dxl_protocol::crc16_umts_continue;

use super::crc_patch_sink::CrcPatchSink;

/// Running chain-CRC fold state for one Fast Last reply. Consumes the
/// per-drain slice contract from [`super::super::codec::CodecRx::drain_raw`]:
/// invoked once per drained RX-ring front slice during the predecessor window;
/// receives `(slice, base_cursor)` where `base_cursor` is the pre-advance
/// value of the codec's `wire_bytes_consumed` counter, so `slice[i]` sits at
/// wire cursor `base_cursor + i`.
pub struct FoldEngine {
    /// Running CRC-16/UMTS state over the folded predecessor bytes. Raw
    /// protocol state (not a [`CrcUmts`] engine) so the per-byte step is a
    /// guaranteed-inline table fold — the spin path cannot afford a call
    /// that may resolve into flash (see the placement note on `on_slice`).
    ///
    /// [`CrcUmts`]: dxl_protocol::CrcUmts
    state: u16,
    /// Lower bound on the wire-byte cursor that contributes to the fold.
    /// Bytes with `cursor < start_cursor` were already past at [`Self::start`]
    /// time (the parser had already moved on); the chain CRC begins from the
    /// byte that completes parsing, which sits at `cursor == start_cursor`.
    /// Compared `<` so the byte at exactly `start_cursor` IS folded.
    start_cursor: u32,
    bytes_folded: u32,
    /// Total predecessor wire bytes to fold before finalize.
    /// `FastSlotInfo::bytes_before` on Last. The composite passes the same
    /// value to the scheduler half at `start`.
    predecessor_bytes: u32,
    active: bool,
}

impl FoldEngine {
    pub fn new() -> Self {
        Self {
            state: 0,
            start_cursor: 0,
            bytes_folded: 0,
            predecessor_bytes: 0,
            active: false,
        }
    }

    // -- events -----------------------------------------------------------------

    /// Bulk slice hook for [`super::super::codec::CodecRx::drain_raw`]'s
    /// callback. Trims the leading slice prefix that sits before
    /// `start_cursor`, caps the fold at the remaining predecessor budget,
    /// folds the resulting active region in one CRC pass, and
    /// finalize-patches on reaching `predecessor_bytes`.
    ///
    /// Finalize mixes our own reply bytes (`sink.own_reply_bytes()` —
    /// everything except the trailing 2-byte placeholder CRC slot) and writes
    /// the result to the placeholder via [`CrcPatchSink::patch_crc`]. The
    /// sink must already be encoded by the caller's earlier `send_slot(Last)`
    /// — empty `own_reply_bytes()` is a programmer error the engine doesn't
    /// try to catch.
    ///
    /// (A predicted-tail shortcut — patching `CRC_BYTES` early by treating
    /// the window's last two bytes as the running chain CRC — was tried and
    /// reverted: this codebase's FAST chain carries ONE trailing CRC, so a
    /// predecessor window ends in plain slot data, not a predictable value.
    /// See task #2 notes; under ROBOTIS' per-slot-CRC FAST layout the trick
    /// would be sound.)
    // RAM placement is load-bearing — see the note on `FsmScheduler::on_step`.
    // No `inline(never)`: see `CodecRx::drain_raw` — same fusion rationale.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    pub fn on_slice(&mut self, slice: &[u8], base_cursor: u32, sink: &mut impl CrcPatchSink) {
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
        self.state = crc16_umts_continue(self.state, &active[..take]);
        self.bytes_folded = self.bytes_folded.wrapping_add(take as u32);
        if self.bytes_folded >= self.predecessor_bytes {
            self.state = crc16_umts_continue(self.state, sink.own_reply_bytes());
            sink.patch_crc(self.state);
            self.active = false;
        }
    }

    // -- commands ---------------------------------------------------------------

    /// Begin folding for one Fast Last reply. `start_cursor` is the
    /// composite-captured `PollEvent::Event::next_status_pos` at
    /// parse-complete (so the first byte of the predecessor's first reply
    /// lands at cursor == start_cursor); `predecessor_bytes` is
    /// `FastSlotInfo::bytes_before`. Resets the running CRC; idempotent.
    pub fn start(&mut self, start_cursor: u32, predecessor_bytes: u32) {
        self.state = 0;
        self.start_cursor = start_cursor;
        self.bytes_folded = 0;
        self.predecessor_bytes = predecessor_bytes;
        self.active = true;
    }

    /// Drop the running state. Idempotent. Called from
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

    /// Cumulative count of folded bytes since the last [`Self::start`]. The
    /// scheduler half reads this through the composite's walker closure as
    /// the busy-wait exit condition.
    pub fn bytes_folded(&self) -> u32 {
        self.bytes_folded
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
    use crate::mocks::MockEdgeDma;
    use dxl_protocol::types::{Id, Slot, StatusError};
    use dxl_protocol::{CrcUmts, SlotPosition, SoftwareCrcUmts};

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
        c.send_slot(&slot, SlotPosition::Successor { crc: 0x0000 })
            .expect("encode fits");
        c
    }

    /// Read the last `n` bytes of the encoded reply as an array.
    ///
    /// SAFETY: `tx_buf` is initialized up to `tx_len` bytes by the earlier
    /// `send_slot` call in `make_codec_with_last_reply`.
    fn tx_trailing<const N: usize>(codec: &TestCodec) -> [u8; N] {
        let len = codec.tx_len() as usize;
        let s = unsafe { core::slice::from_raw_parts(codec.tx_buf_addr() as *const u8, len) };
        let mut out = [0u8; N];
        out.copy_from_slice(&s[len - N..]);
        out
    }

    #[test]
    fn skips_bytes_before_start_cursor() {
        let mut codec = make_codec_with_last_reply();
        let mut fl = FoldEngine::new();
        fl.start(
            /* start_cursor = */ 10, /* predecessor_bytes = */ 4,
        );

        // Slice cursors 0..9; all before start_cursor → fold drops silently.
        fl.on_slice(&[0xAA; 10], 0, &mut codec.tx);
        assert_eq!(fl.bytes_folded(), 0);
        assert!(fl.is_active());
    }

    #[test]
    fn folds_byte_at_start_cursor() {
        let mut codec = make_codec_with_last_reply();
        let mut fl = FoldEngine::new();
        fl.start(5, 4);

        // Slice base_cursor == start_cursor; byte IS included.
        fl.on_slice(&[0xAA], 5, &mut codec.tx);
        assert_eq!(fl.bytes_folded(), 1);
    }

    /// Arbitrary predecessor window bytes for fold tests (body plus two
    /// trailing bytes standing in for wherever the window happens to end).
    fn chain_window(body: &[u8]) -> alloc::vec::Vec<u8> {
        let mut w = body.to_vec();
        w.extend_from_slice(&crc16_umts_continue(0, body).to_le_bytes());
        w
    }

    extern crate alloc;

    #[test]
    fn finalize_patches_trailing_crc_slot_on_target_hit() {
        let mut codec = make_codec_with_last_reply();
        // Pre-patch: placeholder 0x0000 LE from `send_slot(Last {crc: 0 })`.
        assert_eq!(tx_trailing::<2>(&codec), [0x00, 0x00]);

        let window = chain_window(&[0x11, 0x22, 0x33]);
        let mut fl = FoldEngine::new();
        fl.start(0, window.len() as u32);
        fl.on_slice(&window, 0, &mut codec.tx);

        assert!(!fl.is_active(), "finalize should clear active");
        let patched = tx_trailing::<2>(&codec);
        assert_ne!(patched, [0x00, 0x00], "patch_crc should overwrite slot");

        // Independently compute the expected CRC the host would: chain-CRC
        // over ALL predecessor wire bytes (incl. the predecessor's own CRC
        // slot) + own_reply_bytes — the predicted-tail shortcut must be
        // byte-for-byte equivalent.
        let mut expected = SoftwareCrcUmts::new();
        expected.update(&window);
        expected.update(codec.own_reply_bytes());
        assert_eq!(u16::from_le_bytes(patched), expected.finalize());
    }

    #[test]
    fn extra_bytes_after_finalize_are_dropped() {
        let mut codec = make_codec_with_last_reply();
        let window = chain_window(&[0x11]);
        let mut fl = FoldEngine::new();
        fl.start(0, window.len() as u32);
        fl.on_slice(&window, 0, &mut codec.tx);
        assert!(!fl.is_active());

        // Already cleared; this is a no-op (doesn't fold, doesn't panic).
        fl.on_slice(&[0x33], window.len() as u32, &mut codec.tx);
        assert_eq!(fl.bytes_folded(), window.len() as u32);
    }

    #[test]
    fn cancel_clears_without_patching() {
        let mut codec = make_codec_with_last_reply();
        let trailing_before: [u8; 2] = tx_trailing(&codec);

        let mut fl = FoldEngine::new();
        fl.start(0, 4);
        fl.on_slice(&[0x11], 0, &mut codec.tx);
        fl.cancel();
        assert!(!fl.is_active());

        // Subsequent bytes silently drop.
        fl.on_slice(&[0x22], 1, &mut codec.tx);
        assert_eq!(fl.bytes_folded(), 1);

        // Trailing slot unchanged.
        assert_eq!(tx_trailing::<2>(&codec), trailing_before);
    }

    #[test]
    fn restart_resets_running_crc() {
        let mut codec = make_codec_with_last_reply();
        let mut fl = FoldEngine::new();

        // First cycle.
        let first_window = chain_window(&[0xFF]);
        fl.start(0, first_window.len() as u32);
        fl.on_slice(&first_window, 0, &mut codec.tx);
        let first_patch: [u8; 2] = tx_trailing(&codec);

        // Re-start; same start_cursor and predecessor_bytes but different byte.
        let second_window = chain_window(&[0x42]);
        fl.start(0, second_window.len() as u32);
        fl.on_slice(&second_window, 0, &mut codec.tx);
        let second_patch: [u8; 2] = tx_trailing(&codec);

        // Different inputs → different patches; proves the state reset on
        // re-start rather than continuing the prior run.
        assert_ne!(first_patch, second_patch);
    }
}
