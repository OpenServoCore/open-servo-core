//! Driver-side primitives shared across drivers — small enough that a
//! standalone home beats inlining into each consumer, large enough that
//! every consumer would reinvent the same math.
//!
//! ## Ring buffer
//! [`HwRing`] is a single-producer, single-consumer ring fed by hardware
//! (DMA): the producer side is [`HwRing::on_publish`], which bridges the
//! channel's NDTR-style ring position into the monotonic u16 seq space the
//! bookkeeping runs in (ring position = `seq % N`). The walking consumer
//! is the [`Reader`] view; producer-relative lookups go through
//! [`HwRing::recent`].
//!
//! ## Lap auto-resync
//! If the consumer falls more than `N` behind the producer, the in-window
//! data is gone. [`Reader::peek_slices`] detects this
//! (`write_seq - read_seq > N`) and bumps `read_seq` to `write_seq - N` so
//! the oldest still-valid slot is the next read. Downstream decoders /
//! classifiers self-heal at their next decision point (CRC mismatch →
//! Resync; large delta → GAP re-anchor).

/// Fixed-size circular buffer with producer + consumer state coupled
/// inside. Hardware producer (publishes via [`Self::on_publish`]); a
/// single walking consumer (the [`Reader`] view); producer-relative
/// lookups via [`Self::recent`].
pub struct HwRing<T, const N: usize> {
    data: [T; N],
    write_seq: u16,
    read_seq: u16,
    /// True once the consumer has ever been lapped. Gates
    /// [`Self::recent`]'s cold-start check — before any lap, only
    /// `offset < write_seq` slots are backed by a producer write.
    lapped: bool,
}

impl<T: Copy, const N: usize> HwRing<T, N> {
    /// Ring capacity as a `u16` — the const generic `N` lifted to a
    /// callable surface so consumers don't have to know the generic param
    /// at use sites.
    pub const LEN: u16 = N as u16;

    const _CHECK_N: () = assert!(
        N.is_power_of_two() && N > 0 && N <= u16::MAX as usize + 1,
        "HwRing N must be a power of two in (0, 1<<16]",
    );

    pub const fn new(init: T) -> Self {
        let _: () = Self::_CHECK_N;
        Self {
            data: [init; N],
            write_seq: 0,
            read_seq: 0,
            lapped: false,
        }
    }

    /// Address of the first storage slot. Hand this to a DMA channel's
    /// MAR — the struct's outer address is offset by the bookkeeping
    /// fields.
    pub fn as_ptr(&self) -> *const T {
        self.data.as_ptr()
    }

    /// Publish the producer's head from a DMA channel's NDTR readback
    /// (`remaining` slots left before the channel wraps). Computes the
    /// ring position as `LEN - remaining`, then the monotonic seq delta
    /// against the previously published head, and advances `write_seq`.
    /// Assumes the producer hasn't lapped since the last publish — true
    /// when the ring is sized to the worst-case burst per ISR period.
    pub fn on_publish(&mut self, remaining: u16) {
        let ring_pos = Self::LEN.wrapping_sub(remaining) as usize;
        let prev_pos = (self.write_seq as usize) % N;
        let delta = (ring_pos + N - prev_pos) % N;
        self.write_seq = self.write_seq.wrapping_add(delta as u16);
    }

    /// Producer-side count of slots that [`Self::recent`] will resolve.
    /// Saturates at `N` when the consumer has been lapped — the
    /// physical buffer still holds the most recent `N` writes, so the
    /// back-search bound is `N`, not the consumer-side `avail` (which
    /// the lap zeroes by design).
    pub fn recent_count(&self) -> u16 {
        let unread = self.write_seq.wrapping_sub(self.read_seq);
        if (unread as usize) > N {
            N as u16
        } else {
            unread
        }
    }

    /// Look up an element by offset back from the producer's head.
    /// `recent(0)` is the most-recently published value; `recent(k)` is
    /// `k` slots earlier. Opaque random-access op — pattern-match
    /// consumers (e.g. the DXL header back-search) sweep `recent(k)`
    /// instead of computing seqs.
    ///
    /// Lap-safe at the primitive level: for `0 ≤ offset < N` the slot at
    /// `write_seq − 1 − offset` is the value the producer wrote in its
    /// last `N` rounds, regardless of how far behind the read cursor has
    /// fallen. This matters when a heavy producer burst (e.g. a DMA
    /// flood past the ring depth) outpaces a paused consumer —
    /// `recent()` still resolves the most recent slots for, say, a Sync
    /// back-search after a heavy wire-noise burst.
    ///
    /// Returns `None` when:
    /// - `offset >= N` (outside ring capacity), or
    /// - the producer hasn't published `offset + 1` slots yet (cold
    ///   start — only enforced before the first lap).
    pub fn recent(&self, offset: u16) -> Option<&T> {
        if (offset as usize) >= N {
            return None;
        }
        // Cold-start gate: before the first lap, only offsets < write_seq
        // are backed by an actual producer write. Reader position is
        // irrelevant here — a consumer that's walked past most of the
        // ring hasn't invalidated the physical slots, which producer-side
        // back-search (e.g. Crc tail-signature read in the RX codec)
        // needs to reach.
        if !self.lapped && offset >= self.write_seq {
            return None;
        }
        let raw = self.write_seq.wrapping_sub(1).wrapping_sub(offset);
        Some(&self.data[(raw as usize) % N])
    }

    /// Open a walking consumer view. The view re-borrows the ring; only
    /// one [`Reader`] may exist at a time.
    pub fn reader(&mut self) -> Reader<'_, T, N> {
        Reader { ring: self }
    }
}

#[cfg(test)]
impl<T: Copy, const N: usize> HwRing<T, N> {
    /// Mirror an external producer's writes into the storage. Tests use
    /// this where DMA (or another driver) would in production. Does not
    /// touch `write_seq` — pair with `set_write_seq_for_test`.
    pub(crate) fn stage(&mut self, at: u16, src: &[T]) {
        for (i, &v) in src.iter().enumerate() {
            self.data[((at.wrapping_add(i as u16)) as usize) % N] = v;
        }
    }

    pub(crate) fn set_write_seq_for_test(&mut self, seq: u16) {
        self.write_seq = seq;
    }

    pub(crate) fn set_read_seq_for_test(&mut self, seq: u16) {
        self.read_seq = seq;
    }

    pub(crate) fn write_seq_for_test(&self) -> u16 {
        self.write_seq
    }

    pub(crate) fn read_seq_for_test(&self) -> u16 {
        self.read_seq
    }
}

/// Walking consumer view over a [`HwRing`]. Auto-resyncs `read_seq` on
/// lap detection.
pub struct Reader<'a, T, const N: usize> {
    ring: &'a mut HwRing<T, N>,
}

impl<'a, T: Copy, const N: usize> Reader<'a, T, N> {
    /// Unread count. Returns 0 when caught up; a lap (`> N` behind)
    /// surfaces only inside [`Self::peek_slices`], not here — calling
    /// `avail` is non-mutating.
    pub fn avail(&self) -> u16 {
        let ahead = self.ring.write_seq.wrapping_sub(self.ring.read_seq);
        if ahead as usize > N { 0 } else { ahead }
    }

    /// Unread region as `(front, back)` slices. `front` runs from the
    /// cursor up to either the producer head or the wrap boundary;
    /// `back` is the wrapped tail or `&[]`. `front.len() + back.len()`
    /// always equals [`Self::avail`]. Auto-resyncs on lap.
    pub fn peek_slices(&mut self) -> (&[T], &[T]) {
        self.resync_if_lapped();
        let avail = self.ring.write_seq.wrapping_sub(self.ring.read_seq) as usize;
        let pos = (self.ring.read_seq as usize) % N;
        let front_len = avail.min(N - pos);
        let back_len = avail - front_len;
        (
            &self.ring.data[pos..pos + front_len],
            &self.ring.data[..back_len],
        )
    }

    /// Advance the consumer head by `n` (wrapping in u16).
    pub fn advance(&mut self, n: u16) {
        self.ring.read_seq = self.ring.read_seq.wrapping_add(n);
    }

    /// Force `read_seq` to `write_seq − N` if the producer has lapped
    /// the consumer (`write_seq − read_seq > N`). Otherwise a no-op.
    /// Called internally by [`Self::peek_slices`]; also `pub` so
    /// producer-side primitives (e.g. the edge-ring back-search after a
    /// heavy wire-noise burst) can establish a known cursor before
    /// computing relative advances — without this, a post-lap
    /// `advance(n)` lands at `pre_lap_read_seq + n` and a subsequent
    /// `peek_slices` auto-resyncs past the caller's intended target.
    pub fn resync_if_lapped(&mut self) {
        let behind = self.ring.write_seq.wrapping_sub(self.ring.read_seq);
        if behind as usize > N {
            self.ring.read_seq = self.ring.write_seq.wrapping_sub(N as u16);
            self.ring.lapped = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type HwBuf = HwRing<u8, 8>;

    #[test]
    fn as_ptr_points_at_first_storage_slot() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22]);
        // SAFETY: reading the first and last slot through the address we'd
        // hand to DMA. Single-threaded test, no concurrent writer.
        unsafe {
            assert_eq!(*b.as_ptr(), 0xAA);
            assert_eq!(*b.as_ptr().add(7), 0x22);
        }
    }

    #[test]
    fn on_publish_advances_write_seq_monotonically() {
        // N=8; on_publish takes `remaining` slots. ring_pos = N - remaining.
        let mut b: HwBuf = HwRing::new(0);
        // Producer at slot 0 → publish slot 5 (remaining=3): delta 5.
        b.on_publish(3);
        assert_eq!(b.write_seq_for_test(), 5);
        // Producer at slot 5 → publish slot 2 (wrapped, remaining=6):
        // delta = (2+8-5)%8 = 5.
        b.on_publish(6);
        assert_eq!(b.write_seq_for_test(), 10);
        // Re-publishing the same remaining is a no-op delta.
        b.on_publish(6);
        assert_eq!(b.write_seq_for_test(), 10);
    }

    #[test]
    fn on_publish_keeps_seq_monotonic_across_u16_wrap() {
        let mut b: HwBuf = HwRing::new(0);
        b.set_write_seq_for_test(u16::MAX - 2);
        // ring slot = 65533 % 8 = 5. Roll past slot 5 to slot 1
        // (remaining = 7) → delta 4. u16::MAX - 2 + 4 wraps to 1.
        b.on_publish(7);
        assert_eq!(b.write_seq_for_test(), 1);
    }

    #[test]
    fn fresh_ring_has_zero_avail() {
        let mut b: HwBuf = HwRing::new(0);
        assert_eq!(b.read_seq_for_test(), 0);
        assert_eq!(b.write_seq_for_test(), 0);
        assert_eq!(b.reader().avail(), 0);
    }

    #[test]
    fn avail_reflects_producer_head() {
        let mut b: HwBuf = HwRing::new(0);
        b.set_write_seq_for_test(5);
        assert_eq!(b.reader().avail(), 5);
    }

    #[test]
    fn avail_handles_u16_wrap_in_subtraction() {
        // Producer's seq wrapped past u16::MAX; consumer still behind.
        let mut b: HwBuf = HwRing::new(0);
        b.set_write_seq_for_test(3);
        b.set_read_seq_for_test(u16::MAX);
        assert_eq!(b.reader().avail(), 4);
    }

    #[test]
    fn recent_returns_most_recently_published() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[10, 20, 30, 40, 50]);
        b.set_write_seq_for_test(5);
        assert_eq!(b.recent(0), Some(&50));
        assert_eq!(b.recent(1), Some(&40));
        assert_eq!(b.recent(4), Some(&10));
    }

    #[test]
    fn recent_returns_none_past_published_head() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[10, 20, 30]);
        b.set_write_seq_for_test(3);
        // Only 3 slots published; offset 3+ reaches past head.
        assert_eq!(b.recent(3), None);
    }

    #[test]
    fn recent_count_saturates_at_n_after_lap() {
        let mut b: HwBuf = HwRing::new(0);
        b.set_write_seq_for_test(5);
        // Cold start: fewer writes than capacity → unsaturated.
        assert_eq!(b.recent_count(), 5);
        b.set_write_seq_for_test(8);
        // Exactly N writes → ring full, still unsaturated by lap math.
        assert_eq!(b.recent_count(), 8);
        b.set_write_seq_for_test(40);
        // Producer raced past the reader by `40 - 0 = 40 > N` → ring
        // wrapped; recent_count saturates so the consumer (e.g. anchor
        // back-search) sees the lap-safe count instead of `Reader::
        // avail`'s zero.
        assert_eq!(b.recent_count(), 8);
    }

    #[test]
    fn recent_stays_lap_safe_when_consumer_falls_behind() {
        // N=8 but `unread = 20 > N` → consumer lapped multiple times.
        // The last N writes overwrote exactly the same physical slots,
        // so `recent(0..N-1)` must still resolve to the producer's
        // most-recent values rather than parroting Reader::avail's
        // "lapped → 0" convention. This is the load-bearing path the
        // edge-ring back-search relies on after a long wire-noise burst.
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        b.set_write_seq_for_test(20);
        // Most-recent write landed at slot `(20 - 1) % 8 = 3` → value 4.
        assert_eq!(b.recent(0), Some(&4));
        // Walking back wraps through the physical buffer.
        assert_eq!(b.recent(1), Some(&3));
        assert_eq!(b.recent(7), Some(&5));
        // offset >= N stays None — outside ring capacity, can't resolve.
        assert_eq!(b.recent(8), None);
    }

    #[test]
    fn recent_resolves_most_recent_n_after_heavy_burst() {
        // Mirrors the noise-burst failure shape: producer writes more
        // than N items past where the consumer last sat, so the
        // physical buffer holds purely producer-burst data. The last N
        // writes must surface through `recent()` regardless of the read
        // cursor — DMA's circular semantics guarantee those slots are
        // valid, and the back-search depends on it.
        let mut b: HwBuf = HwRing::new(0);
        // First fill: 8 zeroes (initial). Then 4 fresh values overwrite
        // slots 0..3. write_seq advances by 12; read_seq stayed at 0
        // (the scenario where the walker paused after Crc(Bad)).
        b.stage(0, &[99, 99, 99, 99, 5, 6, 7, 8]);
        b.set_write_seq_for_test(12);
        // recent(0) = slot at `(12 - 1) % 8 = 3` = 99 (most recent
        // wrap's 4th write). recent(1) = slot 2 = 99. recent(4) walks
        // back to the previous wrap's slot 7 = 8.
        assert_eq!(b.recent(0), Some(&99));
        assert_eq!(b.recent(3), Some(&99));
        assert_eq!(b.recent(4), Some(&8));
        assert_eq!(b.recent(7), Some(&5));
    }

    #[test]
    fn recent_bounded_by_unread() {
        // unread=8, N=8 → offsets 0..7 addressable, offset 8 past head.
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        b.set_write_seq_for_test(8);
        assert_eq!(b.recent(7), Some(&1));
        assert_eq!(b.recent(8), None);
    }

    #[test]
    fn recent_reaches_past_consumer_cursor_before_first_lap() {
        // Producer wrote 6 items, consumer walked forward to slot 5 (so
        // unread=1). Cold-start gate must key off `write_seq`, not
        // `unread`, so `recent(0..5)` still surfaces the physical writes
        // — the codec's Crc-time tail-signature read hits this shape
        // when RX HT drains most of the ring before the IDLE poll runs
        // the last byte.
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[10, 20, 30, 40, 50, 60, 0, 0]);
        b.set_write_seq_for_test(6);
        b.set_read_seq_for_test(5);
        assert_eq!(b.recent(0), Some(&60));
        assert_eq!(b.recent(3), Some(&30));
        assert_eq!(b.recent(5), Some(&10));
        // offset == write_seq (nothing written that far back yet).
        assert_eq!(b.recent(6), None);
    }

    #[test]
    fn recent_walks_correctly_across_u16_wrap() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        b.set_write_seq_for_test(2);
        b.set_read_seq_for_test(u16::MAX - 5);
        // write_seq=2 wrapped past u16::MAX; the cold-start gate keys off
        // the raw write_seq, so only offsets 0..1 resolve here — the
        // wrap-adjacent slots are still addressed correctly.
        assert_eq!(b.recent(0), Some(&2));
        assert_eq!(b.recent(1), Some(&1));
    }

    #[test]
    fn peek_slices_front_only_when_no_wrap() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        b.set_write_seq_for_test(3);
        let mut r = b.reader();
        let (front, back) = r.peek_slices();
        assert_eq!(front, &[1, 2, 3]);
        assert!(back.is_empty());
    }

    #[test]
    fn peek_slices_splits_across_wrap() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(6, &[70, 80, 10, 20, 30]);
        b.set_write_seq_for_test(11);
        b.set_read_seq_for_test(6);
        let mut r = b.reader();
        let (front, back) = r.peek_slices();
        assert_eq!(front, &[70, 80]);
        assert_eq!(back, &[10, 20, 30]);
    }

    #[test]
    fn peek_slices_lens_sum_to_avail() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        b.set_write_seq_for_test(10);
        b.set_read_seq_for_test(6);
        let mut r = b.reader();
        let avail = r.avail();
        let (front, back) = r.peek_slices();
        assert_eq!(front.len() + back.len(), avail as usize);
    }

    #[test]
    fn advance_through_reader_moves_cursor() {
        let mut b: HwBuf = HwRing::new(0);
        b.set_write_seq_for_test(8);
        {
            let mut r = b.reader();
            assert_eq!(r.avail(), 8);
            r.advance(3);
            assert_eq!(r.avail(), 5);
        }
        assert_eq!(b.read_seq_for_test(), 3);
    }

    #[test]
    fn advance_through_reader_wraps_in_u16_space() {
        let mut b: HwBuf = HwRing::new(0);
        b.set_write_seq_for_test(2);
        b.set_read_seq_for_test(u16::MAX - 1);
        {
            let mut r = b.reader();
            r.advance(4); // (u16::MAX - 1) + 4 wraps to 2
            assert_eq!(r.avail(), 0);
        }
        assert_eq!(b.read_seq_for_test(), 2);
    }

    #[test]
    fn peek_slices_auto_resyncs_when_consumer_lapped() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        // Producer 10 ahead of consumer = 2 past full lap (N=8).
        b.set_write_seq_for_test(10);
        b.set_read_seq_for_test(0);

        let mut r = b.reader();
        // peek_slices detects the lap, jumps read_seq to write_seq - N = 2,
        // and starts the front slice at the slot for seq 2 (value 3).
        let (front, _) = r.peek_slices();
        assert_eq!(front.first(), Some(&3));
        assert_eq!(b.read_seq_for_test(), 2);
    }

    #[test]
    fn supports_non_byte_element_types() {
        let mut b: HwRing<u16, 4> = HwRing::new(0);
        b.stage(0, &[0x1111, 0x2222, 0x3333, 0x4444]);
        b.set_write_seq_for_test(4);
        let mut r = b.reader();
        let (front, back) = r.peek_slices();
        assert_eq!(front, &[0x1111, 0x2222, 0x3333, 0x4444]);
        assert!(back.is_empty());
    }
}
