//! Driver-side primitives shared across drivers — small enough that a
//! standalone home beats inlining into each consumer, large enough that
//! every consumer would reinvent the same math.

/// Fixed-size circular buffer fed by an external producer — DMA hardware,
/// another ISR, anything writing outside this code's control flow. Distinct
/// from a software ring queue (SPSC `push`/`pop`): there's no producer
/// endpoint and no commit step. The producer publishes its head into
/// `write_seq` (e.g. derived from NDTR readback); consumers open a
/// [`RingView`] starting at their own `read_seq` and read up to the
/// producer's head.
///
/// Two address spaces coexist in every ring:
/// - **Sequence number** — monotonic `u16`, wraps at 65536. Producer and
///   consumer track positions here; `write_seq.wrapping_sub(read_seq)` is
///   the count outstanding.
/// - **Ring position** — physical slot `[0, N)`, derived via `seq % N`.
///
/// Pow-2 `N` keeps `seq % N` collapsing to AND on chips without a hardware
/// modulo (e.g. V006's riscv32ec_zmmul) and matches the chip-side NDTR
/// head math.
pub struct DmaBuffer<T, const N: usize> {
    data: [T; N],
    write_seq: u16,
}

impl<T: Copy, const N: usize> DmaBuffer<T, N> {
    const _CHECK_N: () = assert!(
        N.is_power_of_two() && N > 0 && N <= u16::MAX as usize + 1,
        "DmaBuffer N must be a power of two in (0, 1<<16]",
    );

    pub const fn new(init: T) -> Self {
        let _: () = Self::_CHECK_N;
        Self {
            data: [init; N],
            write_seq: 0,
        }
    }

    /// Address of the first storage slot. Hand this to the DMA channel's
    /// MAR — the struct's outer address is not the storage address (the
    /// `write_seq` field also sits inside).
    pub fn as_ptr(&self) -> *const T {
        self.data.as_ptr()
    }

    /// Producer's published head — sequence number of the next slot the
    /// producer will write.
    pub fn write_seq(&self) -> u16 {
        self.write_seq
    }

    /// Publish a new producer head. Called from the side that knows where
    /// the producer is (NDTR readback for DMA-fed rings, software writer
    /// for ISR-fed rings).
    pub fn set_write_seq(&mut self, seq: u16) {
        self.write_seq = seq;
    }

    /// Open a [`RingView`] starting at `read_seq`. The view reads
    /// `write_seq` from this buffer to bound consumption.
    pub fn view(&self, read_seq: u16) -> RingView<'_, T, N> {
        RingView {
            buf: self,
            read_seq,
        }
    }

    /// Slot at the ring position implied by `seq`.
    #[inline]
    pub fn at(&self, seq: u16) -> &T {
        &self.data[(seq as usize) % N]
    }

    /// Longest contiguous slice starting at `seq`, bounded by `max` and the
    /// wrap boundary. The caller advances `seq` by the bytes actually
    /// consumed (the slice's length is an upper bound, not a contract — a
    /// decoder may consume fewer).
    #[inline]
    pub fn slice_from(&self, seq: u16, max: u16) -> &[T] {
        let pos = (seq as usize) % N;
        let until_wrap = N - pos;
        let len = (max as usize).min(until_wrap);
        &self.data[pos..pos + len]
    }
}

#[cfg(test)]
impl<T: Copy, const N: usize> DmaBuffer<T, N> {
    /// Mirror an external producer's writes. Tests use this where DMA
    /// would in production; production code has no business writing to a
    /// DMA-fed buffer from Rust.
    pub(crate) fn stage(&mut self, at: u16, src: &[T]) {
        for (i, &v) in src.iter().enumerate() {
            self.data[((at.wrapping_add(i as u16)) as usize) % N] = v;
        }
    }
}

/// Read-only cursor-bearing handle on a [`DmaBuffer`]. One per consumer —
/// the buffer's `write_seq` bounds the read; the view's `read_seq` tracks
/// per-consumer progress. Sync the view's cursor back to the consumer's
/// persistent state on exit.
pub struct RingView<'a, T, const N: usize> {
    buf: &'a DmaBuffer<T, N>,
    read_seq: u16,
}

impl<'a, T: Copy, const N: usize> RingView<'a, T, N> {
    /// This consumer's current sequence number.
    pub fn read_seq(&self) -> u16 {
        self.read_seq
    }

    /// Outstanding count — slots published past this consumer's cursor.
    /// `write_seq − read_seq`, wrapping in u16.
    pub fn avail(&self) -> u16 {
        self.buf.write_seq.wrapping_sub(self.read_seq)
    }

    /// Longest contiguous slice ahead of the cursor, bounded by `max`,
    /// `avail()`, and the wrap boundary. Returns `&[]` when the consumer
    /// is caught up to the producer.
    pub fn slice_ahead(&self, max: u16) -> &[T] {
        let pos = (self.read_seq as usize) % N;
        let until_wrap = N - pos;
        let len = (max as usize).min(self.avail() as usize).min(until_wrap);
        &self.buf.data[pos..pos + len]
    }

    /// Advance the cursor by `n` (wrapping). Caller passes whatever a
    /// downstream consumer actually consumed.
    pub fn advance(&mut self, n: u16) {
        self.read_seq = self.read_seq.wrapping_add(n);
    }

    /// Slot at the ring position implied by `seq` — absolute access,
    /// independent of the cursor. For callers that already have a seq.
    pub fn at(&self, seq: u16) -> &T {
        &self.buf.data[(seq as usize) % N]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type Buf = DmaBuffer<u8, 8>;

    #[test]
    fn at_wraps_seq_to_ring_position() {
        let mut b = Buf::new(0);
        b.stage(0, &[10, 20, 30, 40, 50, 60, 70, 80]);
        // seq 0..8 maps to slot 0..8
        assert_eq!(*b.at(0), 10);
        assert_eq!(*b.at(7), 80);
        // seq 8 wraps to slot 0; seq 15 wraps to slot 7
        assert_eq!(*b.at(8), 10);
        assert_eq!(*b.at(15), 80);
        // seq high in u16 space still wraps cleanly
        assert_eq!(*b.at(u16::MAX), *b.at(7));
    }

    #[test]
    fn slice_from_returns_pre_wrap_chunk() {
        let mut b = Buf::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        // From slot 0, max 4 → first 4 bytes
        assert_eq!(b.slice_from(0, 4), &[1, 2, 3, 4]);
        // From slot 2, max 8 → bounded by wrap to 6 bytes
        assert_eq!(b.slice_from(2, 8), &[3, 4, 5, 6, 7, 8]);
        // From slot 7, max 4 → bounded by wrap to 1 byte
        assert_eq!(b.slice_from(7, 4), &[8]);
    }

    #[test]
    fn slice_from_at_wrap_boundary_returns_post_wrap_chunk() {
        let mut b = Buf::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        // seq 8 maps to slot 0; max 3 → first 3 bytes
        assert_eq!(b.slice_from(8, 3), &[1, 2, 3]);
        // seq 10 maps to slot 2; max 2 → slots 2..4
        assert_eq!(b.slice_from(10, 2), &[3, 4]);
    }

    #[test]
    fn slice_from_with_zero_max_returns_empty() {
        let b = Buf::new(0);
        assert_eq!(b.slice_from(0, 0).len(), 0);
        assert_eq!(b.slice_from(3, 0).len(), 0);
    }

    #[test]
    fn slice_from_does_not_exceed_array_length() {
        let mut b = Buf::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        // max larger than N still bounded by wrap remainder
        assert_eq!(b.slice_from(0, 100).len(), 8);
        assert_eq!(b.slice_from(5, 100).len(), 3);
    }

    #[test]
    fn stage_wraps_writes_across_ring_boundary() {
        let mut b = Buf::new(0);
        // Write 4 bytes starting at seq 6 — straddles wrap.
        b.stage(6, &[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(*b.at(6), 0xAA);
        assert_eq!(*b.at(7), 0xBB);
        assert_eq!(*b.at(8), 0xCC); // wrapped to slot 0
        assert_eq!(*b.at(9), 0xDD); // wrapped to slot 1
    }

    #[test]
    fn as_ptr_points_at_first_storage_slot() {
        let mut b = Buf::new(0);
        b.stage(0, &[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22]);
        // SAFETY: reading the first and last slot through the address we'd
        // hand to DMA. Single-threaded test, no concurrent writer.
        unsafe {
            assert_eq!(*b.as_ptr(), 0xAA);
            assert_eq!(*b.as_ptr().add(7), 0x22);
        }
    }

    #[test]
    fn supports_non_byte_element_types() {
        let mut b: DmaBuffer<u16, 4> = DmaBuffer::new(0);
        b.stage(0, &[0x1111, 0x2222, 0x3333, 0x4444]);
        assert_eq!(*b.at(2), 0x3333);
        assert_eq!(b.slice_from(0, 4), &[0x1111, 0x2222, 0x3333, 0x4444]);
    }

    #[test]
    fn write_seq_round_trips() {
        let mut b = Buf::new(0);
        assert_eq!(b.write_seq(), 0);
        b.set_write_seq(42);
        assert_eq!(b.write_seq(), 42);
        b.set_write_seq(u16::MAX);
        assert_eq!(b.write_seq(), u16::MAX);
    }

    #[test]
    fn view_avail_is_zero_when_caught_up() {
        let b = Buf::new(0);
        assert_eq!(b.view(0).avail(), 0);
        // Cursor matches producer at an arbitrary seq.
        let mut b = b;
        b.set_write_seq(123);
        assert_eq!(b.view(123).avail(), 0);
    }

    #[test]
    fn view_avail_reflects_producer_head() {
        let mut b = Buf::new(0);
        b.set_write_seq(5);
        assert_eq!(b.view(0).avail(), 5);
    }

    #[test]
    fn view_avail_wraps_in_u16_space() {
        // Producer's seq has wrapped past u16::MAX; consumer is still behind
        // the wrap. `write_seq - read_seq` in wrapping u16 still gives the
        // correct outstanding count.
        let mut b = Buf::new(0);
        b.set_write_seq(3); // imagine this wrapped from 65539
        let view = b.view(u16::MAX); // sat at u16::MAX before producer wrap
        assert_eq!(view.avail(), 4);
    }

    #[test]
    fn slice_ahead_bounded_by_avail() {
        let mut b = Buf::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        b.set_write_seq(3);
        // avail=3, max=100, wrap=8 → 3 bytes (avail wins)
        assert_eq!(b.view(0).slice_ahead(100), &[1, 2, 3]);
    }

    #[test]
    fn slice_ahead_bounded_by_max() {
        let mut b = Buf::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        b.set_write_seq(8);
        // avail=8, max=2, wrap=8 → 2 bytes (max wins)
        assert_eq!(b.view(0).slice_ahead(2), &[1, 2]);
    }

    #[test]
    fn slice_ahead_bounded_by_wrap() {
        let mut b = Buf::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        b.set_write_seq(10);
        // From seq 6: avail=4, max=100, wrap-remainder=2 → 2 bytes (wrap wins)
        assert_eq!(b.view(6).slice_ahead(100), &[7, 8]);
    }

    #[test]
    fn slice_ahead_empty_when_caught_up() {
        let b = Buf::new(0);
        assert_eq!(b.view(0).slice_ahead(100), &[]);
    }

    #[test]
    fn advance_moves_read_seq_and_shrinks_avail() {
        let mut b = Buf::new(0);
        b.set_write_seq(10);
        let mut view = b.view(0);
        assert_eq!(view.avail(), 10);
        view.advance(3);
        assert_eq!(view.read_seq(), 3);
        assert_eq!(view.avail(), 7);
    }

    #[test]
    fn advance_wraps_read_seq() {
        let mut b = Buf::new(0);
        b.set_write_seq(2); // imagine this wrapped from 65538
        let mut view = b.view(u16::MAX - 1);
        view.advance(4); // (u16::MAX - 1) + 4 wraps to 2
        assert_eq!(view.read_seq(), 2);
        assert_eq!(view.avail(), 0);
    }

    #[test]
    fn view_at_matches_buf_at_for_arbitrary_seq() {
        let mut b = Buf::new(0);
        b.stage(0, &[10, 20, 30, 40, 50, 60, 70, 80]);
        let view = b.view(0);
        for seq in [0u16, 3, 7, 8, 15, 100, u16::MAX] {
            assert_eq!(view.at(seq), b.at(seq));
        }
    }
}
