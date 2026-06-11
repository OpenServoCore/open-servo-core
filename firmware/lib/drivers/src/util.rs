//! Driver-side primitives shared across drivers — small enough that a
//! standalone home beats inlining into each consumer, large enough that
//! every consumer would reinvent the same math.

/// Fixed-size circular buffer fed by an external producer — DMA hardware,
/// another ISR, anything writing outside this code's control flow. Distinct
/// from a software ring queue (SPSC `push`/`pop`): there's no producer
/// endpoint and no commit step. The consumer reads through a wrapping
/// sequence-number cursor; the producer publishes its head independently
/// (e.g. NDTR readback) and the consumer trusts it.
///
/// Two address spaces coexist in every ring:
/// - **Sequence number** — monotonic `u16`, wraps at 65536. Producer and
///   consumer track positions here; `head.wrapping_sub(tail)` is the byte
///   count outstanding.
/// - **Ring position** — physical slot `[0, N)`, derived via `seq % N`.
///
/// `#[repr(transparent)]` keeps `as_ptr()` returning the array's start
/// address for DMA peripheral setup; pow-2 `N` keeps `seq % N` collapsing
/// to AND and matches the chip-side NDTR head math.
#[repr(transparent)]
pub struct DmaBuffer<T, const N: usize> {
    data: [T; N],
}

impl<T: Copy, const N: usize> DmaBuffer<T, N> {
    const _CHECK_N: () = assert!(
        N.is_power_of_two() && N > 0 && N <= u16::MAX as usize + 1,
        "DmaBuffer N must be a power of two in (0, 1<<16]",
    );

    pub const fn new(init: T) -> Self {
        let _: () = Self::_CHECK_N;
        Self { data: [init; N] }
    }

    /// Pointer to the underlying storage. For DMA peripheral setup.
    pub fn as_ptr(&self) -> *const T {
        self.data.as_ptr()
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
    fn as_ptr_matches_first_byte_address() {
        // `repr(transparent)` guarantees the buffer's address equals its
        // first slot's address, which is what DMA setup wants.
        let b = Buf::new(0);
        let buf_addr = &b as *const _ as usize;
        assert_eq!(buf_addr, b.as_ptr() as usize);
    }

    #[test]
    fn supports_non_byte_element_types() {
        let mut b: DmaBuffer<u16, 4> = DmaBuffer::new(0);
        b.stage(0, &[0x1111, 0x2222, 0x3333, 0x4444]);
        assert_eq!(*b.at(2), 0x3333);
        assert_eq!(b.slice_from(0, 4), &[0x1111, 0x2222, 0x3333, 0x4444]);
    }
}
