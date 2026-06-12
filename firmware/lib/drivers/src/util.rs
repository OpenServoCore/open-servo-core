//! Driver-side primitives shared across drivers — small enough that a
//! standalone home beats inlining into each consumer, large enough that
//! every consumer would reinvent the same math.
//!
//! ## Ring buffer
//! Single-producer, single-consumer (walking) ring with a typed producer
//! kind ([`Hw`] / [`Sw`]) that gates the producer-side API:
//! - [`HwRing`] (DMA-fed): exposes [`Ring::on_publish`] for NDTR-style
//!   ring-position publication; no [`Writer`] view.
//! - [`SwRing`] (software-fed): exposes [`Ring::writer`] for in-software
//!   push; no `on_publish`.
//!
//! Consumer surface ([`Reader`] + [`Ring::at`]) is shared. Lookup-only
//! consumers (drift, fire) never carry a [`Reader`] — they just call
//! `at(seq)`. Walking consumers (parser, classifier) take a reader.
//!
//! ## Seq vs ring position
//! - **Seq** ([`Seq<T, N>`]) — monotonic identity of an element in the
//!   infinite stream. Stored as `u16` (wraps at 65536). Two seqs in the
//!   same space subtracted give a count; absolute values are meaningless.
//! - **Ring position** — slot index `[0, N)`, derived as `seq % N`.
//!
//! Hardware (DMA NDTR) lives in position-world; software bookkeeping lives
//! in seq-world. [`Ring::on_publish`] is the one bridge — it takes a ring
//! position and advances `write_seq` monotonically.
//!
//! Two rings whose seq spaces are *physically coupled* (e.g. `rx_buf` /
//! `byte_ts` per doc §8.3) can convert via `From` impls; the conversion
//! is value-preserving (just retags the type).
//!
//! ## Lap auto-resync
//! If a consumer falls more than `N` behind the producer, the in-window
//! data is gone. [`Reader::peek`] / [`Reader::peek_slices`] detect this
//! (`write_seq - read_seq > N`), bump `read_seq` to `write_seq - N` so
//! the oldest still-valid slot is the next read, and increment
//! [`Ring::lap_count`] as a telemetry signal. Downstream decoders /
//! classifiers self-heal at their next decision point (CRC mismatch →
//! Resync; large delta → GAP re-anchor).

use core::marker::PhantomData;

/// Producer-kind marker. See [`HwRing`] / [`SwRing`].
mod sealed {
    pub trait Sealed {}
    impl Sealed for super::Hw {}
    impl Sealed for super::Sw {}
}

/// Marker: hardware producer (DMA). Gates [`Ring::on_publish`]; no
/// software [`Writer`] view.
pub struct Hw;
/// Marker: software producer. Gates [`Ring::writer`]; no `on_publish`.
pub struct Sw;

/// Sealed trait identifying ring producer kind. Implemented only by
/// [`Hw`] / [`Sw`].
pub trait Producer: sealed::Sealed {}
impl Producer for Hw {}
impl Producer for Sw {}

/// Opaque monotonic sequence identity for a ring element. `Seq<T, N>` is
/// bound to its ring's element type and size; external code holds it as
/// a token (no arithmetic) and hands it back to [`Ring::at`].
///
/// Hardware-coupled seq spaces (doc §8.3) can convert via `From` —
/// value-preserving, just retags the type.
pub struct Seq<T, const N: usize> {
    raw: u16,
    _phantom: PhantomData<fn() -> T>,
}

impl<T, const N: usize> Seq<T, N> {
    /// Construct from a raw `u16`. `pub(crate)` so only this module's
    /// rings and explicit `From` impls mint seqs — external code holds
    /// what it gets, doesn't fabricate.
    pub(crate) const fn from_raw(raw: u16) -> Self {
        Self {
            raw,
            _phantom: PhantomData,
        }
    }

    /// Unwrap to the raw `u16`. Diagnostic and cross-component
    /// comparison only — math on the value belongs in [`Ring`].
    pub const fn raw(&self) -> u16 {
        self.raw
    }
}

impl<T, const N: usize> Clone for Seq<T, N> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T, const N: usize> Copy for Seq<T, N> {}

impl<T, const N: usize> PartialEq for Seq<T, N> {
    fn eq(&self, other: &Self) -> bool {
        self.raw == other.raw
    }
}
impl<T, const N: usize> Eq for Seq<T, N> {}

impl<T, const N: usize> core::fmt::Debug for Seq<T, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("Seq").field(&self.raw).finish()
    }
}

/// `rx_buf` (Hw, u8) shares a seq space with `byte_ts` (Sw, u16) per
/// doc §8.3 — one UART byte → one start-bit edge → one BT entry, so the
/// nth byte and the nth BT entry have the same seq. The conversion just
/// retags the type; the raw value is preserved.
impl<const RX: usize, const BT: usize> From<Seq<u8, RX>> for Seq<u16, BT> {
    fn from(s: Seq<u8, RX>) -> Self {
        Seq::from_raw(s.raw)
    }
}

/// Fixed-size circular buffer with producer + consumer state coupled
/// inside. Single producer (kind determined by `P`); a single walking
/// consumer (the [`Reader`] view); arbitrary stateless lookup consumers
/// ([`Ring::at`]).
pub struct Ring<P: Producer, T, const N: usize> {
    data: [T; N],
    write_seq: u16,
    read_seq: u16,
    lap_count: u32,
    _producer: PhantomData<fn() -> P>,
}

/// DMA-fed ring. Producer surface: [`Ring::on_publish`].
pub type HwRing<T, const N: usize> = Ring<Hw, T, N>;
/// Software-fed ring. Producer surface: [`Ring::writer`].
pub type SwRing<T, const N: usize> = Ring<Sw, T, N>;

impl<P: Producer, T: Copy, const N: usize> Ring<P, T, N> {
    /// Ring capacity as a `u16` — the const generic `N` lifted to a
    /// callable surface so consumers don't have to know the generic param
    /// at use sites.
    pub const LEN: u16 = N as u16;

    const _CHECK_N: () = assert!(
        N.is_power_of_two() && N > 0 && N <= u16::MAX as usize + 1,
        "Ring N must be a power of two in (0, 1<<16]",
    );

    pub const fn new(init: T) -> Self {
        let _: () = Self::_CHECK_N;
        Self {
            data: [init; N],
            write_seq: 0,
            read_seq: 0,
            lap_count: 0,
            _producer: PhantomData,
        }
    }

    /// Address of the first storage slot. Hand this to a DMA channel's
    /// MAR — the struct's outer address is offset by the bookkeeping
    /// fields.
    pub fn as_ptr(&self) -> *const T {
        self.data.as_ptr()
    }

    /// Consumer's read head as an opaque [`Seq`].
    pub fn read_seq(&self) -> Seq<T, N> {
        Seq::from_raw(self.read_seq)
    }

    /// Producer's published head as an opaque [`Seq`].
    pub fn write_seq(&self) -> Seq<T, N> {
        Seq::from_raw(self.write_seq)
    }

    /// Monotonic count of lap events observed by [`Reader::peek`] /
    /// [`Reader::peek_slices`]. Bench telemetry signal — should be 0 at
    /// correct ring sizing.
    pub fn lap_count(&self) -> u32 {
        self.lap_count
    }

    /// Look up an element by [`Seq`]. `None` if `seq` is not yet
    /// published (at or past write head) or has lapped out of the ring
    /// window. Stateless — does not advance the read cursor.
    pub fn at(&self, seq: Seq<T, N>) -> Option<&T> {
        let ahead = self.write_seq.wrapping_sub(seq.raw);
        if ahead == 0 || ahead as usize > N {
            return None;
        }
        Some(&self.data[(seq.raw as usize) % N])
    }

    /// Open a walking consumer view. The view re-borrows the ring; only
    /// one [`Reader`] may exist at a time.
    pub fn reader(&mut self) -> Reader<'_, P, T, N> {
        Reader { ring: self }
    }
}

impl<T: Copy, const N: usize> HwRing<T, N> {
    /// Publish the producer's head from a ring position in `[0, N)`
    /// (typically `N - NDTR` for a DMA channel). Computes the monotonic
    /// seq delta against the previously published head and advances
    /// `write_seq`. Assumes the producer hasn't lapped since the last
    /// publish — true when the ring is sized to the worst-case burst
    /// per ISR period.
    pub fn on_publish(&mut self, ring_pos: u16) {
        let prev_pos = (self.write_seq as usize) % N;
        let delta = ((ring_pos as usize) + N - prev_pos) % N;
        self.write_seq = self.write_seq.wrapping_add(delta as u16);
    }

    /// Publish from a DMA channel's NDTR readback (`remaining` slots left
    /// before the channel wraps). Computes the ring position as
    /// `LEN - remaining` and forwards to [`Self::on_publish`].
    pub fn on_publish_remaining(&mut self, remaining: u16) {
        self.on_publish(Self::LEN.wrapping_sub(remaining));
    }
}

impl<T: Copy, const N: usize> SwRing<T, N> {
    /// Open a software producer view. The view re-borrows the ring;
    /// only one [`Writer`] may exist at a time.
    pub fn writer(&mut self) -> Writer<'_, T, N> {
        Writer { ring: self }
    }
}

#[cfg(test)]
impl<P: Producer, T: Copy, const N: usize> Ring<P, T, N> {
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
}

/// Walking consumer view over a [`Ring`]. Auto-resyncs `read_seq` on
/// lap detection (bumps [`Ring::lap_count`]).
pub struct Reader<'a, P: Producer, T, const N: usize> {
    ring: &'a mut Ring<P, T, N>,
}

impl<'a, P: Producer, T: Copy, const N: usize> Reader<'a, P, T, N> {
    /// Unread count. Returns 0 when caught up; a lap (`> N` behind)
    /// surfaces only inside [`Self::peek`] / [`Self::peek_slices`], not
    /// here — calling `avail` is non-mutating.
    pub fn avail(&self) -> u16 {
        let ahead = self.ring.write_seq.wrapping_sub(self.ring.read_seq);
        if ahead as usize > N { 0 } else { ahead }
    }

    /// Element at the cursor; `None` when caught up. Auto-resyncs on
    /// lap (jumps `read_seq` to `write_seq - N`, bumps `lap_count`),
    /// then returns the oldest still-valid slot.
    pub fn peek(&mut self) -> Option<&T> {
        self.resync_if_lapped();
        if self.ring.write_seq == self.ring.read_seq {
            return None;
        }
        let pos = (self.ring.read_seq as usize) % N;
        Some(&self.ring.data[pos])
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

    fn resync_if_lapped(&mut self) {
        let behind = self.ring.write_seq.wrapping_sub(self.ring.read_seq);
        if behind as usize > N {
            self.ring.read_seq = self.ring.write_seq.wrapping_sub(N as u16);
            self.ring.lap_count = self.ring.lap_count.wrapping_add(1);
        }
    }
}

/// Software producer view over a [`SwRing`]. `push` writes the next
/// slot and advances `write_seq`.
pub struct Writer<'a, T, const N: usize> {
    ring: &'a mut SwRing<T, N>,
}

impl<'a, T: Copy, const N: usize> Writer<'a, T, N> {
    /// Write `value` to the next slot and advance `write_seq` by 1.
    pub fn push(&mut self, value: T) {
        let pos = (self.ring.write_seq as usize) % N;
        self.ring.data[pos] = value;
        self.ring.write_seq = self.ring.write_seq.wrapping_add(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type HwBuf = HwRing<u8, 8>;
    type SwBuf = SwRing<u8, 8>;

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
        let mut b: HwBuf = HwRing::new(0);
        // Producer at slot 0 → publish slot 5: delta 5.
        b.on_publish(5);
        assert_eq!(b.write_seq().raw(), 5);
        // Producer at slot 5 → publish slot 2 (wrapped): delta = (2+8-5)%8 = 5.
        b.on_publish(2);
        assert_eq!(b.write_seq().raw(), 10);
        // Re-publishing the same position is a no-op delta.
        b.on_publish(2);
        assert_eq!(b.write_seq().raw(), 10);
    }

    #[test]
    fn on_publish_keeps_seq_monotonic_across_u16_wrap() {
        let mut b: HwBuf = HwRing::new(0);
        b.set_write_seq_for_test(u16::MAX - 2);
        // ring slot = 65533 % 8 = 5. Roll past slot 5 to slot 1 → delta 4.
        b.on_publish(1);
        // u16::MAX - 2 + 4 wraps to 1.
        assert_eq!(b.write_seq().raw(), 1);
    }

    #[test]
    fn fresh_ring_has_zero_avail_and_lap_count() {
        let mut b: HwBuf = HwRing::new(0);
        assert_eq!(b.lap_count(), 0);
        assert_eq!(b.read_seq().raw(), 0);
        assert_eq!(b.write_seq().raw(), 0);
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
    fn peek_returns_element_at_cursor() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[10, 20, 30, 40, 50, 60, 70, 80]);
        b.set_write_seq_for_test(8);
        let mut r = b.reader();
        assert_eq!(r.peek(), Some(&10));
        r.advance(3);
        assert_eq!(r.peek(), Some(&40));
    }

    #[test]
    fn peek_is_none_when_caught_up() {
        let mut b: HwBuf = HwRing::new(0);
        assert_eq!(b.reader().peek(), None);
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
        assert_eq!(b.read_seq().raw(), 3);
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
        assert_eq!(b.read_seq().raw(), 2);
    }

    #[test]
    fn peek_auto_resyncs_when_consumer_lapped() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        // Producer 10 ahead of consumer = 2 past full lap (N=8).
        b.set_write_seq_for_test(10);
        b.set_read_seq_for_test(0);

        let mut r = b.reader();
        // First peek detects lap, jumps read_seq to write_seq - N = 2,
        // returns the slot at seq 2 (storage pos 2 = value 3).
        assert_eq!(r.peek(), Some(&3));
        assert_eq!(b.read_seq().raw(), 2);
        assert_eq!(b.lap_count(), 1);
        // Subsequent peeks at the same point don't re-trigger lap.
        let mut r = b.reader();
        assert_eq!(r.peek(), Some(&3));
        assert_eq!(b.lap_count(), 1);
    }

    #[test]
    fn at_returns_element_in_window() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[10, 20, 30, 40, 50, 60, 70, 80]);
        b.set_write_seq_for_test(8);
        assert_eq!(b.at(Seq::from_raw(0)), Some(&10));
        assert_eq!(b.at(Seq::from_raw(3)), Some(&40));
        assert_eq!(b.at(Seq::from_raw(7)), Some(&80));
    }

    #[test]
    fn at_returns_none_past_write_head() {
        let mut b: HwBuf = HwRing::new(0);
        b.stage(0, &[10, 20, 30, 40, 50, 60, 70, 80]);
        b.set_write_seq_for_test(3);
        assert_eq!(b.at(Seq::from_raw(2)), Some(&30));
        assert_eq!(b.at(Seq::from_raw(3)), None);
        assert_eq!(b.at(Seq::from_raw(5)), None);
    }

    #[test]
    fn at_returns_none_for_lapped_seq() {
        let mut b: HwBuf = HwRing::new(0);
        b.set_write_seq_for_test(20); // window is seqs [12, 20)
        assert_eq!(b.at(Seq::from_raw(11)), None);
        assert!(b.at(Seq::from_raw(12)).is_some());
        assert!(b.at(Seq::from_raw(19)).is_some());
        assert_eq!(b.at(Seq::from_raw(20)), None);
    }

    #[test]
    fn writer_push_appends_and_advances_write_seq() {
        let mut b: SwBuf = SwRing::new(0);
        {
            let mut w = b.writer();
            w.push(11);
            w.push(22);
            w.push(33);
        }
        assert_eq!(b.write_seq().raw(), 3);
        assert_eq!(b.at(Seq::from_raw(0)), Some(&11));
        assert_eq!(b.at(Seq::from_raw(1)), Some(&22));
        assert_eq!(b.at(Seq::from_raw(2)), Some(&33));
    }

    #[test]
    fn writer_push_wraps_through_ring_storage() {
        let mut b: SwBuf = SwRing::new(0);
        {
            let mut w = b.writer();
            for i in 0..12u8 {
                w.push(i);
            }
        }
        // write_seq advanced to 12; ring window [4, 12).
        assert_eq!(b.write_seq().raw(), 12);
        assert_eq!(b.at(Seq::from_raw(3)), None); // lapped out
        assert_eq!(b.at(Seq::from_raw(4)), Some(&4));
        assert_eq!(b.at(Seq::from_raw(11)), Some(&11));
    }

    #[test]
    fn seq_from_impl_retags_shared_space() {
        // rx_buf <u8, RX> and byte_ts <u16, BT> share a seq space per
        // doc §8.3; conversion preserves raw and just retags.
        let s: Seq<u8, 64> = Seq::from_raw(42);
        let t: Seq<u16, 64> = s.into();
        assert_eq!(t.raw(), 42);
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
