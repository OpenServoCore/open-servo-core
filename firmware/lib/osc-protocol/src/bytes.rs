//! A frame's bytes as up to two contiguous segments (`docs/osc-native-protocol.md`
//! sec 3.1). The RX ring hands frames to the parser in place; a frame that wraps
//! the ring seam arrives as head + tail. `tail` is empty in the common case, so
//! the contiguous accessors branch once and stay cheap.

/// A frame's bytes as up to two contiguous segments -- `tail` logically follows
/// `head`. Every accessor is non-panicking (bounds-checked, `Option` returns).
#[derive(Copy, Clone)]
pub struct FrameBytes<'a> {
    head: &'a [u8],
    tail: &'a [u8],
}

impl<'a> FrameBytes<'a> {
    #[inline]
    pub fn new(head: &'a [u8], tail: &'a [u8]) -> Self {
        Self { head, tail }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.head.len() + self.tail.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.head.is_empty() && self.tail.is_empty()
    }

    /// Byte `i` in the logical concatenation, or `None` past the end.
    #[inline]
    pub fn u8_at(&self, i: usize) -> Option<u8> {
        let hlen = self.head.len();
        if i < hlen {
            self.head.get(i).copied()
        } else {
            self.tail.get(i - hlen).copied()
        }
    }

    /// Little-endian `u16` at logical offset `i`.
    #[inline]
    pub fn u16_le_at(&self, i: usize) -> Option<u16> {
        Some(u16::from_le_bytes([self.u8_at(i)?, self.u8_at(i + 1)?]))
    }

    /// The `len`-byte sub-range at logical offset `start`, itself possibly
    /// spanning the seam. `None` if `[start, start+len)` leaves the view.
    #[inline]
    pub fn sub(&self, start: usize, len: usize) -> Option<FrameBytes<'a>> {
        let end = start.checked_add(len)?;
        if end > self.len() {
            return None;
        }
        let hlen = self.head.len();
        if start >= hlen {
            let s = start - hlen;
            Some(FrameBytes::new(self.tail.get(s..s + len)?, &[]))
        } else if end <= hlen {
            Some(FrameBytes::new(self.head.get(start..end)?, &[]))
        } else {
            Some(FrameBytes::new(
                self.head.get(start..)?,
                self.tail.get(..end - hlen)?,
            ))
        }
    }

    /// The raw head/tail spans, for callers that copy or forward them directly
    /// (the control-table split write, staged push).
    #[inline]
    pub fn segments(&self) -> (&'a [u8], &'a [u8]) {
        (self.head, self.tail)
    }

    /// Copy the whole view into `dst`; `None` (nothing copied) if `dst` is
    /// shorter than the view.
    #[inline]
    pub fn copy_into(&self, dst: &mut [u8]) -> Option<()> {
        let (d0, d1) = dst.split_at_mut_checked(self.head.len())?;
        d0.copy_from_slice(self.head);
        d1.get_mut(..self.tail.len())?.copy_from_slice(self.tail);
        Some(())
    }

    /// The logical bytes in order.
    #[inline]
    pub fn bytes(&self) -> impl Iterator<Item = u8> + 'a {
        let (head, tail) = (self.head, self.tail);
        head.iter().chain(tail.iter()).copied()
    }
}

impl<'a> From<&'a [u8]> for FrameBytes<'a> {
    #[inline]
    fn from(bytes: &'a [u8]) -> Self {
        FrameBytes::new(bytes, &[])
    }
}

impl PartialEq for FrameBytes<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.len() == other.len() && self.bytes().eq(other.bytes())
    }
}

impl Eq for FrameBytes<'_> {}

impl core::fmt::Debug for FrameBytes<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_list().entries(self.bytes()).finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn split_eq(full: &[u8]) {
        for k in 0..=full.len() {
            let fb = FrameBytes::new(&full[..k], &full[k..]);
            assert_eq!(fb.len(), full.len());
            assert_eq!(fb.is_empty(), full.is_empty());
            for (i, &b) in full.iter().enumerate() {
                assert_eq!(fb.u8_at(i), Some(b));
            }
            assert_eq!(fb.u8_at(full.len()), None);
            // Whole-view equality is split-invariant.
            assert_eq!(fb, FrameBytes::from(full));
        }
    }

    #[test]
    fn accessors_are_split_invariant() {
        split_eq(&[]);
        split_eq(&[0xAA]);
        split_eq(&[1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn u16_le_across_seam() {
        let full = [0x11, 0x22, 0x33, 0x44];
        for k in 0..=full.len() {
            let fb = FrameBytes::new(&full[..k], &full[k..]);
            assert_eq!(fb.u16_le_at(0), Some(0x2211));
            assert_eq!(fb.u16_le_at(2), Some(0x4433));
            assert_eq!(fb.u16_le_at(3), None);
        }
    }

    #[test]
    fn sub_spans_seam() {
        let full = [10, 11, 12, 13, 14, 15];
        for k in 0..=full.len() {
            let fb = FrameBytes::new(&full[..k], &full[k..]);
            let s = fb.sub(1, 4).unwrap();
            assert_eq!(s, FrameBytes::from(&full[1..5]));
            assert_eq!(fb.sub(4, 3), None);
            assert_eq!(fb.sub(0, full.len()), Some(fb));
        }
    }

    #[test]
    fn copy_into_stitches() {
        let full = [1u8, 2, 3, 4, 5];
        for k in 0..=full.len() {
            let fb = FrameBytes::new(&full[..k], &full[k..]);
            let mut dst = [0u8; 5];
            assert_eq!(fb.copy_into(&mut dst), Some(()));
            assert_eq!(dst, full);
            let mut small = [0u8; 4];
            assert_eq!(fb.copy_into(&mut small), None);
        }
    }
}
