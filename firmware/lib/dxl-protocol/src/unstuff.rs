//! DXL 2.0 byte-*de*stuffing over a borrowed param slice.
//!
//! The [`encode`](crate::encode) emitters insert an inline `0xFD` after every
//! `FF FF FD` window so an embedded sequence can't synthesize a fake header.
//! [`Bytes`] wraps the stuffed wire slice and its [`ByteIter`] yields
//! the logical bytes lazily, dropping those inserted `0xFD`s on the way out.
//! Nothing is copied until the caller consumes.

/// A borrowed param region, either raw or DXL-stuffed. Iterate with
/// [`iter`](Self::iter) to recover the logical bytes.
#[derive(Copy, Clone, Debug)]
pub enum Bytes<'a> {
    /// Stuffed wire bytes plus the three logical bytes preceding `slice`, so
    /// a tail-slice can still detect a trigger that spanned the cut. Top-level
    /// params seed `[0; 3]` — no valid prefix completes a trigger there.
    Stuffed { slice: &'a [u8], prefix: [u8; 3] },
    /// Already-logical bytes; iterated verbatim.
    Raw(&'a [u8]),
}

/// [`Bytes::copy_into`] ran out of destination space.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Overflow;

impl<'a> Bytes<'a> {
    pub const fn stuffed(slice: &'a [u8]) -> Self {
        Bytes::Stuffed {
            slice,
            prefix: [0; 3],
        }
    }

    pub const fn raw(slice: &'a [u8]) -> Self {
        Bytes::Raw(slice)
    }

    pub fn iter(&self) -> ByteIter<'a> {
        match *self {
            Bytes::Stuffed { slice, prefix } => ByteIter::with_prefix(slice, prefix),
            Bytes::Raw(s) => ByteIter::raw(s),
        }
    }

    /// Logical (unstuffed) byte count. `O(n)` for the stuffed case.
    pub fn unstuffed_len(&self) -> usize {
        match *self {
            Bytes::Raw(s) => s.len(),
            Bytes::Stuffed { .. } => self.iter().count(),
        }
    }

    /// Copy the logical bytes into `dst`, returning the count written.
    pub fn copy_into(&self, dst: &mut [u8]) -> Result<usize, Overflow> {
        let mut n = 0;
        for b in self.iter() {
            if n >= dst.len() {
                return Err(Overflow);
            }
            dst[n] = b;
            n += 1;
        }
        Ok(n)
    }
}

/// Lazy destuffing iterator over a [`Bytes`] slice.
#[derive(Copy, Clone, Debug)]
pub struct ByteIter<'a> {
    src: &'a [u8],
    i: usize,
    last3: [u8; 3],
    stuffed: bool,
}

impl<'a> ByteIter<'a> {
    pub(crate) fn raw(src: &'a [u8]) -> Self {
        Self {
            src,
            i: 0,
            last3: [0; 3],
            stuffed: false,
        }
    }

    pub(crate) fn stuffed(src: &'a [u8]) -> Self {
        Self::with_prefix(src, [0; 3])
    }

    pub(crate) fn with_prefix(src: &'a [u8], prefix: [u8; 3]) -> Self {
        Self {
            src,
            i: 0,
            last3: prefix,
            stuffed: true,
        }
    }

    /// The still-unconsumed tail as a fresh [`Bytes`], carrying the sliding
    /// window so a trigger straddling the cut still unstuffs.
    pub(crate) fn rest_bytes(&self) -> Bytes<'a> {
        Bytes::Stuffed {
            slice: &self.src[self.i..],
            prefix: self.last3,
        }
    }
}

impl Iterator for ByteIter<'_> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        while self.i < self.src.len() {
            let b = self.src[self.i];
            self.i += 1;
            if self.stuffed && b == 0xFD && self.last3 == [0xFF, 0xFF, 0xFD] {
                // Advance past the inserted FD so a logical FD right after
                // isn't re-suppressed. Loop rather than recurse — long runs
                // of triggers must not grow the stack on the firmware path.
                self.last3 = [self.last3[1], self.last3[2], 0xFD];
                continue;
            }
            self.last3 = [self.last3[1], self.last3[2], b];
            return Some(b);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate alloc;
    use alloc::vec::Vec;

    fn logical(b: &Bytes) -> Vec<u8> {
        b.iter().collect()
    }

    #[test]
    fn raw_passes_through() {
        let b = Bytes::raw(&[0x01, 0xFD, 0x02]);
        assert_eq!(logical(&b), &[0x01, 0xFD, 0x02]);
        assert_eq!(b.unstuffed_len(), 3);
    }

    #[test]
    fn stuffed_drops_inserted_fd() {
        // FF FF FD FD -> logical FF FF FD.
        let b = Bytes::stuffed(&[0xFF, 0xFF, 0xFD, 0xFD]);
        assert_eq!(logical(&b), &[0xFF, 0xFF, 0xFD]);
        assert_eq!(b.unstuffed_len(), 3);
    }

    #[test]
    fn consecutive_triggers_unstuff() {
        let b = Bytes::stuffed(&[0xFF, 0xFF, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD, 0xFD]);
        assert_eq!(logical(&b), &[0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD]);
    }

    #[test]
    fn copy_into_reports_overflow() {
        let b = Bytes::raw(&[1, 2, 3]);
        let mut dst = [0u8; 2];
        assert_eq!(b.copy_into(&mut dst), Err(Overflow));
    }
}
