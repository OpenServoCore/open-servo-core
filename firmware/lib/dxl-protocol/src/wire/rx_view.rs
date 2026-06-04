use super::bytes::Bytes;
use super::crc::CrcUmts;
use super::frame::HEADER;

/// Logical view over received bytes, possibly split across a ring-buffer
/// wrap. Construct via [`RxView::linear`] for a contiguous buffer or
/// [`RxView::ring`] for a head + tail pair. The `(head, tail)` order is
/// the wire-byte order — `head` is read first.
#[derive(Copy, Clone)]
pub struct RxView<'a> {
    head: &'a [u8],
    tail: &'a [u8],
}

impl<'a> RxView<'a> {
    /// View over a single contiguous slice (no ring wrap).
    pub const fn linear(buf: &'a [u8]) -> Self {
        Self {
            head: buf,
            tail: &[],
        }
    }

    /// View over `head` followed logically by `tail`. Pass `tail = &[]` for
    /// the no-wrap case; or use [`RxView::linear`].
    pub const fn ring(head: &'a [u8], tail: &'a [u8]) -> Self {
        Self { head, tail }
    }

    pub fn len(&self) -> usize {
        self.head.len() + self.tail.len()
    }

    pub fn is_empty(&self) -> bool {
        self.head.is_empty() && self.tail.is_empty()
    }

    pub(super) fn get(&self, i: usize) -> Option<u8> {
        let h = self.head.len();
        if i < h {
            Some(self.head[i])
        } else {
            self.tail.get(i - h).copied()
        }
    }

    /// Virtual subrange `[start, end)` carved as a stuffed `Bytes` (the
    /// consumer iterator will unstuff). Picks the contiguous form
    /// (`tail = &[]`) when the range stays on one side of the wrap, the
    /// split form when it straddles. Top-level params start with empty prefix.
    pub(super) fn slice_stuffed(&self, start: usize, end: usize) -> Bytes<'a> {
        let h = self.head.len();
        if end <= h {
            Bytes::stuffed(&self.head[start..end])
        } else if start >= h {
            Bytes::stuffed(&self.tail[start - h..end - h])
        } else {
            Bytes::stuffed_split(&self.head[start..], &self.tail[..end - h])
        }
    }

    /// CRC over the virtual range, chaining seed across the wrap.
    pub(super) fn crc<CRC: CrcUmts>(&self, start: usize, end: usize) -> u16 {
        let h = self.head.len();
        if end <= h {
            CRC::accumulate(0, &self.head[start..end])
        } else if start >= h {
            CRC::accumulate(0, &self.tail[start - h..end - h])
        } else {
            let seed = CRC::accumulate(0, &self.head[start..]);
            CRC::accumulate(seed, &self.tail[..end - h])
        }
    }

    /// Find the first occurrence of `HEADER` starting at virtual offset 0,
    /// or the longest trailing prefix-of-`HEADER` suffix length if none
    /// found. Returns `Ok(start)` on match, `Err(suffix_keep)` otherwise.
    pub(super) fn find_header(&self) -> Result<usize, usize> {
        let n = self.len();
        if n < 4 {
            return Err(n.min(longest_header_prefix_suffix(self, n)));
        }
        let mut i = 0;
        while i + 4 <= n {
            if self.get(i) == Some(HEADER[0])
                && self.get(i + 1) == Some(HEADER[1])
                && self.get(i + 2) == Some(HEADER[2])
                && self.get(i + 3) == Some(HEADER[3])
            {
                return Ok(i);
            }
            i += 1;
        }
        Err(longest_header_prefix_suffix(self, n))
    }
}

/// Length of the longest suffix of `rx[..n]` that is a proper prefix of
/// `HEADER`. Candidates are 3, 2, 1, 0 because `HEADER` has no internal
/// repetition.
fn longest_header_prefix_suffix(rx: &RxView<'_>, n: usize) -> usize {
    for k in (1..=3.min(n)).rev() {
        let start = n - k;
        let matched = HEADER
            .iter()
            .take(k)
            .enumerate()
            .all(|(j, h)| rx.get(start + j) == Some(*h));
        if matched {
            return k;
        }
    }
    0
}
