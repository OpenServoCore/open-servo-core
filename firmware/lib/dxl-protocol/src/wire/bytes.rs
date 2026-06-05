/// A logically contiguous byte sequence carried as a head + tail split, so a
/// frame parsed straight out of a ring buffer can span the wrap without being
/// copied first. Non-wrapped callers pass `tail = &[]`.
///
/// `Stuffed` bytes are wire-form: iterating yields the logical bytes after
/// removing every stuffing `0xFD` that follows a `0xFF 0xFF 0xFD` triple.
/// `Unstuffed` bytes are caller-owned in logical form: iteration yields them
/// verbatim. The Stuffed/Unstuffed tag is what the iterator dispatches on.
///
/// `Stuffed::prefix` is the three logical bytes preceding `head` so an
/// iterator handed a tail-slice can detect a trigger that spanned the cut.
/// Top-level params use `[0; 3]` — instruction 0xFD is undefined so no valid
/// prefix completes a trigger.
#[derive(Copy, Clone, Debug)]
pub enum Bytes<'a> {
    Stuffed {
        head: &'a [u8],
        tail: &'a [u8],
        prefix: [u8; 3],
    },
    Unstuffed {
        head: &'a [u8],
        tail: &'a [u8],
    },
}

#[derive(Copy, Clone, Debug)]
pub struct Overflow;

impl<'a> Bytes<'a> {
    /// Stuffed bytes contained in a single contiguous slice.
    pub const fn stuffed(slice: &'a [u8]) -> Self {
        Bytes::Stuffed {
            head: slice,
            tail: &[],
            prefix: [0; 3],
        }
    }

    /// Stuffed bytes split across two slices (e.g. ring buffer wrap).
    pub const fn stuffed_split(head: &'a [u8], tail: &'a [u8]) -> Self {
        Bytes::Stuffed {
            head,
            tail,
            prefix: [0; 3],
        }
    }

    /// Unstuffed bytes contained in a single contiguous slice — the usual case
    /// for caller-owned buffers passed into `new(..., data)`.
    pub const fn unstuffed(slice: &'a [u8]) -> Self {
        Bytes::Unstuffed {
            head: slice,
            tail: &[],
        }
    }

    /// Unstuffed bytes split across two slices.
    pub const fn unstuffed_split(head: &'a [u8], tail: &'a [u8]) -> Self {
        Bytes::Unstuffed { head, tail }
    }

    pub fn iter(&self) -> ByteIter<'a> {
        match *self {
            Bytes::Stuffed { head, tail, prefix } => ByteIter::with_prefix(head, tail, prefix),
            Bytes::Unstuffed { head, tail } => ByteIter::unstuffed(head, tail),
        }
    }

    pub fn unstuffed_len(&self) -> usize {
        match *self {
            Bytes::Unstuffed { head, tail } => head.len() + tail.len(),
            Bytes::Stuffed { .. } => self.iter().count(),
        }
    }

    pub fn copy_to_slice(&self, dst: &mut [u8]) -> Result<usize, Overflow> {
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

    /// Re-tag the same underlying head/tail bytes as `Unstuffed`, bypassing
    /// any stuffing transform on iteration. Used for wire payloads the spec
    /// says were never stuffed (e.g. Fast Read responses, which decode
    /// positionally).
    pub fn as_unstuffed(&self) -> Self {
        match *self {
            Bytes::Stuffed { head, tail, .. } => Bytes::Unstuffed { head, tail },
            Bytes::Unstuffed { head, tail } => Bytes::Unstuffed { head, tail },
        }
    }

    /// Pop one logical byte from the front and return `(byte, rest)`. Returns
    /// `None` if empty, or if `self` is [`Bytes::Stuffed`] — call
    /// [`Self::as_unstuffed`] first when you know the payload is unstuffed
    /// (e.g. Fast Read responses), or iterate via [`Self::iter`] otherwise.
    pub fn split_first(self) -> Option<(u8, Bytes<'a>)> {
        let Bytes::Unstuffed { head, tail } = self else {
            return None;
        };
        if let Some((&b, rest)) = head.split_first() {
            return Some((b, Bytes::Unstuffed { head: rest, tail }));
        }
        let (&b, rest) = tail.split_first()?;
        Some((
            b,
            Bytes::Unstuffed {
                head: &[],
                tail: rest,
            },
        ))
    }

    /// Split into a `n`-byte prefix and the remainder, preserving the head/
    /// tail split when the cut straddles it. Returns `None` if fewer than `n`
    /// bytes are available, or if `self` is [`Bytes::Stuffed`] (same caveat
    /// as [`Self::split_first`]).
    pub fn split_at(self, n: usize) -> Option<(Bytes<'a>, Bytes<'a>)> {
        let Bytes::Unstuffed { head, tail } = self else {
            return None;
        };
        if head.len() + tail.len() < n {
            return None;
        }
        if n <= head.len() {
            let (taken, rest) = head.split_at(n);
            return Some((
                Bytes::Unstuffed {
                    head: taken,
                    tail: &[],
                },
                Bytes::Unstuffed { head: rest, tail },
            ));
        }
        if head.is_empty() {
            let (taken, rest) = tail.split_at(n);
            return Some((
                Bytes::Unstuffed {
                    head: taken,
                    tail: &[],
                },
                Bytes::Unstuffed {
                    head: &[],
                    tail: rest,
                },
            ));
        }
        let tail_take = n - head.len();
        let (taken_tail, tail_rest) = tail.split_at(tail_take);
        Some((
            Bytes::Unstuffed {
                head,
                tail: taken_tail,
            },
            Bytes::Unstuffed {
                head: &[],
                tail: tail_rest,
            },
        ))
    }
}

impl<'a> IntoIterator for Bytes<'a> {
    type Item = u8;
    type IntoIter = ByteIter<'a>;
    fn into_iter(self) -> ByteIter<'a> {
        self.iter()
    }
}

use super::stuffing::Unstuffer;

/// Walks `head` then `tail` virtual indices, yielding wire bytes. Used as the
/// inner iterator of [`ByteIter`]; for stuffed bytes it's wrapped in
/// [`Unstuffer`].
#[derive(Copy, Clone, Debug)]
struct RawWalker<'a> {
    head: &'a [u8],
    tail: &'a [u8],
    /// Virtual index across `head` then `tail` (0..head.len()+tail.len()).
    i: usize,
}

impl<'a> RawWalker<'a> {
    fn new(head: &'a [u8], tail: &'a [u8]) -> Self {
        Self { head, tail, i: 0 }
    }
}

impl<'a> Iterator for RawWalker<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        let h = self.head.len();
        if self.i < h {
            let b = self.head[self.i];
            self.i += 1;
            return Some(b);
        }
        let ti = self.i - h;
        if ti < self.tail.len() {
            let b = self.tail[ti];
            self.i += 1;
            return Some(b);
        }
        None
    }
}

/// Iterator over [`Bytes`] — yields logical bytes regardless of whether the
/// underlying storage is stuffed wire bytes or unstuffed caller-owned bytes.
#[derive(Copy, Clone, Debug)]
pub struct ByteIter<'a> {
    inner: ByteIterInner<'a>,
}

#[derive(Copy, Clone, Debug)]
enum ByteIterInner<'a> {
    Unstuffed(RawWalker<'a>),
    Stuffed(Unstuffer<RawWalker<'a>>),
}

impl<'a> ByteIter<'a> {
    pub(crate) fn unstuffed(head: &'a [u8], tail: &'a [u8]) -> Self {
        Self {
            inner: ByteIterInner::Unstuffed(RawWalker::new(head, tail)),
        }
    }

    pub(crate) fn with_prefix(head: &'a [u8], tail: &'a [u8], prefix: [u8; 3]) -> Self {
        Self {
            inner: ByteIterInner::Stuffed(Unstuffer::with_prefix(
                RawWalker::new(head, tail),
                prefix,
            )),
        }
    }

    /// Snapshot the unconsumed remainder as a stuffed `Bytes`, threading
    /// `last3` forward so a successor iterator picks up trigger detection
    /// mid-stream. Callers consume this only on iterators derived from
    /// `Bytes::Stuffed`; unstuffed-source remainders carry empty prefix.
    pub(crate) fn rest_bytes(&self) -> Bytes<'a> {
        let (walker, prefix) = match &self.inner {
            ByteIterInner::Unstuffed(w) => (w, [0u8; 3]),
            ByteIterInner::Stuffed(u) => (u.inner(), u.last3()),
        };
        let h = walker.head.len();
        if walker.i >= h {
            Bytes::Stuffed {
                head: &walker.tail[walker.i - h..],
                tail: &[],
                prefix,
            }
        } else {
            Bytes::Stuffed {
                head: &walker.head[walker.i..],
                tail: walker.tail,
                prefix,
            }
        }
    }
}

impl<'a> Iterator for ByteIter<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        match &mut self.inner {
            ByteIterInner::Unstuffed(w) => w.next(),
            ByteIterInner::Stuffed(u) => u.next(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collect(b: Bytes<'_>) -> heapless::Vec<u8, 16> {
        b.iter().collect()
    }

    #[test]
    fn split_first_pops_head_byte() {
        let b = Bytes::unstuffed(&[10, 20, 30]);
        let (first, rest) = b.split_first().unwrap();
        assert_eq!(first, 10);
        assert_eq!(&collect(rest)[..], &[20, 30]);
    }

    #[test]
    fn split_first_crosses_into_tail_when_head_empty() {
        let b = Bytes::unstuffed_split(&[], &[40, 50]);
        let (first, rest) = b.split_first().unwrap();
        assert_eq!(first, 40);
        assert_eq!(&collect(rest)[..], &[50]);
    }

    #[test]
    fn split_first_returns_none_when_empty() {
        assert!(Bytes::unstuffed(&[]).split_first().is_none());
    }

    #[test]
    fn split_first_returns_none_for_stuffed() {
        // split_first is unstuffed-only — caller must as_unstuffed first.
        assert!(Bytes::stuffed(&[1, 2, 3]).split_first().is_none());
    }

    #[test]
    fn split_at_within_head_keeps_tail_attached() {
        let b = Bytes::unstuffed_split(&[1, 2, 3, 4], &[5, 6]);
        let (left, right) = b.split_at(2).unwrap();
        assert_eq!(&collect(left)[..], &[1, 2]);
        assert_eq!(&collect(right)[..], &[3, 4, 5, 6]);
    }

    #[test]
    fn split_at_straddles_head_tail_boundary() {
        let b = Bytes::unstuffed_split(&[1, 2, 3], &[4, 5, 6]);
        let (left, right) = b.split_at(4).unwrap();
        assert_eq!(&collect(left)[..], &[1, 2, 3, 4]);
        assert_eq!(&collect(right)[..], &[5, 6]);
    }

    #[test]
    fn split_at_with_empty_head_takes_from_tail() {
        let b = Bytes::unstuffed_split(&[], &[1, 2, 3, 4]);
        let (left, right) = b.split_at(2).unwrap();
        assert_eq!(&collect(left)[..], &[1, 2]);
        assert_eq!(&collect(right)[..], &[3, 4]);
    }

    #[test]
    fn split_at_returns_none_when_short() {
        assert!(Bytes::unstuffed(&[1, 2, 3]).split_at(4).is_none());
    }

    #[test]
    fn split_at_returns_none_for_stuffed() {
        assert!(Bytes::stuffed(&[1, 2, 3, 4]).split_at(2).is_none());
    }
}
