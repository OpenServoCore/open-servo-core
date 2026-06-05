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
