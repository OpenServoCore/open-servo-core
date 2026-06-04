/// A logically contiguous byte sequence carried as a head + tail split, so a
/// frame parsed straight out of a ring buffer can span the wrap without being
/// copied first. Non-wrapped callers pass `tail = &[]`.
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
    Raw {
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

    /// Raw (unstuffed) bytes contained in a single contiguous slice — the
    /// usual case for caller-owned buffers passed into `new(..., data)`.
    pub const fn raw(slice: &'a [u8]) -> Self {
        Bytes::Raw {
            head: slice,
            tail: &[],
        }
    }

    /// Raw bytes split across two slices.
    pub const fn raw_split(head: &'a [u8], tail: &'a [u8]) -> Self {
        Bytes::Raw { head, tail }
    }

    pub fn iter(&self) -> ByteIter<'a> {
        match *self {
            Bytes::Stuffed { head, tail, prefix } => ByteIter::with_prefix(head, tail, prefix),
            Bytes::Raw { head, tail } => ByteIter::raw(head, tail),
        }
    }

    pub fn unstuffed_len(&self) -> usize {
        match *self {
            Bytes::Raw { head, tail } => head.len() + tail.len(),
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

/// Iterator over Bytes — walks `head` then `tail`, applying `0xFF 0xFF 0xFD 0xFD`
/// → `0xFF 0xFF 0xFD` unstuffing when `stuffed` is set. The 3-byte `last3`
/// window threads across the head/tail boundary so triggers split by the cut
/// still unstuff correctly.
#[derive(Copy, Clone, Debug)]
pub struct ByteIter<'a> {
    head: &'a [u8],
    tail: &'a [u8],
    /// Virtual index across `head` then `tail` (0..head.len()+tail.len()).
    i: usize,
    last3: [u8; 3],
    stuffed: bool,
}

impl<'a> ByteIter<'a> {
    pub(crate) fn raw(head: &'a [u8], tail: &'a [u8]) -> Self {
        Self {
            head,
            tail,
            i: 0,
            last3: [0; 3],
            stuffed: false,
        }
    }

    pub(crate) fn with_prefix(head: &'a [u8], tail: &'a [u8], prefix: [u8; 3]) -> Self {
        Self {
            head,
            tail,
            i: 0,
            last3: prefix,
            stuffed: true,
        }
    }

    /// Snapshot the unconsumed remainder as a stuffed `Bytes`, threading
    /// `last3` forward so a successor iterator picks up trigger detection
    /// mid-stream.
    pub(crate) fn rest_bytes(&self) -> Bytes<'a> {
        let h = self.head.len();
        if self.i >= h {
            Bytes::Stuffed {
                head: &self.tail[self.i - h..],
                tail: &[],
                prefix: self.last3,
            }
        } else {
            Bytes::Stuffed {
                head: &self.head[self.i..],
                tail: self.tail,
                prefix: self.last3,
            }
        }
    }
}

impl<'a> Iterator for ByteIter<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        let h = self.head.len();
        let total = h + self.tail.len();
        loop {
            if self.i >= total {
                return None;
            }
            let b = if self.i < h {
                self.head[self.i]
            } else {
                self.tail[self.i - h]
            };
            self.i += 1;
            if self.stuffed && b == 0xFD && self.last3 == [0xFF, 0xFF, 0xFD] {
                // Advance past trigger so a logical FD right after isn't re-suppressed.
                self.last3 = [self.last3[1], self.last3[2], 0xFD];
                continue;
            }
            self.last3 = [self.last3[1], self.last3[2], b];
            return Some(b);
        }
    }
}
