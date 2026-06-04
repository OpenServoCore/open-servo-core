/// `Stuffed::prefix` carries the three logical bytes preceding `slice` so an
/// iterator handed a tail-slice can detect a trigger that spanned the cut.
/// Top-level params use `[0; 3]` — instruction 0xFD is undefined so no valid
/// prefix completes a trigger.
#[derive(Copy, Clone, Debug)]
pub enum Bytes<'a> {
    Stuffed { slice: &'a [u8], prefix: [u8; 3] },
    Raw(&'a [u8]),
}

#[derive(Copy, Clone, Debug)]
pub struct Overflow;

impl<'a> Bytes<'a> {
    pub const fn stuffed(slice: &'a [u8]) -> Self {
        Bytes::Stuffed {
            slice,
            prefix: [0; 3],
        }
    }

    pub fn iter(&self) -> ByteIter<'a> {
        match *self {
            Bytes::Stuffed { slice, prefix } => ByteIter::with_prefix(slice, prefix),
            Bytes::Raw(s) => ByteIter::raw(s),
        }
    }

    pub fn unstuffed_len(&self) -> usize {
        match *self {
            Bytes::Raw(s) => s.len(),
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

    pub(crate) fn rest_bytes(&self) -> Bytes<'a> {
        Bytes::Stuffed {
            slice: &self.src[self.i..],
            prefix: self.last3,
        }
    }
}

impl<'a> Iterator for ByteIter<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        if self.i >= self.src.len() {
            return None;
        }
        let b = self.src[self.i];
        self.i += 1;
        if self.stuffed && b == 0xFD && self.last3 == [0xFF, 0xFF, 0xFD] {
            // Advance past trigger so a logical FD right after isn't re-suppressed.
            self.last3 = [self.last3[1], self.last3[2], 0xFD];
            return self.next();
        }
        self.last3 = [self.last3[1], self.last3[2], b];
        Some(b)
    }
}
