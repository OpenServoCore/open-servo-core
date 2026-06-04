use super::bytes::Bytes;
use super::crc::CrcUmts;
use super::frame::{BROADCAST_ID, HEADER, MAX_LENGTH, RawFrame};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    Incomplete,
    Resync { skip: usize },
    BadCrc { skip: usize },
    BadInstruction { skip: usize },
    BadLength { skip: usize },
}

/// Status instruction byte. Hardcoded here so the wire parser can detect
/// Fast chain headers (broadcast + Status with multi-slot length) without
/// depending on the typed `Instruction` enum.
const STATUS_INSTRUCTION_BYTE: u8 = 0x55;

/// Virtual ring view over `head` then `tail`. Non-wrapped callers pass
/// `tail = &[]` and get linear-input semantics with no per-byte overhead
/// on `get()` since the index branch predicts well.
#[derive(Copy, Clone)]
struct RingView<'a> {
    head: &'a [u8],
    tail: &'a [u8],
}

impl<'a> RingView<'a> {
    fn len(&self) -> usize {
        self.head.len() + self.tail.len()
    }

    fn get(&self, i: usize) -> Option<u8> {
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
    fn slice_stuffed(&self, start: usize, end: usize) -> Bytes<'a> {
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
    fn crc<CRC: CrcUmts>(&self, start: usize, end: usize) -> u16 {
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
    fn find_header(&self) -> Result<usize, usize> {
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

/// Length of the longest suffix of `ring[..n]` that is a proper prefix of
/// `HEADER`. Candidates are 3, 2, 1, 0 because `HEADER` has no internal
/// repetition.
fn longest_header_prefix_suffix(ring: &RingView<'_>, n: usize) -> usize {
    for k in (1..=3.min(n)).rev() {
        let start = n - k;
        let matched = HEADER
            .iter()
            .take(k)
            .enumerate()
            .all(|(j, h)| ring.get(start + j) == Some(*h));
        if matched {
            return k;
        }
    }
    0
}

/// Wire-layer parse: locate the frame, validate length and CRC, and return a
/// `RawFrame` with the params slice + raw instruction byte. Does NOT resolve
/// the instruction byte to the `Instruction` enum or decode the params.
///
/// On error variants other than `Incomplete`, `skip` is the number of virtual
/// bytes the caller should drop before retrying.
pub fn parse_raw<'a, CRC: CrcUmts>(
    head: &'a [u8],
    tail: &'a [u8],
) -> Result<(RawFrame<'a>, usize), ParseError> {
    let ring = RingView { head, tail };

    let header_off = match ring.find_header() {
        Ok(0) => 0,
        Ok(n) => return Err(ParseError::Resync { skip: n }),
        Err(keep) => {
            let skip = ring.len() - keep;
            if skip == 0 {
                return Err(ParseError::Incomplete);
            }
            return Err(ParseError::Resync { skip });
        }
    };

    let total = ring.len();
    if total - header_off < 7 {
        return Err(ParseError::Incomplete);
    }

    let id = ring.get(header_off + 4).unwrap();
    let length = u16::from_le_bytes([
        ring.get(header_off + 5).unwrap(),
        ring.get(header_off + 6).unwrap(),
    ]) as usize;

    // Bad length: don't trust this header. Step past 4 bytes rather than
    // honoring its claimed size, so a phantom can't mask a real frame after.
    if !(3..=MAX_LENGTH).contains(&length) {
        return Err(ParseError::BadLength { skip: HEADER.len() });
    }

    let frame_len = 7 + length;
    if total - header_off < frame_len {
        // Fast First/Only chain headers use BROADCAST_ID + Status with a
        // length covering the WHOLE multi-slot reply. When such a header
        // shows up incomplete, the missing bytes never land here — they're
        // on the wire during another node's TX. Resync past this phantom.
        if total - header_off >= 8
            && id == BROADCAST_ID
            && ring.get(header_off + 7) == Some(STATUS_INSTRUCTION_BYTE)
        {
            return Err(ParseError::BadInstruction { skip: HEADER.len() });
        }
        return Err(ParseError::Incomplete);
    }

    let frame_start = header_off;
    let crc_pos = frame_start + frame_len - 2;
    let computed = ring.crc::<CRC>(frame_start, crc_pos);
    let received = u16::from_le_bytes([ring.get(crc_pos).unwrap(), ring.get(crc_pos + 1).unwrap()]);
    if computed != received {
        // Could be a corrupted real frame *or* a phantom header — drop past
        // the header and let resync find the next one.
        return Err(ParseError::BadCrc { skip: HEADER.len() });
    }

    let instruction = ring.get(frame_start + 7).unwrap();
    let params_start = frame_start + 8;
    let params_end = crc_pos;
    let params = ring.slice_stuffed(params_start, params_end);

    Ok((
        RawFrame {
            id,
            instruction,
            params,
        },
        frame_start + frame_len,
    ))
}
