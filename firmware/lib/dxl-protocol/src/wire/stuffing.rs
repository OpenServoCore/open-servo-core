//! Byte-stuffing adapters for the DXL 2.0 wire format.
//!
//! DXL 2.0 reserves `0xFF 0xFF 0xFD` as the frame header. Anywhere that triple
//! could appear inside a frame's payload, an extra `0xFD` is inserted after it
//! so a header-scan can't match a fake header mid-frame. Decoding strips that
//! extra `0xFD` back out.
//!
//! [`Stuffer`] converts a logical-byte stream to wire form (inserts `0xFD`
//! after every `0xFF 0xFF 0xFD` triple). [`Unstuffer`] does the reverse —
//! strips the `0xFD` after every triple.
//!
//! Both are zero-copy iterator adapters with a small sliding window of state.
//! Fast Read payloads skip stuffing entirely (positional decode); these
//! adapters are not used on that path.

/// Inserts the stuffing byte (`0xFD`) after every `0xFF 0xFF 0xFD` triple in
/// the underlying logical byte stream.
#[derive(Copy, Clone, Debug)]
pub struct Stuffer<I> {
    inner: I,
    /// Bytes already emitted, last two only — used to detect a triple ending
    /// at the byte we're about to emit.
    last2: [u8; 2],
    /// When `Some(b)`, the next `next()` call returns `b` immediately
    /// (the inserted `0xFD`) before consuming from `inner` again.
    pending: Option<u8>,
}

impl<I> Stuffer<I> {
    /// Construct a stuffer over `inner` with no preceding context — use for
    /// payloads whose first byte starts a fresh stream.
    pub fn new(inner: I) -> Self {
        Self {
            inner,
            last2: [0, 0],
            pending: None,
        }
    }

    /// Construct a stuffer threading two bytes of prior context — used when
    /// stuffing must continue across a field boundary (e.g. the instruction
    /// byte just before the params region).
    pub fn with_prefix(inner: I, prefix: [u8; 2]) -> Self {
        Self {
            inner,
            last2: prefix,
            pending: None,
        }
    }
}

impl<I: Iterator<Item = u8>> Iterator for Stuffer<I> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        if let Some(b) = self.pending.take() {
            self.last2 = [self.last2[1], b];
            return Some(b);
        }
        let b = self.inner.next()?;
        if self.last2[0] == 0xFF && self.last2[1] == 0xFF && b == 0xFD {
            self.pending = Some(0xFD);
        }
        self.last2 = [self.last2[1], b];
        Some(b)
    }
}

/// Strips the inserted `0xFD` after every `0xFF 0xFF 0xFD` triple. The
/// underlying iterator yields wire-form bytes; this adapter yields the
/// logical bytes the encoder started with.
#[derive(Copy, Clone, Debug)]
pub struct Unstuffer<I> {
    inner: I,
    /// Last three emitted logical bytes — wide enough to detect the triple
    /// that triggers stuffing-byte suppression on the next input.
    last3: [u8; 3],
}

impl<I> Unstuffer<I> {
    pub fn new(inner: I) -> Self {
        Self {
            inner,
            last3: [0; 3],
        }
    }

    /// Construct with three bytes of prior context — used when an unstuffer
    /// resumes mid-stream (e.g. the parser snapshotting an iterator's tail).
    pub fn with_prefix(inner: I, prefix: [u8; 3]) -> Self {
        Self {
            inner,
            last3: prefix,
        }
    }

    /// Three logical bytes already emitted — used by `Bytes::rest_bytes` to
    /// thread trigger context forward when snapshotting the unconsumed tail.
    pub(crate) fn last3(&self) -> [u8; 3] {
        self.last3
    }

    pub(crate) fn inner(&self) -> &I {
        &self.inner
    }
}

impl<I: Iterator<Item = u8>> Iterator for Unstuffer<I> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        loop {
            let b = self.inner.next()?;
            if b == 0xFD && self.last3 == [0xFF, 0xFF, 0xFD] {
                // Advance past the trigger so a logical FD right after isn't re-suppressed.
                self.last3 = [self.last3[1], self.last3[2], 0xFD];
                continue;
            }
            self.last3 = [self.last3[1], self.last3[2], b];
            return Some(b);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use heapless::Vec;

    fn stuff(bytes: &[u8]) -> Vec<u8, 64> {
        Stuffer::new(bytes.iter().copied()).collect()
    }

    fn unstuff(bytes: &[u8]) -> Vec<u8, 64> {
        Unstuffer::new(bytes.iter().copied()).collect()
    }

    #[test]
    fn stuffer_passes_through_when_no_trigger() {
        assert_eq!(&stuff(&[1, 2, 3, 4])[..], &[1, 2, 3, 4]);
    }

    #[test]
    fn stuffer_inserts_fd_after_trigger() {
        // 0xFF 0xFF 0xFD followed by something → 0xFF 0xFF 0xFD 0xFD <something>
        assert_eq!(
            &stuff(&[0xFF, 0xFF, 0xFD, 0x42])[..],
            &[0xFF, 0xFF, 0xFD, 0xFD, 0x42]
        );
    }

    #[test]
    fn stuffer_inserts_fd_when_trigger_is_terminal() {
        // Trigger at end still gets the stuffing byte appended.
        assert_eq!(
            &stuff(&[0xFF, 0xFF, 0xFD])[..],
            &[0xFF, 0xFF, 0xFD, 0xFD]
        );
    }

    #[test]
    fn stuffer_handles_multiple_triggers() {
        let input = [0xFF, 0xFF, 0xFD, 0x01, 0xFF, 0xFF, 0xFD, 0x02];
        let expected = [0xFF, 0xFF, 0xFD, 0xFD, 0x01, 0xFF, 0xFF, 0xFD, 0xFD, 0x02];
        assert_eq!(&stuff(&input)[..], &expected);
    }

    #[test]
    fn stuffer_with_prefix_continues_trigger_detection() {
        // last2 = [0xFF, 0xFF] from prior context; first byte 0xFD completes trigger.
        let s = Stuffer::with_prefix([0xFD, 0x42].iter().copied(), [0xFF, 0xFF]);
        let out: Vec<u8, 8> = s.collect();
        assert_eq!(&out[..], &[0xFD, 0xFD, 0x42]);
    }

    #[test]
    fn unstuffer_passes_through_when_no_trigger() {
        assert_eq!(&unstuff(&[1, 2, 3, 4])[..], &[1, 2, 3, 4]);
    }

    #[test]
    fn unstuffer_strips_fd_after_trigger() {
        assert_eq!(
            &unstuff(&[0xFF, 0xFF, 0xFD, 0xFD, 0x42])[..],
            &[0xFF, 0xFF, 0xFD, 0x42]
        );
    }

    #[test]
    fn unstuffer_strips_terminal_stuffing_byte() {
        assert_eq!(
            &unstuff(&[0xFF, 0xFF, 0xFD, 0xFD])[..],
            &[0xFF, 0xFF, 0xFD]
        );
    }

    #[test]
    fn unstuffer_handles_multiple_triggers() {
        let input = [0xFF, 0xFF, 0xFD, 0xFD, 0x01, 0xFF, 0xFF, 0xFD, 0xFD, 0x02];
        let expected = [0xFF, 0xFF, 0xFD, 0x01, 0xFF, 0xFF, 0xFD, 0x02];
        assert_eq!(&unstuff(&input)[..], &expected);
    }

    #[test]
    fn unstuffer_with_prefix_treats_prior_context_as_trigger() {
        let u = Unstuffer::with_prefix([0xFD, 0x42].iter().copied(), [0xFF, 0xFF, 0xFD]);
        let out: Vec<u8, 8> = u.collect();
        assert_eq!(&out[..], &[0x42]);
    }

    #[test]
    fn round_trip_preserves_logical_stream() {
        let original: Vec<u8, 64> = (0..16u8)
            .chain([0xFF, 0xFF, 0xFD, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD, 0])
            .collect();
        let stuffed: Vec<u8, 96> = Stuffer::new(original.iter().copied()).collect();
        let recovered: Vec<u8, 64> = Unstuffer::new(stuffed.iter().copied()).collect();
        assert_eq!(&recovered[..], &original[..]);
    }
}
