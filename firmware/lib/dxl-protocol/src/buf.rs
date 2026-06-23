//! Output buffer trait. Blanket impls for `heapless::Vec` / `alloc::vec::Vec`
//! are feature-gated; without either, callers bring their own (e.g. DMA
//! `&mut [u8]` + cursor).

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WriteError {
    Overflow,
    Invalid,
}

pub trait WriteBuf {
    fn push(&mut self, b: u8) -> Result<(), WriteError>;
    fn len(&self) -> usize;
    fn truncate(&mut self, n: usize);
    fn set(&mut self, idx: usize, b: u8);
    fn as_slice(&self) -> &[u8];

    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Default byte-by-byte loop; impls override for `extend_from_slice`
    /// fast paths. Atomicity (on `Overflow`, leave the buffer truncated to
    /// entry length) is the caller's responsibility — encoders already
    /// truncate on failure at the frame boundary.
    fn push_slice(&mut self, slice: &[u8]) -> Result<(), WriteError> {
        for &b in slice {
            self.push(b)?;
        }
        Ok(())
    }

    /// Append `n` zero bytes. Default byte-by-byte; impls may override.
    fn push_zero(&mut self, n: u16) -> Result<(), WriteError> {
        for _ in 0..n {
            self.push(0)?;
        }
        Ok(())
    }
}

/// One run yielded by a chunk-iterator on the way to the wire: either a
/// borrowed slice the encoder writes verbatim, or a phantom span the
/// encoder materializes as `n` zero bytes. Used by the streamed encoder
/// paths so the dispatcher can hand a control-table read iterator (or any
/// other source) to `StatusEncoder` / `SlotEncoder` without a scratch
/// buffer in between.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Chunk<'a> {
    Slice(&'a [u8]),
    Zero(u16),
}

#[cfg(feature = "heapless")]
impl<const N: usize> WriteBuf for heapless::Vec<u8, N> {
    fn push(&mut self, b: u8) -> Result<(), WriteError> {
        heapless::Vec::push(self, b).map_err(|_| WriteError::Overflow)
    }
    fn len(&self) -> usize {
        heapless::Vec::as_slice(self).len()
    }
    fn truncate(&mut self, n: usize) {
        heapless::Vec::truncate(self, n);
    }
    fn set(&mut self, idx: usize, b: u8) {
        self[idx] = b;
    }
    fn as_slice(&self) -> &[u8] {
        heapless::Vec::as_slice(self)
    }
    fn push_slice(&mut self, slice: &[u8]) -> Result<(), WriteError> {
        heapless::Vec::extend_from_slice(self, slice).map_err(|_| WriteError::Overflow)
    }
}

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
impl WriteBuf for alloc::vec::Vec<u8> {
    fn push(&mut self, b: u8) -> Result<(), WriteError> {
        alloc::vec::Vec::push(self, b);
        Ok(())
    }
    fn len(&self) -> usize {
        alloc::vec::Vec::len(self)
    }
    fn truncate(&mut self, n: usize) {
        alloc::vec::Vec::truncate(self, n);
    }
    fn set(&mut self, idx: usize, b: u8) {
        self[idx] = b;
    }
    fn as_slice(&self) -> &[u8] {
        alloc::vec::Vec::as_slice(self)
    }
}

#[cfg(all(test, feature = "heapless"))]
mod heapless_tests {
    use super::*;
    use heapless::Vec;

    #[test]
    fn fresh_buffer_is_empty() {
        let buf: Vec<u8, 4> = Vec::new();
        assert!(WriteBuf::is_empty(&buf));
        assert_eq!(WriteBuf::len(&buf), 0);
        assert_eq!(WriteBuf::as_slice(&buf), &[]);
    }

    #[test]
    fn push_appends_and_tracks_length() {
        let mut buf: Vec<u8, 4> = Vec::new();
        WriteBuf::push(&mut buf, 0xAA).unwrap();
        WriteBuf::push(&mut buf, 0xBB).unwrap();
        assert_eq!(WriteBuf::as_slice(&buf), &[0xAA, 0xBB]);
        assert_eq!(WriteBuf::len(&buf), 2);
    }

    #[test]
    fn push_past_capacity_returns_overflow_and_leaves_buffer_intact() {
        let mut buf: Vec<u8, 2> = Vec::new();
        WriteBuf::push(&mut buf, 1).unwrap();
        WriteBuf::push(&mut buf, 2).unwrap();
        assert_eq!(WriteBuf::push(&mut buf, 3), Err(WriteError::Overflow));
        assert_eq!(WriteBuf::as_slice(&buf), &[1, 2]);
    }

    #[test]
    fn truncate_shrinks_to_requested_length() {
        let mut buf: Vec<u8, 8> = Vec::new();
        for b in 1..=5 {
            WriteBuf::push(&mut buf, b).unwrap();
        }
        WriteBuf::truncate(&mut buf, 3);
        assert_eq!(WriteBuf::as_slice(&buf), &[1, 2, 3]);
    }

    #[test]
    fn truncate_beyond_length_is_a_no_op() {
        let mut buf: Vec<u8, 8> = Vec::new();
        WriteBuf::push(&mut buf, 0xAA).unwrap();
        WriteBuf::truncate(&mut buf, 99);
        assert_eq!(WriteBuf::as_slice(&buf), &[0xAA]);
    }

    #[test]
    fn set_overwrites_in_place_without_changing_length() {
        let mut buf: Vec<u8, 4> = Vec::new();
        WriteBuf::push(&mut buf, 0x10).unwrap();
        WriteBuf::push(&mut buf, 0x20).unwrap();
        WriteBuf::push(&mut buf, 0x30).unwrap();
        WriteBuf::set(&mut buf, 1, 0xFF);
        assert_eq!(WriteBuf::as_slice(&buf), &[0x10, 0xFF, 0x30]);
        assert_eq!(WriteBuf::len(&buf), 3);
    }
}

#[cfg(all(test, feature = "alloc"))]
mod alloc_tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn push_is_infallible() {
        let mut buf: Vec<u8> = Vec::new();
        for b in 0..100u8 {
            assert!(WriteBuf::push(&mut buf, b).is_ok());
        }
        assert_eq!(WriteBuf::len(&buf), 100);
    }

    #[test]
    fn truncate_and_set_track_trait_semantics() {
        let mut buf: Vec<u8> = Vec::new();
        for b in 1..=5 {
            WriteBuf::push(&mut buf, b).unwrap();
        }
        WriteBuf::set(&mut buf, 0, 0xAA);
        WriteBuf::truncate(&mut buf, 2);
        assert_eq!(WriteBuf::as_slice(&buf), &[0xAA, 2]);
    }
}
