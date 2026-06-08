//! Callers provide the output buffer. Blanket impls for `heapless::Vec` and
//! `alloc::vec::Vec` opt-in via the `heapless` / `alloc` features; without
//! either, downstream provides its own impl (e.g. a DMA `&mut [u8]` + cursor).

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
