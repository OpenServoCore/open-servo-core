//! View traits for type-safe shadow table access.
//!
//! These traits provide a uniform interface for reading and writing
//! shadow table bytes, implemented for both KernelView and HostView.

use open_servo_shadow::{HostView, KernelView, ShadowError, StagingBuffer};

/// Trait for reading bytes from a shadow table view.
pub trait ViewRead {
    /// Read bytes from the given offset into the buffer.
    fn read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError>;
}

/// Trait for writing bytes to a shadow table view.
pub trait ViewWrite {
    /// Write bytes to the given offset.
    fn write(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError>;
}

impl ViewRead for KernelView<'_> {
    fn read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        KernelView::read(self, offset, buf)
    }
}

impl ViewWrite for KernelView<'_> {
    fn write(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        KernelView::write(self, offset, data)
    }
}

impl<S: StagingBuffer> ViewRead for HostView<'_, S> {
    fn read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        HostView::read(self, offset, buf)
    }
}

impl<S: StagingBuffer> ViewWrite for HostView<'_, S> {
    fn write(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        HostView::write(self, offset, data)
    }
}
