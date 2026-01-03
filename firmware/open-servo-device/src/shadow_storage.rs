//! Device-layer shadow storage with cross-context access control.
//!
//! This module provides `ShadowStorage`, which holds the shadow table and
//! provides safe access from both the main loop (host) and ISR (kernel).
//!
//! # Concurrency Model
//!
//! - **Host methods** (`host_*`): Called from main loop only, use `critical_section`
//! - **Kernel methods** (`kernel_*`): Called from ISR only, no CS (single-writer)
//!
//! # Staging
//!
//! Staging is implemented via `HeaplessStagingBuffer`, which uses heapless
//! vectors to store staged writes until `action()` is called.

use core::cell::UnsafeCell;

use heapless::Vec;
use open_servo_kernel_api::shadow::{
    layout, HostView, KernelView, ShadowError, ShadowTable, StageResult, StagedWrite,
    StagingBuffer, DIRTY_BLOCK_SIZE,
};

/// Staging buffer capacity (bytes).
pub const STAGE_DATA_CAP: usize = 64;

/// Maximum number of staged write entries.
pub const STAGE_ENTRY_CAP: usize = 8;

/// Standard table size (1024 bytes, Dynamixel Protocol 2.0 compatible).
pub const DEFAULT_TABLE_SIZE: usize = 1024;

/// Heapless-based staging buffer implementation.
pub struct HeaplessStagingBuffer {
    /// Staged data bytes.
    data: Vec<u8, STAGE_DATA_CAP>,
    /// Staged write entries (offset + range in data buffer).
    entries: Vec<StagedWrite, STAGE_ENTRY_CAP>,
}

impl HeaplessStagingBuffer {
    /// Create a new empty staging buffer.
    pub const fn new() -> Self {
        Self {
            data: Vec::new(),
            entries: Vec::new(),
        }
    }
}

impl Default for HeaplessStagingBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl StagingBuffer for HeaplessStagingBuffer {
    fn try_stage(&mut self, offset: u16, data: &[u8]) -> Result<(), StageResult> {
        let start = self.data.len() as u16;
        let len = data.len() as u16;

        // Check capacity
        if self.data.len() + data.len() > STAGE_DATA_CAP {
            return Err(StageResult::BufferFull);
        }
        if self.entries.len() >= STAGE_ENTRY_CAP {
            return Err(StageResult::BufferFull);
        }

        // Copy data
        self.data
            .extend_from_slice(data)
            .map_err(|_| StageResult::BufferFull)?;

        // Record entry
        self.entries
            .push(StagedWrite { offset, start, len })
            .map_err(|_| StageResult::BufferFull)?;

        Ok(())
    }

    fn has_staged(&self) -> bool {
        !self.entries.is_empty()
    }

    fn apply_to(&mut self, bytes: &mut [u8], dirty: &mut [u32]) -> usize {
        let count = self.entries.len();

        for entry in self.entries.iter() {
            let start = entry.start as usize;
            let end = start + entry.len as usize;
            let offset = entry.offset as usize;

            if end <= self.data.len() && offset + entry.len as usize <= bytes.len() {
                bytes[offset..offset + entry.len as usize].copy_from_slice(&self.data[start..end]);
                // Mark dirty
                mark_range_dirty(dirty, entry.offset, entry.len);
            }
        }

        // Clear staging
        self.data.clear();
        self.entries.clear();

        count
    }

    fn clear(&mut self) {
        self.data.clear();
        self.entries.clear();
    }
}

/// Helper to mark dirty bits.
fn mark_range_dirty(dirty: &mut [u32], offset: u16, len: u16) {
    if len == 0 {
        return;
    }
    let start_block = offset as usize / DIRTY_BLOCK_SIZE;
    let end_block = (offset as usize + len as usize).saturating_sub(1) / DIRTY_BLOCK_SIZE;
    for block in start_block..=end_block {
        let word_idx = block / 32;
        let bit_idx = block % 32;
        if word_idx < dirty.len() {
            dirty[word_idx] |= 1 << bit_idx;
        }
    }
}

/// Device-layer shadow storage.
///
/// Holds the shadow table and provides safe cross-context access.
///
/// # Safety
///
/// - `host_*` methods are called from main loop only, use `critical_section`
/// - `kernel_*` methods are called from ISR only, no CS (single-writer)
pub struct ShadowStorage<const N: usize> {
    /// Shadow table (protected by caller discipline).
    table: UnsafeCell<ShadowTable<N>>,
    /// Staging buffer (only accessed from host context with CS).
    staging: UnsafeCell<HeaplessStagingBuffer>,
}

// SAFETY: Access discipline enforced by caller:
// - Host uses critical_section
// - Kernel is single-writer in ISR
unsafe impl<const N: usize> Sync for ShadowStorage<N> {}

impl<const N: usize> ShadowStorage<N> {
    /// Create a new shadow storage.
    pub const fn new() -> Self {
        Self {
            table: UnsafeCell::new(ShadowTable::new()),
            staging: UnsafeCell::new(HeaplessStagingBuffer::new()),
        }
    }

    // ========================================================================
    // Host methods (main loop, uses critical_section)
    // ========================================================================

    /// Read bytes from shadow table (main loop context).
    pub fn host_read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        critical_section::with(|_cs| {
            // SAFETY: We're in a critical section
            let table = unsafe { &*self.table.get() };
            table.read(offset, buf)
        })
    }

    /// Write bytes to shadow table (main loop context).
    ///
    /// Only writes to RW regions (control/config).
    pub fn host_write(&self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        critical_section::with(|_cs| {
            // SAFETY: We're in a critical section
            let table = unsafe { &mut *self.table.get() };
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = HostView::new(bytes, dirty);
            view.write(offset, data)
        })
    }

    /// Write bytes to shadow table and check if EEPROM region was touched.
    ///
    /// Returns `Ok(true)` if the write succeeded and touched EEPROM region,
    /// `Ok(false)` if the write succeeded but didn't touch EEPROM.
    pub fn host_write_check_eeprom(&self, offset: u16, data: &[u8]) -> Result<bool, ShadowError> {
        self.host_write(offset, data)?;
        // Check if any byte in the write range is in EEPROM region (0x00-0x3F)
        let end = offset.saturating_add(data.len() as u16);
        let touched_eeprom = offset < layout::EEPROM_START + layout::EEPROM_LEN && end > 0;
        Ok(touched_eeprom)
    }

    /// Access host view within critical section (scoped borrow).
    ///
    /// Provides full access including staging operations.
    pub fn host_with_view<R>(
        &self,
        f: impl FnOnce(&mut HostView<'_, HeaplessStagingBuffer>) -> R,
    ) -> R {
        critical_section::with(|_cs| {
            // SAFETY: We're in a critical section
            let table = unsafe { &mut *self.table.get() };
            let staging = unsafe { &mut *self.staging.get() };
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = HostView::with_staging(bytes, dirty, staging);
            f(&mut view)
        })
    }

    // ========================================================================
    // Kernel methods (ISR only, no CS, single-writer)
    // ========================================================================

    /// Check if any region has pending writes from host.
    ///
    /// # Safety
    ///
    /// Must be called from ISR context only.
    pub fn kernel_any_dirty(&self) -> bool {
        // SAFETY: ISR context, single-writer
        let table = unsafe { &*self.table.get() };
        let dirty = table.dirty_bits();

        // Check if any dirty bit is set
        dirty.iter().any(|&word| word != 0)
    }

    /// Access kernel view via scoped closure (ISR context).
    ///
    /// Prevents accidental aliasing by enforcing single mutable borrow.
    ///
    /// # Safety
    ///
    /// Must be called from ISR context only.
    pub fn kernel_with_view<R>(&self, f: impl FnOnce(&mut KernelView<'_>) -> R) -> R {
        // SAFETY: ISR context, single-writer
        let table = unsafe { &mut *self.table.get() };
        let (bytes, dirty) = table.as_mut_slices();
        let mut view = KernelView::new(bytes, dirty);
        f(&mut view)
    }
}

impl<const N: usize> Default for ShadowStorage<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Type alias for standard shadow storage (512 bytes).
pub type StdShadowStorage = ShadowStorage<DEFAULT_TABLE_SIZE>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_staging_buffer() {
        let mut staging = HeaplessStagingBuffer::new();

        // Stage some writes
        assert!(staging.try_stage(0x80, &[1, 2, 3, 4]).is_ok());
        assert!(staging.try_stage(0x84, &[5, 6]).is_ok());
        assert!(staging.has_staged());

        // Apply to bytes
        let mut bytes = [0u8; 512];
        let mut dirty = [0u32; 2];
        let count = staging.apply_to(&mut bytes, &mut dirty);

        assert_eq!(count, 2);
        assert_eq!(&bytes[0x80..0x84], &[1, 2, 3, 4]);
        assert_eq!(&bytes[0x84..0x86], &[5, 6]);
        assert!(!staging.has_staged());
    }

    #[test]
    fn test_staging_buffer_full() {
        let mut staging = HeaplessStagingBuffer::new();

        // Fill up the data buffer
        let big_data = [0u8; STAGE_DATA_CAP];
        assert!(staging.try_stage(0x80, &big_data).is_ok());

        // Next stage should fail
        assert_eq!(staging.try_stage(0x100, &[1]), Err(StageResult::BufferFull));
    }

    #[test]
    fn test_shadow_storage_host_read_write() {
        let storage = ShadowStorage::<1024>::new();

        // Write to RAM region (position_d_gain at 0x50 = 80)
        storage.host_write(0x50, &[0xAB, 0xCD]).unwrap();

        // Read back
        let mut buf = [0u8; 2];
        storage.host_read(0x50, &mut buf).unwrap();
        assert_eq!(buf, [0xAB, 0xCD]);

        // Read EEPROM region (initially zero)
        storage.host_read(0x00, &mut buf).unwrap();
        assert_eq!(buf, [0, 0]);
    }

    #[test]
    fn test_shadow_storage_kernel_view() {
        let storage = ShadowStorage::<1024>::new();

        // Host writes to RAM region (position_d_gain at 0x50 = 80)
        storage.host_write(0x50, &[1, 2, 3, 4]).unwrap();

        // Kernel reads via view
        storage.kernel_with_view(|view| {
            assert!(view.any_dirty());

            let mut buf = [0u8; 4];
            view.read(0x50, &mut buf).unwrap();
            assert_eq!(buf, [1, 2, 3, 4]);

            // Write to vendor region (present_pos_cdeg at 516)
            view.write(516, &[0xDE, 0xAD]).unwrap();

            // Clear dirty
            view.clear_range_dirty(0x50, 4);
            assert!(!view.any_dirty());
        });

        // Verify vendor write was applied
        let mut buf = [0u8; 2];
        storage.host_read(516, &mut buf).unwrap();
        assert_eq!(buf, [0xDE, 0xAD]);
    }

    #[test]
    fn test_shadow_storage_staging() {
        let storage = ShadowStorage::<1024>::new();

        // Stage writes via host view (using RAM region addresses)
        storage.host_with_view(|view| {
            assert_eq!(view.stage_write_range(0x50, &[1, 2]), StageResult::Ok);
            assert_eq!(view.stage_write_range(0x54, &[3, 4]), StageResult::Ok);
            assert!(view.has_staged());

            // ACTION: apply staged writes
            let count = view.action();
            assert_eq!(count, 2);
            assert!(!view.has_staged());
        });

        // Verify writes applied
        let mut buf = [0u8; 4];
        storage.host_read(0x50, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 0, 0]); // First write at 0x50

        storage.host_read(0x54, &mut buf).unwrap();
        assert_eq!(buf, [3, 4, 0, 0]); // Second write at 0x54

        // Check dirty bits set
        assert!(storage.kernel_any_dirty());
    }
}
