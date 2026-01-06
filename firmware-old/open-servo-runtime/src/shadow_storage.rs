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
//!
//! # Persist Signaling
//!
//! ShadowStorage is generic over a [`PersistW`] which is signaled when
//! EEPROM regions are written. This allows executor-agnostic persist notification.

use core::cell::UnsafeCell;

use heapless::Vec;
use open_servo_shadow::{
    HostShadow, HostView, KernelView, ShadowError, ShadowTable, StageResult, StagedWrite,
    StagingBuffer, DIRTY_BLOCK_SIZE,
};

// Re-export signal traits for convenience.
pub use open_servo_hw::v2::{
    PersistR, PersistW, ResetLevel, ResetR, ResetW, SignalReader, SignalWriter,
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

    /// Check if any staged write touches EEPROM region. O(1) per entry via bitmap.
    fn touches_eeprom(&self) -> bool {
        self.entries
            .iter()
            .any(|entry| open_servo_registry::touches_eeprom(entry.offset, entry.len))
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
/// # Type Parameters
///
/// - `N`: Table size in bytes (typically 1024 for Dynamixel Protocol 2.0)
/// - `PW`: Signal type implementing [`PersistW`] for persist notifications
///
/// # Safety
///
/// - `host_*` methods are called from main loop only, use `critical_section`
/// - `kernel_*` methods are called from ISR only, no CS (single-writer)
pub struct ShadowStorage<const N: usize, PW: PersistW> {
    /// Shadow table (protected by caller discipline).
    table: UnsafeCell<ShadowTable<N>>,
    /// Staging buffer (only accessed from host context with CS).
    staging: UnsafeCell<HeaplessStagingBuffer>,
    /// Save signal (signaled when EEPROM region is written).
    save_signal: PW,
}

// SAFETY: Access discipline enforced by caller:
// - Host uses critical_section
// - Kernel is single-writer in ISR
unsafe impl<const N: usize, PW: PersistW> Sync for ShadowStorage<N, PW> {}

impl<const N: usize, PW: PersistW> ShadowStorage<N, PW> {
    /// Create a new shadow storage with the given save signal.
    ///
    /// The signal will be notified when EEPROM regions are written.
    pub fn new(save_signal: PW) -> Self {
        Self {
            table: UnsafeCell::new(ShadowTable::new()),
            staging: UnsafeCell::new(HeaplessStagingBuffer::new()),
            save_signal,
        }
    }

    /// Get reference to the save signal (for persist service to wait on).
    pub fn save_signal(&self) -> &PW {
        &self.save_signal
    }

    /// Helper to signal persist service.
    fn signal_persist(&self) {
        self.save_signal.signal(());
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
    /// Auto-signals persist if EEPROM region is touched.
    pub fn host_write(&self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        critical_section::with(|_cs| {
            // SAFETY: We're in a critical section
            let table = unsafe { &mut *self.table.get() };
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = HostView::new(bytes, dirty);
            view.write(offset, data)?;

            // Auto-signal persist if EEPROM region touched (O(1) bitmap check)
            if open_servo_registry::touches_eeprom(offset, data.len() as u16) {
                self.signal_persist();
            }
            Ok(())
        })
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

    /// Stage a write for later ACTION commit.
    ///
    /// Use with `host_action()` to apply staged writes atomically.
    pub fn host_stage(&self, offset: u16, data: &[u8]) -> StageResult {
        critical_section::with(|_cs| {
            let staging = unsafe { &mut *self.staging.get() };
            staging
                .try_stage(offset, data)
                .map_or_else(|e| e, |()| StageResult::Ok)
        })
    }

    /// Commit staged writes (ACTION instruction).
    ///
    /// Applies all staged writes atomically and signals persist if EEPROM was touched.
    /// Returns the number of writes applied.
    pub fn host_action(&self) -> usize {
        critical_section::with(|_cs| {
            let table = unsafe { &mut *self.table.get() };
            let staging = unsafe { &mut *self.staging.get() };

            // Check EEPROM touch before applying (entries will be cleared)
            let touches_eeprom = staging.touches_eeprom();

            let (bytes, dirty) = table.as_mut_slices();
            let count = staging.apply_to(bytes, dirty);

            // Signal persist if EEPROM was touched
            if touches_eeprom && count > 0 {
                self.signal_persist();
            }

            count
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

// Implement HostShadow trait for services to use.
impl<const N: usize, PW: PersistW> HostShadow for ShadowStorage<N, PW> {
    type Staging = HeaplessStagingBuffer;

    fn with_host_view<R>(&self, f: impl FnOnce(&mut HostView<'_, Self::Staging>) -> R) -> R {
        self.host_with_view(f)
    }

    fn host_read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        ShadowStorage::host_read(self, offset, buf)
    }

    fn host_write(&self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        ShadowStorage::host_write(self, offset, data)
    }

    fn host_stage(&self, offset: u16, data: &[u8]) -> StageResult {
        ShadowStorage::host_stage(self, offset, data)
    }

    fn host_action(&self) -> usize {
        ShadowStorage::host_action(self)
    }
}

// Note: No Default impl - signal must be provided at construction

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;

    /// Mock persist signal for testing (writer side only).
    struct MockPersistSignal {
        signaled: Cell<bool>,
    }

    impl MockPersistSignal {
        const fn new() -> Self {
            Self {
                signaled: Cell::new(false),
            }
        }

        fn was_signaled(&self) -> bool {
            self.signaled.get()
        }

        fn reset(&self) {
            self.signaled.set(false);
        }
    }

    // Only need SignalWriter<()> - blanket impl provides PersistW
    impl SignalWriter<()> for MockPersistSignal {
        fn signal(&self, _: ()) {
            self.signaled.set(true);
        }
    }

    fn test_storage() -> ShadowStorage<1024, MockPersistSignal> {
        ShadowStorage::new(MockPersistSignal::new())
    }

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
        let storage = test_storage();

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
        let storage = test_storage();

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
        let storage = test_storage();

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
