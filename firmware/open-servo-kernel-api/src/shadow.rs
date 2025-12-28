//! Shadow table for host-visible register/telemetry access.
//!
//! The shadow table provides a byte-addressable control table for the servo firmware,
//! enabling Dynamixel-like range-based register access.
//!
//! # Architecture
//!
//! - **Host** writes to RW regions (control/config) via `HostView`
//! - **Kernel** reads RW regions, validates, commits to live state
//! - **Kernel** writes to RO region (telemetry) for host to read
//! - Dirty tracking is block-based (16-byte blocks)
//!
//! # Single-Plane Rule
//!
//! The shadow table is the **only** host/control plane for config and telemetry.
//! All register reads/writes go through the shadow table, not through `HostOp`.

/// Block size for dirty tracking (16 bytes per block).
pub const DIRTY_BLOCK_SIZE: usize = 16;

/// Maximum dirty words supported (covers up to 1024 bytes).
/// Using fixed max to avoid const-generic sizing complexity.
pub const MAX_DIRTY_WORDS: usize = 2;

/// Shadow table errors.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ShadowError {
    /// Access extends beyond table bounds.
    OutOfBounds,
    /// Attempted write to read-only region (telemetry).
    ReadOnly,
    /// Attempted read from write-only region (if any).
    WriteOnly,
}

/// Shadow table storage with block-based dirty tracking.
///
/// The table is divided into 16-byte blocks. Each block has a dirty bit
/// tracked in a compact u32 array. Dirty bits represent only host→kernel
/// RW writes that require commit.
///
/// # Type Parameters
///
/// - `N`: Table size in bytes (typically 512)
pub struct ShadowTable<const N: usize> {
    /// Raw byte storage.
    bytes: [u8; N],
    /// Dirty bits: one bit per 16-byte block.
    /// Fixed size to avoid const-generic complexity.
    dirty: [u32; MAX_DIRTY_WORDS],
}

impl<const N: usize> ShadowTable<N> {
    /// Maximum table size supported by dirty tracking (1024 bytes).
    ///
    /// MAX_DIRTY_WORDS * 32 bits * DIRTY_BLOCK_SIZE bytes/block.
    pub const MAX_TABLE_SIZE: usize = MAX_DIRTY_WORDS * 32 * DIRTY_BLOCK_SIZE;

    /// Create a new shadow table initialized to zero.
    ///
    /// # Compile-time check
    ///
    /// Fails to compile if N exceeds dirty tracking capacity (1024 bytes).
    pub const fn new() -> Self {
        // Compile-time capacity check: array length mismatch if N > MAX_TABLE_SIZE.
        // RHS is [(); 1] when N <= MAX, [(); 0] when N > MAX → type error.
        const { assert!(N <= Self::MAX_TABLE_SIZE, "ShadowTable size exceeds dirty tracking capacity (max 1024 bytes)") };

        Self {
            bytes: [0u8; N],
            dirty: [0u32; MAX_DIRTY_WORDS],
        }
    }

    /// Table size in bytes.
    pub const fn size(&self) -> usize {
        N
    }

    /// Read a byte range from the table.
    ///
    /// No side effects; does not affect dirty bits.
    pub fn read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        let start = offset as usize;
        let end = start.saturating_add(buf.len());
        if end > N {
            return Err(ShadowError::OutOfBounds);
        }
        buf.copy_from_slice(&self.bytes[start..end]);
        Ok(())
    }

    /// Write a byte range to the table.
    ///
    /// This is the low-level write that marks blocks dirty.
    /// Use `HostView::write()` for access control.
    pub fn write(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        let start = offset as usize;
        let end = start.saturating_add(data.len());
        if end > N {
            return Err(ShadowError::OutOfBounds);
        }
        self.bytes[start..end].copy_from_slice(data);
        self.mark_range_dirty(offset, data.len() as u16);
        Ok(())
    }

    /// Write a byte range **without** marking dirty.
    ///
    /// Used for telemetry writes from kernel side.
    pub fn write_no_dirty(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        let start = offset as usize;
        let end = start.saturating_add(data.len());
        if end > N {
            return Err(ShadowError::OutOfBounds);
        }
        self.bytes[start..end].copy_from_slice(data);
        Ok(())
    }

    /// Check if any blocks are dirty.
    pub fn is_dirty(&self) -> bool {
        self.dirty.iter().any(|&w| w != 0)
    }

    /// Check if a specific block is dirty.
    pub fn is_block_dirty(&self, block: usize) -> bool {
        let word_idx = block / 32;
        let bit_idx = block % 32;
        if word_idx >= MAX_DIRTY_WORDS {
            return false;
        }
        (self.dirty[word_idx] & (1 << bit_idx)) != 0
    }

    /// Check if any block in the given byte range is dirty.
    pub fn is_range_dirty(&self, offset: u16, len: u16) -> bool {
        if len == 0 {
            return false;
        }
        let start_block = offset as usize / DIRTY_BLOCK_SIZE;
        let end_block = (offset as usize + len as usize).saturating_sub(1) / DIRTY_BLOCK_SIZE;
        for block in start_block..=end_block {
            if self.is_block_dirty(block) {
                return true;
            }
        }
        false
    }

    /// Mark blocks covering the given byte range as dirty.
    pub fn mark_range_dirty(&mut self, offset: u16, len: u16) {
        if len == 0 {
            return;
        }
        let start_block = offset as usize / DIRTY_BLOCK_SIZE;
        let end_block = (offset as usize + len as usize).saturating_sub(1) / DIRTY_BLOCK_SIZE;
        for block in start_block..=end_block {
            self.mark_block_dirty(block);
        }
    }

    /// Mark a specific block as dirty.
    fn mark_block_dirty(&mut self, block: usize) {
        let word_idx = block / 32;
        let bit_idx = block % 32;
        if word_idx < MAX_DIRTY_WORDS {
            self.dirty[word_idx] |= 1 << bit_idx;
        }
    }

    /// Clear all dirty bits.
    pub fn clear_dirty(&mut self) {
        for w in self.dirty.iter_mut() {
            *w = 0;
        }
    }

    /// Clear dirty bits for a specific byte range.
    pub fn clear_range_dirty(&mut self, offset: u16, len: u16) {
        if len == 0 {
            return;
        }
        let start_block = offset as usize / DIRTY_BLOCK_SIZE;
        let end_block = (offset as usize + len as usize).saturating_sub(1) / DIRTY_BLOCK_SIZE;
        for block in start_block..=end_block {
            self.clear_block_dirty(block);
        }
    }

    /// Clear a specific block's dirty bit.
    fn clear_block_dirty(&mut self, block: usize) {
        let word_idx = block / 32;
        let bit_idx = block % 32;
        if word_idx < MAX_DIRTY_WORDS {
            self.dirty[word_idx] &= !(1 << bit_idx);
        }
    }

    /// Get raw byte slice (immutable).
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Get mutable byte slice.
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        &mut self.bytes
    }

    /// Get dirty bits slice (for views).
    pub fn dirty_bits(&self) -> &[u32] {
        &self.dirty
    }

    /// Get mutable dirty bits slice (for views).
    pub fn dirty_bits_mut(&mut self) -> &mut [u32] {
        &mut self.dirty
    }

    /// Get both mutable slices at once (for creating views).
    ///
    /// This avoids borrow checker issues when creating `HostView` or `KernelView`.
    pub fn as_mut_slices(&mut self) -> (&mut [u8], &mut [u32]) {
        (&mut self.bytes, &mut self.dirty)
    }
}

impl<const N: usize> Default for ShadowTable<N> {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Region Layout (Stable ABI)
// ============================================================================

/// Region layout constants (stable ABI).
///
/// Field offsets *within* regions are kernel-crate-specific, not part of this API.
pub mod layout {
    /// Telemetry region: Host-RO, kernel writes. Bytes 0x0000..0x007F.
    pub const TELEM_START: u16 = 0x0000;
    /// Telemetry region length: 128 bytes.
    pub const TELEM_LEN: u16 = 128;

    /// Control region: Host-RW, kernel reads. Bytes 0x0080..0x00FF.
    pub const CTRL_START: u16 = 0x0080;
    /// Control region length: 128 bytes.
    pub const CTRL_LEN: u16 = 128;

    /// Config region: Host-RW, kernel reads (persistent). Bytes 0x0100..0x01FF.
    pub const CONFIG_START: u16 = 0x0100;
    /// Config region length: 256 bytes.
    pub const CONFIG_LEN: u16 = 256;

    /// Check if an offset is in the telemetry (RO) region.
    #[inline]
    pub const fn is_telem(offset: u16) -> bool {
        offset >= TELEM_START && offset < TELEM_START + TELEM_LEN
    }

    /// Check if an offset is in a RW region (control or config).
    #[inline]
    pub const fn is_rw(offset: u16) -> bool {
        (offset >= CTRL_START && offset < CTRL_START + CTRL_LEN)
            || (offset >= CONFIG_START && offset < CONFIG_START + CONFIG_LEN)
    }

    /// Check if a range is entirely in a RW region.
    #[inline]
    pub const fn is_range_rw(offset: u16, len: u16) -> bool {
        if len == 0 {
            return true;
        }
        let end = offset.saturating_add(len).saturating_sub(1);
        // Entire range in control region
        let in_ctrl = offset >= CTRL_START
            && end < CTRL_START + CTRL_LEN;
        // Entire range in config region
        let in_config = offset >= CONFIG_START
            && end < CONFIG_START + CONFIG_LEN;
        in_ctrl || in_config
    }

    /// Check if a range is entirely in the telemetry region.
    #[inline]
    pub const fn is_range_telem(offset: u16, len: u16) -> bool {
        if len == 0 {
            return true;
        }
        let end = offset.saturating_add(len).saturating_sub(1);
        offset >= TELEM_START && end < TELEM_START + TELEM_LEN
    }
}

// ============================================================================
// Staging Types
// ============================================================================

/// Staging result.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StageResult {
    /// Staging succeeded.
    Ok,
    /// Staging buffer capacity exceeded.
    BufferFull,
    /// Access extends beyond table bounds.
    OutOfBounds,
    /// Attempted stage to read-only region.
    ReadOnly,
    /// Staging not enabled (no staging buffers provided).
    NotEnabled,
}

/// Staging entry: describes a staged write's location in the staging buffer.
#[derive(Copy, Clone, Debug, Default)]
pub struct StagedWrite {
    /// Target offset in shadow table.
    pub offset: u16,
    /// Start index in staging data buffer.
    pub start: u16,
    /// Length of staged data.
    pub len: u16,
}

/// Staging buffer trait for device-owned staging storage.
///
/// Device crate provides the concrete implementation using heapless.
pub trait StagingBuffer {
    /// Try to stage data for a write.
    ///
    /// Returns the entry if successful, or `StageResult` error.
    fn try_stage(&mut self, offset: u16, data: &[u8]) -> Result<(), StageResult>;

    /// Check if there are staged writes pending.
    fn has_staged(&self) -> bool;

    /// Apply all staged writes to the shadow table bytes.
    ///
    /// Returns number of writes applied.
    fn apply_to(&mut self, bytes: &mut [u8], dirty: &mut [u32]) -> usize;

    /// Clear all staged writes without applying.
    fn clear(&mut self);
}

// ============================================================================
// Host View
// ============================================================================

/// Host-side view of the shadow table.
///
/// Enforces region access rules:
/// - Can read anywhere (telemetry + control + config)
/// - Can only write to RW regions (control + config)
///
/// Dirty bits are set on writes to RW regions.
///
/// Optionally supports staging via `stage_write_range()` and `action()`.
pub struct HostView<'a, S: StagingBuffer = NoStaging> {
    bytes: &'a mut [u8],
    dirty: &'a mut [u32],
    staging: Option<&'a mut S>,
}

/// No-op staging buffer (staging disabled).
#[derive(Default)]
pub struct NoStaging;

impl StagingBuffer for NoStaging {
    fn try_stage(&mut self, _offset: u16, _data: &[u8]) -> Result<(), StageResult> {
        Err(StageResult::NotEnabled)
    }
    fn has_staged(&self) -> bool {
        false
    }
    fn apply_to(&mut self, _bytes: &mut [u8], _dirty: &mut [u32]) -> usize {
        0
    }
    fn clear(&mut self) {}
}

impl<'a> HostView<'a, NoStaging> {
    /// Create a new host view without staging support.
    ///
    /// # Safety (caller must ensure)
    /// - `bytes` and `dirty` come from the same `ShadowTable`
    /// - Access is protected by critical section in device crate
    pub fn new(bytes: &'a mut [u8], dirty: &'a mut [u32]) -> Self {
        Self {
            bytes,
            dirty,
            staging: None,
        }
    }
}

impl<'a, S: StagingBuffer> HostView<'a, S> {
    /// Create a new host view with staging support.
    pub fn with_staging(bytes: &'a mut [u8], dirty: &'a mut [u32], staging: &'a mut S) -> Self {
        Self {
            bytes,
            dirty,
            staging: Some(staging),
        }
    }

    /// Table size.
    pub fn size(&self) -> usize {
        self.bytes.len()
    }

    /// Read bytes from any region.
    pub fn read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        let start = offset as usize;
        let end = start.saturating_add(buf.len());
        if end > self.bytes.len() {
            return Err(ShadowError::OutOfBounds);
        }
        buf.copy_from_slice(&self.bytes[start..end]);
        Ok(())
    }

    /// Write bytes to RW regions only.
    ///
    /// Marks affected blocks as dirty. For immediate commit.
    pub fn write(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        let len = data.len() as u16;
        if !layout::is_range_rw(offset, len) {
            return Err(ShadowError::ReadOnly);
        }
        let start = offset as usize;
        let end = start.saturating_add(data.len());
        if end > self.bytes.len() {
            return Err(ShadowError::OutOfBounds);
        }
        self.bytes[start..end].copy_from_slice(data);
        self.mark_range_dirty(offset, len);
        Ok(())
    }

    /// Stage a write for later ACTION commit.
    ///
    /// Data is copied to staging buffer; applied atomically on `action()`.
    /// Returns `StageResult::NotEnabled` if staging is not configured.
    pub fn stage_write_range(&mut self, offset: u16, data: &[u8]) -> StageResult {
        let len = data.len() as u16;
        if !layout::is_range_rw(offset, len) {
            return StageResult::ReadOnly;
        }
        let end = offset as usize + data.len();
        if end > self.bytes.len() {
            return StageResult::OutOfBounds;
        }

        match &mut self.staging {
            Some(staging) => match staging.try_stage(offset, data) {
                Ok(()) => StageResult::Ok,
                Err(e) => e,
            },
            None => StageResult::NotEnabled,
        }
    }

    /// Commit all staged writes atomically to shadow table.
    ///
    /// Clears staging buffer. Returns number of writes applied.
    /// Marks affected blocks as dirty.
    pub fn action(&mut self) -> usize {
        match &mut self.staging {
            Some(staging) => staging.apply_to(self.bytes, self.dirty),
            None => 0,
        }
    }

    /// Check if there are staged writes pending.
    pub fn has_staged(&self) -> bool {
        match &self.staging {
            Some(staging) => staging.has_staged(),
            None => false,
        }
    }

    /// Clear staging buffer without applying.
    pub fn clear_staged(&mut self) {
        if let Some(staging) = &mut self.staging {
            staging.clear();
        }
    }

    /// Mark blocks covering the given byte range as dirty.
    fn mark_range_dirty(&mut self, offset: u16, len: u16) {
        mark_range_dirty_impl(self.dirty, offset, len);
    }
}

/// Helper to mark dirty bits (shared between views).
fn mark_range_dirty_impl(dirty: &mut [u32], offset: u16, len: u16) {
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

// ============================================================================
// Kernel View
// ============================================================================

/// Kernel-side view of the shadow table.
///
/// The kernel can:
/// - Read from RW regions (control/config) to get host-written values
/// - Write to telemetry region (RO from host) **without** marking dirty
/// - Check and clear dirty bits for commit processing
pub struct KernelView<'a> {
    bytes: &'a mut [u8],
    dirty: &'a mut [u32],
}

impl<'a> KernelView<'a> {
    /// Create a new kernel view from raw byte and dirty slices.
    ///
    /// # Safety (caller must ensure)
    /// - Called from ISR context only (single-writer)
    /// - `bytes` and `dirty` come from the same `ShadowTable`
    pub fn new(bytes: &'a mut [u8], dirty: &'a mut [u32]) -> Self {
        Self { bytes, dirty }
    }

    /// Read from control/config region.
    ///
    /// Returns `ReadOnly` if trying to read from telemetry region
    /// (kernel should already know telemetry values).
    pub fn read_ctrl(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        let len = buf.len() as u16;
        if !layout::is_range_rw(offset, len) {
            return Err(ShadowError::ReadOnly);
        }
        let start = offset as usize;
        let end = start.saturating_add(buf.len());
        if end > self.bytes.len() {
            return Err(ShadowError::OutOfBounds);
        }
        buf.copy_from_slice(&self.bytes[start..end]);
        Ok(())
    }

    /// Write to telemetry region. **Does NOT mark dirty.**
    ///
    /// Telemetry updates never trigger commits.
    pub fn write_telem(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        let len = data.len() as u16;
        if !layout::is_range_telem(offset, len) {
            return Err(ShadowError::WriteOnly);
        }
        let start = offset as usize;
        let end = start.saturating_add(data.len());
        if end > self.bytes.len() {
            return Err(ShadowError::OutOfBounds);
        }
        self.bytes[start..end].copy_from_slice(data);
        // NOTE: No dirty marking for telemetry writes
        Ok(())
    }

    /// Check if control/config region has any pending writes.
    pub fn ctrl_dirty(&self) -> bool {
        // Check blocks covering CTRL and CONFIG regions
        let ctrl_start_block = layout::CTRL_START as usize / DIRTY_BLOCK_SIZE;
        let config_end = (layout::CONFIG_START + layout::CONFIG_LEN) as usize;
        let config_end_block = config_end.saturating_sub(1) / DIRTY_BLOCK_SIZE;

        for block in ctrl_start_block..=config_end_block {
            let word_idx = block / 32;
            let bit_idx = block % 32;
            if word_idx < self.dirty.len() && (self.dirty[word_idx] & (1 << bit_idx)) != 0 {
                return true;
            }
        }
        false
    }

    /// Check if any block in the given byte range is dirty.
    pub fn is_range_dirty(&self, offset: u16, len: u16) -> bool {
        if len == 0 {
            return false;
        }
        let start_block = offset as usize / DIRTY_BLOCK_SIZE;
        let end_block = (offset as usize + len as usize).saturating_sub(1) / DIRTY_BLOCK_SIZE;
        for block in start_block..=end_block {
            let word_idx = block / 32;
            let bit_idx = block % 32;
            if word_idx < self.dirty.len() && (self.dirty[word_idx] & (1 << bit_idx)) != 0 {
                return true;
            }
        }
        false
    }

    /// Clear dirty bits for a specific byte range (per-field clearing).
    ///
    /// Use this to clear only fields that were successfully committed.
    pub fn clear_range_dirty(&mut self, offset: u16, len: u16) {
        if len == 0 {
            return;
        }
        let start_block = offset as usize / DIRTY_BLOCK_SIZE;
        let end_block = (offset as usize + len as usize).saturating_sub(1) / DIRTY_BLOCK_SIZE;
        for block in start_block..=end_block {
            let word_idx = block / 32;
            let bit_idx = block % 32;
            if word_idx < self.dirty.len() {
                self.dirty[word_idx] &= !(1 << bit_idx);
            }
        }
    }
}

// ============================================================================
// Kernel Commit Trait
// ============================================================================

/// Result of a shadow→live commit attempt.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommitResult {
    /// Commit succeeded, state updated.
    Ok,
    /// No dirty data to commit.
    NothingToCommit,
    /// Commit refused due to system state (e.g., engaged, busy).
    Busy,
    /// Validation failed for a field at the given offset.
    ///
    /// Dirty bit for this field remains set so host can fix and retry.
    ValidationError {
        /// Offset of the invalid field.
        offset: u16,
    },
}

/// Field descriptor for commit processing.
///
/// Used by kernels to define their field layout for commit logic.
#[derive(Copy, Clone, Debug)]
pub struct FieldDesc {
    /// Offset in shadow table.
    pub offset: u16,
    /// Field length in bytes.
    pub len: u8,
}

impl FieldDesc {
    /// Create a new field descriptor.
    pub const fn new(offset: u16, len: u8) -> Self {
        Self { offset, len }
    }

    /// Check if this field overlaps a dirty range.
    ///
    /// Used to determine if a field needs to be committed.
    pub const fn overlaps(&self, dirty_start: u16, dirty_len: u16) -> bool {
        if dirty_len == 0 || self.len == 0 {
            return false;
        }
        let field_end = self.offset.saturating_add(self.len as u16);
        let dirty_end = dirty_start.saturating_add(dirty_len);
        // Overlap if: field_start < dirty_end && dirty_start < field_end
        self.offset < dirty_end && dirty_start < field_end
    }
}

/// Kernel commit contract.
///
/// Kernels implement this to:
/// 1. Check which dirty ranges overlap known fields via `view.is_range_dirty()`
/// 2. Read shadow bytes via `view.read_ctrl()`
/// 3. Validate and apply to live kernel config/state
/// 4. Clear dirty bits via `view.clear_range_dirty()` for each successfully committed field
///
/// # Error Handling
///
/// On validation error:
/// - Do NOT blindly clear all dirty bits
/// - Clear only fields that were successfully applied
/// - Return `CommitResult::ValidationError { offset }` with dirty bits intact
/// - Host can fix the invalid value and retry
pub trait ShadowKernel {
    /// Publish telemetry data to shadow table.
    ///
    /// Called from ISR context at a safe tick boundary (e.g., ControlMedium).
    /// Writes kernel state to telemetry region via `view.write_telem()`.
    ///
    /// # Note
    ///
    /// Telemetry writes do NOT mark dirty bits. This is read-only data for the host.
    fn publish_telemetry(&self, view: &mut KernelView<'_>);

    /// Commit dirty shadow data to live kernel state.
    ///
    /// Called from ISR context at a safe tick boundary (e.g., ControlMedium).
    fn commit_shadow(&mut self, view: &mut KernelView<'_>) -> CommitResult;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shadow_table_read_write() {
        let mut table = ShadowTable::<512>::new();

        // Write some data
        table.write(0x10, &[1, 2, 3, 4]).unwrap();

        // Read it back
        let mut buf = [0u8; 4];
        table.read(0x10, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);

        // Check dirty
        assert!(table.is_range_dirty(0x10, 4));
        assert!(!table.is_range_dirty(0x00, 4));
    }

    #[test]
    fn test_dirty_tracking_block_boundaries() {
        let mut table = ShadowTable::<512>::new();

        // Write spanning two blocks (block 0 and 1)
        table.write(14, &[1, 2, 3, 4]).unwrap(); // bytes 14-17

        assert!(table.is_block_dirty(0)); // block 0: bytes 0-15
        assert!(table.is_block_dirty(1)); // block 1: bytes 16-31
        assert!(!table.is_block_dirty(2));
    }

    #[test]
    fn test_clear_range_dirty() {
        let mut table = ShadowTable::<512>::new();

        table.write(0x00, &[1, 2, 3, 4]).unwrap();
        table.write(0x20, &[5, 6, 7, 8]).unwrap();

        assert!(table.is_range_dirty(0x00, 4));
        assert!(table.is_range_dirty(0x20, 4));

        // Clear only first range
        table.clear_range_dirty(0x00, 4);

        assert!(!table.is_range_dirty(0x00, 4));
        assert!(table.is_range_dirty(0x20, 4));
    }

    #[test]
    fn test_write_no_dirty() {
        let mut table = ShadowTable::<512>::new();

        table.write_no_dirty(0x10, &[1, 2, 3, 4]).unwrap();

        // Data should be written
        let mut buf = [0u8; 4];
        table.read(0x10, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);

        // But no dirty bits
        assert!(!table.is_dirty());
    }

    #[test]
    fn test_out_of_bounds() {
        let mut table = ShadowTable::<64>::new();

        assert_eq!(table.write(60, &[1, 2, 3, 4, 5]), Err(ShadowError::OutOfBounds));
        assert_eq!(table.read(60, &mut [0u8; 5]), Err(ShadowError::OutOfBounds));
    }

    #[test]
    fn test_layout_region_checks() {
        // Telemetry region: 0x00..0x7F
        assert!(layout::is_telem(0x00));
        assert!(layout::is_telem(0x7F));
        assert!(!layout::is_telem(0x80));

        // Control region: 0x80..0xFF
        assert!(layout::is_rw(0x80));
        assert!(layout::is_rw(0xFF));

        // Config region: 0x100..0x1FF
        assert!(layout::is_rw(0x100));
        assert!(layout::is_rw(0x1FF));

        // Gap between telemetry and control
        assert!(!layout::is_telem(0x80));

        // Range checks
        assert!(layout::is_range_telem(0x00, 128));
        assert!(!layout::is_range_telem(0x00, 129)); // extends past
        assert!(layout::is_range_rw(0x80, 128));
        assert!(layout::is_range_rw(0x100, 256));
    }

    #[test]
    fn test_host_view_read_write() {
        let mut table = ShadowTable::<512>::new();
        let (bytes, dirty) = table.as_mut_slices();
        let mut view = HostView::new(bytes, dirty);

        // Host can read anywhere
        let mut buf = [0u8; 4];
        assert!(view.read(0x00, &mut buf).is_ok()); // telemetry region
        assert!(view.read(0x80, &mut buf).is_ok()); // control region

        // Host can only write to RW regions
        assert_eq!(view.write(0x00, &[1, 2, 3, 4]), Err(ShadowError::ReadOnly)); // telemetry
        assert!(view.write(0x80, &[1, 2, 3, 4]).is_ok()); // control - OK
        assert!(view.write(0x100, &[5, 6, 7, 8]).is_ok()); // config - OK
    }

    #[test]
    fn test_kernel_view_read_write() {
        let mut table = ShadowTable::<512>::new();

        // Pre-populate some data
        table.as_bytes_mut()[0x80..0x84].copy_from_slice(&[1, 2, 3, 4]);

        let (bytes, dirty) = table.as_mut_slices();
        let mut view = KernelView::new(bytes, dirty);

        // Kernel can read control/config
        let mut buf = [0u8; 4];
        assert!(view.read_ctrl(0x80, &mut buf).is_ok());
        assert_eq!(buf, [1, 2, 3, 4]);

        // Kernel cannot read telemetry region (should use internal state)
        assert_eq!(view.read_ctrl(0x00, &mut buf), Err(ShadowError::ReadOnly));

        // Kernel can write to telemetry region
        assert!(view.write_telem(0x00, &[10, 20, 30, 40]).is_ok());

        // Kernel cannot write to control region via write_telem
        assert_eq!(view.write_telem(0x80, &[1, 2]), Err(ShadowError::WriteOnly));

        // write_telem does NOT mark dirty
        assert!(!view.ctrl_dirty());
    }

    #[test]
    fn test_host_view_marks_dirty_kernel_view_reads() {
        let mut table = ShadowTable::<512>::new();

        // Host writes to control region
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut host = HostView::new(bytes, dirty);
            host.write(0x80, &[0xAB, 0xCD]).unwrap();
        }

        // Kernel checks dirty and reads
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut kernel = KernelView::new(bytes, dirty);

            assert!(kernel.ctrl_dirty());
            assert!(kernel.is_range_dirty(0x80, 2));

            let mut buf = [0u8; 2];
            kernel.read_ctrl(0x80, &mut buf).unwrap();
            assert_eq!(buf, [0xAB, 0xCD]);

            // Clear per-field
            kernel.clear_range_dirty(0x80, 2);
            assert!(!kernel.is_range_dirty(0x80, 2));
            assert!(!kernel.ctrl_dirty());
        }
    }

    #[test]
    fn test_kernel_telem_write_no_dirty() {
        let mut table = ShadowTable::<512>::new();

        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut kernel = KernelView::new(bytes, dirty);

            // Write telemetry
            kernel.write_telem(0x00, &[1, 2, 3, 4]).unwrap();

            // No dirty bits set
            assert!(!kernel.ctrl_dirty());
            assert!(!kernel.is_range_dirty(0x00, 4));
        }

        // Data was written
        let mut buf = [0u8; 4];
        table.read(0x00, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
    }
}
