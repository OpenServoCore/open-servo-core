//! Shadow table mechanism for host-visible register/telemetry access.
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

#![no_std]

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
        const {
            assert!(
                N <= Self::MAX_TABLE_SIZE,
                "ShadowTable size exceeds dirty tracking capacity (max 1024 bytes)"
            )
        };

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
// Region Layout (Dynamixel Protocol 2.0)
// ============================================================================

/// Region layout constants (Dynamixel Protocol 2.0 compatible).
///
/// The shadow table provides 1:1 mapping to Dynamixel protocol addresses.
/// Access control (R/Rw/RwEepromLocked) is per-register and handled by
/// the registry crate, not by region.
pub mod layout {
    /// Total shadow table size (protocol address space).
    pub const TABLE_SIZE: u16 = 1024;

    /// EEPROM region: Non-volatile configuration. Bytes 0x000..0x03F.
    /// Host RW when torque disabled, kernel reads.
    pub const EEPROM_START: u16 = 0x000;
    pub const EEPROM_LEN: u16 = 64;

    /// RAM region: Runtime state and control. Bytes 0x040..0x0FB.
    /// Mixed R/Rw per register (defined in registry).
    pub const RAM_START: u16 = 0x040;
    pub const RAM_LEN: u16 = 188; // 0x40..0xFB = 188 bytes

    /// Reserved region. Bytes 0x0FC..0x1FF.
    pub const RESERVED_START: u16 = 0x0FC;
    pub const RESERVED_LEN: u16 = 260;

    /// Vendor region: High-resolution and debug. Bytes 0x200..0x3FF.
    /// Mixed R/Rw per register (defined in registry).
    pub const VENDOR_START: u16 = 0x200;
    pub const VENDOR_LEN: u16 = 512;

    /// Check if address is in EEPROM region.
    #[inline]
    pub const fn is_eeprom(offset: u16) -> bool {
        offset < EEPROM_START + EEPROM_LEN
    }

    /// Check if address is in RAM region.
    #[inline]
    pub const fn is_ram(offset: u16) -> bool {
        offset >= RAM_START && offset < RAM_START + RAM_LEN
    }

    /// Check if address is in vendor region.
    #[inline]
    pub const fn is_vendor(offset: u16) -> bool {
        offset >= VENDOR_START && offset < VENDOR_START + VENDOR_LEN
    }

    /// Check if address is valid (in any defined region).
    #[inline]
    pub const fn is_valid(offset: u16) -> bool {
        offset < TABLE_SIZE
    }

    /// Check if a range is entirely within table bounds.
    #[inline]
    pub const fn is_range_valid(offset: u16, len: u16) -> bool {
        if len == 0 {
            return true;
        }
        let end = offset.saturating_add(len);
        end <= TABLE_SIZE
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

    /// Write bytes to shadow table.
    ///
    /// Marks affected blocks as dirty. Access control (R/Rw/RwEepromLocked)
    /// is handled by the registry/protocol layer, not here.
    pub fn write(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        let len = data.len() as u16;
        if !layout::is_range_valid(offset, len) {
            return Err(ShadowError::OutOfBounds);
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
    /// Access control is handled by the registry/protocol layer.
    pub fn stage_write_range(&mut self, offset: u16, data: &[u8]) -> StageResult {
        let len = data.len() as u16;
        if !layout::is_range_valid(offset, len) {
            return StageResult::OutOfBounds;
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

    /// Read bytes from shadow table.
    ///
    /// Kernel can read any address to get host-written values.
    pub fn read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        let len = buf.len() as u16;
        if !layout::is_range_valid(offset, len) {
            return Err(ShadowError::OutOfBounds);
        }
        let start = offset as usize;
        let end = start.saturating_add(buf.len());
        if end > self.bytes.len() {
            return Err(ShadowError::OutOfBounds);
        }
        buf.copy_from_slice(&self.bytes[start..end]);
        Ok(())
    }

    /// Write bytes to shadow table. **Does NOT mark dirty.**
    ///
    /// Used for publishing telemetry (read-only registers from host perspective).
    /// The registry defines which registers are R vs Rw.
    pub fn write(&mut self, offset: u16, data: &[u8]) -> Result<(), ShadowError> {
        let len = data.len() as u16;
        if !layout::is_range_valid(offset, len) {
            return Err(ShadowError::OutOfBounds);
        }
        let start = offset as usize;
        let end = start.saturating_add(data.len());
        if end > self.bytes.len() {
            return Err(ShadowError::OutOfBounds);
        }
        self.bytes[start..end].copy_from_slice(data);
        // NOTE: No dirty marking for kernel writes (telemetry publishing)
        Ok(())
    }

    /// Check if any block in the table has pending writes.
    pub fn any_dirty(&self) -> bool {
        self.dirty.iter().any(|&w| w != 0)
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

    /// Mark dirty bits for a specific byte range.
    ///
    /// Used by facade translation to mark vendor targets dirty after writes.
    pub fn mark_range_dirty(&mut self, offset: u16, len: u16) {
        if len == 0 {
            return;
        }
        let start_block = offset as usize / DIRTY_BLOCK_SIZE;
        let end_block = (offset as usize + len as usize).saturating_sub(1) / DIRTY_BLOCK_SIZE;
        for block in start_block..=end_block {
            let word_idx = block / 32;
            let bit_idx = block % 32;
            if word_idx < self.dirty.len() {
                self.dirty[word_idx] |= 1 << bit_idx;
            }
        }
    }
}

// ============================================================================
// Kernel Shadow Trait
// ============================================================================

// Re-export CommitResult from kernel-api for convenience
pub use open_servo_kernel_api::CommitResult;

/// Kernel commit contract.
///
/// Kernels implement this to:
/// 1. Check which dirty ranges overlap known fields via `view.is_range_dirty()`
/// 2. Read shadow bytes via `view.read()`
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
    /// Writes kernel state to telemetry region via `view.write()`.
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

// ============================================================================
// Host Shadow Trait
// ============================================================================

/// Host-side shadow access trait.
///
/// This trait abstracts host-side shadow table access for services.
/// The implementor handles:
/// - Thread safety (e.g., critical sections)
/// - Persist signaling (auto-trigger save on EEPROM writes)
///
/// # Example
///
/// ```rust,ignore
/// fn read_register<S: HostShadow>(shadow: &S, addr: u16) -> u32 {
///     let mut buf = [0u8; 4];
///     shadow.host_read(addr, &mut buf).unwrap();
///     u32::from_le_bytes(buf)
/// }
/// ```
pub trait HostShadow {
    /// Staging buffer type used by the implementation.
    type Staging: StagingBuffer;

    /// Execute a closure with scoped access to [`HostView`].
    ///
    /// The implementor ensures thread-safety (e.g., via critical section).
    /// The `HostView` is only valid within the closure scope.
    fn with_host_view<R>(&self, f: impl FnOnce(&mut HostView<'_, Self::Staging>) -> R) -> R;

    /// Read bytes from shadow table.
    fn host_read(&self, offset: u16, buf: &mut [u8]) -> Result<(), ShadowError>;

    /// Write bytes to shadow table.
    ///
    /// The implementor should auto-signal persist if EEPROM region is touched.
    fn host_write(&self, offset: u16, data: &[u8]) -> Result<(), ShadowError>;

    /// Stage a write for later ACTION commit.
    fn host_stage(&self, offset: u16, data: &[u8]) -> StageResult;

    /// Apply all staged writes atomically.
    ///
    /// The implementor should auto-signal persist if EEPROM was touched.
    /// Returns the number of writes applied.
    fn host_action(&self) -> usize;
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

        assert_eq!(
            table.write(60, &[1, 2, 3, 4, 5]),
            Err(ShadowError::OutOfBounds)
        );
        assert_eq!(table.read(60, &mut [0u8; 5]), Err(ShadowError::OutOfBounds));
    }

    #[test]
    fn test_layout_region_checks() {
        // EEPROM region: 0x00..0x3F
        assert!(layout::is_eeprom(0x00));
        assert!(layout::is_eeprom(0x3F));
        assert!(!layout::is_eeprom(0x40));

        // RAM region: 0x40..0xFB
        assert!(layout::is_ram(0x40));
        assert!(layout::is_ram(0xFB));
        assert!(!layout::is_ram(0xFC));

        // Vendor region: 0x200..0x3FF
        assert!(layout::is_vendor(0x200));
        assert!(layout::is_vendor(0x3FF));
        assert!(!layout::is_vendor(0x400));

        // Valid address checks
        assert!(layout::is_valid(0x00));
        assert!(layout::is_valid(0x3FF));
        assert!(!layout::is_valid(0x400));

        // Range checks
        assert!(layout::is_range_valid(0x00, 64));
        assert!(layout::is_range_valid(0x200, 512));
        assert!(!layout::is_range_valid(0x3F0, 32)); // extends past 1024
    }

    #[test]
    fn test_host_view_read_write() {
        let mut table = ShadowTable::<1024>::new();
        let (bytes, dirty) = table.as_mut_slices();
        let mut view = HostView::new(bytes, dirty);

        // Host can read anywhere
        let mut buf = [0u8; 4];
        assert!(view.read(0x00, &mut buf).is_ok()); // EEPROM
        assert!(view.read(0x40, &mut buf).is_ok()); // RAM
        assert!(view.read(0x200, &mut buf).is_ok()); // Vendor

        // Host can write anywhere (access control in registry)
        assert!(view.write(0x00, &[1, 2, 3, 4]).is_ok()); // EEPROM
        assert!(view.write(0x40, &[5, 6, 7, 8]).is_ok()); // RAM
        assert!(view.write(0x200, &[9, 10, 11, 12]).is_ok()); // Vendor
    }

    #[test]
    fn test_kernel_view_read_write() {
        let mut table = ShadowTable::<1024>::new();

        // Pre-populate some data at RAM address (Goal Position = 116)
        table.as_bytes_mut()[116..120].copy_from_slice(&[1, 2, 3, 4]);

        let (bytes, dirty) = table.as_mut_slices();
        let mut view = KernelView::new(bytes, dirty);

        // Kernel can read anywhere
        let mut buf = [0u8; 4];
        assert!(view.read(116, &mut buf).is_ok());
        assert_eq!(buf, [1, 2, 3, 4]);

        // Kernel can write anywhere (for telemetry publishing)
        assert!(view.write(132, &[10, 20, 30, 40]).is_ok()); // Present Position

        // Kernel write does NOT mark dirty
        assert!(!view.any_dirty());
    }

    #[test]
    fn test_host_view_marks_dirty_kernel_view_reads() {
        let mut table = ShadowTable::<1024>::new();

        // Host writes to RAM region (Goal Position = 116)
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut host = HostView::new(bytes, dirty);
            host.write(116, &[0xAB, 0xCD, 0xEF, 0x12]).unwrap();
        }

        // Kernel checks dirty and reads
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut kernel = KernelView::new(bytes, dirty);

            assert!(kernel.any_dirty());
            assert!(kernel.is_range_dirty(116, 4));

            let mut buf = [0u8; 4];
            kernel.read(116, &mut buf).unwrap();
            assert_eq!(buf, [0xAB, 0xCD, 0xEF, 0x12]);

            // Clear per-field
            kernel.clear_range_dirty(116, 4);
            assert!(!kernel.is_range_dirty(116, 4));
            assert!(!kernel.any_dirty());
        }
    }

    #[test]
    fn test_kernel_write_no_dirty() {
        let mut table = ShadowTable::<1024>::new();

        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut kernel = KernelView::new(bytes, dirty);

            // Write to Present Position (132) - read-only from host perspective
            kernel.write(132, &[1, 2, 3, 4]).unwrap();

            // No dirty bits set
            assert!(!kernel.any_dirty());
            assert!(!kernel.is_range_dirty(132, 4));
        }

        // Data was written
        let mut buf = [0u8; 4];
        table.read(132, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
    }

    #[test]
    fn test_1024_byte_table() {
        let table = ShadowTable::<1024>::new();
        assert_eq!(table.size(), 1024);

        // Should be able to access vendor region at end
        let mut buf = [0u8; 4];
        assert!(table.read(0x3FC, &mut buf).is_ok()); // Last 4 bytes
    }

    #[test]
    fn test_kernel_view_mark_range_dirty() {
        let mut table = ShadowTable::<1024>::new();

        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut kernel = KernelView::new(bytes, dirty);

            // Initially no dirty bits
            assert!(!kernel.any_dirty());

            // Mark a single block dirty (offset 0x50, len 4 = block 5)
            kernel.mark_range_dirty(0x50, 4);
            assert!(kernel.is_range_dirty(0x50, 4));
            assert!(kernel.any_dirty());

            // Clear and verify
            kernel.clear_range_dirty(0x50, 4);
            assert!(!kernel.is_range_dirty(0x50, 4));
            assert!(!kernel.any_dirty());

            // Mark multiple blocks dirty (offset 0x10, len 20 spans blocks 1 and 2)
            kernel.mark_range_dirty(0x10, 20);
            assert!(kernel.is_range_dirty(0x10, 20));
            assert!(kernel.is_range_dirty(0x10, 1)); // block 1
            assert!(kernel.is_range_dirty(0x20, 1)); // block 2

            // Zero length should be no-op
            kernel.clear_range_dirty(0, 1024);
            kernel.mark_range_dirty(0x100, 0);
            assert!(!kernel.is_range_dirty(0x100, 4));
        }
    }
}
