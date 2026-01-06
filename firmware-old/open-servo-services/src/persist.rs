//! EEPROM persistence service.
//!
//! Saves individual EEPROM fields to internal flash using
//! `sequential-storage` for wear-leveling.
//!
//! ## Design
//!
//! - **Per-field persistence**: Each EEPROM field is stored independently
//! - **Semantic keys**: Uses field name_hash (FNV-1a) as key, stable across address changes
//! - **Length validation**: Detects field size changes on restore
//! - **Executor-agnostic**: Pure async, no runtime dependencies
//! - **Diff-based writes**: Only writes changed fields to minimize flash wear
//! - **Signal-driven**: Two signals for Save and FactoryReset operations
//!
//! ## Usage
//!
//! ```rust,ignore
//! // Firmware provides signals and context
//! let signals = PersistSignals::new(save_rx, reset_rx);
//! let mut task = PersistTask::new(flash, 0..FLASH_SIZE, signals);
//!
//! // Create context with shadow and timer
//! let ctx = ServiceCtx::new(&shadow, &timer);
//!
//! task.init(&ctx).await;  // Restore from flash on boot
//!
//! // Service loop waits on signals:
//! loop {
//!     task.handle_once(&ctx).await;  // Waits for signal, then executes
//! }
//!
//! // Producers signal ops:
//! save_writer.signal(());                    // trigger save
//! reset_writer.signal(ResetLevel::ExceptId); // trigger factory reset
//! ```

use core::ops::Range;

use embedded_storage_async::nor_flash::MultiwriteNorFlash;
use futures::future::Either;
use futures::pin_mut;
use open_servo_hw::v2::{AsyncTimer, PersistR, ResetR};
use open_servo_registry::{eeprom_idx_by_hash, RegSpec, EEPROM_COUNT, EEPROM_FIELDS};
use open_servo_shadow::HostShadow;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{self, Key, SerializationError, Value};

use crate::service_ctx::ServiceCtx;
use crate::task::ServiceTask;

// Re-export from hw for convenience
pub use open_servo_hw::v2::ResetLevel;

/// Check if a field address should be preserved at a given reset level.
///
/// Returns `true` if the field should NOT be deleted (preserved).
#[inline]
fn should_preserve(level: ResetLevel, addr: u16) -> bool {
    use open_servo_registry::dxl::addr::{BAUD_RATE, ID};
    match level {
        ResetLevel::All => false,
        ResetLevel::ExceptId => addr == ID,
        ResetLevel::ExceptIdBaud => addr == ID || addr == BAUD_RATE,
    }
}

/// Bundled signals for persist service operations.
///
/// Combines Save and FactoryReset signals into a single struct
/// to simplify type parameters in [`PersistTask`] and [`Services`](crate::Services).
pub struct PersistSignals<PR, RR>
where
    PR: PersistR,
    RR: ResetR,
{
    /// Signal for Save operations.
    pub save: PR,
    /// Signal for FactoryReset operations.
    pub reset: RR,
}

impl<PR, RR> PersistSignals<PR, RR>
where
    PR: PersistR,
    RR: ResetR,
{
    /// Create a new signal bundle.
    pub fn new(save: PR, reset: RR) -> Self {
        Self { save, reset }
    }
}

/// Maximum field data size (largest EEPROM field is 4 bytes).
const MAX_FIELD_LEN: usize = 8;

/// Torque enable register address (RAM region, offset 64).
const TORQUE_ENABLE_ADDR: u16 = 64;

/// Persist result.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PersistResult {
    /// Successfully persisted.
    Ok,
    /// Refused because servo is engaged (torque on).
    Busy,
    /// Write failed (hardware error).
    WriteFailed,
    /// Nothing changed, no write needed.
    NoChange,
    /// First boot, no stored data.
    FirstBoot,
    /// Some fields had length mismatch (registry changed).
    LengthMismatch,
}

/// Cache entry for a single field.
#[derive(Clone, Copy)]
struct CacheEntry {
    /// Field data.
    data: [u8; MAX_FIELD_LEN],
    /// Actual length of valid data.
    len: u8,
}

impl CacheEntry {
    /// Create an empty cache entry (erased flash state).
    const fn empty() -> Self {
        Self {
            data: [0xFF; MAX_FIELD_LEN],
            len: 0,
        }
    }

    /// Create from data slice.
    fn from_slice(data: &[u8]) -> Self {
        let mut entry = Self::empty();
        let len = data.len().min(MAX_FIELD_LEN);
        entry.data[..len].copy_from_slice(&data[..len]);
        entry.len = len as u8;
        entry
    }

    /// Get data slice.
    fn as_slice(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// Compare with another slice.
    fn matches(&self, other: &[u8]) -> bool {
        self.as_slice() == other
    }
}

/// Key for sequential-storage map (field name_hash).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct FieldKey(u32);

impl Key for FieldKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 4 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[..4].copy_from_slice(&self.0.to_le_bytes());
        Ok(4)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.len() < 4 {
            return Err(SerializationError::BufferTooSmall);
        }
        let hash = u32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
        Ok((FieldKey(hash), 4))
    }
}

/// Value for sequential-storage map (length + data).
///
/// Format: [len: u8, data: [u8; len]]
#[derive(Clone, Copy)]
struct FieldValue {
    data: [u8; MAX_FIELD_LEN],
    len: u8,
}

impl FieldValue {
    fn from_slice(data: &[u8]) -> Self {
        let mut value = Self {
            data: [0; MAX_FIELD_LEN],
            len: data.len().min(MAX_FIELD_LEN) as u8,
        };
        value.data[..value.len as usize].copy_from_slice(&data[..value.len as usize]);
        value
    }

    fn as_slice(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

impl<'a> Value<'a> for FieldValue {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let total = 1 + self.len as usize;
        if buffer.len() < total {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = self.len;
        buffer[1..total].copy_from_slice(&self.data[..self.len as usize]);
        Ok(total)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        let len = buffer[0];
        if len as usize > MAX_FIELD_LEN {
            return Err(SerializationError::InvalidFormat);
        }
        let total = 1 + len as usize;
        if buffer.len() < total {
            return Err(SerializationError::BufferTooSmall);
        }
        let mut data = [0u8; MAX_FIELD_LEN];
        data[..len as usize].copy_from_slice(&buffer[1..total]);
        Ok(FieldValue { data, len })
    }
}

/// Persistence task state.
///
/// Generic over flash type and signal types. Shadow access is provided via
/// [`ServiceCtx`] passed to task methods.
///
/// Note: Uses [`MultiwriteNorFlash`] bound (superset of `NorFlash`) to support
/// factory reset's `remove_item` operation.
pub struct PersistTask<F, PR, RR>
where
    F: MultiwriteNorFlash,
    PR: PersistR,
    RR: ResetR,
{
    /// Flash driver.
    flash: F,
    /// Flash address range for storage (relative offsets).
    flash_range: Range<u32>,
    /// Bundled signals for persist operations.
    signals: PersistSignals<PR, RR>,
    /// Cache of last known field values, indexed by position in EEPROM_FIELDS.
    cache: [CacheEntry; EEPROM_COUNT],
    /// Whether init() has been called.
    initialized: bool,
    /// Last persist result.
    last_result: Option<PersistResult>,
    /// Generation counter (increments on successful persist).
    generation: u16,
}

impl<F, PR, RR> PersistTask<F, PR, RR>
where
    F: MultiwriteNorFlash,
    PR: PersistR,
    RR: ResetR,
{
    /// Create a new persist task.
    ///
    /// - `flash`: Flash driver implementing `NorFlash`
    /// - `flash_range`: Address range for storage (relative to flash start)
    /// - `signals`: Bundled Save and FactoryReset signals
    pub fn new(flash: F, flash_range: Range<u32>, signals: PersistSignals<PR, RR>) -> Self {
        Self {
            flash,
            flash_range,
            signals,
            cache: [CacheEntry::empty(); EEPROM_COUNT],
            initialized: false,
            last_result: None,
            generation: 0,
        }
    }

    /// Write persist telemetry to shadow registers.
    fn write_telemetry<S: HostShadow, T: AsyncTimer>(
        &self,
        ctx: &ServiceCtx<'_, S, T>,
        result: PersistResult,
        pending: bool,
    ) {
        use open_servo_registry::vendor::telem;
        ctx.shadow.with_host_view(|view| {
            let r1 = view.write(telem::PERSIST_STATUS, &[result as u8]);
            let r2 = view.write(telem::PERSIST_GEN, &self.generation.to_le_bytes());
            let r3 = view.write(telem::PERSIST_PENDING, &[pending as u8]);
            debug_assert!(r1.is_ok() && r2.is_ok() && r3.is_ok());
        });
    }

    /// Get the last persist result.
    pub const fn last_result(&self) -> Option<PersistResult> {
        self.last_result
    }

    /// Check if initialized.
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Restore a single field from flash to shadow.
    ///
    /// - `shadow`: Shadow table for writing restored values
    /// - `field`: Field specification
    /// - `buf`: Scratch buffer for sequential-storage
    async fn restore_field<S: HostShadow>(
        &mut self,
        shadow: &S,
        field: &RegSpec,
        buf: &mut [u8],
    ) -> Result<bool, ()> {
        let key = FieldKey(field.name_hash);

        let result = map::fetch_item::<FieldKey, FieldValue, _>(
            &mut self.flash,
            self.flash_range.clone(),
            &mut NoCache::new(),
            buf,
            &key,
        )
        .await;

        match result {
            Ok(Some(value)) => {
                // Validate length matches current registry
                let expected_len = field.len();
                if value.len != expected_len {
                    // Length mismatch - registry changed, skip restore
                    #[cfg(feature = "defmt")]
                    defmt::warn!(
                        "Field {} length mismatch: stored={}, expected={}",
                        field.name,
                        value.len,
                        expected_len
                    );
                    return Ok(false);
                }

                // Write to shadow
                if shadow.host_write(field.address, value.as_slice()).is_err() {
                    return Err(());
                }

                // Update cache using hash lookup (should always succeed for EEPROM_FIELDS)
                match eeprom_idx_by_hash(field.name_hash) {
                    Some(idx) => self.cache[idx] = CacheEntry::from_slice(value.as_slice()),
                    None => debug_assert!(false, "EEPROM field {} not in hash map", field.name),
                }

                Ok(true)
            }
            Ok(None) => {
                // Field not stored - first boot for this field
                Ok(false)
            }
            Err(_) => {
                // Flash error
                Err(())
            }
        }
    }

    /// Save a single field to flash.
    ///
    /// - `field`: Field specification
    /// - `data`: Field data to save
    /// - `buf`: Scratch buffer for sequential-storage
    async fn save_field(&mut self, field: &RegSpec, data: &[u8], buf: &mut [u8]) -> Result<(), ()> {
        let key = FieldKey(field.name_hash);
        let value = FieldValue::from_slice(data);

        let result = map::store_item::<FieldKey, FieldValue, _>(
            &mut self.flash,
            self.flash_range.clone(),
            &mut NoCache::new(),
            buf,
            &key,
            &value,
        )
        .await;

        match result {
            Ok(()) => {
                // Update cache using hash lookup (should always succeed for EEPROM_FIELDS)
                match eeprom_idx_by_hash(field.name_hash) {
                    Some(idx) => self.cache[idx] = CacheEntry::from_slice(data),
                    None => debug_assert!(false, "EEPROM field {} not in hash map", field.name),
                }
                Ok(())
            }
            Err(_) => Err(()),
        }
    }

    /// Snapshot all EEPROM field values from shadow in a single critical section.
    ///
    /// Returns `Ok((snapshot, read_errors))` if torque is disabled.
    /// Returns `Err(PersistResult)` for early exit (busy or read error).
    fn take_snapshot<S: HostShadow>(
        &self,
        shadow: &S,
    ) -> Result<([CacheEntry; EEPROM_COUNT], u8), PersistResult> {
        let mut snapshot = [CacheEntry::empty(); EEPROM_COUNT];
        let mut read_errors = 0u8;

        let result = shadow.with_host_view(|view| {
            // Check torque enable first
            let mut torque_buf = [0u8; 1];
            if view.read(TORQUE_ENABLE_ADDR, &mut torque_buf).is_err() {
                return Err(PersistResult::WriteFailed);
            }
            if torque_buf[0] != 0 {
                return Err(PersistResult::Busy);
            }

            // Read all EEPROM fields into snapshot
            let mut data = [0u8; MAX_FIELD_LEN];
            for field in EEPROM_FIELDS.iter() {
                let len = field.len() as usize;
                if len > MAX_FIELD_LEN {
                    continue;
                }

                let idx = match eeprom_idx_by_hash(field.name_hash) {
                    Some(idx) => idx,
                    None => {
                        debug_assert!(false, "EEPROM field {} not in hash map", field.name);
                        continue;
                    }
                };

                if view.read(field.address, &mut data[..len]).is_ok() {
                    snapshot[idx] = CacheEntry::from_slice(&data[..len]);
                } else {
                    read_errors += 1;
                }
            }

            Ok(())
        });

        match result {
            Ok(()) => Ok((snapshot, read_errors)),
            Err(e) => Err(e),
        }
    }

    /// Compare snapshot to cache and save changed fields to flash.
    ///
    /// Returns `(saved_count, error_count)`.
    async fn save_changed_fields<S: HostShadow, T: AsyncTimer>(
        &mut self,
        ctx: &ServiceCtx<'_, S, T>,
        snapshot: &[CacheEntry; EEPROM_COUNT],
    ) -> (usize, usize) {
        let mut buf = [0u8; 64];
        let mut saved = 0;
        let mut errors = 0;

        for (idx, field) in EEPROM_FIELDS.iter().enumerate() {
            let len = field.len() as usize;
            if len > MAX_FIELD_LEN || snapshot[idx].len == 0 {
                continue;
            }

            // Compare snapshot to cache
            if self.cache[idx].matches(snapshot[idx].as_slice()) {
                continue;
            }

            // Field changed, save it
            if saved == 0 {
                self.write_telemetry(ctx, PersistResult::Ok, true);
            }

            match self
                .save_field(field, snapshot[idx].as_slice(), &mut buf)
                .await
            {
                Ok(()) => saved += 1,
                Err(_) => errors += 1,
            }
        }

        (saved, errors)
    }
}

// =============================================================================
// Operation Handlers
// =============================================================================

impl<F, PR, RR> PersistTask<F, PR, RR>
where
    F: MultiwriteNorFlash,
    PR: PersistR,
    RR: ResetR,
{
    /// Handle Save operation: save dirty EEPROM fields to flash.
    async fn handle_save<S: HostShadow, T: AsyncTimer>(&mut self, ctx: &ServiceCtx<'_, S, T>) {
        // Take snapshot (single CS for torque check + all field reads)
        let (snapshot, read_errors) = match self.take_snapshot(ctx.shadow) {
            Ok(result) => result,
            Err(result) => {
                self.last_result = Some(result);
                self.write_telemetry(ctx, result, false);
                #[cfg(feature = "defmt")]
                if result == PersistResult::WriteFailed {
                    defmt::error!("Persist failed: torque read error");
                }
                return;
            }
        };

        // Save changed fields (outside CS)
        let (saved, write_errors) = self.save_changed_fields(ctx, &snapshot).await;

        // Report results
        let total_errors = read_errors as usize + write_errors;
        if total_errors > 0 {
            self.last_result = Some(PersistResult::WriteFailed);
            self.write_telemetry(ctx, PersistResult::WriteFailed, false);
            #[cfg(feature = "defmt")]
            defmt::error!("Persist failed: {} errors", total_errors);
        } else if saved > 0 {
            self.generation = self.generation.wrapping_add(1);
            self.last_result = Some(PersistResult::Ok);
            self.write_telemetry(ctx, PersistResult::Ok, false);
        } else {
            self.last_result = Some(PersistResult::NoChange);
            self.write_telemetry(ctx, PersistResult::NoChange, false);
        }
    }

    /// Handle FactoryReset operation: delete non-preserved EEPROM keys.
    ///
    /// Deletes keys based on [`ResetLevel`], then triggers soft reset.
    async fn handle_factory_reset<S: HostShadow, T: AsyncTimer>(
        &mut self,
        ctx: &ServiceCtx<'_, S, T>,
        level: ResetLevel,
    ) {
        let mut buf = [0u8; 64];
        #[cfg(feature = "defmt")]
        let mut deleted = 0usize;
        let mut errors = 0usize;

        // Delete non-preserved EEPROM keys
        for field in EEPROM_FIELDS.iter() {
            if should_preserve(level, field.address) {
                continue;
            }

            let key = FieldKey(field.name_hash);
            let result = map::remove_item::<FieldKey, _>(
                &mut self.flash,
                self.flash_range.clone(),
                &mut NoCache::new(),
                &mut buf,
                &key,
            )
            .await;

            match result {
                Ok(()) => {
                    #[cfg(feature = "defmt")]
                    {
                        deleted += 1;
                    }
                    // Clear cache for this field
                    if let Some(idx) = eeprom_idx_by_hash(field.name_hash) {
                        self.cache[idx] = CacheEntry::empty();
                    }
                }
                Err(_) => errors += 1,
            }
        }

        #[cfg(feature = "defmt")]
        defmt::info!("Factory reset: deleted={}, errors={}", deleted, errors);

        if errors > 0 {
            self.last_result = Some(PersistResult::WriteFailed);
            self.write_telemetry(ctx, PersistResult::WriteFailed, false);
        } else {
            self.last_result = Some(PersistResult::Ok);
            self.write_telemetry(ctx, PersistResult::Ok, false);
        }

        // TODO: Trigger soft reset via KernelOp after deletion
        // For now, caller must handle reset separately
    }
}

// =============================================================================
// ServiceTask implementation
// =============================================================================

impl<F, PR, RR, S, T> ServiceTask<ServiceCtx<'_, S, T>> for PersistTask<F, PR, RR>
where
    F: MultiwriteNorFlash,
    PR: PersistR,
    RR: ResetR,
    S: HostShadow,
    T: AsyncTimer,
{
    async fn init(&mut self, ctx: &ServiceCtx<'_, S, T>) {
        // Buffer for sequential-storage (key=4 + len=1 + data=8 + overhead)
        let mut buf = [0u8; 64];

        let mut restored = 0;
        let mut mismatched = 0;
        let mut errors = 0;

        // Restore each EEPROM field
        for field in EEPROM_FIELDS.iter() {
            match self.restore_field(ctx.shadow, field, &mut buf).await {
                Ok(true) => restored += 1,
                Ok(false) => mismatched += 1,
                Err(_) => errors += 1,
            }
        }

        if errors > 0 {
            // Some flash errors
            self.last_result = Some(PersistResult::WriteFailed);
            self.write_telemetry(ctx, PersistResult::WriteFailed, false);
            #[cfg(feature = "defmt")]
            defmt::error!("Persist init: {} errors restoring fields", errors);
        } else if restored == 0 {
            // First boot - no stored data
            self.initialized = true;
            self.last_result = Some(PersistResult::FirstBoot);
            self.write_telemetry(ctx, PersistResult::FirstBoot, false);
        } else if mismatched > 0 {
            // Some fields had length mismatch
            self.initialized = true;
            self.last_result = Some(PersistResult::LengthMismatch);
            self.write_telemetry(ctx, PersistResult::LengthMismatch, false);
            #[cfg(feature = "defmt")]
            defmt::warn!("Persist init: {} fields length mismatch", mismatched);
        } else {
            // All good
            self.initialized = true;
            self.last_result = Some(PersistResult::Ok);
            self.write_telemetry(ctx, PersistResult::Ok, false);
        }
    }

    async fn handle_once(&mut self, ctx: &ServiceCtx<'_, S, T>) {
        // Wait for either signal using select, then drop futures before handling
        let op = {
            let save_fut = self.signals.save.wait();
            let reset_fut = self.signals.reset.wait();
            pin_mut!(save_fut);
            pin_mut!(reset_fut);

            match futures::future::select(save_fut, reset_fut).await {
                Either::Left(((), _)) => None,
                Either::Right((level, _)) => Some(level),
            }
        };

        // Futures dropped, now safe to call handlers
        match op {
            None => self.handle_save(ctx).await,
            Some(level) => self.handle_factory_reset(ctx, level).await,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_field_key_serialization() {
        let key = FieldKey(0x12345678);
        let mut buf = [0u8; 8];

        let len = key.serialize_into(&mut buf).unwrap();
        assert_eq!(len, 4);
        assert_eq!(&buf[..4], &0x12345678u32.to_le_bytes());

        let (deserialized, dlen) = FieldKey::deserialize_from(&buf).unwrap();
        assert_eq!(dlen, 4);
        assert_eq!(deserialized.0, 0x12345678);
    }

    #[test]
    fn test_field_value_serialization() {
        let data = [0x42, 0x43, 0x44, 0x45];
        let value = FieldValue::from_slice(&data);
        let mut buf = [0u8; 16];

        let len = value.serialize_into(&mut buf).unwrap();
        assert_eq!(len, 5); // 1 byte len + 4 bytes data
        assert_eq!(buf[0], 4); // length
        assert_eq!(&buf[1..5], &data);

        let deserialized = FieldValue::deserialize_from(&buf[..len]).unwrap();
        assert_eq!(deserialized.len, 4);
        assert_eq!(deserialized.as_slice(), &data);
    }

    #[test]
    fn test_cache_entry() {
        let data = [0x01, 0x02, 0x03];
        let entry = CacheEntry::from_slice(&data);

        assert_eq!(entry.len, 3);
        assert_eq!(entry.as_slice(), &data);
        assert!(entry.matches(&data));
        assert!(!entry.matches(&[0x01, 0x02, 0x04]));
        assert!(!entry.matches(&[0x01, 0x02]));
    }

    #[test]
    fn test_eeprom_fields_available() {
        // Verify EEPROM_FIELDS is accessible and non-empty
        assert!(!EEPROM_FIELDS.is_empty());

        // Verify all fields have name_hash
        for field in EEPROM_FIELDS {
            assert_ne!(field.name_hash, 0);
        }
    }

    #[test]
    fn test_cache_matches_eeprom_fields() {
        // Cache size should match EEPROM_FIELDS count
        assert_eq!(
            EEPROM_FIELDS.len(),
            EEPROM_COUNT,
            "EEPROM_FIELDS.len() should equal EEPROM_COUNT"
        );
    }
}
