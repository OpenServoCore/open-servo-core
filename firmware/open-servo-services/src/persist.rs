//! Persistence service task.
//!
//! This embassy task handles saving the EEPROM region (64 bytes) to internal
//! flash using `sequential-storage` for wear-leveling.
//!
//! ## Architecture
//!
//! ```text
//! dxl_req_task → Signal<PersistRequest> → persist_task → sequential-storage → Flash
//! ```
//!
//! ## Safety
//!
//! Persistence is only allowed when the servo is disengaged (torque off).
//! The task checks this condition before starting a write operation.
//!
//! ## Write Strategy
//!
//! The shadow table's EEPROM region is compared against stored values.
//! Only changed bytes trigger a write to minimize wear.

use core::ops::Range;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_storage_async::nor_flash::NorFlash;
use open_servo_device::shadow_storage::ShadowStorage;
use open_servo_shadow::layout::{EEPROM_LEN, EEPROM_START};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{self, Key, SerializationError, Value};

/// EEPROM region size in bytes.
pub const EEPROM_SIZE: usize = EEPROM_LEN as usize;

/// Torque enable register address (RAM region, offset 64).
const TORQUE_ENABLE_ADDR: u16 = 64;

/// Signal type for persist request (req task → persist task).
pub type PersistSignal = Signal<CriticalSectionRawMutex, ()>;

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
}

/// Key for sequential-storage map (single EEPROM blob).
///
/// We use a single key (0) to store the entire 64-byte EEPROM region.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct EepromKey;

impl Key for EepromKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = 0; // Single key ID
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        if buffer[0] != 0 {
            return Err(SerializationError::InvalidFormat);
        }
        Ok((EepromKey, 1))
    }
}

/// EEPROM data wrapper for sequential-storage Value trait.
#[derive(Clone, Copy)]
struct EepromData([u8; EEPROM_SIZE]);

impl<'a> Value<'a> for EepromData {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < EEPROM_SIZE {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[..EEPROM_SIZE].copy_from_slice(&self.0);
        Ok(EEPROM_SIZE)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < EEPROM_SIZE {
            return Err(SerializationError::BufferTooSmall);
        }
        let mut data = [0u8; EEPROM_SIZE];
        data.copy_from_slice(&buffer[..EEPROM_SIZE]);
        Ok(EepromData(data))
    }
}

/// Persistence task state.
///
/// Generic over the flash type to allow testing with mock flash.
pub struct PersistTask<F: NorFlash> {
    /// Flash driver.
    flash: F,
    /// Flash address range for storage (relative offsets).
    flash_range: Range<u32>,
    /// Last known EEPROM data (for diff-based writes).
    last_known: [u8; EEPROM_SIZE],
    /// Whether init() has been called.
    initialized: bool,
    /// Last persist result.
    last_result: Option<PersistResult>,
}

impl<F: NorFlash> PersistTask<F> {
    /// Create a new persist task.
    ///
    /// `flash` is the flash driver implementing `NorFlash`.
    /// `flash_range` is the address range for storage (relative to flash start).
    pub fn new(flash: F, flash_range: Range<u32>) -> Self {
        Self {
            flash,
            flash_range,
            last_known: [0xFF; EEPROM_SIZE], // Erased flash state
            initialized: false,
            last_result: None,
        }
    }

    /// Initialize from flash (restore EEPROM region on boot).
    ///
    /// Call this once at startup to load saved data into shadow table.
    pub async fn init<const N: usize>(
        &mut self,
        shadow: &ShadowStorage<N>,
    ) -> Result<(), PersistResult> {
        // Buffer for sequential-storage (key + value + overhead)
        let mut buf = [0u8; EEPROM_SIZE + 32];
        let mut cache = NoCache::new();

        match map::fetch_item::<EepromKey, EepromData, _>(
            &mut self.flash,
            self.flash_range.clone(),
            &mut cache,
            &mut buf,
            &EepromKey,
        )
        .await
        {
            Ok(Some(data)) => {
                // Restore to shadow table
                self.last_known = data.0;
                shadow
                    .host_write(EEPROM_START, &self.last_known)
                    .map_err(|_| PersistResult::WriteFailed)?;
                self.initialized = true;
                self.last_result = Some(PersistResult::Ok);
                Ok(())
            }
            Ok(None) => {
                // First boot - no saved data
                self.initialized = true;
                self.last_result = Some(PersistResult::FirstBoot);
                Ok(())
            }
            Err(_) => {
                // Flash error during init
                self.last_result = Some(PersistResult::WriteFailed);
                Err(PersistResult::WriteFailed)
            }
        }
    }

    /// Persist EEPROM region if changed.
    ///
    /// Returns the result of the operation.
    pub async fn persist<const N: usize>(&mut self, shadow: &ShadowStorage<N>) -> PersistResult {
        // Check torque enable
        let mut torque_buf = [0u8; 1];
        if shadow
            .host_read(TORQUE_ENABLE_ADDR, &mut torque_buf)
            .is_err()
        {
            self.last_result = Some(PersistResult::WriteFailed);
            return PersistResult::WriteFailed;
        }
        if torque_buf[0] != 0 {
            self.last_result = Some(PersistResult::Busy);
            return PersistResult::Busy;
        }

        // Read current EEPROM region from shadow
        let mut current = [0u8; EEPROM_SIZE];
        if shadow.host_read(EEPROM_START, &mut current).is_err() {
            self.last_result = Some(PersistResult::WriteFailed);
            return PersistResult::WriteFailed;
        }

        // Diff check - skip write if unchanged
        if current == self.last_known {
            self.last_result = Some(PersistResult::NoChange);
            return PersistResult::NoChange;
        }

        // Write to flash
        let mut buf = [0u8; EEPROM_SIZE + 32];
        let mut cache = NoCache::new();

        let result = map::store_item::<EepromKey, EepromData, _>(
            &mut self.flash,
            self.flash_range.clone(),
            &mut cache,
            &mut buf,
            &EepromKey,
            &EepromData(current),
        )
        .await;

        match result {
            Ok(()) => {
                self.last_known = current;
                self.last_result = Some(PersistResult::Ok);
                PersistResult::Ok
            }
            Err(_) => {
                self.last_result = Some(PersistResult::WriteFailed);
                PersistResult::WriteFailed
            }
        }
    }

    /// Get the last persist result.
    pub const fn last_result(&self) -> Option<PersistResult> {
        self.last_result
    }

    /// Check if initialized.
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eeprom_key_serialization() {
        let key = EepromKey;
        let mut buf = [0u8; 2];

        let len = key.serialize_into(&mut buf).unwrap();
        assert_eq!(len, 1);
        assert_eq!(buf[0], 0);

        let (deserialized, dlen) = EepromKey::deserialize_from(&buf).unwrap();
        assert_eq!(dlen, 1);
        assert_eq!(deserialized, key);
    }

    #[test]
    fn test_eeprom_data_serialization() {
        let mut data = [0u8; EEPROM_SIZE];
        data[0] = 0x42;
        data[63] = 0xFF;

        let value = EepromData(data);
        let mut buf = [0u8; EEPROM_SIZE + 16];

        let len = value.serialize_into(&mut buf).unwrap();
        assert_eq!(len, EEPROM_SIZE);
        assert_eq!(buf[0], 0x42);
        assert_eq!(buf[63], 0xFF);

        let deserialized = EepromData::deserialize_from(&buf[..EEPROM_SIZE]).unwrap();
        assert_eq!(deserialized.0, data);
    }
}
