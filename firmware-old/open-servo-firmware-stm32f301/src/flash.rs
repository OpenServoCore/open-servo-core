//! Internal flash driver for STM32F301.
//!
//! Provides `embedded-storage-async` trait implementations for the internal flash,
//! limited to the EEPROM storage region (last 4KB).
//!
//! ## Flash Characteristics
//!
//! - Total: 64KB at 0x0800_0000
//! - Page size: 2KB
//! - Write size: 2 bytes (half-word)
//! - EEPROM region: 0x0800_F000 - 0x0800_FFFF (4KB, 2 pages)
//!
//! ## Implementation Notes
//!
//! Based on embassy-stm32's proven flash driver pattern (f1f3.rs).
//!
//! **IMPORTANT**: HSI must remain enabled for flash programming to work!
//! See RM0313: "For program and erase operations on the Flash memory,
//! the internal RC oscillator (HSI) must be ON."

use core::ptr::write_volatile;
use core::sync::atomic::{fence, Ordering};

use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash, ReadNorFlash};
use stm32f3::stm32f301::FLASH;

/// Flash unlock keys (same as embassy).
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

/// Page size (2KB).
pub const PAGE_SIZE: usize = 2048;

/// Write size (half-word = 2 bytes).
pub const WRITE_SIZE: usize = 2;

/// EEPROM region start address.
pub const EEPROM_FLASH_START: u32 = 0x0800_F000;

/// EEPROM region size (4KB = 2 pages).
pub const EEPROM_FLASH_SIZE: usize = 4 * 1024;

/// Flash error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashError {
    /// Address out of range.
    OutOfBounds,
    /// Programming error (PGERR).
    ProgramError,
    /// Write protection error (WRPRTERR).
    WriteProtection,
    /// Alignment error (offset or length not aligned).
    Alignment,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            FlashError::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            FlashError::ProgramError => NorFlashErrorKind::Other,
            FlashError::WriteProtection => NorFlashErrorKind::Other,
            FlashError::Alignment => NorFlashErrorKind::NotAligned,
        }
    }
}

/// Flash driver for EEPROM storage region.
pub struct EepromFlash {
    _private: (),
}

impl EepromFlash {
    /// Create a new EEPROM flash driver.
    ///
    /// # Safety
    ///
    /// Only one instance should exist at a time.
    pub unsafe fn new() -> Self {
        Self { _private: () }
    }

    /// Get the FLASH peripheral.
    #[inline(always)]
    fn flash(&self) -> &stm32f3::stm32f301::flash::RegisterBlock {
        unsafe { &*FLASH::PTR }
    }

    // =========================================================================
    // Embassy-stm32 f1f3.rs functions (adapted)
    // =========================================================================

    /// Lock flash.
    fn lock(&self) {
        self.flash().cr.modify(|_, w| w.lock().lock());
    }

    /// Unlock flash.
    fn unlock(&self) {
        let flash = self.flash();
        if flash.cr.read().lock().is_locked() {
            flash.keyr.write(|w| w.bits(KEY1));
            flash.keyr.write(|w| w.bits(KEY2));
        }
    }

    /// Enable blocking write.
    ///
    /// NOTE: Must use modify() not write() - stm32f3 PAC's write() starts from
    /// reset value (0x80 = LOCK), which would re-lock the flash!
    fn enable_blocking_write(&self) {
        self.flash().cr.modify(|_, w| w.pg().program());
    }

    /// Disable blocking write.
    fn disable_blocking_write(&self) {
        self.flash().cr.modify(|_, w| w.pg().clear_bit());
    }

    /// Wait for flash operation to complete.
    fn wait_ready_blocking(&self) -> Result<(), FlashError> {
        loop {
            let sr = self.flash().sr.read();

            if !sr.bsy().is_active() {
                if sr.wrprterr().is_error() {
                    return Err(FlashError::WriteProtection);
                }
                if sr.pgerr().is_error() {
                    return Err(FlashError::ProgramError);
                }
                return Ok(());
            }
        }
    }

    /// Clear all error flags.
    fn clear_all_err(&self) {
        self.flash().sr.modify(|_, w| w);
    }

    /// Write bytes to flash (must be half-word aligned).
    fn blocking_write(&self, start_address: u32, buf: &[u8]) -> Result<(), FlashError> {
        let mut address = start_address;
        for chunk in buf.chunks(2) {
            unsafe {
                write_volatile(
                    address as *mut u16,
                    u16::from_le_bytes([chunk[0], chunk[1]]),
                );
            }
            address += chunk.len() as u32;

            // Prevents parallelism errors
            fence(Ordering::SeqCst);

            self.wait_ready_blocking()?;
        }
        Ok(())
    }

    /// Erase a flash sector (page).
    fn blocking_erase_sector(&self, sector_start: u32) -> Result<(), FlashError> {
        let flash = self.flash();

        flash.cr.modify(|_, w| w.per().page_erase());
        flash.ar.write(|w| w.bits(sector_start));
        flash.cr.modify(|_, w| w.strt().start());

        // Wait for at least one clock cycle before reading BSY
        // (STM32F3 errata section 2.2.8)
        let _ = flash.cr.read();

        let mut ret = self.wait_ready_blocking();

        // Check EOP flag
        if !flash.sr.read().eop().is_event() {
            ret = Err(FlashError::ProgramError);
        } else {
            // Clear EOP by writing 1
            flash.sr.write(|w| w.eop().reset());
        }

        flash.cr.modify(|_, w| w.per().clear_bit());
        self.clear_all_err();

        ret
    }

    /// Write chunk with unlock/lock handling.
    fn write_chunk_unlocked(&self, address: u32, chunk: &[u8]) -> Result<(), FlashError> {
        self.clear_all_err();
        fence(Ordering::SeqCst);
        self.unlock();
        fence(Ordering::SeqCst);
        self.enable_blocking_write();
        fence(Ordering::SeqCst);

        let result = self.blocking_write(address, chunk);

        self.disable_blocking_write();
        fence(Ordering::SeqCst);
        self.lock();

        result
    }

    /// Erase sector with unlock/lock handling.
    fn erase_sector_unlocked(&self, sector_start: u32) -> Result<(), FlashError> {
        self.clear_all_err();
        fence(Ordering::SeqCst);
        self.unlock();
        fence(Ordering::SeqCst);

        let result = self.blocking_erase_sector(sector_start);

        self.lock();

        result
    }

    /// Convert offset to absolute address, with bounds check.
    fn offset_to_addr(&self, offset: u32, len: usize) -> Result<u32, FlashError> {
        let end = offset
            .checked_add(len as u32)
            .ok_or(FlashError::OutOfBounds)?;

        if end > EEPROM_FLASH_SIZE as u32 {
            return Err(FlashError::OutOfBounds);
        }

        Ok(EEPROM_FLASH_START + offset)
    }
}

impl ErrorType for EepromFlash {
    type Error = FlashError;
}

impl ReadNorFlash for EepromFlash {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let addr = self.offset_to_addr(offset, bytes.len())?;

        // Direct memory read (flash is memory-mapped)
        let flash_data = unsafe { core::slice::from_raw_parts(addr as *const u8, bytes.len()) };
        bytes.copy_from_slice(flash_data);

        Ok(())
    }

    fn capacity(&self) -> usize {
        EEPROM_FLASH_SIZE
    }
}

impl NorFlash for EepromFlash {
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = PAGE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        // Check alignment
        if from as usize % PAGE_SIZE != 0 || to as usize % PAGE_SIZE != 0 {
            return Err(FlashError::Alignment);
        }

        // Check bounds
        if to > EEPROM_FLASH_SIZE as u32 || from > to {
            return Err(FlashError::OutOfBounds);
        }

        let mut addr = EEPROM_FLASH_START + from;
        let end = EEPROM_FLASH_START + to;

        while addr < end {
            self.erase_sector_unlocked(addr)?;
            addr += PAGE_SIZE as u32;
        }

        Ok(())
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        // Check alignment
        if offset as usize % WRITE_SIZE != 0 || bytes.len() % WRITE_SIZE != 0 {
            return Err(FlashError::Alignment);
        }

        let mut address = self.offset_to_addr(offset, bytes.len())?;

        // Write each WRITE_SIZE chunk
        for chunk in bytes.chunks(WRITE_SIZE) {
            self.write_chunk_unlocked(address, chunk)?;
            address += WRITE_SIZE as u32;
        }

        Ok(())
    }
}

/// Marker trait for multiwrite support.
///
/// STM32F3 flash doesn't support true multiwrite (rewriting arbitrary data),
/// but it CAN write zeros over ones. This is sufficient for `sequential-storage`
/// which only writes deletion markers (0x00 bytes) over existing data.
impl MultiwriteNorFlash for EepromFlash {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_offset_bounds() {
        let flash = unsafe { EepromFlash::new() };

        // Valid offset
        assert!(flash.offset_to_addr(0, 64).is_ok());
        assert!(flash.offset_to_addr(4096 - 64, 64).is_ok());

        // Out of bounds
        assert!(flash.offset_to_addr(4096, 1).is_err());
        assert!(flash.offset_to_addr(0, 4097).is_err());
    }
}
