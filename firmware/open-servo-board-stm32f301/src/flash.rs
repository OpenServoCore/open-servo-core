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

use core::ptr;

use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};
use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};
use stm32f3::stm32f301::FLASH;

/// Flash unlock keys.
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

/// Flash base address.
pub const FLASH_BASE: u32 = 0x0800_0000;

/// Total flash size (64KB).
pub const FLASH_SIZE: u32 = 64 * 1024;

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
///
/// This driver only allows access to the reserved EEPROM region
/// (last 4KB of flash) to prevent accidental code corruption.
///
/// Offsets are relative to the EEPROM region start (0 = 0x0800_F000).
pub struct EepromFlash {
    _private: (),
}

impl EepromFlash {
    /// Create a new EEPROM flash driver.
    ///
    /// # Safety
    ///
    /// Only one instance should exist at a time. The caller must ensure
    /// exclusive access to the flash peripheral.
    pub unsafe fn new() -> Self {
        Self { _private: () }
    }

    /// Get the FLASH peripheral.
    #[inline]
    fn flash(&self) -> &stm32f3::stm32f301::flash::RegisterBlock {
        unsafe { &*FLASH::PTR }
    }

    /// Unlock the flash for programming/erase.
    fn unlock(&self) {
        let flash = self.flash();

        // Check if already unlocked
        if flash.cr.read().lock().is_unlocked() {
            return;
        }

        // Write unlock sequence
        flash.keyr.write(|w| w.bits(KEY1));
        flash.keyr.write(|w| w.bits(KEY2));
    }

    /// Lock the flash to prevent accidental writes.
    fn lock(&self) {
        let flash = self.flash();
        flash.cr.modify(|_, w| w.lock().lock());
    }

    /// Wait for flash operation to complete.
    ///
    /// Returns error if operation failed.
    fn wait_ready(&self) -> Result<(), FlashError> {
        let flash = self.flash();

        // Poll BSY bit
        while flash.sr.read().bsy().is_active() {
            // In an async context, we could yield here
            // For now, busy-wait
            cortex_m::asm::nop();
        }

        // Check for errors
        let sr = flash.sr.read();

        if sr.pgerr().is_error() {
            // Clear error flag
            flash.sr.write(|w| w.pgerr().reset());
            return Err(FlashError::ProgramError);
        }

        if sr.wrprterr().is_error() {
            // Clear error flag
            flash.sr.write(|w| w.wrprterr().reset());
            return Err(FlashError::WriteProtection);
        }

        Ok(())
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

    /// Erase a single page.
    fn erase_page(&self, page_addr: u32) -> Result<(), FlashError> {
        let flash = self.flash();

        // Set page erase mode
        flash.cr.modify(|_, w| w.per().page_erase());

        // Write page address
        flash.ar.write(|w| w.bits(page_addr));

        // Start erase
        flash.cr.modify(|_, w| w.strt().start());

        // Wait for completion
        self.wait_ready()?;

        // Clear page erase mode
        flash.cr.modify(|_, w| w.per().clear_bit());

        Ok(())
    }

    /// Program a half-word (2 bytes).
    fn program_halfword(&self, addr: u32, data: u16) -> Result<(), FlashError> {
        let flash = self.flash();

        // Set programming mode
        flash.cr.modify(|_, w| w.pg().program());

        // Write half-word
        unsafe {
            ptr::write_volatile(addr as *mut u16, data);
        }

        // Wait for completion
        self.wait_ready()?;

        // Clear programming mode
        flash.cr.modify(|_, w| w.pg().clear_bit());

        Ok(())
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
        unsafe {
            ptr::copy_nonoverlapping(addr as *const u8, bytes.as_mut_ptr(), bytes.len());
        }

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

        self.unlock();

        let result = (|| {
            let mut addr = EEPROM_FLASH_START + from;
            let end = EEPROM_FLASH_START + to;

            while addr < end {
                self.erase_page(addr)?;
                addr += PAGE_SIZE as u32;

                // Yield to executor between pages
                // embassy_futures::yield_now().await would go here in true async
            }

            Ok(())
        })();

        self.lock();
        result
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        // Check alignment
        if offset as usize % WRITE_SIZE != 0 || bytes.len() % WRITE_SIZE != 0 {
            return Err(FlashError::Alignment);
        }

        let addr = self.offset_to_addr(offset, bytes.len())?;

        self.unlock();

        let result = (|| {
            let mut current_addr = addr;

            for chunk in bytes.chunks(WRITE_SIZE) {
                // Convert 2 bytes to u16 (little-endian)
                let halfword = u16::from_le_bytes([chunk[0], chunk[1]]);
                self.program_halfword(current_addr, halfword)?;
                current_addr += WRITE_SIZE as u32;
            }

            Ok(())
        })();

        self.lock();
        result
    }
}

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
