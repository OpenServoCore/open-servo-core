#![allow(dead_code)]

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use crate::hal::flash::{self, PAGE_SIZE};

#[derive(Debug)]
pub enum FlashError {
    NotAligned,
    OutOfBounds,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            FlashError::NotAligned => NorFlashErrorKind::NotAligned,
            FlashError::OutOfBounds => NorFlashErrorKind::OutOfBounds,
        }
    }
}

pub struct ChipFlash {
    base: u32,
    size: usize,
}

impl ChipFlash {
    pub const fn new(base: u32, size: usize) -> Self {
        Self { base, size }
    }
}

impl ErrorType for ChipFlash {
    type Error = FlashError;
}

impl ReadNorFlash for ChipFlash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        if offset as usize + bytes.len() > self.size {
            return Err(FlashError::OutOfBounds);
        }
        let src = unsafe { core::slice::from_raw_parts(self.base as *const u8, self.size) };
        let offset = offset as usize;
        bytes.copy_from_slice(&src[offset..offset + bytes.len()]);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.size
    }
}

impl NorFlash for ChipFlash {
    const WRITE_SIZE: usize = PAGE_SIZE;
    const ERASE_SIZE: usize = PAGE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        debug_assert!(
            (from as usize).is_multiple_of(PAGE_SIZE) && (to as usize).is_multiple_of(PAGE_SIZE),
            "erase alignment: from={from}, to={to}"
        );
        debug_assert!(to as usize <= self.size, "erase out of bounds");
        let mut addr = self.base + from;
        let end = self.base + to;
        while addr < end {
            flash::erase(addr);
            addr += PAGE_SIZE as u32;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        debug_assert!(
            (offset as usize).is_multiple_of(PAGE_SIZE) && bytes.len() <= PAGE_SIZE,
            "write alignment: offset={offset}, len={}",
            bytes.len()
        );
        debug_assert!(
            offset as usize + bytes.len() <= self.size,
            "write out of bounds"
        );
        flash::write(self.base + offset, bytes);
        Ok(())
    }
}
