//! Typed register handles for shadow table access.
//!
//! The `Reg<A, T>` struct provides type-safe read/write access to shadow
//! table registers, where `A` is a type-level access marker and `T` is
//! the data type.
//!
//! # Example
//!
//! ```ignore
//! use open_servo_registry::reg::{Reg, RO, RW};
//! use open_servo_registry::ViewRead;
//!
//! const GOAL_POS: Reg<RW, i32> = Reg::new(512);
//! const PRESENT_POS: Reg<RO, i32> = Reg::new(516);
//!
//! // Read a register
//! let pos = PRESENT_POS.read(&view)?;
//!
//! // Write a register
//! GOAL_POS.write(&mut view, 1000)?;
//! ```

use crate::view::{ViewRead, ViewWrite};
use core::marker::PhantomData;
use open_servo_shadow::ShadowError;

// ============================================================================
// Type-level access markers
// ============================================================================

/// Read-only access marker.
pub struct RO;

/// Write-only access marker.
pub struct WO;

/// Read-write access marker.
///
/// Note: EEPROM fields with RW access are locked when torque is enabled.
/// The EEPROM flag is separate from the access marker.
pub struct RW;

/// Facade alias marker (access derived from vendor register at runtime).
pub struct Facade;

// ============================================================================
// Reg<A, T> struct
// ============================================================================

/// Typed register handle.
///
/// `A` is a type-level access marker (RO, WO, RW, Facade).
/// `T` is the data type (i32, i16, u16, u32, u8, bool).
#[derive(Clone, Copy)]
pub struct Reg<A, T> {
    /// Register offset in the shadow table.
    pub offset: u16,
    _marker: PhantomData<(A, T)>,
}

impl<A, T> Reg<A, T> {
    /// Create a new register handle at the given offset.
    pub const fn new(offset: u16) -> Self {
        Self {
            offset,
            _marker: PhantomData,
        }
    }
}

// ============================================================================
// Per-type read/write implementations
// ============================================================================

impl<A> Reg<A, i32> {
    /// Byte length of this register type.
    pub const fn len() -> u16 {
        4
    }

    /// Read an i32 value from the register.
    pub fn read<V: ViewRead>(&self, view: &V) -> Result<i32, ShadowError> {
        let mut buf = [0u8; 4];
        view.read(self.offset, &mut buf)?;
        Ok(i32::from_le_bytes(buf))
    }

    /// Write an i32 value to the register.
    pub fn write<V: ViewWrite>(&self, view: &mut V, val: i32) -> Result<(), ShadowError> {
        view.write(self.offset, &val.to_le_bytes())
    }
}

impl<A> Reg<A, i16> {
    /// Byte length of this register type.
    pub const fn len() -> u16 {
        2
    }

    /// Read an i16 value from the register.
    pub fn read<V: ViewRead>(&self, view: &V) -> Result<i16, ShadowError> {
        let mut buf = [0u8; 2];
        view.read(self.offset, &mut buf)?;
        Ok(i16::from_le_bytes(buf))
    }

    /// Write an i16 value to the register.
    pub fn write<V: ViewWrite>(&self, view: &mut V, val: i16) -> Result<(), ShadowError> {
        view.write(self.offset, &val.to_le_bytes())
    }
}

impl<A> Reg<A, u16> {
    /// Byte length of this register type.
    pub const fn len() -> u16 {
        2
    }

    /// Read a u16 value from the register.
    pub fn read<V: ViewRead>(&self, view: &V) -> Result<u16, ShadowError> {
        let mut buf = [0u8; 2];
        view.read(self.offset, &mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }

    /// Write a u16 value to the register.
    pub fn write<V: ViewWrite>(&self, view: &mut V, val: u16) -> Result<(), ShadowError> {
        view.write(self.offset, &val.to_le_bytes())
    }
}

impl<A> Reg<A, u32> {
    /// Byte length of this register type.
    pub const fn len() -> u16 {
        4
    }

    /// Read a u32 value from the register.
    pub fn read<V: ViewRead>(&self, view: &V) -> Result<u32, ShadowError> {
        let mut buf = [0u8; 4];
        view.read(self.offset, &mut buf)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Write a u32 value to the register.
    pub fn write<V: ViewWrite>(&self, view: &mut V, val: u32) -> Result<(), ShadowError> {
        view.write(self.offset, &val.to_le_bytes())
    }
}

impl<A> Reg<A, u8> {
    /// Byte length of this register type.
    pub const fn len() -> u16 {
        1
    }

    /// Read a u8 value from the register.
    pub fn read<V: ViewRead>(&self, view: &V) -> Result<u8, ShadowError> {
        let mut buf = [0u8; 1];
        view.read(self.offset, &mut buf)?;
        Ok(buf[0])
    }

    /// Write a u8 value to the register.
    pub fn write<V: ViewWrite>(&self, view: &mut V, val: u8) -> Result<(), ShadowError> {
        view.write(self.offset, &[val])
    }
}

impl<A> Reg<A, bool> {
    /// Byte length of this register type.
    pub const fn len() -> u16 {
        1
    }

    /// Read a bool value from the register (0 = false, nonzero = true).
    pub fn read<V: ViewRead>(&self, view: &V) -> Result<bool, ShadowError> {
        let mut buf = [0u8; 1];
        view.read(self.offset, &mut buf)?;
        Ok(buf[0] != 0)
    }

    /// Write a bool value to the register (false = 0, true = 1).
    pub fn write<V: ViewWrite>(&self, view: &mut V, val: bool) -> Result<(), ShadowError> {
        view.write(self.offset, &[if val { 1 } else { 0 }])
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reg_new() {
        const TEST_REG: Reg<RW, i32> = Reg::new(512);
        assert_eq!(TEST_REG.offset, 512);
    }

    #[test]
    fn test_reg_copy() {
        const REG: Reg<RO, i16> = Reg::new(100);
        let copy = REG;
        assert_eq!(copy.offset, 100);
    }
}
