//! Register vocabulary (registry / wire protocol integration).
//!
//! - Wire protocol (Dynamixel) can produce `(addr, value)` writes.
//! - Kernel can route those writes to a registry/state manager.
//! - Board can expose persistent storage / EEPROM handling.
//!
//! Keeping the vocabulary in the API crate avoids every crate inventing its own
//! register types.

/// Register address type.
pub type RegAddr = u16;

/// Register value “sum type”.
///
/// This is intended to be minimal and practical for embedded servo registers.
/// Add new variants carefully because it affects multiple crates.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RegValue {
    U8(u8),
    U16(u16),
    I16(i16),
    U32(u32),
    I32(i32),
    Bool(bool),
}

/// Register access error.
///
/// This is intentionally general; concrete reasons live in kernel/board crates.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RegError {
    /// Address not defined by the register map.
    InvalidAddr,
    /// Value type or range invalid for this address.
    InvalidValue,
    /// Attempted write to read-only register.
    ReadOnly,
    /// Attempted read from write-only register.
    WriteOnly,
    /// Operation refused because the system is in a state that forbids it
    /// (e.g., EEPROM write while torque enabled).
    Busy,
}

/// Optional boundary trait for a register map implementation.
///
/// This is **not** the “registry architecture” yet — it is only the minimum interface
/// that lets multiple crates speak the same language.
///
/// You can implement this in:
/// - `open-servo-kernel` (state manager / registry)
/// - or in a dedicated “regmap” crate later.
pub trait RegMap {
    fn get(&self, addr: RegAddr) -> Result<RegValue, RegError>;
    fn set(&mut self, addr: RegAddr, value: RegValue) -> Result<(), RegError>;
}
