//! Register specification types for REPL and tooling.

/// Access control for registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Access {
    /// Read-only (telemetry/status).
    R,
    /// Read-write (RAM, always writable).
    Rw,
    /// Read-write but requires Torque Enable = 0 (EEPROM).
    RwEepromLocked,
}

/// Field encoding hint for display/parsing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Encoding {
    /// Unsigned 8-bit integer.
    U8,
    /// Signed 16-bit little-endian integer.
    I16Le,
    /// Unsigned 16-bit little-endian integer.
    U16Le,
    /// Signed 32-bit little-endian integer.
    I32Le,
    /// Unsigned 32-bit little-endian integer.
    U32Le,
    /// Boolean (u8: 0=false, 1=true).
    Bool,
    /// Enumeration with string names indexed by value.
    Enum(&'static [&'static str]),
}

impl Encoding {
    /// Get the byte length for this encoding.
    pub const fn len(&self) -> u8 {
        match self {
            Encoding::U8 | Encoding::Bool | Encoding::Enum(_) => 1,
            Encoding::I16Le | Encoding::U16Le => 2,
            Encoding::I32Le | Encoding::U32Le => 4,
        }
    }
}

/// Register field specification.
#[derive(Clone, Copy, Debug)]
pub struct RegSpec {
    /// Field name (e.g., "goal_pos_cdeg", "present_pos_cdeg").
    pub name: &'static str,
    /// Address in shadow table (Dynamixel protocol address).
    pub address: u16,
    /// Encoding hint for display/parsing (length derived from encoding).
    pub encoding: Encoding,
    /// Access control.
    pub access: Access,
}

impl RegSpec {
    /// Create a new register specification.
    pub const fn new(name: &'static str, address: u16, encoding: Encoding, access: Access) -> Self {
        Self {
            name,
            address,
            encoding,
            access,
        }
    }

    /// Get the byte length of this register.
    pub const fn len(&self) -> u8 {
        self.encoding.len()
    }

    /// Check if this register is writable.
    pub const fn is_writable(&self) -> bool {
        matches!(self.access, Access::Rw | Access::RwEepromLocked)
    }
}

/// Repeated register range (e.g., Indirect Address 1-28).
#[derive(Clone, Copy, Debug)]
pub struct RangeSpec {
    /// Name prefix (e.g., "indirect_addr").
    pub name_prefix: &'static str,
    /// Starting address.
    pub base_address: u16,
    /// Number of entries.
    pub count: u8,
    /// Per-entry encoding.
    pub encoding: Encoding,
    /// Access control.
    pub access: Access,
}

impl RangeSpec {
    /// Create a new range specification.
    pub const fn new(
        name_prefix: &'static str,
        base_address: u16,
        count: u8,
        encoding: Encoding,
        access: Access,
    ) -> Self {
        Self {
            name_prefix,
            base_address,
            count,
            encoding,
            access,
        }
    }

    /// Address of the nth entry (0-indexed).
    pub const fn address_of(&self, index: u8) -> u16 {
        self.base_address + (index as u16) * (self.encoding.len() as u16)
    }

    /// Get the byte length of one entry.
    pub const fn entry_len(&self) -> u8 {
        self.encoding.len()
    }
}
