//! Register specification types for REPL and tooling.

use open_servo_units::{CentiC, CentiDeg, CentiDeg32, MilliAmp, MilliVolt};

// ============================================================================
// Encoding trait for type-safe register definitions
// ============================================================================

/// Trait for types that have a well-defined register encoding.
///
/// Implement this for unit types to enable type-safe register specifications.
pub trait UnitEncoding {
    /// The register encoding for this type.
    const ENCODING: Encoding;
}

impl UnitEncoding for u8 {
    const ENCODING: Encoding = Encoding::U8;
}

impl UnitEncoding for i16 {
    const ENCODING: Encoding = Encoding::I16Le;
}

impl UnitEncoding for u16 {
    const ENCODING: Encoding = Encoding::U16Le;
}

impl UnitEncoding for i32 {
    const ENCODING: Encoding = Encoding::I32Le;
}

impl UnitEncoding for u32 {
    const ENCODING: Encoding = Encoding::U32Le;
}

impl UnitEncoding for bool {
    const ENCODING: Encoding = Encoding::Bool;
}

// Unit type encodings (derived from their backing storage)
impl UnitEncoding for CentiC {
    const ENCODING: Encoding = Encoding::I16Le; // CentiC(pub i16)
}

impl UnitEncoding for CentiDeg {
    const ENCODING: Encoding = Encoding::I16Le; // CentiDeg(pub i16)
}

impl UnitEncoding for CentiDeg32 {
    const ENCODING: Encoding = Encoding::I32Le; // CentiDeg32(pub i32)
}

impl UnitEncoding for MilliVolt {
    const ENCODING: Encoding = Encoding::I16Le; // MilliVolt(pub i16)
}

impl UnitEncoding for MilliAmp {
    const ENCODING: Encoding = Encoding::I16Le; // MilliAmp(pub i16)
}

// ============================================================================
// Access control
// ============================================================================

/// Access control for registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Access {
    /// Read-only (telemetry/status).
    RO,
    /// Write-only.
    WO,
    /// Read-write (RAM, always writable).
    RW,
    /// Read-write but requires Torque Enable = 0 (EEPROM).
    RWE,
    /// Reserved - access error on read/write.
    Reserved,
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
    /// Reserved region with explicit byte length.
    Reserved(u8),
}

impl Encoding {
    /// Get the byte length for this encoding.
    pub const fn len(&self) -> u8 {
        match self {
            Encoding::U8 | Encoding::Bool | Encoding::Enum(_) => 1,
            Encoding::I16Le | Encoding::U16Le => 2,
            Encoding::I32Le | Encoding::U32Le => 4,
            Encoding::Reserved(n) => *n,
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

    /// Create a type-safe register specification.
    ///
    /// The encoding is derived from the type's `UnitEncoding` implementation,
    /// ensuring the register encoding matches the unit type.
    ///
    /// # Example
    /// ```ignore
    /// use open_servo_units::CentiDeg32;
    /// let spec = RegSpec::typed::<CentiDeg32>("goal_pos_cdeg", 512, Access::RW);
    /// assert_eq!(spec.encoding, Encoding::I32Le);
    /// ```
    pub const fn typed<T: UnitEncoding>(name: &'static str, address: u16, access: Access) -> Self {
        Self {
            name,
            address,
            encoding: T::ENCODING,
            access,
        }
    }

    /// Get the byte length of this register.
    pub const fn len(&self) -> u8 {
        self.encoding.len()
    }

    /// Check if this register is writable.
    pub const fn is_writable(&self) -> bool {
        matches!(self.access, Access::RW | Access::RWE | Access::WO)
    }

    /// Check if this register is readable.
    pub const fn is_readable(&self) -> bool {
        matches!(self.access, Access::RO | Access::RW | Access::RWE)
    }

    /// Check if this is a reserved/undefined region.
    pub const fn is_reserved(&self) -> bool {
        matches!(self.access, Access::Reserved)
    }

    /// Create a reserved field specification.
    ///
    /// Reserved fields return Access Error on read/write.
    pub const fn reserved(name: &'static str, address: u16, len: u8) -> Self {
        Self {
            name,
            address,
            encoding: Encoding::Reserved(len),
            access: Access::Reserved,
        }
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
