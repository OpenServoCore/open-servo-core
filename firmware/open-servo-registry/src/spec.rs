//! Register specification types for REPL and tooling.

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

/// Register field specification.
#[derive(Clone, Copy, Debug)]
pub struct RegSpec {
    /// Field name (e.g., "ctrl.engaged", "telem.pos").
    pub name: &'static str,
    /// Offset in shadow table.
    pub offset: u16,
    /// Field length in bytes.
    pub len: u8,
    /// Encoding hint for display/parsing.
    pub encoding: Encoding,
    /// True if host can write (ctrl/config), false if read-only (telemetry).
    pub writable: bool,
}

impl RegSpec {
    /// Create a new register specification.
    pub const fn new(
        name: &'static str,
        offset: u16,
        len: u8,
        encoding: Encoding,
        writable: bool,
    ) -> Self {
        Self {
            name,
            offset,
            len,
            encoding,
            writable,
        }
    }
}
