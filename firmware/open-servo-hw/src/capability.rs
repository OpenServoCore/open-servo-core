//! Runtime hardware capability detection.
//!
//! This module provides runtime capability flags for boards, replacing compile-time
//! feature flags. All sensor/peripheral code is compiled in, but boards report
//! which capabilities are actually available at runtime.

// ============================================================================
// Motor topology
// ============================================================================

/// Motor topology/type supported by this board.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MotorType {
    /// Brushed DC motor (H-bridge driver).
    Bdc,
    /// Brushless DC motor (3-phase driver).
    Bldc,
}

#[cfg(feature = "defmt")]
impl defmt::Format for MotorType {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            MotorType::Bdc => defmt::write!(fmt, "BDC"),
            MotorType::Bldc => defmt::write!(fmt, "BLDC"),
        }
    }
}

// ============================================================================
// Optional sensor capabilities
// ============================================================================

/// Runtime sensor capability flags (bitflags).
///
/// Use `has()` to check individual capabilities.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SensorCapabilities(u32);

/// Individual capability identifier for **optional** sensors.
///
/// Invariant sensors (always present on all boards) don't need capability flags:
/// - Board/MCU temperature, servo position, motor current, MCU voltage
///
/// This enum covers only **optional** sensors that may or may not be present.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SensorCapability {
    /// Motor terminal voltage sensing (per-node via resistor divider).
    /// BDC: 2 readings (one per terminal). BLDC: TBD.
    MotorVoltage = 0,

    /// Dedicated motor temperature sensor (NTC on winding/case).
    /// Distinct from board/MCU temp which is invariant.
    MotorTemp = 1,

    /// System/battery supply voltage (VSYS/VBAT via resistor divider).
    VsysVoltage = 2,

    /// High-resolution motor position encoder (custom dual optical).
    /// Distinct from servo position which is invariant.
    MotorEncoder = 3,
}

impl SensorCapabilities {
    /// No capabilities.
    pub const fn empty() -> Self {
        Self(0)
    }

    /// Check if a specific capability is present.
    #[inline]
    pub const fn has(&self, cap: SensorCapability) -> bool {
        (self.0 & (1 << cap as u8)) != 0
    }

    /// Add a capability flag.
    #[inline]
    pub const fn with(self, cap: SensorCapability) -> Self {
        Self(self.0 | (1 << cap as u8))
    }

    /// Remove a capability flag.
    #[inline]
    pub const fn without(self, cap: SensorCapability) -> Self {
        Self(self.0 & !(1 << cap as u8))
    }

    /// Get raw bits (for debug/serialization).
    #[inline]
    pub const fn bits(&self) -> u32 {
        self.0
    }

    /// Create from raw bits.
    #[inline]
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SensorCapabilities {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SensorCapabilities({=u32:#010x})", self.0)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SensorCapability {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            SensorCapability::MotorVoltage => defmt::write!(fmt, "MotorVoltage"),
            SensorCapability::MotorTemp => defmt::write!(fmt, "MotorTemp"),
            SensorCapability::VsysVoltage => defmt::write!(fmt, "VsysVoltage"),
            SensorCapability::MotorEncoder => defmt::write!(fmt, "MotorEncoder"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_has_no_capabilities() {
        let caps = SensorCapabilities::empty();
        assert!(!caps.has(SensorCapability::MotorVoltage));
        assert!(!caps.has(SensorCapability::MotorTemp));
    }

    #[test]
    fn test_with_adds_capability() {
        let caps = SensorCapabilities::empty()
            .with(SensorCapability::MotorVoltage)
            .with(SensorCapability::VsysVoltage);
        assert!(caps.has(SensorCapability::MotorVoltage));
        assert!(caps.has(SensorCapability::VsysVoltage));
        assert!(!caps.has(SensorCapability::MotorEncoder));
    }

    #[test]
    fn test_without_removes_capability() {
        let caps = SensorCapabilities::empty()
            .with(SensorCapability::MotorVoltage)
            .with(SensorCapability::MotorTemp)
            .without(SensorCapability::MotorVoltage);
        assert!(!caps.has(SensorCapability::MotorVoltage));
        assert!(caps.has(SensorCapability::MotorTemp));
    }
}
