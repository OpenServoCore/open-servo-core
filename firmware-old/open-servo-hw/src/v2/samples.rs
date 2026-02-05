//! Sample vocabulary.
//!
//! This module defines small **vocabulary types** representing measurements that
//! cross crate boundaries (board ↔ kernel ↔ controllers).
//!
//! # Design
//!
//! Two orthogonal concerns for sensor readings:
//! - **Availability**: Is the sensor present on this board? → Use `Option<Reading<T>>`
//! - **Validity**: Is *this* reading good? → Use `Reading<T>::Valid` vs `Reading<T>::Invalid`
//!
//! For sensors that are always present, use `Reading<T, R>` directly.
//! For optional sensors, wrap in `Option<Reading<T, R>>`.
//!
//! The raw ADC value is always accessible via `Reading::raw()`, even when the
//! reading is invalid - this is critical for debugging.
//!
//! This is intentionally *not* a math/processing module. No accumulation, filtering,
//! or estimation belongs here.

use open_servo_units::*;

// =============================================================================
// Reading<T, R> - core validity + raw tracking
// =============================================================================

/// A sensor reading that tracks validity and raw ADC value.
///
/// Use this for sensors that always exist on the board. For optional sensors,
/// wrap in `Option<Reading<T, R>>`.
///
/// # Type Parameters
/// - `T`: The converted/processed value type (e.g., `CentiDeg32`, `MilliAmp`)
/// - `R`: The raw ADC value type (defaults to `u16`)
///
/// # Examples
/// ```ignore
/// // Always-present sensor
/// let pos: Reading<CentiDeg32> = Reading::Valid { value: pos_cdeg, raw: adc_raw };
///
/// // Optional sensor
/// let motor_temp: Option<Reading<CentiC>> = Some(Reading::Valid { value: temp, raw });
/// let no_sensor: Option<Reading<CentiC>> = None;  // sensor not on this board
/// ```
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reading<T: Copy, R: Copy = u16> {
    /// Reading is invalid (out of range, ADC error, etc.) but raw value is available.
    Invalid { raw: R },
    /// Valid reading with converted value and raw ADC value.
    Valid { value: T, raw: R },
}

impl<T: Copy, R: Copy> Reading<T, R> {
    /// Returns `true` if this reading is valid.
    #[inline]
    pub fn is_valid(&self) -> bool {
        matches!(self, Reading::Valid { .. })
    }

    /// Get the converted value if valid, or `None` if invalid.
    #[inline]
    pub fn value(&self) -> Option<T> {
        match self {
            Reading::Valid { value, .. } => Some(*value),
            Reading::Invalid { .. } => None,
        }
    }

    /// Get the raw ADC value. Always available, even for invalid readings.
    ///
    /// This is critical for debugging - you can always see what the ADC read.
    #[inline]
    pub fn raw(&self) -> R {
        match self {
            Reading::Valid { raw, .. } | Reading::Invalid { raw } => *raw,
        }
    }

    /// Get the converted value or a fallback if invalid.
    #[inline]
    pub fn value_or(self, fallback: T) -> T {
        match self {
            Reading::Valid { value, .. } => value,
            Reading::Invalid { .. } => fallback,
        }
    }

    /// Map the converted value if valid, preserving raw.
    #[inline]
    pub fn map<U: Copy>(self, f: impl FnOnce(T) -> U) -> Reading<U, R> {
        match self {
            Reading::Valid { value, raw } => Reading::Valid {
                value: f(value),
                raw,
            },
            Reading::Invalid { raw } => Reading::Invalid { raw },
        }
    }
}

// =============================================================================
// Raw ADC types for multi-channel sensors
// =============================================================================

/// Raw ADC values for motor current sensing.
///
/// The enum discriminant matches `MotorCurrent` - board creates the appropriate variant.
/// Kernel pattern-matches to handle both BDC and BLDC without compile-time flags.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MotorCurrentRaw {
    /// BDC: single bus current sense.
    Bdc(u16),
    /// BLDC: three phase current ADC values (a, b, c).
    ///
    /// If the board only measures 2 phases, it should derive the third
    /// (ic = -(ia + ib)) and store the derived raw value here.
    Bldc(u16, u16, u16),
}

impl Default for MotorCurrentRaw {
    fn default() -> Self {
        Self::Bdc(0)
    }
}

/// Raw ADC values for motor voltage sensing.
///
/// The enum discriminant matches `MotorVoltage` - board creates the appropriate variant.
/// Kernel pattern-matches to handle both BDC and BLDC without compile-time flags.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MotorVoltageRaw {
    /// BDC: two terminal voltage ADC values (V+, V-).
    Bdc(u16, u16),
    /// BLDC: three phase voltage ADC values (a, b, c).
    ///
    /// If the board only measures 2 phases, it should derive the third
    /// and store the derived raw value here.
    Bldc(u16, u16, u16),
}

impl Default for MotorVoltageRaw {
    fn default() -> Self {
        Self::Bdc(0, 0)
    }
}

// =========================
// Typed sample aliases
// =========================
//
// These keep call sites readable and allow you to change underlying representation
// in one place if needed.
//
// Convention:
// - Mandatory sensors use `Reading<T>` directly
// - Optional sensors use `Option<Reading<T>>`

/// Primary servo position sensor sample (pot/hall).
/// Always present; tracks validity and raw ADC value.
pub type ServoPosSample = Reading<CentiDeg32>;

/// Optional secondary motor position sensor sample (custom optical encoder).
pub type MotorPosSample = Option<Reading<EncoderCount>>;

/// Ambient temperature proxy sample.
///
/// This is the single "environment" temperature channel exposed to the kernel.
/// Boards are free to source this from:
/// - MCU internal temperature sensor
/// - an NTC on the PCB
///
/// The intent is: a stable ambient-ish reference for supervisors and thermal models,
/// not a precise die-junction measurement.
pub type AmbientTempSample = Reading<CentiC>;

/// Optional motor temperature sample (NTC).
pub type MotorTempSample = Option<Reading<CentiC>>;

/// MCU supply / internal ADC VDD/VDDIO sample.
/// Always present; tracks validity and raw ADC value.
pub type McuVddSample = Reading<MilliVolt>;

/// System supply (VSYS/VBAT) sample.
pub type VsysSample = Option<Reading<MilliVolt>>;

/// Motor voltage measurement shape.
///
/// This is conservative by design.
/// - Many BDC designs can provide two node voltages (`a`, `b`) per frame.
/// - BLDC per-phase voltage sensing schema varies widely; we provide all three phases.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MotorVoltage {
    /// BDC / H-bridge node voltages (two nodes).
    Bdc { a: MilliVolt, b: MilliVolt },

    /// BLDC: three phase voltages in (a, b, c) order.
    BldcPhases {
        a: MilliVolt,
        b: MilliVolt,
        c: MilliVolt,
    },
}

/// Motor voltage sample wrapper (optional; uses MotorVoltageRaw for raw values).
pub type MotorVoltageSample = Option<Reading<MotorVoltage, MotorVoltageRaw>>;

/// Motor current measurement shape (topology-aware).
///
/// This is **measurement vocabulary**, not control policy.
///
/// - `Bdc(i)` is a single motor current reading.
/// - `BldcPhases((a, b, c))` is three phase currents.
///
/// ## Phase ordering convention
/// We use **(a, b, c)** to mean phases **(1, 2, 3)** respectively.
/// (Equivalent to `(ia, ib, ic)` in many motor-control codebases.)
///
/// If your hardware measures only two phases, the board adapter may derive the
/// third (often via `a + b + c = 0`) and still populate all three fields.
///
/// Sign convention is board-defined but should be consistent across a product line.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MotorCurrent {
    /// Brushed DC / H-bridge: one current reading.
    Bdc(MilliAmp),

    /// BLDC: three phase currents in (a, b, c) order.
    BldcPhases((MilliAmp, MilliAmp, MilliAmp)),
}

impl Default for MotorCurrent {
    /// Default to BDC zero current.
    fn default() -> Self {
        Self::Bdc(MilliAmp::from_ma(0))
    }
}

/// Motor current sample wrapper (always present; uses MotorCurrentRaw for raw values).
pub type MotorCurrentSample = Reading<MotorCurrent, MotorCurrentRaw>;
