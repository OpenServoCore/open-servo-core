//! Sample vocabulary.
//!
//! This module defines small **vocabulary types** representing measurements that
//! cross crate boundaries (board ↔ kernel ↔ controllers).
//!
//! # Why this exists
//! Many real embedded pipelines need to represent more than "present or not".
//! We distinguish three states for each measurement:
//! - [`Sampled::Unavailable`]: not implemented / not routed on this board/build
//! - [`Sampled::Invalid`]: implemented, but not valid right now (boot, ADC not ready, fault)
//! - [`Sampled::Value`]: valid sample value
//!
//! This is intentionally *not* a math/processing module. No accumulation, filtering,
//! or estimation belongs here.

use open_servo_units::*;

/// A sampled measurement with explicit availability/validity.
///
/// Intended use:
/// - `Unavailable`: sensor/feature not present on this board/build
/// - `Invalid`: present, but not usable *right now*
/// - `Value(v)`: valid sample
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Sampled<T: Copy> {
    /// Not implemented / not available on this board.
    Unavailable,
    /// Implemented, but not valid right now.
    Invalid,
    /// Valid sample.
    Value(T),
}

impl<T: Copy> Default for Sampled<T> {
    #[inline]
    fn default() -> Self {
        Sampled::Unavailable
    }
}

impl<T: Copy> Sampled<T> {
    /// Returns `true` iff this sample is a valid value.
    #[inline]
    pub fn is_valid(self) -> bool {
        matches!(self, Sampled::Value(_))
    }

    /// Convert to `Option<T>` (drops Invalid/Unavailable).
    #[inline]
    pub fn as_option(self) -> Option<T> {
        match self {
            Sampled::Value(v) => Some(v),
            _ => None,
        }
    }

    /// Map the contained value if valid.
    #[inline]
    pub fn map<U: Copy>(self, f: impl FnOnce(T) -> U) -> Sampled<U> {
        match self {
            Sampled::Unavailable => Sampled::Unavailable,
            Sampled::Invalid => Sampled::Invalid,
            Sampled::Value(v) => Sampled::Value(f(v)),
        }
    }

    /// Get the value or a fallback.
    #[inline]
    pub fn unwrap_or(self, fallback: T) -> T {
        match self {
            Sampled::Value(v) => v,
            _ => fallback,
        }
    }
}

// =========================
// Typed sample aliases
// =========================
//
// These keep call sites readable and allow you to change underlying representation
// in one place if needed.

/// Primary servo position sensor sample (pot/hall).
///
/// Uses overflow-safe internal representation.
pub type ServoPosSample = Sampled<CentiDeg32>;

/// Optional secondary motor position sensor sample (custom optical encoder).
pub type MotorPosSample = Sampled<EncoderCount>;

/// Ambient temperature proxy sample.
///
/// This is the single "environment" temperature channel exposed to the kernel.
/// Boards are free to source this from:
/// - MCU internal temperature sensor
/// - an NTC on the PCB
///
/// The intent is: a stable ambient-ish reference for supervisors and thermal models,
/// not a precise die-junction measurement.
pub type AmbientTempSample = Sampled<CentiC>;

/// Optional motor temperature sample (NTC).
pub type MotorTempSample = Sampled<CentiC>;

/// MCU supply / internal ADC VDD/VDDIO sample.
pub type McuVddSample = Sampled<MilliVolt>;

/// System supply (VSYS/VBAT) sample.
pub type VsysSample = Sampled<MilliVolt>;

/// Motor voltage measurement shape.
///
/// This is conservative by design.
/// - Many BDC designs can provide two node voltages (`a`, `b`) per frame.
/// - BLDC per-phase voltage sensing schema is not decided yet; add later when stable.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MotorVoltage {
    /// BDC / H-bridge node voltages (two nodes).
    Bdc { a: MilliVolt, b: MilliVolt },
}

/// Motor voltage sample wrapper.
pub type MotorVoltageSample = Sampled<MotorVoltage>;

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

/// Motor current sample wrapper.
pub type MotorCurrentSample = Sampled<MotorCurrent>;
