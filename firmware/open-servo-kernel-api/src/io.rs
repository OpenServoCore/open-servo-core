//! Kernel↔board IO vocabulary.
//!
//! This module defines the *boundary* shapes (`SensorFrame`, `MotorCommand`) plus
//! a small set of **doc-only naming anchors** (`InputFast`, `OutputFast`, …).
//!
//! # Naming anchors
//! Many embedded codebases talk about “fast/medium/slow inputs/outputs”.
//! In this architecture, those names are about **scheduling domains**, not about
//! “what controllers should take as inputs”.
//!
//! - Controllers/nodes should use small purpose-built `Node::In` / `Node::Out` wire types.
//! - `SensorFrame` / `MotorCommand` are boundary types between board and kernel.
//! - Derived/windowed “medium/slow measurements” are kernel policy and live in the kernel crate.

use crate::samples::*;
use crate::units::*;

/// Per-frame sensor readings produced by the board/HAL layer.
///
/// “Frame” refers to the board sampling cadence (PWM/ADC/DMA ISR), not a specific control-loop rate.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SensorFrame {
    pub pos: ServoPosSample,
    pub current: MotorCurrentSample,
    pub mcu_vdd: McuVddSample,
    pub vsys: VsysSample,
    pub ambient_temp: AmbientTempSample,
    pub motor_temp: MotorTempSample,
    pub motor_pos: MotorPosSample,
    pub driver_ok: bool,
    pub motor_v: MotorVoltageSample,
}

/// Actuator command produced by the kernel for the board to apply.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct MotorCommand {
    pub enable: bool,
    pub effort: Effort,
}

impl MotorCommand {
    #[inline]
    pub const fn safe() -> Self {
        Self {
            enable: false,
            effort: Effort::ZERO,
        }
    }
}

// ============================================================================
// Doc-only naming anchors (zero-sized types)
// ============================================================================

/// Naming anchor: “fast domain input”.
///
/// This is a **documentation-only** type used to standardize terminology in docs
/// and comments. It is not intended to be instantiated.
///
/// Suggested mapping in *this* project:
/// - Kernel↔board boundary “fast input”: [`SensorFrame`]
/// - Node/controller inputs: small `Copy` wire types (tuples/small structs)
pub struct InputFast;

/// Naming anchor: “fast domain output”.
///
/// Suggested mapping in *this* project:
/// - Kernel↔board boundary “fast output”: [`MotorCommand`]
/// - Intermediate node outputs: small `Copy` wire types (often [`Effort`])
pub struct OutputFast;

/// Naming anchor: “medium domain input”.
///
/// **Not defined in this API** because medium-rate views are typically derived
/// (windowed/filtered/supervisor results) and are kernel policy.
///
/// Define kernel-specific `MediumFrame` / `WindowStats` types in the kernel crate.
pub struct InputMedium;

/// Naming anchor: “medium domain output”.
///
/// Define kernel-specific outputs in the kernel crate if needed (most systems don’t
/// have a medium-rate board output boundary).
pub struct OutputMedium;

/// Naming anchor: “slow domain input”.
///
/// Slow-rate environmental/supervisor inputs may still be present in [`SensorFrame`]
/// as `Sampled<T>`; “slow” here refers to how the kernel chooses to *consume* them.
///
/// If you maintain a separate slow supervisor view, define it in the kernel crate.
pub struct InputSlow;

/// Naming anchor: “slow domain output”.
///
/// Define kernel-specific slow outputs in the kernel crate if needed (LEDs, fans, etc.).
pub struct OutputSlow;
