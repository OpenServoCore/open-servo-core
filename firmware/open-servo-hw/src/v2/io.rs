//! Board↔kernel IO boundary types.
//!
//! This module defines the **boundary shapes** between board/HAL and kernel.
//!
//! ## Why this is in `open-servo-hw` (not kernel-api)
//! These types are “hardware-adjacent”: they describe what the board can *measure*
//! and what the kernel can *command* at the motor-driver boundary. They are shared
//! vocabulary, but they are not part of the kernel scheduling / node contracts.
//!
//! ## Design goals
//! - **Stable, small vocabulary** that works for both BDC and BLDC (FOC) boards.
//! - **Intent-level commands** (Drive/Brake/Coast) with optional board hints.
//! - **No electrical implementation leakage** into kernel logic.
//!   - e.g. BDC “fast/slow decay” is a PWM strategy knob, not a universal concept.
//! - **No const generics / DMA buffer types**: the board owns DMA buffers internally.
//!
//! ## Sampling model
//! [`SensorFrame`] represents “one sampling instant” at the board cadence
//! (PWM/ADC/DMA ISR), not a specific control-loop rate.
//!
//! Boards may sample *everything* every frame, but the kernel is free to use only
//! a subset per tick domain.

use crate::v2::samples::*;
use open_servo_units::*;

// ============================================================================
// Sensor sampling
// ============================================================================

/// Per-frame sensor readings produced by the board/HAL layer.
///
/// "Frame" refers to the board sampling cadence (PWM → ADC → DMA → ISR),
/// not a specific control-loop rate.
///
/// ### Sensor availability vs validity
///
/// Two orthogonal concerns for sensor readings:
/// - **Availability**: Is the sensor present on this board? → Use `Option<Reading<T>>`
/// - **Validity**: Is *this particular* reading good? → Use `Reading::Valid` vs `Reading::Invalid`
///
/// For mandatory sensors that are always present on all boards, use `Reading<T>` directly.
/// For optional sensors that may not exist on all boards, wrap in `Option<Reading<T>>`.
///
/// The raw ADC value is always accessible via `Reading::raw()`, even when the
/// reading is invalid - this is critical for debugging.
///
/// ### Motor topology differences
/// Motor current and voltage samples use topology-aware enums (BDC vs BLDC) that
/// carry the appropriate number of channels. The board creates the correct variant
/// based on motor type; the kernel pattern-matches to handle both without compile-time flags.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SensorFrame {
    /// Raw servo position sample from the primary position sensor.
    /// - For 180°/270°: bounded range.
    /// - For 360° absolute: wraps at 0/360.
    pub pos: ServoPosSample,

    /// Motor current measurement.
    ///
    /// Shape depends on motor type:
    /// - BDC: a single signed current
    /// - BLDC: A/B/C phase currents (or derived equivalents)
    pub current: MotorCurrentSample,

    /// MCU supply voltage / VDD reading (internal ADC / divider).
    pub mcu_vdd: McuVddSample,

    /// System supply voltage (VSYS/VBAT). Optional on some boards.
    pub vsys: VsysSample,

    /// Ambient/board temperature proxy used for thermal modeling. Typically required.
    ///
    /// If you physically have both MCU and board NTC temps, it’s fine to merge them
    /// here into a single “ambient proxy” sample and keep the more detailed breakdown
    /// board-local.
    pub ambient_temp: AmbientTempSample,

    /// Dedicated motor temperature sensor (optional).
    pub motor_temp: MotorTempSample,

    /// Optional high-resolution motor-side encoder (distinct from servo output position).
    pub motor_pos: MotorPosSample,

    /// Motor terminal/phase voltage sampling (optional; topology-dependent).
    pub motor_v: MotorVoltageSample,

    /// Hardware driver health (non-latched “is OK right now”).
    ///
    /// This should reflect the instantaneous condition of the power stage:
    /// - gate-driver fault pin OK
    /// - undervoltage lockout not active
    /// - etc.
    ///
    /// Latched faults and gating policy belong in the kernel; this is just raw signal.
    pub driver_ok: bool,
}

impl SensorFrame {
    /// Construct a frame from individual fields.
    ///
    /// Prefer explicit construction in board code; avoid `Default` for frames
    /// unless your sample types intentionally default to `Unavailable`.
    #[inline]
    pub const fn new(
        pos: ServoPosSample,
        current: MotorCurrentSample,
        mcu_vdd: McuVddSample,
        vsys: VsysSample,
        ambient_temp: AmbientTempSample,
        motor_temp: MotorTempSample,
        motor_pos: MotorPosSample,
        motor_v: MotorVoltageSample,
        driver_ok: bool,
    ) -> Self {
        Self {
            pos,
            current,
            mcu_vdd,
            vsys,
            ambient_temp,
            motor_temp,
            motor_pos,
            motor_v,
            driver_ok,
        }
    }
}

impl Default for SensorFrame {
    /// Default frame with invalid/unavailable values.
    ///
    /// Useful for kernel state initialization. Not recommended for production
    /// board code—prefer explicit construction with actual readings.
    fn default() -> Self {
        Self {
            pos: Reading::Invalid { raw: 0 },
            current: Reading::Invalid {
                raw: MotorCurrentRaw::default(),
            },
            mcu_vdd: Reading::Invalid { raw: 0 },
            vsys: None,
            ambient_temp: Reading::Invalid { raw: 0 },
            motor_temp: None,
            motor_pos: None,
            motor_v: None,
            driver_ok: false,
        }
    }
}

// ============================================================================
// Motor actuation boundary
// ============================================================================

/// Intent-level motor behavior requested by the kernel.
///
/// This is intentionally **topology-neutral**. Boards translate these intents into
/// their electrical implementation.
///
/// - BDC (H-bridge):
///   - `Drive` → PWM drive using `effort`
///   - `Coast` → high-Z / freewheel
///   - `Brake` → dynamic braking (e.g. short terminals) if supported
///
/// - BLDC (FOC):
///   - `Drive` → torque-producing current command derived from `effort`
///   - `Coast` → inverter high-Z / disable outputs
///   - `Brake` → damping/braking behavior (e.g. active damping torque or active-short)
///
/// The kernel may assume that supported boards implement both `Coast` and `Brake`
/// if you make that a project requirement. If a board cannot honor an intent,
/// it should degrade to the safest behavior (often `driver_en=false`) and may
/// emit telemetry.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DriveMode {
    /// High impedance / freewheel (no commanded torque).
    Coast,
    /// Apply damping / braking (resist motion).
    Brake,
    /// Actively drive according to `effort`.
    Drive,
}

/// BDC-only PWM strategy hint.
///
/// Fast/slow decay is a *power-stage implementation detail* of BDC H-bridge PWM.
/// It is not a universal concept and should be treated as an optional hint:
/// - BDC boards may implement it
/// - BLDC boards should ignore it
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PwmStrategy {
    /// Fast decay PWM strategy (more aggressive current decay, typically “stiffer” feel).
    FastDecay,
    /// Slow decay PWM strategy (more recirculation, typically “softer” feel).
    SlowDecay,
}

/// Optional motor-driver hints.
///
/// These allow higher-level features (e.g. compliance) to express preferences without
/// baking hardware-specific details into the core command vocabulary.
///
/// Boards are free to ignore any/all hints.
///
/// ### Separation of concerns
/// - `DriveMode` expresses *what behavior* the kernel wants.
/// - `MotorHints` expresses *how the board might implement it* (optional).
///
/// In particular, **do not overload brake knobs to mean drive softness**:
/// - `drive_pwm` is for Drive-mode PWM strategy (BDC-only)
/// - `brake_strength` is for Brake-mode damping intensity (topology-neutral)
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct MotorHints {
    /// Preferred PWM strategy while in `DriveMode::Drive`.
    ///
    /// - BDC boards may map this to fast/slow decay selection.
    /// - BLDC boards should ignore this.
    pub drive_pwm: Option<PwmStrategy>,

    /// Preferred braking/damping intensity while in `DriveMode::Brake`.
    ///
    /// Generic “strength” knob so the kernel can express “gentle brake” vs “firm brake”.
    /// Interpretation is board-defined but should be monotonic (higher = stronger).
    ///
    /// Suggested semantics:
    /// - `0`   → minimal damping (almost coast)
    /// - `32767` → strong damping (clamped by hardware limits)
    ///
    /// Boards may clamp or ignore this depending on capability.
    pub brake_strength: Option<u16>,
}

/// Kernel → board motor command at the motor-driver boundary.
///
/// This is the only "actuation output" the kernel needs to produce.
/// The board maps it to:
/// - enable pins
/// - PWM / inverter commands
/// - driver configuration
///
/// ### Safety
/// The kernel should use [`MotorCommand::safe()`] for fault gating or disengage.
///
/// ### Effort Semantics
///
/// `effort` is a **dimensionless normalized command** (see [`Effort`]).
///
/// - **Range:** -32768..32767 (full `i16`, Q15-ish)
/// - **Meaning:** "how much actuator authority", not current/torque/voltage
/// - **Board mapping:**
///   - BDC: typically PWM duty cycle
///   - BLDC FOC: typically torque-producing current (Iq) request
///
/// The kernel produces normalized `Effort`; the board translates to electrical commands.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct MotorCommand {
    /// Driver chip enable (power stage allowed to do anything).
    ///
    /// - `false`: driver outputs must be disabled regardless of `mode/effort/hints`.
    /// - `true`: board applies `mode` + `effort` (and may use `hints`).
    pub driver_en: bool,

    /// Desired actuator behavior when `driver_en == true`.
    pub mode: DriveMode,

    /// Dimensionless normalized actuator command (-32768..32767).
    ///
    /// Only meaningful when `driver_en == true && mode == DriveMode::Drive`.
    /// Board maps to PWM duty (BDC) or Iq request (BLDC FOC).
    pub effort: Effort,

    /// Optional board hints (safe to ignore).
    pub hints: MotorHints,
}

impl MotorCommand {
    /// Unambiguously safe: driver disabled + zero effort.
    ///
    /// This is the default output for fault gating and disengage.
    #[inline]
    pub const fn safe() -> Self {
        Self {
            driver_en: false,
            mode: DriveMode::Coast,
            effort: Effort::ZERO,
            hints: MotorHints {
                drive_pwm: None,
                brake_strength: None,
            },
        }
    }

    /// Enable driver and actively drive with effort.
    ///
    /// `drive_pwm` is a BDC-only hint; BLDC boards should ignore it.
    #[inline]
    pub const fn drive(effort: Effort, drive_pwm: Option<PwmStrategy>) -> Self {
        Self {
            driver_en: true,
            mode: DriveMode::Drive,
            effort,
            hints: MotorHints {
                drive_pwm,
                brake_strength: None,
            },
        }
    }

    /// Enable driver and coast (high-Z outputs).
    #[inline]
    pub const fn coast_enabled() -> Self {
        Self {
            driver_en: true,
            mode: DriveMode::Coast,
            effort: Effort::ZERO,
            hints: MotorHints {
                drive_pwm: None,
                brake_strength: None,
            },
        }
    }

    /// Enable driver and brake with an optional strength hint.
    ///
    /// If `strength` is `None`, the board chooses a reasonable default.
    #[inline]
    pub const fn brake_enabled(strength: Option<u16>) -> Self {
        Self {
            driver_en: true,
            mode: DriveMode::Brake,
            effort: Effort::ZERO,
            hints: MotorHints {
                drive_pwm: None,
                brake_strength: strength,
            },
        }
    }

    /// Disable driver regardless of mode/effort.
    #[inline]
    pub const fn driver_off() -> Self {
        Self::safe()
    }
}
