//! Discrete-time fixed-point PID controller with configurable derivative mode
//! and compile-time selection of I/D terms.
//!
//! - Gains are i16 in Q8.8 fixed-point (gain = raw / 256.0).
//! - Inputs (sp, pv) and output are i32 in "plant units" (e.g. centideg, mA).
//! - Output is clamped to [out_min, out_max].
//! - Integral anti-windup is auto-derived from output limits by default.

/// Derivative calculation mode.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum DerivativeMode {
    /// D = Kd * (error - prev_error)
    #[default]
    OnError,

    /// D = -Kd * (pv - prev_pv)  (avoids derivative kick on setpoint steps)
    OnMeasurement,
}

/// Fixed-point PID controller with const-generic I/D term selection.
///
/// The `HAS_I` and `HAS_D` const parameters enable compile-time elimination
/// of unused integral and derivative code paths.
///
/// # Type Aliases
/// - [`PControllerI16`] - Pure P controller (no I, no D)
/// - [`PiControllerI16`] - PI controller (no D)
/// - [`PidControllerI16`] - Full PID controller
pub struct PidI16<const HAS_I: bool, const HAS_D: bool> {
    /// Gains in Q8.8 fixed-point
    kp: i16,
    ki: i16,
    kd: i16,

    mode: DerivativeMode,

    /// Integral accumulator in "pre-shift" units (error * gain).
    integ: i32,

    /// Anti-windup clamp for integral accumulator
    integ_min: i32,
    integ_max: i32,

    /// Error / measurement history for derivative
    prev_error: i32,
    prev_meas: i32,

    /// Output clamp in plant units (e.g. PWM command range)
    out_min: i32,
    out_max: i32,
}

/// Pure P controller (no I, no D)
pub type PControllerI16 = PidI16<false, false>;

/// PI controller (no D)
pub type PiControllerI16 = PidI16<true, false>;

/// Full PID controller
pub type PidControllerI16 = PidI16<true, true>;

impl<const HAS_I: bool, const HAS_D: bool> PidI16<HAS_I, HAS_D> {
    /// Number of fractional bits in gain representation (Q8.8).
    pub const GAIN_FRAC_BITS: u32 = 8;

    /// Construct a PID controller with gains in Q8.8 and
    /// output limits [out_min, out_max].
    ///
    /// Integral limits are auto-derived from output limits:
    ///   integ_min = out_min << FRAC, integ_max = out_max << FRAC.
    #[inline]
    pub fn new_auto_anti_windup(
        kp: i16,
        ki: i16,
        kd: i16,
        mode: DerivativeMode,
        out_min: i32,
        out_max: i32,
    ) -> Self {
        let frac = Self::GAIN_FRAC_BITS;

        // Derive integral clamp from output clamp.
        // This ensures I-term alone can never exceed output range.
        let integ_min = out_min.saturating_mul(1 << frac);
        let integ_max = out_max.saturating_mul(1 << frac);

        Self::new_with_integral_limits(kp, ki, kd, mode, out_min, out_max, integ_min, integ_max)
    }

    /// Construct a PID controller with explicit integral limits.
    ///
    /// `integ_min` / `integ_max` are in accumulator units (before >> GAIN_FRAC_BITS).
    #[inline]
    pub fn new_with_integral_limits(
        kp: i16,
        ki: i16,
        kd: i16,
        mode: DerivativeMode,
        out_min: i32,
        out_max: i32,
        integ_min: i32,
        integ_max: i32,
    ) -> Self {
        // Ensure limits are sane
        let (out_min, out_max) = if out_min <= out_max {
            (out_min, out_max)
        } else {
            (out_max, out_min)
        };

        let (integ_min, integ_max) = if integ_min <= integ_max {
            (integ_min, integ_max)
        } else {
            (integ_max, integ_min)
        };

        Self {
            kp,
            ki,
            kd,
            mode,
            integ: 0,
            integ_min,
            integ_max,
            prev_error: 0,
            prev_meas: 0,
            out_min,
            out_max,
        }
    }

    /// Reset integral and derivative history.
    #[inline]
    pub fn reset(&mut self) {
        self.integ = 0;
        self.prev_error = 0;
        self.prev_meas = 0;
    }

    /// One PID step.
    ///
    /// - `sp`: setpoint in plant units (e.g. centideg).
    /// - `pv`: process variable (measurement) in same units.
    /// - returns: control output clamped to [out_min, out_max].
    #[inline]
    pub fn step(&mut self, sp: i32, pv: i32) -> i32 {
        let frac = Self::GAIN_FRAC_BITS;

        // Error signal (always computed)
        let err = sp - pv;

        // ----- Proportional term -----
        let p = (self.kp as i32 * err) >> frac;

        // ----- Integral term (only if HAS_I) -----
        let i = if HAS_I {
            // Accumulate Ki * error in pre-shift units
            self.integ = self.integ.saturating_add((self.ki as i32) * err);

            // Anti-windup clamp
            if self.integ > self.integ_max {
                self.integ = self.integ_max;
            } else if self.integ < self.integ_min {
                self.integ = self.integ_min;
            }

            // Scale back to output units
            self.integ >> frac
        } else {
            0
        };

        // ----- Derivative term (only if HAS_D) -----
        let d = if HAS_D {
            let d_input = match self.mode {
                DerivativeMode::OnError => {
                    let d_err = err - self.prev_error;
                    self.prev_error = err;
                    d_err
                }
                DerivativeMode::OnMeasurement => {
                    let d_meas = pv - self.prev_meas;
                    self.prev_meas = pv;
                    -d_meas
                }
            };

            (self.kd as i32 * d_input) >> frac
        } else {
            0
        };

        // ----- Sum + output clamp -----
        let mut out = p + i + d;

        if out > self.out_max {
            out = self.out_max;
        } else if out < self.out_min {
            out = self.out_min;
        }

        out
    }

    /// Set gains at runtime.
    #[inline]
    pub fn set_gains(&mut self, kp: i16, ki: i16, kd: i16) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Get current gains.
    #[inline]
    pub fn gains(&self) -> (i16, i16, i16) {
        (self.kp, self.ki, self.kd)
    }

    /// Set derivative mode at runtime.
    #[inline]
    pub fn set_derivative_mode(&mut self, mode: DerivativeMode) {
        self.mode = mode;
    }

    /// Get current derivative mode.
    #[inline]
    pub fn derivative_mode(&self) -> DerivativeMode {
        self.mode
    }

    /// Set output limits at runtime.
    #[inline]
    pub fn set_output_limits(&mut self, out_min: i32, out_max: i32) {
        let (out_min, out_max) = if out_min <= out_max {
            (out_min, out_max)
        } else {
            (out_max, out_min)
        };
        self.out_min = out_min;
        self.out_max = out_max;
    }

    /// Get current output limits.
    #[inline]
    pub fn output_limits(&self) -> (i32, i32) {
        (self.out_min, self.out_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_p_only() {
        // kp = 1.0 in Q8.8 = 256
        let mut p: PControllerI16 = PControllerI16::new_auto_anti_windup(
            256, // kp = 1.0
            0,   // ki (ignored)
            0,   // kd (ignored)
            DerivativeMode::OnError,
            -1000,
            1000,
        );

        // error = 100, output = 1.0 * 100 = 100
        assert_eq!(p.step(100, 0), 100);
        // error = -50, output = 1.0 * -50 = -50
        assert_eq!(p.step(0, 50), -50);
    }

    #[test]
    fn test_pi_accumulation() {
        // kp = 0.5 (128), ki = 0.25 (64)
        let mut pi: PiControllerI16 = PiControllerI16::new_auto_anti_windup(
            128, // kp = 0.5
            64,  // ki = 0.25
            0,   // kd (ignored)
            DerivativeMode::OnError,
            -1000,
            1000,
        );

        // First step: error = 100
        // P = 0.5 * 100 = 50
        // I = 0.25 * 100 = 25 (first accumulation)
        let out1 = pi.step(100, 0);
        assert_eq!(out1, 50 + 25);

        // Second step: same error
        // P = 50
        // I = 25 + 25 = 50
        let out2 = pi.step(100, 0);
        assert_eq!(out2, 50 + 50);
    }

    #[test]
    fn test_output_clamping() {
        let mut p: PControllerI16 = PControllerI16::new_auto_anti_windup(
            256, // kp = 1.0
            0,
            0,
            DerivativeMode::OnError,
            -100,
            100,
        );

        // Large error should be clamped
        assert_eq!(p.step(500, 0), 100);
        assert_eq!(p.step(0, 500), -100);
    }

    #[test]
    fn test_derivative_on_error() {
        // kd = 1.0 (256)
        let mut pid: PidControllerI16 = PidControllerI16::new_auto_anti_windup(
            0,   // kp
            0,   // ki
            256, // kd = 1.0
            DerivativeMode::OnError,
            -1000,
            1000,
        );

        // First step: error = 100, prev_error = 0, d_input = 100
        assert_eq!(pid.step(100, 0), 100);
        // Second step: error = 100, prev_error = 100, d_input = 0
        assert_eq!(pid.step(100, 0), 0);
        // Third step: error = 50, prev_error = 100, d_input = -50
        assert_eq!(pid.step(50, 0), -50);
    }

    #[test]
    fn test_derivative_on_measurement() {
        // kd = 1.0 (256)
        let mut pid: PidControllerI16 = PidControllerI16::new_auto_anti_windup(
            0,   // kp
            0,   // ki
            256, // kd = 1.0
            DerivativeMode::OnMeasurement,
            -1000,
            1000,
        );

        // First step: pv = 0, prev_meas = 0, d_input = 0
        assert_eq!(pid.step(100, 0), 0);
        // Second step: pv = 50, prev_meas = 0, d_input = -50
        assert_eq!(pid.step(100, 50), -50);
        // Third step: pv = 50, prev_meas = 50, d_input = 0
        assert_eq!(pid.step(100, 50), 0);
    }

    #[test]
    fn test_reset() {
        let mut pi: PiControllerI16 = PiControllerI16::new_auto_anti_windup(
            128, // kp = 0.5
            64,  // ki = 0.25
            0,
            DerivativeMode::OnError,
            -1000,
            1000,
        );

        // Accumulate some integral
        pi.step(100, 0);
        pi.step(100, 0);

        // Reset
        pi.reset();

        // Should behave like fresh controller
        let out = pi.step(100, 0);
        assert_eq!(out, 50 + 25);
    }

    #[test]
    fn test_integral_anti_windup() {
        let mut pi: PiControllerI16 = PiControllerI16::new_auto_anti_windup(
            0,   // kp = 0
            256, // ki = 1.0
            0,
            DerivativeMode::OnError,
            -100,
            100,
        );

        // Keep adding error, integral should be clamped
        for _ in 0..1000 {
            pi.step(1000, 0);
        }

        // Output should be clamped to max
        let out = pi.step(1000, 0);
        assert_eq!(out, 100);

        // After setpoint reversal, integral should unwind
        for _ in 0..1000 {
            pi.step(-1000, 0);
        }
        let out = pi.step(-1000, 0);
        assert_eq!(out, -100);
    }
}
