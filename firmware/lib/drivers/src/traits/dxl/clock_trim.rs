/// Trim a clock by chip-independent ppm corrections. The provider quantizes
/// to its nearest hardware step and clamps to its envelope; the driver-side
/// integrator stays free of chip vocabulary (HSI step Hz, register width).
///
/// `STEP_PPM` and `ENVELOPE_PPM` let the integrator pre-compute its
/// half-step emission threshold and clamp pending corrections without
/// importing chip consts. `apply_ppm` semantics are *absolute relative to
/// factory cal* — the register, not an accumulator, is the state of record;
/// brownout-clean and matches the M3 host-CAL hook.
pub trait ClockTrim {
    /// Smallest correction the provider can represent, in ppm. The
    /// integrator gates its emission threshold at half this and never emits
    /// a correction the provider can't act on.
    const STEP_PPM: u32;
    /// Saturation rails relative to factory cal, in ppm: `(min, max)`. The
    /// integrator clamps pending corrections so it stops emitting once
    /// saturated.
    const ENVELOPE_PPM: (i32, i32);
    /// Apply an absolute correction relative to factory cal, in ppm.
    /// Provider quantizes to the nearest hardware step and clamps.
    fn apply_ppm(&mut self, ppm: i32);
}
