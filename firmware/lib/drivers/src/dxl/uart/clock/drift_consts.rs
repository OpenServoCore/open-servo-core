//! Batch sizes for [`DriftIntegrator`]. Split from the integrator so the
//! control law reads as policy and this file as the tuned span counts.
//!
//! [`DriftIntegrator`]: super::drift_integrator::DriftIntegrator

/// Spans that close the boot (first-emit) batch. Short so the first burst
/// of contiguous Instruction traffic lands the factory-drift correction.
pub(super) const DRIFT_MIN_SAMPLES_BOOT: u16 = 6;
/// Spans that close a steady batch after the boot phase's one-shot
/// emission. The longer window's half-step deadband clears one-sided
/// entry-latency noise before emitting.
pub(super) const DRIFT_MIN_SAMPLES_STEADY: u16 = 20;
