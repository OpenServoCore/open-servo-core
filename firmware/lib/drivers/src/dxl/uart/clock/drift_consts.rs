//! Batch sizes for [`DriftIntegrator`]. Split from the integrator so the
//! control law reads as policy and this file as the tuned span counts.
//!
//! [`DriftIntegrator`]: super::drift_integrator::DriftIntegrator

/// Spans that close a steady batch after the boot phase's one-shot
/// emission. The longer window's half-step deadband clears one-sided
/// entry-latency noise before emitting.
///
/// The boot batch has no span count of its own — it closes at the first
/// RX packet boundary that holds ≥ 1 span (see
/// [`DriftIntegrator::on_rx_packet_end`]) so a single short-packet window
/// span lands the factory-drift correction inside the first exchange.
///
/// [`DriftIntegrator::on_rx_packet_end`]: super::drift_integrator::DriftIntegrator::on_rx_packet_end
pub(super) const DRIFT_MIN_SAMPLES_STEADY: u16 = 20;
