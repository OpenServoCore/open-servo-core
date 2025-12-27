//! Stub FaultSink and TelemetrySink implementations.
//!
//! These are minimal no-op implementations for the green build.
//! Replace with real implementations (RTT, UART, CAN, ringbuffer) later.

use open_servo_kernel_api::faults::{FaultKind, FaultSink};
use open_servo_kernel_api::telemetry::{TelemetryId, TelemetrySink};

/// Stub fault sink that does nothing.
///
/// Stage-0: faults are raised but not latched anywhere.
/// Replace with a real FaultLatch when you need fault gating.
pub struct StubFaultSink;

impl FaultSink for StubFaultSink {
    fn raise(&mut self, _kind: FaultKind) -> bool {
        // No-op: always returns "not newly latched"
        false
    }
}

/// Stub telemetry sink that discards all samples.
///
/// Stage-0: telemetry is ignored.
/// Replace with RTT, UART, or ringbuffer sink for system ID.
pub struct StubTelemetrySink;

impl TelemetrySink for StubTelemetrySink {
    fn sample_i32(&mut self, _id: TelemetryId, _value: i32) {}
    fn sample_u32(&mut self, _id: TelemetryId, _value: u32) {}
}
