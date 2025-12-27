//! Minimal structured telemetry boundary.
//!
//! System identification needs structured, numeric telemetry that can be recorded
//! consistently across builds and boards.
//!
//! This module defines a very small sink interface. Concrete implementations live
//! in kernel/board crates (RTT, UART, CAN, ringbuffer, etc.).
//!
//! ```rust,ignore
//! use open_servo_kernel_api::telemetry::{TelemetrySink, ids};
//!
//! // Record a couple signals (kernel-side).
//! fn record(t: &mut impl TelemetrySink, pos_cdeg32: i32, curr_ma: i32) {
//!     t.sample_i32(ids::POS_CDEG32, pos_cdeg32);
//!     t.sample_i32(ids::CURR_MA, curr_ma);
//!     t.incr(ids::SATURATION_COUNT); // counters are just numeric signals too
//! }
//! ```
//!
//! ## Fault observability
//!
//! Nodes can **raise** faults via [`crate::FaultSink`]. The fault state/latch is owned
//! by the kernel and **is not readable from nodes**.
//!
//! Kernels should:
//! - gate control outputs when faulted (emit safe outputs / skip control stages)
//! - emit fault snapshots through telemetry (see [`ids::FAULT_MASK`] etc.)
//!
//! This keeps the safety policy centralized while still enabling system-id,
//! debugging, and dashboards.

/// Telemetry signal ID.
///
/// `u16` is small enough for embedded and can be used directly as a register-like key.
/// The mapping is project-defined; `ids::*` below suggests a common baseline.
pub type TelemetryId = u16;

/// Minimal telemetry sink.
///
/// Design goals:
/// - no allocations
/// - numeric-only
/// - usable with or without `defmt`
///
/// Note: this is intentionally *not* a logging framework.
/// It's a structured data plumbing interface.
///
/// ## Counters vs events
/// Telemetry backends vary:
/// - some store the *latest value* per id (sample-based)
/// - some append a *stream of samples* (log-based)
///
/// To support both styles, this trait provides:
/// - `sample_*` for absolute values
/// - `incr/add_*` for counter-style accumulation
/// - `pulse_*` for “event happened” markers
pub trait TelemetrySink {
    /// Record a signed sample (absolute value).
    fn sample_i32(&mut self, id: TelemetryId, value: i32);

    /// Record an unsigned sample (absolute value).
    fn sample_u32(&mut self, id: TelemetryId, value: u32);

    /// Add `delta` to a signed counter.
    ///
    /// Backends that only support “latest value” may interpret this as “write delta”.
    /// If you need strict counters, keep the counter in kernel state and emit it with `sample_*`.
    #[inline]
    fn add_i32(&mut self, id: TelemetryId, delta: i32) {
        self.sample_i32(id, delta);
    }

    /// Add `delta` to an unsigned counter.
    ///
    /// Backends that only support “latest value” may interpret this as “write delta”.
    /// If you need strict counters, keep the counter in kernel state and emit it with `sample_*`.
    #[inline]
    fn add_u32(&mut self, id: TelemetryId, delta: u32) {
        self.sample_u32(id, delta);
    }

    /// Convenience: increment an unsigned counter by 1.
    #[inline]
    fn incr(&mut self, id: TelemetryId) {
        self.add_u32(id, 1);
    }

    /// Emit a “pulse” event (an instantaneous marker).
    ///
    /// Use this for things like:
    /// - “fault raised”
    /// - “mode changed”
    /// - “reset happened”
    ///
    /// The default encoding is `sample_u32(id, 1)`. Stream backends will record an
    /// explicit event sample; snapshot backends may show a sticky `1` until overwritten.
    /// Tooling should treat pulses as edge markers, not state.
    #[inline]
    fn pulse(&mut self, id: TelemetryId) {
        self.sample_u32(id, 1);
    }
}

/// Suggested common telemetry IDs.
///
/// You can ignore these and define your own mapping. The main benefit of these
/// constants is reusable tooling (system-id scripts, plotters, dashboards).
///
/// Guidelines:
/// - Prefer stable IDs over time (tooling depends on them).
/// - Use `*_RAW` for wire-friendly integer representations (e.g. Q15-like effort).
pub mod ids {
    use super::TelemetryId;

    // =========================
    // Common signals (physics)
    // =========================
    pub const POS_CDEG32: TelemetryId = 1;
    pub const VEL_DPS10: TelemetryId = 2;
    pub const CURR_MA: TelemetryId = 3;
    pub const VOLT_MV: TelemetryId = 4;

    /// Dimensionless normalized actuator command (raw `i16` stored as `i32`).
    ///
    /// - **Range:** -32768..32767 (full `i16`, Q15-ish)
    /// - **Meaning:** "how much actuator authority", not current/torque/voltage
    ///
    /// This is the kernel-level "what we asked the actuator to do" signal.
    /// Board maps to PWM duty (BDC) or Iq request (BLDC FOC).
    pub const EFFORT_RAW: TelemetryId = 5;

    // =========================
    // Kernel state (high level)
    // =========================
    /// Current active operating mode (`OperatingMode` discriminant).
    pub const MODE: TelemetryId = 20;

    /// Whether actuator output is enabled (kernel-defined; typically 0/1).
    pub const ENGAGED: TelemetryId = 21;

    // =========================
    // Mode observability (kernel-owned)
    // =========================

    /// Monotonic mode sequence counter (increments on each accepted mode change).
    pub const MODE_SEQ: TelemetryId = 26;

    /// Last requested mode (useful if requests can be deferred/rejected).
    pub const MODE_REQUESTED: TelemetryId = 27;

    /// Last mode error (kernel-defined mapping; 0 = none).
    pub const MODE_ERROR: TelemetryId = 28;

    // =========================
    // Fault snapshots (kernel-owned)
    // =========================

    /// Bitmask of currently latched faults.
    ///
    /// Encoding is kernel-defined but should be stable for a given product line.
    /// Common approach: bit index corresponds to `FaultKind` discriminant.
    pub const FAULT_MASK: TelemetryId = 22;

    /// Most recently raised fault kind (enum discriminant), or 0 if none.
    pub const FAULT_LAST_KIND: TelemetryId = 23;

    /// Monotonic fault sequence counter (increments on raise/clear), optional but handy.
    pub const FAULT_SEQ: TelemetryId = 24;

    // =========================
    // Reset events (kernel-owned)
    // =========================

    /// Most recent reset reason (enum discriminant), or 0 if none.
    /// See [`crate::reset::ResetReason`] for the mapping.
    pub const RESET_LAST_REASON: TelemetryId = 30;

    /// Monotonic reset sequence counter (increments on each reset propagation).
    /// Useful for tooling to detect reset edges without polling reason.
    pub const RESET_SEQ: TelemetryId = 31;

    // Per-reason counters (histograms)
    pub const RESET_COUNT_DISENGAGE: TelemetryId = 32;
    pub const RESET_COUNT_ENGAGE: TelemetryId = 33;
    pub const RESET_COUNT_FAULT_RAISED: TelemetryId = 34;
    pub const RESET_COUNT_FAULT_CLEARED: TelemetryId = 35;
    pub const RESET_COUNT_MODE_CHANGED: TelemetryId = 36;
    pub const RESET_COUNT_CONFIG_CHANGED: TelemetryId = 37;

    // =========================
    // Debug counters
    // =========================
    pub const SATURATION_COUNT: TelemetryId = 40;

    // Event-style pulse IDs (optional but useful)
    pub const EVT_MODE_CHANGED: TelemetryId = 60;
    pub const EVT_FAULT_RAISED: TelemetryId = 61;
    pub const EVT_FAULT_CLEARED: TelemetryId = 62;
    pub const EVT_RESET: TelemetryId = 63;
}
