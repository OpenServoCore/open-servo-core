//! Spec-driven black-box integration tests for the OpenServoCore DXL stack.
//!
//! See `tests/` for the integration suite. The [`sim`] module provides a
//! discrete-event hardware simulator with per-bit UART line fidelity; the
//! [`mocks`] module wraps `osc-drivers`'s mockall-generated mocks with
//! state companions and spy harnesses tailored to integration testing.

pub mod mocks;
pub mod sim;
