//! Spec-driven black-box integration tests for the OpenServoCore DXL stack.
//!
//! See `tests/` for the integration suite. This crate's library surface
//! (`TestStack`, `report`) is populated in subsequent tasks; the
//! [`mocks`] module wraps `osc-drivers`'s mockall-generated mocks with
//! state companions and spy harnesses tailored to integration testing.

pub mod mocks;
