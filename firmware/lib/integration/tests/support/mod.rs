//! Shared harness for the integration suite: the baud sweep template and its
//! `Sim` builder. A test `#[apply(matrix)]`s a case per wire baud (§2 of
//! `osc-native-protocol.md`); the folded-in `#[test_log::test]` routes the
//! core-lib `log` traces to env_logger (gated on `RUST_LOG`).
//!
//! Included via `mod support;` from every test binary, so each item carries a
//! dead-code allow — no single binary uses all of them.
#![allow(unused_macros)]

use osc_core::BaudRate;
use osc_integration::sim::Sim;
use rstest_reuse::template;

/// One `#[test]` per wire baud (0.5M / 1M / 2M / 3M — `BaudRate` indices).
/// Edit the `#[values(...)]` list to change baud coverage everywhere at once.
/// Folds in `#[test_log::test]`, so an apply site needs only `#[apply(matrix)]`.
#[template]
#[rstest]
#[test_log::test]
#[allow(dead_code, unused_macros)]
pub fn matrix(#[values(0u8, 1, 2, 3)] baud_idx: u8) {}

/// A fresh `Sim` at the matrix baud.
#[allow(dead_code)]
pub fn sim(baud_idx: u8) -> Sim {
    Sim::new(BaudRate::from_idx(baud_idx).expect("valid baud idx"))
}

/// Sim ticks for one wire byte-time (10 bits) at the matrix baud. Mirrors the
/// sim core's own byte timing (48 ticks/µs); exact integer division for every
/// baud in the set. Tests that assert on wire-proportional bounds build them
/// from this so the bound scales with baud instead of pinning to 1 M.
#[allow(dead_code)]
pub fn byte_ticks(baud_idx: u8) -> u64 {
    let hz = BaudRate::from_idx(baud_idx)
        .expect("valid baud idx")
        .as_hz() as u64;
    48 * 1_000_000 / hz * 10
}

/// reply gap in sim ticks — fixed time at every baud (§7), imported from the
/// driver so the pin cannot drift from the spec constant.
#[allow(dead_code)]
pub fn reply_gap_ticks() -> u64 {
    osc_drivers::bus::REPLY_GAP_US as u64 * 48
}
