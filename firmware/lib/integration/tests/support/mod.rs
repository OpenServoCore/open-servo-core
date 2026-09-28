//! Shared harness for the integration suite: the baud sweep template and its
//! `Sim` builder. A test `#[apply(matrix)]`s a case per wire baud (sec 2 of
//! `osc-native-protocol.md`); the folded-in `#[test_log::test]` routes the
//! core-lib `log` traces to env_logger (gated on `RUST_LOG`).
//!
//! Included via `mod support;` from every test binary, so each item carries a
//! dead-code allow -- no single binary uses all of them.
#![allow(unused_macros)]

use osc_integration::sim::Sim;
use osc_servo_core::BaudRate;
use osc_servo_core::pot_lut::{INTERVALS, KNOTS};
use rstest_reuse::template;

/// mg90-a on the 2S session, stops 209/3849, covered 542..3520 (bringup
/// captures/mg90/pot-lut-mg90-a-grid.json), the same 256 knots the core
/// unit tests carry, pinned to the Python reference by CRC in `pot_lut.rs`.
#[allow(dead_code)]
pub const MG90_A_MIN: u16 = 209;
#[allow(dead_code)]
pub const MG90_A_MAX: u16 = 3849;
#[allow(dead_code)]
pub const MG90_A: [i16; INTERVALS] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 4, -1, -5, -10, -16, -21, -23, -21, -14, -12, -11, -11, -14, //
    -22, -25, -28, -28, -25, -27, -28, -29, -31, -30, -22, -23, -24, -20, -18, -20, //
    -21, -29, -32, -35, -34, -33, -34, -33, -28, -27, -25, -25, -27, -30, -28, -25, //
    -28, -24, -23, -23, -25, -23, -6, 3, 2, -2, -6, -8, -8, -3, -3, -7, //
    -12, -14, -16, -17, -17, -12, 2, 15, 19, 16, 15, 17, 22, 25, 28, 30, //
    28, 29, 29, 28, 25, 21, 19, 16, 14, 13, 12, 11, 6, 10, 20, 31, //
    33, 38, 42, 42, 39, 35, 30, 26, 23, 22, 23, 24, 22, 26, 27, 30, //
    34, 38, 41, 44, 42, 42, 42, 41, 43, 44, 45, 44, 43, 44, 45, 44, //
    42, 42, 45, 47, 53, 59, 66, 71, 70, 70, 69, 65, 60, 56, 51, 47, //
    44, 45, 43, 43, 42, 40, 39, 38, 39, 37, 35, 34, 33, 30, 27, 23, //
    20, 18, 14, 9, 7, 8, 13, 16, 17, 18, 16, 13, 7, 6, 5, 6, //
    3, 2, 7, 10, 10, 8, 13, 12, 9, 6, 2, -1, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
];

/// The mg90-a table with the fixed last knot appended.
#[allow(dead_code)]
pub fn mg90_a() -> [i16; KNOTS] {
    let mut k = [0; KNOTS];
    k[..INTERVALS].copy_from_slice(&MG90_A);
    k
}

/// One `#[test]` per wire baud (0.5M / 1M / 2M / 3M -- `BaudRate` indices).
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
/// sim core's own byte timing (48 ticks/us); exact integer division for every
/// baud in the set. Tests that assert on wire-proportional bounds build them
/// from this so the bound scales with baud instead of pinning to 1 M.
#[allow(dead_code)]
pub fn byte_ticks(baud_idx: u8) -> u64 {
    let hz = BaudRate::from_idx(baud_idx)
        .expect("valid baud idx")
        .as_hz() as u64;
    48 * 1_000_000 / hz * 10
}

/// reply gap in sim ticks -- fixed time at every baud (sec 7), imported from the
/// driver so the pin cannot drift from the spec constant.
#[allow(dead_code)]
pub fn reply_gap_ticks() -> u64 {
    osc_servo_drivers::bus::REPLY_GAP_US as u64 * 48
}
