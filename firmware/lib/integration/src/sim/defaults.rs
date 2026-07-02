//! Default clock / baud / buffer sizes for sim devices.
//!
//! Defaults mirror the V006 chip lib so tests reproduce the production
//! topology (`firmware/ch32/src/runtime/registry.rs` + `hal/clocks.rs`).
//! Tests that sweep non-default values use `setup_with(...)` from
//! `tests/support.rs` — see the `matrix` rstest_reuse template.

use osc_core::BaudRate;
use osc_drivers::dxl::uart::codec::edge_buf_len;

use crate::sim::Clock;

/// Master device clock frequency, in MHz. Matches CH32V307-class
/// upper-tier SYSCLK. Whole-MHz only: downstream baud/timer divisors
/// assume integer-MHz SYSCLK.
pub const HOST_CLOCK_MHZ: u32 = 144;

/// Servo device clock frequency, in MHz. V006 SYSCLK (HSI 24 MHz × PLL 2).
/// Whole-MHz only — see [`HOST_CLOCK_MHZ`].
pub const SERVO_CLOCK_MHZ: u32 = 48;

/// Construct a fresh master-device clock at the default frequency. Returned
/// by value so each `Host` owns an independent clock — model HSE/HSI drift
/// per device by overriding via `Host::with_clock` after construction.
pub const fn default_host_clock() -> Clock {
    Clock::new(HOST_CLOCK_MHZ * 1_000_000)
}

/// Construct a fresh servo-device clock at the default frequency. Returned
/// by value so each `Servo` owns an independent clock — model per-device
/// HSI base + drift by overriding via `Servo::set_clock` after construction.
pub const fn default_servo_clock() -> Clock {
    Clock::new(SERVO_CLOCK_MHZ * 1_000_000)
}

/// Bus baud rate. DXL-2.0 spec default (1 Mbaud, idx 3). Tests that sweep
/// other baud rates use `setup_with(n, baud, rdt_us)`.
pub const DEFAULT_BAUD: BaudRate = match BaudRate::from_idx(3) {
    Some(b) => b,
    None => panic!("baud idx 3 is the DXL-2.0 spec default"),
};

/// Per-servo return delay (µs). Mirrors the driver-owned DXL spec
/// factory default (`osc_drivers::dxl::DEFAULT_RDT_2US`); tests that
/// sweep other RDT values use `setup_with(n, baud, rdt_us)`.
pub const DEFAULT_RDT_US: u32 = (osc_drivers::dxl::DEFAULT_RDT_2US as u32) * 2;

/// Default RDT in nanoseconds — the same delay as [`DEFAULT_RDT_US`]
/// pre-scaled into the unit the wire-timing tests assert against.
pub const DEFAULT_RDT_NS: u64 = (DEFAULT_RDT_US as u64) * 1_000;

/// RX byte-ring length. Mirrors V006's `DXL_RX_BUF_LEN` in
/// `firmware/ch32/src/runtime/registry.rs`. Gated from below by Fast
/// Last's `RX_BUF_LEN/2 ≥ grid_interval`.
pub const RX_BUF_LEN: usize = 32;

/// Edge-DMA ring length, derived from [`RX_BUF_LEN`] via
/// [`osc_drivers::dxl::uart::codec::edge_buf_len`].
pub const EDGE_BUF_LEN: usize = edge_buf_len(RX_BUF_LEN);

/// TX-buffer length. Derived from
/// `osc_core::services::dxl::limits::DXL_TX_MAX_BYTES`, which is itself
/// env-driven via `OSC_MAX_CONTROL_RW`.
pub const TX_BUF_LEN: usize = osc_core::services::dxl::limits::DXL_TX_MAX_BYTES;
