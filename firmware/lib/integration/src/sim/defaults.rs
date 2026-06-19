//! Default clock / baud / buffer sizes for sim devices.
//!
//! Defaults mirror the V006 chip lib so tests reproduce the production
//! topology (`firmware/ch32/src/runtime/registry.rs` + `hal/clocks.rs`).
//! Tests that sweep non-default values use `setup_with(...)` from
//! `tests/support.rs` — see the `matrix` rstest_reuse template.

use osc_core::BaudRate;
use osc_drivers::dxl::uart::codec::rx::{edge_buf_len, rx_buf_len};

use crate::sim::{Clock, SimTime};

/// Default Host status-reply timeout. Mirrors a real DXL host's
/// `LATENCY_TIMER` window — long enough for a coalesced chain reply to
/// land, short enough that tests don't pay multi-ms penalties per command.
/// Override per-call with [`Host::wait_for_status_within`].
pub const DEFAULT_STATUS_TIMEOUT: SimTime = SimTime::from_us(1000);

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

/// Per-servo return delay (µs). DXL-2.0 spec factory default (250 µs).
/// Tests that sweep other RDT values use `setup_with(n, baud, rdt_us)`.
pub const DEFAULT_RDT_US: u32 = 250;

/// Anchor back-search depth target (in edges). Mirrors V006's
/// `DXL_SYNC_LOOKBACK_EDGES` in `firmware/ch32/src/runtime/registry.rs`.
/// See [`osc_drivers::dxl::uart::codec::rx::sync_lookback_edges`] for the
/// CPU / RAM cost per increment.
pub const SYNC_LOOKBACK_EDGES: u16 = 59;

/// RX byte-ring length, derived from [`SYNC_LOOKBACK_EDGES`].
pub const RX_BUF_LEN: usize = rx_buf_len(SYNC_LOOKBACK_EDGES);

/// Edge-DMA ring length, derived from [`SYNC_LOOKBACK_EDGES`].
pub const EDGE_BUF_LEN: usize = edge_buf_len(SYNC_LOOKBACK_EDGES);

/// TX-buffer length. Derived from
/// `osc_core::services::dxl::limits::DXL_TX_MAX_BYTES`, which is itself
/// env-driven via `OSC_MAX_CONTROL_RW`.
pub const TX_BUF_LEN: usize = osc_core::services::dxl::limits::DXL_TX_MAX_BYTES;
