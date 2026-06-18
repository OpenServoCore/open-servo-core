//! Default clock / baud / buffer sizes for sim devices.
//!
//! Defaults mirror the V006 chip lib so tests reproduce the production
//! topology (`firmware/ch32/src/runtime/registry.rs` + `hal/clocks.rs`).
//! Each value is overridable at build time via an env var so the same
//! tests can run under different chip / link configs without forking.
//! Values are baked in at compile time — set the env vars in
//! `.cargo/config.toml` `[env]` and rebuild.

use osc_core::BaudRate;
use osc_drivers::dxl::uart::codec::rx::{edge_buf_len, rx_buf_len};

use crate::sim::{Clock, SimTime};

/// Default Host status-reply timeout. Mirrors a real DXL host's
/// `LATENCY_TIMER` window — long enough for a coalesced chain reply to
/// land, short enough that tests don't pay multi-ms penalties per command.
/// Override per-call with [`Host::wait_for_status_within`].
pub const DEFAULT_STATUS_TIMEOUT: SimTime = SimTime::from_us(1000);

/// Master device clock frequency, in MHz. Default matches CH32V307-class
/// upper-tier SYSCLK. Override with `OSC_TEST_HOST_CLOCK_MHZ`. Whole-MHz
/// only: downstream baud/timer divisors assume integer-MHz SYSCLK.
pub const HOST_CLOCK_MHZ: u32 = env_u32(option_env!("OSC_TEST_HOST_CLOCK_MHZ"), 144);

/// Servo device clock frequency, in MHz. Default = V006 SYSCLK
/// (HSI 24 MHz × PLL 2). Override with `OSC_TEST_SERVO_CLOCK_MHZ`. Whole-MHz
/// only — see [`HOST_CLOCK_MHZ`].
pub const SERVO_CLOCK_MHZ: u32 = env_u32(option_env!("OSC_TEST_SERVO_CLOCK_MHZ"), 48);

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

/// Bus baud rate. Default = DXL-2.0 spec default (1 Mbaud, idx 3).
/// Override with `OSC_TEST_BAUD_IDX` (0..=5; see [`BaudRate::from_idx`]).
pub const DEFAULT_BAUD: BaudRate = {
    let idx = env_u32(option_env!("OSC_TEST_BAUD_IDX"), 3) as u8;
    match BaudRate::from_idx(idx) {
        Some(b) => b,
        None => panic!("OSC_TEST_BAUD_IDX must be in 0..=5"),
    }
};

/// Anchor back-search depth target (in edges). Mirrors V006's
/// `DXL_SYNC_LOOKBACK_EDGES` in `firmware/ch32/src/runtime/registry.rs`.
/// Override with `OSC_TEST_SYNC_LOOKBACK_EDGES`. See
/// [`osc_drivers::dxl::uart::codec::rx::sync_lookback_edges`] for the
/// CPU / RAM cost per increment.
pub const SYNC_LOOKBACK_EDGES: u16 =
    env_u32(option_env!("OSC_TEST_SYNC_LOOKBACK_EDGES"), 59) as u16;

/// RX byte-ring length, derived from [`SYNC_LOOKBACK_EDGES`].
pub const RX_BUF_LEN: usize = rx_buf_len(SYNC_LOOKBACK_EDGES);

/// Edge-DMA ring length, derived from [`SYNC_LOOKBACK_EDGES`].
pub const EDGE_BUF_LEN: usize = edge_buf_len(SYNC_LOOKBACK_EDGES);

/// TX-buffer length. Derived from
/// `osc_core::services::dxl::limits::DXL_TX_MAX_BYTES`, which is itself
/// env-driven via `OSC_MAX_CONTROL_RW`.
pub const TX_BUF_LEN: usize = osc_core::services::dxl::limits::DXL_TX_MAX_BYTES;

const fn env_u32(value: Option<&'static str>, default: u32) -> u32 {
    match value {
        Some(s) => parse_u32(s),
        None => default,
    }
}

const fn parse_u32(s: &str) -> u32 {
    let bytes = s.as_bytes();
    let mut n: u32 = 0;
    let mut i = 0;
    while i < bytes.len() {
        let b = bytes[i];
        assert!(
            b >= b'0' && b <= b'9',
            "OSC_TEST_* must be a decimal integer"
        );
        n = n * 10 + (b - b'0') as u32;
        i += 1;
    }
    n
}
