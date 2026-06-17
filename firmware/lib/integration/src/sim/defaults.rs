//! Default clock / baud / buffer sizes for sim devices.
//!
//! Defaults mirror the V006 chip lib so tests reproduce the production
//! topology (`firmware/ch32/src/runtime/registry.rs` + `hal/clocks.rs`).
//! Each value is overridable at build time via an env var so the same
//! tests can run under different chip / link configs without forking.
//! Values are baked in at compile time — set the env vars in
//! `.cargo/config.toml` `[env]` and rebuild.

use osc_core::BaudRate;

use crate::sim::Clock;

/// Master device clock. Default matches CH32V307-class upper-tier SYSCLK.
/// Override with `OSC_TEST_HOST_CLOCK_HZ`.
pub const HOST_CLOCK: Clock =
    Clock::new(env_u32(option_env!("OSC_TEST_HOST_CLOCK_HZ"), 144_000_000));

/// Servo device clock. Default = V006 SYSCLK (HSI 24 MHz × PLL 2).
/// Override with `OSC_TEST_SERVO_CLOCK_HZ`.
pub const SERVO_CLOCK: Clock =
    Clock::new(env_u32(option_env!("OSC_TEST_SERVO_CLOCK_HZ"), 48_000_000));

/// Bus baud rate. Default = DXL-2.0 spec default (1 Mbaud, idx 3).
/// Override with `OSC_TEST_BAUD_IDX` (0..=5; see [`BaudRate::from_idx`]).
pub const DEFAULT_BAUD: BaudRate = {
    let idx = env_u32(option_env!("OSC_TEST_BAUD_IDX"), 3) as u8;
    match BaudRate::from_idx(idx) {
        Some(b) => b,
        None => panic!("OSC_TEST_BAUD_IDX must be in 0..=5"),
    }
};

/// RX byte-ring length in the codec. Default mirrors
/// `firmware/ch32/src/runtime/registry.rs::DXL_RX_BUF_LEN`.
/// Override with `OSC_TEST_RX_BUF_LEN`.
pub const RX_BUF_LEN: usize = env_usize(option_env!("OSC_TEST_RX_BUF_LEN"), 64);

/// Edge-DMA ring length. Default mirrors
/// `firmware/ch32/src/runtime/registry.rs::DXL_EDGE_BUF_LEN`.
/// Override with `OSC_TEST_EDGE_BUF_LEN`.
pub const EDGE_BUF_LEN: usize = env_usize(option_env!("OSC_TEST_EDGE_BUF_LEN"), 128);

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

const fn env_usize(value: Option<&'static str>, default: usize) -> usize {
    match value {
        Some(s) => parse_u32(s) as usize,
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
