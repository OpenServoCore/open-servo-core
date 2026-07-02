//! Shared helpers for cargo-test-driven bench suites.
//!
//! Hardware config comes from env — [[`BusArgs::from_env`]] reads
//! `BENCH_PORT` (empty=autodetect) + `BENCH_BAUD` (default 1_000_000).
//!
//! **Serialisation.** Tests are marked `#[serial_test::serial]` so cargo's
//! default parallel runner can't race the shared `Bus`.
//!
//! **State discipline.** One `Bus` per test binary (see [`bus`]), acquired
//! lazily on the first call. Discovery + baud switch happens once at that
//! acquisition; every subsequent test reuses the running chip without any
//! reset. Tests that mutate persistent chip state (torque_enable, mode,
//! pending RegWrite) restore it before returning so adjacent tests see a
//! clean starting point without a reboot in the setup path.

#![allow(dead_code, unused_imports)]

use std::sync::{LazyLock, Mutex, MutexGuard};

use anyhow::Result;
use bench::{Bus, BusArgs, DEFAULT_IDLE_US, StatusReply, parse_status_reply};
use dxl_protocol::types::Id;

/// Foreign servo ids used as chain positioning stand-ins. Chosen high
/// enough that a real bench setup won't have colliding assignments.
pub const FOREIGN_A: u8 = 99;
pub const FOREIGN_B: u8 = 100;
/// DXL broadcast id — reused across silence + broadcast-write checks.
pub const BROADCAST_ID: u8 = 0xFE;

/// `comms::TORQUE_ENABLE` — RW, control-table base. Round-tripped by the
/// state-mutating tests and cleared by their restore-guards.
pub const CONTROL_BASE_ADDR: u16 = 0x0300;
/// `comms::MODE` at CONTROL_BASE + 1.
pub const CONTROL_MODE_ADDR: u16 = CONTROL_BASE_ADDR + 1;

static BUS: LazyLock<Mutex<Bus>> = LazyLock::new(|| {
    let args = BusArgs::from_env().expect("parse BENCH_* env");
    let bus = Bus::start(args).expect("bench Bus::start (no pirate/servo attached?)");
    Mutex::new(bus)
});

/// Acquire the shared `Bus`. First caller triggers discovery + baud switch;
/// subsequent callers reuse the running chip.
///
/// Recovers from poison: a panic mid-xfer leaves the mutex poisoned but
/// the underlying `Bus` state is fine to reuse (each xfer starts with
/// `drain_all` which resets the pirate ring). Otherwise a single test
/// failure would cascade into every subsequent test.
pub fn bus() -> MutexGuard<'static, Bus> {
    BUS.lock().unwrap_or_else(|e| e.into_inner())
}

/// Servo id resolved during discovery. Take a `&Bus` so callers holding
/// the shared-bus guard don't re-lock the mutex (std Mutex is not
/// re-entrant → recursive `bus()` deadlocks).
pub fn servo_id(bus: &Bus) -> Id {
    Id::new(bus.id())
}

/// Xfer `frame`, parse the Status reply, assert the header error byte is
/// zero. Returns the parsed reply for further assertions.
pub fn expect_status_ok(bus: &mut Bus, frame: &[u8]) -> Result<StatusReply> {
    let reply = bus.xfer_reply(frame, DEFAULT_IDLE_US)?.expect("no reply");
    let parsed = parse_status_reply(&reply, None)?;
    assert_eq!(
        parsed.error.as_byte(),
        0,
        "status header error 0x{:02X}",
        parsed.error.as_byte(),
    );
    Ok(parsed)
}

/// Xfer `frame` with a caller-supplied idle window; assert the wire stays
/// silent (no Status reply).
pub fn expect_no_reply(bus: &mut Bus, frame: &[u8], idle_us: u32) {
    let reply = bus.xfer_reply(frame, idle_us).expect("pirate xfer");
    assert!(
        reply.is_none(),
        "expected wire silence, got {} bytes",
        reply.as_ref().map(|r| r.len()).unwrap_or(0),
    );
}

/// One byte-time on the wire, in µs. Used to size wait windows for
/// broadcast + silence checks.
pub fn byte_time_us(baud: u32) -> u32 {
    (10u32 * 1_000_000).div_ceil(baud)
}

/// Silence-window budget for a request that names `n_slots` chain
/// positions each carrying `payload_len` data bytes. Covers the full
/// chain (slot 0..n-1) plus a small margin so a genuine reply always
/// wins vs a false-quiet exit.
pub fn silent_window_us(baud: u32, max_slot_index: u32, payload_len: u32) -> u32 {
    let slot_bytes = 14 + payload_len;
    (max_slot_index + 1) * slot_bytes * byte_time_us(baud) + 20_000
}
