//! Wire-disconnect modes — the servo must silence itself while
//! disconnected and answer the next valid packet cleanly once
//! reconnected. Three categories per [[two_disconnect_modes]] and the
//! sim's `Servo::disconnect(reset)` surface:
//!
//! - **Data-line disconnect** (`reset = false`) — servo goes deaf to
//!   the bus but keeps its RAM. On reconnect, prior `set_*` writes and
//!   any in-flight parser state persist; the parser stays mid-Header /
//!   mid-Crc until a fresh sync signature or a `PARSER_FLUSH`-style
//!   drain resets it.
//! - **Power-line disconnect** (`reset = true`) — servo goes deaf AND
//!   the chip-side wipes back to defaults on `connect()`. Prior `set_*`
//!   writes vanish; a `set_dxl_id(5)` before disconnect answers at
//!   `DEFAULT_DXL_ID` afterwards, not id 5.
//! - **`StatusReturnLevel::None`** — dispatcher still runs, so the
//!   parser advances and the control table mutates on Write; only
//!   non-Ping Status frames are suppressed (Ping always replies per
//!   DXL 2.0 spec).
//!
//! Mid-reply disconnect (servo dies during its own TX burst) is
//! deliberately omitted — the sim advances device state via `settle`
//! rather than per-byte, so there's no clean seam between TX bytes to
//! insert the disconnect. Mid-instruction (host TX split across a
//! disconnect) exercises the same parser-wedge surface and is covered
//! below.
//!
//! Fresh-packet-after-long-quiet is baked into every reconnect case:
//! `assert_bus_healthy` opens `wait_for_reply` with the default
//! FTDI-shaped first-byte timeout, so the sim silence between the
//! failed packet and the recovery Ping is already ~50 ms of simulated
//! wall time.

use crate::support::{PARSER_FLUSH, Setup, assert_bus_healthy, encode_ping, matrix, setup_with};
use dxl_protocol::types::Id;
use osc_core::regions::config::addr::comms;
use osc_core::{BaudRate, StatusReturnLevel};
use osc_integration::sim::{DEFAULT_BAUD, DEFAULT_DXL_ID, DEFAULT_RDT_US, Host, Servo, Sim};
use rstest::rstest;
use rstest_reuse::apply;

const TARGET: Id = Id::new(1);
const CUSTOM_ID: Id = Id::new(5);

/// Data-line disconnect before the host sends anything — the target is
/// silent, then reconnect + broadcast Ping brings it back in chain order.
/// Baud sweep because the recovery Ping's TX pacing is baud-shaped and
/// the reconnect+recovery seam is the load-bearing behavior.
#[apply(matrix)]
#[test_log::test]
fn data_line_disconnect_before_instruction_reconnect_recovers(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);
    sim.servo_mut(servos[0]).disconnect(false);

    sim.with_host(host, |h| {
        h.send_ping(TARGET);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "expected silent drop while disconnected, got {:?}",
        sim.host(host).rx_bytes(),
    );

    sim.servo_mut(servos[0]).connect();
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Power-line disconnect before the host sends anything — connect wipes
/// the control table, so the servo answers at `DEFAULT_DXL_ID` regardless
/// of any `set_dxl_id` before disconnect. Baud-independent shape; run at
/// default baud + RDT.
#[test_log::test]
fn power_line_disconnect_before_instruction_next_ping_answers_at_default_id() {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    let servo = sim.add_device(|id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(CUSTOM_ID);
            s.set_baud(DEFAULT_BAUD);
            s.set_rdt_us(DEFAULT_RDT_US);
        })
    });

    {
        let s = sim.servo_mut(servo);
        s.disconnect(true);
        s.connect();
    }

    sim.with_host(host, |h| {
        h.send_ping(CUSTOM_ID);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "prior set_dxl_id must not survive power-cycle, got {:?}",
        sim.host(host).rx_bytes(),
    );

    sim.host_mut(host).clear_logs();
    sim.with_host(host, |h| {
        h.send_ping(DEFAULT_DXL_ID);
        h.wait_for_reply();
    });
    assert!(
        !sim.host(host).rx_bytes().is_empty(),
        "servo must answer at DEFAULT_DXL_ID after power-cycle reset",
    );
}

/// Host sends the first half of a Ping, servo goes data-line-deaf mid-
/// header, host sends the tail. The servo's parser had committed to
/// Header on the prefix bytes; the tail bytes arrive with the wire in
/// deaf mode so they never reach the codec. On reconnect the parser is
/// still mid-header — a real host retries with a `PARSER_FLUSH`-style
/// drain that clears the stuck state, then the recovery broadcast Ping
/// lands. Baud sweep because the split-write TX pacing exercises the
/// wire edge model per baud.
#[apply(matrix)]
#[test_log::test]
fn data_line_disconnect_mid_instruction_reconnect_recovers(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    let bytes = encode_ping(TARGET.as_byte());
    // Split after sync (4) + id (1); parser is mid-Header stage on the
    // prefix, waiting for LEN/INSTR/CRC.
    let (prefix, suffix) = bytes.split_at(5);

    sim.with_host(host, |h| {
        h.send_raw(prefix);
        h.wait_for_reply();
    });
    sim.servo_mut(servos[0]).disconnect(false);
    sim.with_host(host, |h| {
        h.send_raw(suffix);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "tail bytes arrived while deaf; no reply expected, got {:?}",
        sim.host(host).rx_bytes(),
    );

    sim.servo_mut(servos[0]).connect();
    // Real-host retry pattern — drain the wedged mid-Header state
    // before the recovery broadcast lands.
    sim.with_host(host, |h| {
        h.send_raw(&PARSER_FLUSH);
        h.wait_for_reply();
    });
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Same split-write shape as the data-line case, but the disconnect is
/// power-line — reconnect wipes the parser AND the control table, so no
/// PARSER_FLUSH is needed to clear the mid-header state. Servo answers
/// at `DEFAULT_DXL_ID` afterwards. Baud-independent seam; single baud is
/// sufficient.
#[test_log::test]
fn power_line_disconnect_mid_instruction_next_ping_answers_at_default_id() {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    let servo = sim.add_device(|id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(CUSTOM_ID);
            s.set_baud(DEFAULT_BAUD);
            s.set_rdt_us(DEFAULT_RDT_US);
        })
    });

    let bytes = encode_ping(CUSTOM_ID.as_byte());
    let (prefix, suffix) = bytes.split_at(5);

    sim.with_host(host, |h| {
        h.send_raw(prefix);
        h.wait_for_reply();
    });
    {
        let s = sim.servo_mut(servo);
        s.disconnect(true);
        s.connect();
    }
    sim.with_host(host, |h| {
        h.send_raw(suffix);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "post-power-cycle servo is at DEFAULT_DXL_ID; suffix targets stale id, got {:?}",
        sim.host(host).rx_bytes(),
    );

    sim.host_mut(host).clear_logs();
    sim.with_host(host, |h| {
        h.send_ping(DEFAULT_DXL_ID);
        h.wait_for_reply();
    });
    assert!(
        !sim.host(host).rx_bytes().is_empty(),
        "servo must answer at DEFAULT_DXL_ID after power-cycle reset",
    );
}

/// `StatusReturnLevel::None` — third category alongside data / power
/// disconnect per [[two_disconnect_modes]]. Dispatcher runs (control-
/// table mutations still commit on Write), but non-Ping Status frames
/// are suppressed. Demonstrated via Read because Ping always replies
/// per DXL 2.0 spec regardless of SRL. Restoring SRL revives Read
/// replies.
#[test_log::test]
fn srl_none_silences_read_restore_recovers() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, DEFAULT_BAUD, DEFAULT_RDT_US);
    sim.servo_mut(servos[0])
        .set_status_return_level(StatusReturnLevel::None);

    sim.with_host(host, |h| {
        h.send_read(TARGET, comms::ID, 1);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "SRL=None must suppress Read reply, got {:?}",
        sim.host(host).rx_bytes(),
    );

    sim.host_mut(host).clear_logs();
    sim.servo_mut(servos[0])
        .set_status_return_level(StatusReturnLevel::All);
    sim.with_host(host, |h| {
        h.send_read(TARGET, comms::ID, 1);
        h.wait_for_reply();
    });
    assert!(
        !sim.host(host).rx_bytes().is_empty(),
        "restoring SRL=All must revive Read reply",
    );
    assert_bus_healthy(&mut sim, host, &servos);
}
