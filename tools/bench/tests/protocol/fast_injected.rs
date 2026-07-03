//! FAST chain positive cases on the single-DUT rig — the pirate plays
//! the missing chain members via `Bus::inject_then_send`, so the chip's
//! slot k > 0 paths (status-start observation, deferred schedule, fold +
//! chain-CRC patch) run against real wire bytes for the first time on
//! silicon.
//!
//! Timing is REPORTED, not asserted (run with `--nocapture` to see it on
//! passing tests): the software-armed TX start (#134 pending) is expected
//! to land the k > 0 slot start off the contiguous ideal — these lines
//! measure how far. Wire-shape and health assertions are hard: the chip
//! must emit the right bytes at its position and answer a ping after
//! every exchange.
//!
//! Chain-position map (fast sync read, L = 2 unless noted):
//! - `middle`: ids `[INJ, chip, FOREIGN_A]`, pirate injects slot 0. The
//!   k = 2 slot is declared but never injected — the pirate has one
//!   scheduled-send slot, and a successor's bytes can't affect the
//!   chip's own k = 1 timing.
//! - `last`:   ids `[INJ, chip]`, pirate injects slot 0; the chip's
//!   trailing chain CRC must validate over injected + own bytes.
//! - `first`:  ids `[chip, INJ]`, pirate injects a successor after the
//!   chip's emission; chip must tolerate the trailing foreign bytes.
//! - `only`:   ids `[chip]` — pure single-slot timing baseline.

use bench::{
    INJ_ID, ReplyCapture, build_fast_sync_read, build_inj_first_bytes, build_ping,
    parse_fast_response,
};
use dxl_protocol::Id;
use dxl_protocol::types::StatusError;
use dxl_protocol::wire::HEADER;
use serial_test::serial;

use super::support::{FOREIGN_A, bus, expect_status_ok, servo_id, silent_window_us};

/// Read length for every slot in these chains.
const L: usize = 2;
/// Injected slot-0 payload.
const INJ_DATA: [u8; L] = [0xAA, 0xBB];
/// Idle gap before the pirate injects slot 0 (mirrors tool-fast-stress).
const INJ_AFTER_IDLE_US: u32 = 250;
/// Idle gap before the pirate injects a SUCCESSOR — must exceed the
/// chip's RDT + First emission so the injection can't collide with the
/// chip's own reply, while staying under the pirate's TIM4 one-shot
/// scheduling horizon (16-bit at 144 MHz ≈ 455 µs; a larger value wraps
/// and fires ~immediately, driving the injection INTO the chip's RDT
/// window).
const SUCCESSOR_AFTER_IDLE_US: u32 = 430;

/// `LEN` field of the coalesced Status frame for an n-slot chain:
/// instruction byte + err + n·(id-tag + data) … per the Fast wire shape
/// `1 + Σ(2 + L_k) + 2`.
fn chain_packet_length(slot_count: usize) -> u16 {
    (1 + slot_count * (2 + L) + 2) as u16
}

/// Per-byte wire duration in µs at `baud` (10 bits per frame).
fn byte_time_us(baud: u32) -> f64 {
    10.0 * 1_000_000.0 / baud as f64
}

/// Wire bytes past the request echo.
fn frame_bytes(cap: &ReplyCapture, req_len: usize) -> Vec<u8> {
    cap.stamps.iter().skip(req_len).map(|s| s.byte).collect()
}

/// Report the chip's slot-start excess over the contiguous ideal:
/// `chip_first_byte − (status_start + bytes_before · byte_time)`.
/// Positive = late (gap on the wire), negative = early (drove into the
/// predecessor's bytes).
fn report_k_gt_zero_timing(
    label: &str,
    cap: &ReplyCapture,
    req_len: usize,
    inj_len: usize,
    baud: u32,
    hz_per_us: f64,
) {
    let status_start = cap.stamps[req_len].tick;
    let chip_first = cap.stamps[req_len + inj_len].tick;
    let actual_us = chip_first.wrapping_sub(status_start) as f64 / hz_per_us;
    let ideal_us = inj_len as f64 * byte_time_us(baud);
    eprintln!(
        "fast-injected[{label}] baud={baud}: chip slot start = status_start + {actual_us:.2} µs \
         (contiguous ideal {ideal_us:.2} µs, excess {:+.2} µs, {:+.2} byte_times)",
        actual_us - ideal_us,
        (actual_us - ideal_us) / byte_time_us(baud),
    );
}

fn assert_healthy(bus: &mut bench::Bus, id: Id) {
    let ping = build_ping(id).expect("encode ping");
    let reply = expect_status_ok(bus, &ping).expect("chip answers ping after the exchange");
    assert_eq!(reply.id, id);
}

#[test]
#[serial]
fn fast_middle_with_injected_first_emits_on_wire() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let hz = bus.hz_per_us().expect("pirate hz") as f64;

    let inj = build_inj_first_bytes(
        Id::new(INJ_ID),
        StatusError::OK,
        &INJ_DATA,
        chain_packet_length(3),
    )
    .unwrap();
    assert_eq!(inj.len(), 10 + L, "First emission wire shape");
    let req = build_fast_sync_read(0, L as u16, &[INJ_ID, id.as_byte(), FOREIGN_A]).unwrap();

    let cap = bus
        .inject_then_send(
            &inj,
            INJ_AFTER_IDLE_US,
            &req,
            silent_window_us(baud, 2, L as u32),
        )
        .expect("pirate xfer");

    let frame = frame_bytes(&cap, req.len());
    assert!(
        frame.len() >= inj.len() + 2 + L,
        "expected injected First + chip Middle bytes, got {} wire bytes: {frame:02X?}",
        frame.len(),
    );
    assert_eq!(&frame[..inj.len()], &inj[..], "injected slot 0 echo");
    let chip = &frame[inj.len()..];
    assert_eq!(chip.len(), 2 + L, "chip Middle = err + id + data");
    assert_eq!(chip[0], 0, "err = 0x{:02X}", chip[0]);
    assert_eq!(chip[1], id.as_byte());

    report_k_gt_zero_timing("middle", &cap, req.len(), inj.len(), baud, hz);
    assert_healthy(&mut bus, id);
}

#[test]
#[serial]
fn fast_last_with_injected_first_patches_chain_crc() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let hz = bus.hz_per_us().expect("pirate hz") as f64;

    let inj = build_inj_first_bytes(
        Id::new(INJ_ID),
        StatusError::OK,
        &INJ_DATA,
        chain_packet_length(2),
    )
    .unwrap();
    let req = build_fast_sync_read(0, L as u16, &[INJ_ID, id.as_byte()]).unwrap();

    let cap = bus
        .inject_then_send(
            &inj,
            INJ_AFTER_IDLE_US,
            &req,
            silent_window_us(baud, 1, L as u32),
        )
        .expect("pirate xfer");

    let frame = frame_bytes(&cap, req.len());
    let expected = inj.len() + 2 + L + 2; // + own body + chain CRC
    assert!(
        frame.len() >= expected,
        "expected full chain frame ({expected} bytes), got {}: {frame:02X?}",
        frame.len(),
    );
    report_k_gt_zero_timing("last", &cap, req.len(), inj.len(), baud, hz);

    // The headline hardware validation: the chip's fold accumulated the
    // INJECTED bytes and patched a chain CRC covering both slots —
    // parse_fast_response rejects the frame on any CRC mismatch.
    let slots = parse_fast_response(&frame, &[L, L]).expect("chain frame parses with valid CRC");
    assert_eq!(slots.len(), 2);
    assert_eq!(slots[0].id.as_byte(), INJ_ID);
    assert_eq!(slots[0].data, INJ_DATA);
    assert_eq!(slots[1].id, id);
    assert_eq!(slots[1].data.len(), L);

    assert_healthy(&mut bus, id);
}

#[test]
#[serial]
fn fast_first_with_injected_successor_stays_healthy() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let hz = bus.hz_per_us().expect("pirate hz") as f64;

    // Successor (k = 1, Last of a 2-slot chain): body + placeholder CRC.
    // The chain CRC can't be precomputed host-side (it covers the chip's
    // First bytes, whose register values we don't fix) — the target here
    // is the chip's own First timing and its tolerance of trailing
    // foreign bytes, not host-side frame acceptance.
    let mut successor = vec![0x00, INJ_ID];
    successor.extend_from_slice(&INJ_DATA);
    successor.extend_from_slice(&[0x00, 0x00]);
    let req = build_fast_sync_read(0, L as u16, &[id.as_byte(), INJ_ID]).unwrap();

    let cap = bus
        .inject_then_send(
            &successor,
            SUCCESSOR_AFTER_IDLE_US,
            &req,
            silent_window_us(baud, 1, L as u32),
        )
        .expect("pirate xfer");

    let frame = frame_bytes(&cap, req.len());
    let first_len = 10 + L;
    assert!(
        frame.len() >= first_len,
        "expected chip First emission, got {} wire bytes: {frame:02X?}",
        frame.len(),
    );
    let chip = &frame[..first_len];
    assert_eq!(&chip[..4], &HEADER);
    assert_eq!(chip[9], id.as_byte());

    let timing = cap.timing.expect("reply timing captured");
    let first_delay_us = timing.reply_first.wrapping_sub(timing.req_end) as f64 / hz;
    eprintln!(
        "fast-injected[first] baud={baud}: chip First starts {first_delay_us:.2} µs after \
         request end (expect ≈ chip RDT + 1 byte_time)",
    );

    assert_healthy(&mut bus, id);
}

#[test]
#[serial]
fn fast_only_reports_timing() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let hz = bus.hz_per_us().expect("pirate hz") as f64;

    let req = build_fast_sync_read(0, L as u16, &[id.as_byte()]).unwrap();
    let cap = bus
        .xfer(&req, silent_window_us(baud, 0, L as u32))
        .expect("pirate xfer");

    let frame = frame_bytes(&cap, req.len());
    let slots = parse_fast_response(&frame, &[L]).expect("solo frame parses with valid CRC");
    assert_eq!(slots.len(), 1);
    assert_eq!(slots[0].id, id);

    let timing = cap.timing.expect("reply timing captured");
    let first_delay_us = timing.reply_first.wrapping_sub(timing.req_end) as f64 / hz;
    eprintln!(
        "fast-injected[only] baud={baud}: chip Only starts {first_delay_us:.2} µs after \
         request end (expect ≈ chip RDT + 1 byte_time)",
    );

    assert_healthy(&mut bus, id);
}
