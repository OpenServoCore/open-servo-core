//! Clock-trim on silicon (protocol sec 9.3), both loops:
//!
//! - CAL: the lying-announce train injects a known clock-offset reading
//!   without touching the chip -- the announce declares a shorter gap
//!   than the pirate's crystal actually paces, every gap reads long by
//!   the same ratio, and the servo trims as if its own clock were fast.
//!   The truthful trains that follow must pull the total back -- the
//!   closed-loop plant-direction proof DES cannot give (the sim fakes
//!   the chip adapter; on silicon an inverted HSITRIM mapping
//!   railed the fleet in max-step clamps while every DES trim test
//!   stayed green).
//! - Tracker: the host-detune probe injects drift the same way -- the
//!   pirate moves one BRR step off nominal WITHOUT a CAL, so every
//!   chain pair reads the shift, and the differential tracker must trim
//!   it out from traffic alone (and trim back when the host returns).

use std::thread::sleep;
use std::time::Duration;

use bench::BOOT_BAUD;
use bench::osc::{build_cal, build_instruction, build_read};
use osc_protocol::wire::{Inst, Opcode};
use osc_servo_core::regions::control::addr::lifecycle::GOAL_POSITION;
use osc_servo_core::regions::telemetry::addr::common::TRIM_STEPS;
use serial_test::serial;

use super::support::{Bench, SETTLE_MS, bench};

/// Wire gap the pirate actually paces, and the gaps per train.
const GAP_US: u16 = 400;
const GAPS: u8 = 8;
/// Announced gap for the lying train: wire runs 400, so every gap reads
/// +8 us ~ +20.4k ppm of "clock fast" -- far past `STEPS_MAX` at any legal
/// step effect, and still inside the per-gap T/16 sanity gate.
const LIE_GAP_US: u16 = 392;
/// One window's clamp (drivers `trim::STEPS_MAX`).
const STEPS_MAX: i32 = 4;

fn read_trim(b: &mut Bench) -> i32 {
    let status = b.status_ok(&build_read(b.id(), TRIM_STEPS, 1));
    assert_eq!(status.payload.len(), 1, "trim_steps is one byte");
    status.payload[0] as i8 as i32
}

fn train(b: &mut Bench, announce_gap_us: u16) {
    b.cal_train(
        &build_cal(announce_gap_us, GAPS),
        GAP_US as u32,
        GAPS as u32 + 1,
    );
    // The decision applies in the servo main loop between frames.
    sleep(Duration::from_millis(SETTLE_MS));
}

#[serial]
#[test]
fn lying_train_trims_and_truth_pulls_back() {
    let mut b = bench();
    let start = read_trim(&mut b);
    assert!(
        start.abs() <= 8,
        "precondition: trim near center, got {start} (reboot the servo)"
    );

    train(&mut b, LIE_GAP_US);
    let lied = read_trim(&mut b);
    assert_eq!(lied - start, STEPS_MAX, "the lie clamps at +STEPS_MAX");

    train(&mut b, GAP_US);
    train(&mut b, GAP_US);
    let back = read_trim(&mut b);
    assert!(
        (back - start).abs() <= 1,
        "truth pulls the trim back: start {start}, back {back}"
    );
}

/// Host detune for the tracker probe: one BRR step off 1M on the pirate
/// (144 MHz / 145 ~ 993.1 kbaud, -6.9k ppm). Inside every gate that
/// matters -- pair qualification (0.69% of span vs the 1/16 gate), the
/// tracker's +/-8k ppm sanity band, and framing margin (+/-3.4%, F10) -- and
/// big against the per-window noise floor.
const DETUNE_BAUD: u32 = 993_103;
/// Frames per food burst. Each adjacent pair inside a burst brackets one
/// CRC-verified silent WRITE(NOREPLY) -- the tracker's food (protocol sec 9.3); the
/// pirate's grid pacing makes the seam stationary by construction.
const FOOD_FRAMES: usize = 24;
/// Bursts that carry one tracker decision with margin: baseline (32
/// pairs) + window (128) + a refinement round, at 23 pairs per burst --
/// sized generously (~2x the ideal-flow minimum) because ~30% of pairs
/// die to service-lag byte-exactness on silicon (probe-measured)
/// and a late apply must still land before the phase's trim read.
const PHASE_BURSTS: u32 = 36;
/// Bursts after a CAL so the re-anchored baseline captures the true
/// seam before the detune shifts it.
const BASELINE_BURSTS: u32 = 3;

fn feed(b: &mut Bench, burst: &[Vec<u8>], bursts: u32) {
    for _ in 0..bursts {
        b.burst_frames(burst);
        // Decisions apply in the servo main loop between frames.
        sleep(Duration::from_millis(SETTLE_MS));
        b.drain_stamps();
    }
}

#[serial]
#[test]
fn tracker_follows_host_detune() {
    let mut b = bench();
    // The detune step is defined against the 1M BRR; pin the bus there.
    b.switch_baud(BOOT_BAUD);

    let mut payload = GOAL_POSITION.to_le_bytes().to_vec();
    payload.extend_from_slice(&0i32.to_le_bytes());
    let frame = build_instruction(b.id(), Opcode::Write, Inst::FLAG_NOREPLY, &payload);
    let burst = vec![frame; FOOD_FRAMES];

    // Anchor: two truthful trains converge the CAL loop; the food after
    // them lets the tracker baseline capture the true host seam.
    train(&mut b, GAP_US);
    train(&mut b, GAP_US);
    let start = read_trim(&mut b);
    feed(&mut b, &burst, BASELINE_BURSTS);

    // Host walks away -6.9k ppm with no CAL: only the tracker can see it.
    b.follow_baud(DETUNE_BAUD);
    feed(&mut b, &burst, PHASE_BURSTS);
    b.follow_baud(BOOT_BAUD);
    let pulled = read_trim(&mut b);

    // Host returns to true baud -- a host-KNOWN behavior change, so the
    // contract's answer is a CAL re-anchor (protocol sec 9.3), not tracker food: the
    // tracker's post-apply seam baseline recaptures against whatever the
    // host is doing at that moment, so a rate STEP landing inside the
    // 32-pair capture is absorbed as seam, and later windows honestly
    // read ~0 (probe-verified on silicon -- pairs and verdicts
    // flow, the measurement is genuinely zero). Thermal drift can never
    // step like that; a host that changes rate re-anchors -- with two
    // trains, per the protocol sec 9.3 boot guidance (the first identifies the
    // chip's step effect, the second finishes).
    train(&mut b, GAP_US);
    train(&mut b, GAP_US);
    let back = read_trim(&mut b);

    // A slow host reads exactly like a fast servo clock: gaps measure
    // long, the correction slows the oscillator, trim_steps rises. >=1 in
    // the right DIRECTION proves the tracker ate the pairs and moved the
    // right way; magnitude is chip-dependent (step effects span 1.4-4k
    // ppm/step, and a strong-step chip's settled answer for -6.9k ppm is
    // legitimately small) -- the noiseless DES twins pin magnitude.
    assert!(
        pulled - start >= 1,
        "tracker follows a slow host: start {start}, detuned {pulled}"
    );
    assert!(
        (back - start).abs() <= 1,
        "the returning host's CAL re-anchors: start {start}, back {back}"
    );
}
