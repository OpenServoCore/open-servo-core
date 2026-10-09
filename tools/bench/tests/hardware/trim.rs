//! Clock-trim on silicon (protocol sec 9.3): the CAL ruler is the only
//! input to the trim.
//!
//! - Plant direction: the lying-announce train injects a known
//!   clock-offset reading without touching the chip - the announce
//!   declares a shorter gap than the adapter's crystal actually paces,
//!   every gap reads long by the same ratio, and the servo trims as if its
//!   own clock were fast. The truthful trains that follow must pull the
//!   total back - the closed-loop proof DES cannot give (the sim fakes
//!   the chip adapter; on silicon an inverted HSITRIM mapping railed the
//!   fleet in max-step clamps while every DES trim test stayed green).
//! - Host detune: the host moves one BRR step off nominal. Traffic at
//!   the detuned rate must leave the trim alone, and CAL trains sent at
//!   that rate must land on the same anchor: the ruler is the adapter's
//!   crystal, not its baud.

use std::thread::sleep;
use std::time::Duration;

use bench::BOOT_BAUD;
use bench::osc::{build_cal, build_instruction, build_read};
use osc_protocol::wire::{Inst, Opcode};
use osc_servo_core::regions::control::addr::lifecycle::GOAL_POSITION;
use osc_servo_core::regions::telemetry::addr::common::TRIM_STEPS;
use serial_test::serial;

use super::support::{Bench, SETTLE_MS, bench};

/// Wire gap the adapter actually paces, and the gaps per train.
const GAP_US: u16 = 400;
const GAPS: u8 = 8;
/// Announced gap for the lying train: wire runs 400, so every gap reads
/// +8 us ~ +20.4k ppm of "clock fast" -- far past `STEPS_MAX` at any legal
/// step effect, and still inside the per-gap T/16 sanity gate.
const LIE_GAP_US: u16 = 392;
/// One window's clamp (drivers `trim::STEPS_MAX`).
const STEPS_MAX: i32 = 4;
/// The accumulated-trim rail (drivers `trim::TOTAL_MAX`).
const TOTAL_MAX: i32 = 15;
/// Trains the anchor may need: the trim enters this file wherever the
/// last CAL left it (anywhere up to the rail), so the walk back from the
/// rail at STEPS_MAX per train, one train to identify the step effect, one
/// to confirm.
const ANCHOR_TRAINS_MAX: u32 = (TOTAL_MAX as u32).div_ceil(STEPS_MAX as u32) + 3;

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

/// Converge the CAL loop and return the converged trim and the trains it
/// took (protocol sec 9.3: converged = the read-back stable between
/// trains). Two trains from boot; more when the trim enters off-center - a
/// fixed pair clamped to STEPS_MAX leaves a railed trim 7 steps short, and
/// `|start| <= 8` cannot tell that from converged (bench: a lie read +2,
/// truth pulled back 8). The tests' subject is plant DIRECTION, so a LIE
/// never goes first: it would poison the apply->remeasure step-effect
/// identification.
fn anchor(b: &mut Bench) -> (i32, u32) {
    let (trim, trains) = converge(b, |b| train(b, GAP_US));
    trim.map(|t| (t, trains))
        .unwrap_or_else(|| panic!("CAL did not converge in {trains} trains"))
}

/// Trains from `send` until two read-backs agree: `(Some(trim), trains)`,
/// or `(None, ANCHOR_TRAINS_MAX)` when they never do.
fn converge(b: &mut Bench, mut send: impl FnMut(&mut Bench)) -> (Option<i32>, u32) {
    send(b);
    let mut trim = read_trim(b);
    for trains in 2..=ANCHOR_TRAINS_MAX {
        send(b);
        let next = read_trim(b);
        if next == trim {
            return (Some(trim), trains);
        }
        trim = next;
    }
    (None, ANCHOR_TRAINS_MAX)
}

#[serial]
#[test]
fn lying_train_trims_and_truth_pulls_back() {
    let mut b = bench();
    let (start, _) = anchor(&mut b);

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

/// Host detune: one BRR step off 1M on the host UART (144 MHz / 145 ~
/// 993.1 kbaud, -6.9k ppm), inside framing margin (+/-3.4%, F10) and
/// nearly three nominal trim steps.
const DETUNE_BAUD: u32 = 993_103;
/// Frames per food burst: silent WRITE(NOREPLY)s, the hot-loop shape.
const FOOD_FRAMES: usize = 24;
/// Food bursts at the detuned rate: 36 x 24 = 864 frames.
const FOOD_BURSTS: u32 = 36;

fn feed(b: &mut Bench, burst: &[Vec<u8>], bursts: u32) {
    for _ in 0..bursts {
        b.burst_frames(burst);
        sleep(Duration::from_millis(SETTLE_MS));
        b.drain_stamps();
    }
}

#[serial]
#[test]
fn cal_holds_its_anchor_through_a_host_detune() {
    let mut b = bench();
    // The detune step is defined against the 1M BRR; pin the bus there.
    b.switch_baud(BOOT_BAUD);

    let mut payload = GOAL_POSITION.to_le_bytes().to_vec();
    payload.extend_from_slice(&b.goal_mid().to_le_bytes());
    let frame = build_instruction(b.id(), Opcode::Write, Inst::FLAG_NOREPLY, &payload);
    let burst = vec![frame; FOOD_FRAMES];

    let (start, _) = anchor(&mut b);

    // Host walks away -6.9k ppm. Read-backs run at the true rate: the
    // capture decodes replies at the host's set rate.
    b.follow_baud(DETUNE_BAUD);
    feed(&mut b, &burst, FOOD_BURSTS);
    b.follow_baud(BOOT_BAUD);
    let fed = read_trim(&mut b);

    // CAL trains sent at the detuned rate, each read back at the true one.
    let (detuned, detuned_trains) = converge(&mut b, |b| {
        b.follow_baud(DETUNE_BAUD);
        train(b, GAP_US);
        b.follow_baud(BOOT_BAUD);
    });

    assert_eq!(
        fed, start,
        "traffic at a detuned host never trims: start {start}, after food {fed}"
    );
    let detuned = detuned.unwrap_or_else(|| {
        panic!("CAL at the detuned rate did not converge in {detuned_trains} trains")
    });
    assert!(
        (detuned - start).abs() <= 1,
        "CAL at a detuned host reads the crystal: start {start}, detuned {detuned} \
         after {detuned_trains} trains"
    );
}
