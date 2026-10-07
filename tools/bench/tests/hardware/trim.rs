//! Clock-trim on silicon (protocol sec 9.3), both loops:
//!
//! - CAL: the lying-announce train injects a known clock-offset reading
//!   without touching the chip -- the announce declares a shorter gap
//!   than the adapter's crystal actually paces, every gap reads long by
//!   the same ratio, and the servo trims as if its own clock were fast.
//!   The truthful trains that follow must pull the total back -- the
//!   closed-loop plant-direction proof DES cannot give (the sim fakes
//!   the chip adapter; on silicon an inverted HSITRIM mapping
//!   railed the fleet in max-step clamps while every DES trim test
//!   stayed green).
//! - Tracker: the host-detune probe injects drift the same way -- the
//!   host moves one BRR step off nominal WITHOUT a CAL, so every
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
/// suite's hot-loop food left it (the tracker walked it to the +15 rail
/// on the bench), so the walk back from the rail at STEPS_MAX per train,
/// one train to identify the step effect, one to confirm.
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

/// Converge the CAL loop and return the converged trim (protocol sec 9.3:
/// converged = the read-back stable between trains). Two trains from
/// boot; more when the trim enters off-center -- a fixed pair clamped to
/// STEPS_MAX leaves a railed trim 7 steps short, and `|start| <= 8` cannot
/// tell that from converged (bench: a lie read +2, truth pulled back 8).
/// The tests' subject is plant DIRECTION, so a LIE never goes first: it
/// would poison the apply->remeasure step-effect identification.
fn anchor(b: &mut Bench) -> (i32, u32) {
    train(b, GAP_US);
    let mut trim = read_trim(b);
    for trains in 2..=ANCHOR_TRAINS_MAX {
        train(b, GAP_US);
        let next = read_trim(b);
        if next == trim {
            return (trim, trains);
        }
        trim = next;
    }
    panic!("precondition: CAL did not converge in {ANCHOR_TRAINS_MAX} trains, trim {trim}")
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

/// Host detune for the tracker probe: one BRR step off 1M on the host UART
/// (144 MHz / 145 ~ 993.1 kbaud, -6.9k ppm). Inside every gate that
/// matters -- pair qualification (0.69% of span vs the 1/16 gate), the
/// tracker's +/-8k ppm sanity band, and framing margin (+/-3.4%, F10) -- and
/// big against the per-window noise floor.
const DETUNE_BAUD: u32 = 993_103;
/// Frames per food burst. Each adjacent pair inside a burst brackets one
/// CRC-verified silent WRITE(NOREPLY) -- the tracker's food (protocol sec 9.3); the
/// adapter's grid pacing makes the seam stationary by construction.
const FOOD_FRAMES: usize = 24;
/// Bursts that carry one tracker decision with margin: a window (128
/// pairs) + a refinement round, at 23 pairs per burst - sized generously
/// (~2x the ideal-flow minimum) because ~30% of pairs die to service-lag
/// byte-exactness on silicon (probe-measured) and a late apply must still
/// land before the phase's trim read.
const PHASE_BURSTS: u32 = 36;
/// Bursts after a CAL so the anchor's baseline window (128 pairs, at the
/// same ~70% pair yield) captures the true seam before the detune shifts
/// it.
const BASELINE_BURSTS: u32 = 10;

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
    payload.extend_from_slice(&b.goal_mid().to_le_bytes());
    let frame = build_instruction(b.id(), Opcode::Write, Inst::FLAG_NOREPLY, &payload);
    let burst = vec![frame; FOOD_FRAMES];

    // The food after the anchor lets the tracker baseline capture the true
    // host seam.
    let (start, trains) = anchor(&mut b);
    feed(&mut b, &burst, BASELINE_BURSTS);

    // Host walks away -6.9k ppm with no CAL: only the tracker can see it.
    b.follow_baud(DETUNE_BAUD);
    feed(&mut b, &burst, PHASE_BURSTS);
    b.follow_baud(BOOT_BAUD);
    let pulled = read_trim(&mut b);

    // Host returns to true baud - a host-KNOWN behavior change, so the
    // contract's answer is a CAL re-anchor (protocol sec 9.3): a rate step
    // is not thermal drift, and a host that changes rate re-anchors - with
    // two trains, per the sec 9.3 boot guidance (the first identifies the
    // chip's step effect, the second finishes). The tracker's baseline is
    // the CAL anchor, so against it the return reads as the detune undone
    // and the tracker pulls back on its own where the food allows; the
    // trains settle it either way.
    train(&mut b, GAP_US);
    train(&mut b, GAP_US);
    let back = read_trim(&mut b);

    // What this test put on the wire after the bench's own setup reads, for
    // the probe gate: breaks the servo should have stamped (one per frame)
    // and the bare ruler marks it must not.
    let trains = trains + 2;
    let frames = (FOOD_FRAMES as u32) * (BASELINE_BURSTS + PHASE_BURSTS) + 2 * trains + 2;
    eprintln!(
        "HOSTCOUNT frames={frames} (food {}, {trains} trains of announce+read, 2 reads) \
         bare_breaks={}",
        FOOD_FRAMES as u32 * (BASELINE_BURSTS + PHASE_BURSTS),
        trains * (GAPS as u32 + 1)
    );

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
