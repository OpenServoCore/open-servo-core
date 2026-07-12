//! MGMT CAL break-pair ruler (`docs/osc-native-protocol.md` §9.3): the host
//! announces a break train, its crystal spaces the breaks, and each servo
//! measures the announced gap with its own clock — break-FE entry stamps at
//! both ends of every gap, so entry latency cancels. Plain assertions on the
//! trim decisions and on the transport's health after trains.

use osc_core::BaudRate;
use osc_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_integration::sim::{Sim, Source, instruction, status};
use osc_protocol::wire::{Inst, Opcode, ResultCode};

mod support;

const ID: u8 = 5;
const BROADCAST: u8 = 0xFE;
const GAP_US: u64 = 400;
const GAPS: u8 = 8;
/// First ruler mark, µs after the announce frame's start — clear of the
/// frame itself at every operational baud.
const TRAIN_LEAD_US: u64 = 200;

fn cal_announce(gap_us: u16, gaps: u8) -> Vec<u8> {
    let [lo, hi] = gap_us.to_le_bytes();
    instruction(BROADCAST, Opcode::Mgmt, 0, &[0x06, lo, hi, gaps])
}

/// Announce at `t0`, then `marks` breaks on exact `GAP_US` spacing.
fn send_train(sim: &mut Sim, t0: u64, marks: u64) {
    sim.host_send_at(t0, &cal_announce(GAP_US as u16, GAPS));
    for k in 0..marks {
        sim.host_send_break_at(t0 + TRAIN_LEAD_US + k * GAP_US);
    }
}

fn train_end(t0: u64) -> u64 {
    t0 + TRAIN_LEAD_US + GAPS as u64 * GAP_US
}

/// The trio's real signature (+5.2k ppm, bench 2026-07-11): one train, one
/// decision, the nominal-seed acquire jump. Positive = slower.
#[test_log::test]
fn cal_train_draws_the_acquire_jump() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

#[test_log::test]
fn slow_clock_draws_the_symmetric_speedup() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, -5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(-2));
}

/// The ruler is µs-denominated, so the measurement is baud-independent —
/// the same train at 3M reads the same skew.
#[test_log::test]
fn cal_at_3m_reads_the_same_skew() {
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

/// Inside half a nominal step the rounding IS the deadband: a well-trimmed
/// chip is never poked.
#[test_log::test]
fn near_nominal_clock_holds_still() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 900, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// A stray FE mid-gap (line noise) splits one gap into two sub-gate halves:
/// both rejected, two of the announced gaps spent, the survivors still carry
/// the decision — a noisy train costs precision, never correctness.
#[test_log::test]
fn spurious_fe_mid_train_costs_gaps_not_the_train() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.inject_garble_at(TRAIN_LEAD_US + 3 * GAP_US + GAP_US / 2, 0xAA);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

/// Every gap outside the ±6% gate (a host that can't keep the announced
/// spacing): fewer than half the gaps validate, and the train decides
/// NOTHING — a mangled ruler yields no reading rather than a wrong one.
#[test_log::test]
fn mangled_train_decides_nothing() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    sim.host_send_at(0, &cal_announce(GAP_US as u16, GAPS));
    // Marks at alternating 150/650 µs — every gap far outside the gate.
    let mut t = TRAIN_LEAD_US;
    for k in 0..(GAPS as u64 + 1) {
        sim.host_send_break_at(t);
        t += if k % 2 == 0 { 150 } else { 650 };
    }
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// The train's break bytes are scan noise the framer's hunt clears silently:
/// the first instruction after a train answers clean, and neither counter
/// moved — CAL is invisible to the link diagnostics.
#[test_log::test]
fn train_then_ping_answers_clean() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.host_send_at(train_end(0) + 500, &instruction(ID, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let replies: Vec<_> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "the ping's ack: {frames:#?}");
    let (inst, _) = status(replies[0]);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let d = sim.servo_diag(s);
    assert_eq!(d.framing_drop_count, 0);
    assert_eq!(d.crc_fail_count, 0);
}

/// A train that dies mid-way trips the silence watchdog: no decision, and
/// the transport is back to answering within two announced gaps.
#[test_log::test]
fn dead_train_frees_the_transport() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID);
    sim.host_send_at(0, &cal_announce(GAP_US as u16, GAPS));
    sim.host_send_break_at(TRAIN_LEAD_US);
    sim.host_send_break_at(TRAIN_LEAD_US + GAP_US);
    sim.host_send_at(5_000, &instruction(ID, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let replies: Vec<_> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "the ping's ack: {frames:#?}");
    let (inst, _) = status(replies[0]);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// CAL is broadcast-only (§9.3): a unicast CAL's ack would put our own break
/// on the wire exactly where the train starts, so it decodes Unsupported and
/// is answered as an instruction error.
#[test_log::test]
fn unicast_cal_is_refused() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID);
    let [lo, hi] = (GAP_US as u16).to_le_bytes();
    sim.host_send_at(0, &instruction(ID, Opcode::Mgmt, 0, &[0x06, lo, hi, GAPS]));
    let frames = sim.run();
    let replies: Vec<_> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "the refusal: {frames:#?}");
    let (inst, _) = status(replies[0]);
    assert_eq!(inst.result(), Some(ResultCode::Instruction));
    assert_eq!(sim.poll_clock_trim(s), None);
}

// ---- differential drift tracker (§9.3) ----------------------------------

const OTHER_ID: u8 = 6;
/// Silent hot-loop stand-in: WRITE|NOREPLY, 42-byte payload → 48-byte
/// footprint, 480 µs of wire at 1M; host seam = 20 µs.
const PERIOD_US: u64 = 500;

fn silent_write() -> Vec<u8> {
    instruction(OTHER_ID, Opcode::Write, Inst::FLAG_NOREPLY, &[0u8; 42])
}

/// `n` silent frames from `t0`, PERIOD_US apart.
fn send_silent(sim: &mut Sim, t0: u64, n: u64) -> u64 {
    let f = silent_write();
    for k in 0..n {
        sim.host_send_at(t0 + k * PERIOD_US, &f);
    }
    t0 + n * PERIOD_US
}

/// The tracker follows drift injected mid-run: the baseline absorbs the
/// host's queuing seam AND the boot-time skew, and a later rate change is
/// read as its shift — one step of drift draws one step of correction,
/// with no CAL in sight.
#[test_log::test]
fn tracker_follows_thermal_drift() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    // Baseline (32 pairs) + one full window (128) at the boot rate.
    let t = send_silent(&mut sim, 0, 180);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None, "no drift, no decision");
    // Thermal drift: +2600 ppm, continuously (the clock never steps).
    sim.set_servo_skew_at(t, s, 2_600);
    send_silent(&mut sim, t + PERIOD_US, 140);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(1));
}

/// A constant seam and a constant skew are BOTH invisible: the tracker
/// measures changes, not states — absolute correction is CAL's job.
#[test_log::test]
fn constant_seam_and_skew_cancel() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_silent(&mut sim, 0, 320);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// Solicited frames never pair — an ack's turnaround rides another clock,
/// and at 1M a reply gap slips under the span gate. The silent-shape rule
/// keeps them out entirely: acked traffic yields no pairs, no windows.
#[test_log::test]
fn solicited_frames_never_pair() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let f = instruction(OTHER_ID, Opcode::Write, 0, &[0u8; 42]); // acked shape
    for k in 0..320 {
        sim.host_send_at(k * PERIOD_US, &f);
    }
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// The full composition: CAL anchors absolute, the tracker holds through
/// quiet, then follows a later drift on top — each layer consuming exactly
/// its own signal.
#[test_log::test]
fn cal_anchors_then_tracker_follows() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(
        sim.poll_clock_trim(s),
        Some(2),
        "CAL takes the acquire jump"
    );
    // Quiet stretch (clear of the post-train hunt's trailing wakes):
    // baseline recaptures post-CAL, windows read no drift.
    let t = send_silent(&mut sim, train_end(0) + 5_000, 180);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None, "no drift since the anchor");
    // Motor heat: +2600 ppm on top of the boot skew.
    sim.set_servo_skew_at(t, s, 5_200 + 2_600);
    send_silent(&mut sim, t + PERIOD_US, 140);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(3));
}

// ---- bench-shape regression (silicon 2026-07-12) -------------------------
//
// The hardware tracker probe feeds 24-frame WRITE|NOREPLY bursts with
// ~4-bit seams and a −6.9k ppm host detune, and the silicon tracker reads
// ZERO — while every DES tracker test above (500 µs-period FOREIGN traffic)
// stays green. These twins replicate the bench shape exactly; the fork
// between them and against silicon localizes the starvation.

/// Bench burst geometry at 1M: 10-byte frame + break = 110 µs wire,
/// 4-bit pirate seam, 24 frames per burst, settle gap between bursts.
const BURST_FRAMES: u64 = 24;
const BURST_PERIOD_US: u64 = 114;
const BURST_SETTLE_US: u64 = 5_000;

fn goal_write(id: u8) -> Vec<u8> {
    use osc_core::regions::control::addr::lifecycle::GOAL_POSITION;
    let mut payload = GOAL_POSITION.to_le_bytes().to_vec();
    payload.extend_from_slice(&0i32.to_le_bytes());
    instruction(id, Opcode::Write, Inst::FLAG_NOREPLY, &payload)
}

fn send_bursts(sim: &mut Sim, frame: &[u8], mut t: u64, bursts: u64) -> u64 {
    for _ in 0..bursts {
        for k in 0..BURST_FRAMES {
            sim.host_send_at(t + k * BURST_PERIOD_US, frame);
        }
        t += BURST_FRAMES * BURST_PERIOD_US + BURST_SETTLE_US;
    }
    t
}

fn last_trim(sim: &mut Sim, s: usize) -> Option<i8> {
    let mut last = None;
    while let Some(v) = sim.poll_clock_trim(s) {
        last = Some(v);
    }
    last
}

#[test_log::test]
fn tracker_follows_bench_bursts_foreign() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let f = goal_write(OTHER_ID);
    let t = send_bursts(&mut sim, &f, 0, 3);
    sim.run();
    assert_eq!(last_trim(&mut sim, s), None, "baseline absorbs the seam");
    sim.set_servo_skew_at(t, s, 6_900);
    send_bursts(&mut sim, &f, t + 100, 16);
    sim.run();
    let moved = last_trim(&mut sim, s);
    assert!(
        matches!(moved, Some(n) if n >= 2),
        "foreign bursts feed the tracker: {moved:?}"
    );
}

#[test_log::test]
fn tracker_follows_bench_bursts_self_addressed() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let f = goal_write(ID);
    let t = send_bursts(&mut sim, &f, 0, 3);
    sim.run();
    assert_eq!(last_trim(&mut sim, s), None, "baseline absorbs the seam");
    sim.set_servo_skew_at(t, s, 6_900);
    send_bursts(&mut sim, &f, t + 100, 16);
    sim.run();
    let moved = last_trim(&mut sim, s);
    assert!(
        matches!(moved, Some(n) if n >= 2),
        "self-addressed bursts feed the tracker: {moved:?}"
    );
}

/// The silicon reality behind the bench-shape starvation (2026-07-12):
/// NOREPLY frames leave the wire-fault flag latched (nothing transmits to
/// retire it), and its level-pend re-fire lands in the seam before the
/// next frame's bytes (§6 A4). A re-fire is NOT a break — stamping the
/// drift tracker from it clobbers the pair in flight and starves the
/// tracker to zero, while clean-break sims stay green. The fix gates the
/// drift stamp on ring freshness (fault contract: fault handling is
/// idempotent), the same cursor idiom the CAL run already uses.
#[test_log::test]
fn tracker_survives_latched_refires_between_frames() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let f = goal_write(ID);
    let send = |sim: &mut Sim, mut t: u64, bursts: u64| -> u64 {
        for _ in 0..bursts {
            for k in 0..BURST_FRAMES {
                sim.host_send_at(t + k * BURST_PERIOD_US, &f);
                // A spurious wake per seam (the FE-era latched re-fire
                // shape), just before the next frame's bytes.
                sim.inject_wake_refire_at(t + k * BURST_PERIOD_US + 112, s);
            }
            t += BURST_FRAMES * BURST_PERIOD_US + BURST_SETTLE_US;
        }
        t
    };
    let t = send(&mut sim, 0, 3);
    sim.run();
    assert_eq!(last_trim(&mut sim, s), None, "baseline absorbs the seam");
    sim.set_servo_skew_at(t, s, 6_900);
    send(&mut sim, t + 100, 16);
    sim.run();
    let moved = last_trim(&mut sim, s);
    assert!(
        matches!(moved, Some(n) if n >= 2),
        "re-fires must not eat the tracker's pairs: {moved:?}"
    );
}
