//! MGMT CAL break-pair ruler (`docs/osc-native-protocol.md` sec 9.3): the host
//! announces a break train, its crystal spaces the breaks, and each servo
//! measures the announced gap with its own clock -- break-FE entry stamps at
//! both ends of every gap, so entry latency cancels. Plain assertions on the
//! trim decisions and on the transport's health after trains.

use osc_integration::sim::{BreakWake, HandlerCost, Sim, Source, instruction, status};
use osc_protocol::wire::{Inst, Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use rstest::rstest;

mod support;

const ID: u8 = 5;
const BROADCAST: u8 = 0xFE;
const GAP_US: u64 = 400;
const GAPS: u8 = 8;
/// First ruler mark, us after the announce frame's start -- clear of the
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

/// The trio's real signature (+5.2k ppm, bench-measured): one train, one
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

/// The ruler is us-denominated, so the measurement is baud-independent --
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
/// the decision -- a noisy train costs precision, never correctness.
#[test_log::test]
fn spurious_fe_mid_train_costs_gaps_not_the_train() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.inject_garble_at(TRAIN_LEAD_US + 3 * GAP_US + GAP_US / 2, 0xAA);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

/// Every gap outside the +/-6% gate (a host that can't keep the announced
/// spacing): fewer than half the gaps validate, and the train decides
/// NOTHING -- a mangled ruler yields no reading rather than a wrong one.
#[test_log::test]
fn mangled_train_decides_nothing() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    sim.host_send_at(0, &cal_announce(GAP_US as u16, GAPS));
    // Marks at alternating 150/650 us -- every gap far outside the gate.
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
/// moved -- CAL is invisible to the link diagnostics.
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

/// CAL is broadcast-only (sec 9.3): a unicast CAL's ack would put our own break
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

// ---- differential drift tracker (sec 9.3) ----------------------------------

const OTHER_ID: u8 = 6;
/// Silent hot-loop stand-in: WRITE|NOREPLY, 42-byte payload -> 48-byte
/// footprint, 480 us of wire at 1M; host seam = 20 us.
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
/// read as its shift -- one step of drift draws one step of correction,
/// with no CAL in sight.
#[test_log::test]
fn tracker_follows_thermal_drift() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    // Baseline window + one full window (128 pairs each) at the boot rate.
    let t = send_silent(&mut sim, 0, 257);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None, "no drift, no decision");
    // Thermal drift: +2600 ppm, continuously (the clock never steps).
    sim.set_servo_skew_at(t, s, 2_600);
    send_silent(&mut sim, t + PERIOD_US, 140);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(1));
}

/// Every frame that ended before a break is recorded ahead of that
/// break's stamp, even one that scheduled no milestone of its own (a
/// foreign frame resolves at the next wake): alternating footprints make a
/// frame recorded one pair late fail the byte-exactness gate, so the
/// tracker would never decide.
#[rstest]
#[test_log::test]
fn tracker_pairs_each_frame_with_its_own_breaks(
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte, BreakWake::Alternating)] wake: BreakWake,
) {
    let long = silent_write();
    let short = instruction(OTHER_ID, Opcode::Write, Inst::FLAG_NOREPLY, &[0u8; 20]);
    // Wire time at 1M: footprint x 10 us, plus a 10 us host seam - inside
    // the SHORT frame's pair gate too (16 us, less the break's 4-bit
    // excess), so both shapes pair.
    let send = |sim: &mut Sim, mut t: u64, n: u64| -> u64 {
        for k in 0..n {
            let f = if k % 2 == 0 { &long } else { &short };
            sim.host_send_at(t, f);
            t += f.len() as u64 * 10 + 10;
        }
        t
    };
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.set_break_wake(wake);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let t = send(&mut sim, 0, 257);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None, "no drift, no decision");
    sim.set_servo_skew_at(t, s, 2_600);
    send(&mut sim, t + PERIOD_US, 140);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(1));
}

/// A constant seam and a constant skew are BOTH invisible: the tracker
/// measures changes, not states -- absolute correction is CAL's job.
#[test_log::test]
fn constant_seam_and_skew_cancel() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_silent(&mut sim, 0, 320);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// Solicited frames never pair -- an ack's turnaround rides another clock,
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
/// quiet, then follows a later drift on top -- each layer consuming exactly
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
    let t = send_silent(&mut sim, train_end(0) + 5_000, 257);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None, "no drift since the anchor");
    // Motor heat: +2600 ppm on top of the boot skew.
    sim.set_servo_skew_at(t, s, 5_200 + 2_600);
    send_silent(&mut sim, t + PERIOD_US, 140);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(3));
}

// ---- bench-shape regression ----------------------------------------------
//
// The hardware tracker probe feeds 24-frame WRITE|NOREPLY bursts with
// ~4-bit seams and a -6.9k ppm host detune, and the silicon tracker reads
// ZERO -- while every DES tracker test above (500 us-period FOREIGN traffic)
// stays green. These twins replicate the bench shape exactly; the fork
// between them and against silicon localizes the starvation.

/// Bench burst geometry at 1M: 10-byte frame + break = 110 us wire,
/// 4-bit host seam, 24 frames per burst, settle gap between bursts.
const BURST_FRAMES: u64 = 24;
const BURST_PERIOD_US: u64 = 114;
const BURST_SETTLE_US: u64 = 5_000;
/// Bursts that complete the 128-pair baseline window at 23 pairs each.
const BASELINE_BURSTS: u64 = 6;

fn goal_write(id: u8) -> Vec<u8> {
    use osc_servo_core::regions::control::addr::lifecycle::GOAL_POSITION;
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
    let t = send_bursts(&mut sim, &f, 0, BASELINE_BURSTS);
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
    let t = send_bursts(&mut sim, &f, 0, BASELINE_BURSTS);
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

/// A low that trips the break detector in an idle gap (wrong-baud traffic
/// on the bench) latches a stamp no frame owns, and the host's baud write
/// behind it restarts the tracker with no floor to skip it by. The rate
/// change drops every untaken stamp, so each frame at the new rate takes
/// its own break's stamp; kept, the orphan put every later frame one stamp
/// behind until the next CAL clear. Equal footprints keep the shifted
/// pairs inside the gate, so only the stamp ledger shows the shift.
#[test_log::test]
fn orphan_stamp_before_a_rate_change_never_shifts_pairs() {
    use osc_servo_core::regions::config::addr::common::BAUD_RATE_IDX;
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let t = send_silent(&mut sim, 0, 8);
    sim.inject_stamp_only_at(t + 4_500, s);
    let [lo, hi] = BAUD_RATE_IDX.to_le_bytes();
    let to_3m = instruction(ID, Opcode::Write, 0, &[lo, hi, BaudRate::B3000000.as_idx()]);
    sim.host_send_at(t + 5_000, &to_3m);
    sim.run();
    sim.set_host_baud(BaudRate::B3000000);
    let t = sim.now_us() + 100;
    send_silent(&mut sim, t, 257);
    sim.run();
    let (latched, taken) = sim.stamps(s);
    assert_eq!(latched, 8 + 1 + 1 + 257);
    assert_eq!(
        latched - taken - sim.stamp_drops(s),
        0,
        "no stamp rides ahead of its frame"
    );
}

/// The bench shape under handler costs just inside the 1M frame period:
/// the frame-end body spans the next break's detector fire, so every wake
/// is served a byte-time or more late with data bytes newest, and the
/// entry lag beats against the frame cadence (fixed costs: a three-frame
/// cycle) by several us per frame against the 7.5 us pair gate. A stamp
/// taken at entry cleared the gate on 13 or 14 of a burst's 23 pairs and
/// needed ten bursts to fill the baseline window. The hardware stamp is
/// the detector's instant, so the lag never enters a pair: the baseline
/// fills in `BASELINE_BURSTS` like lag-free food, and as many bursts after
/// the detune decide on it.
#[rstest]
#[test_log::test]
fn drift_stamps_ignore_isr_entry_lag(
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte)] wake: BreakWake,
) {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.set_break_wake(wake);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    sim.set_handler_cost(
        s,
        HandlerCost {
            on_break_us: 18,
            on_deadline_us: 30,
            on_tx_complete_us: 5,
            per_frame_us: 0,
        },
    );
    let f = goal_write(ID);
    let t = send_bursts(&mut sim, &f, 0, BASELINE_BURSTS);
    sim.run();
    assert_eq!(last_trim(&mut sim, s), None, "baseline absorbs the seam");
    sim.set_servo_skew_at(t, s, 6_900);
    send_bursts(&mut sim, &f, t + 100, BASELINE_BURSTS);
    sim.run();
    let moved = last_trim(&mut sim, s);
    assert!(
        matches!(moved, Some(n) if n >= 2),
        "every lagged pair clears the gate: {moved:?}"
    );
}

/// Breaks whose wakes merged into one entry each still stamp: break bodies
/// longer than two frame periods pend the next two breaks into one
/// delivery (the bench's bodies run to 226 us against a 123 us frame), the
/// ladder passes each break byte in order and takes its stamp, and every
/// pair brackets exactly its frame. One stamp per entry paired two frames
/// to one stamp and lost both pairs.
#[test_log::test]
fn coalesced_breaks_each_stamp() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    sim.set_handler_cost(
        s,
        HandlerCost {
            on_break_us: 280,
            on_deadline_us: 30,
            on_tx_complete_us: 5,
            per_frame_us: 0,
        },
    );
    let f = goal_write(OTHER_ID);
    let t = send_bursts(&mut sim, &f, 0, BASELINE_BURSTS);
    sim.run();
    assert_eq!(last_trim(&mut sim, s), None, "baseline absorbs the seam");
    let breaks = BASELINE_BURSTS * BURST_FRAMES;
    let entries = sim.delivered_breaks(s);
    assert!(
        entries <= breaks * 2 / 3,
        "the load must merge wakes: {entries} entries for {breaks} breaks"
    );
    assert_eq!(
        sim.stamps(s),
        (breaks, breaks),
        "one stamp per break, all taken"
    );
    sim.set_servo_skew_at(t, s, 6_900);
    send_bursts(&mut sim, &f, t + 100, BASELINE_BURSTS);
    sim.run();
    let moved = last_trim(&mut sim, s);
    assert!(
        matches!(moved, Some(n) if n >= 2),
        "merged breaks still pair: {moved:?}"
    );
}

/// The ladder behind the wire by more than one resolver drive's bound
/// (the flood shape): at 130 us of service per frame against 114 us of
/// wire (transport sec 5.9: 80-160 us per host frame), a 257-frame burst
/// queues some 30 whole frames by its end, and the bodies that serve it
/// resolve 16 at a time. Every frame's break still takes its stamp, at
/// the frame's verdict, in order, and the pairs behind the bound decide
/// like any others. A stamp taken at the break service with a clear of
/// the rest at the service's end lost the stamp of every frame queued
/// past the bound (bench: cleared == unstamped == span_many).
#[test_log::test]
fn saturated_backlog_stamps_every_break() {
    /// Two drift windows of pairs per burst: the first burst's baseline and
    /// a zero-drift window, the second's a window of detune.
    const FRAMES: u64 = 257;
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    sim.set_handler_cost(
        s,
        HandlerCost {
            on_break_us: 18,
            on_deadline_us: 30,
            on_tx_complete_us: 5,
            per_frame_us: 130,
        },
    );
    let f = goal_write(ID);
    let burst = |sim: &mut Sim, t: u64| {
        for k in 0..FRAMES {
            sim.host_send_at(t + k * BURST_PERIOD_US, &f);
        }
    };
    burst(&mut sim, 0);
    sim.run();
    assert_eq!(last_trim(&mut sim, s), None, "baseline absorbs the seam");
    let deepest = sim.frames_per_body_max(s);
    assert!(
        deepest >= 17,
        "the load must run the ladder past one drive's bound: {deepest} frames in one body"
    );
    assert_eq!(
        sim.stamps(s),
        (FRAMES, FRAMES),
        "one stamp per break, all taken"
    );
    assert_eq!(sim.stamp_drops(s), 0, "no stamp cleared");
    assert_eq!(sim.servo_diag(s).crc_fail_count, 0);
    assert_eq!(sim.servo_diag(s).framing_drop_count, 0);
    let t = sim.now_us() + BURST_SETTLE_US;
    sim.set_servo_skew_at(t, s, 6_900);
    burst(&mut sim, t);
    sim.run();
    let moved = last_trim(&mut sim, s);
    assert!(
        matches!(moved, Some(n) if n >= 2),
        "queued breaks still pair: {moved:?}"
    );
}

/// The servo's own break is sent with the detector's stamp request muted
/// along with its interrupt, so a reply latches nothing and the stamps
/// stay one per host break, in step with the ring (own bytes never ring,
/// F9). A silent write followed back-to-back by a ping to this servo: the
/// write's pair clears, the ping's is solicited, and the replies between
/// them leave the pairing exact.
#[test_log::test]
fn own_breaks_never_stamped() {
    /// Write, ping, reply, then quiet: well clear of the reply.
    const EXCHANGE_US: u64 = 600;
    let write = goal_write(OTHER_ID);
    let ping = instruction(ID, Opcode::Ping, 0, &[]);
    // One run per exchange: the scripted host does not wait for replies.
    let send = |sim: &mut Sim, t0: u64, n: u64| -> (u64, usize) {
        let mut replies = 0;
        for k in 0..n {
            let t = t0 + k * EXCHANGE_US;
            sim.host_send_at(t, &write);
            sim.host_send_at(t, &ping);
            replies += sim
                .run()
                .iter()
                .filter(|f| matches!(f.from, Source::Servo(_)))
                .count();
        }
        (t0 + n * EXCHANGE_US, replies)
    };
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let (t, replies) = send(&mut sim, 0, 257);
    assert_eq!(last_trim(&mut sim, s), None, "baseline absorbs the seam");
    assert_eq!(replies, 257, "every ping answered");
    assert_eq!(sim.stamps(s), (2 * 257, 2 * 257), "host breaks only");
    sim.set_servo_skew_at(t, s, 6_900);
    send(&mut sim, t, 140);
    let moved = last_trim(&mut sim, s);
    assert!(
        matches!(moved, Some(n) if n >= 2),
        "the write pairs decide between replies: {moved:?}"
    );
}

/// The silicon reality behind the bench-shape starvation:
/// NOREPLY frames leave the wire-fault flag latched (nothing transmits to
/// retire it), and its level-pend re-fire lands in the seam before the
/// next frame's bytes (transport sec 7). A re-fire is NOT a break -- stamping
/// the drift tracker from it clobbers the pair in flight and starves the
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
    let t = send(&mut sim, 0, BASELINE_BURSTS);
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

// ---- CAL-anchored baseline: the restoring force ---------------------------
//
// The tracker's baseline is captured once per CAL and stands across every
// drift decision, so a window reads the clock's residual against the CAL
// anchor - the steps already applied included - never an increment
// against the tracker's own last decision. These tests close the loop the
// way the chip does (main-loop poll, HSITRIM apply) and run it long enough
// for noise to integrate if it could.

/// The sim's Deadline provider's nominal step effect (`SimDeadline`).
const STEP_PPM: i32 = 2_500;
/// The sim's SBK break, bit-times: a pair's wire time exceeds its byte
/// footprint by the 4 bits the break runs past one byte.
const BREAK_BITS: u64 = 14;

/// The chip adapter's side of the closed loop, which the sim does not own:
/// the main loop polls between frames and HSITRIM moves the oscillator by
/// the nominal effect per applied step (positive = slower), on top of the
/// thermal skew the scenario sets.
struct Oscillator {
    thermal_ppm: i32,
    total: i32,
}

impl Oscillator {
    fn new(sim: &mut Sim, id: u8, thermal_ppm: i32) -> (Self, usize) {
        let s = sim.add_servo_with(id, thermal_ppm, DEFAULT_RESPONSE_DEADLINE_US);
        (
            Self {
                thermal_ppm,
                total: 0,
            },
            s,
        )
    }

    fn apply(&self, sim: &mut Sim, s: usize) {
        let now = sim.now_us();
        sim.set_servo_skew_at(now, s, self.thermal_ppm - self.total * STEP_PPM);
    }

    fn poll(&mut self, sim: &mut Sim, s: usize) -> Option<i8> {
        let out = sim.poll_clock_trim(s);
        if let Some(total) = out {
            self.total = total as i32;
            self.apply(sim, s);
        }
        out
    }

    fn drift_to(&mut self, sim: &mut Sim, s: usize, thermal_ppm: i32) {
        self.thermal_ppm = thermal_ppm;
        self.apply(sim, s);
    }
}

/// Two truthful trains converge the CAL anchor (sec 9.3 boot guidance);
/// returns the quiet instant after the second train's hunt.
fn anchor(sim: &mut Sim, osc: &mut Oscillator, s: usize) -> u64 {
    send_train(sim, 0, GAPS as u64 + 1);
    sim.run();
    osc.poll(sim, s);
    let t1 = train_end(0) + 5_000;
    send_train(sim, t1, GAPS as u64 + 1);
    sim.run();
    osc.poll(sim, s);
    train_end(t1) + 5_000
}

/// xorshift32: the flood's seam noise replays bit-exactly run to run.
fn xorshift(x: &mut u32) -> u32 {
    *x ^= *x << 13;
    *x ^= *x >> 17;
    *x ^= *x << 5;
    *x
}

/// 130k silent frames - the hardware suite's hot-loop and plain-flood
/// food count - with a host seam that coin-flips per frame between
/// back-to-back and 3/8 of the pair gate: ~1k ppm of window noise (0.4
/// step sd, 1.2 at three sigma), the bench's 1M/2M scatter. Re-baselining
/// after every apply integrated this noise into a random walk (bench: -1
/// -> +13 on plain flood; this test on that design: 11 steps off, parked
/// 10 off for 941 of the 1015 windows); against the CAL anchor each noisy
/// step is read back out by the next window and the trim stays within two
/// steps (one, but for the frozen baseline's own draw) - while the noise
/// is real enough to flip hundreds of decisions.
#[test_log::test]
fn noisy_silent_flood_holds_the_cal_anchor() {
    const FRAMES: u64 = 130_000;
    const WINDOW: u64 = 128;
    let mut sim = Sim::new(BaudRate::B1000000);
    let (mut osc, s) = Oscillator::new(&mut sim, ID, 5_200);
    let mut t = anchor(&mut sim, &mut osc, s);
    let anchor = osc.total;
    assert_eq!(anchor, 2, "the trio's acquire jump");

    // 26-byte footprint: 260 us of span at 1M, a 16 us pair gate.
    let f = instruction(OTHER_ID, Opcode::Write, Inst::FLAG_NOREPLY, &[0u8; 20]);
    let span_us = f.len() as u64 * 10;
    let wire_us = span_us - 10 + BREAK_BITS;
    let seam_hi_us = span_us / 16 * 3 / 8;
    // One frame opens pair continuity, so every chunk below lands a whole
    // window and its decision applies before the next window starts - the
    // chip's prompt main-loop poll, not a late apply that would re-read a
    // stale residual. A window closes at the verdict of the frame after its
    // last pair (the stamp is taken there), so the poll follows that frame.
    // The next chunk is queued before the current one runs, so the main
    // loop's turn never disturbs the host's cadence.
    sim.host_send_at(t, &f);
    t += wire_us;
    let mut rng = 0x9E37_79B9u32;
    let mut queue = |sim: &mut Sim, t: &mut u64| {
        for _ in 0..WINDOW {
            sim.host_send_at(*t, &f);
            *t += wire_us + (xorshift(&mut rng) & 1) as u64 * seam_hi_us;
        }
    };
    queue(&mut sim, &mut t);
    let (mut moves, mut worst) = (0u32, 0i32);
    for _ in 0..FRAMES / WINDOW {
        let boundary = t;
        queue(&mut sim, &mut t);
        sim.run_until(boundary + wire_us + BREAK_BITS);
        // The main loop's poll between frames: one window, one decision.
        if osc.poll(&mut sim, s).is_some() {
            moves += 1;
        }
        worst = worst.max((osc.total - anchor).abs());
    }
    assert!(worst <= 2, "the trim walked {worst} steps off the anchor");
    assert!(
        moves >= 100,
        "the noise must be real: only {moves} decisions moved the trim"
    );
}

/// A genuine shift still pulls the trim, and the anchored baseline makes
/// it SETTLE: a -6.9k ppm host detune (one BRR step at 1M, the bench
/// probe's shape) reads as +6.9k of servo-fast; the window takes the steps
/// (3 at the nominal effect), the next window reads the residual against
/// the anchor (-600 ppm, inside the rounding deadband), and the trim holds
/// there window after window.
#[test_log::test]
fn host_detune_pulls_the_trim_and_settles() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let (mut osc, s) = Oscillator::new(&mut sim, ID, 0);
    let t = send_silent(&mut sim, 0, 257);
    sim.run();
    assert_eq!(osc.poll(&mut sim, s), None, "no drift, no decision");
    osc.drift_to(&mut sim, s, 6_900);
    // One frame re-opens pair continuity across the pause (its own pair
    // gates out), so every chunk below lands a whole window, closed at the
    // verdict of the next chunk's first frame; the next chunk is queued
    // before the current one runs, so the main loop's turn at the frame
    // boundary never disturbs the host's cadence.
    let t = send_silent(&mut sim, t + PERIOD_US, 1);
    let mut t = send_silent(&mut sim, t, 128);
    let mut decisions = Vec::new();
    for _ in 0..8 {
        let boundary = t;
        t = send_silent(&mut sim, t, 128);
        sim.run_until(boundary + PERIOD_US);
        decisions.push(osc.poll(&mut sim, s));
    }
    assert_eq!(decisions[0], Some(3), "the detune draws its steps");
    assert!(
        decisions[1..].iter().all(Option::is_none),
        "the residual holds inside the deadband: {decisions:?}"
    );
    assert_eq!(osc.total, 3);
}

/// A window reading past the +/-8k ppm sanity band is not thermal: the
/// decision is dropped, every window while the shift persists keeps
/// dropping (the anchor stands, nothing re-baselines around the jump), and
/// a CAL re-anchors - the ruler takes the clamped acquire jump.
#[test_log::test]
fn sanity_band_crossing_drops_until_a_cal_re_anchors() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let t = send_silent(&mut sim, 0, 257);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None, "no drift, no decision");
    sim.set_servo_skew_at(t, s, 9_000);
    let t = send_silent(&mut sim, t + PERIOD_US, 1);
    let mut t = send_silent(&mut sim, t, 128);
    for _ in 0..3 {
        let boundary = t;
        t = send_silent(&mut sim, t, 128);
        sim.run_until(boundary + PERIOD_US);
        assert_eq!(sim.poll_clock_trim(s), None, "past the band: dropped");
    }
    sim.run();
    send_train(&mut sim, t + 5_000, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(4), "CAL re-anchors");
}
