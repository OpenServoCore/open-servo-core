//! The production hot loop (§5.2/§6): the host bombards the bus with
//! back-to-back silent frames — `[GWRITE(HOLD) × N, COMMIT, GREAD]` repeated
//! forever — and the servos must keep up with zero gap between frames: no
//! drops, no CRC fails, staged state applied atomically at COMMIT, and the
//! GREAD (sent back-to-back after COMMIT) already reading the new values.
//!
//! The sim delivers wire events at exact ticks with zero-cost handlers, so
//! these tests pin the LOGICAL zero-gap contract (framer re-anchor per break,
//! staging across frames, commit-before-read ordering). ISR-latency effects
//! are silicon-only and out of scope here.

use osc_core::BaudRate;
use osc_core::regions::control::addr::lifecycle::GOAL_VELOCITY;
use osc_core::regions::control::addr::streaming::STREAM_DECIMATION;
use osc_integration::sim::{Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Inst, Opcode, ResultCode};

const RATE: BaudRate = BaudRate::B1000000;
/// See chains.rs: break-keyed reclaim makes the 60 µs default sound at 1 M.
const CHAIN_DEADLINE_US: u16 = 60;
const BCAST: u8 = 0xFE;
const IDS: [u8; 3] = [1, 2, 3];

fn gwrite_uniform(addr: u16, count: u8, entries: &[(u8, &[u8])]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&addr.to_le_bytes());
    p.push(count);
    for &(id, data) in entries {
        assert_eq!(data.len(), count as usize, "uniform GWRITE stride");
        p.push(id);
        p.extend_from_slice(data);
    }
    p
}

fn gwrite_per_target(entries: &[(u8, u16, &[u8])]) -> Vec<u8> {
    let mut p = Vec::new();
    for &(id, addr, data) in entries {
        p.push(id);
        p.extend_from_slice(&addr.to_le_bytes());
        p.push(data.len() as u8);
        p.extend_from_slice(data);
    }
    p
}

fn gread_uniform(addr: u16, count: u16, ids: &[u8]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&addr.to_le_bytes());
    p.extend_from_slice(&count.to_le_bytes());
    p.extend_from_slice(ids);
    p
}

fn host_frames(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames.iter().filter(|f| f.from == Source::Host).collect()
}

fn replies(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

/// Every queued host frame must start exactly where the previous one ended —
/// the test is void if the scheduler ever re-introduces pacing gaps.
fn assert_zero_gap(hosts: &[&WireFrame]) {
    for w in hosts.windows(2) {
        assert_eq!(
            w[1].at, w[0].end,
            "host frames must be back-to-back: {hosts:#?}"
        );
    }
}

#[test]
fn hot_loop_cycles_survive_zero_gap() {
    let mut sim = Sim::new(RATE);
    for id in IDS {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }

    for cycle in 0..10u32 {
        let gv: Vec<i32> = IDS
            .iter()
            .map(|&id| (cycle as i32) * 100 + id as i32)
            .collect();
        let deci = (cycle % 128) as u8 + 1;

        // GWRITE(HOLD) × 2, COMMIT, GREAD — one burst, zero gap throughout.
        let bytes: Vec<[u8; 4]> = gv.iter().map(|v| v.to_le_bytes()).collect();
        let entries: Vec<(u8, &[u8])> = IDS
            .iter()
            .zip(bytes.iter())
            .map(|(&id, d)| (id, &d[..]))
            .collect();
        sim.host_send(&instruction(
            BCAST,
            Opcode::Gwrite,
            Inst::FLAG_HOLD,
            &gwrite_uniform(GOAL_VELOCITY, 4, &entries),
        ));
        let deci_entries: Vec<(u8, u16, &[u8])> = IDS
            .iter()
            .map(|&id| (id, STREAM_DECIMATION, std::slice::from_ref(&deci)))
            .collect();
        sim.host_send(&instruction(
            BCAST,
            Opcode::Gwrite,
            Inst::FLAG_HOLD | Inst::FLAG_PER_TARGET,
            &gwrite_per_target(&deci_entries),
        ));
        sim.host_send(&instruction(BCAST, Opcode::Commit, 0, &[]));
        sim.host_send(&instruction(
            BCAST,
            Opcode::Gread,
            0,
            &gread_uniform(GOAL_VELOCITY, 4, &IDS),
        ));
        let frames = sim.run();

        let hosts = host_frames(&frames);
        assert_eq!(hosts.len(), 4, "cycle {cycle}: 4 host frames");
        assert_zero_gap(&hosts);

        // Both staged writes applied at COMMIT.
        for (i, v) in gv.iter().enumerate() {
            assert_eq!(
                sim.servo_table(i, |t| t.control.lifecycle.goal_velocity),
                *v,
                "cycle {cycle}: servo {i} goal_velocity committed"
            );
            assert_eq!(
                sim.servo_table(i, |t| t.control.streaming.stream_decimation),
                deci,
                "cycle {cycle}: servo {i} stream_decimation committed"
            );
        }

        // GREAD chained replies in id-list order, already carrying the values
        // committed one frame earlier on the wire.
        let r = replies(&frames);
        assert_eq!(r.len(), IDS.len(), "cycle {cycle}: {frames:#?}");
        for (k, (&id, v)) in IDS.iter().zip(gv.iter()).enumerate() {
            assert_eq!(r[k].from, Source::Servo(id), "cycle {cycle} slot {k}");
            assert_valid(r[k]);
            let (inst, payload) = status(r[k]);
            assert_eq!(inst.result(), Some(ResultCode::Ok));
            assert_eq!(payload, v.to_le_bytes(), "cycle {cycle} slot {k} data");
        }

        for i in 0..IDS.len() {
            let d = sim.servo_diag(i);
            assert_eq!(d.crc_fail_count, 0, "cycle {cycle}: servo {i} crc");
            assert_eq!(d.framing_drop_count, 0, "cycle {cycle}: servo {i} drops");
        }
    }
}

#[test]
fn noreply_write_bombardment_then_read() {
    let mut sim = Sim::new(RATE);
    let s = sim.add_servo_with(IDS[0], 0, CHAIN_DEADLINE_US);

    // 20 back-to-back silent writes to the same field; only the last survives.
    let a = GOAL_VELOCITY.to_le_bytes();
    let mut last: i32 = 0;
    for k in 1..=20i32 {
        last = k * 7;
        let v = last.to_le_bytes();
        sim.host_send(&instruction(
            IDS[0],
            Opcode::Write,
            Inst::FLAG_NOREPLY,
            &[a[0], a[1], v[0], v[1], v[2], v[3]],
        ));
    }
    // ... then a plain READ, still zero gap after the burst.
    sim.host_send(&instruction(IDS[0], Opcode::Read, 0, &[a[0], a[1], 4, 0]));
    let frames = sim.run();

    let hosts = host_frames(&frames);
    assert_eq!(hosts.len(), 21, "20 writes + 1 read");
    assert_zero_gap(&hosts);

    let r = replies(&frames);
    assert_eq!(r.len(), 1, "writes are silent, only the read replies");
    assert_valid(r[0]);
    let (inst, payload) = status(r[0]);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(payload, last.to_le_bytes());
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_velocity),
        last
    );

    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0);
    assert_eq!(d.framing_drop_count, 0);
}

/// THE zero-gap repro (§5 of the transport pillar): the same hot loop under
/// silicon-realistic handler latency. Dispatch runs inside the deadline body
/// on this architecture, so a GWRITE-class body (~70 µs measured) makes
/// following breaks pend and coalesce; a base that anchors on ISR-entry
/// cursor snapshots then silently discards intact frames (staleness, not
/// latency — Step-0 evidence). The amended design (A1 `last_break_tick` +
/// A2 stream-continuity resolution) must keep this green at both bauds.
fn hot_loop_with_latency(rate: BaudRate) {
    use osc_integration::sim::HandlerCost;

    let mut sim = Sim::new(rate);
    // RESPONSE_DEADLINE must exceed worst-case dispatch+staging latency or a
    // chain slot reclaims into a live-but-slow predecessor (band finding,
    // pillar §7 acceptance): with 70 µs handler bodies the 60 µs default is
    // dishonest — a real deployment tunes this register to its worst case.
    for id in IDS {
        sim.add_servo_with(id, 0, 250);
    }
    for i in 0..IDS.len() {
        sim.set_handler_cost(
            i,
            HandlerCost {
                on_break_us: 2,
                on_deadline_us: 5,
                on_tx_complete_us: 2,
                on_dispatch_us: 70,
            },
        );
    }

    for cycle in 0..10u32 {
        let gv: Vec<i32> = IDS
            .iter()
            .map(|&id| (cycle as i32) * 100 + id as i32)
            .collect();
        let bytes: Vec<[u8; 4]> = gv.iter().map(|v| v.to_le_bytes()).collect();
        let entries: Vec<(u8, &[u8])> = IDS
            .iter()
            .zip(bytes.iter())
            .map(|(&id, d)| (id, &d[..]))
            .collect();
        sim.host_send(&instruction(
            BCAST,
            Opcode::Gwrite,
            Inst::FLAG_HOLD,
            &gwrite_uniform(GOAL_VELOCITY, 4, &entries),
        ));
        sim.host_send(&instruction(BCAST, Opcode::Commit, 0, &[]));
        sim.host_send(&instruction(
            BCAST,
            Opcode::Gread,
            0,
            &gread_uniform(GOAL_VELOCITY, 4, &IDS),
        ));
        let frames = sim.run();
        assert_zero_gap(&host_frames(&frames));

        for (i, v) in gv.iter().enumerate() {
            assert_eq!(
                sim.servo_table(i, |t| t.control.lifecycle.goal_velocity),
                *v,
                "cycle {cycle}: servo {i} lost a GWRITE or COMMIT under latency"
            );
        }
        let r = replies(&frames);
        assert_eq!(
            r.len(),
            IDS.len(),
            "cycle {cycle}: chain incomplete under latency"
        );
    }
}

#[test]
fn hot_loop_survives_handler_latency_1m() {
    hot_loop_with_latency(BaudRate::B1000000);
}

#[test]
fn hot_loop_survives_handler_latency_3m() {
    hot_loop_with_latency(BaudRate::B3000000);
}

/// Silicon-captured trap (event-trace 2026-07-08): when a frame's own break
/// FE delivery lags past its covered checkpoint (pended behind a long
/// deadline body), the resolver reaches Covered INSIDE that on_break wake and
/// stages the reply — and the FE-kill in the same wake must not murder it.
/// The reply belongs to the frontier frame still arriving, not to a frame the
/// host abandoned; killing it leaves a stale speculation that later arms the
/// chain over an empty engine (ghost trigger, silent no-reply).
fn plain_burst_with_deadline_latency(rate: BaudRate) {
    use osc_integration::sim::HandlerCost;

    // Sweep the body cost so some delivery lands inside the covered window
    // ([frame end - 2 byte-times, frame end)) regardless of rate — the trap
    // is a ~2-byte-time bullseye, not a single magic latency.
    for dl in (10u32..100).step_by(2) {
        let mut sim = Sim::new(rate);
        sim.add_servo_with(1, 0, 250);
        sim.set_handler_cost(
            0,
            HandlerCost {
                on_break_us: 2,
                on_deadline_us: dl,
                on_tx_complete_us: 2,
                on_dispatch_us: dl,
            },
        );

        for cycle in 0..5u32 {
            let v = cycle as i32 + 1;
            let d = v.to_le_bytes();
            let a = GOAL_VELOCITY.to_le_bytes();
            let mut p = vec![a[0], a[1]];
            p.extend_from_slice(&d);
            sim.host_send(&instruction(1, Opcode::Write, Inst::FLAG_NOREPLY, &p));
            sim.host_send(&instruction(1, Opcode::Write, Inst::FLAG_NOREPLY, &p));
            sim.host_send(&instruction(1, Opcode::Read, 0, &[a[0], a[1], 4, 0]));
            let frames = sim.run();
            assert_zero_gap(&host_frames(&frames));
            assert_eq!(
                sim.servo_table(0, |t| t.control.lifecycle.goal_velocity),
                v,
                "dl={dl} cycle {cycle}: write lost under deadline latency"
            );
            let r = replies(&frames);
            assert_eq!(
                r.len(),
                1,
                "dl={dl} cycle {cycle}: READ reply lost (staged reply killed in its own wake?)"
            );
            assert_valid(r[0]);
            let (inst, payload) = status(r[0]);
            assert_eq!(inst.result(), Some(ResultCode::Ok), "dl={dl} cycle {cycle}");
            assert_eq!(payload, d, "dl={dl} cycle {cycle}: stale read-back");
        }
        // A clean zero-gap wire gives the framer no excuse: every give-up
        // here is a live frame sacrificed by starvation accounting.
        let diag = sim.servo_diag(0);
        assert_eq!(
            diag.framing_drop_count, 0,
            "dl={dl}: live frames sacrificed"
        );
        assert_eq!(diag.crc_fail_count, 0, "dl={dl}: trusted frame failed");
    }
}

#[test]
fn plain_burst_survives_deadline_latency_1m() {
    plain_burst_with_deadline_latency(BaudRate::B1000000);
}

#[test]
fn plain_burst_survives_deadline_latency_3m() {
    plain_burst_with_deadline_latency(BaudRate::B3000000);
}

/// The bench's plain burst (`tool-osc-burst --plain`: 8 NOREPLY WRITEs +
/// READ, host waits out the reply between cycles) with IRREGULAR intra-burst
/// gaps: the pirate's TXE-poll bubbles put 0–3-byte-time pauses between
/// frames, so the framer flips between caught-up (speculation at HIGH) and
/// backlog (consumer at LOW) mid-burst — every seam of the A3(b) handoff.
/// Silicon showed ~0.3% no-reply/stale residuals at 1M in exactly this
/// pattern; a drop here is that residual, deterministic.
fn plain_burst_with_gap_jitter(rate: BaudRate) {
    use osc_integration::sim::HandlerCost;

    let a = GOAL_VELOCITY.to_le_bytes();
    for dl in [5u32, 20, 45, 70] {
        // Deterministic LCG per cost point; gaps vary per frame AND drift
        // per cycle so cycle boundaries never resonate with the sweep.
        let mut lcg: u64 = 0x243F6A8885A308D3 ^ (dl as u64);
        let mut gap = move || {
            lcg = lcg
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            // 0..=30 µs: from zero-gap up to ~3 byte-times at 1M.
            (lcg >> 33) % 31
        };

        let mut sim = Sim::new(rate);
        sim.add_servo_with(1, 0, 250);
        sim.set_handler_cost(
            0,
            HandlerCost {
                on_break_us: 2,
                on_deadline_us: 5,
                on_tx_complete_us: 2,
                on_dispatch_us: dl,
            },
        );

        for cycle in 0..60u32 {
            let v = cycle as i32 + 1;
            let d = v.to_le_bytes();
            let mut at = sim.now_us() + 20;
            for k in 0..8u32 {
                let vk = v - (7 - k as i32);
                let dk = vk.to_le_bytes();
                let p = vec![a[0], a[1], dk[0], dk[1], dk[2], dk[3]];
                sim.host_send_at(at, &instruction(1, Opcode::Write, Inst::FLAG_NOREPLY, &p));
                // Next frame starts after this one plus the jittered bubble.
                at += frame_us(rate, 10) + gap();
            }
            sim.host_send_at(at, &instruction(1, Opcode::Read, 0, &[a[0], a[1], 4, 0]));
            let frames = sim.run();

            assert_eq!(
                sim.servo_table(0, |t| t.control.lifecycle.goal_velocity),
                v,
                "dl={dl} cycle {cycle}: write lost"
            );
            let r = replies(&frames);
            assert_eq!(r.len(), 1, "dl={dl} cycle {cycle}: READ reply lost");
            assert_valid(r[0]);
            let (inst, payload) = status(r[0]);
            assert_eq!(inst.result(), Some(ResultCode::Ok), "dl={dl} cycle {cycle}");
            assert_eq!(payload, d, "dl={dl} cycle {cycle}: stale read-back");
        }
        let diag = sim.servo_diag(0);
        assert_eq!(diag.framing_drop_count, 0, "dl={dl}: live frames dropped");
        assert_eq!(diag.crc_fail_count, 0, "dl={dl}: trusted frame failed");
    }
}

/// Wire time of one frame (break + `n` data bytes) in µs at `rate`, rounded
/// up — the jitter scheduler only needs a "no overlap" floor, not accuracy.
fn frame_us(rate: BaudRate, n: u64) -> u64 {
    let bits = 14 + n * 10;
    (bits * 1_000_000).div_ceil(rate.as_hz() as u64)
}

#[test]
fn plain_burst_survives_gap_jitter_1m() {
    plain_burst_with_gap_jitter(BaudRate::B1000000);
}

#[test]
fn plain_burst_survives_gap_jitter_3m() {
    plain_burst_with_gap_jitter(BaudRate::B3000000);
}

/// The failing silicon signature (bench 2026-07-08, `tool-reply-edges`
/// STALE dump): the pirate's TXE feed stalls 50–100 bit-times INSIDE a
/// frame — mid-write4 and mid-READ on the captured cycle — so the frontier
/// burns its recheck budget and parks at the starvation horizon while the
/// frame quietly completes (ring bytes raise no IRQ). The reply then rode
/// a ~2×640 µs-late wake carrying a value one write behind. Sweep the
/// stall length and position across every frame of the burst, capped
/// below the dead-transmitter horizon (64 byte-times, §3.3): a stall past
/// it IS a dead transmitter and the frame is sacrificed by design.
fn plain_burst_with_midframe_stall(rate: BaudRate) {
    use osc_integration::sim::HandlerCost;

    let horizon_us = 64 * 10 * 1_000_000u64 / rate.as_hz() as u64;
    let stalls: Vec<u64> = [30u64, 60, 94, 150, 400, 700]
        .into_iter()
        .filter(|s| *s < horizon_us * 3 / 4)
        .collect();
    let a = GOAL_VELOCITY.to_le_bytes();
    for dl in [5u32, 20, 45, 70] {
        for &stall_us in &stalls {
            for stalled_frame in 0..9usize {
                let mut sim = Sim::new(rate);
                sim.add_servo_with(1, 0, 250);
                sim.set_handler_cost(
                    0,
                    HandlerCost {
                        on_break_us: 2,
                        on_deadline_us: 5,
                        on_tx_complete_us: 2,
                        on_dispatch_us: dl,
                    },
                );

                for cycle in 0..3u32 {
                    let v = (cycle as i32 + 1) + 100 * stalled_frame as i32;
                    let d = v.to_le_bytes();
                    for k in 0..8u32 {
                        let vk = v - (7 - k as i32);
                        let dk = vk.to_le_bytes();
                        let p = vec![a[0], a[1], dk[0], dk[1], dk[2], dk[3]];
                        let f = instruction(1, Opcode::Write, Inst::FLAG_NOREPLY, &p);
                        if k as usize == stalled_frame {
                            // Split after the header + 2 payload bytes, the
                            // captured shape.
                            sim.host_send_stalled(&f, 6, stall_us);
                        } else {
                            sim.host_send(&f);
                        }
                    }
                    let read = instruction(1, Opcode::Read, 0, &[a[0], a[1], 4, 0]);
                    if stalled_frame == 8 {
                        sim.host_send_stalled(&read, 6, stall_us);
                    } else {
                        sim.host_send(&read);
                    }
                    let frames = sim.run();

                    let ctx =
                        format!("dl={dl} stall={stall_us} frame={stalled_frame} cycle={cycle}");
                    assert_eq!(
                        sim.servo_table(0, |t| t.control.lifecycle.goal_velocity),
                        v,
                        "{ctx}: write lost"
                    );
                    let r = replies(&frames);
                    assert_eq!(r.len(), 1, "{ctx}: reply count");
                    assert_valid(r[0]);
                    let (inst, payload) = status(r[0]);
                    assert_eq!(inst.result(), Some(ResultCode::Ok), "{ctx}");
                    assert_eq!(payload, d, "{ctx}: stale read-back");
                }
                let diag = sim.servo_diag(0);
                assert_eq!(
                    diag.crc_fail_count, 0,
                    "dl={dl} stall={stall_us} frame={stalled_frame}: trusted frame failed"
                );
            }
        }
    }
}

/// §7 throughput invariant: the consumer outruns the wire, so a sustained
/// zero-gap flood of minimal frames drains through the ring (the queue, A2)
/// with zero loss — the 512 B ring laps several times over the burst. The
/// cost sweep stays inside the contract (per-frame dispatch below per-frame
/// wire time); sustained overrun is out of contract by design.
fn zero_gap_flood(rate: BaudRate, dispatch_costs: &[u32]) {
    use osc_integration::sim::HandlerCost;

    const FRAMES: i32 = 100;
    let a = GOAL_VELOCITY.to_le_bytes();
    for &dl in dispatch_costs {
        let mut sim = Sim::new(rate);
        sim.add_servo_with(1, 0, 250);
        sim.set_handler_cost(
            0,
            HandlerCost {
                on_break_us: 2,
                on_deadline_us: 5,
                on_tx_complete_us: 2,
                on_dispatch_us: dl,
            },
        );

        for k in 1..=FRAMES {
            let d = k.to_le_bytes();
            let p = vec![a[0], a[1], d[0], d[1], d[2], d[3]];
            sim.host_send(&instruction(1, Opcode::Write, Inst::FLAG_NOREPLY, &p));
        }
        let read = instruction(1, Opcode::Read, 0, &[a[0], a[1], 4, 0]);
        sim.host_send(&read);
        let frames = sim.run();

        assert_zero_gap(&host_frames(&frames));
        assert_eq!(
            sim.servo_table(0, |t| t.control.lifecycle.goal_velocity),
            FRAMES,
            "dl={dl}: flood tail write lost"
        );
        let r = replies(&frames);
        assert_eq!(r.len(), 1, "dl={dl}: reply count");
        assert_valid(r[0]);
        let (inst, payload) = status(r[0]);
        assert_eq!(inst.result(), Some(ResultCode::Ok), "dl={dl}");
        assert_eq!(
            payload,
            FRAMES.to_le_bytes(),
            "dl={dl}: stale flood read-back"
        );
        let diag = sim.servo_diag(0);
        assert_eq!(diag.crc_fail_count, 0, "dl={dl}: trusted frame failed");
        assert_eq!(diag.framing_drop_count, 0, "dl={dl}: flood dropped frames");
    }
}

#[test]
fn zero_gap_flood_1m() {
    zero_gap_flood(BaudRate::B1000000, &[5, 20, 45, 70]);
}

/// At 3M a minimal write is ~40 µs of wire; the sweep stays below it — a
/// 70 µs consumer would be sustained overrun, out of contract.
#[test]
fn zero_gap_flood_3m() {
    zero_gap_flood(BaudRate::B3000000, &[5, 20, 30]);
}

#[test]
fn plain_burst_survives_midframe_stall_1m() {
    plain_burst_with_midframe_stall(BaudRate::B1000000);
}

#[test]
fn plain_burst_survives_midframe_stall_3m() {
    plain_burst_with_midframe_stall(BaudRate::B3000000);
}
