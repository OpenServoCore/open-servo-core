//! The kernel lane (`sim::cpu`): the 20 kHz kernel tick as a periodic
//! preemptor below or above the bus vectors, held against the bench's
//! lost-tick counts for polled reads (1.33-1.56 per frame with the kernel
//! below the bus), and the TEL burst it paces from its tail.

use osc_host::engine::{Command, Outcome};
use osc_integration::sim::{
    HostEvent, KERNEL_HOLD, KERNEL_MOVING, KERNEL_QUIET, KERNEL_STEP, KERNEL_STEP_SPREAD_US,
    KernelLane, KernelLevel, KernelStats, READ32_3M_COST, Sim, Source, WireFrame, frame_crc_ok,
    instruction, status,
};
use osc_protocol::build;
use osc_protocol::wire::{Id, Inst, Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::control::addr::lifecycle::{GOAL_VELOCITY, TEL_COUNT, TEL_MASK};
use osc_servo_core::tel::FRAME_SAMPLES;
use rstest::rstest;

const ID: u8 = 1;
const FRAMES: u64 = 200;
const START_US: u64 = 1_000;

/// Own 32 B READs at 3M polled at `frames_per_s` with the bench's bus costs
/// and a kernel lane at `level`; the lane's stats and the replies seen.
fn polled_reads(
    level: KernelLevel,
    phases: &'static [u64],
    frames_per_s: u64,
) -> (KernelStats, u64) {
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, READ32_3M_COST);
    // The extra us walks the frames across the kernel grid: the host's
    // clock is not the servo's.
    let gap_us = 1_000_000 / frames_per_s + 1;
    sim.set_kernel_lane(s, KernelLane { level, phases }, START_US + FRAMES * gap_us);
    let mut replies = 0;
    for k in 0..FRAMES {
        let at = START_US + k * gap_us;
        sim.host_send_at(at, &instruction(ID, Opcode::Read, 0, &[0, 0, 32, 0]));
        replies += sim
            .run_until(at + gap_us - 1)
            .iter()
            .filter(|f| f.from == Source::Servo(ID))
            .count() as u64;
    }
    sim.run();
    (sim.kernel_stats(s), replies)
}

#[rstest]
fn kernel_lane_below_bus_reproduces_bench_loss(
    #[values(&KERNEL_QUIET, &KERNEL_HOLD)] phases: &'static [u64; 10],
    #[values(50, 140, 280)] frames_per_s: u64,
) {
    let (st, replies) = polled_reads(KernelLevel::BelowBus, phases, frames_per_s);
    assert_eq!(replies, FRAMES);
    let per_frame = st.lost as f64 / FRAMES as f64;
    assert!(
        (1.0..=1.6).contains(&per_frame),
        "{per_frame:.2} lost per frame: {st:?}"
    );
}

#[rstest]
fn kernel_on_top_loses_no_tick_under_polling(
    #[values(&KERNEL_QUIET, &KERNEL_HOLD, &KERNEL_MOVING)] phases: &'static [u64; 10],
    #[values(50, 140, 280)] frames_per_s: u64,
) {
    let (st, replies) = polled_reads(KernelLevel::AboveBus, phases, frames_per_s);
    assert_eq!(replies, FRAMES);
    assert_eq!((st.lost, st.entries), (0, st.scans), "{st:?}");
    assert_eq!(st.entry_latency_max, 0, "every entry at its scan: {st:?}");
    assert!(st.bus_stretch > 0, "the lane preempted no bus body: {st:?}");
}

/// The attached host's exchanges at 3M, the kernel above the bus: a polled
/// 32 B READ, and the hot loop's GREAD behind a GWRITE(HOLD) and a COMMIT
/// the servo is still working through. Each reply lands inside the host's
/// await window at the default RESPONSE_DEADLINE, moving included.
#[rstest]
fn reply_bound_covers_turnaround_under_kernel_preemption(
    #[values(&KERNEL_QUIET, &KERNEL_HOLD, &KERNEL_MOVING)] phases: &'static [u64; 10],
    #[values(false, true)] hot_loop: bool,
) {
    const CYCLES: u64 = 100;
    let mut sim = Sim::new(BaudRate::B3000000);
    sim.attach_host();
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, READ32_3M_COST);
    let gap_us = 1_000_000 / 280 + 1;
    let lane = KernelLane {
        level: KernelLevel::AboveBus,
        phases,
    };
    sim.set_kernel_lane(s, lane, START_US + CYCLES * gap_us);
    let mut read = [0u8; 4];
    let n = build::read(&mut read, 0, 32).unwrap();
    let read = &read[..n];
    let mut gread = [0u8; 8];
    let n = build::gread_uniform(&mut gread, GOAL_VELOCITY, 4, &[Id::new(ID)]).unwrap();
    let gread = &gread[..n];
    let mut gwrite = [0u8; 8];
    let mut w = build::GwriteUniform::new(&mut gwrite, GOAL_VELOCITY, 4).unwrap();
    w.push(Id::new(ID), &[1, 0, 0, 0]).unwrap();
    let n = w.finish().unwrap();
    let gwrite = &gwrite[..n];
    for k in 0..CYCLES {
        let at = START_US + k * gap_us;
        sim.run_until(at);
        let last = if hot_loop {
            exchange(
                &mut sim,
                Id::BROADCAST,
                Opcode::Gwrite,
                Inst::FLAG_HOLD,
                gwrite,
            );
            exchange(&mut sim, Id::BROADCAST, Opcode::Commit, 0, &[]);
            exchange(&mut sim, Id::BROADCAST, Opcode::Gread, 0, gread)
        } else {
            exchange(&mut sim, Id::new(ID), Opcode::Read, 0, read)
        };
        assert_eq!(last, Outcome::Complete, "cycle {k}");
    }
}

/// Submit one exchange to the attached host and step the wire 1 us at a
/// time until it terminates; its outcome.
fn exchange(sim: &mut Sim, id: Id, op: Opcode, flags: u8, payload: &[u8]) -> Outcome {
    sim.host_submit(Command::Exchange {
        id,
        inst: Inst::instruction(op, flags),
        payload,
    })
    .unwrap();
    let from = sim.now_us();
    for t in from + 1..from + 10_000 {
        sim.run_until(t);
        for ev in sim.host_events() {
            if let HostEvent::Done(t) = ev {
                return t.outcome;
            }
        }
    }
    panic!("the exchange never terminated");
}

/// A GREAD whose slot 0 is absent, at 3M with the kernel above the bus at
/// hold: slot 1 dispatches at the covered checkpoint and serves its verdict
/// 600 us late (the deadline held behind a backlog), then reclaims one
/// RESPONSE_DEADLINE after it is ready. The attached host is still waiting
/// when its status arrives, flagged.
#[test]
fn host_waits_for_a_reclaim_counted_from_readiness() {
    const LAG_US: u64 = 600;
    let mut sim = Sim::new(BaudRate::B3000000);
    sim.attach_host();
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, READ32_3M_COST);
    let lane = KernelLane {
        level: KernelLevel::AboveBus,
        phases: &KERNEL_HOLD,
    };
    sim.set_kernel_lane(s, lane, START_US + 10_000);
    sim.run_until(START_US);
    let mut p = [0u8; 16];
    let n = build::gread_uniform(&mut p, 0, 16, &[Id::new(ID + 1), Id::new(ID)]).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::BROADCAST,
        inst: Inst::instruction(Opcode::Gread, 0),
        payload: &p[..n],
    })
    .unwrap();
    let mut t = START_US;
    while sim.dispatched(s) == 0 {
        t += 1;
        assert!(t < START_US + LAG_US, "the GREAD never dispatched");
        sim.run_until(t);
    }
    sim.preempt_before_ring_read(s, LAG_US);
    sim.run();
    let ev = sim.host_events();
    match ev.first() {
        Some(HostEvent::Status { id, inst, .. }) => {
            assert_eq!(*id, ID);
            assert_eq!(Inst(*inst).result(), Some(ResultCode::PredecessorSilent));
        }
        other => panic!("expected slot 1's status first, got {other:?}"),
    }
}

fn write_u16(addr: u16, v: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let d = v.to_le_bytes();
    instruction(ID, Opcode::Write, 0, &[a[0], a[1], d[0], d[1]])
}

/// The six-field soak mask: a 142 B frame per 550 us batch at 3M.
const SIX_FIELDS: u16 = 0x1cd;
const TEL_FRAMES: u16 = 100;

#[test]
fn tel_six_fields_fit_at_rest_with_the_kernel_on_top() {
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, READ32_3M_COST);
    let rows = TEL_FRAMES * FRAME_SAMPLES as u16;
    let lane = KernelLane {
        level: KernelLevel::AboveBus,
        phases: &KERNEL_QUIET,
    };
    sim.set_kernel_lane(s, lane, START_US + 2 * rows as u64 * 50);
    sim.host_send_at(START_US, &write_u16(TEL_MASK, SIX_FIELDS));
    sim.run_until(START_US + 1_000);
    sim.host_send_at(START_US + 1_000, &write_u16(TEL_COUNT, rows));
    let frames = sim.run();
    let stream: Vec<_> = frames
        .iter()
        .filter(|f| f.from == Source::Servo(ID) && status(f).0.result() == Some(ResultCode::Stream))
        .collect();
    assert_eq!(stream.len(), TEL_FRAMES as usize);
    assert!(stream.iter().all(|f| frame_crc_ok(f)));
    assert_eq!(sim.tel_drops(s), 0, "rows dropped of {rows}");
}

/// A six-field burst of `rows` at 3M under the step soak's kernel, every
/// body `extra_us` heavier, spread by `seed`: the stream frames, the rows
/// dropped and the kernel's stats.
fn step_burst(rows: u16, extra_us: u64, seed: u64) -> (Vec<WireFrame>, u16, KernelStats) {
    let phases: &'static [u64] = Box::leak(
        KERNEL_STEP
            .iter()
            .map(|c| c + extra_us * 48)
            .collect::<Vec<_>>()
            .into_boxed_slice(),
    );
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, READ32_3M_COST);
    let lane = KernelLane {
        level: KernelLevel::AboveBus,
        phases,
    };
    sim.set_kernel_lane(s, lane, START_US + 2 * rows as u64 * 50);
    sim.spread_kernel(s, KERNEL_STEP_SPREAD_US, seed);
    sim.host_send_at(START_US, &write_u16(TEL_MASK, SIX_FIELDS));
    sim.run_until(START_US + 1_000);
    sim.host_send_at(START_US + 1_000, &write_u16(TEL_COUNT, rows));
    let stream = sim
        .run()
        .into_iter()
        .filter(|f| f.from == Source::Servo(ID) && status(f).0.result() == Some(ResultCode::Stream))
        .collect();
    (stream, sim.tel_drops(s), sim.kernel_stats(s))
}

const STEP_ROWS: u16 = 4000;

/// The step lane carries the soak's kernel: with the burst's encode on top,
/// 4-7% of its ticks run over one period (bench: 5.3-5.5%).
#[rstest]
fn kernel_step_lane_overruns_as_the_step_soak(#[values(1, 2, 3, 4)] seed: u64) {
    let (_, drops, st) = step_burst(STEP_ROWS, 0, seed);
    let ticks = (STEP_ROWS + drops) as f64;
    let over = st.over as f64 / ticks;
    assert!(
        (0.04..=0.07).contains(&over),
        "{:.1}% over: {st:?}",
        100.0 * over
    );
}

/// Six fields at 3M keep up while the motor steps: every 11-row frame
/// leaves as one arm from its CRC'd buffer, chained from the previous
/// frame's TC, so the wire waits only for that TC between frames, and the
/// third buffer absorbs runs of kernel overruns. The two-arm stager staged
/// from the SW vector, at 16 rows, loses 17-20% of the rows here (bench:
/// 19.6%). Margin: zero lost with every body 2 us heavier still.
#[rstest]
fn tel_six_fields_keep_up_while_stepping(
    #[values(1, 2, 3, 4)] seed: u64,
    #[values(0, 2)] extra_us: u64,
) {
    let (stream, drops, st) = step_burst(STEP_ROWS, extra_us, seed);
    assert_eq!(drops, 0, "rows dropped: {st:?}");
    let frames = (STEP_ROWS as usize).div_ceil(FRAME_SAMPLES);
    assert_eq!(stream.len(), frames);
    assert!(stream.iter().all(frame_crc_ok));
    for (k, f) in stream.iter().enumerate() {
        assert_eq!(status(f).1[0], k as u8, "stream_seq");
    }
}
