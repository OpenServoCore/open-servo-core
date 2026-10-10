//! The kernel lane (`sim::cpu`): the 20 kHz kernel tick as a periodic
//! preemptor below or above the bus vectors, held against the bench's
//! lost-tick counts for polled reads (1.33-1.56 per frame with the kernel
//! below the bus), and the TEL burst it paces from its tail. Above the bus,
//! costs come from the budget table (`osc_servo_core::budget`): at budget
//! (every body at its maximum) where the claim holds at the worst case, and
//! typical (bodies drawn below their maxima at the mean budgets) where only
//! the measured load supports it.

use osc_host::engine::{Command, Outcome};
use osc_integration::sim::{
    HandlerCost, HostEvent, KernelLane, KernelLevel, KernelStats, Sim, Source, TelCosts,
    frame_crc_ok, instruction, status,
};
use osc_protocol::build;
use osc_protocol::wire::{Id, Inst, Opcode, ResultCode, STREAM_SAMPLES_MAX};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::{Frame, Regime};
use osc_servo_core::regions::control::addr::lifecycle::{GOAL_VELOCITY, TEL_COUNT, TEL_MASK};
use rstest::rstest;

const ID: u8 = 1;
const FRAMES: u64 = 200;
const START_US: u64 = 1_000;

/// The kernel-below-bus image the loss pin reproduces, as cpu-probe v2
/// measured it (LOW-exclusive means, three 20 s windows): quiet and holding.
static BELOW_BUS_QUIET: [u64; 10] = [981, 1250, 732, 1087, 768, 1033, 763, 943, 702, 712];
static BELOW_BUS_HOLD: [u64; 10] = [1023, 1250, 1239, 1096, 792, 1072, 763, 943, 702, 712];

/// Its own 32 B READ at 3M (cpu-probe v1, polling ladder): wake 16.0 us,
/// deadline 82.1 us spent over three compare bodies, three arms 20.8 us.
const BELOW_BUS_READ32: HandlerCost = HandlerCost {
    on_break_us: 16,
    on_deadline_us: 20,
    on_tx_complete_us: 7,
    per_frame_us: 22,
};

/// Own 32 B READs at 3M polled at `frames_per_s` with bus costs `cost` and
/// kernel `lane`; the lane's stats and the replies seen.
fn polled_reads(lane: KernelLane, cost: HandlerCost, frames_per_s: u64) -> (KernelStats, u64) {
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, cost);
    // The extra us walks the frames across the kernel grid: the host's
    // clock is not the servo's.
    let gap_us = 1_000_000 / frames_per_s + 1;
    sim.set_kernel_lane(s, lane, START_US + FRAMES * gap_us);
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
    #[values(&BELOW_BUS_QUIET, &BELOW_BUS_HOLD)] phases: &'static [u64; 10],
    #[values(50, 140, 280)] frames_per_s: u64,
) {
    let lane = KernelLane {
        level: KernelLevel::BelowBus,
        phases,
        floor: None,
        tel: TelCosts::default(),
    };
    let (st, replies) = polled_reads(lane, BELOW_BUS_READ32, frames_per_s);
    assert_eq!(replies, FRAMES);
    let per_frame = st.lost as f64 / FRAMES as f64;
    assert!(
        (1.0..=1.6).contains(&per_frame),
        "{per_frame:.2} lost per frame: {st:?}"
    );
}

#[rstest]
fn kernel_on_top_loses_no_tick_under_polling(
    #[values(Regime::Quiet, Regime::Hold, Regime::Moving)] regime: Regime,
    #[values(50, 140, 280)] frames_per_s: u64,
) {
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, regime);
    let cost = HandlerCost::at_budget(Frame::Read32);
    let (st, replies) = polled_reads(lane, cost, frames_per_s);
    assert_eq!(replies, FRAMES);
    assert_eq!((st.lost, st.entries), (0, st.scans), "{st:?}");
    assert_eq!(st.entry_latency_max, 0, "every entry at its scan: {st:?}");
    assert!(st.bus_stretch > 0, "the lane preempted no bus body: {st:?}");
}

/// The attached host's exchanges at 3M, the kernel above the bus: a polled
/// 32 B READ, and the hot loop's GREAD behind a GWRITE(HOLD) and a COMMIT
/// the servo is still working through. Each reply lands inside the host's
/// await window at the default RESPONSE_DEADLINE, moving included, at the
/// typical load: with every moving body at its maximum the hot loop
/// overruns the window.
#[rstest]
fn reply_bound_covers_turnaround_under_kernel_preemption(
    #[values(Regime::Quiet, Regime::Hold, Regime::Moving)] regime: Regime,
    #[values(false, true)] hot_loop: bool,
) {
    const CYCLES: u64 = 100;
    let mut sim = Sim::new(BaudRate::B3000000);
    sim.attach_host();
    let s = sim.add_servo(ID);
    let frame = if hot_loop {
        Frame::Write
    } else {
        Frame::Read32
    };
    sim.set_handler_cost(s, HandlerCost::typical(frame));
    let gap_us = 1_000_000 / 280 + 1;
    let lane = KernelLane::typical(KernelLevel::AboveBus, regime, 1);
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
/// hold, at budget: slot 1 dispatches at the covered checkpoint and serves
/// its verdict behind 400 us of bus backlog, then reclaims one
/// RESPONSE_DEADLINE after it is ready. The attached host is still waiting
/// when its status arrives, flagged. The kernel stretches a backlog by
/// 1 / (1 - U), so the host's one-deadline allowance covers less backlog
/// the heavier the kernel.
#[test]
fn host_waits_for_a_reclaim_counted_from_readiness() {
    const LAG_US: u64 = 400;
    let mut sim = Sim::new(BaudRate::B3000000);
    sim.attach_host();
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::at_budget(Frame::GreadSlot));
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, Regime::Hold);
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

/// The six-field soak mask: a 202 B frame per 800 us batch at 3M.
const SIX_FIELDS: u16 = 0x1cd;
const TEL_FRAMES: u16 = 100;

/// At the typical load: with every body at its maximum the stream loses rows
/// at rest.
#[test]
fn tel_six_fields_fit_at_rest_with_the_kernel_on_top() {
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::typical(Frame::Write));
    let rows = TEL_FRAMES * STREAM_SAMPLES_MAX as u16;
    let lane = KernelLane::typical(KernelLevel::AboveBus, Regime::Quiet, 1);
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
