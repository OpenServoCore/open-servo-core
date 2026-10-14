//! The kernel lane (`sim::cpu`): the 20 kHz kernel tick as a periodic
//! preemptor below or above the bus vectors, held against the bench's
//! lost-tick counts for polled reads (1.33-1.56 per frame with the kernel
//! below the bus), and the TEL burst it paces from its tail. Above the bus,
//! costs come from the budget table (`osc_servo_core::budget`) at budget:
//! every body at its maximum.
//! Pins run at budget; a red pin is a design finding, never a looser
//! threshold.

use osc_host::engine::{CHAIN_READY_LAG_US, Command, Outcome};
use osc_integration::sim::{
    HandlerCost, HostEvent, KernelEvents, KernelLane, KernelLevel, KernelStats, Motion, Sim,
    Source, TelCosts, WireFrame, frame_crc_ok, instruction, status,
};
use osc_protocol::build;
use osc_protocol::wire::{Id, Inst, Opcode, ResultCode, STREAM_SAMPLES_MAX};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::{self, Body, Events, Frame, PHASES, Regime};
use osc_servo_core::kernel::{self, phase};
use osc_servo_core::regions::control::addr::lifecycle::GOAL_VELOCITY;
use osc_servo_core::regions::control::addr::tel::{CHUNK0_ADDR, TEL_COUNT};
use osc_servo_core::regions::telemetry::addr::{estimates, tick};
use osc_servo_core::tel::{Chunk, Chunks};
use rstest::rstest;

const ID: u8 = 1;
const FRAMES: u64 = 200;
const TICKS_PER_US: u64 = budget::TICKS_PER_US as u64;
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
        motion: Motion::Quiet,
        phases: [phases, phases],
        floor: None,
        tel: TelCosts::default(),
        events: KernelEvents::default(),
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
#[case::quiet(Motion::Quiet)]
#[case::hold(Motion::Hold)]
#[case::moving(Motion::Moving)]
fn kernel_on_top_loses_no_tick_under_polling(
    #[case] motion: Motion,
    #[values(50, 140, 280)] frames_per_s: u64,
) {
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, motion);
    let cost = HandlerCost::at_budget(Frame::Read32);
    let (st, replies) = polled_reads(lane, cost, frames_per_s);
    assert_eq!(replies, FRAMES);
    assert_eq!((st.lost, st.entries), (0, st.scans), "{st:?}");
    assert_eq!(st.entry_latency_max, 0, "every entry at its scan: {st:?}");
    assert!(st.bus_stretch > 0, "the lane preempted no bus body: {st:?}");
}

/// A Closed tick at budget carrying `ev` on `phase`, with the health close
/// where it can land and interrupt entry and exit.
fn event_tick(phase: u8, ev: Events) -> u32 {
    let health = kernel::health_close_lands(phase as usize);
    budget::kernel_tick(Regime::Closed, phase as usize, Events { health, ..ev })
        + budget::ENTRY_EXIT
}

/// A seated reference close, staged: the SLOW finish that freezes the
/// window, the install, the side work after it, each Closed at budget, and
/// streaming with a TEL row and its bank on the same tick.
#[rstest]
#[case::quiet(false)]
#[ignore = "the thermometer's stages are being trimmed: THERM_INSTALL and THERM_SIDE streaming run over the period at budget"]
#[case::streaming(true)]
fn thermal_close_fits_at_budget_while_seated(#[case] tel: bool) {
    let carried = Events {
        tel,
        bank: tel,
        ..Events::default()
    };
    let stages = [
        (
            phase::SLOW,
            Events {
                finish: true,
                close: true,
                ..carried
            },
        ),
        (
            phase::SPARE_A,
            Events {
                install: true,
                ..carried
            },
        ),
        (
            phase::SPARE_B,
            Events {
                side: true,
                ..carried
            },
        ),
    ];
    let over: Vec<_> = stages
        .into_iter()
        .map(|(p, ev)| (p, event_tick(p, ev)))
        .filter(|&(_, t)| t > budget::TICK_PERIOD)
        .collect();
    assert!(
        over.is_empty(),
        "(phase, ticks) over {}: {over:?}",
        budget::TICK_PERIOD
    );
}

const NONE: Events = Events {
    refresh: false,
    tel: false,
    bank: false,
    slow: false,
    finish: false,
    close: false,
    install: false,
    side: false,
    health: false,
};

/// The SLOW block's stages, the due tick, the finish and a finish that
/// freezes a reference, each on its own SLOW tick.
const SLOW_STAGES: [Events; 3] = [
    Events { slow: true, ..NONE },
    Events {
        finish: true,
        ..NONE
    },
    Events {
        finish: true,
        close: true,
        ..NONE
    },
];

/// The SLOW stages while six fields stream and step, Closed at budget,
/// with a bank on the tick.
#[test]
fn slow_stages_fit_at_budget_while_streaming() {
    for stage in SLOW_STAGES {
        let ev = Events {
            tel: true,
            bank: true,
            ..stage
        };
        let t = event_tick(phase::SLOW, ev);
        assert!(
            t <= budget::TICK_PERIOD,
            "{ev:?}: {t} over {}",
            budget::TICK_PERIOD
        );
    }
}

/// Every phase's plain tick while a burst streams, Closed at budget: the
/// TEL row and a bank or the TX lane's dearest step, with the health close
/// where it can land and interrupt entry and exit.
#[test]
fn plain_phases_fit_at_budget_while_streaming() {
    let carried = Events {
        tel: true,
        bank: true,
        ..NONE
    };
    let over: Vec<_> = (0..PHASES as u8)
        .map(|p| (p, event_tick(p, carried)))
        .filter(|&(_, t)| t > budget::TICK_PERIOD)
        .collect();
    assert!(
        over.is_empty(),
        "(phase, ticks) over {}: {over:?}",
        budget::TICK_PERIOD
    );
}

/// A row copy's pace on DMA1 CH6, halfwords memory to memory: ~0.125 us a
/// byte (transport sec 2).
const ROW_COPY_TICKS_PER_BYTE: u32 = budget::TICKS_PER_US / 8;

/// The scans the ADC DMA lands in the tick block hold still for the tick
/// that reads them: at budget, the trough's samples leave the next trough
/// scan's way, and every read of the crest scan and every row's one DMA
/// copy of it is armed, behind the dearest stage that runs ahead of the
/// fast step, before the next crest scan lands, the copy's own time
/// included. The DMA copies the row's longer chunk, at most the whole row
/// of one chunk; the CPU copies the shorter behind the arm, while the DMA
/// runs, inside FRAME_DONE. CH6 is idle at every arm: a row's copy lands
/// before the next tick's entry, and a late tick's row arms an exit and
/// an entry behind the last at least.
#[test]
fn the_tick_reads_its_scan_before_the_next_lands() {
    use budget::{
        ENTRY, ENTRY_EXIT, FRAME_DONE, NEXT_CREST, NEXT_TROUGH, REFRESH, THERM_INSTALL, THERM_SIDE,
        TROUGH_DONE,
    };
    let copy = osc_servo_core::tel::ROW_BYTES_MAX as u32 * ROW_COPY_TICKS_PER_BYTE;
    let ahead = [REFRESH, THERM_INSTALL, THERM_SIDE]
        .into_iter()
        .max()
        .unwrap_or(0);
    const { assert!(ENTRY + TROUGH_DONE <= NEXT_TROUGH) };
    let done = ENTRY + ahead + FRAME_DONE + copy;
    assert!(done <= NEXT_CREST, "{done} past {NEXT_CREST}");
    assert!(copy <= ENTRY_EXIT, "a {copy} copy outlasts {ENTRY_EXIT}");
}

/// What any tick may carry: nothing, a TEL row, a row and its bank.
fn carried() -> [Events; 3] {
    let tel = Events { tel: true, ..NONE };
    [NONE, tel, Events { bank: true, ..tel }]
}

/// `stage` on the SLOW phase on top of `c`.
fn slow_tick(r: Regime, stage: Events, c: Events) -> u32 {
    let ev = Events {
        slow: stage.slow,
        finish: stage.finish,
        close: stage.close,
        ..c
    };
    budget::kernel_tick(r, phase::SLOW as usize, ev)
}

/// Closed, the SLOW block never sets the period's longest tick: each stage
/// on the SLOW phase, carrying what any tick may carry, is no longer than
/// the heaviest other phase carrying the same. So wherever the plain
/// Closed phases fit, the SLOW ticks fit too.
#[test]
fn slow_stages_never_set_the_closed_tick_max_at_budget() {
    let closed = Regime::Closed;
    for c in carried() {
        let heaviest = (0..PHASES)
            .filter(|&p| p != phase::SLOW as usize)
            .map(|p| budget::kernel_tick(closed, p, c))
            .max()
            .unwrap_or(0);
        for stage in SLOW_STAGES {
            let t = slow_tick(closed, stage, c);
            assert!(t <= heaviest, "{stage:?} on {c:?}: {t} over {heaviest}");
        }
    }
}

/// Open ticks leave at least this much of the period free at a SLOW stage.
const OPEN_SLOW_FREE: u32 = budget::TICK_PERIOD / 10;

/// Open, a SLOW stage may set the period's longest tick: the due stage is
/// priced on a tracked seat, which OpenLoop can hold. Each stage tick,
/// carrying what any tick may carry, with the health close where it can
/// land and interrupt entry and exit, still leaves [`OPEN_SLOW_FREE`].
#[test]
fn open_slow_stages_leave_a_tenth_of_the_period_at_budget() {
    let health = kernel::health_close_lands(phase::SLOW as usize);
    for c in carried() {
        for stage in SLOW_STAGES {
            let t = slow_tick(Regime::Open, stage, Events { health, ..c }) + budget::ENTRY_EXIT;
            assert!(
                t + OPEN_SLOW_FREE <= budget::TICK_PERIOD,
                "{stage:?} on {c:?}: {t}"
            );
        }
    }
}

/// The attached host's exchanges at 3M, the kernel above the bus at
/// budget: a polled 32 B READ, and the hot loop's GREAD behind a
/// GWRITE(HOLD) and a COMMIT the servo is still working through. Each reply
/// lands inside the host's await window at the default RESPONSE_DEADLINE,
/// moving included; the GREAD's through the chain's readiness allowance.
#[rstest]
fn reply_bound_covers_turnaround_under_kernel_preemption(
    #[values(Motion::Quiet, Motion::Hold, Motion::Moving)] motion: Motion,
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
    sim.set_handler_cost(s, HandlerCost::at_budget(frame));
    let gap_us = 1_000_000 / 280 + 1;
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, motion);
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

/// The most bus work a chain slot can work through before it is ready,
/// HCLK ticks: ahead of it the hot loop's GWRITE(HOLD) and COMMIT (protocol
/// sec 7), each at most a `frame`, then its own GREAD up to the verdict.
/// Every servo on the chain works through the same backlog.
fn chain_backlog(frame: Frame) -> (u64, u64) {
    let ahead = 2 * budget::frame(frame);
    let own = budget::frame(Frame::GreadSlot) - Frame::REPLY_ARMS * budget::body(Body::TxDone);
    (ahead as u64, own as u64)
}

/// A GREAD whose slot 0 is absent, at 3M, the kernel above the bus at the
/// moving budget, the slot behind the worst backlog of `frame`s: slot 1
/// dispatches at the covered checkpoint, serves its verdict behind the hot
/// loop's frames stretched by the kernel, then reclaims one
/// RESPONSE_DEADLINE after it is ready. The attached host is still waiting
/// when its status arrives, flagged: its chain allowance covers the
/// backlog's longest wall time under the kernel.
fn host_waits_for_a_reclaim_behind(frame: Frame) {
    let (ahead, own) = chain_backlog(frame);
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Moving);
    let lag = lane.wall(ahead + own);
    let allowance = CHAIN_READY_LAG_US as u64 * TICKS_PER_US;
    assert!(
        lag <= allowance,
        "ready {} us after the frame at the moving budget, allowance {} us",
        lag / TICKS_PER_US,
        CHAIN_READY_LAG_US
    );
    let mut sim = Sim::new(BaudRate::B3000000);
    sim.attach_host();
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::at_budget(Frame::GreadSlot));
    sim.set_kernel_lane(s, lane, START_US + 20_000);
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
        assert!(t < START_US + 10_000, "the GREAD never dispatched");
        sim.run_until(t);
    }
    sim.preempt_before_ring_read(s, ahead.div_ceil(TICKS_PER_US));
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

#[test]
fn host_waits_for_a_reclaim_counted_from_readiness() {
    host_waits_for_a_reclaim_behind(Frame::Write);
}

/// The same with a configuration write ahead: each frame carries the
/// configuration count's pair (`budget::CONFIG_COUNT`).
#[test]
fn a_config_write_frame_fits_the_chain_and_reclaim_pins() {
    host_waits_for_a_reclaim_behind(Frame::ConfigWrite);
}

fn write_u16(addr: u16, v: u16) -> Vec<u8> {
    write_at(addr, &v.to_le_bytes())
}

fn write_at(addr: u16, data: &[u8]) -> Vec<u8> {
    let mut p = addr.to_le_bytes().to_vec();
    p.extend_from_slice(data);
    instruction(ID, Opcode::Write, 0, &p)
}

/// The six default fields, pos and current_raw to current_trough: 12 B
/// over two chunks, the longest row, a 202 B frame per 800 us batch at 3M.
const SIX: Chunks = Chunks::two(Chunk::new(tick::POS, 2), Chunk::new(tick::CURRENT_RAW, 10));
/// The ident four: pos, then duty to vdiff (current_trough riding along).
const IDENT: Chunks = Chunks::two(Chunk::new(tick::POS, 2), Chunk::new(tick::DUTY_TICKS, 8));
/// The held current and differential with the duty, and the rail
/// estimate: 10 B over two chunks, the second outside the tick block.
const IDENT_RAIL: Chunks = Chunks::two(
    Chunk::new(tick::DUTY_TICKS, 8),
    Chunk::new(estimates::VBUS_COUNTS, 2),
);
const POS: Chunks = Chunks::one(tick::POS, 2);
const TEL_FRAMES: u16 = 100;
/// Host start offsets against the kernel grid, 13 us apart across one
/// 16-tick batch (806 us), so the batches meet the SLOW stages at every
/// alignment; the worst one gates.
const STARTS: u64 = 62;

/// One burst of `TEL_FRAMES` frames of `c` at 3M, the kernel above the
/// bus at budget for `motion`, the host `k` x 13 us late: the stream
/// frames with a good CRC, and the rows dropped.
fn tel_burst(c: Chunks, motion: Motion, k: u64) -> (usize, u16) {
    let (frames, drops, _) = tel_burst_at(BaudRate::B3000000, c, motion, k);
    let stream = |f: &&WireFrame| status(f).0.result() == Some(ResultCode::Stream);
    let good = frames
        .iter()
        .filter(stream)
        .filter(|f| frame_crc_ok(f))
        .count();
    (good, drops)
}

/// [`tel_burst`] at `rate`, every servo frame from the count write on (its
/// ack first), at budget, and the kernel lane's stats.
fn tel_burst_at(
    rate: BaudRate,
    c: Chunks,
    motion: Motion,
    k: u64,
) -> (Vec<WireFrame>, u16, KernelStats) {
    let start = START_US + 13 * k;
    let mut sim = Sim::new(rate);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::at_budget(Frame::Write));
    let rows = TEL_FRAMES * STREAM_SAMPLES_MAX as u16;
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, motion);
    sim.set_kernel_lane(s, lane, start + 3 * rows as u64 * 50 + 10_000);
    sim.host_send_at(start, &write_at(CHUNK0_ADDR, &c.to_le_bytes()));
    sim.run_until(start + 2_000);
    sim.host_send_at(start + 2_000, &write_u16(TEL_COUNT, rows));
    let frames = sim
        .run()
        .into_iter()
        .filter(|f| f.from == Source::Servo(ID))
        .collect();
    (frames, sim.tel_drops(s), sim.kernel_stats(s))
}

#[test]
fn tel_six_fields_fit_at_rest_with_the_kernel_on_top() {
    for k in 0..STARTS {
        let got = tel_burst(SIX, Motion::Quiet, k);
        assert_eq!(
            got,
            (TEL_FRAMES as usize, 0),
            "frames, rows dropped: start {k}"
        );
    }
}

/// The default six fields as a 12 B row, the longest a row runs, stepping
/// at budget at 3M: every frame good, no row dropped, no body over the
/// period but a thermometer stage's (those stages are being trimmed; their
/// pins report alone).
#[test]
fn tel_twelve_byte_rows_fit_at_budget_while_stepping() {
    assert_eq!(SIX.row_bytes(), osc_servo_core::tel::ROW_BYTES_MAX);
    for k in 0..STARTS {
        let (frames, drops, st) = tel_burst_at(BaudRate::B3000000, SIX, Motion::Moving, k);
        let stream: Vec<_> = frames
            .iter()
            .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
            .collect();
        let bad = stream.iter().filter(|f| !frame_crc_ok(f)).count();
        assert_eq!(
            (stream.len(), bad, drops, st.over - st.therm_over),
            (TEL_FRAMES as usize, 0, 0, 0),
            "start {k}: frames, CRC-bad, rows dropped, bodies over the period ({} thermal)",
            st.therm_over
        );
    }
}

/// Lost ticks and late bodies, stepping at budget: every row of a good
/// frame carries one tick's table bytes, both chunks, and row 0 is the
/// tick its frame's stamp names.
#[test]
fn tel_rows_hold_one_ticks_values() {
    use osc_integration::sim::{tel_row, tel_sample};
    use osc_protocol::tel::StreamFrame;
    for k in (0..STARTS).step_by(7) {
        let start = START_US + 13 * k;
        let mut sim = Sim::new(BaudRate::B3000000);
        let s = sim.add_servo(ID);
        sim.set_handler_cost(s, HandlerCost::at_budget(Frame::Write));
        let rows = 20 * STREAM_SAMPLES_MAX as u16;
        let lane = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Moving);
        sim.set_kernel_lane(s, lane, start + 3 * rows as u64 * 50 + 10_000);
        sim.lose_tel_ticks(s, 40 + k as u32, 43 + k as u32);
        sim.host_send_at(start, &write_at(CHUNK0_ADDR, &IDENT_RAIL.to_le_bytes()));
        sim.run_until(start + 2_000);
        sim.host_send_at(start + 2_000, &write_u16(TEL_COUNT, rows));
        let frames: Vec<_> = sim
            .run()
            .into_iter()
            .filter(|f| f.from == Source::Servo(ID))
            .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
            .filter(frame_crc_ok)
            .collect();
        assert!(
            frames.len() >= 18,
            "start {k}: {} good frames",
            frames.len()
        );
        // Each synthesized tick's row is unique: a row matches at most one.
        let ticks: Vec<Vec<u8>> = (0..rows as u32 + 8)
            .map(|n| tel_row(IDENT_RAIL, &tel_sample(n), n))
            .collect();
        for f in &frames {
            let f = StreamFrame::parse(IDENT_RAIL.row_bytes(), status(f).1).expect("a frame");
            for (i, row) in f.rows().enumerate() {
                let n = ticks.iter().position(|t| t == row);
                assert!(
                    n.is_some(),
                    "start {k} seq {} row {i}: torn {row:02X?}",
                    f.seq
                );
                if i == 0 {
                    assert_eq!(n, Some(f.tick as usize), "start {k} seq {}", f.seq);
                }
            }
        }
    }
}

/// Each row the stream carries, stepping at budget: a frame the DMA reads
/// ahead of its rows' copies ships stale bytes and fails its CRC, so a
/// clean stream with nothing dropped is a lane that never underran.
#[test]
fn tel_kernel_lane_never_underruns_at_budget() {
    for c in [SIX, IDENT, IDENT_RAIL, POS] {
        assert!(c.valid());
        for k in (0..STARTS).step_by(4) {
            let (frames, drops, _) = tel_burst_at(BaudRate::B3000000, c, Motion::Moving, k);
            let stream: Vec<_> = frames
                .iter()
                .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
                .collect();
            let bad = stream.iter().filter(|f| !frame_crc_ok(f)).count();
            assert_eq!(
                (stream.len(), bad, drops),
                (TEL_FRAMES as usize, 0, 0),
                "{} B start {k}: frames, CRC-bad, rows dropped",
                c.row_bytes()
            );
        }
    }
}

/// The lane passes at the arming ack's release, so the kernel's first feed
/// can never reset the CRC engine under the ack: at budget, at every start
/// offset, the ack and the burst's frames all verify.
#[test]
fn tel_ack_crc_survives_a_burst_start() {
    for c in [SIX, POS] {
        for k in 0..STARTS {
            let (frames, _, _) = tel_burst_at(BaudRate::B3000000, c, Motion::Moving, k);
            let ack = &frames[0];
            assert_eq!(status(ack).0.result(), Some(ResultCode::Ok), "{k}");
            assert!(frame_crc_ok(ack), "start {k}: the ack");
            let first = &frames[1];
            assert_eq!(status(first).0.result(), Some(ResultCode::Stream));
            assert!(frame_crc_ok(first), "start {k}: the first frame");
        }
    }
}

/// A SLOW or thermometer stage tick holds the TX lane: at every start
/// offset, stepping at budget, the burst's stage ticks carry their rows
/// and never a lane step.
#[test]
fn tel_lane_never_steps_on_a_stage_tick() {
    for c in [SIX, IDENT_RAIL, POS] {
        for k in 0..STARTS {
            let (_, _, st) = tel_burst_at(BaudRate::B3000000, c, Motion::Moving, k);
            assert!(st.stage_tel > 0, "start {k}: no stage tick streamed");
            assert_eq!(st.stage_steps, 0, "start {k}: {st:?}");
        }
    }
}

/// The tick-load close lands on phase PUBLISH alone, never on a SLOW or
/// thermometer stage's phase: at every start offset of a burst, stepping
/// at budget.
#[test]
fn health_close_lands_only_on_publish_while_streaming() {
    for k in 0..STARTS {
        let (_, _, st) = tel_burst_at(BaudRate::B3000000, SIX, Motion::Moving, k);
        let lands: Vec<_> = (0..PHASES).filter(|&p| st.closes[p] > 0).collect();
        assert_eq!(
            lands,
            [phase::PUBLISH as usize],
            "start {k}: {:?}",
            st.closes
        );
    }
}

/// Send `frame` at `at_us` and run until the servo answers; the us it did.
fn exchange_at(sim: &mut Sim, at_us: u64, frame: &[u8]) -> u64 {
    sim.host_send_at(at_us, frame);
    let mut t = at_us;
    while !sim.run_until(t).iter().any(|f| f.from == Source::Servo(ID)) {
        t += 1;
        assert!(
            t < at_us + 10_000,
            "the frame at {at_us} us was never answered"
        );
    }
    t
}

/// The kernel's inputs in their writable runs: the span less the reserved
/// bytes behind the limits and the fault blocks.
fn span_runs() -> [(u16, u16); 3] {
    use osc_servo_core::kernel::{SPAN_END, SPAN_START};
    use osc_servo_core::regions::CALIB_BASE_ADDR;
    use osc_servo_core::regions::config::addr::{fault_cfg, limits, thermal};
    [
        (SPAN_START, limits::OPENLOOP_ZERO_BRAKE + 1),
        (thermal::DERATE_START_CC, fault_cfg::SENSOR_BAD_COUNT + 1),
        (CALIB_BASE_ADDR, SPAN_END),
    ]
}

/// The kernel takes a configuration within this many ticks of its commit:
/// the next SPARE_A is at most a medium period away, and the kernel
/// above the bus enters every tick on its scan.
const APPLY_TICKS: u64 = DECIM_MED_TICKS + 1;
const DECIM_MED_TICKS: u64 = PHASES as u64;

/// How the COMMIT that carries a configuration write and a TEL arm ends.
#[derive(Copy, Clone, Debug)]
enum ConfigArm {
    /// Addressed: the lane lends at its ack's release.
    Acked,
    /// Broadcast, as `osc` arms a step: no ack, the lane lends at the arm,
    /// with the take still pending.
    Silent,
}

/// A bulk configuration write committed with the TEL arm, at budget while
/// stepping, the host `k` x 13 us late: the TEL chunks, then the span's
/// runs and the burst's count staged with HOLD, one COMMIT ending `how`.
/// Every servo frame from the chunks' ack on, the rows dropped, and the
/// kernel lane's stats.
fn config_burst(how: ConfigArm, k: u64) -> (Vec<WireFrame>, u16, KernelStats) {
    let start = START_US + 13 * k;
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::at_budget(Frame::ConfigWrite));
    let rows = TEL_FRAMES * STREAM_SAMPLES_MAX as u16;
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Moving);
    sim.set_kernel_lane(s, lane, start + 3 * rows as u64 * 50 + 10_000);
    let mut t = exchange_at(&mut sim, start, &write_at(CHUNK0_ADDR, &SIX.to_le_bytes()));
    for (lo, hi) in span_runs() {
        let mut payload = lo.to_le_bytes().to_vec();
        payload.extend(sim.servo_table(s, |tb| {
            // SAFETY: the table is repr(C) bytes; the run lies inside it.
            let p = (tb as *const osc_servo_core::ControlTable).cast::<u8>();
            unsafe { core::slice::from_raw_parts(p.add(lo as usize), (hi - lo) as usize) }.to_vec()
        }));
        t = exchange_at(
            &mut sim,
            t + 1,
            &instruction(ID, Opcode::Write, Inst::FLAG_HOLD, &payload),
        );
    }
    let mut payload = TEL_COUNT.to_le_bytes().to_vec();
    payload.extend(rows.to_le_bytes());
    t = exchange_at(
        &mut sim,
        t + 1,
        &instruction(ID, Opcode::Write, Inst::FLAG_HOLD, &payload),
    );
    let to = match how {
        ConfigArm::Acked => ID,
        ConfigArm::Silent => Id::BROADCAST.0,
    };
    sim.host_send_at(t + 1, &instruction(to, Opcode::Commit, 0, &[]));
    let frames = sim
        .run()
        .into_iter()
        .filter(|f| f.from == Source::Servo(ID))
        .collect();
    (frames, sim.tel_drops(s), sim.kernel_stats(s))
}

/// At every start offset the kernel takes the committed set once, within
/// [`APPLY_TICKS`] of the commit, the burst it armed streaming meanwhile.
#[test]
fn config_apply_lands_within_its_bound_at_budget() {
    for k in 0..STARTS {
        let (_, _, st) = config_burst(ConfigArm::Acked, k);
        assert_eq!(st.takes, 1, "start {k}: {st:?}");
        assert!(st.apply_max <= APPLY_TICKS, "start {k}: {st:?}");
    }
}

/// A burst armed with a configuration take pending, at every start offset:
/// its first row waits out the take, at most [`APPLY_TICKS`] ticks, so no
/// tick carries the take and a row; the burst then streams whole, every
/// row charged where the feed copied one. Some offsets must hold, or the
/// pin tests nothing.
#[test]
fn a_burst_never_streams_through_a_config_take() {
    let mut held = 0;
    for (how, k) in [ConfigArm::Acked, ConfigArm::Silent]
        .into_iter()
        .flat_map(|how| (0..STARTS).map(move |k| (how, k)))
    {
        let (frames, drops, st) = config_burst(how, k);
        let stream: Vec<_> = frames
            .iter()
            .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
            .collect();
        let bad = stream.iter().filter(|f| !frame_crc_ok(f)).count();
        assert_eq!(
            (
                st.takes,
                st.take_rows,
                st.row_misses,
                st.over - st.therm_over
            ),
            (1, 0, 0, 0),
            "{how:?} start {k}: takes, rows on a take's tick, row charge misses, bodies over \
             the period ({} thermal): {st:?}",
            st.therm_over
        );
        assert_eq!(
            (stream.len(), bad, drops),
            (TEL_FRAMES as usize, 0, 0),
            "{how:?} start {k}: frames, CRC-bad, rows dropped"
        );
        assert!(st.held_starts <= APPLY_TICKS, "{how:?} start {k}: {st:?}");
        held = held.max(st.held_starts);
    }
    assert!(held > 0, "no start held");
}
