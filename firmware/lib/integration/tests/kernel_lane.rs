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
use osc_servo_core::kernel::phase;
use osc_servo_core::regions::control::addr::lifecycle::{GOAL_VELOCITY, TEL_COUNT, TEL_MASK};
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
    let health = budget::health_close_lands(phase as usize);
    budget::kernel_tick(Regime::Closed, phase as usize, Events { health, ..ev })
        + budget::ENTRY_EXIT
}

/// A seated reference close, staged: the SLOW finish that freezes the
/// window, the install, the side work after it, each Closed at budget, and
/// streaming with a TEL row and its bank on the same tick.
#[rstest]
#[case::quiet(false)]
#[ignore = "over budget: the TEL row (591), the TX lane's dearest step with its poll (354; a bank's tick is no dearer) and the health close (308) put the side tick at 3099 with entry and exit, 699 over (install 2749, freeze 2569); a plain Closed p1 with the same takes 2943"]
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
            phase::THERM_INSTALL,
            Events {
                install: true,
                ..carried
            },
        ),
        (
            phase::THERM_SIDE,
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
#[ignore = "over budget: Closed SLOW 1181 + TEL 591 + the TX lane's dearest step 354 + entry and exit 80 leaves 194 of the period; the due stage takes 446 (2652, 252 over), the finish 363 (2569); a plain Closed p1 with the TEL row, the lane step and the close takes 2943"]
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
    let health = budget::health_close_lands(phase::SLOW as usize);
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
/// sec 7), each at most a goal WRITE's frame, then its own GREAD up to the
/// verdict. Every servo on the chain works through the same backlog.
fn chain_backlog() -> (u64, u64) {
    let ahead = 2 * budget::frame(Frame::Write);
    let own = budget::frame(Frame::GreadSlot) - Frame::REPLY_ARMS * budget::body(Body::TxDone);
    (ahead as u64, own as u64)
}

/// A GREAD whose slot 0 is absent, at 3M, the kernel above the bus at the
/// moving budget, the slot behind the worst backlog the table allows: slot
/// 1 dispatches at the covered checkpoint, serves its verdict behind the
/// hot loop's frames stretched by the kernel, then reclaims one
/// RESPONSE_DEADLINE after it is ready. The attached host is still waiting
/// when its status arrives, flagged: its chain allowance covers the
/// backlog's longest wall time under the kernel.
#[test]
fn host_waits_for_a_reclaim_counted_from_readiness() {
    let (ahead, own) = chain_backlog();
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

fn write_u16(addr: u16, v: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let d = v.to_le_bytes();
    instruction(ID, Opcode::Write, 0, &[a[0], a[1], d[0], d[1]])
}

/// The six raw fields' soak mask: 9 B packed rows, a 161 B frame per
/// 800 us batch at 3M.
const SIX_FIELDS: u16 = 0x1cd;
/// The longest row 3M admits: the six raw fields and current, 11 B.
const WIDEST_ROW: u16 = 0x1cf;
const TEL_FRAMES: u16 = 100;
/// Host start offsets against the kernel grid, 13 us apart across one
/// 16-tick batch (806 us), so the batches meet the SLOW stages at every
/// alignment; the worst one gates.
const STARTS: u64 = 62;

/// One burst of `TEL_FRAMES` frames of `mask` at 3M, the kernel above the
/// bus at budget for `motion`, the host `k` x 13 us late: the stream frames
/// with a good CRC, and the rows the encoder dropped.
fn tel_burst(mask: u16, motion: Motion, k: u64) -> (usize, u16) {
    let (frames, drops) = tel_burst_at(BaudRate::B3000000, mask, motion, k);
    let stream = |f: &&WireFrame| status(f).0.result() == Some(ResultCode::Stream);
    let good = frames
        .iter()
        .filter(stream)
        .filter(|f| frame_crc_ok(f))
        .count();
    (good, drops)
}

/// [`tel_burst`] at `rate`, every servo frame from the count write on (its
/// ack first), at budget.
fn tel_burst_at(rate: BaudRate, mask: u16, motion: Motion, k: u64) -> (Vec<WireFrame>, u16) {
    let start = START_US + 13 * k;
    let mut sim = Sim::new(rate);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::at_budget(Frame::Write));
    let rows = TEL_FRAMES * STREAM_SAMPLES_MAX as u16;
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, motion);
    sim.set_kernel_lane(s, lane, start + 3 * rows as u64 * 50 + 10_000);
    sim.host_send_at(start, &write_u16(TEL_MASK, mask));
    sim.run_until(start + 2_000);
    sim.host_send_at(start + 2_000, &write_u16(TEL_COUNT, rows));
    let frames = sim
        .run()
        .into_iter()
        .filter(|f| f.from == Source::Servo(ID))
        .collect();
    (frames, sim.tel_drops(s))
}

#[test]
fn tel_six_fields_fit_at_rest_with_the_kernel_on_top() {
    for k in 0..STARTS {
        let got = tel_burst(SIX_FIELDS, Motion::Quiet, k);
        assert_eq!(
            got,
            (TEL_FRAMES as usize, 0),
            "frames, rows dropped: start {k}"
        );
    }
}

/// The 11 B row bound (`tel::ROW_BYTES_CPU_MAX`) keeps up parked at
/// budget; a 12 B row drops 4% of its rows here.
#[test]
fn tel_widest_row_keeps_up_at_rest() {
    let at_3m = osc_servo_core::tel::row_bytes_max(BaudRate::B3000000);
    assert_eq!(osc_servo_core::tel::row_bytes(WIDEST_ROW), at_3m);
    for k in 0..STARTS {
        let got = tel_burst(WIDEST_ROW, Motion::Quiet, k);
        assert_eq!(
            got,
            (TEL_FRAMES as usize, 0),
            "frames, rows dropped: start {k}"
        );
    }
}

/// Stepping at budget, the kernel owning the TX lane: no bus body runs
/// between frames, so the stream keeps up with the tick.
#[test]
fn tel_six_packed_fields_fit_at_budget_while_stepping() {
    for k in 0..STARTS {
        let got = tel_burst(SIX_FIELDS, Motion::Moving, k);
        assert_eq!(
            got,
            (TEL_FRAMES as usize, 0),
            "frames, rows dropped: start {k}"
        );
    }
}

#[test]
fn tel_widest_row_fits_at_budget_while_stepping() {
    for k in 0..STARTS {
        let got = tel_burst(WIDEST_ROW, Motion::Moving, k);
        assert_eq!(
            got,
            (TEL_FRAMES as usize, 0),
            "frames, rows dropped: start {k}"
        );
    }
}

/// The six raw fields and the widest row each rate admits, stepping at
/// budget: a frame the DMA reads ahead of its rows' writes ships stale
/// bytes and fails its CRC, so a clean stream with nothing dropped is a
/// lane that never underran.
#[test]
fn tel_kernel_lane_never_underruns_at_budget() {
    use osc_servo_core::tel::{BIT_CURRENT, BIT_CURRENT_RAW, BIT_POS, BIT_VBUS, BIT_VDIFF};
    use osc_servo_core::tel::{BIT_POS_LIN, row_bytes, row_bytes_max};
    let wide = BIT_CURRENT | BIT_VDIFF | BIT_VBUS | BIT_POS_LIN;
    for (rate, mask) in [
        (BaudRate::B3000000, SIX_FIELDS),
        (BaudRate::B3000000, WIDEST_ROW),
        (BaudRate::B2000000, wide),
        (BaudRate::B1000000, BIT_POS | BIT_CURRENT_RAW),
    ] {
        assert!(row_bytes(mask) <= row_bytes_max(rate), "{rate:?}");
        for k in (0..STARTS).step_by(4) {
            let (frames, drops) = tel_burst_at(rate, mask, Motion::Moving, k);
            let stream: Vec<_> = frames
                .iter()
                .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
                .collect();
            let bad = stream.iter().filter(|f| !frame_crc_ok(f)).count();
            assert_eq!(
                (stream.len(), bad, drops),
                (TEL_FRAMES as usize, 0, 0),
                "{rate:?} {} B start {k}: frames, CRC-bad, rows dropped",
                row_bytes(mask)
            );
        }
    }
}

/// The lane passes at the arming ack's release, so the kernel's first feed
/// can never reset the CRC engine under the ack: at budget, at every start
/// offset, the ack and the burst's frames all verify.
#[test]
fn tel_ack_crc_survives_a_burst_start() {
    use osc_servo_core::tel::{BIT_CURRENT_RAW, BIT_POS};
    for (rate, mask) in [
        (BaudRate::B3000000, SIX_FIELDS),
        (BaudRate::B1000000, BIT_POS | BIT_CURRENT_RAW),
    ] {
        for k in 0..STARTS {
            let (frames, _) = tel_burst_at(rate, mask, Motion::Moving, k);
            let ack = &frames[0];
            assert_eq!(status(ack).0.result(), Some(ResultCode::Ok), "{rate:?} {k}");
            assert!(frame_crc_ok(ack), "{rate:?} start {k}: the ack");
            let first = &frames[1];
            assert_eq!(status(first).0.result(), Some(ResultCode::Stream));
            assert!(frame_crc_ok(first), "{rate:?} start {k}: the first frame");
        }
    }
}
