//! The DES charges the budget table (`osc_servo_core::budget`), the one the
//! bench test `budgets` holds the chip to; at budget it predicts no fewer
//! TEL rows lost than the bench measured.

use osc_integration::sim::{
    HandlerCost, KERNEL_AT_BUDGET, KERNEL_FLOOR, KernelLane, KernelLevel, Motion, Sim, Source,
    instruction, status,
};
use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::probe::BusProbe;
use osc_servo_core::budget::{
    self, Body, ENTRY_EXIT, Events, Frame, HOLD_APPROACH_TICKS, KERNEL, KERNEL_MEAN, PHASES,
    Regime, TEL_BANK, TEL_ENCODE, TICK_HZ, TICK_PERIOD, WINDOW_LOAD, WINDOW_LOAD_TEL, WINDOW_TICKS,
};
use osc_servo_core::kernel::{DECIM_SLOW, phase};
use osc_servo_core::regions::control::addr::lifecycle::{GOAL_POSITION, TEL_COUNT, TEL_MASK};

const ID: u8 = 1;
const START_US: u64 = 1_000;

/// The six raw fields of the bench soaks: a 161 B frame per 16-row batch.
const SIX_FIELDS: u16 = 0x1cd;
const BURST_ROWS: u16 = 4000;

/// The bench step soak at 3M (Position, a 1200-count step before each of
/// 20 bursts of 4000 rows, six fields packed) dropped no rows against 80000
/// sent.
const BENCH_STEP_DROPPED: f64 = 0.0 / (80000.0 + 0.0);

/// Interrupt entry and exit in production form, HCLK ticks: trampoline and
/// prologue to the entry stamp 37, exit stamp to the resumed instruction 43
/// (interrupt timing images, two runs, min == max).
const MEASURED_ENTRY_EXIT: u32 = 80;

const TICK_US: u64 = 1_000_000 / TICK_HZ as u64;

fn write_u16(addr: u16, v: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let d = v.to_le_bytes();
    instruction(ID, Opcode::Write, 0, &[a[0], a[1], d[0], d[1]])
}

/// One six-field burst of `BURST_ROWS` at 3M under `lane`: the stream
/// frames and the rows dropped.
fn burst(lane: KernelLane, cost: HandlerCost) -> (usize, u16) {
    let (frames, sim, s) = burst_sim(lane, cost);
    (frames, sim.tel_drops(s))
}

/// [`burst`]'s stream frames and the servo's bus probe.
fn burst_probe(lane: KernelLane, cost: HandlerCost) -> (usize, BusProbe) {
    let (frames, sim, s) = burst_sim(lane, cost);
    (frames, sim.bus_probe(s))
}

fn burst_sim(lane: KernelLane, cost: HandlerCost) -> (usize, Sim, usize) {
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, cost);
    sim.set_kernel_lane(s, lane, START_US + 3 * BURST_ROWS as u64 * 50);
    sim.host_send_at(START_US, &write_u16(TEL_MASK, SIX_FIELDS));
    // The host speaks again once the reply is in: at the moving budget it
    // takes about 1 ms.
    let mut t = START_US;
    while !sim.run_until(t).iter().any(|f| f.from == Source::Servo(ID)) {
        t += 1;
        assert!(t < START_US + 10_000, "the mask write was never answered");
    }
    sim.host_send_at(t + 1, &write_u16(TEL_COUNT, BURST_ROWS));
    let frames = sim
        .run()
        .into_iter()
        .filter(|f| f.from == Source::Servo(ID) && status(f).0.result() == Some(ResultCode::Stream))
        .count();
    (frames, sim, s)
}

/// Every moving body at its budget, the stager and arms at theirs: the
/// DES loses at least the bench's share of rows. With the measured means
/// it predicted none.
#[test]
fn step_soak_at_budget_drops_at_least_the_bench() {
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Moving);
    let (frames, dropped) = burst(lane, HandlerCost::at_budget(Frame::Write));
    let share = dropped as f64 / (BURST_ROWS + dropped) as f64;
    assert!(frames > 0, "no stream at all");
    assert!(
        share >= BENCH_STEP_DROPPED,
        "{:.1}% dropped at budget, bench {:.1}%",
        100.0 * share,
        100.0 * BENCH_STEP_DROPPED
    );
}

/// The probe's frame rule on the DES's event order, with known costs: a
/// window of polled READs books every exchange whole, wake to last arm,
/// once the dump closes the last one; a burst books one TEL frame per
/// stream frame.
#[test]
fn the_probe_books_every_frame_whole() {
    const POLLS: u64 = 20;
    const GAP_US: u64 = 1_000;
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    let cost = HandlerCost::at_budget(Frame::Read32);
    sim.set_handler_cost(s, cost);
    for k in 0..POLLS {
        let at = START_US + k * GAP_US;
        sim.host_send_at(at, &instruction(ID, Opcode::Read, 0, &[0, 0, 32, 0]));
        sim.run_until(at + GAP_US - 1);
    }
    let p = sim.bus_probe(s);
    assert_eq!(u64::from(p.host.n), POLLS - 1, "open until the dump");
    let p = p.closed();
    assert_eq!(u64::from(p.host.n), POLLS);
    assert_eq!(u64::from(p.host.max), cost.frame_ticks());
    assert_eq!(u64::from(p.host.sum), POLLS * cost.frame_ticks());

    let lane = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Quiet);
    let (frames, p) = burst_probe(lane, HandlerCost::at_budget(Frame::Write));
    let p = p.closed();
    assert_eq!((p.host.n, p.tel.n as usize), (2, frames));
}

#[test]
fn the_des_charges_the_budget_table() {
    let entry = ENTRY_EXIT as u64;
    for r in Regime::ALL {
        let i = r as usize;
        for p in 0..PHASES {
            assert_eq!(KERNEL_AT_BUDGET[i][p], KERNEL[i][p] as u64 + entry);
            let mean = (KERNEL_FLOOR[i][p] + KERNEL_AT_BUDGET[i][p]) / 2;
            assert!(mean >= KERNEL_MEAN[i][p] as u64 + entry, "{r:?} phase {p}");
        }
    }
    let tel = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Moving).tel;
    assert_eq!(
        (tel.encode, tel.bank),
        (TEL_ENCODE.map(u64::from), TEL_BANK as u64)
    );
    assert_eq!(
        tel.stage,
        (budget::body(Body::TelStage) + ENTRY_EXIT) as u64
    );
    assert_eq!(tel.poll, (budget::body(Body::TelPoll) + ENTRY_EXIT) as u64);
    for f in Frame::ALL {
        if f == Frame::TelBatch {
            continue;
        }
        let at = HandlerCost::at_budget(f).frame_ticks();
        let typical = HandlerCost::typical(f).frame_ticks();
        assert!(at >= budget::frame(f) as u64 + 4 * entry, "{f:?} {at}");
        assert!(
            typical >= budget::frame_mean(f) as u64 + 4 * entry,
            "{f:?} {typical}"
        );
    }
}

#[test]
fn the_des_charges_the_measured_entry_exit() {
    for r in Regime::ALL {
        for p in 0..PHASES {
            let body = KERNEL[r as usize][p] + MEASURED_ENTRY_EXIT;
            assert_eq!(
                KERNEL_AT_BUDGET[r as usize][p], body as u64,
                "{r:?} phase {p}"
            );
        }
    }
    let poll = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Quiet)
        .tel
        .poll;
    let body = budget::body(Body::TelPoll) + MEASURED_ENTRY_EXIT;
    assert_eq!(poll, body as u64);
}

/// The kernel lane's own gamma(k) at budget against the bench's for each
/// regime: over two SLOW cycles with no traffic, and through a six-field
/// burst. A k the bench has not measured fails until a fill records it.
#[test]
fn window_load_at_budget_is_never_below_bench() {
    let ticks = 2 * DECIM_SLOW as u64 * PHASES as u64;
    let mut short = Vec::new();
    for (r, motion) in [
        (Regime::Open, Motion::Quiet),
        (Regime::Closed, Motion::Moving),
    ] {
        let lane = KernelLane::at_budget(KernelLevel::AboveBus, motion);
        let mut sim = Sim::new(BaudRate::B3000000);
        let s = sim.add_servo(ID);
        sim.set_kernel_lane(s, lane, START_US + ticks * TICK_US);
        sim.run();
        let quiet = sim.kernel_stats(s).windows.gamma;
        let (_, sim, s) = burst_sim(lane, HandlerCost::at_budget(Frame::Write));
        let tel = sim.kernel_stats(s).windows.gamma;
        for (des, bench, what) in [
            (quiet, WINDOW_LOAD[r as usize], ""),
            (tel, WINDOW_LOAD_TEL[r as usize], " TEL"),
        ] {
            for ((k, des), bench) in WINDOW_TICKS.into_iter().zip(des).zip(bench) {
                if bench == 0 || des < bench {
                    short.push(format!("{r:?}{what} gamma({k}): DES {des}, bench {bench}"));
                }
            }
        }
    }
    assert!(short.is_empty(), "{short:#?}");
}

/// A hold charges the Closed rows for the approach after the lane starts
/// and again after the goal moves, the Open rows once parked.
#[test]
fn hold_lane_is_closed_for_the_approach_then_open() {
    let approach = HOLD_APPROACH_TICKS as u64;
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Hold);
    sim.set_kernel_lane(s, lane, START_US + 4 * approach * TICK_US);
    let parked = START_US + 2 * approach * TICK_US;
    sim.run_until(parked);
    let st = sim.kernel_stats(s);
    assert_eq!(st.closed, approach);
    assert!(st.entries > 3 * approach / 2, "{st:?}");
    let goal = sim.servo_table(s, |t| t.control.lifecycle.goal_position) + 100;
    let a = GOAL_POSITION.to_le_bytes();
    let d = goal.to_le_bytes();
    let write = instruction(ID, Opcode::Write, 0, &[a[0], a[1], d[0], d[1], d[2], d[3]]);
    sim.host_send_at(parked, &write);
    sim.run();
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_position),
        goal
    );
    assert_eq!(sim.kernel_stats(s).closed, 2 * approach);
}

/// The spare phases while stepping, Closed at budget: each carries a
/// refresh stage or, in its place, the thermometer's stage, whichever is
/// dearer, with the health close where it can land and interrupt entry and
/// exit; TEL ticks of a burst on top.
fn spare_phase_ticks(tel: bool) -> [(u8, u32); 2] {
    let carried = Events {
        tel,
        bank: tel,
        ..Events::default()
    };
    let refresh = Events {
        refresh: true,
        ..carried
    };
    let install = Events {
        install: true,
        ..carried
    };
    let side = Events {
        side: true,
        ..carried
    };
    [(phase::THERM_INSTALL, install), (phase::THERM_SIDE, side)].map(|(p, therm)| {
        let health = budget::health_close_lands(p as usize);
        let tick = |ev| budget::kernel_tick(Regime::Closed, p as usize, Events { health, ..ev });
        (p, tick(refresh).max(tick(therm)) + ENTRY_EXIT)
    })
}

#[test]
fn config_write_while_moving_fits_at_budget() {
    for (p, t) in spare_phase_ticks(false) {
        assert!(t <= TICK_PERIOD, "phase {p}: {t} > {TICK_PERIOD}");
    }
}

#[test]
#[ignore = "over budget: Closed p9 with the side stage (639), the TEL row (591), its bank (198), the health close (308) and entry and exit takes 2943, 543 over; p8 with the install 2593, 193 over"]
fn config_write_while_moving_and_streaming_fits_at_budget() {
    for (p, t) in spare_phase_ticks(true) {
        assert!(t <= TICK_PERIOD, "phase {p}: {t} > {TICK_PERIOD}");
    }
}
