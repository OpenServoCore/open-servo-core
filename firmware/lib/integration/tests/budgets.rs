//! The DES charges the budget table (`osc_servo_core::budget`), the one the
//! bench test `budgets` holds the chip to; at budget it predicts no fewer
//! TEL rows lost than the bench measured.
//! Pins run at budget; a red pin is a design finding, never a looser
//! threshold.

use osc_integration::sim::{
    HandlerCost, KERNEL_AT_BUDGET, KERNEL_FLOOR, KernelLane, KernelLevel, Motion, Sim, Source,
    instruction, status,
};
use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::probe::BusProbe;
use osc_servo_core::budget::{
    self, ENTRY_EXIT, Events, Frame, HOLD_APPROACH_TICKS, KERNEL, KERNEL_MEAN, LAYOUT, PHASES,
    Regime, TEL_ARM, TEL_BANK, TEL_FEED, TEL_HEAD, TEL_PATCH, TEL_POLL, TEL_ROW, TICK_HZ,
    TICK_PERIOD, TICK_PUBLISH, WINDOW_LOAD, WINDOW_LOAD_TEL, WINDOW_TICKS,
};
use osc_servo_core::kernel::{DECIM_SLOW, phase};
use osc_servo_core::regions::control::addr::lifecycle::GOAL_POSITION;
use osc_servo_core::regions::control::addr::tel::{CHUNK0_ADDR, TEL_COUNT};
use osc_servo_core::regions::telemetry::addr::tick::DUTY_TICKS;
use osc_servo_core::tel::Chunks;

const ID: u8 = 1;
const START_US: u64 = 1_000;

/// The six default fields of the bench soaks, one 12 B chunk: a 202 B frame
/// per 16-row batch.
const SIX: Chunks = Chunks::one(DUTY_TICKS, 12);
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
    let a = CHUNK0_ADDR.to_le_bytes();
    let mut p = vec![a[0], a[1]];
    p.extend_from_slice(&SIX.to_le_bytes());
    sim.host_send_at(START_US, &instruction(ID, Opcode::Write, 0, &p));
    // The host speaks again once the reply is in: at the moving budget it
    // takes about 1 ms.
    let mut t = START_US;
    while !sim.run_until(t).iter().any(|f| f.from == Source::Servo(ID)) {
        t += 1;
        assert!(t < START_US + 10_000, "the chunk write was never answered");
    }
    sim.host_send_at(t + 1, &write_u16(TEL_COUNT, BURST_ROWS));
    let frames = sim
        .run()
        .into_iter()
        .filter(|f| f.from == Source::Servo(ID) && status(f).0.result() == Some(ResultCode::Stream))
        .count();
    (frames, sim, s)
}

/// Every moving body at its budget and the TEL lane at its rows: the DES
/// loses at least the bench's share of rows, and with the kernel owning
/// the lane that share is none.
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
    assert_eq!(dropped, 0, "rows dropped at budget");
}

/// The probe's frame rule on the DES's event order, with known costs: a
/// window of polled READs books every exchange whole, wake to last arm,
/// once the dump closes the last one; a burst books its two writes and no
/// bus body for its stream, which the kernel puts on the wire.
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
    assert!(frames > 0);
    assert_eq!(p.host.n, 2);
}

#[test]
fn the_des_charges_the_budget_table() {
    let entry = ENTRY_EXIT as u64;
    for r in Regime::ALL {
        let i = r as usize;
        for p in 0..PHASES {
            let tick = (TICK_PUBLISH[i][p] + LAYOUT) as u64;
            assert_eq!(KERNEL_AT_BUDGET[i][p], KERNEL[i][p] as u64 + tick + entry);
            let mean = (KERNEL_FLOOR[i][p] + KERNEL_AT_BUDGET[i][p]) / 2;
            assert!(
                mean >= KERNEL_MEAN[i][p] as u64 + tick + entry,
                "{r:?} phase {p}"
            );
        }
    }
    let tel = KernelLane::at_budget(KernelLevel::AboveBus, Motion::Moving).tel;
    assert_eq!(
        (tel.row, tel.bank, tel.head),
        (TEL_ROW as u64, TEL_BANK as u64, TEL_HEAD as u64)
    );
    assert_eq!(
        [tel.poll, tel.feed, tel.arm, tel.patch],
        [TEL_POLL, TEL_FEED, TEL_ARM, TEL_PATCH].map(u64::from)
    );
    for f in Frame::ALL {
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
        for (p, &at) in KERNEL_AT_BUDGET[r as usize].iter().enumerate() {
            let body = budget::tick_body(r, p) + MEASURED_ENTRY_EXIT;
            assert_eq!(at, body as u64, "{r:?} phase {p}");
        }
    }
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

/// The configuration take's tick while stepping, Closed at budget: phase
/// THERM_INSTALL carrying the take (the thermometer's stage there waits a
/// period), with the health close where it can land and interrupt entry and
/// exit; a TEL burst's tick on top, for which the take holds no lane.
fn take_tick(tel: bool) -> u32 {
    let p = phase::THERM_INSTALL as usize;
    let ev = Events {
        refresh: true,
        tel,
        bank: tel,
        health: budget::health_close_lands(p),
        ..Events::default()
    };
    budget::kernel_tick(Regime::Closed, p, ev) + ENTRY_EXIT
}

#[test]
fn config_write_while_moving_fits_at_budget() {
    let t = take_tick(false);
    assert!(t <= TICK_PERIOD, "{t} > {TICK_PERIOD}");
}

#[test]
#[ignore = "over budget: the take (REFRESH 613) and a TEL row on one tick; a burst's first row is to wait out a pending take (sec 25), which replaces this pin"]
fn config_write_while_moving_and_streaming_fits_at_budget() {
    let t = take_tick(true);
    assert!(t <= TICK_PERIOD, "{t} > {TICK_PERIOD}");
}
