//! The DES charges the budget table (`osc_servo_core::budget`), the one the
//! bench test `budgets` holds the chip to; at budget it predicts no fewer
//! TEL rows lost than the bench measured.

use osc_integration::sim::{
    HandlerCost, KERNEL_AT_BUDGET, KERNEL_FLOOR, KernelLane, KernelLevel, Sim, Source, instruction,
    status,
};
use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::{
    self, Body, ENTRY_EXIT, Frame, KERNEL, KERNEL_MEAN, PHASES, Regime, TEL_BANK, TEL_ENCODE,
};
use osc_servo_core::regions::control::addr::lifecycle::{TEL_COUNT, TEL_MASK};

const ID: u8 = 1;
const START_US: u64 = 1_000;

/// The six-field mask of the bench soaks: a 202 B frame per 16-row batch.
const SIX_FIELDS: u16 = 0x1cd;
const BURST_ROWS: u16 = 4000;

/// The bench step soak at 3M (Position, a 1200-count step before each of
/// 20 bursts of 4000 rows): 19699 rows dropped against 80000 sent.
const BENCH_STEP_DROPPED: f64 = 19699.0 / (80000.0 + 19699.0);

fn write_u16(addr: u16, v: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let d = v.to_le_bytes();
    instruction(ID, Opcode::Write, 0, &[a[0], a[1], d[0], d[1]])
}

/// One six-field burst of `BURST_ROWS` at 3M under `lane`: the stream
/// frames and the rows dropped.
fn burst(lane: KernelLane, cost: HandlerCost) -> (usize, u16) {
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, cost);
    sim.set_kernel_lane(s, lane, START_US + 3 * BURST_ROWS as u64 * 50);
    sim.host_send_at(START_US, &write_u16(TEL_MASK, SIX_FIELDS));
    sim.run_until(START_US + 1_000);
    sim.host_send_at(START_US + 1_000, &write_u16(TEL_COUNT, BURST_ROWS));
    let frames = sim
        .run()
        .into_iter()
        .filter(|f| f.from == Source::Servo(ID) && status(f).0.result() == Some(ResultCode::Stream))
        .count();
    (frames, sim.tel_drops(s))
}

/// Every moving body at its budget, the stager and arms at theirs: the
/// DES loses at least the bench's share of rows. With the measured means
/// it predicted none.
#[test]
fn step_soak_at_budget_drops_at_least_the_bench() {
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, Regime::Moving);
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

#[test]
fn the_des_charges_the_budget_table() {
    let entry = ENTRY_EXIT as u64;
    for r in Regime::ALL {
        let i = r as usize;
        for p in 0..PHASES {
            assert_eq!(KERNEL_AT_BUDGET[i][p], KERNEL[i][p] as u64 + entry);
            let mean = (KERNEL_FLOOR[i][p] + KERNEL_AT_BUDGET[i][p]) / 2;
            assert_eq!(mean, KERNEL_MEAN[i][p] as u64 + entry, "{r:?} phase {p}");
        }
        let tel = KernelLane::at_budget(KernelLevel::AboveBus, r).tel;
        assert_eq!(
            (tel.encode, tel.bank),
            (TEL_ENCODE[i] as u64, TEL_BANK as u64)
        );
        assert_eq!(
            tel.stage,
            (budget::body(Body::TelStage) + ENTRY_EXIT) as u64
        );
        assert_eq!(tel.poll, (budget::body(Body::TelPoll) + ENTRY_EXIT) as u64);
    }
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
