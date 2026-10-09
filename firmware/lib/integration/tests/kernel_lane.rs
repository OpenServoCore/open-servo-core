//! The kernel lane (`sim::cpu`): the 20 kHz kernel tick as a periodic
//! preemptor below or above the bus vectors, held against the bench's
//! lost-tick counts for polled reads (1.33-1.56 per frame with the kernel
//! below the bus).

use osc_integration::sim::{
    KERNEL_HOLD, KERNEL_QUIET, KernelLane, KernelLevel, KernelStats, READ32_3M_COST, Sim, Source,
    instruction,
};
use osc_protocol::wire::Opcode;
use osc_servo_core::BaudRate;
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
fn kernel_lane_above_bus_loses_nothing(
    #[values(&KERNEL_QUIET, &KERNEL_HOLD)] phases: &'static [u64; 10],
    #[values(50, 140, 280)] frames_per_s: u64,
) {
    let (st, replies) = polled_reads(KernelLevel::AboveBus, phases, frames_per_s);
    assert_eq!(replies, FRAMES);
    assert_eq!((st.lost, st.entries), (0, st.scans), "{st:?}");
    assert_eq!(st.entry_latency_max, 0, "every entry at its scan: {st:?}");
    assert!(st.bus_stretch > 0, "the lane preempted no bus body: {st:?}");
}
