//! The identification watch: every pause of a host drive streams TEL rows
//! of the pot, then the duty, the held current and the differential, in
//! bursts of whole 16-row windows up to 250 ms long, whatever drives.
//! Pinned at budget: every burst arrives whole with no row dropped, at
//! every host start offset against the kernel grid, parked, holding and
//! stepping; below 3M a watch never streams.

use osc_integration::sim::{
    HandlerCost, KernelLane, KernelLevel, Motion, Sim, Source, frame_crc_ok, instruction, status,
};
use osc_protocol::tel::{Chunk, Chunks, StreamFrame};
use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::Frame;
use osc_servo_core::regions::control::addr::tel::CHUNK0_ADDR;
use osc_servo_core::regions::telemetry::addr::tick::{DUTY_TICKS, POS, VDIFF_COUNTS};
use rstest::rstest;

const ID: u8 = 1;
const START_US: u64 = 1_000;
/// The watch the host arms: the pot, then duty to the differential.
const WATCH: Chunks = Chunks::two(
    Chunk::new(POS, 2),
    Chunk::new(DUTY_TICKS, VDIFF_COUNTS + 2 - DUTY_TICKS),
);
/// Host start offsets 13 us apart across one 16-tick batch (806 us), so the
/// burst meets every kernel phase and SLOW stage alignment.
const STARTS: u64 = 62;

/// One watch burst of `rows` at `baud`, the kernel above the bus at budget
/// for `motion`, the host `k` x 13 us late: the rows that arrived in
/// CRC-good stream frames and the rows the encoder dropped.
fn watch(baud: BaudRate, motion: Motion, rows: u16, k: u64) -> (usize, u16) {
    let start = START_US + 13 * k;
    let mut sim = Sim::new(baud);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::at_budget(Frame::Write));
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, motion);
    sim.set_kernel_lane(s, lane, start + 4_000 + 3 * rows as u64 * 50);
    let mut p = CHUNK0_ADDR.to_le_bytes().to_vec();
    p.extend_from_slice(&WATCH.to_le_bytes());
    p.extend_from_slice(&rows.to_le_bytes());
    sim.host_send_at(start, &instruction(ID, Opcode::Write, 0, &p));
    let mut got = 0;
    for f in sim.run() {
        let (inst, payload) = status(&f);
        if f.from == Source::Servo(ID)
            && inst.result() == Some(ResultCode::Stream)
            && frame_crc_ok(&f)
        {
            got += StreamFrame::parse(WATCH.row_bytes(), payload).map_or(0, |p| p.rows().count());
        }
    }
    (got, sim.tel_drops(s))
}

/// Parked or OpenLoop: a 2 ms poll's watch, a 10 ms one, a 250 ms one.
#[rstest]
fn an_open_watch_streams_whole_at_budget(#[values(32, 192, 4992)] rows: u16) {
    for k in 0..STARTS {
        assert_eq!(
            watch(BaudRate::B3000000, Motion::Quiet, rows, k),
            (rows as usize, 0),
            "rows, rows dropped: start {k}"
        );
    }
}

/// A closed loop at budget: a 2 ms poll's watch and a 250 ms one, the
/// longest a pause streams.
#[rstest]
fn a_closed_watch_streams_whole_at_budget(
    #[values(Motion::Hold, Motion::Moving)] motion: Motion,
    #[values(32, 4992)] rows: u16,
) {
    for k in 0..STARTS {
        assert_eq!(
            watch(BaudRate::B3000000, motion, rows, k),
            (rows as usize, 0),
            "rows, rows dropped: start {k}"
        );
    }
}

/// Below 3M the servo acks a watch's arm and never streams: the host
/// refuses a watch there before it drives.
#[test]
fn below_three_megabaud_a_watch_never_streams() {
    for baud in [BaudRate::B1000000, BaudRate::B2000000] {
        assert_eq!(watch(baud, Motion::Quiet, 32, 0), (0, 0), "{baud:?}");
    }
}
