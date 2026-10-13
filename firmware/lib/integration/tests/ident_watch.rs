//! The identification watch: every pause of a host drive streams TEL rows
//! of `current`, `vdiff` and `duty`, with `pos` when the line's rows hold
//! it, in bursts of whole 16-row windows up to 250 ms long, whatever
//! drives. Pinned at budget: every burst arrives whole with no row
//! dropped, at every host start offset against the kernel grid, at both
//! rates that admit a watch, parked, holding and stepping.

use osc_integration::sim::{
    HandlerCost, KernelLane, KernelLevel, Motion, Sim, Source, frame_crc_ok, instruction, status,
};
use osc_protocol::tel::{BIT_CURRENT, BIT_DUTY, BIT_POS, BIT_VDIFF, StreamFrame, row_bytes};
use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::Frame;
use osc_servo_core::regions::control::addr::lifecycle::{TEL_COUNT, TEL_MASK};
use osc_servo_core::tel::row_bytes_max;
use rstest::rstest;

const ID: u8 = 1;
const START_US: u64 = 1_000;
const WINDOW_FIELDS: u16 = BIT_CURRENT | BIT_VDIFF | BIT_DUTY;
const WITH_POS: u16 = WINDOW_FIELDS | BIT_POS;
/// Host start offsets 13 us apart across one 16-tick batch (806 us), so the
/// burst meets every kernel phase and SLOW stage alignment.
const STARTS: u64 = 62;

/// The watch's fields at `baud`, as the host picks them: the pot too when
/// the row holds it.
fn watch_mask(baud: BaudRate) -> u16 {
    [WITH_POS, WINDOW_FIELDS]
        .into_iter()
        .find(|&m| row_bytes(m) <= row_bytes_max(baud))
        .expect("2 M and 3 M carry a watch")
}

fn write_u16(addr: u16, v: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let d = v.to_le_bytes();
    instruction(ID, Opcode::Write, 0, &[a[0], a[1], d[0], d[1]])
}

/// One watch burst of `rows` at `baud`, the kernel above the bus at budget
/// for `motion`, the host `k` x 13 us late: the rows that arrived in
/// CRC-good stream frames and the rows the encoder dropped.
fn watch(baud: BaudRate, motion: Motion, rows: u16, k: u64) -> (usize, u16) {
    let mask = watch_mask(baud);
    let start = START_US + 13 * k;
    let mut sim = Sim::new(baud);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::at_budget(Frame::Write));
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, motion);
    sim.set_kernel_lane(s, lane, start + 4_000 + 3 * rows as u64 * 50);
    sim.host_send_at(start, &write_u16(TEL_MASK, mask));
    sim.run_until(start + 2_000);
    sim.host_send_at(start + 2_000, &write_u16(TEL_COUNT, rows));
    let mut got = 0;
    for f in sim.run() {
        let (inst, payload) = status(&f);
        if f.from == Source::Servo(ID)
            && inst.result() == Some(ResultCode::Stream)
            && frame_crc_ok(&f)
        {
            got += StreamFrame::parse(mask, payload).map_or(0, |p| p.len());
        }
    }
    (got, sim.tel_drops(s))
}

/// Parked or OpenLoop: a 2 ms poll's watch, a 10 ms one, a 250 ms one.
#[rstest]
fn an_open_watch_streams_whole_at_budget(
    #[values(BaudRate::B2000000, BaudRate::B3000000)] baud: BaudRate,
    #[values(32, 192, 4992)] rows: u16,
) {
    for k in 0..STARTS {
        assert_eq!(
            watch(baud, Motion::Quiet, rows, k),
            (rows as usize, 0),
            "rows, rows dropped: start {k}"
        );
    }
}

/// A closed loop at budget: a 2 ms poll's watch and a 250 ms one, the
/// longest a pause streams.
#[rstest]
fn a_closed_watch_streams_whole_at_budget(
    #[values(BaudRate::B2000000, BaudRate::B3000000)] baud: BaudRate,
    #[values(Motion::Hold, Motion::Moving)] motion: Motion,
    #[values(32, 4992)] rows: u16,
) {
    for k in 0..STARTS {
        assert_eq!(
            watch(baud, motion, rows, k),
            (rows as usize, 0),
            "rows, rows dropped: start {k}"
        );
    }
}

/// 1 M carries no watch: the host refuses it before it drives.
#[test]
fn one_megabaud_carries_no_watch() {
    assert!(row_bytes(WINDOW_FIELDS) > row_bytes_max(BaudRate::B1000000));
}
