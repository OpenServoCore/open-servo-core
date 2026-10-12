//! The TEL stager runs only when woken (drivers `servo_bus::tel`): by the
//! kernel when it banks a batch, and by the bus when a TX release or a
//! deadline body leaves a banked batch unblocked. Pinned at budget with
//! the kernel above the bus: the stager bodies a batch costs, and how long
//! a banked batch waits for the wire. The deadline-body wake needs a host
//! frame riding the arm's verdict, which the drivers' composite suite
//! builds (`tel_verdict_in_a_deadline_body_wakes_the_stager`).

use osc_integration::sim::{
    HandlerCost, KernelLane, KernelLevel, Motion, Sim, Source, WireFrame, instruction, status,
};
use osc_protocol::wire::{Opcode, ResultCode, STREAM_SAMPLES_MAX};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::{Frame, TICK_PERIOD};
use osc_servo_core::regions::control::addr::lifecycle::{TEL_COUNT, TEL_MASK};
use osc_servo_core::tel::BIT_POS;

const ID: u8 = 1;
const START_US: u64 = 1_000;
/// Host start offsets against the kernel grid, 13 us apart; the worst one
/// gates.
const STARTS: u64 = 16;
/// The six raw fields of the bench soaks.
const SIX_FIELDS: u16 = 0x1cd;
const BATCHES: u16 = 20;

/// A full batch's fill time, HCLK ticks. A banked batch whose wake went
/// missing waits at least this long, for its successor's bank, or for ever
/// when it is the last.
const BATCH: u64 = STREAM_SAMPLES_MAX as u64 * TICK_PERIOD as u64;

struct Burst {
    /// Servo frames from the `tel_count` write on: its ack, then the stream.
    frames: Vec<WireFrame>,
    banks: Vec<u64>,
    stage_bodies: u64,
}

impl Burst {
    fn stream(&self) -> Vec<&WireFrame> {
        self.frames
            .iter()
            .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
            .collect()
    }
}

fn write_u16(addr: u16, v: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let d = v.to_le_bytes();
    instruction(ID, Opcode::Write, 0, &[a[0], a[1], d[0], d[1]])
}

/// A burst of `rows` rows of `mask` at `rate`, the kernel above the bus at
/// budget for `motion` and every bus body at its budget, the host `k` x
/// 13 us late against the kernel grid. Each write waits for its ack.
fn burst(rate: BaudRate, mask: u16, rows: u16, motion: Motion, k: u64) -> Burst {
    let start = START_US + 13 * k;
    let mut sim = Sim::new(rate);
    let s = sim.add_servo(ID);
    sim.set_handler_cost(s, HandlerCost::at_budget(Frame::Write));
    let lane = KernelLane::at_budget(KernelLevel::AboveBus, motion);
    sim.set_kernel_lane(s, lane, start + 3_000 + 3 * rows as u64 * 50);
    sim.host_send_at(start, &write_u16(TEL_MASK, mask));
    let mut t = start;
    while !sim.run_until(t).iter().any(|f| f.from == Source::Servo(ID)) {
        t += 1;
        assert!(t < start + 10_000, "the mask write was never answered");
    }
    let before = sim.entries(s).stage;
    sim.host_send_at(t + 1, &write_u16(TEL_COUNT, rows));
    let frames = sim
        .run()
        .into_iter()
        .filter(|f| f.from == Source::Servo(ID))
        .collect();
    Burst {
        frames,
        banks: sim.tel_banks(s).to_vec(),
        stage_bodies: sim.entries(s).stage - before,
    }
}

fn six_fields(motion: Motion, k: u64) -> Burst {
    let rows = BATCHES * STREAM_SAMPLES_MAX as u16;
    burst(BaudRate::B3000000, SIX_FIELDS, rows, motion, k)
}

/// One wake per bank and at most one per TX release: main pended the
/// stager on every streaming tick, 15 bodies per batch at rest.
#[test]
fn tel_poll_bodies_per_batch_at_most_three() {
    for motion in [Motion::Quiet, Motion::Moving] {
        for k in 0..STARTS {
            let b = six_fields(motion, k);
            let batches = b.banks.len() as u64;
            assert!(batches > 0, "{motion:?} start {k}: nothing banked");
            assert!(
                b.stage_bodies <= 3 * batches,
                "{motion:?} start {k}: {} stager bodies for {batches} batches",
                b.stage_bodies
            );
        }
    }
}

/// Every batch breaks before its successor could bank, counted from its
/// own bank or from its predecessor's last byte when that frame still
/// held the wire.
#[test]
fn tel_banked_batch_never_waits_for_the_next_bank() {
    for k in 0..STARTS {
        let b = six_fields(Motion::Quiet, k);
        let stream = b.stream();
        assert_eq!(stream.len(), BATCHES as usize, "start {k}");
        assert_eq!(b.banks.len(), stream.len(), "start {k}");
        let mut free = 0;
        for (i, (f, &bank)) in stream.iter().zip(&b.banks).enumerate() {
            let wait = f.at.saturating_sub(bank.max(free));
            assert!(wait < BATCH, "start {k} batch {i}: waited {wait}");
            free = f.end;
        }
    }
}

/// The TX release wake: a one-row burst at 1M banks while the count
/// write's own ack holds the wire, so only the ack's release can stage it.
#[test]
fn tel_ack_release_stages_a_batch_banked_behind_it() {
    let mut behind = 0;
    for k in 0..STARTS {
        let b = burst(BaudRate::B1000000, BIT_POS, 1, Motion::Quiet, k);
        let stream = b.stream();
        assert_eq!(stream.len(), 1, "start {k}: the banked batch never left");
        let ack = &b.frames[0];
        assert_eq!(status(ack).0.result(), Some(ResultCode::Ok), "start {k}");
        let bank = b.banks[0];
        behind += (bank < ack.end) as u32;
        let wait = stream[0].at.saturating_sub(bank.max(ack.end));
        assert!(wait < BATCH, "start {k}: waited {wait}");
    }
    assert!(behind > 0, "no start banked behind the ack");
}
