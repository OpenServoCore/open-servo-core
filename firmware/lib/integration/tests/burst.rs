//! Shunt-burst table surface over the wire: the arm/page write rules in
//! CONTROL and the RO paged readback in BURST. The capture mechanism itself is
//! chip-side (DMA + ADC), so what the sim pins here is the ABI a host drives it
//! through -- addresses, access, the reply ceiling, and the atomic arm.

use osc_integration::sim::{Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Id, Inst, Opcode, ResultCode};
use osc_servo_core::regions::BURST_BASE_ADDR;
use osc_servo_core::regions::burst::addr::window::{CHANS_ECHO, FRAME_LEN, SAMPLES_LEN};
use osc_servo_core::regions::burst::{
    PAGE_SAMPLES, PAGES, chans, dir, frame_len, page_span, state,
};
use osc_servo_core::regions::control::addr::burst::{ARM, CHANS, DUTY_Q15, PAGE};
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{matrix, sim};

const ID5: u8 = 5;

/// One READ from the section base carries the page and the whole header.
const HEADER_AND_PAGE: u16 = 252;

fn servo_frames(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

fn sole_reply(frames: &[WireFrame]) -> &WireFrame {
    let replies = servo_frames(frames);
    assert_eq!(replies.len(), 1, "expected one servo reply: {frames:#?}");
    assert_valid(replies[0]);
    replies[0]
}

fn read_args(addr: u16, count: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let c = count.to_le_bytes();
    vec![a[0], a[1], c[0], c[1]]
}

fn write_args(addr: u16, data: &[u8]) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let mut p = vec![a[0], a[1]];
    p.extend_from_slice(data);
    p
}

#[apply(matrix)]
fn burst_duty_over_duty_max_is_validation(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);
    // The default ceiling is full scale, so nothing an i16 carries could
    // exceed it; lower it first, the way an identification run would.
    sim.servo_table_mut(s, |t| t.config.loop_current.duty_max_q15 = 1000);

    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        0,
        &write_args(DUTY_Q15, &2000i16.to_le_bytes()),
    ));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Validation));
    assert_eq!(sim.servo_table(s, |t| t.control.burst.duty_q15), 0);

    // The rule is |duty| <= duty_max: the negative step rejects the same way.
    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        0,
        &write_args(DUTY_Q15, &(-2000i16).to_le_bytes()),
    ));
    let (inst, _) = status(sole_reply(&sim.run()));
    assert_eq!(inst.result(), Some(ResultCode::Validation));

    // And a step inside the ceiling lands.
    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        0,
        &write_args(DUTY_Q15, &(-1000i16).to_le_bytes()),
    ));
    let (inst, _) = status(sole_reply(&sim.run()));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(sim.servo_table(s, |t| t.control.burst.duty_q15), -1000);
}

#[apply(matrix)]
fn burst_chans_past_the_extras_is_validation(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // Bit 3 names no channel; the field rule rejects it before the arm gate
    // ever sees it.
    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        0,
        &write_args(CHANS, &[chans::ALL + 1]),
    ));
    let (inst, _) = status(sole_reply(&sim.run()));
    assert_eq!(inst.result(), Some(ResultCode::Validation));
    assert_eq!(sim.servo_table(s, |t| t.control.burst.chans), 0);

    // The byte behind it is alignment, reserved like the tail it came from.
    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        0,
        &write_args(CHANS + 1, &[0]),
    ));
    let (inst, _) = status(sole_reply(&sim.run()));
    assert_eq!(inst.result(), Some(ResultCode::Access));

    // Every legal mask lands.
    for mask in 0..=chans::ALL {
        sim.host_send(&instruction(
            ID5,
            Opcode::Write,
            0,
            &write_args(CHANS, &[mask]),
        ));
        let (inst, _) = status(sole_reply(&sim.run()));
        assert_eq!(inst.result(), Some(ResultCode::Ok), "chans {mask}");
        assert_eq!(sim.servo_table(s, |t| t.control.burst.chans), mask);
    }
}

#[apply(matrix)]
fn burst_window_rejects_host_writes(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);
    sim.servo_table_mut(s, |t| t.burst.window.state = state::DONE);

    // Every byte of the section is chip-owned; a host write is an access fault,
    // not a silent no-op.
    for addr in [
        BURST_BASE_ADDR,
        BURST_BASE_ADDR + 1,
        BURST_BASE_ADDR + 2,
        CHANS_ECHO,
        FRAME_LEN,
    ] {
        sim.host_send(&instruction(ID5, Opcode::Write, 0, &write_args(addr, &[7])));
        let (inst, _) = status(sole_reply(&sim.run()));
        assert_eq!(inst.result(), Some(ResultCode::Access), "addr {addr:#x}");
    }
    assert_eq!(sim.servo_table(s, |t| t.burst.window.state), state::DONE);
}

#[apply(matrix)]
fn one_read_returns_the_page_and_the_header(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);
    sim.servo_table_mut(s, |t| {
        let w = &mut t.burst.window;
        w.page_echo = 3;
        w.state = state::DONE;
        w.samples_len = (PAGES * PAGE_SAMPLES) as u16;
        w.step_index = 481;
        w.start_cnt = 640;
        w.pwm_arr = 1200;
        w.start_dir = dir::DOWN;
        w.restore_dir = dir::DOWN;
        for (i, s) in w.samples.iter_mut().enumerate() {
            *s = 0x0400 + i as u16;
        }
    });

    sim.host_send(&instruction(
        ID5,
        Opcode::Read,
        0,
        &read_args(BURST_BASE_ADDR, HEADER_AND_PAGE),
    ));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(payload.len(), HEADER_AND_PAGE as usize);
    assert_eq!(
        payload[0], 3,
        "page_echo leads, so the snapshot sees it first"
    );
    assert_eq!(payload[1], state::DONE);
    for i in 0..PAGE_SAMPLES {
        let code = u16::from_le_bytes([payload[2 + 2 * i], payload[3 + 2 * i]]);
        assert_eq!(code, 0x0400 + i as u16, "sample {i}");
    }
    let tail = &payload[2 + 2 * PAGE_SAMPLES..];
    assert_eq!(u16::from_le_bytes([tail[0], tail[1]]), 960);
    assert_eq!(u16::from_le_bytes([tail[2], tail[3]]), 481);
    assert_eq!(u16::from_le_bytes([tail[4], tail[5]]), 640);
    assert_eq!(u16::from_le_bytes([tail[6], tail[7]]), 1200);
    // The geometry witness: DOWN means the crest scan landed second.
    assert_eq!((tail[8], tail[9]), (dir::DOWN, dir::DOWN));
}

/// The frame words sit one past the page READ, so a host fetches them with
/// the header: `samples_len` through `frame_len` in one READ.
#[apply(matrix)]
fn header_read_carries_the_frame_words(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);
    let mask = chans::VMOTOR_A | chans::VBUS;
    sim.servo_table_mut(s, |t| {
        let w = &mut t.burst.window;
        w.samples_len = (PAGES * PAGE_SAMPLES) as u16;
        w.step_index = 480;
        w.restore_dir = dir::DOWN;
        w.chans_echo = mask;
        w.frame_len = frame_len(mask);
    });

    let count = FRAME_LEN + 1 - SAMPLES_LEN;
    sim.host_send(&instruction(
        ID5,
        Opcode::Read,
        0,
        &read_args(SAMPLES_LEN, count),
    ));
    let frames = sim.run();
    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(payload.len(), count as usize);
    assert_eq!(u16::from_le_bytes([payload[0], payload[1]]), 960);
    assert_eq!(u16::from_le_bytes([payload[2], payload[3]]), 480);
    assert_eq!(payload[9], dir::DOWN);
    assert_eq!((payload[10], payload[11]), (mask, 3));
}

#[apply(matrix)]
fn read_past_the_window_is_limit(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    // 253 is one past the frame ceiling (sec 5.1), which is exactly what the
    // section geometry is sized against.
    sim.host_send(&instruction(
        ID5,
        Opcode::Read,
        0,
        &read_args(BURST_BASE_ADDR, HEADER_AND_PAGE + 1),
    ));
    let (inst, _) = status(sole_reply(&sim.run()));
    assert_eq!(inst.result(), Some(ResultCode::Limit));
}

#[apply(matrix)]
fn step_and_arm_commit_in_one_instant(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // The step target, the frame, and the arm must never be live apart: a
    // committed arm over a stale duty or mask would capture the wrong run.
    for (addr, data) in [
        (DUTY_Q15, 8520i16.to_le_bytes().to_vec()),
        (CHANS, vec![chans::VBUS]),
        (ARM, vec![1]),
    ] {
        sim.host_send(&instruction(
            ID5,
            Opcode::Write,
            Inst::FLAG_HOLD,
            &write_args(addr, &data),
        ));
        let (inst, _) = status(sole_reply(&sim.run()));
        assert_eq!(inst.result(), Some(ResultCode::Ok));
    }
    let live = |t: &osc_servo_core::regions::ControlTable| {
        let b = &t.control.burst;
        (b.duty_q15, b.chans, b.arm)
    };
    assert_eq!(
        sim.servo_table(s, live),
        (0, 0, 0),
        "staged writes stay off the live fields"
    );

    sim.host_send(&instruction(Id::BROADCAST.0, Opcode::Commit, 0, &[]));
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());
    assert_eq!(sim.servo_table(s, live), (8520, chans::VBUS, 1));
}

#[apply(matrix)]
fn page_walk_selects_every_page(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    for page in 0..PAGES as u8 {
        sim.host_send(&instruction(
            ID5,
            Opcode::Write,
            0,
            &write_args(PAGE, &[page]),
        ));
        let (inst, _) = status(sole_reply(&sim.run()));
        assert_eq!(inst.result(), Some(ResultCode::Ok));
        assert_eq!(sim.servo_table(s, |t| t.control.burst.page), page);
        assert!(page_span(page).is_some());
    }

    // Past the last page the chip publishes nothing, so `page_echo` never
    // matches and the host's own reject is what stops the walk.
    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        0,
        &write_args(PAGE, &[PAGES as u8]),
    ));
    let (inst, _) = status(sole_reply(&sim.run()));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(page_span(PAGES as u8).is_none());
}
