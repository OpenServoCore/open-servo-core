//! Host-in-the-loop: the production `osc_host` engine scheduling the
//! production servo stack over the sim wire -- both grid columns closed
//! end-to-end with no scripted host bytes. Assertions read decoded events
//! and table state (spec style), not raw hex.

use osc_host::engine::{Command, Outcome};
use osc_integration::sim::{HOST_MARK_LAG_US, HostEvent, Sim, Source, assert_valid};
use osc_protocol::build;
use osc_protocol::wire::{BaudRate, Id, Inst, Opcode, ResultCode, UID_LEN};
use osc_servo_core::regions::control::addr::lifecycle::GOAL_VELOCITY;
use osc_servo_core::regions::control::addr::tel::{CHUNK0_ADDR, TEL_COUNT};
use osc_servo_core::regions::telemetry::addr::tick::I_TICK_COUNTS;
use osc_servo_core::tel::Chunks;

const ID5: u8 = 5;

fn ping(id: u8) -> Command<'static> {
    Command::Exchange {
        id: Id::new(id),
        inst: Inst::instruction(Opcode::Ping, 0),
        payload: &[],
    }
}

fn status_result(inst: u8) -> Option<ResultCode> {
    Inst(inst).result()
}

#[test]
fn ping_round_trips_through_the_engine() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.attach_host();
    sim.add_servo(ID5);

    sim.host_submit(ping(ID5)).unwrap();
    let frames = sim.run();

    // Both wire frames are real and valid: the engine's instruction and the
    // servo's status.
    assert_eq!(frames.len(), 2, "{frames:#?}");
    assert_eq!(frames[0].from, Source::Host);
    assert_eq!(frames[1].from, Source::Servo(ID5));
    assert_valid(&frames[0]);
    assert_valid(&frames[1]);

    let ev = sim.host_events();
    assert_eq!(ev.len(), 2, "{ev:#?}");
    match &ev[0] {
        HostEvent::Status {
            slot,
            id,
            inst,
            payload,
        } => {
            assert_eq!((*slot, *id), (0, ID5));
            assert_eq!(status_result(*inst), Some(ResultCode::Ok));
            assert_eq!(payload.len(), 4, "model(2) + fw(2)");
        }
        other => panic!("expected Status, got {other:?}"),
    }
    match &ev[1] {
        HostEvent::Done(t) => {
            assert_eq!(t.outcome, Outcome::Complete);
            assert_eq!(t.evidence.statuses, 1);
            assert_eq!(t.evidence.garble, 0);
        }
        other => panic!("expected Done, got {other:?}"),
    }
}

#[test]
fn write_acks_and_reads_back() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.attach_host();
    let s = sim.add_servo(ID5);

    let mut p = [0u8; 16];
    let val = 0x0B0B_0B0Bu32;
    let n = build::write(&mut p, GOAL_VELOCITY, &val.to_le_bytes()).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::new(ID5),
        inst: Inst::instruction(Opcode::Write, 0),
        payload: &p[..n],
    })
    .unwrap();
    sim.run();
    let ev = sim.host_events();
    assert!(
        matches!(&ev[1], HostEvent::Done(t) if t.outcome == Outcome::Complete),
        "{ev:#?}"
    );
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_velocity),
        val as i32
    );

    let n = build::read(&mut p, GOAL_VELOCITY, 4).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::new(ID5),
        inst: Inst::instruction(Opcode::Read, 0),
        payload: &p[..n],
    })
    .unwrap();
    sim.run();
    let ev = sim.host_events();
    match &ev[0] {
        HostEvent::Status { payload, .. } => {
            assert_eq!(payload.as_slice(), &val.to_le_bytes());
        }
        other => panic!("expected Status, got {other:?}"),
    }
}

#[test]
fn noreply_write_is_sent_and_applied() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.attach_host();
    let s = sim.add_servo(ID5);

    let mut p = [0u8; 16];
    let n = build::write(&mut p, GOAL_VELOCITY, &0x11223344u32.to_le_bytes()).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::new(ID5),
        inst: Inst::instruction(Opcode::Write, Inst::FLAG_NOREPLY),
        payload: &p[..n],
    })
    .unwrap();
    let frames = sim.run();

    let ev = sim.host_events();
    assert_eq!(ev.len(), 1, "no status for NOREPLY: {ev:#?}");
    assert!(matches!(&ev[0], HostEvent::Done(t) if t.outcome == Outcome::Sent));
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_velocity),
        0x11223344
    );
    assert_eq!(frames.len(), 1, "instruction only, no reply on the wire");
}

#[test]
fn gread_chains_the_fleet_in_slot_order() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.attach_host();
    for id in [1u8, 2, 3] {
        sim.add_servo(id);
    }

    let mut p = [0u8; 16];
    let ids = [Id::new(1), Id::new(2), Id::new(3)];
    let n = build::gread_uniform(&mut p, GOAL_VELOCITY, 4, &ids).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::BROADCAST,
        inst: Inst::instruction(Opcode::Gread, 0),
        payload: &p[..n],
    })
    .unwrap();
    sim.run();

    let ev = sim.host_events();
    assert_eq!(ev.len(), 4, "{ev:#?}");
    for (k, want_id) in [1u8, 2, 3].iter().enumerate() {
        match &ev[k] {
            HostEvent::Status {
                slot, id, payload, ..
            } => {
                assert_eq!((*slot, *id), (k as u8, *want_id));
                assert_eq!(payload.len(), 4);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }
    assert!(matches!(&ev[3], HostEvent::Done(t) if t.outcome == Outcome::Complete));
}

/// 3M GREAD of four servos: slot k's break follows slot k-1's last byte,
/// the tightest spacing a host hears. Statuses delivered.
fn gread_3m_statuses(mark_lag_us: f64) -> u16 {
    let mut sim = Sim::new(BaudRate::B3000000);
    sim.attach_host();
    sim.set_host_mark_lag_us(mark_lag_us);
    let ids = [1u8, 2, 3, 4].map(Id::new);
    for id in ids {
        sim.add_servo(id.as_byte());
    }
    let mut p = [0u8; 16];
    let n = build::gread_uniform(&mut p, GOAL_VELOCITY, 4, &ids).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::BROADCAST,
        inst: Inst::instruction(Opcode::Gread, 0),
        payload: &p[..n],
    })
    .unwrap();
    sim.run();
    match sim.host_events().last() {
        Some(HostEvent::Done(t)) => t.evidence.statuses,
        other => panic!("expected Done, got {other:?}"),
    }
}

/// A break mark is the DMA count at the detector's service, so it names the
/// break's byte only while that service lands inside one byte-time of the
/// break's end (3.3 us at 3M). The adapter's vector preempts the engine to
/// hold that bound; at the budget lag every slot resolves, one byte-time
/// late they do not.
#[test]
fn a_3m_chain_resolves_every_slot_while_marks_land_inside_a_byte_time() {
    assert_eq!(gread_3m_statuses(HOST_MARK_LAG_US as f64), 4);
    assert!(gread_3m_statuses(3.5) < 4);
}

#[test]
fn absent_servo_times_out() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.attach_host();
    sim.add_servo(ID5);

    sim.host_submit(ping(9)).unwrap();
    sim.run();
    let ev = sim.host_events();
    assert_eq!(ev.len(), 1);
    assert!(
        matches!(&ev[0], HostEvent::Done(t) if t.outcome == Outcome::Timeout { slot: 0 }),
        "{ev:#?}"
    );
}

#[test]
fn enum_exact_prefix_elects_the_sole_matcher() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.attach_host();
    let a = sim.add_servo(1);
    sim.add_servo(2);
    sim.seed_servo_uid(a, [0xA1; UID_LEN]);
    sim.seed_servo_uid(1, [0xB2; UID_LEN]);

    let mut p = [0u8; 24];
    let n = build::mgmt_enum(&mut p, 128, &[0xA1; UID_LEN]).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::BROADCAST,
        inst: Inst::instruction(Opcode::Mgmt, 0),
        payload: &p[..n],
    })
    .unwrap();
    sim.run();

    let ev = sim.host_events();
    assert_eq!(ev.len(), 2, "{ev:#?}");
    match &ev[0] {
        HostEvent::Status { id, payload, .. } => {
            assert_eq!(*id, 1);
            assert_eq!(payload.as_slice(), &[0xA1; UID_LEN]);
        }
        other => panic!("expected Status, got {other:?}"),
    }
    // Collect completes at the quiet horizon; one matcher, no garble.
    match &ev[1] {
        HostEvent::Done(t) => {
            assert_eq!(t.outcome, Outcome::Complete);
            assert_eq!(t.evidence.statuses, 1);
            assert!(!t.evidence.garble_after_last_frame);
        }
        other => panic!("expected Done, got {other:?}"),
    }

    // A prefix nobody carries: quiet completion with zero statuses -- the
    // walk's empty-subtree verdict.
    let n = build::mgmt_enum(&mut p, 128, &[0x77; UID_LEN]).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::BROADCAST,
        inst: Inst::instruction(Opcode::Mgmt, 0),
        payload: &p[..n],
    })
    .unwrap();
    sim.run();
    let ev = sim.host_events();
    assert!(
        matches!(&ev[0], HostEvent::Done(t) if t.outcome == Outcome::Complete && t.evidence.statuses == 0),
        "{ev:#?}"
    );
}

#[test]
fn rescue_verb_reunites_host_and_fleet_at_the_rescue_rate() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.attach_host();
    sim.add_servo(ID5);

    sim.host_submit(Command::Rescue).unwrap();
    sim.run();
    let ev = sim.host_events();
    assert!(
        matches!(&ev[0], HostEvent::Done(t) if t.outcome == Outcome::Sent),
        "{ev:#?}"
    );

    // Both ends dropped to 0.5M together: a ping now round-trips (the
    // engine also spent its post-rescue pacing gap first).
    sim.host_submit(ping(ID5)).unwrap();
    sim.run();
    let ev = sim.host_events();
    assert_eq!(ev.len(), 2, "{ev:#?}");
    assert!(matches!(&ev[1], HostEvent::Done(t) if t.outcome == Outcome::Complete));
}

#[test]
fn cal_train_is_wire_invisible_to_the_fleet() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.attach_host();
    let s = sim.add_servo(ID5);

    let mut p = [0u8; 8];
    let n = build::mgmt_cal(&mut p, 400, 4).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::BROADCAST,
        inst: Inst::instruction(Opcode::Mgmt, 0),
        payload: &p[..n],
    })
    .unwrap();
    let frames = sim.run();

    let ev = sim.host_events();
    assert!(
        matches!(&ev[0], HostEvent::Done(t) if t.outcome == Outcome::Sent),
        "{ev:#?}"
    );
    // The announce plus gaps + 1 = 5 bare ruler marks, all host-sourced.
    let host_frames: Vec<_> = frames.iter().filter(|f| f.from == Source::Host).collect();
    assert_eq!(host_frames.len(), 6, "{frames:#?}");
    let bare = host_frames.iter().filter(|f| f.bytes == [0x00]).count();
    assert_eq!(bare, 5, "ruler marks record as empty break frames");
    // sec 9.3: the train is invisible to the link counters.
    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0);
    assert_eq!(d.framing_drop_count, 0);
}

/// One 3M TEL burst through the engine, the adapter's main loop held off
/// the engine for `stall_us` from the arm. Returns the burst's terminal.
fn tel_burst_with_stalled_poll(stall_us: u64) -> osc_host::engine::Terminal {
    // the ident four: held current, differential, duty, pos
    const ROW: Chunks = Chunks::one(I_TICK_COUNTS, 8);
    const SAMPLES: u16 = 160;
    let mut sim = Sim::new(BaudRate::B3000000);
    sim.attach_host();
    sim.add_servo(ID5);
    let mut p = [0u8; 16];
    let n = build::write(&mut p, CHUNK0_ADDR, &ROW.to_le_bytes()).unwrap();
    sim.host_submit(Command::Exchange {
        id: Id::new(ID5),
        inst: Inst::instruction(Opcode::Write, 0),
        payload: &p[..n],
    })
    .unwrap();
    sim.run();
    sim.host_events();

    let n = build::write(&mut p, TEL_COUNT, &SAMPLES.to_le_bytes()).unwrap();
    sim.host_submit(Command::ExchangeStream {
        id: Id::new(ID5),
        inst: Inst::instruction(Opcode::Write, 0),
        payload: &p[..n],
        window_us: 100_000,
    })
    .unwrap();
    sim.stall_host_poll(stall_us);
    sim.run();
    match sim.host_events().pop() {
        Some(HostEvent::Done(t)) => t,
        other => panic!("expected Done, got {other:?}"),
    }
}

#[test]
fn a_stalled_host_poll_reports_the_tel_ring_lap() {
    let clean = tel_burst_with_stalled_poll(0);
    assert_eq!(clean.outcome, Outcome::Complete);
    assert_eq!((clean.evidence.laps, clean.evidence.garble), (0, 0));

    // 6 ms of a ~150 B/ms burst is more than the sim's 512 B ring.
    let stalled = tel_burst_with_stalled_poll(6_000);
    assert_eq!(stalled.outcome, Outcome::Complete);
    assert_eq!(stalled.evidence.laps, 1);
    assert!(stalled.evidence.statuses < clean.evidence.statuses);
}
