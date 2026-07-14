//! Persistence semantics (`docs/osc-native-protocol.md` sec 9.4/9.5) over the
//! wire: MGMT SAVE/FACTORY against the RAM store, the config-dirty telemetry
//! bit, and the boot overlay a rebuilt servo takes -- sharing one leaked
//! store across two `Sim`s models a reboot with flash intact.

use osc_integration::sim::{RamStore, Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::table::STATUS_FLAG_CONFIG_DIRTY;
use osc_protocol::wire::{Id, MgmtOp, Opcode, ResultCode};
use osc_servo_core::persist::{Image, Slot};
use osc_servo_core::regions::config::addr::common::RESPONSE_DEADLINE_US;
use osc_servo_core::regions::control::addr::lifecycle::TORQUE_ENABLE;
use osc_servo_core::regions::telemetry::addr::common::STATUS_FLAGS;
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{matrix, sim};

const ID5: u8 = 5;
const BROADCAST: u8 = Id::BROADCAST.as_byte();

fn servo_frames(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

/// Assert exactly one well-formed servo reply and return it.
fn sole_reply(frames: &[WireFrame]) -> &WireFrame {
    let replies = servo_frames(frames);
    assert_eq!(replies.len(), 1, "expected one servo reply: {frames:#?}");
    assert_valid(replies[0]);
    replies[0]
}

/// One MGMT exchange (empty args beyond the sub-op byte).
fn mgmt(sim: &mut Sim, id: u8, op: MgmtOp) -> Vec<WireFrame> {
    sim.host_send(&instruction(id, Opcode::Mgmt, 0, &[op as u8]));
    sim.run()
}

/// One acked WRITE.
fn write_ok(sim: &mut Sim, id: u8, addr: u16, data: &[u8]) {
    let a = addr.to_le_bytes();
    let mut payload = vec![a[0], a[1]];
    payload.extend_from_slice(data);
    sim.host_send(&instruction(id, Opcode::Write, 0, &payload));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

/// READ one byte back.
fn read_byte(sim: &mut Sim, id: u8, addr: u16) -> u8 {
    let a = addr.to_le_bytes();
    sim.host_send(&instruction(id, Opcode::Read, 0, &[a[0], a[1], 1, 0]));
    let frames = sim.run();
    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    payload[0]
}

#[apply(matrix)]
fn save_persists_the_live_regions_and_clears_dirty(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    sim.add_servo_with_store(ID5, store);

    write_ok(&mut sim, ID5, RESPONSE_DEADLINE_US, &200u16.to_le_bytes());
    assert_eq!(
        read_byte(&mut sim, ID5, STATUS_FLAGS) & STATUS_FLAG_CONFIG_DIRTY,
        1
    );

    let frames = mgmt(&mut sim, ID5, MgmtOp::Save);
    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(payload.is_empty());
    assert_eq!(store.saves(), 1);
    assert_eq!(
        read_byte(&mut sim, ID5, STATUS_FLAGS) & STATUS_FLAG_CONFIG_DIRTY,
        0
    );

    // The stored image is codec-valid and carries the written value.
    let img = store.slot(Slot::A).expect("first save lands in slot A");
    let parsed = Image::parse(&img).expect("stored image parses");
    let at = RESPONSE_DEADLINE_US as usize;
    assert_eq!(parsed.config[at..at + 2], 200u16.to_le_bytes());
}

#[apply(matrix)]
fn save_with_torque_on_nacks_access(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    sim.add_servo_with_store(ID5, store);

    write_ok(&mut sim, ID5, TORQUE_ENABLE, &[1]);
    let frames = mgmt(&mut sim, ID5, MgmtOp::Save);
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Access));
    assert_eq!(store.saves(), 0);
}

#[apply(matrix)]
fn save_failure_nacks_hardware_and_stays_dirty(baud_idx: u8) {
    let store = RamStore::leak();
    store.set_fail(true);
    let mut sim = sim(baud_idx);
    sim.add_servo_with_store(ID5, store);

    write_ok(&mut sim, ID5, RESPONSE_DEADLINE_US, &200u16.to_le_bytes());
    let frames = mgmt(&mut sim, ID5, MgmtOp::Save);
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Hardware));
    assert_eq!(
        read_byte(&mut sim, ID5, STATUS_FLAGS) & STATUS_FLAG_CONFIG_DIRTY,
        1
    );
}

#[apply(matrix)]
fn factory_wipes_the_store_and_stages_reboot(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with_store(ID5, store);

    let frames = mgmt(&mut sim, ID5, MgmtOp::Save);
    assert_eq!(status(sole_reply(&frames)).0.result(), Some(ResultCode::Ok));
    assert!(store.slot(Slot::A).is_some());

    let frames = mgmt(&mut sim, ID5, MgmtOp::Factory);
    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(payload.is_empty());
    assert_eq!(store.wipes(), 1);
    assert!(store.slot(Slot::A).is_none() && store.slot(Slot::B).is_none());
    assert!(sim.take_reboot(s).is_some(), "factory stages the reboot");
}

/// The sec 9.2 + 9.4 field story: ASSIGN takes a new id immediately, SAVE
/// persists it, and the servo still answers on it after a reboot (fresh
/// `Sim`, same store) -- while a factory-wiped store boots board defaults.
#[apply(matrix)]
fn saved_id_survives_reboot_until_factory(baud_idx: u8) {
    let store = RamStore::leak();
    let assign = {
        let mut args = vec![MgmtOp::Assign as u8];
        args.extend_from_slice(&[ID5; 16]); // the sim's default UID: id repeated
        args.push(9);
        args
    };

    {
        let mut sim = sim(baud_idx);
        sim.add_servo_with_store(ID5, store);
        sim.host_send(&instruction(BROADCAST, Opcode::Mgmt, 0, &assign));
        let frames = sim.run();
        let reply = sole_reply(&frames);
        assert_eq!(
            reply.from,
            Source::Servo(9),
            "the ack leaves from the new id"
        );
        let frames = mgmt(&mut sim, 9, MgmtOp::Save);
        assert_eq!(status(sole_reply(&frames)).0.result(), Some(ResultCode::Ok));
    }

    // Reboot: fresh sim, flash intact -- the saved id answers, the board
    // default doesn't.
    {
        let mut sim = sim(baud_idx);
        sim.add_servo_with_store(ID5, store);
        sim.host_send(&instruction(9, Opcode::Ping, 0, &[]));
        let frames = sim.run();
        assert_eq!(sole_reply(&frames).from, Source::Servo(9));
        sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
        assert!(servo_frames(&sim.run()).is_empty());

        let frames = mgmt(&mut sim, 9, MgmtOp::Factory);
        assert_eq!(status(sole_reply(&frames)).0.result(), Some(ResultCode::Ok));
    }

    // Reboot after FACTORY: the wiped store boots board defaults again.
    {
        let mut sim = sim(baud_idx);
        sim.add_servo_with_store(ID5, store);
        sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
        let frames = sim.run();
        assert_eq!(sole_reply(&frames).from, Source::Servo(ID5));
    }
}
