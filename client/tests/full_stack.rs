//! Client -> records -> production LinkServer -> production engine -> sim
//! wire -> production servo stack, all in-process (fake-adapter backend).
//! Exercised through the blocking facade so both wrappers stay covered.

#![cfg(feature = "fake-adapter")]

use std::time::Duration;

use osc_client::blocking::Client;
use osc_client::common::{Health, Identity};
use osc_client::cyclic::{Cycle, Group, Telemetry};
use osc_client::data_state::{
    self, CALIB_VIRGIN, CONFIG_VIRGIN, PLANT_UNSET, Reason, STAMP_MISMATCH, fault,
};
use osc_client::descriptor::{Descriptor, Kind, Value, encode};
use osc_client::fake::{FakePipe, seed};
use osc_client::mgmt::{Found, Uid};
use osc_client::pos_lut::{self, INTERVALS, LutError, PosLut};
use osc_client::session::{EngineCommand, Record, Session};
use osc_client::{
    BaudRate, Error, Id, Inst, LinkError, Opcode, Outcome, RejectReason, ResultCode, StreamReply,
};
use osc_integration::sim::{Source, expect_tel_payload, status};
use osc_protocol::models::MODEL_OSC_SERVO;
use osc_protocol::table;
use osc_protocol::wire::UID_LEN;

/// V006 map fact used by read/write round trips (control.lifecycle
/// goal_velocity); the common block is the only protocol-fixed address space.
const GOAL_VELOCITY: u16 = 396;
/// Slot 0 of the profile region (protocol sec 5.2 pin).
const PROFILE_SLOT0: u16 = 0x280;
/// V006 map fact: config.loop_position.velocity_limit_cps, and the board
/// default the factory state comes back with.
const VELOCITY_LIMIT_CPS: u16 = 68;
const DEFAULT_VELOCITY_LIMIT_CPS: u16 = 1500;

/// V006 map facts the seeded calibration lands in: calib.pot.raw_max,
/// calib.kinematics.angle_max_cdeg and .gear_ratio_centi, the RO board fact
/// calib.sense.shunt_r_mohm, and config.pos_limits.pos_max_soft_counts.
const RAW_MAX: u16 = 130;
const ANGLE_MAX_CDEG: u16 = 174;
const GEAR_RATIO_CENTI: u16 = 176;
const SHUNT_R_MOHM: u16 = 132;
const POS_MAX_SOFT_COUNTS: u16 = 44;
/// The soft limit a wiped store boots: the board default travel, the pot's
/// full 12-bit span.
const DEFAULT_POS_MAX_SOFT_COUNTS: i32 = 4095;

/// Span word encoding, protocol sec 5.2: `[addr:10][count:6]`.
const fn span_word(addr: u16, count: u16) -> u16 {
    (addr << 6) | count
}

fn fleet(ids: &[u8]) -> Client<FakePipe> {
    Client::connect(FakePipe::new(BaudRate::B1000000, ids)).expect("connect")
}

fn read_u16(c: &mut Client<FakePipe>, id: Id, addr: u16) -> u16 {
    let b = c.read(id, addr, 2).expect("read u16");
    u16::from_le_bytes([b[0], b[1]])
}

fn read_i32(c: &mut Client<FakePipe>, id: Id, addr: u16) -> i32 {
    let b = c.read(id, addr, 4).expect("read i32");
    i32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

#[test]
fn connect_reports_link_info() {
    let c = fleet(&[1]);
    assert_eq!(c.info().version, osc_host::link::record::LINK_VERSION);
    assert!(c.info().ticks_per_us > 0);
    // A chip-less adapter has nothing to report, but the tail is present.
    assert_eq!(c.info().diag, Some(Default::default()));
}

#[test]
fn ping_digests_model_and_fw() {
    let mut c = fleet(&[5]);
    let ping = c.ping(Id::new(5)).expect("ping");
    assert!(!ping.alert);
    // Model/fw come from the servo's identity block; nonzero model is the
    // seeded default.
    assert!(ping.model > 0);
}

#[test]
fn write_reads_back() {
    let mut c = fleet(&[5]);
    let val = 0x0B0B_0B0Bu32.to_le_bytes();
    c.write(Id::new(5), GOAL_VELOCITY, &val).expect("write");
    let got = c.read(Id::new(5), GOAL_VELOCITY, 4).expect("read");
    assert_eq!(got, val);
}

#[test]
fn noreply_write_applies_silently() {
    let mut c = fleet(&[5]);
    let val = 0x1122_3344u32.to_le_bytes();
    c.write_noreply(Id::new(5), GOAL_VELOCITY, &val)
        .expect("noreply write");
    let got = c.read(Id::new(5), GOAL_VELOCITY, 4).expect("read");
    assert_eq!(got, val);
}

#[test]
fn hold_then_commit_applies_atomically() {
    let mut c = fleet(&[5]);
    let val = 0x0505_0505u32.to_le_bytes();
    c.write_hold(Id::new(5), GOAL_VELOCITY, &val).expect("hold");
    let before = c.read(Id::new(5), GOAL_VELOCITY, 4).expect("read");
    assert_ne!(before, val, "held write must not apply before COMMIT");
    c.commit().expect("commit");
    let after = c.read(Id::new(5), GOAL_VELOCITY, 4).expect("read");
    assert_eq!(after, val);
}

#[test]
fn gread_chains_in_slot_order() {
    let mut c = fleet(&[1, 2, 3]);
    let ids = [Id::new(1), Id::new(2), Id::new(3)];
    let chain = c.gread(&ids, GOAL_VELOCITY, 4).expect("gread");
    assert_eq!(chain.timeout_slot, None);
    let order: Vec<(u8, u8)> = chain.statuses.iter().map(|s| (s.slot, s.id)).collect();
    assert_eq!(order, vec![(0, 1), (1, 2), (2, 3)]);
}

#[test]
fn gwrite_fans_out_per_target_values() {
    let mut c = fleet(&[1, 2]);
    let a = 0x0000_00AAu32.to_le_bytes();
    let b = 0x0000_00BBu32.to_le_bytes();
    c.gwrite(
        GOAL_VELOCITY,
        4,
        &[(Id::new(1), &a[..]), (Id::new(2), &b[..])],
    )
    .expect("gwrite");
    assert_eq!(c.read(Id::new(1), GOAL_VELOCITY, 4).expect("read"), a);
    assert_eq!(c.read(Id::new(2), GOAL_VELOCITY, 4).expect("read"), b);
}

#[test]
fn absent_servo_times_out() {
    let mut c = fleet(&[5]);
    match c.ping(Id::new(9)) {
        Err(Error::Timeout { slot: 0 }) => {}
        other => panic!("expected timeout, got {other:?}"),
    }
}

#[test]
fn invalid_id_rejects_before_the_wire() {
    let mut c = fleet(&[5]);
    match c.ping(Id::new(0)) {
        Err(Error::Link(LinkError::Rejected(RejectReason::BadId))) => {}
        other => panic!("expected BadId rejection, got {other:?}"),
    }
}

#[test]
fn servo_result_codes_surface_as_errors() {
    let mut c = fleet(&[5]);
    // A read past the table answers `range` at the instruction layer.
    match c.read(Id::new(5), 0x3FF, 64) {
        Err(Error::Servo(osc_client::ResultCode::Range)) => {}
        other => panic!("expected Servo(Range), got {other:?}"),
    }
}

#[test]
fn rescue_reunites_at_the_rescue_rate() {
    let mut c = fleet(&[5]);
    let roster = c.rescue_sweep(&[Id::new(5)]).expect("rescue sweep");
    assert_eq!(roster, vec![(Id::new(5), true)]);
}

#[test]
fn cal_trains_complete_and_stay_wire_invisible() {
    let mut c = fleet(&[5]);
    c.cal(2, 400, 4).expect("cal");
    // sec 9.3: the train is invisible to the link counters.
    let diag = c.pipe_mut().sim_mut().servo_diag(0);
    assert_eq!(diag.crc_fail_count, 0);
    assert_eq!(diag.framing_drop_count, 0);
    c.ping(Id::new(5)).expect("ping after cal");
}

#[test]
fn discover_walks_out_every_uid_with_its_id() {
    let mut c = fleet(&[1, 2, 3]);
    let mut want = Vec::new();
    for (i, seed) in [0x11u8, 0x2E, 0x93].into_iter().enumerate() {
        let mut uid = [0u8; UID_LEN];
        uid[0] = seed;
        uid[5] = 0xC0 | i as u8;
        c.pipe_mut().sim_mut().seed_servo_uid(i, uid);
        want.push(Found {
            uid: Uid(uid),
            id: Id::new(i as u8 + 1),
        });
    }
    want.sort_by_key(|f| f.uid);
    let found = c.discover().expect("discover");
    assert_eq!(found, want);
}

/// `osc discover`'s sweep: a walk that found the fleet leaves nothing behind
/// that the next rate's baud switch or walk trips over.
#[test]
fn discover_sweeps_every_rate_past_the_one_that_answers() {
    let mut c = fleet(&[1]);
    let uid = [0x5Au8; UID_LEN];
    c.pipe_mut().sim_mut().seed_servo_uid(0, uid);
    let mut seen = Vec::new();
    for rate in [
        BaudRate::B500000,
        BaudRate::B1000000,
        BaudRate::B2000000,
        BaudRate::B3000000,
    ] {
        c.host_baud(rate).expect("host baud");
        seen.push((rate, c.discover().expect("discover")));
    }
    let found = Found {
        uid: Uid(uid),
        id: Id::new(1),
    };
    assert_eq!(
        seen,
        vec![
            (BaudRate::B500000, vec![]),
            (BaudRate::B1000000, vec![found]),
            (BaudRate::B2000000, vec![]),
            (BaudRate::B3000000, vec![]),
        ]
    );
}

#[test]
fn find_bus_baud_follows_the_fleet() {
    let mut c = fleet(&[1, 2]);
    assert_eq!(
        c.find_bus_baud().expect("probe at boot"),
        Some(BaudRate::B1000000)
    );
    let roster = c.set_baud(&[Id::new(1), Id::new(2)], BaudRate::B3000000);
    assert!(roster.expect("migrate").iter().all(|&(_, alive)| alive));
    assert_eq!(
        c.find_bus_baud().expect("probe after migration"),
        Some(BaudRate::B3000000)
    );
}

#[test]
fn assign_moves_the_matcher_to_its_new_id() {
    let mut c = fleet(&[1]);
    let uid = [0xA7u8; UID_LEN];
    c.pipe_mut().sim_mut().seed_servo_uid(0, uid);
    c.assign(&Uid(uid), Id::new(9)).expect("assign");
    c.ping(Id::new(9)).expect("ping at the new id");
    match c.ping(Id::new(1)) {
        Err(Error::Timeout { .. }) => {}
        other => panic!("old id must be vacant, got {other:?}"),
    }
}

/// protocol sec 9.4/9.5 end to end: every simulated servo carries its own
/// store, so SAVE is durable across the reboot that follows and FACTORY
/// really wipes - the GUI rescans straight after, so the wiped servo has to
/// answer on its board id.
#[test]
fn save_survives_reboot_and_factory_restores_defaults() {
    let mut c = fleet(&[1]);
    let id = Id::new(1);
    let val = 777u16.to_le_bytes();
    c.write(id, VELOCITY_LIMIT_CPS, &val).expect("write");
    c.save(id).expect("save");
    c.reboot(id).expect("reboot");
    assert_eq!(
        c.read(id, VELOCITY_LIMIT_CPS, 2)
            .expect("read after reboot"),
        val,
        "the saved image is what booted"
    );

    c.factory(id).expect("factory");
    assert_eq!(
        c.read(id, VELOCITY_LIMIT_CPS, 2)
            .expect("read after factory"),
        DEFAULT_VELOCITY_LIMIT_CPS.to_le_bytes(),
        "a wiped store boots board defaults"
    );
    let found = c.discover().expect("discover after factory");
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].id, id);
}

/// The seeded fleet ships the way a servo leaves the calibration bench: the
/// dump is SAVEd, not just written into the live table. So it boots back
/// after a reboot and FACTORY wipes it exactly as hardware does - CALIB to
/// zero, the travel limits to the board's full span - while the board's own
/// facts (install re-stamps the sense chain, ESIG carries the UID) stand.
#[test]
fn factory_wipes_the_seeded_calibration_like_hardware() {
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
    pipe.seed_calibrated(0);
    let uid = pipe.sim_mut().servo_uid(0);
    let mut c = Client::connect(pipe).expect("connect");
    let id = Id::new(1);

    assert_eq!(read_u16(&mut c, id, RAW_MAX), seed::RAW_MAX);
    assert_eq!(
        read_u16(&mut c, id, ANGLE_MAX_CDEG),
        seed::ANGLE_MAX_CDEG as u16
    );
    assert_eq!(
        read_u16(&mut c, id, GEAR_RATIO_CENTI),
        seed::GEAR_RATIO_CENTI
    );
    assert_eq!(
        read_i32(&mut c, id, POS_MAX_SOFT_COUNTS),
        seed::POS_MAX_SOFT_COUNTS
    );

    c.reboot(id).expect("reboot");
    assert_eq!(
        read_u16(&mut c, id, RAW_MAX),
        seed::RAW_MAX,
        "a SAVEd calibration is what boots"
    );

    c.factory(id).expect("factory");
    assert_eq!(read_u16(&mut c, id, RAW_MAX), 0, "CALIB is wiped");
    assert_eq!(read_u16(&mut c, id, ANGLE_MAX_CDEG), 0);
    assert_eq!(read_u16(&mut c, id, GEAR_RATIO_CENTI), 0);
    assert_eq!(
        read_i32(&mut c, id, POS_MAX_SOFT_COUNTS),
        DEFAULT_POS_MAX_SOFT_COUNTS,
        "the travel limits fall back to board defaults"
    );
    assert_eq!(
        read_u16(&mut c, id, SHUNT_R_MOHM),
        seed::SENSE.shunt_r_mohm,
        "the sense chain is board data, not table state"
    );
    assert_eq!(
        c.pipe_mut().sim_mut().servo_uid(0),
        uid,
        "the UID is silicon"
    );
}

#[test]
fn rails_and_bootloader_ack() {
    let mut c = fleet(&[1]);
    assert_eq!(c.set_rails(true, false).expect("rails"), (true, false));
    c.enter_bootloader().expect("bootloader");
}

#[test]
fn rails_masked_sets_compose_and_read_back() {
    let mut c = fleet(&[1]);
    // Boot mirror: both rails on.
    assert_eq!(c.rails().expect("readback"), (true, true));
    assert_eq!(c.set_rail_5v(false).expect("5v off"), (true, false));
    assert_eq!(c.set_rail_3v3(false).expect("3v3 off"), (false, false));
    assert_eq!(c.set_rail_5v(true).expect("5v on"), (false, true));
    assert_eq!(c.rails().expect("readback"), (false, true));
}

#[test]
fn identity_decodes_the_config_front() {
    let mut c = fleet(&[5]);
    let fw = c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.config.common.hardware_revision = 3;
        t.config.common.capability_flags = 0x8000_0001;
        t.config.common.firmware_version
    });
    let got = c.identity(Id::new(5)).expect("identity");
    assert_eq!(
        got,
        Identity {
            model: MODEL_OSC_SERVO, // the sim mirrors the servo's registry identity
            fw,                     // osc_servo_core::FIRMWARE_VERSION
            hw: 3,
            capabilities: 0x8000_0001,
        }
    );
}

#[test]
fn health_reads_the_telemetry_front_and_tracks_dirty() {
    let mut c = fleet(&[5]);
    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.telemetry.common.fault_flags = 0x04;
        t.telemetry.common.trim_steps = -3;
        t.telemetry.common.crc_fail_count = 7;
        t.telemetry.common.framing_drop_count = 9;
    });
    // fault_flags nonzero sets ALERT on the reply; the digest must not
    // mistake that for an error.
    let got = c.health(Id::new(5)).expect("health");
    assert_eq!(
        got,
        Health {
            fault_flags: 0x04,
            config_dirty: false,
            trim_steps: -3,
            crc_fail_count: 7,
            framing_drop_count: 9,
        }
    );
    c.write(
        Id::new(5),
        table::RESPONSE_DEADLINE_US,
        &60u16.to_le_bytes(),
    )
    .expect("config write");
    assert!(c.health(Id::new(5)).expect("health").config_dirty);
}

#[test]
fn clear_counters_zeroes_both_in_one_write() {
    let mut c = fleet(&[5]);
    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.telemetry.common.crc_fail_count = 41;
        t.telemetry.common.framing_drop_count = 8;
    });
    c.clear_counters(Id::new(5)).expect("clear");
    let h = c.health(Id::new(5)).expect("health");
    assert_eq!((h.crc_fail_count, h.framing_drop_count), (0, 0));
}

// --- data state and the plant stamp ---

/// The checked-in descriptor: the stamp recipe and the register names the
/// data-state helpers resolve.
const DESCRIPTOR: &str = include_str!("../../descriptors/osc-servo/0.1.json");
/// The kernel's latched data fault (`kernel/faults.rs` BIT_DATA).
const BIT_DATA: u8 = 1 << 6;
/// A never-saved, never-identified servo.
const FRESH: u8 = CONFIG_VIRGIN | CALIB_VIRGIN | STAMP_MISMATCH | PLANT_UNSET;

fn descriptor() -> Descriptor {
    Descriptor::parse(DESCRIPTOR).expect("descriptor parses")
}

fn write_field(c: &mut Client<FakePipe>, id: Id, d: &Descriptor, name: &str, v: Value) {
    let f = d
        .field(name)
        .unwrap_or_else(|| panic!("{name} in descriptor"));
    let bytes = encode(f, &v).unwrap_or_else(|e| panic!("{name}: {e}"));
    c.write(id, f.addr, &bytes)
        .unwrap_or_else(|e| panic!("{name}: {e}"));
}

fn firmware_stamp(c: &mut Client<FakePipe>) -> u16 {
    c.pipe_mut()
        .sim_mut()
        .servo_table(0, |t| osc_servo_core::stamp::compute(t, None))
}

#[test]
fn seeded_servo_boots_identified_and_stamped() {
    let d = descriptor();
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
    pipe.seed_calibrated(0);
    let mut c = Client::connect(pipe).expect("connect");
    let id = Id::new(1);

    let s = c.data_state(id, &d).expect("data state");
    assert_eq!((s.flags, s.fault_code), (0, fault::NONE));
    assert_eq!(s.message(), None);
    let v = c.stamp_verdict(id, &d).expect("verdict");
    assert!(v.matches(), "{v:?}");
    assert_eq!(v.stored, firmware_stamp(&mut c));
    assert!(data_state::allows(s.flags, true));

    c.reboot(id).expect("reboot");
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);

    c.factory(id).expect("factory");
    let s = c.data_state(id, &d).expect("data state");
    assert_eq!(s.flags, FRESH, "a wiped store boots factory-fresh");
    assert_eq!(s.reasons()[0], Reason::ConfigVirgin);
    assert_eq!(
        c.stamp_verdict(id, &d).expect("verdict").stored,
        osc_client::stamp::UNSTAMPED
    );
}

/// Board data alone leaves the servo factory-fresh: the GUI's virgin
/// variant.
#[test]
fn board_seed_alone_is_factory_fresh() {
    let d = descriptor();
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
    pipe.seed_board(0);
    let mut c = Client::connect(pipe).expect("connect");
    let s = c.data_state(Id::new(1), &d).expect("data state");
    assert_eq!(s.flags, FRESH);
    assert_eq!(
        read_u16(&mut c, Id::new(1), SHUNT_R_MOHM),
        seed::SENSE.shunt_r_mohm
    );
}

/// The host stamp is the firmware's, byte for byte: over the seeded set,
/// over a set written through the wire, and over tables filled with
/// arbitrary bytes; the firmware's own checkpoint (a torque-off stamp
/// write) is the second witness each time.
#[test]
fn host_stamp_is_the_firmwares_over_the_same_table() {
    let d = descriptor();
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
    pipe.seed_calibrated(0);
    let mut c = Client::connect(pipe).expect("connect");
    let id = Id::new(1);
    let stamp = d.stamp().expect("recipe");
    assert_eq!(stamp.covered().len(), 35);

    let check = |c: &mut Client<FakePipe>, case: &str| {
        let host = c.plant_stamp(id, &d).expect("host stamp");
        assert_eq!(host, firmware_stamp(c), "{case}");
        assert_eq!(c.restamp(id, &d).expect("restamp"), host);
        let s = c.data_state(id, &d).expect("data state");
        assert_eq!(s.flags & STAMP_MISMATCH, 0, "{case}: the checkpoint agrees");
        host
    };
    let seeded = check(&mut c, "seeded");

    write_field(&mut c, id, &d, "v_kp_q88", Value::Uint(700));
    write_field(&mut c, id, &d, "recip_ke_q", Value::Uint(4321));
    write_field(&mut c, id, &d, "drive_polarity", Value::Bool(false));
    write_field(&mut c, id, &d, "pos_deadband_counts", Value::Uint(9));
    write_field(&mut c, id, &d, "l1_q016", Value::Uint(12345));
    assert_eq!(
        c.data_state(id, &d).expect("data state").flags,
        STAMP_MISMATCH,
        "covered writes mark the set stale"
    );
    let wired = check(&mut c, "wire-written");
    assert_ne!(wired, seeded);

    // Arbitrary bytes in every covered field, past any write rule: the
    // hash must not depend on what the values mean.
    let mut x = 0x2545_F491u32;
    for round in 0..4 {
        c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            let bytes = table_bytes(t);
            for f in stamp.covered() {
                for b in &mut bytes[f.addr as usize..f.end() as usize] {
                    x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    *b = (x >> 24) as u8;
                    if f.kind == Kind::Bool {
                        *b &= 1;
                    }
                }
            }
        });
        let table = c
            .pipe_mut()
            .sim_mut()
            .servo_table_mut(0, |t| table_bytes(t).to_vec());
        let offline = stamp.compute(0, &table, None).expect("offline");
        let host = check(&mut c, &format!("fill round {round}"));
        assert_eq!(host, offline, "the span read and the whole table agree");
    }
}

/// The table as the flat byte map the firmware hashes (repr(C), padding-free
/// by the derive's assert, every byte initialized).
fn table_bytes(t: &mut osc_servo_core::ControlTable) -> &mut [u8] {
    // SAFETY: see fn doc; the borrow of `t` bounds the slice, and the
    // caller writes only int and 0/1 bool bytes.
    unsafe {
        std::slice::from_raw_parts_mut(
            (t as *mut osc_servo_core::ControlTable).cast::<u8>(),
            std::mem::size_of::<osc_servo_core::ControlTable>(),
        )
    }
}

/// The commit sequence on a factory-fresh servo, as `osc cal` then
/// `osc ident write` run it: the set, the stamp, SAVE, and only then a
/// clean data state; a covered edit afterwards shuts closed loop until a
/// restamp.
#[test]
fn virgin_servo_commit_sequence_then_a_covered_edit_refuses_closed_loop() {
    let d = descriptor();
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
    pipe.seed_board(0);
    let mut c = Client::connect(pipe).expect("connect");
    let id = Id::new(1);

    let s = c.data_state(id, &d).expect("data state");
    assert_eq!(s.flags, FRESH);
    assert!(
        data_state::allows(s.flags, false),
        "OpenLoop and Current run"
    );
    assert!(!data_state::allows(s.flags, true));

    // cal: stops, travel, polarity, angles, gear
    for (name, v) in [
        ("pos_min_phys_counts", Value::Int(5)),
        ("pos_max_phys_counts", Value::Int(4095)),
        ("pos_min_soft_counts", Value::Int(228)),
        ("pos_max_soft_counts", Value::Int(3872)),
        ("raw_min", Value::Uint(5)),
        ("raw_max", Value::Uint(4095)),
        ("drive_polarity", Value::Bool(true)),
        ("angle_min_cdeg", Value::Int(0)),
        ("angle_max_cdeg", Value::Int(20200)),
        ("gear_ratio_centi", Value::Uint(25464)),
    ] {
        write_field(&mut c, id, &d, name, v);
    }
    // ident write: the identified set
    for (name, v) in [
        ("r_q12", seed::R_Q12),
        ("recip_ke_q", seed::RECIP_KE_Q),
        ("b_i_q313", seed::B_I_Q313),
        ("fric_fc_counts", seed::FRIC_FC_COUNTS),
        ("fric_fv_q016", seed::FRIC_FV_Q016),
        ("ke_vpc_q", seed::KE_VPC_Q),
        ("v_kp_q88", 543),
        ("p_kp_q88", 40212),
    ] {
        write_field(&mut c, id, &d, name, Value::Uint(v as u64));
    }
    assert_eq!(c.data_state(id, &d).expect("data state").flags, FRESH);

    // the stamp, with torque off, is the checkpoint
    let stamp = c.restamp(id, &d).expect("restamp");
    assert_eq!(stamp, firmware_stamp(&mut c));
    let s = c.data_state(id, &d).expect("data state");
    assert_eq!(s.flags, CONFIG_VIRGIN | CALIB_VIRGIN);
    assert_eq!(s.reasons(), [Reason::ConfigVirgin, Reason::CalibVirgin]);
    assert!(
        !data_state::allows(s.flags, true),
        "VIRGIN clears only on SAVE"
    );

    c.save(id).expect("save");
    let s = c.data_state(id, &d).expect("data state");
    assert_eq!(s.flags, 0);
    assert!(data_state::allows(s.flags, true));
    c.reboot(id).expect("reboot");
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);
    assert!(c.stamp_verdict(id, &d).expect("verdict").matches());

    // a gain edit: the set is no longer the stamped one
    write_field(&mut c, id, &d, "v_kp_q88", Value::Uint(600));
    let s = c.data_state(id, &d).expect("data state");
    assert_eq!(s.flags, STAMP_MISMATCH);
    assert!(!data_state::allows(s.flags, true));
    assert!(data_state::allows(s.flags, false));
    let v = c.stamp_verdict(id, &d).expect("verdict");
    assert!(!v.matches());
    assert_eq!(v.stored, stamp);

    // the kernel's refusal of the next closed-loop enable, as the fault
    // ISR publishes it (the fake adapter runs no kernel)
    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.telemetry.common.fault_flags = BIT_DATA;
        t.telemetry.mode.fault_code = fault::DATA;
    });
    let s = c.data_state(id, &d).expect("data state");
    assert_eq!(fault::name(s.fault_code), Some("data"));
    assert_eq!(
        s.message().unwrap(),
        format!("closed loop refused: {}", Reason::StampMismatch.text())
    );
    assert_eq!(c.health(id).expect("health").fault_flags, BIT_DATA);

    // torque on: the restamp lands unverified until torque drops
    write_field(&mut c, id, &d, "torque_enable", Value::Bool(true));
    c.restamp(id, &d).expect("restamp under torque");
    assert_eq!(
        c.data_state(id, &d).expect("data state").flags,
        STAMP_MISMATCH
    );
    write_field(&mut c, id, &d, "torque_enable", Value::Bool(false));
    c.restamp(id, &d).expect("restamp");
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);
}

// --- position table ---

/// The mg90-a table on the 2S session, built against stops 209/3849.
const MG90_A_IMAGE: &str = include_str!("../../ident/testdata/lut/pos-lut-mg90-a-grid.json");

fn mg90_a() -> [i16; INTERVALS] {
    let img: serde_json::Value = serde_json::from_str(MG90_A_IMAGE).expect("image parses");
    let points: Vec<i16> = img["points"]
        .as_array()
        .expect("points")
        .iter()
        .map(|v| v.as_i64().expect("point") as i16)
        .collect();
    points.try_into().expect("256 points")
}

/// A stamped servo with the mg90-a stops in place of the seeded SG90's.
fn mg90_servo() -> (Descriptor, Client<FakePipe>, Id) {
    let d = descriptor();
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
    pipe.seed_calibrated(0);
    let mut c = Client::connect(pipe).expect("connect");
    let id = Id::new(1);
    write_field(&mut c, id, &d, "raw_min", Value::Uint(209));
    write_field(&mut c, id, &d, "raw_max", Value::Uint(3849));
    c.restamp(id, &d).expect("restamp");
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);
    (d, c, id)
}

fn servo_lut(c: &mut Client<FakePipe>) -> (u8, [i16; INTERVALS]) {
    let state = c
        .pipe_mut()
        .sim_mut()
        .servo_table(0, |t| t.control.pos_lut.pos_lut_state);
    let points = c.pipe_mut().sim_mut().servo_pos_lut(0, |k| {
        let mut out = [0i16; INTERVALS];
        out.copy_from_slice(&k[..INTERVALS]);
        out
    });
    (state, points)
}

/// The per-servo switch: the table goes LIVE and reads back point for point,
/// the COMMIT checkpoint marks the set stale, and the host stamp hashes the
/// live points exactly as the firmware does, so a restamp clears it.
#[test]
fn lut_write_goes_live_and_the_stamp_hashes_the_live_points() {
    let (d, mut c, id) = mg90_servo();
    let points = mg90_a();
    assert_eq!(
        c.pos_lut(id, &d).expect("read"),
        PosLut {
            state: pos_lut::state::IDENTITY,
            points: [0; INTERVALS],
        }
    );

    c.write_pos_lut(id, &d, &points).expect("write");
    let lut = c.pos_lut(id, &d).expect("read");
    assert_eq!(lut.state, pos_lut::state::LIVE);
    assert_eq!(lut.points, points);
    assert_eq!(servo_lut(&mut c), (pos_lut::state::LIVE, points));
    assert_eq!(
        c.pos_lut_state(id, &d).expect("state"),
        pos_lut::state::LIVE
    );

    let s = c.data_state(id, &d).expect("data state");
    assert_eq!(s.flags, STAMP_MISMATCH, "the hashed points changed");
    assert!(!data_state::allows(s.flags, true));
    let v = c.stamp_verdict(id, &d).expect("verdict");
    assert!(!v.matches());
    let firmware = c
        .pipe_mut()
        .sim_mut()
        .servo_table(0, |t| osc_servo_core::stamp::compute(t, Some(&points)));
    assert_eq!(v.computed, firmware, "the host hashes the live array");
    assert_ne!(v.computed, firmware_stamp(&mut c), "not the identity");

    let stamp = c.restamp(id, &d).expect("restamp");
    assert_eq!(stamp, firmware);
    let s = c.data_state(id, &d).expect("data state");
    assert_eq!(s.flags, 0, "stamped over the live table");
    assert!(c.stamp_verdict(id, &d).expect("verdict").matches());

    // the same table again: STORE marks the set stale, COMMIT lands the
    // same hash, so the checkpoint matches with no restamp
    c.write_pos_lut(id, &d, &points).expect("rewrite");
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);

    // the identity as a LIVE table hashes like no table
    c.clear_pos_lut(id, &d).expect("clear");
    let lut = c.pos_lut(id, &d).expect("read");
    assert_eq!(
        (lut.state, lut.points),
        (pos_lut::state::LIVE, [0; INTERVALS])
    );
    assert_eq!(
        c.data_state(id, &d).expect("data state").flags,
        STAMP_MISMATCH
    );
    assert_eq!(c.restamp(id, &d).expect("restamp"), firmware_stamp(&mut c));
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);
}

#[test]
fn lut_write_reports_the_servos_rejects_and_refuses_under_torque() {
    let (d, mut c, id) = mg90_servo();
    let good = mg90_a();

    // a nonzero point inside the low inset of stop 209
    let mut ends = good;
    ends[14] = 1;
    assert_eq!(
        c.write_pos_lut(id, &d, &ends),
        Err(Error::Lut(LutError::RejectEnds))
    );
    assert_eq!(
        c.pos_lut_state(id, &d).expect("state"),
        pos_lut::state::REJECT_ENDS
    );
    assert_eq!(
        c.data_state(id, &d).expect("data state").flags,
        0,
        "identity hashes as before"
    );
    let mut shape = good;
    shape[100] = shape[99] - 16;
    assert_eq!(
        c.write_pos_lut(id, &d, &shape),
        Err(Error::Lut(LutError::RejectShape))
    );
    assert_eq!(
        c.pos_lut_state(id, &d).expect("state"),
        pos_lut::state::REJECT_SHAPE
    );
    // the rejected array is still there, and readable
    assert_eq!(c.pos_lut(id, &d).expect("read").points, shape);

    c.write_pos_lut(id, &d, &good).expect("write");
    assert_eq!(
        c.pos_lut_state(id, &d).expect("state"),
        pos_lut::state::LIVE
    );

    // torque on: refused host-side after the one torque read, the live
    // table stands
    write_field(&mut c, id, &d, "torque_enable", Value::Bool(true));
    c.pipe_mut().take_frames();
    assert_eq!(
        c.write_pos_lut(id, &d, &ends),
        Err(Error::Lut(LutError::TorqueOn))
    );
    let frames = c.pipe_mut().take_frames();
    assert_eq!(
        frames
            .iter()
            .filter(|f| matches!(f.from, Source::Host))
            .count(),
        1,
        "one instruction on the wire"
    );
    assert_eq!(servo_lut(&mut c), (pos_lut::state::LIVE, good));
    assert_eq!(c.clear_pos_lut(id, &d), Err(Error::Lut(LutError::TorqueOn)));
    write_field(&mut c, id, &d, "torque_enable", Value::Bool(false));
    c.clear_pos_lut(id, &d).expect("clear");
    assert_eq!(servo_lut(&mut c), (pos_lut::state::LIVE, [0; INTERVALS]));
}

/// Cal moving the stops under a LIVE table: the servo validates a table
/// only at COMMIT and at boot, so the host re-COMMITs the array in RAM and
/// the state says whether it still fits the new stops. Either way the
/// stamp that follows hashes what the kernel applies.
#[test]
fn lut_recommit_judges_the_live_table_against_the_new_stops() {
    let (d, mut c, id) = mg90_servo();
    let points = mg90_a();
    c.write_pos_lut(id, &d, &points).expect("write");
    c.restamp(id, &d).expect("restamp");
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);

    // the stops move inside the insets: the table still fits
    write_field(&mut c, id, &d, "raw_min", Value::Uint(232));
    assert_eq!(
        c.data_state(id, &d).expect("data state").flags,
        STAMP_MISMATCH,
        "a covered write"
    );
    assert_eq!(c.recommit_pos_lut(id, &d), Ok(pos_lut::state::LIVE));
    assert_eq!(servo_lut(&mut c), (pos_lut::state::LIVE, points));
    let live = c
        .pipe_mut()
        .sim_mut()
        .servo_table(0, |t| osc_servo_core::stamp::compute(t, Some(&points)));
    assert_eq!(c.restamp(id, &d).expect("restamp"), live);
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);

    // a stop into the covered span: a nonzero point lands in the low inset,
    // the kernel runs the identity, the array stays for a rebuild to
    // overwrite, and the stamp hashes the identity
    write_field(&mut c, id, &d, "raw_min", Value::Uint(600));
    assert_eq!(c.recommit_pos_lut(id, &d), Ok(pos_lut::state::REJECT_ENDS));
    assert_eq!(servo_lut(&mut c).0, pos_lut::state::REJECT_ENDS);
    assert_eq!(c.pos_lut(id, &d).expect("read").points, points);
    assert_eq!(
        c.data_state(id, &d).expect("data state").flags,
        STAMP_MISMATCH
    );
    assert_eq!(c.restamp(id, &d).expect("restamp"), firmware_stamp(&mut c));
    assert_eq!(c.data_state(id, &d).expect("data state").flags, 0);

    // torque on: refused before the wire, the state stands
    write_field(&mut c, id, &d, "torque_enable", Value::Bool(true));
    assert_eq!(
        c.recommit_pos_lut(id, &d),
        Err(Error::Lut(LutError::TorqueOn))
    );
    assert_eq!(servo_lut(&mut c).0, pos_lut::state::REJECT_ENDS);
}

/// The park mechanism itself is pinned in osc-integration's `cross_baud`
/// suite -- the link-mode rig cannot hold a parked resolver across
/// commands (every exchange drains the sim queue = unbounded quiet), so
/// this level pins the paced choreography only.
#[test]
fn set_baud_migrates_servo_first_and_reunites() {
    let mut c = fleet(&[1, 2]);
    let ids = [Id::new(1), Id::new(2)];
    let roster = c.set_baud(&ids, BaudRate::B3000000).expect("set_baud");
    assert_eq!(roster, vec![(Id::new(1), true), (Id::new(2), true)]);
    let chain = c.gread(&ids, GOAL_VELOCITY, 4).expect("gread at 3M");
    assert_eq!(chain.timeout_slot, None);
    assert_eq!(chain.statuses.len(), 2);
}

#[test]
fn cal_verify_traces_every_train() {
    let mut c = fleet(&[5]);
    let traces = c.cal_verify(&[Id::new(5)], 3, 400, 4).expect("cal_verify");
    assert_eq!(traces.len(), 1);
    assert_eq!(traces[0].id, Id::new(5));
    assert_eq!(traces[0].trims.len(), 3);
    // Sim clocks don't drift, so the applied total never moves -- the
    // convergence verdict is what silicon exercises.
    assert!(traces[0].converged());
}

#[test]
fn cycle_step_commits_the_writes_the_chain_echoes() {
    let mut c = fleet(&[1, 2]);
    let ids = vec![Id::new(1), Id::new(2)];
    let cycle = Cycle::new(
        vec![Group {
            addr: GOAL_VELOCITY,
            count: 4,
            ids: ids.clone(),
        }],
        Telemetry::Span {
            ids: ids.clone(),
            addr: GOAL_VELOCITY,
            count: 4,
        },
    );
    let a = 0x0000_00AAu32.to_le_bytes();
    let b = 0x0000_00BBu32.to_le_bytes();
    let chain = c.step(&cycle, &[&a, &b]).expect("step");
    assert_eq!(chain.timeout_slot, None);
    let echoed: Vec<&[u8]> = chain.statuses.iter().map(|s| &s.payload[..]).collect();
    assert_eq!(echoed, vec![&a[..], &b[..]]);
}

#[test]
fn cycle_step_rejects_a_malformed_payload_batch() {
    let mut c = fleet(&[1]);
    let cycle = Cycle::new(
        vec![Group {
            addr: GOAL_VELOCITY,
            count: 4,
            ids: vec![Id::new(1)],
        }],
        Telemetry::Span {
            ids: vec![Id::new(1)],
            addr: GOAL_VELOCITY,
            count: 4,
        },
    );
    match c.step(&cycle, &[]) {
        Err(Error::Servo(osc_client::ResultCode::Range)) => {}
        other => panic!("expected Range on a missing slot, got {other:?}"),
    }
    match c.step(&cycle, &[&[0u8; 2][..]]) {
        Err(Error::Servo(osc_client::ResultCode::Range)) => {}
        other => panic!("expected Range on a narrow slice, got {other:?}"),
    }
}

#[test]
fn cycle_profile_telemetry_streams_the_slot() {
    let mut c = fleet(&[5]);
    let ids = vec![Id::new(5)];
    // Slot 0: the 4-byte goal echo plus the alarm/status pair -- scattered
    // registers one slot byte fetches per step.
    let words = [
        span_word(GOAL_VELOCITY, 4),
        span_word(table::FAULT_FLAGS, 2),
    ];
    let mut spans = Vec::new();
    for w in words {
        spans.extend_from_slice(&w.to_le_bytes());
    }
    c.write(Id::new(5), PROFILE_SLOT0, &spans)
        .expect("configure slot");
    let cycle = Cycle::new(
        vec![Group {
            addr: GOAL_VELOCITY,
            count: 4,
            ids: ids.clone(),
        }],
        Telemetry::Profile { ids, slot: 0 },
    );
    let goal = 0x0000_0C0Cu32.to_le_bytes();
    let chain = c.step(&cycle, &[&goal]).expect("step");
    assert_eq!(chain.timeout_slot, None);
    let [status] = chain.statuses.as_slice() else {
        panic!("one status, got {}", chain.statuses.len());
    };
    assert_eq!(status.payload.len(), 6);
    assert_eq!(&status.payload[..4], &goal);
    // The single-target PROFILE read streams the same slot.
    let direct = c.read_profile(Id::new(5), 0).expect("read_profile");
    assert_eq!(direct, status.payload);
}

// --- wire instrument (0x6x family) ---

#[cfg(feature = "bench")]
mod wire {
    use super::*;

    #[test]
    fn wire_send_releases_and_reports_tick() {
        let mut c = fleet(&[5]);
        // A CRC-broken ping: rings at the servo, dies at its verdict, draws no
        // reply -- raw means raw, the engine never validated it.
        let t1 = c.wire_send(&[0x01, 0x03, 0x10, 0xFF, 0xFF]).expect("send");
        let t2 = c.wire_send(&[0x55, 0xAA]).expect("send again");
        assert!(t2 > t1, "release ticks advance with sim time");
    }

    #[test]
    fn engine_commands_stay_healthy_across_instrument_use() {
        let mut c = fleet(&[5]);
        c.wire_send(&[0x01, 0x03, 0x10, 0xFF, 0xFF]).expect("send");
        c.wire_pulse_low(50).expect("pulse");
        // The pulse reads as a break at the fleet (any >=10-bit low span), so
        // the servo parks on a phantom candidate -- instrument traffic obeys
        // the sec 8 pacing rule like any other garble source: one starve
        // horizon of quiet before expecting crisp turnarounds.
        c.pause(std::time::Duration::from_millis(1));
        let ping = c.ping(Id::new(5)).expect("ping after instrument ops");
        assert!(ping.model > 0);
    }

    #[test]
    fn wire_burst_chains_frames() {
        let mut c = fleet(&[5]);
        let f1 = [0x01, 0x03, 0x10, 0xFF, 0xFF];
        let f2 = [0x02, 0x03, 0x10, 0xFF, 0xFF];
        c.wire_burst(&[&f1, &f2]).expect("burst");
    }

    #[test]
    fn wire_burst_rejects_unencodable_frames_client_side() {
        let mut c = fleet(&[5]);
        let err = c.wire_burst(&[&[]]).unwrap_err();
        assert_eq!(
            err,
            Error::Link(LinkError::Rejected(RejectReason::Malformed))
        );
        let long = vec![0u8; 256];
        let err = c.wire_burst(&[&long]).unwrap_err();
        assert_eq!(
            err,
            Error::Link(LinkError::Rejected(RejectReason::Malformed))
        );
    }

    #[test]
    fn empty_wire_send_is_rejected_by_the_adapter() {
        let mut c = fleet(&[5]);
        // A bare break never raises TC, so the engine refuses to wedge on it --
        // pinned here as the round-trip REJECTED path.
        let err = c.wire_send(&[]).unwrap_err();
        assert_eq!(
            err,
            Error::Link(LinkError::Rejected(RejectReason::Malformed))
        );
    }

    #[test]
    fn edge_drain_and_reset_round_trip() {
        let mut c = fleet(&[5]);
        // The sim has no edge model (capture is silicon-only); the shape and
        // the acks are what this pins.
        let drain = c.drain_edges().expect("drain");
        assert!(drain.edges.is_empty());
        assert!(!drain.overflow);
        c.reset_capture().expect("reset");
        let drain = c.drain_edges().expect("drain after reset");
        assert!(drain.edges.is_empty());
    }

    #[test]
    fn wire_train_and_raw_baud_round_trip() {
        let mut c = fleet(&[5]);
        // A truthful CAL announce via the raw train: MGMT CAL 400us x 3 gaps.
        let mut p = [0u8; 8];
        let n = osc_protocol::build::mgmt_cal(&mut p, 400, 3).expect("cal payload");
        let announce = {
            // Frame it as wire bytes: the raw verb takes the whole frame.
            let mut f = vec![0xFE, (n + 3) as u8, 0x70];
            f.extend_from_slice(&p[..n]);
            let crc = osc_protocol::crc::osc_crc(&f);
            f.extend_from_slice(&crc.to_le_bytes());
            f
        };
        c.wire_train(&announce, 400, 4).expect("train");

        // Raw baud: one BRR step off 1M models as the nearest catalog rate in
        // the sim; the verb completes and the engine stays healthy.
        c.wire_baud(993_103).expect("detune");
        c.pause(std::time::Duration::from_millis(2));
        let ping = c.ping(Id::new(5)).expect("ping after detune");
        assert!(ping.model > 0);
        c.wire_baud(1_000_000).expect("restore");
    }
}

// --- TEL stream (burst carrier) ---

/// V006 map facts (control.lifecycle): the burst arm registers.
const TEL_MASK: u16 = 386;
const TEL_COUNT: u16 = 402;
/// pos + current + duty + vdiff -- the ident ladder's mask.
const TEL_LADDER_MASK: u16 = 0x1B;
const BURST_WINDOW: Duration = Duration::from_millis(50);

/// TEL runs at 3M: at lower rates the producer outruns the wire and drops
/// samples by design (see the DES tel suite).
fn tel_fleet() -> Client<FakePipe> {
    Client::connect(FakePipe::new(BaudRate::B3000000, &[5])).expect("connect")
}

fn write_mask(c: &mut Client<FakePipe>) {
    c.write(Id::new(5), TEL_MASK, &TEL_LADDER_MASK.to_le_bytes())
        .expect("mask");
}

/// Arm the burst: a stream-tagged unicast WRITE to tel_count.
fn arm_tel(c: &mut Client<FakePipe>, count: u16) -> StreamReply {
    let mut p = [0u8; 8];
    let n = osc_protocol::build::write(&mut p, TEL_COUNT, &count.to_le_bytes()).expect("payload");
    let inst = Inst::instruction(Opcode::Write, 0);
    c.exchange_stream(Id::new(5), inst, &p[..n], BURST_WINDOW)
        .expect("stream exchange")
}

#[test]
fn tel_stream_collects_ack_frames_and_last() {
    let mut c = tel_fleet();
    write_mask(&mut c);
    let reply = arm_tel(&mut c, 40);
    assert_eq!(reply.outcome, Outcome::Complete);
    let ack = reply.ack.expect("acked WRITE arm");
    assert_eq!(ack.result, Some(ResultCode::Ok));
    assert_eq!(reply.frames.len(), 3, "40 samples = 16 + 16 + 8");
    for (i, f) in reply.frames.iter().enumerate() {
        assert_eq!(f.result, Some(ResultCode::Stream));
        assert_eq!(
            f.payload,
            expect_tel_payload(TEL_LADDER_MASK, 40, i),
            "frame {i} payload"
        );
    }
    assert_eq!(reply.statuses, 4, "ack + three frames");
    assert_eq!(reply.garble, 0);
    assert!(!reply.trailing);

    // The line frees after LAST: an ordinary read still answers.
    let got = c.read(Id::new(5), TEL_MASK, 2).expect("read after burst");
    assert_eq!(got, TEL_LADDER_MASK.to_le_bytes());
}

#[test]
fn reopen_mid_burst_orphans_it_and_the_next_session_starts_clean() {
    let mut c = tel_fleet();
    write_mask(&mut c);
    let mut pipe = c.into_pipe();

    // A host arms a burst and vanishes before reading a record.
    let mut p = [0u8; 8];
    let n = osc_protocol::build::write(&mut p, TEL_COUNT, &40u16.to_le_bytes()).expect("payload");
    let arm = EngineCommand::ExchangeStream {
        id: Id::new(5),
        inst: Inst::instruction(Opcode::Write, 0),
        payload: &p[..n],
        window_us: BURST_WINDOW.as_micros() as u32,
    };
    let mut out = Vec::new();
    Session::new().encode_submit(&mut out, &arm);
    pipe.sim_mut().link_send(&out);
    pipe.reopen();

    // The next session starts clean: INFO first, and its seq 0 is refused
    // busy while the orphan still owns the bus.
    let mut s = Session::new();
    let mut out = Vec::new();
    Session::encode_hello(&mut out);
    let ping = s.encode_submit(
        &mut out,
        &EngineCommand::Exchange {
            id: Id::new(5),
            inst: Inst::instruction(Opcode::Ping, 0),
            payload: &[],
        },
    );
    pipe.sim_mut().link_send(&out);
    s.on_bytes(&pipe.sim_mut().link_recv());
    assert!(matches!(s.next_record(), Ok(Some(Record::Info(_)))));
    assert_eq!(
        s.next_record(),
        Ok(Some(Record::Rejected {
            seq: ping,
            reason: osc_client::record::REASON_BUSY
        }))
    );

    // The orphan plays out on the wire, unrecorded.
    let frames = pipe.sim_mut().run();
    let burst = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
        .count();
    assert_eq!(burst, 3, "40 samples = 16 + 16 + 8");
    assert!(pipe.sim_mut().link_recv().is_empty());

    let mut c = Client::connect(pipe).expect("connect");
    c.ping(Id::new(5)).expect("the bus is free");
}

#[test]
fn tel_stream_hold_commit_broadcast_carrier() {
    let mut c = tel_fleet();
    c.write_hold(Id::new(5), TEL_MASK, &TEL_LADDER_MASK.to_le_bytes())
        .expect("hold mask");
    c.write_hold(Id::new(5), TEL_COUNT, &24u16.to_le_bytes())
        .expect("hold count");
    let inst = Inst::instruction(Opcode::Commit, 0);
    let reply = c
        .exchange_stream(Id::BROADCAST, inst, &[], BURST_WINDOW)
        .expect("commit stream");
    assert_eq!(reply.outcome, Outcome::Complete);
    assert!(reply.ack.is_none(), "broadcast COMMIT owes no ack");
    assert_eq!(reply.frames.len(), 2, "24 samples = 16 + 8");
    for (i, f) in reply.frames.iter().enumerate() {
        assert_eq!(
            f.payload,
            expect_tel_payload(TEL_LADDER_MASK, 24, i),
            "frame {i} payload"
        );
    }
    assert_eq!(reply.statuses, 2);
}

#[test]
fn tel_stream_alert_marks_the_faulted_batch() {
    let mut c = tel_fleet();
    c.pipe_mut().sim_mut().set_tel_fault_ticks(0, 16, 32);
    write_mask(&mut c);
    let reply = arm_tel(&mut c, 48);
    assert_eq!(reply.outcome, Outcome::Complete);
    assert!(!reply.ack.expect("ack").alert);
    let alerts: Vec<bool> = reply.frames.iter().map(|f| f.alert).collect();
    assert_eq!(alerts, [false, true, false], "ALERT on the faulted batch");
}

#[test]
fn tel_stream_drops_a_corrupt_frame_as_garble() {
    // Probe run: the sim is deterministic, so frame 1's wire span in a
    // clean run holds for the injected rerun.
    let garble_at_us = {
        let mut c = tel_fleet();
        write_mask(&mut c);
        c.pipe_mut().take_frames();
        let reply = arm_tel(&mut c, 48);
        assert_eq!(reply.frames.len(), 3);
        let tpu = c.info().ticks_per_us as u64;
        let frames = c.pipe_mut().take_frames();
        let f1 = frames
            .iter()
            .filter(|f| {
                matches!(f.from, Source::Servo(_))
                    && status(f).0.result() == Some(ResultCode::Stream)
            })
            .nth(1)
            .expect("frame 1 recorded");
        (f1.at + f1.end) / 2 / tpu
    };

    let mut c = tel_fleet();
    write_mask(&mut c);
    c.pipe_mut().sim_mut().inject_garble_at(garble_at_us, 0xA5);
    let reply = arm_tel(&mut c, 48);
    assert_eq!(
        reply.outcome,
        Outcome::Complete,
        "LAST still closes the burst"
    );
    assert!(reply.garble > 0, "the corrupt frame is evidence");
    let seqs: Vec<u8> = reply.frames.iter().map(|f| f.payload[0]).collect();
    assert_eq!(
        seqs,
        [0, 2],
        "corrupt frame dropped; the seq gap exposes it"
    );
    assert_eq!(reply.statuses, 3, "ack + two clean frames");
    assert_eq!(
        reply.frames[1].payload,
        expect_tel_payload(TEL_LADDER_MASK, 48, 2)
    );
}
