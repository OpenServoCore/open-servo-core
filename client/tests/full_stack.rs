//! Client -> records -> production LinkServer -> production engine -> sim
//! wire -> production servo stack, all in-process (fake-adapter backend).
//! Exercised through the blocking facade so both wrappers stay covered.

#![cfg(feature = "fake-adapter")]

use std::time::Duration;

use osc_client::blocking::Client;
use osc_client::boot::{
    BootInfo, Chosen, Device, Fate, Image, InstallError, Phase, Progress, Report, ResetFlags,
    Selected, Status, WriteFlags,
};
use osc_client::calib::GearClass;
use osc_client::common::{Health, Identity};
use osc_client::cyclic::{Cycle, Group, Telemetry};
use osc_client::data_state::{
    self, CALIB_VIRGIN, CONFIG_VIRGIN, PLANT_UNSET, Reason, STAMP_MISMATCH, fault,
};
use osc_client::descriptor::{Descriptor, Kind, Value, encode};
use osc_client::fake::{FakePipe, TelSample, TelemetryTick, Therm, seed};
use osc_client::mgmt::{Found, Uid};
use osc_client::pipe::{Pipe, PipeError};
use osc_client::pos_lut::{self, INTERVALS, LutError, PosLut};
use osc_client::session::{EngineCommand, LinkInfo, Record, Session};
use osc_client::{
    BaudRate, BusMode, Error, Id, Inst, LinkError, Opcode, Outcome, RejectReason, ResultCode,
    StreamReply, TbExpect,
};
use osc_integration::sim::{Source, expect_tel_payload, status};
use osc_protocol::models::MODEL_OSC_SERVO;
use osc_protocol::table;
use osc_protocol::tel::{Chunk, Chunks};
use osc_protocol::wire::{DEFAULT_RESPONSE_DEADLINE_US, UID_LEN};
use osc_servo_core::estimator::thermal::{UNSET_CC, flag};
use osc_servo_core::regions::config::addr::limits::CURRENT_LIMIT_COUNTS;
use osc_servo_core::regions::telemetry::addr::estimates::{I_LIM_COUNTS, T_WINDING_CC};
use osc_servo_core::regions::telemetry_ext::addr::therm::{T_NTC_CC, THERM_FLAGS};

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
/// calib.sense.shunt_r_mohm, and config.pos_soft.pos_max_soft_counts.
const RAW_MAX: u16 = 162;
const ANGLE_MAX_CDEG: u16 = 166;
const GEAR_RATIO_CENTI: u16 = 168;
const SHUNT_R_MOHM: u16 = 172;
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
    use osc_host::link::record as rec;
    let c = fleet(&[1]);
    let info = c.info();
    assert!(info.ticks_per_us > 0);
    // The sim stands in for a LinkE.
    let instrument = if cfg!(feature = "bench") {
        rec::FEATURE_INSTRUMENT
    } else {
        0
    };
    assert_eq!(
        info.features,
        rec::FEATURE_RAILS | rec::FEATURE_ADAPTER_IAP | rec::FEATURE_TINYBOOT | instrument
    );
    assert_eq!(info.max_request as usize, rec::MAX_REQUEST);
    assert_eq!(info.max_response as usize, rec::MAX_RESPONSE);
    assert_eq!(
        info.max_tb_payload as usize,
        osc_host::engine::MAX_TB_PAYLOAD
    );
    assert_eq!(info.mode, rec::MODE_OSC);
    assert_eq!(info.bps, 1_000_000);
    assert_eq!(info.deadline_us, 1000);
    // A chip-less adapter has nothing to report.
    assert_eq!(info.diag, Default::default());
}

#[test]
fn adapter_errors_surface_as_their_reject_reason() {
    let mut c = fleet(&[5]);
    // An ERROR on the active seq names its code; the session stays in step.
    let read = Inst::instruction(Opcode::Read, 0);
    assert_eq!(
        c.exchange(Id::new(5), read, &[1]).unwrap_err(),
        Error::Link(LinkError::Rejected(RejectReason::BadPayload))
    );
    let status = Inst::status(ResultCode::Ok, false);
    assert_eq!(
        c.exchange(Id::new(5), status, &[]).unwrap_err(),
        Error::Link(LinkError::Rejected(RejectReason::BadInst))
    );
    c.ping(Id::new(5)).expect("still in step");

    // A framing breach answers on no seq; the client surfaces it as the
    // adapter refusing its request, and only a new session recovers.
    let mut pipe = c.into_pipe();
    pipe.sim_mut().link_send(&[1, 0]);
    assert_eq!(
        Client::connect(pipe).err(),
        Some(Error::Link(LinkError::Rejected(RejectReason::Framing)))
    );
}

#[test]
fn a_new_session_recovers_from_a_framing_breach() {
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[5]);
    pipe.sim_mut().link_send(&[0xFF, 0xFF]);
    pipe.reopen();
    let mut c = Client::connect(pipe).expect("connect");
    c.ping(Id::new(5)).expect("ping");
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
    assert_eq!(c.info().bps, BaudRate::RESCUE.as_hz());
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
    let d = descriptor();
    assert_eq!(
        c.gear_class(id, &d).expect("gear class"),
        GearClass::Plastic
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
        c.gear_class(id, &d).expect("gear class"),
        GearClass::Unknown
    );
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

/// GET_INFO sent beside the client's own session, so its answer is the
/// adapter's view rather than the client's cached copy.
fn fresh_info(c: &mut Client<FakePipe>) -> LinkInfo {
    let mut s = Session::new();
    let mut out = Vec::new();
    s.encode_get_info(&mut out);
    let sim = c.pipe_mut().sim_mut();
    sim.link_send(&out);
    sim.run();
    s.on_bytes(&sim.link_recv());
    let Ok(Some(Record::Done { result, .. })) = s.next_record() else {
        panic!("no INFO");
    };
    LinkInfo::parse(&result).expect("INFO parses")
}

const WRONG_MODE: Error = Error::Link(LinkError::Rejected(RejectReason::WrongMode));
const TB_WINDOW: Duration = Duration::from_millis(5);

#[test]
fn set_bus_moves_the_bus_to_tinyboot_and_back() {
    use osc_host::link::record as rec;
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 115_200).expect("to tinyboot");
    assert_eq!((c.info().mode, c.info().bps), (rec::MODE_TINYBOOT, 115_200));
    let fresh = fresh_info(&mut c);
    assert_eq!((fresh.mode, fresh.bps), (rec::MODE_TINYBOOT, 115_200));

    c.set_bus(BusMode::Osc, 2_000_000).expect("back to osc");
    assert_eq!((c.info().mode, c.info().bps), (rec::MODE_OSC, 2_000_000));
    let fresh = fresh_info(&mut c);
    assert_eq!((fresh.mode, fresh.bps), (rec::MODE_OSC, 2_000_000));
}

#[test]
fn tinyboot_mode_refuses_osc_exchanges() {
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 115_200).expect("to tinyboot");
    assert_eq!(c.ping(Id::new(1)).unwrap_err(), WRONG_MODE);
}

#[test]
fn osc_mode_refuses_tb_exchange() {
    let mut c = fleet(&[1]);
    assert_eq!(
        c.tb_exchange(TbExpect::Collect, TB_WINDOW, 0, 0, 0, &[])
            .unwrap_err(),
        WRONG_MODE
    );
}

#[test]
fn tb_collect_with_no_device_completes_empty() {
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 115_200).expect("to tinyboot");
    let r = c
        .tb_exchange(TbExpect::Collect, TB_WINDOW, 0, 0, 0, &[])
        .expect("collect");
    assert_eq!(r.outcome, Outcome::Complete);
    assert!(r.frames.is_empty());
    assert_eq!(r.garble, 0);
}

#[test]
fn tb_expect_one_with_no_device_times_out() {
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 115_200).expect("to tinyboot");
    let r = c
        .tb_exchange(TbExpect::One, TB_WINDOW, 0, 0, 0, &[])
        .expect("one");
    assert!(matches!(r.outcome, Outcome::Timeout { .. }), "{r:?}");
    assert!(r.frames.is_empty());
}

#[test]
fn tb_expect_none_is_sent() {
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 115_200).expect("to tinyboot");
    let r = c
        .tb_exchange(
            TbExpect::None,
            Duration::ZERO,
            3,
            0x0008_0400,
            0x01,
            &[0xDE, 0xAD],
        )
        .expect("none");
    assert_eq!(r.outcome, Outcome::Sent);
    assert!(r.frames.is_empty());
}

#[test]
fn tb_addr_past_24_bits_is_refused_unsent() {
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 115_200).expect("to tinyboot");
    assert_eq!(
        c.tb_exchange(TbExpect::None, Duration::ZERO, 3, 0x0100_0000, 0, &[])
            .unwrap_err(),
        Error::Link(LinkError::Rejected(RejectReason::Malformed))
    );
    c.tb_exchange(TbExpect::None, Duration::ZERO, 3, 0x00FF_FFFF, 0, &[])
        .expect("24-bit addr");
}

#[test]
fn boot_info_with_no_device_times_out() {
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 500_000).expect("to tinyboot");
    assert_eq!(c.boot_info().unwrap_err(), Error::Timeout { slot: 0 });
}

#[test]
fn boot_select_with_no_device_finds_none() {
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 500_000).expect("to tinyboot");
    assert_eq!(
        c.boot_select(0, &[], 0).expect("select"),
        Selected {
            ids: vec![],
            garble: 0,
        }
    );
}

/// An image for the fake bootloader, its tail `[compat, version]`.
fn boot_image(len: usize, compat: &str, version: u16) -> Vec<u8> {
    let mut img: Vec<u8> = (0..len).map(|i| (i * 7) as u8).collect();
    img[len - 6..len - 2].copy_from_slice(&tinyboot_protocol::compat_key(compat).to_le_bytes());
    img[len - 2..].copy_from_slice(&version.to_le_bytes());
    img
}

/// The fake adapter, counting the records the client sends.
struct Counted {
    pipe: FakePipe,
    sends: usize,
}

impl Pipe for Counted {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        self.sends += 1;
        self.pipe.send(bytes).await
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        self.pipe.recv().await
    }

    async fn pause(&mut self, d: Duration) {
        self.pipe.pause(d).await
    }
}

/// Servos `uids` wait in their bootloaders, the bus in tinyboot at 0.5 M.
fn boot_fleet(uids: &[[u8; UID_LEN]]) -> Client<Counted> {
    let ids: Vec<u8> = (1..=uids.len() as u8).collect();
    let pipe = FakePipe::new(BaudRate::B1000000, &ids);
    let mut c = Client::connect(Counted { pipe, sends: 0 }).expect("connect");
    let sim = c.pipe_mut().pipe.sim_mut();
    for (i, &uid) in uids.iter().enumerate() {
        sim.seed_servo_uid(i, uid);
        sim.start_in_bootloader(i);
    }
    c.set_bus(BusMode::Tinyboot, 500_000).expect("to tinyboot");
    c
}

#[test]
fn firmware_update_round_trip() {
    let id = Id::new(1);
    let mut c = fleet(&[1]);
    c.reboot_bootloader(id, BaudRate::B500000)
        .expect("to bootloader");
    c.set_bus(BusMode::Tinyboot, 500_000).expect("to tinyboot");
    let esig = c.pipe_mut().sim_mut().servo_uid(0)[..12].to_vec();
    assert_eq!(
        c.boot_select(0, &esig, 96).expect("select"),
        Selected {
            ids: vec![esig],
            garble: 0,
        }
    );
    assert_eq!(
        c.boot_info().expect("info"),
        BootInfo {
            capacity: 58_112,
            erase_size: 256,
            boot_version: tinyboot_protocol::pack_version(0, 1, 0),
            app_version: 0xFFFF,
            mode: 0,
            boot_baud: tinyboot_protocol::BaudRate::B500000 as u8,
            compat: tinyboot_protocol::compat_key("osc-dev-v006"),
            max_payload: 240,
        }
    );

    let v = tinyboot_protocol::pack_version(1, 2, 3);
    let img = boot_image(298, "osc-dev-v006", v);
    c.boot_erase(0, 512).expect("erase");
    c.boot_write(0, &img[..240], WriteFlags::empty())
        .expect("write");
    c.boot_write(240, &img[240..], WriteFlags::FLUSH)
        .expect("flush");
    let crc = tinyboot_protocol::crc::crc16(tinyboot_protocol::crc::CRC_INIT, &img);
    assert_eq!(c.boot_verify(298).expect("verify"), crc);
    c.boot_reset(ResetFlags::empty(), None).expect("to app");

    c.set_bus(BusMode::Osc, 1_000_000).expect("to osc");
    assert_eq!(c.ping(id).expect("ping").fw, v);
}

/// Slots 4 and 14 for seed 0 (`crc16(esig) & 15`): the replies stand ten
/// units of 96 bit-times apart, clean.
#[test]
fn boot_select_hears_two_devices_in_their_slots() {
    let (a, b) = ([1; UID_LEN], [3; UID_LEN]);
    let mut c = boot_fleet(&[a, b]);
    assert_eq!(
        c.boot_select(0, &[], 0).expect("select"),
        Selected {
            ids: vec![a[..12].to_vec(), b[..12].to_vec()],
            garble: 0,
        }
    );
}

/// Both draw slot 0 for seed 0: one span of colliding energy.
#[test]
fn boot_select_same_slot_garbles() {
    let mut c = boot_fleet(&[[5; UID_LEN], [8; UID_LEN]]);
    let sel = c.boot_select(0, &[], 0).expect("select");
    assert!(sel.ids.is_empty() && sel.garble > 0, "{sel:?}");
}

#[test]
fn boot_refuses_a_foreign_image_and_a_reset_mid_update() {
    let uid = [0x42; UID_LEN];
    let mut c = boot_fleet(&[uid]);
    c.boot_select(0, &uid[..12], 96).expect("select");
    let img = boot_image(26, "another-board", 1);
    c.boot_erase(0, 256).expect("erase");
    c.boot_write(0, &img, WriteFlags::FLUSH).expect("write");
    assert_eq!(
        c.boot_verify(26).unwrap_err(),
        Error::Boot(Status::Incompatible)
    );
    assert_eq!(
        c.boot_reset(ResetFlags::empty(), None).unwrap_err(),
        Error::Boot(Status::Unsupported)
    );
}

const BOARD: &str = "osc-dev-v006";

/// `len` bytes for the fake bootloader, parsed.
fn image(len: usize, compat: &str, version: u16) -> Image {
    Image::parse(boot_image(len, compat, version)).expect("image")
}

/// The app version device `uid`'s Info reports.
fn app_version(c: &mut Client<Counted>, uid: &[u8]) -> u16 {
    let bits = (uid.len() * 8) as u8;
    c.boot_select(0, uid, bits).expect("select");
    c.boot_info().expect("info").app_version
}

/// Five writes over four pages: the target holds the image and boots it,
/// its neighbour still waits with an empty app.
#[test]
fn boot_install_writes_one_device_and_leaves_the_other() {
    let (a, b) = ([1; UID_LEN], [3; UID_LEN]);
    let mut c = boot_fleet(&[a, b]);
    let v = tinyboot_protocol::pack_version(1, 2, 3);
    let img = image(1002, BOARD, v);
    let mut seen = Vec::new();
    c.boot_install(&a[..12], &img, |w, t| seen.push((w, t)))
        .expect("install");
    assert_eq!(seen, [240, 480, 720, 960, 1002].map(|w| (w, 1002)).to_vec());
    assert_eq!(app_version(&mut c, &b[..12]), 0xFFFF, "untouched");
    assert_eq!(app_version(&mut c, &a[..12]), v);

    c.boot_reset(ResetFlags::empty(), None).expect("to app");
    c.set_bus(BusMode::Osc, 1_000_000).expect("to osc");
    assert_eq!(c.ping(Id::new(1)).expect("ping").fw, v);
    assert!(c.pipe_mut().pipe.sim_mut().in_bootloader(1));
}

/// A foreign or oversized image is refused before the erase: the device
/// still holds its verified app, which an erase would have dropped.
#[test]
fn boot_install_refuses_a_foreign_or_oversized_image_unerased() {
    let uid = [0x42; UID_LEN];
    let mut c = boot_fleet(&[uid]);
    let v = tinyboot_protocol::pack_version(1, 0, 0);
    c.boot_install(&uid[..12], &image(26, BOARD, v), |_, _| {})
        .expect("install");

    let foreign = image(26, "another-board", 2);
    assert_eq!(
        c.boot_install(&uid[..12], &foreign, |_, _| panic!("written")),
        Err(Error::Install(InstallError::Incompatible {
            image: tinyboot_protocol::compat_key("another-board"),
            device: tinyboot_protocol::compat_key(BOARD),
        }))
    );
    assert_eq!(app_version(&mut c, &uid[..12]), v);

    let huge = image(58_114, BOARD, 2);
    assert_eq!(
        c.boot_install(&uid[..12], &huge, |_, _| panic!("written")),
        Err(Error::Install(InstallError::TooLarge {
            size: 58_114,
            capacity: 58_112,
        }))
    );
    assert_eq!(app_version(&mut c, &uid[..12]), v);
}

#[test]
fn boot_install_returns_the_device_status() {
    let uid = [0x42; UID_LEN];
    let mut c = boot_fleet(&[uid]);
    let img = image(1002, BOARD, 1);
    for cmd in [tinyboot_protocol::Cmd::Erase, tinyboot_protocol::Cmd::Write] {
        let sim = c.pipe_mut().pipe.sim_mut();
        sim.fail_next_boot(0, cmd, Status::WriteError);
        assert_eq!(
            c.boot_install(&uid[..12], &img, |_, _| {}),
            Err(Error::Boot(Status::WriteError)),
            "{cmd:?}"
        );
    }
}

#[test]
fn boot_install_on_an_absent_id_finds_no_device() {
    let mut c = boot_fleet(&[[0x42; UID_LEN]]);
    assert_eq!(
        c.boot_install(&[0x43; 12], &image(26, BOARD, 1), |_, _| {}),
        Err(Error::Install(InstallError::NoDevice))
    );
}

#[test]
fn boot_discover_on_an_empty_bus_finds_nothing() {
    let mut c = boot_fleet(&[]);
    assert_eq!(c.boot_discover().expect("discover"), Vec::<Vec<u8>>::new());
}

#[test]
fn boot_discover_finds_one_device() {
    let uid = [0x42; UID_LEN];
    let mut c = boot_fleet(&[uid]);
    assert_eq!(
        c.boot_discover().expect("discover"),
        vec![uid[..12].to_vec()]
    );
}

/// The prefixes two or more ids share, the empty one included: the walk's
/// collisions.
fn shared_prefixes(ids: &[&[u8]]) -> usize {
    let bit = |id: &[u8], k: usize| id[k / 8] >> (k % 8) & 1;
    (0..=ids[0].len() * 8)
        .map(|len| {
            let mut keys: Vec<Vec<u8>> = ids
                .iter()
                .map(|id| (0..len).map(|k| bit(id, k)).collect())
                .collect();
            keys.sort();
            keys.chunk_by(|a, b| a == b)
                .filter(|run| run.len() > 1)
                .count()
        })
        .sum()
}

/// The ids that share all but their last bit with another: the walk finds
/// them by their whole length.
fn whole_length(ids: &[&[u8]]) -> usize {
    let bit = |id: &[u8], k: usize| id[k / 8] >> (k % 8) & 1;
    let common = |a: &[u8], b: &[u8]| {
        (0..a.len() * 8)
            .take_while(|&k| bit(a, k) == bit(b, k))
            .count()
    };
    ids.iter()
        .filter(|a| ids.iter().any(|b| common(a, b) + 1 == a.len() * 8))
        .count()
}

/// Seven devices: `a` and `b` share slot 0 for seed 0 and garble, `c` and
/// `d` answer clean in slots 4 and 14; `e` and `f` share 95 bits, `e` and
/// `g` 40. On a clean bus the walk is a full binary tree over the `S`
/// shared prefixes: each collides and splits, `N` leaves find an id, the
/// other `S + 1 - N` stay empty for two probes each, `3S + 2 - N`. Every
/// leaf but the `W` found by their whole length (`e` and `f`) costs three
/// confirm probes more: its own child answers, the other stays empty
/// twice.
#[test]
fn boot_discover_walks_a_fleet_in_the_fewest_probes() {
    let mut e = [0x42; UID_LEN];
    e[12..].fill(0);
    let (mut f, mut g) = (e, e);
    f[11] ^= 0x80;
    g[5] ^= 0x01;
    let uids = [
        [5; UID_LEN],
        [8; UID_LEN],
        [1; UID_LEN],
        [3; UID_LEN],
        e,
        f,
        g,
    ];
    let mut c = boot_fleet(&uids);
    let mut want: Vec<Vec<u8>> = uids.iter().map(|u| u[..12].to_vec()).collect();
    want.sort();

    let root = c.boot_select(0, &[], 0).expect("select");
    assert!(root.garble > 0, "{root:?}");
    assert!(
        root.ids.contains(&want[0]) && root.ids.contains(&want[1]),
        "{root:?}"
    );

    c.pipe_mut().sends = 0;
    assert_eq!(c.boot_discover().expect("discover"), want);
    let ids: Vec<&[u8]> = want.iter().map(Vec::as_slice).collect();
    let shared = shared_prefixes(&ids);
    let whole = whole_length(&ids);
    assert_eq!(whole, 2);
    assert_eq!(
        c.pipe_mut().sends,
        3 * shared + 2 + 2 * ids.len() - 3 * whole
    );
}

/// `v` and `w` share 18 bits, `x` the first 17. The probe of those 17 bits
/// draws all three, `v`'s reply overlapping another, so a `v` deaf after a
/// collision misses the next probe, of the 18 bits, and `w` answers it
/// alone. The confirm of that lone reply hears `v` under the other child.
#[test]
fn boot_discover_confirms_a_lone_reply_past_a_deaf_device() {
    let mut v = [0u8; UID_LEN];
    v[3..11].fill(0x5A);
    let (mut w, mut x) = (v, v);
    w[2] = 0b100;
    w[11] = 0x33;
    x[2] = 0b010;
    x[11] = 0x77;
    let mut want: Vec<Vec<u8>> = [v, w, x].iter().map(|u| u[..12].to_vec()).collect();
    want.sort();

    let mut c = boot_fleet(&[v, w, x]);
    c.pipe_mut().pipe.sim_mut().boot_deaf_after_collision(0);
    assert_eq!(c.boot_discover().expect("discover"), want);
    let sim = c.pipe_mut().pipe.sim_mut();
    let mut heard = |i| -> Vec<u64> { sim.boot_log(i).iter().map(|t| t.request_end).collect() };
    let (heard_v, heard_w) = (heard(0), heard(1));
    assert!(heard_v.contains(&heard_w[17]), "v answers the 17-bit probe");
    assert!(!heard_v.contains(&heard_w[18]), "v misses the 18-bit probe");

    let mut c = boot_fleet(&[v, w, x]);
    assert_eq!(c.boot_discover().expect("discover"), want);
}

/// Servo `i`'s tinyboot id: its ESIG, the osc UID's first 12 bytes.
fn chip_uid(c: &mut Client<FakePipe>, i: usize) -> Vec<u8> {
    c.pipe_mut().sim_mut().servo_uid(i)[..12].to_vec()
}

fn dev(uid: &[u8], id: Option<u8>, fate: Fate) -> Device {
    Device {
        uid: uid.to_vec(),
        id: id.map(Id::new),
        fate,
    }
}

fn fws(c: &mut Client<FakePipe>, ids: &[u8]) -> Vec<u16> {
    ids.iter()
        .map(|&id| c.ping(Id::new(id)).expect("ping").fw)
        .collect()
}

/// The host sent no tinyboot byte after the first bootloader handed off to
/// its app: no app heard one.
fn assert_no_tinyboot_after_handoff(c: &mut Client<FakePipe>) {
    let (handoff, tinyboot) = c.pipe_mut().sim_mut().tinyboot_handoff();
    let (handoff, tinyboot) = (handoff.expect("a hand-off"), tinyboot.expect("tinyboot"));
    assert!(
        tinyboot <= handoff,
        "tinyboot until {tinyboot}, hand-off at {handoff}"
    );
}

fn update(c: &mut Client<FakePipe>, img: &Image, chosen: Chosen) -> Report {
    c.boot_update(img, &chosen, BaudRate::B500000, |_| {})
        .expect("update")
}

#[test]
fn update_installs_the_chosen_and_returns_the_fleet() {
    let mut c = fleet(&[1, 2, 3]);
    let old = fws(&mut c, &[3])[0];
    let v = tinyboot_protocol::pack_version(1, 2, 3);
    let (a, b) = (chip_uid(&mut c, 0), chip_uid(&mut c, 1));
    let (mut phases, mut done) = (Vec::new(), Vec::new());
    let report = c
        .boot_update(
            &image(1002, BOARD, v),
            &Chosen::Ids(vec![Id::new(1), Id::new(2)]),
            BaudRate::B500000,
            |p| match p {
                Progress::Phase(p) => phases.push(p),
                Progress::Bytes {
                    uid,
                    written,
                    total,
                } if written == total => done.push((uid.to_vec(), total)),
                Progress::Bytes { .. } => {}
            },
        )
        .expect("update");
    assert_eq!(
        report,
        Report {
            devices: vec![
                dev(&a, Some(1), Fate::Confirmed),
                dev(&b, Some(2), Fate::Confirmed),
            ],
        }
    );
    use Phase::*;
    assert_eq!(phases, [Enter, Walk, Install, Reset, Confirm]);
    assert_eq!(done, [(a, 1002), (b, 1002)]);
    assert_no_tinyboot_after_handoff(&mut c);
    assert_eq!(c.info().bps, 1_000_000);
    assert_eq!(fws(&mut c, &[1, 2, 3]), [v, v, old]);
}

/// A bootloader off the roster belongs to the step commands: the session
/// reports it and leaves it waiting.
#[test]
fn update_leaves_a_bootloader_off_the_roster_waiting() {
    let mut c = fleet(&[1, 2, 3]);
    c.pipe_mut().sim_mut().start_in_bootloader(2);
    let v = tinyboot_protocol::pack_version(1, 2, 3);
    let uids: Vec<Vec<u8>> = (0..3).map(|i| chip_uid(&mut c, i)).collect();
    let report = update(&mut c, &image(1002, BOARD, v), Chosen::All);
    assert_eq!(
        report.devices,
        [
            dev(&uids[2], None, Fate::Waiting),
            dev(&uids[0], Some(1), Fate::Confirmed),
            dev(&uids[1], Some(2), Fate::Confirmed),
        ]
    );
    assert!(c.pipe_mut().sim_mut().in_bootloader(2));
    assert_eq!(fws(&mut c, &[1, 2]), [v, v]);
}

#[test]
fn update_installs_only_the_chosen_ids() {
    let mut c = fleet(&[1, 2, 3]);
    let old = fws(&mut c, &[2])[0];
    let v = tinyboot_protocol::pack_version(1, 2, 3);
    let uids: Vec<Vec<u8>> = (0..3).map(|i| chip_uid(&mut c, i)).collect();
    let report = update(
        &mut c,
        &image(1002, BOARD, v),
        Chosen::Ids(vec![Id::new(1), Id::new(3)]),
    );
    assert_eq!(
        report.devices,
        [
            dev(&uids[0], Some(1), Fate::Confirmed),
            dev(&uids[2], Some(3), Fate::Confirmed),
        ]
    );
    assert_no_tinyboot_after_handoff(&mut c);
    assert_eq!(fws(&mut c, &[1, 2, 3]), [v, old, v]);
}

/// `apps` finds each app at its own rate and puts the bus rate back; a
/// servo off the bus rate is out of `enter`'s reach, and `apps` still
/// shows it.
#[test]
fn apps_finds_each_app_at_its_own_rate() {
    let mut c = fleet(&[1, 2]);
    let roster = c.set_baud(&[Id::new(2)], BaudRate::B3000000);
    assert!(roster.expect("migrate").iter().all(|&(_, alive)| alive));
    c.host_baud(BaudRate::B1000000).expect("host baud");
    let found = |c: &mut Client<FakePipe>, i: usize| Found {
        uid: Uid(c.pipe_mut().sim_mut().servo_uid(i)),
        id: Id::new(i as u8 + 1),
    };
    let odd = (BaudRate::B3000000, found(&mut c, 1));
    let both = vec![(BaudRate::B1000000, found(&mut c, 0)), odd];
    assert_eq!(c.boot_apps().expect("apps"), both);
    assert_eq!(c.info().bps, 1_000_000, "the sweep puts the rate back");

    c.boot_enter(BaudRate::B500000).expect("enter");
    assert!(c.pipe_mut().sim_mut().in_bootloader(0));
    assert_eq!(c.boot_apps().expect("apps"), [odd]);
}

/// The step commands' path for a fresh board: enter, list, install the
/// one with no app by its tinyboot id, reset.
#[test]
fn boot_steps_by_hand_install_a_bootloader_only_board() {
    let mut c = fleet(&[1, 2]);
    let old = fws(&mut c, &[1])[0];
    let (servo, board) = (chip_uid(&mut c, 0), chip_uid(&mut c, 1));
    let board_uid = Uid(c.pipe_mut().sim_mut().servo_uid(1));
    c.pipe_mut().sim_mut().start_in_bootloader(1);

    let roster = c.boot_enter(BaudRate::B500000).expect("enter");
    assert_eq!(
        roster.iter().map(|f| f.id).collect::<Vec<_>>(),
        [Id::new(1)]
    );

    c.set_bus(BusMode::Tinyboot, 500_000).expect("to tinyboot");
    let listed = c.boot_list().expect("list");
    let mut want = [servo.clone(), board.clone()];
    want.sort();
    assert_eq!(
        listed.iter().map(|d| &d.uid).collect::<Vec<_>>(),
        [&want[0], &want[1]]
    );
    let info = |uid: &[u8]| listed.iter().find(|d| d.uid == uid).expect("listed").info;
    assert_eq!(info(&board).app_version, 0xFFFF, "no app");

    let v = tinyboot_protocol::pack_version(1, 2, 3);
    c.boot_install(&board, &image(1002, BOARD, v), |_, _| {})
        .expect("install");
    c.boot_reset_all().expect("reset");
    c.set_bus(BusMode::Osc, 1_000_000).expect("to osc");
    c.pause(Duration::from_millis(200));

    let found = c.discover().expect("discover");
    let id_of = |uid: &Uid| found.iter().find(|f| &f.uid == uid).expect("back").id;
    assert_eq!(c.ping(id_of(&board_uid)).expect("ping").fw, v);
    assert_eq!(fws(&mut c, &[1]), [old]);
    assert!((0..2).all(|i| !c.pipe_mut().sim_mut().in_bootloader(i)));
}

#[test]
fn update_records_a_failed_install_and_goes_on() {
    let mut c = fleet(&[1, 2, 3]);
    let uids: Vec<Vec<u8>> = (0..3).map(|i| chip_uid(&mut c, i)).collect();
    let sim = c.pipe_mut().sim_mut();
    sim.fail_next_boot(1, tinyboot_protocol::Cmd::Write, Status::WriteError);
    let v = tinyboot_protocol::pack_version(1, 2, 3);
    let report = update(&mut c, &image(1002, BOARD, v), Chosen::All);
    assert_eq!(
        report.devices,
        [
            dev(
                &uids[1],
                Some(2),
                Fate::Failed(Error::Boot(Status::WriteError))
            ),
            dev(&uids[0], Some(1), Fate::Confirmed),
            dev(&uids[2], Some(3), Fate::Confirmed),
        ]
    );
    assert_eq!(fws(&mut c, &[1, 3]), [v, v]);
    let sim = c.pipe_mut().sim_mut();
    sim.power_cycle(1);
    assert!(sim.in_bootloader(1), "waits mid-update");
}

#[test]
fn update_reports_a_chosen_id_nobody_has() {
    let mut c = fleet(&[1, 2]);
    let old = fws(&mut c, &[1, 2]);
    let report = update(
        &mut c,
        &image(1002, BOARD, 7),
        Chosen::Ids(vec![Id::new(7)]),
    );
    assert_eq!(report.devices, [dev(&[], Some(7), Fate::NotFound)]);
    assert!((0..2).all(|i| !c.pipe_mut().sim_mut().in_bootloader(i)));
    assert_eq!(fws(&mut c, &[1, 2]), old);
}

#[test]
fn reconnect_returns_the_bus_to_osc() {
    use osc_host::link::record as rec;
    let mut c = fleet(&[1]);
    c.set_bus(BusMode::Tinyboot, 115_200).expect("to tinyboot");
    let mut pipe = c.into_pipe();
    pipe.reopen();
    let c = Client::connect(pipe).expect("reconnect");
    assert_eq!((c.info().mode, c.info().bps), (rec::MODE_OSC, 500_000));
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

fn sim_now_us(c: &mut Client<FakePipe>) -> u64 {
    c.pipe_mut().sim_mut().now_us()
}

#[test]
fn deadline_sync_writes_a_differing_servo_once() {
    let mut c = fleet(&[5]);
    c.pipe_mut()
        .sim_mut()
        .servo_table_mut(0, |t| t.config.common.response_deadline_us = 400);
    let before = c.pipe_mut().sim_mut().dispatched(0);
    let rewritten = c
        .sync_response_deadline(&[Id::new(5)], DEFAULT_RESPONSE_DEADLINE_US)
        .expect("sync");
    assert_eq!(rewritten, vec![(Id::new(5), 400)]);
    assert_eq!(
        c.pipe_mut().sim_mut().dispatched(0) - before,
        2,
        "one READ, one WRITE"
    );
    assert_eq!(
        read_u16(&mut c, Id::new(5), table::RESPONSE_DEADLINE_US),
        DEFAULT_RESPONSE_DEADLINE_US
    );
    assert!(c.health(Id::new(5)).expect("health").config_dirty);
}

#[test]
fn deadline_sync_leaves_a_matching_servo_clean() {
    let mut c = fleet(&[5]);
    let before = c.pipe_mut().sim_mut().dispatched(0);
    let rewritten = c
        .sync_response_deadline(&[Id::new(5)], DEFAULT_RESPONSE_DEADLINE_US)
        .expect("sync");
    assert!(rewritten.is_empty());
    assert_eq!(
        c.pipe_mut().sim_mut().dispatched(0) - before,
        1,
        "the READ only"
    );
    assert!(!c.health(Id::new(5)).expect("health").config_dirty);
}

#[test]
fn deadline_sync_sets_the_engine_mirror_with_no_servo_listed() {
    const STRETCHED_US: u16 = 5_000;
    let mut c = fleet(&[5]);
    let timeout_us = |c: &mut Client<FakePipe>| {
        let t0 = sim_now_us(c);
        assert!(matches!(c.ping(Id::new(9)), Err(Error::Timeout { .. })));
        sim_now_us(c) - t0
    };
    assert!(timeout_us(&mut c) < STRETCHED_US as u64);
    assert!(
        c.sync_response_deadline(&[], STRETCHED_US)
            .expect("sync")
            .is_empty()
    );
    assert!(timeout_us(&mut c) >= STRETCHED_US as u64);
}

// --- data state and the plant stamp ---

/// The checked-in descriptor: the stamp recipe and the register names the
/// data-state helpers resolve.
const DESCRIPTOR: &str = include_str!("../../descriptors/osc-servo/0.1.json");
/// The kernel's latched data fault (`kernel/phases/loops/faults.rs` BIT_DATA).
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
    assert_eq!(stamp.covered().len(), 40);

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
    // each covered write checkpoints: identified now, never stamped
    assert_eq!(
        c.data_state(id, &d).expect("data state").flags,
        FRESH & !PLANT_UNSET
    );

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
    fn wire_burst_fills_one_link_record_and_refuses_more() {
        let mut c = fleet(&[5]);
        assert_eq!(osc_host::link::record::WIRE_BURST_STREAM_MAX, 507);
        let full = vec![0xAAu8; 168];
        c.wire_burst(&[&full, &full, &full]).expect("507 B train");
        let over = vec![0xAAu8; 126];
        let err = c.wire_burst(&[&over, &over, &over, &over]).unwrap_err();
        assert_eq!(
            err,
            Error::Link(LinkError::Rejected(RejectReason::Malformed))
        );
        c.wire_burst(&[&full]).expect("the link is still in step");
    }

    #[test]
    fn empty_wire_send_is_rejected_by_the_adapter() {
        let mut c = fleet(&[5]);
        // A bare break never raises TC, so the engine refuses to wedge on it --
        // pinned here as the round-trip ERROR path.
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

/// V006 map facts (control.tel): the burst arm registers, the chunks right
/// ahead of the count; the tick block's kernel fields.
const CHUNK0_ADDR: u16 = 0x1E2;
const TEL_COUNT: u16 = 0x1EA;
const TICK: u16 = 0x246;
/// Duty, trough, held current and differential.
const IDENT: Chunks = Chunks::one(TICK, 8);
const BURST_WINDOW: Duration = Duration::from_millis(50);

/// TEL runs at 3M only.
fn tel_fleet() -> Client<FakePipe> {
    Client::connect(FakePipe::new(BaudRate::B3000000, &[5])).expect("connect")
}

fn write_chunks(c: &mut Client<FakePipe>, chunks: Chunks) {
    c.write(Id::new(5), CHUNK0_ADDR, &chunks.to_le_bytes())
        .expect("chunks");
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
    write_chunks(&mut c, IDENT);
    let reply = arm_tel(&mut c, 40);
    assert_eq!(reply.outcome, Outcome::Complete);
    let ack = reply.ack.expect("acked WRITE arm");
    assert_eq!(ack.result, Some(ResultCode::Ok));
    assert_eq!(reply.frames.len(), 3, "40 samples = 16 + 16 + 8");
    for (i, f) in reply.frames.iter().enumerate() {
        assert_eq!(f.result, Some(ResultCode::Stream));
        assert_eq!(
            f.payload,
            expect_tel_payload(IDENT, 40, i),
            "frame {i} payload"
        );
    }
    assert_eq!(reply.statuses, 4, "ack + three frames");
    assert_eq!(reply.garble, 0);
    assert!(!reply.trailing);

    // The line frees after LAST: an ordinary read still answers, the
    // chunks as written and the count back at 0.
    let got = c
        .read(Id::new(5), CHUNK0_ADDR, 10)
        .expect("read after burst");
    assert_eq!(got[..8], IDENT.to_le_bytes());
    assert_eq!(got[8..], [0, 0], "the servo zeroes tel_count after LAST");
}

/// Below 3M an arm acks and never streams: the stream-tagged exchange
/// completes on the ack alone.
#[test]
fn tel_arm_below_three_megabaud_streams_nothing() {
    let mut c = Client::connect(FakePipe::new(BaudRate::B2000000, &[5])).expect("connect");
    write_chunks(&mut c, IDENT);
    let reply = arm_tel(&mut c, 40);
    assert_eq!(reply.ack.expect("acked").result, Some(ResultCode::Ok));
    assert!(reply.frames.is_empty());
    let got = c.read(Id::new(5), TEL_COUNT, 2).expect("read");
    assert_eq!(got, [0, 0]);
}

#[test]
fn reopen_mid_burst_cancels_it_and_the_next_frame_waits_for_quiet() {
    let mut c = tel_fleet();
    write_chunks(&mut c, IDENT);
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

    // The next session starts clean: INFO first, and its bus request is
    // accepted at once, the cancelled burst no longer owning the bus.
    let mut s = Session::new();
    let mut out = Vec::new();
    let info = s.encode_get_info(&mut out);
    let ping = s.encode_submit(
        &mut out,
        &EngineCommand::Exchange {
            id: Id::new(5),
            inst: Inst::instruction(Opcode::Ping, 0),
            payload: &[],
        },
    );
    pipe.sim_mut().link_send(&out);
    let frames = pipe.sim_mut().run();

    // The servo still streams what the arm asked for; the ping waits out
    // the burst plus a starve horizon of silence instead of colliding.
    let burst: Vec<_> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
        .collect();
    assert_eq!(burst.len(), 3, "40 samples = 16 + 16 + 8");
    assert!(frames.iter().all(|f| !f.collided));
    let ping_at = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Host))
        .map(|f| f.at)
        .max()
        .expect("the ping went out");
    assert!(ping_at > burst.last().unwrap().end, "ping after the burst");

    s.on_bytes(&pipe.sim_mut().link_recv());
    assert!(matches!(
        s.next_record(),
        Ok(Some(Record::Done { seq, kind, .. })) if (seq, kind) == (info.seq, info.kind)
    ));
    assert!(matches!(
        s.next_record(),
        Ok(Some(Record::Frame { seq, .. })) if seq == ping.seq
    ));
    let Ok(Some(Record::Done { seq, kind, result })) = s.next_record() else {
        panic!("the ping's DONE");
    };
    assert_eq!((seq, kind), (ping.seq, ping.kind));
    assert_eq!(
        osc_host::link::record::ExchangeResult::parse(&result).map(|x| x.outcome),
        Some(Outcome::Complete)
    );
    assert_eq!(s.next_record(), Ok(None), "no burst frame leaked");
}

#[test]
fn tel_stream_hold_commit_broadcast_carrier() {
    let mut c = tel_fleet();
    let mut arm = IDENT.to_le_bytes().to_vec();
    arm.extend_from_slice(&24u16.to_le_bytes());
    c.write_hold(Id::new(5), CHUNK0_ADDR, &arm)
        .expect("hold the arm");
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
            expect_tel_payload(IDENT, 24, i),
            "frame {i} payload"
        );
    }
    assert_eq!(reply.statuses, 2);
}

/// INST ALERT is the first row's fault.
#[test]
fn tel_stream_alert_is_the_first_rows_fault() {
    let mut c = tel_fleet();
    c.pipe_mut().sim_mut().set_tel_fault_ticks(0, 20, 36);
    write_chunks(&mut c, IDENT);
    let reply = arm_tel(&mut c, 48);
    assert_eq!(reply.outcome, Outcome::Complete);
    assert!(!reply.ack.expect("ack").alert);
    let alerts: Vec<bool> = reply.frames.iter().map(|f| f.alert).collect();
    assert_eq!(alerts, [false, false, true], "ALERT where row 0 faulted");
}

#[test]
fn tel_stream_drops_a_corrupt_frame_as_garble() {
    // Probe run: the sim is deterministic, so frame 1's wire span in a
    // clean run holds for the injected rerun.
    let garble_at_us = {
        let mut c = tel_fleet();
        write_chunks(&mut c, IDENT);
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
    write_chunks(&mut c, IDENT);
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
    assert_eq!(reply.frames[1].payload, expect_tel_payload(IDENT, 48, 2));
}

/// Burst frame `seq`'s wire span in a clean probe run, in ticks, its ring
/// bytes and the tick rate: the sim is deterministic, so the span holds for
/// an injected rerun.
fn tel_frame_span(chunks: Chunks, count: u16, seq: usize) -> (u64, u64, u64, u64) {
    let mut c = tel_fleet();
    write_chunks(&mut c, chunks);
    c.pipe_mut().take_frames();
    assert_eq!(arm_tel(&mut c, count).outcome, Outcome::Complete);
    let tpu = c.info().ticks_per_us as u64;
    let f = c
        .pipe_mut()
        .take_frames()
        .into_iter()
        .filter(|f| {
            matches!(f.from, Source::Servo(_)) && status(f).0.result() == Some(ResultCode::Stream)
        })
        .nth(seq)
        .expect("frame recorded");
    (f.at, f.end, f.bytes.len() as u64, tpu)
}

/// Rows of every width a burst streams, one chunk and two.
fn row_widths() -> [Chunks; 8] {
    let vbus = 0x266;
    [
        Chunks::one(TICK, 2),
        Chunks::one(TICK, 4),
        Chunks::one(TICK, 6),
        IDENT,
        Chunks::one(TICK, 10),
        Chunks::one(TICK + 4, 12),
        Chunks::two(Chunk::new(TICK, 6), Chunk::new(TICK + 26, 2)),
        Chunks::two(Chunk::new(TICK + 6, 2), Chunk::new(vbus, 2)),
    ]
}

/// A LEN that outruns the burst must never park the walk over the clean
/// frames behind it: garble in a header corrupts LEN itself, garble in the
/// body turns a data zero into a hunt anchor whose next bytes read as a
/// header. LAST lands at every row width, burst length and garble
/// position across the second-to-last frame: each byte, and sixteenths
/// between.
#[test]
fn tel_stream_garble_never_swallows_the_next_frame() {
    for chunks in row_widths() {
        for count in [40u16, 48] {
            let (at, end, bytes, tpu) = tel_frame_span(chunks, count, 1);
            let byte = (end - at) / bytes;
            let each = (0..=bytes + 1).map(|b| (at + b * byte, format!("byte {b}")));
            let frac = (0..=16).map(|k| (at + (end - at) * k / 16, format!("{k}/16")));
            for (garble_at, pos) in each.chain(frac) {
                let w = chunks.row_bytes();
                let case = format!("{w} B count {count} garble at {pos} of frame 1");
                let mut c = tel_fleet();
                write_chunks(&mut c, chunks);
                c.pipe_mut()
                    .sim_mut()
                    .inject_garble_at(garble_at / tpu, 0xA5);
                let reply = arm_tel(&mut c, count);
                assert_eq!(reply.outcome, Outcome::Complete, "{case}");
                assert!(reply.garble > 0, "{case}");
                assert_eq!(
                    usize::from(reply.statuses),
                    reply.frames.len() + 1,
                    "{case}"
                );
                let seqs: Vec<u8> = reply.frames.iter().map(|f| f.payload[0]).collect();
                assert!(seqs == [0, 2] || seqs == [0, 1, 2], "{case}: seqs {seqs:?}");
                for f in &reply.frames {
                    let seq = usize::from(f.payload[0]);
                    let want = expect_tel_payload(chunks, u32::from(count), seq);
                    assert_eq!(f.payload, want, "{case}: frame {seq}");
                }
            }
        }
    }
}

/// A false length never holds the burst's terminal past LAST: the claim
/// dies at the next break mark, so LAST closes the burst within the mark's
/// service lag of its own wire end, never at the 50 ms window. Garble lands
/// at each byte of the second-to-last frame, every row width and length.
#[test]
fn tel_stream_garble_never_holds_last_to_the_window() {
    // LAST's own wire end, its last byte's stop sample, plus the host's
    // mark lag at budget: the terminal surfaces on that byte's event.
    let slack_us = osc_integration::sim::HOST_MARK_LAG_US + 1;
    for chunks in row_widths() {
        for count in [40u16, 48] {
            let (at, end, bytes, tpu) = tel_frame_span(chunks, count, 1);
            let byte = (end - at) / bytes;
            for b in 0..=bytes + 1 {
                let w = chunks.row_bytes();
                let case = format!("{w} B count {count} garble at byte {b} of frame 1");
                let mut c = tel_fleet();
                write_chunks(&mut c, chunks);
                c.pipe_mut().take_frames();
                c.pipe_mut()
                    .sim_mut()
                    .inject_garble_at((at + b * byte) / tpu, 0xA5);
                let reply = arm_tel(&mut c, count);
                assert_eq!(reply.outcome, Outcome::Complete, "{case}");
                let last_end = c
                    .pipe_mut()
                    .take_frames()
                    .into_iter()
                    .filter(|f| matches!(f.from, Source::Servo(_)))
                    .map(|f| f.end)
                    .max()
                    .expect("servo frames");
                let late = u64::from(reply.tick.wrapping_sub(last_end as u32));
                assert!(
                    late <= slack_us * tpu,
                    "{case}: terminal {} us after LAST",
                    late / tpu
                );
            }
        }
    }
}

/// (t_winding_cc, therm_flags, i_lim_counts, current_limit_counts) over the wire.
fn therm(c: &mut Client<FakePipe>, id: Id) -> (i16, u8, u16, u16) {
    (
        read_u16(c, id, T_WINDING_CC) as i16,
        c.read(id, THERM_FLAGS, 1).expect("read flags")[0],
        read_u16(c, id, I_LIM_COUNTS),
        read_u16(c, id, CURRENT_LIMIT_COUNTS),
    )
}

/// No CALIB thermal block - a factory-fresh servo, or a calibrated one with
/// the thermometer off - reads the firmware's sentinel and the UNSET flag,
/// and nothing derates.
#[test]
fn fake_thermometer_without_a_thermal_block_reads_unset() {
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1, 2]);
    pipe.seed_board(0);
    pipe.seed_calibrated_with(1, Therm::Off);
    let mut c = Client::connect(pipe).expect("connect");
    for id in [Id::new(1), Id::new(2)] {
        let (t, flags, i_lim, limit) = therm(&mut c, id);
        assert_eq!(t, UNSET_CC);
        assert_eq!(flags, flag::UNSET);
        assert_eq!(i_lim, limit);
    }
}

/// With the ident set stamped, a held current heats the winding over the
/// 28 C NTC toward its steady excess, g i^2 r0, on the 128 s time constant
/// (a quarter of it here: the sim stops at 60 s); below the derate start
/// i_lim is the configured limit.
#[test]
fn fake_winding_rises_on_the_time_constant_toward_the_steady_excess() {
    const HOLD_COUNTS: i16 = 280;
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
    pipe.seed_calibrated(0);
    pipe.set_track(
        0,
        vec![TelSample {
            tick: TelemetryTick {
                i_tick_counts: HOLD_COUNTS,
                ..Default::default()
            },
            ..Default::default()
        }],
    );
    let mut c = Client::connect(pipe).expect("connect");
    let id = Id::new(1);
    let t_ntc = read_u16(&mut c, id, T_NTC_CC) as i16;
    assert!((2795..=2810).contains(&t_ntc), "t_ntc_cc {t_ntc}");
    let (t0, flags, i_lim, limit) = therm(&mut c, id);
    assert_eq!(flags, 0);
    assert!((t0 - t_ntc).abs() <= 1, "boots at the NTC: {t0}");
    assert_eq!(i_lim, limit);

    c.pause(Duration::from_secs(32));
    let steady =
        seed::TH_G_Q016 as f64 / 65536.0 * (HOLD_COUNTS as f64).powi(2) * seed::R_Q12 as f64
            / 4096.0;
    let want = t_ntc as f64 - steady * (-0.25f64).exp_m1();
    let (t1, _, i_lim, _) = therm(&mut c, id);
    assert!((t1 as f64 - want).abs() < 10.0, "t {t1} want {want:.0}");
    assert_eq!(i_lim, limit);
}

/// Forced derating pins the winding midway from the derate start to the
/// cutoff, where the limit fold halves the current ceiling.
#[test]
fn fake_thermometer_forced_derating_halves_i_lim() {
    let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
    pipe.seed_calibrated_with(0, Therm::Derating);
    let mut c = Client::connect(pipe).expect("connect");
    let (t, flags, i_lim, limit) = therm(&mut c, Id::new(1));
    assert_eq!(flags, 0);
    assert_eq!(t, 9000);
    assert!(
        (i_lim as i32 - limit as i32 / 2).abs() <= 1,
        "{i_lim} of {limit}"
    );
}
