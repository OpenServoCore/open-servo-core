//! The data-state fault (`data_state` module): the runaway a closed loop
//! would run on unloaded, corrupt, stale or unidentified data is impossible
//! by construction. Boot verdicts come from the RAM store's slots through
//! the real overlay path, the kernel runs the plant rig on the table that
//! left, and SAVE/FACTORY go over the wire.

use std::cell::RefCell;
use std::rc::Rc;

use osc_integration::plant::{
    FakeIo, FakeMotor, FakeSensors, Plant, TIMING, duty_of, kernel, last_cmd, seed, stamp,
};
use osc_integration::sim::{
    ImageKind, RamStore, Sim, Source, WireFrame, assert_valid, instruction, status,
};
use osc_protocol::wire::{MgmtOp, Opcode, ResultCode};
use osc_servo_core::data_state::{
    CALIB_CORRUPT, CALIB_STALE, CALIB_VIRGIN, CONFIG_CORRUPT, CONFIG_STALE, CONFIG_VIRGIN,
    PLANT_UNSET, STAMP_MISMATCH, allows,
};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::kernel::faults::{BIT_DATA, CODE_DATA, CODE_NONE};
use osc_servo_core::persist::HEADER_LEN;
use osc_servo_core::persist::Slot;
use osc_servo_core::regions::CALIB_BASE_ADDR;
use osc_servo_core::regions::calib::addr::motor::{KE_VPC_Q, RECIP_KE_Q};
use osc_servo_core::regions::calib::addr::stamp::PLANT_STAMP;
use osc_servo_core::regions::config::addr::limits::STALL_TIME_MS;
use osc_servo_core::regions::config::addr::loop_velocity::V_KP_Q88;
use osc_servo_core::regions::control::addr::lifecycle::TORQUE_ENABLE;
use osc_servo_core::regions::telemetry::addr::mode::DATA_FLAGS;
use osc_servo_core::stamp::compute;
use osc_servo_core::tel::{TelSample, TelStream};
use osc_servo_core::{ControlTable, Kernel, Mode, MotorCmd, RegionStorage, Shared};
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{matrix, sim};

const ID5: u8 = 5;
const START_POS: u16 = 1500;
const FRESH: u8 = CONFIG_VIRGIN | CALIB_VIRGIN | STAMP_MISMATCH | PLANT_UNSET;

/// A store holding one full save of the identified rig.
fn identified_store() -> &'static RamStore {
    let store = RamStore::leak();
    let sh = Shared::new();
    seed(&sh);
    store.save_table(&sh);
    store
}

/// Boot the rig off `store`: board defaults first (the rig seed with Ke
/// unset and never stamped, as a board that was never identified), then
/// the store's overlay and the data state it makes.
fn boot(store: &RamStore) -> Shared {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.calib.motor.recip_ke_q = 0;
        t.calib.motor.ke_vpc_q = 0;
        t.calib.stamp.plant_stamp = 0;
    });
    store.boot_load(&sh);
    sh
}

fn data_flags(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.mode.data_flags)
}

fn fault(sh: &Shared) -> (u8, u8) {
    sh.table
        .with(|t| (t.telemetry.common.fault_flags, t.telemetry.mode.fault_code))
}

fn set(sh: &Shared, f: impl FnOnce(&mut ControlTable)) {
    sh.table.with_mut(f);
}

/// Torque on in `mode` with a goal that asks for motion.
fn enable(sh: &Shared, mode: Mode) {
    set(sh, |t| {
        let l = &mut t.control.lifecycle;
        l.mode = mode;
        l.goal_position = START_POS as i32 + 400;
        l.goal_velocity = 800;
        l.goal_current = 150;
        l.goal_duty = 5000;
        l.torque_enable = true;
    });
}

struct Rig {
    k: Kernel<FakeIo>,
    plant: Plant,
    duty: i16,
}

impl Rig {
    fn new() -> Self {
        Self {
            k: kernel(),
            plant: Plant::new(START_POS),
            duty: 0,
        }
    }

    /// `ticks` fast ticks of the plant under the kernel; every command.
    fn run(&mut self, sh: &Shared, ticks: u32) -> Vec<MotorCmd> {
        (0..ticks)
            .map(|_| {
                let f = self.plant.step(self.duty);
                self.k.on_tick(f, sh);
                let cmd = last_cmd(&self.k);
                self.duty = duty_of(cmd);
                cmd
            })
            .collect()
    }
}

fn drives(cmds: &[MotorCmd]) -> bool {
    cmds.iter().any(|c| matches!(c, MotorCmd::Drive { .. }))
}

fn all_disabled(cmds: &[MotorCmd]) -> bool {
    cmds.iter().all(|c| matches!(c, MotorCmd::Disabled))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Fixture {
    Loaded,
    Virgin,
    Corrupt,
    Stale,
    /// A loaded image whose identified set has Ke = 0.
    ZeroKe,
}

impl Fixture {
    fn apply(self, store: &RamStore, kind: ImageKind) {
        match self {
            Fixture::Loaded | Fixture::ZeroKe => {}
            Fixture::Virgin => store.erase(kind),
            Fixture::Corrupt => store.corrupt_slot(kind, Slot::A),
            Fixture::Stale => store.stale_slot(kind, Slot::A),
        }
    }

    fn flags(self, virgin: u8, corrupt: u8, stale: u8) -> u8 {
        match self {
            Fixture::Loaded | Fixture::ZeroKe => 0,
            Fixture::Virgin => virgin,
            Fixture::Corrupt => corrupt,
            Fixture::Stale => stale,
        }
    }
}

/// Every boot state the store can hand the kernel, times both closed-loop
/// modes: Velocity and Position emit `Drive` only when no reason is set and
/// both Ke are nonzero. The clean rig is the positive control. A CALIB that
/// did not load leaves the board's never-stamped defaults under the saved
/// gains, so the stamp mismatches too; a stamped zero-Ke set is consistent.
#[test_log::test]
fn no_boot_state_drives_closed_loop_with_zero_recip_ke() {
    const CONFIGS: [Fixture; 4] = [
        Fixture::Loaded,
        Fixture::Virgin,
        Fixture::Corrupt,
        Fixture::Stale,
    ];
    const CALIBS: [Fixture; 5] = [
        Fixture::Loaded,
        Fixture::Virgin,
        Fixture::Corrupt,
        Fixture::Stale,
        Fixture::ZeroKe,
    ];
    for config in CONFIGS {
        for calib in CALIBS {
            for mode in [Mode::Velocity, Mode::Position] {
                let store = if calib == Fixture::ZeroKe {
                    let store = RamStore::leak();
                    let sh = Shared::new();
                    seed(&sh);
                    set(&sh, |t| t.calib.motor.recip_ke_q = 0);
                    stamp(&sh);
                    store.save_table(&sh);
                    store
                } else {
                    identified_store()
                };
                config.apply(store, ImageKind::Config);
                calib.apply(store, ImageKind::Calib);
                let sh = boot(store);
                let ke_set = calib == Fixture::Loaded;
                let stamped = matches!(calib, Fixture::Loaded | Fixture::ZeroKe);
                let want = config.flags(CONFIG_VIRGIN, CONFIG_CORRUPT, CONFIG_STALE)
                    | calib.flags(CALIB_VIRGIN, CALIB_CORRUPT, CALIB_STALE)
                    | if ke_set { 0 } else { PLANT_UNSET }
                    | if stamped { 0 } else { STAMP_MISMATCH };
                let case = format!("config {config:?} calib {calib:?} {mode:?}");
                assert_eq!(data_flags(&sh), want, "{case}");

                let mut rig = Rig::new();
                rig.run(&sh, 200);
                enable(&sh, mode);
                let cmds = rig.run(&sh, 3000);
                if want == 0 {
                    assert!(drives(&cmds), "{case}: the clean rig drives");
                    assert_eq!(fault(&sh), (0, CODE_NONE), "{case}");
                } else {
                    assert!(all_disabled(&cmds), "{case}: {cmds:?}");
                    assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA), "{case}");
                }
            }
        }
    }
}

#[test_log::test]
fn calib_reset_under_saved_gains_refuses_velocity() {
    let store = identified_store();
    store.erase(ImageKind::Calib);
    let sh = boot(store);
    assert_eq!(data_flags(&sh), CALIB_VIRGIN | STAMP_MISMATCH | PLANT_UNSET);
    // the saved gains did load: this is the live-gains + no-Ke runaway
    assert_eq!(sh.table.with(|t| t.config.loop_velocity.v_kp_q88), 64);

    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, Mode::Velocity);
    let cmds = rig.run(&sh, 2000);
    assert!(all_disabled(&cmds), "{cmds:?}");
    assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA));
}

#[test_log::test]
fn calib_version_bump_boots_stale_and_gates_closed_loop() {
    let store = identified_store();
    store.stale_slot(ImageKind::Calib, Slot::A);
    let sh = boot(store);
    assert_eq!(data_flags(&sh), CALIB_STALE | STAMP_MISMATCH | PLANT_UNSET);

    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, Mode::Position);
    assert!(all_disabled(&rig.run(&sh, 500)));
    assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA));
    // stale is a fresh servo, not a corrupt one: open loop still runs
    set(&sh, |t| t.control.lifecycle.torque_enable = false);
    rig.run(&sh, 20);
    enable(&sh, Mode::OpenLoop);
    assert!(drives(&rig.run(&sh, 500)));
    assert_eq!(fault(&sh), (0, CODE_NONE));

    // a stale CONFIG says so by name and gates the same way
    let store = identified_store();
    store.stale_slot(ImageKind::Config, Slot::A);
    let sh = boot(store);
    assert_eq!(data_flags(&sh), CONFIG_STALE);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, Mode::Velocity);
    assert!(all_disabled(&rig.run(&sh, 500)));
    assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA));
}

#[test_log::test]
fn corrupt_config_refuses_every_mode() {
    let store = identified_store();
    store.corrupt_slot(ImageKind::Config, Slot::A);
    let sh = boot(store);
    assert_eq!(data_flags(&sh), CONFIG_CORRUPT);
    for mode in [
        Mode::OpenLoop,
        Mode::Current,
        Mode::Velocity,
        Mode::Position,
    ] {
        let mut rig = Rig::new();
        rig.run(&sh, 200);
        enable(&sh, mode);
        let cmds = rig.run(&sh, 500);
        assert!(all_disabled(&cmds), "{mode:?}: {cmds:?}");
        assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA), "{mode:?}");
        set(&sh, |t| t.control.lifecycle.torque_enable = false);
    }
}

/// The physics belt: a zero Ke written into a running closed loop stops it
/// on the next fast tick, and the publish lands within the medium tick.
#[rstest]
#[case(Mode::Velocity, RECIP_KE_Q)]
#[case(Mode::Velocity, KE_VPC_Q)]
#[case(Mode::Position, RECIP_KE_Q)]
#[case(Mode::Position, KE_VPC_Q)]
#[test_log::test]
fn live_zero_ke_write_stops_a_running_closed_loop(#[case] mode: Mode, #[case] reg: u16) {
    let sh = Shared::new();
    seed(&sh);
    assert_eq!(data_flags(&sh), 0);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, mode);
    let cmds = rig.run(&sh, 3000);
    assert!(drives(&cmds));
    assert!(
        matches!(cmds[cmds.len() - 1], MotorCmd::Drive { .. }),
        "still driving: {:?}",
        cmds[cmds.len() - 1]
    );
    set(&sh, |t| {
        if reg == RECIP_KE_Q {
            t.calib.motor.recip_ke_q = 0;
        } else {
            t.calib.motor.ke_vpc_q = 0;
        }
    });
    let cmds = rig.run(&sh, DECIM_MED as u32);
    assert!(all_disabled(&cmds), "{cmds:?}");
    assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA));
    // the ack re-latches while the reason holds
    set(&sh, |t| t.control.lifecycle.torque_enable = false);
    rig.run(&sh, 20);
    set(&sh, |t| t.control.lifecycle.torque_enable = true);
    assert!(all_disabled(&rig.run(&sh, 3 * DECIM_MED as u32)));
    assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA));
}

/// Records every TEL sample the kernel emits; the handle outlives the kernel.
struct RecTel(Rc<RefCell<Vec<TelSample>>>);

impl TelStream for RecTel {
    fn active(&self) -> bool {
        true
    }
    fn on_tick(&mut self, sample: &TelSample) {
        self.0.borrow_mut().push(*sample);
    }
}

/// The cal/ident path on a factory-fresh servo: OpenLoop and Current drive
/// with no fault latched, so no ALERT and no TEL sample carries one; a
/// mode change into closed loop under torque is what latches.
#[test_log::test]
fn virgin_servo_drives_openloop_and_current_without_alert() {
    let store = RamStore::leak();
    let sh = boot(store);
    assert_eq!(data_flags(&sh), FRESH);
    let tel = Rc::new(RefCell::new(Vec::new()));
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        RecTel(tel.clone()),
        TIMING,
    );
    let mut plant = Plant::new(START_POS);
    let mut duty = 0i16;
    let mut run = |sh: &Shared, ticks: u32| -> Vec<MotorCmd> {
        (0..ticks)
            .map(|_| {
                let f = plant.step(duty);
                k.on_tick(f, sh);
                let cmd = k.io.motor.last.expect("a motor write happened");
                duty = duty_of(cmd);
                cmd
            })
            .collect()
    };
    run(&sh, 200);
    enable(&sh, Mode::OpenLoop);
    assert!(drives(&run(&sh, 1000)));
    assert_eq!(fault(&sh), (0, CODE_NONE));
    set(&sh, |t| t.control.lifecycle.mode = Mode::Current);
    assert!(drives(&run(&sh, 1000)));
    assert_eq!(fault(&sh), (0, CODE_NONE));

    set(&sh, |t| t.control.lifecycle.mode = Mode::Velocity);
    let cmds = run(&sh, 500);
    assert!(all_disabled(&cmds), "{cmds:?}");
    assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA));
    // TEL carried no fault until the closed-loop attempt
    let samples = tel.borrow();
    let first_fault = samples
        .iter()
        .position(|s| s.fault)
        .expect("the latch shows");
    assert!(first_fault >= 200 + 2000, "clean through the open modes");
    assert!(samples[..first_fault].iter().all(|s| !s.fault));
}

// SAVE and FACTORY over the wire

fn sole_reply(frames: &[WireFrame]) -> &WireFrame {
    let replies: Vec<&WireFrame> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "expected one servo reply: {frames:#?}");
    assert_valid(replies[0]);
    replies[0]
}

fn mgmt(sim: &mut Sim, op: MgmtOp) -> ResultCode {
    sim.host_send(&instruction(ID5, Opcode::Mgmt, 0, &[op as u8]));
    let frames = sim.run();
    status(sole_reply(&frames)).0.result().expect("result code")
}

fn write_ok(sim: &mut Sim, addr: u16, data: &[u8]) {
    let a = addr.to_le_bytes();
    let mut payload = vec![a[0], a[1]];
    payload.extend_from_slice(data);
    sim.host_send(&instruction(ID5, Opcode::Write, 0, &payload));
    let frames = sim.run();
    assert_eq!(status(sole_reply(&frames)).0.result(), Some(ResultCode::Ok));
}

fn read_byte(sim: &mut Sim, addr: u16) -> u8 {
    let a = addr.to_le_bytes();
    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[a[0], a[1], 1, 0]));
    let frames = sim.run();
    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    payload[0]
}

fn write_ke(sim: &mut Sim) {
    write_ok(sim, RECIP_KE_Q, &3700u16.to_le_bytes());
    write_ok(sim, KE_VPC_Q, &1150u16.to_le_bytes());
}

/// The host's stamp over the set it intends: what the servo holds now.
fn write_stamp(sim: &mut Sim, s: usize) {
    let stamp = sim.servo_table(s, |t| compute(t, None));
    write_ok(sim, PLANT_STAMP, &stamp.to_le_bytes());
}

fn set_torque(sim: &mut Sim, on: bool) {
    write_ok(sim, TORQUE_ENABLE, &[on as u8]);
}

/// A servo whose store holds one identified, stamped, SAVEd set.
fn stamped_servo(sim: &mut Sim, store: &'static RamStore) -> usize {
    let s = sim.add_servo_with_store(ID5, store);
    write_ke(sim);
    write_stamp(sim, s);
    assert_eq!(mgmt(sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(read_byte(sim, DATA_FLAGS), 0);
    s
}

#[apply(matrix)]
fn save_retires_the_fresh_reasons_and_recomputes_the_checkpoint(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with_store(ID5, store);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), FRESH);
    // a failed program changes nothing but the checkpoint
    store.set_fail(true);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), FRESH);
    store.set_fail(false);
    // the images are this servo's own now; the plant is still unidentified
    // and the set never stamped
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        STAMP_MISMATCH | PLANT_UNSET
    );
    write_ke(&mut sim);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        STAMP_MISMATCH | PLANT_UNSET,
        "a covered write is not a checkpoint"
    );
    // the stamp write is: both verdicts follow the live set
    write_stamp(&mut sim, s);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), 0);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), 0);
    // reboot with flash intact: loaded, identified, stamped
    let mut rebooted = support::sim(baud_idx);
    rebooted.add_servo_with_store(ID5, store);
    assert_eq!(read_byte(&mut rebooted, DATA_FLAGS), 0);
}

#[apply(matrix)]
fn save_retires_stale_images_like_virgin_ones(baud_idx: u8) {
    let store = RamStore::leak();
    store.stale_slot(ImageKind::Config, Slot::A);
    store.stale_slot(ImageKind::Calib, Slot::B);
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with_store(ID5, store);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        CONFIG_STALE | CALIB_STALE | STAMP_MISMATCH | PLANT_UNSET
    );
    write_ke(&mut sim);
    write_stamp(&mut sim, s);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), 0);
}

#[apply(matrix)]
fn save_does_not_clear_corrupt_config(baud_idx: u8) {
    let store = RamStore::leak();
    store.corrupt_slot(ImageKind::Config, Slot::A);
    store.corrupt_slot(ImageKind::Calib, Slot::B);
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with_store(ID5, store);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        CONFIG_CORRUPT | CALIB_CORRUPT | STAMP_MISMATCH | PLANT_UNSET
    );
    write_ke(&mut sim);
    write_stamp(&mut sim, s);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        CONFIG_CORRUPT,
        "calib recovers by SAVE, config only by FACTORY"
    );
    // the verdict is about what boot found: the SAVE that just persisted
    // board defaults over the rotten slot is what the next boot loads
    let mut rebooted = support::sim(baud_idx);
    rebooted.add_servo_with_store(ID5, store);
    assert_eq!(read_byte(&mut rebooted, DATA_FLAGS), 0);
}

#[apply(matrix)]
fn factory_after_corrupt_config_boots_virgin(baud_idx: u8) {
    let store = RamStore::leak();
    store.corrupt_slot(ImageKind::Config, Slot::A);
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with_store(ID5, store);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        CONFIG_CORRUPT | CALIB_VIRGIN | STAMP_MISMATCH | PLANT_UNSET
    );
    assert_eq!(mgmt(&mut sim, MgmtOp::Factory), ResultCode::Ok);
    assert!(sim.take_reboot(s).is_some());
    let mut rebooted = support::sim(baud_idx);
    rebooted.add_servo_with_store(ID5, store);
    assert_eq!(read_byte(&mut rebooted, DATA_FLAGS), FRESH);
}

// The plant stamp over the wire

/// The `osc ident` commit sequence: write the set, stamp it with torque
/// off, and the stamp write is the checkpoint that opens closed loop.
#[apply(matrix)]
fn stamp_write_verifies_and_opens_closed_loop(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with_store(ID5, store);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), FRESH);
    write_ke(&mut sim);
    // a wrong stamp is a miss: some intended write did not land
    let stamp = sim.servo_table(s, |t| compute(t, None));
    write_ok(&mut sim, PLANT_STAMP, &(stamp ^ 1).to_le_bytes());
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        CONFIG_VIRGIN | CALIB_VIRGIN | STAMP_MISMATCH
    );
    write_stamp(&mut sim, s);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        CONFIG_VIRGIN | CALIB_VIRGIN
    );
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), 0);
    assert!(allows(Mode::Position, 0));
}

#[apply(matrix)]
fn stamp_write_with_torque_on_stays_unverified(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = stamped_servo(&mut sim, store);
    set_torque(&mut sim, true);
    write_ok(&mut sim, V_KP_Q88, &70u16.to_le_bytes());
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), STAMP_MISMATCH);
    write_stamp(&mut sim, s);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        STAMP_MISMATCH,
        "landed, not verified"
    );
    set_torque(&mut sim, false);
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        STAMP_MISMATCH,
        "torque off alone is no checkpoint"
    );
    write_stamp(&mut sim, s);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), 0);
}

/// A host that wrote some of the set and died: the servo refuses closed
/// loop until a stamp over the whole set lands.
#[apply(matrix)]
fn partial_covered_write_blocks_next_enable(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = stamped_servo(&mut sim, store);
    write_ok(&mut sim, STALL_TIME_MS, &200u16.to_le_bytes());
    assert_eq!(
        read_byte(&mut sim, DATA_FLAGS),
        0,
        "a user limit is not covered"
    );
    write_ok(&mut sim, V_KP_Q88, &70u16.to_le_bytes());
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), STAMP_MISMATCH);
    assert!(!allows(Mode::Velocity, STAMP_MISMATCH));
    // the old stamp is not the answer
    let old = store.calib_slot(Slot::A).expect("saved");
    let at = HEADER_LEN + (PLANT_STAMP - CALIB_BASE_ADDR) as usize;
    write_ok(&mut sim, PLANT_STAMP, &old[at..at + 2]);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), STAMP_MISMATCH);
    write_stamp(&mut sim, s);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), 0);
}

/// A gain edit under torque marks the mismatch but never yanks the loop;
/// the next enable is what it refuses, until a torque-off restamp.
#[test_log::test]
fn live_gain_edit_keeps_the_running_loop_until_reenable() {
    let sh = Shared::new();
    seed(&sh);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, Mode::Velocity);
    assert!(drives(&rig.run(&sh, 2000)));
    // the dispatcher's post-commit hook on a covered write
    set(&sh, |t| t.config.loop_velocity.v_kp_q88 = 70);
    sh.data_state_after_commit(V_KP_Q88, 2);
    assert_eq!(data_flags(&sh), STAMP_MISMATCH);
    let cmds = rig.run(&sh, 3 * DECIM_MED as u32);
    assert!(cmds.iter().all(|c| matches!(c, MotorCmd::Drive { .. })));
    assert_eq!(fault(&sh), (0, CODE_NONE));

    set(&sh, |t| t.control.lifecycle.torque_enable = false);
    rig.run(&sh, 20);
    set(&sh, |t| t.control.lifecycle.torque_enable = true);
    assert!(all_disabled(&rig.run(&sh, 500)));
    assert_eq!(fault(&sh), (BIT_DATA, CODE_DATA));

    set(&sh, |t| t.control.lifecycle.torque_enable = false);
    rig.run(&sh, 20);
    stamp(&sh);
    sh.data_state_after_commit(PLANT_STAMP, 2);
    assert_eq!(data_flags(&sh), 0);
    set(&sh, |t| t.control.lifecycle.torque_enable = true);
    assert!(drives(&rig.run(&sh, 2000)));
    assert_eq!(fault(&sh), (0, CODE_NONE));
}

/// SAVE persists a stale stamp on purpose (ack == durable); the reboot's
/// recompute is what keeps closed loop shut.
#[apply(matrix)]
fn save_with_stale_stamp_gates_after_reboot(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    stamped_servo(&mut sim, store);
    write_ok(&mut sim, V_KP_Q88, &70u16.to_le_bytes());
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), STAMP_MISMATCH);
    let mut rebooted = support::sim(baud_idx);
    rebooted.add_servo_with_store(ID5, store);
    assert_eq!(read_byte(&mut rebooted, DATA_FLAGS), STAMP_MISMATCH);
}

/// A power cut between the CONFIG and CALIB images boots a new/old mix:
/// the recompute over the mix differs from the stamp that loaded with the
/// old CALIB. When nothing covered moved, the mix is harmless and matches.
#[apply(matrix)]
fn torn_save_boots_stamp_mismatch(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = stamped_servo(&mut sim, store);
    write_ok(&mut sim, V_KP_Q88, &70u16.to_le_bytes());
    write_stamp(&mut sim, s);
    assert_eq!(read_byte(&mut sim, DATA_FLAGS), 0);
    store.fail_after(ImageKind::Config);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, store);
    assert_eq!(
        rebooted.servo_table(s, |t| t.config.loop_velocity.v_kp_q88),
        70
    );
    assert_eq!(read_byte(&mut rebooted, DATA_FLAGS), STAMP_MISMATCH);

    // the same tear under an uncovered edit
    let store = RamStore::leak();
    let mut sim = support::sim(baud_idx);
    stamped_servo(&mut sim, store);
    write_ok(&mut sim, STALL_TIME_MS, &200u16.to_le_bytes());
    store.fail_after(ImageKind::Config);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    let mut rebooted = support::sim(baud_idx);
    rebooted.add_servo_with_store(ID5, store);
    assert_eq!(read_byte(&mut rebooted, DATA_FLAGS), 0);
}
