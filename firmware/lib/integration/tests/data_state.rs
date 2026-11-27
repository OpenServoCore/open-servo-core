//! The stored state (`data_state` module), the unsaved bit and the belt:
//! `status_flags` names the images boot did not load and whether the
//! commits changed anything since SAVE, and only the belt refuses a mode,
//! on the constants a loop would run on, publishing what it refuses in
//! `refused_modes`. Boot verdicts come from the RAM store's slots through
//! the real overlay path, the kernel runs the plant rig on the table that
//! left, and SAVE/FACTORY go over the wire.

use std::cell::RefCell;
use std::rc::Rc;

use control_table::RegisterFile;
use osc_integration::plant::{
    FakeIo, FakeMotor, FakeSensors, Plant, TIMING, duty_of, kernel, last_cmd, seed,
};
use osc_integration::sim::{
    ImageKind, RamStore, Sim, Source, Tear, WireFrame, assert_valid, instruction, status,
};
use osc_protocol::table::{
    STATUS_FLAG_CALIB_DEFAULTS, STATUS_FLAG_CONFIG_DEFAULTS, STATUS_FLAG_UNSAVED, STATUS_FLAGS,
};
use osc_protocol::wire::{MgmtOp, Opcode, ResultCode};
use osc_servo_core::kernel::belt::{KE_UNSET, LIMITS_UNSET};
use osc_servo_core::kernel::phases::loops::faults::{BIT_CONFIG, CODE_CONFIG, CODE_NONE};
use osc_servo_core::kernel::{DECIM_MED, phase as kernel_phase};
use osc_servo_core::persist::Slot;
use osc_servo_core::pos_lut::cmd;
use osc_servo_core::regions::calib::addr::motor::{KE_VPC_Q, R_Q12, RECIP_KE_Q};
use osc_servo_core::regions::calib::addr::pot::{RAW_MAX, RAW_MIN};
use osc_servo_core::regions::config::addr::limits::CURRENT_LIMIT_COUNTS;
use osc_servo_core::regions::config::addr::loop_position::SPEED_CEILING_CPS;
use osc_servo_core::regions::config::addr::loop_velocity::V_KP_Q88;
use osc_servo_core::regions::control::addr::pos_lut::POS_LUT_CMD;
use osc_servo_core::regions::telemetry::tick_flag;
use osc_servo_core::regions::{CALIB_REGION_SIZE, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE};
use osc_servo_core::{ControlTable, Kernel, Mode, MotorCmd, RegionStorage, Shared, TelStream};
use osc_servo_core::{ControlTableCell, TelemetryTick};
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{matrix, sim};

const ID5: u8 = 5;
const START_POS: u16 = 1500;
const CONFIG_DEFAULTS: u8 = STATUS_FLAG_CONFIG_DEFAULTS;
const CALIB_DEFAULTS: u8 = STATUS_FLAG_CALIB_DEFAULTS;
const FRESH: u8 = CONFIG_DEFAULTS | CALIB_DEFAULTS;

/// A store holding one full save of the identified rig.
fn identified_store() -> &'static RamStore {
    let store = RamStore::leak();
    let sh = Shared::new();
    seed(&sh);
    store.save_table(&sh);
    store
}

/// Boot the rig off `store`: board defaults first (the rig seed with Ke
/// unset, as a board that was never identified), then
/// the store's overlay and the store bits it makes.
fn boot(store: &RamStore) -> Shared {
    boot_on(store, |_| {})
}

/// [`boot`] with `defaults` applied to the board defaults ahead of the
/// overlay.
fn boot_on(store: &RamStore, defaults: impl FnOnce(&mut ControlTable)) -> Shared {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.calib.motor.recip_ke_q = 0;
        t.calib.motor.ke_vpc_q = 0;
        defaults(t);
    });
    store.boot_load(&sh);
    sh
}

/// The store bits of `status_flags`.
fn store_bits(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.common.status_flags) & FRESH
}

fn refused(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.mode.refused_modes)
}

fn fault(sh: &Shared) -> (u8, u8) {
    sh.table
        .with(|t| (t.telemetry.common.fault_flags, t.telemetry.mode.fault_code))
}

fn set(sh: &Shared, f: impl FnOnce(&mut ControlTable)) {
    sh.table.with_mut(f);
}

/// A committed CONFIG/CALIB write, as the dispatcher lands one: the kernel
/// takes it at its next medium period.
fn commit(sh: &Shared, f: impl FnOnce(&mut ControlTable)) {
    sh.config_commit_begin();
    sh.table.with_mut(f);
    sh.config_commit_end();
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
                self.k.on_scan(f, sh);
                self.duty = duty_of(&self.k);
                last_cmd(&self.k)
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
    OtherLayout,
    /// A loaded image whose identified set has Ke = 0.
    ZeroKe,
}

impl Fixture {
    fn apply(self, store: &RamStore, kind: ImageKind) {
        match self {
            Fixture::Loaded | Fixture::ZeroKe => {}
            Fixture::Virgin => store.erase(kind),
            Fixture::Corrupt => store.corrupt_slot(kind, Slot::A),
            Fixture::OtherLayout => store.other_layout_slot(kind, Slot::A),
        }
    }

    /// `bit` when the image did not load: blank, unreadable and another
    /// layout's are one case.
    fn flag(self, bit: u8) -> u8 {
        match self {
            Fixture::Loaded | Fixture::ZeroKe => 0,
            Fixture::Virgin | Fixture::Corrupt | Fixture::OtherLayout => bit,
        }
    }
}

/// Every boot state the store can hand the kernel, times both closed-loop
/// modes: Velocity and Position emit `Drive` exactly when both Ke are
/// nonzero, whatever the store bits name. A CALIB that did not load
/// leaves the board's defaults under the saved gains; a saved zero-Ke set
/// loads with no store bit set.
#[test_log::test]
fn no_boot_state_drives_closed_loop_with_zero_recip_ke() {
    const CONFIGS: [Fixture; 4] = [
        Fixture::Loaded,
        Fixture::Virgin,
        Fixture::Corrupt,
        Fixture::OtherLayout,
    ];
    const CALIBS: [Fixture; 5] = [
        Fixture::Loaded,
        Fixture::Virgin,
        Fixture::Corrupt,
        Fixture::OtherLayout,
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
                    store.save_table(&sh);
                    store
                } else {
                    identified_store()
                };
                config.apply(store, ImageKind::Config);
                calib.apply(store, ImageKind::Calib);
                let sh = boot(store);
                let ke_set = calib == Fixture::Loaded;
                let want = config.flag(CONFIG_DEFAULTS) | calib.flag(CALIB_DEFAULTS);
                let case = format!("config {config:?} calib {calib:?} {mode:?}");
                assert_eq!(store_bits(&sh), want, "{case}");

                let mut rig = Rig::new();
                rig.run(&sh, 200);
                enable(&sh, mode);
                let cmds = rig.run(&sh, 3000);
                if ke_set {
                    assert!(drives(&cmds), "{case}: Ke set drives");
                    assert_eq!(fault(&sh), (0, CODE_NONE), "{case}");
                } else {
                    assert!(all_disabled(&cmds), "{case}: {cmds:?}");
                    assert_eq!(fault(&sh), (BIT_CONFIG, CODE_CONFIG), "{case}");
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
    assert_eq!(store_bits(&sh), CALIB_DEFAULTS);
    // the saved gains did load: this is the live-gains + no-Ke runaway
    assert_eq!(sh.table.with(|t| t.config.loop_velocity.v_kp_q88), 64);

    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, Mode::Velocity);
    let cmds = rig.run(&sh, 2000);
    assert!(all_disabled(&cmds), "{cmds:?}");
    assert_eq!(fault(&sh), (BIT_CONFIG, CODE_CONFIG));
}

/// The CONFIG and CALIB bytes a boot left.
fn stored_regions(sh: &Shared) -> Vec<u8> {
    RegisterFile::read(
        &sh.table,
        CONFIG_BASE_ADDR,
        CONFIG_REGION_SIZE + CALIB_REGION_SIZE,
    )
    .expect("CONFIG and CALIB are contiguous")
    .to_vec()
}

/// An image of another layout boots exactly as a blank one: the same
/// defaults byte for byte, the same store bit, the same refused modes. It
/// gates nothing itself: such a CALIB leaves Ke unset, which the belt
/// refuses until a Ke is written, and such a CONFIG under a loaded CALIB
/// drives.
#[test_log::test]
fn an_older_layout_image_boots_on_defaults_like_a_blank_one() {
    for (kind, bit) in [
        (ImageKind::Config, CONFIG_DEFAULTS),
        (ImageKind::Calib, CALIB_DEFAULTS),
    ] {
        let older = identified_store();
        older.other_layout_slot(kind, Slot::A);
        let blank = identified_store();
        blank.erase(kind);
        let (older, blank) = (boot(older), boot(blank));
        assert_eq!(stored_regions(&older), stored_regions(&blank), "{kind:?}");
        assert_eq!(store_bits(&older), bit, "{kind:?}");
        assert_eq!(store_bits(&blank), bit, "{kind:?}");
        let (mut a, mut b) = (Rig::new(), Rig::new());
        a.run(&older, 200);
        b.run(&blank, 200);
        assert_eq!(refused(&older), refused(&blank), "{kind:?}");
    }

    let store = identified_store();
    store.other_layout_slot(ImageKind::Calib, Slot::A);
    let sh = boot(store);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    assert_eq!(refused(&sh), KE_UNSET);
    enable(&sh, Mode::Position);
    assert!(all_disabled(&rig.run(&sh, 500)));
    assert_eq!(fault(&sh), (BIT_CONFIG, CODE_CONFIG), "Ke unset");
    set(&sh, |t| t.control.lifecycle.torque_enable = false);
    rig.run(&sh, 20);
    enable(&sh, Mode::OpenLoop);
    assert!(drives(&rig.run(&sh, 500)));
    assert_eq!(fault(&sh), (0, CODE_NONE));
    set(&sh, |t| t.control.lifecycle.torque_enable = false);
    commit(&sh, |t| {
        t.calib.motor.recip_ke_q = 3700;
        t.calib.motor.ke_vpc_q = 1150;
    });
    rig.run(&sh, 20);
    enable(&sh, Mode::Position);
    assert!(drives(&rig.run(&sh, 3000)), "a written Ke drives, unsaved");
    assert_eq!(fault(&sh), (0, CODE_NONE));
    assert_eq!(store_bits(&sh), CALIB_DEFAULTS, "the bit stands until SAVE");

    let store = identified_store();
    store.other_layout_slot(ImageKind::Config, Slot::A);
    let sh = boot(store);
    assert_eq!(store_bits(&sh), CONFIG_DEFAULTS);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, Mode::Velocity);
    assert!(drives(&rig.run(&sh, 3000)));
    assert_eq!(fault(&sh), (0, CODE_NONE));
}

/// The modes the belt refuses read before any enable, torque off, and move
/// within a medium period of the write that changes them. A factory-fresh
/// servo (Ke, cap and ceiling 0) refuses every closed mode; a loaded set
/// with a zero Ke refuses Velocity and Position with no store bit set; a
/// usable set refuses nothing. OpenLoop is never refused.
#[test_log::test]
fn the_refused_modes_read_before_any_enable() {
    let closed = KE_UNSET | LIMITS_UNSET;
    let store = RamStore::leak();
    let sh = boot_on(store, |t| {
        t.config.limits.current_limit_counts = 0;
        t.config.loop_position.speed_ceiling_cps = 0;
    });
    assert_eq!(store_bits(&sh), FRESH);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    assert_eq!(refused(&sh), closed);
    assert_eq!(closed & (1 << Mode::OpenLoop as u8), 0);
    assert_eq!(
        closed,
        (1 << Mode::Current as u8) | (1 << Mode::Velocity as u8) | (1 << Mode::Position as u8)
    );

    let (cap, ceiling) = rig_limits();
    commit(&sh, |t| {
        t.config.limits.current_limit_counts = cap;
        t.config.loop_position.speed_ceiling_cps = ceiling;
    });
    rig.run(&sh, DECIM_MED as u32);
    assert_eq!(refused(&sh), KE_UNSET, "limits written, Ke still 0");
    commit(&sh, |t| {
        t.calib.motor.recip_ke_q = 3700;
        t.calib.motor.ke_vpc_q = 1150;
    });
    rig.run(&sh, DECIM_MED as u32);
    assert_eq!(refused(&sh), 0, "a usable set");
    commit(&sh, |t| t.calib.motor.ke_vpc_q = 0);
    rig.run(&sh, DECIM_MED as u32);
    assert_eq!(refused(&sh), KE_UNSET, "a zero Ke written back");
    assert_eq!(
        fault(&sh),
        (0, CODE_NONE),
        "nothing enabled, nothing latched"
    );

    // images that loaded, a saved set with a zero Ke: no store bit, and
    // the refusal shows all the same
    let store = RamStore::leak();
    let saved = Shared::new();
    seed(&saved);
    set(&saved, |t| t.calib.motor.recip_ke_q = 0);
    store.save_table(&saved);
    let sh = boot(store);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    assert_eq!((store_bits(&sh), refused(&sh)), (0, KE_UNSET));
}

/// An unreadable CONFIG is a blank one: board defaults run, and their
/// unset current cap and speed ceiling are what the belt refuses, so
/// OpenLoop drives and every closed mode latches the config fault. Once the
/// limits are written (volatile, unsaved) Position drives under the
/// standing `CONFIG_DEFAULTS`.
#[test_log::test]
fn corrupt_config_boots_on_defaults_and_runs_openloop() {
    let store = identified_store();
    store.corrupt_slot(ImageKind::Config, Slot::A);
    let (cap, ceiling) = rig_limits();
    let sh = boot_on(store, |t| {
        t.config.limits.current_limit_counts = 0;
        t.config.loop_position.speed_ceiling_cps = 0;
    });
    let want = CONFIG_DEFAULTS;
    assert_eq!(store_bits(&sh), want);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, Mode::OpenLoop);
    assert!(drives(&rig.run(&sh, 500)), "OpenLoop");
    assert_eq!(fault(&sh), (0, CODE_NONE), "OpenLoop");
    for mode in [Mode::Current, Mode::Velocity, Mode::Position] {
        set(&sh, |t| t.control.lifecycle.torque_enable = false);
        rig.run(&sh, 20);
        enable(&sh, mode);
        assert!(all_disabled(&rig.run(&sh, 500)), "{mode:?}");
        assert_eq!(
            fault(&sh),
            (BIT_CONFIG, CODE_CONFIG),
            "{mode:?}: limits unset"
        );
    }
    set(&sh, |t| t.control.lifecycle.torque_enable = false);
    commit(&sh, |t| {
        t.config.limits.current_limit_counts = cap;
        t.config.loop_position.speed_ceiling_cps = ceiling;
    });
    rig.run(&sh, 20);
    enable(&sh, Mode::Position);
    assert!(drives(&rig.run(&sh, 3000)), "limits written");
    assert_eq!(fault(&sh), (0, CODE_NONE));
    assert_eq!(store_bits(&sh), want, "the boot verdict stands until SAVE");
}

/// The rig's current cap and speed ceiling.
fn rig_limits() -> (u16, u16) {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with(|t| {
        (
            t.config.limits.current_limit_counts,
            t.config.loop_position.speed_ceiling_cps,
        )
    })
}

/// The physics belt: a zero Ke written into a running closed loop reaches
/// the kernel at the first CONTROL phase after the commit, where the fault
/// latches and the drive stops; the PUBLISH phase of that period shows it.
/// The loops read the same snapshot, so every tick before that boundary
/// runs on the old Ke, command for command what a servo without the write
/// drives, and no tick drives on a zero Ke. Pinned for a write at each
/// phase of the medium period.
#[rstest]
#[case(Mode::Velocity, RECIP_KE_Q)]
#[case(Mode::Velocity, KE_VPC_Q)]
#[case(Mode::Position, RECIP_KE_Q)]
#[case(Mode::Position, KE_VPC_Q)]
#[test_log::test]
fn live_zero_ke_write_stops_a_running_closed_loop(#[case] mode: Mode, #[case] reg: u16) {
    stops_at_the_next_control(mode, |t| {
        if reg == RECIP_KE_Q {
            t.calib.motor.recip_ke_q = 0;
        } else {
            t.calib.motor.ke_vpc_q = 0;
        }
    });
}

/// The same belt on the two limits every closed mode clamps against: a
/// zero current cap or speed ceiling written into a running loop stops it
/// at the first CONTROL after the take, and every tick before runs on the
/// limits it had, so no closed-loop tick runs on an unset limit.
#[rstest]
#[case(Mode::Current, CURRENT_LIMIT_COUNTS)]
#[case(Mode::Current, SPEED_CEILING_CPS)]
#[case(Mode::Velocity, CURRENT_LIMIT_COUNTS)]
#[case(Mode::Velocity, SPEED_CEILING_CPS)]
#[case(Mode::Position, CURRENT_LIMIT_COUNTS)]
#[case(Mode::Position, SPEED_CEILING_CPS)]
#[test_log::test]
fn live_zero_limit_write_stops_a_running_closed_loop(#[case] mode: Mode, #[case] reg: u16) {
    stops_at_the_next_control(mode, |t| {
        if reg == CURRENT_LIMIT_COUNTS {
            t.config.limits.current_limit_counts = 0;
        } else {
            t.config.loop_position.speed_ceiling_cps = 0;
        }
    });
}

/// A running `mode` loop, `zero` committed at each phase of the medium
/// period: the drive matches an unwritten twin through the write of the
/// CONTROL after the take, then stops with the config fault, and the ack
/// re-latches.
fn stops_at_the_next_control(mode: Mode, zero: impl Fn(&mut ControlTable)) {
    const RUN: u32 = 3000;
    for phase in 0..DECIM_MED as u32 {
        let (sh, twin) = (Shared::new(), Shared::new());
        seed(&sh);
        seed(&twin);
        assert_eq!(store_bits(&sh), 0);
        let (mut rig, mut twin_rig) = (Rig::new(), Rig::new());
        for (sh, rig) in [(&sh, &mut rig), (&twin, &mut twin_rig)] {
            rig.run(sh, 200);
            enable(sh, mode);
            let cmds = rig.run(sh, RUN + phase);
            assert!(
                matches!(cmds[cmds.len() - 1], MotorCmd::Drive { .. }),
                "phase {phase}: still driving: {:?}",
                cmds[cmds.len() - 1]
            );
        }
        sh.config_commit_begin();
        set(&sh, &zero);
        sh.config_commit_end();
        // tick n runs medium phase n % DECIM_MED; the write reaches the
        // kernel at the next SPARE_A, and the belt latches at the
        // CONTROL phase after it, after that tick's write
        let ticks = 200 + RUN + phase;
        let med = DECIM_MED as u32;
        let stage = (kernel_phase::SPARE_A as u32 + med - ticks % med) % med;
        let boundary = (stage + med - kernel_phase::SPARE_A as u32) as usize;
        let published = boundary + kernel_phase::PUBLISH as usize;
        for n in 0..2 * DECIM_MED as usize {
            let cmd = rig.run(&sh, 1)[0];
            twin_rig.run(&twin, 1);
            if n <= boundary {
                assert!(
                    matches!(cmd, MotorCmd::Drive { .. }),
                    "phase {phase} tick {n}: {cmd:?}"
                );
                assert_eq!(
                    rig.duty, twin_rig.duty,
                    "phase {phase} tick {n}: off the old set"
                );
                assert_eq!(fault(&sh), (0, CODE_NONE), "phase {phase} tick {n}");
            } else {
                assert!(
                    matches!(cmd, MotorCmd::Disabled),
                    "phase {phase} tick {n}: {cmd:?}"
                );
                let want = if n < published {
                    (0, CODE_NONE)
                } else {
                    (BIT_CONFIG, CODE_CONFIG)
                };
                assert_eq!(fault(&sh), want, "phase {phase} tick {n}");
            }
        }
        assert!(
            all_disabled(&rig.run(&sh, 3 * DECIM_MED as u32)),
            "phase {phase}"
        );
        // the ack re-latches while the reason holds
        set(&sh, |t| t.control.lifecycle.torque_enable = false);
        rig.run(&sh, 20);
        set(&sh, |t| t.control.lifecycle.torque_enable = true);
        assert!(
            all_disabled(&rig.run(&sh, 3 * DECIM_MED as u32)),
            "phase {phase}"
        );
        assert_eq!(fault(&sh), (BIT_CONFIG, CODE_CONFIG), "phase {phase}");
    }
}

/// Records the tick block every row would take; the handle outlives the
/// kernel, the table it reads the kernel too.
struct RecTel(Rc<RefCell<Vec<TelemetryTick>>>, *const ControlTableCell);

impl TelStream for RecTel {
    fn on_publish(&mut self, _: impl FnOnce() -> bool) {
        // SAFETY: the test's table outlives the kernel that owns this.
        let t = unsafe { (*self.1).with(|t| t.telemetry.tick) };
        self.0.borrow_mut().push(t);
    }
}

/// The cal/ident path on a factory-fresh servo: OpenLoop and Current drive
/// with no fault latched, so no ALERT and no TEL sample carries one; a
/// mode change into closed loop under torque is what latches.
#[test_log::test]
fn virgin_servo_drives_openloop_and_current_without_alert() {
    let store = RamStore::leak();
    let sh = boot(store);
    assert_eq!(store_bits(&sh), FRESH);
    let tel = Rc::new(RefCell::new(Vec::new()));
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        RecTel(tel.clone(), &sh.table),
        TIMING,
    );
    let mut plant = Plant::new(START_POS);
    let mut duty = 0i16;
    let mut run = |sh: &Shared, ticks: u32| -> Vec<MotorCmd> {
        (0..ticks)
            .map(|_| {
                let f = plant.step(duty);
                k.on_scan(f, sh);
                duty = duty_of(&k);
                k.io.motor.last.expect("a motor write happened")
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
    // the next tick is CONTROL, which refuses the mode after its write
    assert!(matches!(cmds[0], MotorCmd::Drive { .. }), "{:?}", cmds[0]);
    assert!(all_disabled(&cmds[1..]), "{cmds:?}");
    assert_eq!(fault(&sh), (BIT_CONFIG, CODE_CONFIG));
    // TEL carried no fault until the closed-loop attempt
    let samples = tel.borrow();
    let first_fault = samples
        .iter()
        .position(|s| s.tick_flags & tick_flag::FAULT != 0)
        .expect("the latch shows");
    assert!(first_fault >= 200 + 2000, "clean through the open modes");
    assert!(
        samples[..first_fault]
            .iter()
            .all(|s| s.tick_flags & tick_flag::FAULT == 0)
    );
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

fn unsaved(sim: &mut Sim) -> bool {
    read_byte(sim, STATUS_FLAGS) & STATUS_FLAG_UNSAVED != 0
}

fn wire_store_bits(sim: &mut Sim) -> u8 {
    read_byte(sim, STATUS_FLAGS) & FRESH
}

/// A servo whose store holds one identified, SAVEd set.
fn saved_servo(sim: &mut Sim, store: &'static RamStore) -> usize {
    let s = sim.add_servo_with_store(ID5, store);
    write_ke(sim);
    assert_eq!(mgmt(sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(read_byte(sim, STATUS_FLAGS), 0);
    s
}

/// SAVE clears the unsaved bit and both store bits together: the table in
/// RAM is the one stored and the one the next boot loads. A failed program
/// changes none of them; a SAVE of a set with Ke unset clears them too, as
/// the stored state is information and never a judgment of the set.
#[apply(matrix)]
fn save_clears_unsaved_and_both_store_bits(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    sim.add_servo_with_store(ID5, store);
    assert_eq!(
        read_byte(&mut sim, STATUS_FLAGS),
        FRESH,
        "a boot reads saved"
    );
    write_ke(&mut sim);
    assert_eq!(
        read_byte(&mut sim, STATUS_FLAGS),
        STATUS_FLAG_UNSAVED | FRESH
    );
    store.set_fail(true);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    assert_eq!(
        read_byte(&mut sim, STATUS_FLAGS),
        STATUS_FLAG_UNSAVED | FRESH,
        "a failed program"
    );
    store.set_fail(false);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(read_byte(&mut sim, STATUS_FLAGS), 0);
    let mut rebooted = support::sim(baud_idx);
    rebooted.add_servo_with_store(ID5, store);
    assert_eq!(
        read_byte(&mut rebooted, STATUS_FLAGS),
        0,
        "both images load"
    );

    let store = RamStore::leak();
    let mut sim = support::sim(baud_idx);
    sim.add_servo_with_store(ID5, store);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok, "Ke unset");
    assert_eq!(read_byte(&mut sim, STATUS_FLAGS), 0);
}

/// What an identification writes (CALIB, the position table, CONFIG) all
/// persists, so each leaves the servo unsaved until SAVE: a set installed
/// and never saved reads so, and a reboot loses it. A failed SAVE leaves
/// it unsaved; a good one clears it, and the reboot boots saved.
#[apply(matrix)]
fn an_install_leaves_the_servo_unsaved_until_save(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    sim.add_servo_with_store(ID5, store);
    assert!(!unsaved(&mut sim), "boot is saved, even a fresh one");

    write_ke(&mut sim);
    assert!(unsaved(&mut sim), "a CALIB write");
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert!(!unsaved(&mut sim));

    write_ok(&mut sim, RAW_MIN, &200u16.to_le_bytes());
    write_ok(&mut sim, RAW_MAX, &3800u16.to_le_bytes());
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    write_ok(&mut sim, POS_LUT_CMD, &[cmd::COMMIT]);
    assert!(unsaved(&mut sim), "a table COMMIT");
    store.set_fail(true);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    assert!(unsaved(&mut sim), "a failed SAVE");
    store.set_fail(false);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert!(!unsaved(&mut sim));

    write_ok(&mut sim, V_KP_Q88, &70u16.to_le_bytes());
    assert!(unsaved(&mut sim), "a CONFIG write");
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, store);
    assert!(!unsaved(&mut rebooted));
    assert_ne!(
        rebooted.servo_table(s, |t| t.config.loop_velocity.v_kp_q88),
        70,
        "the reboot lost it"
    );
}

/// SAVE clears an unreadable image's bit and another layout's like a
/// blank one's: the table in RAM is what it persists and what the next
/// boot loads.
#[apply(matrix)]
fn save_clears_the_store_bits_whatever_the_store_held(baud_idx: u8) {
    type Rot = fn(&RamStore, ImageKind, Slot);
    let rots: [Rot; 2] = [
        |f, k, s| f.corrupt_slot(k, s),
        |f, k, s| f.other_layout_slot(k, s),
    ];
    for rot in rots {
        let store = RamStore::leak();
        rot(store, ImageKind::Config, Slot::A);
        rot(store, ImageKind::Calib, Slot::B);
        let mut sim = sim(baud_idx);
        sim.add_servo_with_store(ID5, store);
        assert_eq!(wire_store_bits(&mut sim), FRESH);
        write_ke(&mut sim);
        assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
        assert_eq!(read_byte(&mut sim, STATUS_FLAGS), 0);
        let mut rebooted = support::sim(baud_idx);
        rebooted.add_servo_with_store(ID5, store);
        assert_eq!(read_byte(&mut rebooted, STATUS_FLAGS), 0);
    }
}

#[apply(matrix)]
fn factory_after_corrupt_config_boots_on_defaults(baud_idx: u8) {
    let store = identified_store();
    store.corrupt_slot(ImageKind::Config, Slot::A);
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with_store(ID5, store);
    assert_eq!(wire_store_bits(&mut sim), CONFIG_DEFAULTS);
    assert_eq!(mgmt(&mut sim, MgmtOp::Factory), ResultCode::Ok);
    assert!(sim.take_reboot(s).is_some());
    let mut rebooted = support::sim(baud_idx);
    rebooted.add_servo_with_store(ID5, store);
    assert_eq!(wire_store_bits(&mut rebooted), FRESH);
}

/// A gain edit under torque never yanks the loop, and the next enable runs
/// on it: a hand edit is information, not a refusal.
#[test_log::test]
fn live_gain_edit_keeps_the_running_loop_and_the_reenable_runs() {
    let sh = Shared::new();
    seed(&sh);
    let mut rig = Rig::new();
    rig.run(&sh, 200);
    enable(&sh, Mode::Velocity);
    assert!(drives(&rig.run(&sh, 2000)));
    commit(&sh, |t| t.config.loop_velocity.v_kp_q88 = 70);
    let cmds = rig.run(&sh, 3 * DECIM_MED as u32);
    assert!(cmds.iter().all(|c| matches!(c, MotorCmd::Drive { .. })));
    assert_eq!(fault(&sh), (0, CODE_NONE));

    set(&sh, |t| t.control.lifecycle.torque_enable = false);
    rig.run(&sh, 20);
    set(&sh, |t| t.control.lifecycle.torque_enable = true);
    assert!(drives(&rig.run(&sh, 2000)));
    assert_eq!(fault(&sh), (0, CODE_NONE));
    assert_eq!(store_bits(&sh), 0);
}

/// A power cut between the CONFIG and CALIB images of one SAVE boots the
/// new CONFIG beside the old CALIB with no store bit set (protocol sec 5.7,
/// the torn SAVE). The SAVE never acked `ok`, so the host knows it did
/// not finish, and its check against its set names the mixed fields.
#[apply(matrix)]
fn torn_save_boots_the_new_config_beside_the_old_calib_unflagged(baud_idx: u8) {
    let store = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = saved_servo(&mut sim, store);
    let r = sim.servo_table(s, |t| t.calib.motor.r_q12);
    write_ok(&mut sim, V_KP_Q88, &70u16.to_le_bytes());
    write_ok(&mut sim, R_Q12, &(r + 100).to_le_bytes());
    store.tear(Tear::AfterConfig);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    assert!(unsaved(&mut sim));
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, store);
    let (v_kp, r_booted) = rebooted.servo_table(s, |t| {
        (t.config.loop_velocity.v_kp_q88, t.calib.motor.r_q12)
    });
    assert_eq!((v_kp, r_booted), (70, r), "new CONFIG, old CALIB");
    assert_eq!(read_byte(&mut rebooted, STATUS_FLAGS), 0);
}

/// Only the usability validator refuses a mode. Every image that did not
/// load (blank, unreadable or of another layout) boots on the board's
/// defaults and gates nothing, and a saved hand edit of an identified
/// value runs as saved: once the set a closed loop consumes is usable
/// (both Ke nonzero, the cap and the ceiling set), Position drives on it,
/// volatile and never saved. While Ke is unset the
/// validator alone refuses, with the `config` fault (code 7).
#[test_log::test]
fn only_the_validator_refuses_closed_loop_whatever_the_store_held() {
    const STORED: [Fixture; 4] = [
        Fixture::Loaded,
        Fixture::Virgin,
        Fixture::Corrupt,
        Fixture::OtherLayout,
    ];
    let mut cases: Vec<(String, &'static RamStore)> = Vec::new();
    for config in STORED {
        for calib in STORED {
            let store = identified_store();
            config.apply(store, ImageKind::Config);
            calib.apply(store, ImageKind::Calib);
            cases.push((format!("config {config:?} calib {calib:?}"), store));
        }
    }
    // the user tuned a gain by hand and saved it
    let edited = RamStore::leak();
    let sh = Shared::new();
    seed(&sh);
    set(&sh, |t| t.config.loop_velocity.v_kp_q88 += 8);
    edited.save_table(&sh);
    cases.push(("a saved hand edit".into(), edited));

    for (case, store) in cases {
        let sh = boot(store);
        let ke_loaded = sh.table.with(|t| t.calib.motor.recip_ke_q != 0);
        if !ke_loaded {
            let mut rig = Rig::new();
            rig.run(&sh, 200);
            enable(&sh, Mode::Position);
            assert!(all_disabled(&rig.run(&sh, 500)), "{case}: Ke unset");
            assert_eq!(fault(&sh), (BIT_CONFIG, CODE_CONFIG), "{case}: Ke unset");
            set(&sh, |t| t.control.lifecycle.torque_enable = false);
            set(&sh, |t| {
                t.calib.motor.recip_ke_q = 3700;
                t.calib.motor.ke_vpc_q = 1150;
            });
        }
        let mut rig = Rig::new();
        rig.run(&sh, 200);
        enable(&sh, Mode::Position);
        let cmds = rig.run(&sh, 3000);
        assert!(drives(&cmds), "{case}: a usable set drives");
        assert_eq!(fault(&sh), (0, CODE_NONE), "{case}");
    }
}
