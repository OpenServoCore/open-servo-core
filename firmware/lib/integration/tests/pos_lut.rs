//! The position table window over the wire (`pos_lut` module):
//! STORE/FETCH/COMMIT round trips at every baud, every refusal leaving the
//! identity behind, the stamp checkpoint a COMMIT runs, the table's place in
//! the CALIB image through SAVE, reboot, torn saves, rot and FACTORY, and the
//! kernel's endstop against the plant rig with the table LIVE. The mg90-a
//! table (`support`) is the same 256 points the core unit tests carry
//! (bringup captures/mg90/pot-lut-mg90-a-grid.json), pinned to the Python
//! reference by CRC.

use std::cell::RefCell;
use std::rc::Rc;

use osc_integration::plant::{
    FakeIo, FakeMotor, FakeSensors, Plant, TIMING, duty_of, kernel, last_cmd, lut_live, seed,
};
use osc_integration::sim::{
    ImageKind, RamStore, Sim, Source, Tear, WireFrame, assert_valid, instruction, status,
};
use osc_protocol::crc::osc_crc_continue;
use osc_protocol::wire::{MgmtOp, Opcode, ResultCode};
use osc_servo_core::data_state::{
    CALIB_CORRUPT, CALIB_STALE, CALIB_VIRGIN, CONFIG_VIRGIN, PLANT_UNSET, STAMP_MISMATCH,
};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::persist::{CalibImage, Slot};
use osc_servo_core::pos_lut::{INTERVALS, PAGE_POINTS, PAGES, POINTS, cmd, interp_q4, state};
use osc_servo_core::regions::CALIB_BASE_ADDR;
use osc_servo_core::regions::calib::addr::motor::{KE_VPC_Q, RECIP_KE_Q};
use osc_servo_core::regions::calib::addr::pot::{RAW_MAX, RAW_MIN};
use osc_servo_core::regions::calib::addr::stamp::PLANT_STAMP;
use osc_servo_core::regions::control::addr::lifecycle::TORQUE_ENABLE;
use osc_servo_core::regions::control::addr::pos_lut::{
    POS_LUT_CMD, POS_LUT_PAGE, POS_LUT_POINTS, POS_LUT_STATE,
};
use osc_servo_core::regions::telemetry::addr::mode::DATA_FLAGS;
use osc_servo_core::stamp::compute;
use osc_servo_core::tel::{TelSample, TelStream};
use osc_servo_core::{Kernel, Mode, MotorCmd, RegionStorage, Shared};
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{MG90_A, MG90_A_MAX, MG90_A_MIN, matrix, mg90_a, sim};

const ID5: u8 = 5;
const MG90_A_Q4_CRC: u16 = 0x8F97;
const ZERO: [i16; POINTS] = [0; POINTS];

fn sole_reply(frames: &[WireFrame]) -> &WireFrame {
    let replies: Vec<&WireFrame> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "expected one servo reply: {frames:#?}");
    assert_valid(replies[0]);
    replies[0]
}

fn write(sim: &mut Sim, addr: u16, data: &[u8]) -> ResultCode {
    let a = addr.to_le_bytes();
    let mut payload = vec![a[0], a[1]];
    payload.extend_from_slice(data);
    sim.host_send(&instruction(ID5, Opcode::Write, 0, &payload));
    let frames = sim.run();
    status(sole_reply(&frames)).0.result().expect("result code")
}

fn write_ok(sim: &mut Sim, addr: u16, data: &[u8]) {
    assert_eq!(write(sim, addr, data), ResultCode::Ok, "write {addr:#06x}");
}

fn read(sim: &mut Sim, addr: u16, count: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let c = count.to_le_bytes();
    sim.host_send(&instruction(
        ID5,
        Opcode::Read,
        0,
        &[a[0], a[1], c[0], c[1]],
    ));
    let frames = sim.run();
    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    payload.to_vec()
}

fn read_byte(sim: &mut Sim, addr: u16) -> u8 {
    read(sim, addr, 1)[0]
}

fn mgmt(sim: &mut Sim, op: MgmtOp) -> ResultCode {
    sim.host_send(&instruction(ID5, Opcode::Mgmt, 0, &[op as u8]));
    let frames = sim.run();
    status(sole_reply(&frames)).0.result().expect("result code")
}

fn pos_lut_state(sim: &mut Sim) -> u8 {
    read_byte(sim, POS_LUT_STATE)
}

fn data_flags(sim: &mut Sim) -> u8 {
    read_byte(sim, DATA_FLAGS)
}

fn set_torque(sim: &mut Sim, on: bool) {
    write_ok(sim, TORQUE_ENABLE, &[on as u8]);
}

/// The host's stamp over the set it intends: what the servo holds now,
/// under the points it expects the kernel to apply.
fn stamp(sim: &mut Sim, s: usize, points: Option<&[i16; INTERVALS]>) {
    let v = sim.servo_table(s, |t| compute(t, points));
    write_ok(sim, PLANT_STAMP, &v.to_le_bytes());
}

fn page_bytes(k: &[i16; POINTS], page: usize) -> Vec<u8> {
    k[page * PAGE_POINTS..][..PAGE_POINTS]
        .iter()
        .flat_map(|c| c.to_le_bytes())
        .collect()
}

/// One 66 B WRITE: page, STORE and the page's points; the state it left.
fn store(sim: &mut Sim, page: usize, k: &[i16; POINTS]) -> u8 {
    let mut w = vec![page as u8, cmd::STORE];
    w.extend(page_bytes(k, page));
    write_ok(sim, POS_LUT_PAGE, &w);
    pos_lut_state(sim)
}

fn store_all(sim: &mut Sim, k: &[i16; POINTS]) {
    for page in 0..PAGES {
        assert_eq!(store(sim, page, k), state::LOADING, "page {page}");
    }
}

fn commit(sim: &mut Sim) -> u8 {
    write_ok(sim, POS_LUT_CMD, &[cmd::COMMIT]);
    pos_lut_state(sim)
}

fn fetch(sim: &mut Sim, page: usize) -> Vec<u8> {
    write_ok(sim, POS_LUT_PAGE, &[page as u8, cmd::FETCH]);
    read(sim, POS_LUT_POINTS, 2 * PAGE_POINTS as u16)
}

fn array(sim: &Sim, s: usize) -> [i16; POINTS] {
    sim.servo_pos_lut(s, |k| *k)
}

/// A servo identified, stamped and SAVEd with the mg90-a stops: every
/// reason clear, the identity in place.
fn mg90_servo(sim: &mut Sim, store: &'static RamStore) -> usize {
    let s = sim.add_servo_with_store(ID5, store);
    write_ok(sim, RECIP_KE_Q, &3700u16.to_le_bytes());
    write_ok(sim, KE_VPC_Q, &1150u16.to_le_bytes());
    write_ok(sim, RAW_MIN, &MG90_A_MIN.to_le_bytes());
    write_ok(sim, RAW_MAX, &MG90_A_MAX.to_le_bytes());
    stamp(sim, s, None);
    assert_eq!(mgmt(sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(data_flags(sim), 0);
    assert_eq!(pos_lut_state(sim), state::IDENTITY);
    s
}

#[test]
fn mg90_a_copy_matches_the_python_reference() {
    let k = mg90_a();
    let mut crc = 0;
    for raw in 0..4096u16 {
        crc = osc_crc_continue(crc, &interp_q4(raw, &k).to_le_bytes());
    }
    assert_eq!(crc, MG90_A_Q4_CRC);
}

#[apply(matrix)]
fn fresh_servo_reports_identity(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with_store(ID5, RamStore::leak());
    let window = read(&mut sim, POS_LUT_PAGE, 2 + 2 * PAGE_POINTS as u16 + 1);
    assert!(window.iter().all(|&b| b == 0), "{window:02x?}");
    assert_eq!(pos_lut_state(&mut sim), state::IDENTITY);
    assert_eq!(fetch(&mut sim, PAGES - 1), vec![0; 2 * PAGE_POINTS]);
    assert_eq!(array(&sim, s), ZERO);
}

/// The `osc lut write` sequence: eight STOREs, a COMMIT, LIVE, the stamp
/// checkpoint refusing closed loop until a restamp over the live array,
/// and a point-for-point FETCH readback.
#[apply(matrix)]
fn store_commit_goes_live_and_fetch_reads_back(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, RamStore::leak());
    let k = mg90_a();
    store_all(&mut sim, &k);
    assert_eq!(data_flags(&mut sim), 0, "loading applies the identity");
    assert_eq!(commit(&mut sim), state::LIVE);
    assert_eq!(array(&sim, s), k);
    assert_eq!(
        read_byte(&mut sim, POS_LUT_CMD),
        cmd::NONE,
        "the command cleared"
    );
    assert_eq!(
        data_flags(&mut sim),
        STAMP_MISMATCH,
        "the hashed points changed: run osc ident"
    );
    stamp(&mut sim, s, Some(&MG90_A));
    assert_eq!(data_flags(&mut sim), 0);
    for page in 0..PAGES {
        assert_eq!(fetch(&mut sim, page), page_bytes(&k, page), "page {page}");
    }
    assert_eq!(pos_lut_state(&mut sim), state::LIVE, "fetch moves nothing");
    // an all-zero table LIVE hashes like the identity
    store_all(&mut sim, &ZERO);
    assert_eq!(data_flags(&mut sim), STAMP_MISMATCH, "left LIVE");
    assert_eq!(commit(&mut sim), state::LIVE);
    assert_eq!(array(&sim, s), ZERO);
    assert_eq!(
        data_flags(&mut sim),
        STAMP_MISMATCH,
        "the stamp covered the mg90-a points"
    );
    stamp(&mut sim, s, None);
    assert_eq!(data_flags(&mut sim), 0);
}

/// COMMIT's reply owes nothing to the validation or the CRC: the commit
/// site leaves LOADING (the kernel applies the identity), marks the stamp
/// and posts; the main loop lands the verdict. With the poll held off the
/// wire sees LOADING behind an `ok` reply; a STORE behind the posted
/// COMMIT cancels it (loading again, only the next COMMIT judges); torque
/// coming on mid-job lands the refusal HIGH would have given; SAVE runs a
/// posted job before it persists.
#[apply(matrix)]
fn commit_lands_its_verdict_in_the_main_loop_after_the_reply(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, RamStore::leak());
    let k = mg90_a();
    sim.set_data_jobs(false);
    store_all(&mut sim, &k);
    assert_eq!(commit(&mut sim), state::LOADING, "the reply left unjudged");
    assert_eq!(data_flags(&mut sim), STAMP_MISMATCH, "refused until judged");
    assert!(sim.poll_data_job(s));
    assert_eq!(pos_lut_state(&mut sim), state::LIVE);
    assert_eq!(
        data_flags(&mut sim),
        STAMP_MISMATCH,
        "the hashed points changed"
    );

    // a STORE behind a posted COMMIT cancels it
    assert_eq!(commit(&mut sim), state::LOADING);
    assert_eq!(store(&mut sim, 0, &k), state::LOADING);
    assert!(!sim.poll_data_job(s), "nothing posted");
    assert_eq!(pos_lut_state(&mut sim), state::LOADING);
    assert_eq!(commit(&mut sim), state::LOADING);
    assert!(sim.poll_data_job(s));
    assert_eq!(pos_lut_state(&mut sim), state::LIVE);

    // torque comes on between the run and its publish
    assert_eq!(commit(&mut sim), state::LOADING);
    let run = sim.data_job_run(s).expect("posted");
    set_torque(&mut sim, true);
    assert!(!sim.data_job_publish(s, run));
    assert_eq!(pos_lut_state(&mut sim), state::LOADING);
    assert!(sim.poll_data_job(s));
    assert_eq!(pos_lut_state(&mut sim), state::REJECT_TORQUE);
    assert_eq!(array(&sim, s), k, "the array stays for the next COMMIT");
    set_torque(&mut sim, false);
    assert_eq!(commit(&mut sim), state::LOADING);
    assert!(sim.poll_data_job(s));
    assert_eq!(pos_lut_state(&mut sim), state::LIVE);

    // SAVE judges a posted COMMIT itself: the table persists LIVE
    assert_eq!(commit(&mut sim), state::LOADING);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(pos_lut_state(&mut sim), state::LIVE);
    assert_eq!(array(&sim, s), k);
    assert!(!sim.poll_data_job(s));
}

#[apply(matrix)]
fn commit_rejects_ends_and_shape_and_leaves_identity(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, RamStore::leak());
    // a nonzero point inside the low inset of stop 209
    let mut ends = mg90_a();
    ends[14] = 1;
    store_all(&mut sim, &ends);
    assert_eq!(commit(&mut sim), state::REJECT_ENDS);
    assert_eq!(
        data_flags(&mut sim),
        0,
        "the identity is what the stamp covers"
    );
    let mut shape = mg90_a();
    shape[100] = shape[99] - 16;
    store_all(&mut sim, &shape);
    assert_eq!(commit(&mut sim), state::REJECT_SHAPE);
    assert_eq!(data_flags(&mut sim), 0);
    assert_eq!(
        array(&sim, s),
        shape,
        "the rejected array stays to be fixed"
    );
    // the stops move into a live table: LIVE falls back
    store_all(&mut sim, &mg90_a());
    assert_eq!(commit(&mut sim), state::LIVE);
    stamp(&mut sim, s, Some(&MG90_A));
    assert_eq!(data_flags(&mut sim), 0);
    write_ok(&mut sim, RAW_MAX, &3000u16.to_le_bytes());
    assert_eq!(commit(&mut sim), state::REJECT_ENDS);
    assert_eq!(data_flags(&mut sim), STAMP_MISMATCH);
}

#[apply(matrix)]
fn store_with_torque_on_is_refused(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, RamStore::leak());
    let k = mg90_a();
    set_torque(&mut sim, true);
    assert_eq!(store(&mut sim, 2, &k), state::REJECT_TORQUE);
    assert_eq!(array(&sim, s), ZERO);
    assert_eq!(commit(&mut sim), state::REJECT_TORQUE);
    assert_eq!(data_flags(&mut sim), 0);
    assert_eq!(
        fetch(&mut sim, 2),
        vec![0; 2 * PAGE_POINTS],
        "fetch is not gated"
    );
    set_torque(&mut sim, false);
    store_all(&mut sim, &k);
    assert_eq!(commit(&mut sim), state::LIVE);
    stamp(&mut sim, s, Some(&MG90_A));
    assert_eq!(data_flags(&mut sim), 0);
    // under torque a live table stands: a refusal never moves what the
    // kernel applies, and the host sees the state not advance
    set_torque(&mut sim, true);
    assert_eq!(store(&mut sim, 2, &ZERO), state::LIVE);
    assert_eq!(commit(&mut sim), state::LIVE);
    assert_eq!(array(&sim, s), k);
    assert_eq!(data_flags(&mut sim), 0);
}

/// A host that died mid-load, or left a rejected array behind: neither is
/// what the kernel applies, so SAVE drops the array to the identity and
/// persists that, and the reboot is the identity too.
#[apply(matrix)]
fn save_while_loading_or_rejected_persists_identity(baud_idx: u8) {
    let flash = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, flash);
    let k = mg90_a();
    for page in 0..3 {
        assert_eq!(store(&mut sim, page, &k), state::LOADING);
    }
    assert_eq!(data_flags(&mut sim), 0);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(
        data_flags(&mut sim),
        0,
        "the checkpoint hashes the identity"
    );
    assert_eq!(pos_lut_state(&mut sim), state::IDENTITY);
    assert_eq!(array(&sim, s), ZERO, "the pages are gone");
    let img = flash
        .calib_slot(Slot::B)
        .expect("the second save lands in B");
    assert_eq!(points_of(&img), [0; INTERVALS]);
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(pos_lut_state(&mut rebooted), state::IDENTITY);
    assert_eq!(array(&rebooted, s), ZERO);
    assert_eq!(data_flags(&mut rebooted), 0);

    let mut shape = k;
    shape[100] = shape[99] - 16;
    store_all(&mut rebooted, &shape);
    assert_eq!(commit(&mut rebooted), state::REJECT_SHAPE);
    assert_eq!(mgmt(&mut rebooted, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(pos_lut_state(&mut rebooted), state::IDENTITY);
    assert_eq!(array(&rebooted, s), ZERO);
    assert_eq!(data_flags(&mut rebooted), 0);
    let mut again = support::sim(baud_idx);
    let s = again.add_servo_with_store(ID5, flash);
    assert_eq!(pos_lut_state(&mut again), state::IDENTITY);
    assert_eq!(array(&again, s), ZERO);
}

// The table in the CALIB image: SAVE, reboot, torn saves, rot, FACTORY

const FRESH: u8 = CONFIG_VIRGIN | CALIB_VIRGIN | STAMP_MISMATCH | PLANT_UNSET;

/// The mg90-a table loaded, LIVE and stamped on `s`.
fn go_live(sim: &mut Sim, s: usize) {
    store_all(sim, &mg90_a());
    assert_eq!(commit(sim), state::LIVE);
    stamp(sim, s, Some(&MG90_A));
    assert_eq!(data_flags(sim), 0);
}

fn points_of(img: &[u8]) -> [i16; INTERVALS] {
    let parsed = CalibImage::parse(img).expect("stored calib image parses");
    let mut k = [0; INTERVALS];
    for (d, s) in k.iter_mut().zip(parsed.lut_points.as_chunks::<2>().0) {
        *d = i16::from_le_bytes(*s);
    }
    k
}

/// The per-servo switch's last step: one SAVE persists CONFIG and CALIB
/// with the LUT inside it; the reboot loads it LIVE, point for point, with
/// every reason clear; FACTORY wipes it back to the virgin identity.
#[apply(matrix)]
fn lut_survives_save_and_reboot_until_factory(baud_idx: u8) {
    let flash = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, flash);
    go_live(&mut sim, s);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(data_flags(&mut sim), 0);
    assert_eq!(pos_lut_state(&mut sim), state::LIVE);
    let img = flash
        .calib_slot(Slot::B)
        .expect("the second save lands in B");
    let parsed = CalibImage::parse(&img).expect("parses");
    assert_eq!(parsed.seq, 2);
    assert_eq!(
        parsed.calib[(RAW_MAX - CALIB_BASE_ADDR) as usize..][..2],
        MG90_A_MAX.to_le_bytes(),
        "the stops ride beside their table"
    );
    assert_eq!(points_of(&img), MG90_A);

    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(pos_lut_state(&mut rebooted), state::LIVE);
    assert_eq!(array(&rebooted, s), mg90_a());
    assert_eq!(data_flags(&mut rebooted), 0);
    for page in 0..PAGES {
        assert_eq!(
            fetch(&mut rebooted, page),
            page_bytes(&mg90_a(), page),
            "page {page}"
        );
    }

    assert_eq!(mgmt(&mut rebooted, MgmtOp::Factory), ResultCode::Ok);
    assert!(rebooted.take_reboot(s).is_some());
    assert!(flash.calib_slot(Slot::A).is_none() && flash.calib_slot(Slot::B).is_none());
    let mut fresh = support::sim(baud_idx);
    let s = fresh.add_servo_with_store(ID5, flash);
    assert_eq!(pos_lut_state(&mut fresh), state::IDENTITY);
    assert_eq!(array(&fresh, s), ZERO);
    assert_eq!(data_flags(&mut fresh), FRESH);
}

/// A power cut partway through the CALIB program, both ways round: the
/// slot being written is torn, the other still holds the last save whole -
/// calibration, stamp and table together - so the reboot is that save with
/// every reason clear. The image is all or nothing.
#[apply(matrix)]
fn torn_calib_save_boots_the_previous_calibration_with_its_tables(baud_idx: u8) {
    // the table went LIVE and was restamped since the identity save
    let flash = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, flash);
    go_live(&mut sim, s);
    flash.tear(Tear::MidCalib);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    let torn = flash
        .calib_slot(Slot::B)
        .expect("the second save tore in B");
    assert!(CalibImage::parse(&torn).is_none());
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(pos_lut_state(&mut rebooted), state::IDENTITY);
    assert_eq!(array(&rebooted, s), ZERO);
    assert_eq!(
        data_flags(&mut rebooted),
        0,
        "the old stamp, the old identity"
    );

    // the table SAVEd, then dropped to the identity and restamped: the old
    // image holds the table under the stamp that covered it
    let flash = RamStore::leak();
    let mut sim = support::sim(baud_idx);
    let s = mg90_servo(&mut sim, flash);
    go_live(&mut sim, s);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    store_all(&mut sim, &ZERO);
    assert_eq!(commit(&mut sim), state::LIVE);
    stamp(&mut sim, s, None);
    assert_eq!(data_flags(&mut sim), 0);
    flash.tear(Tear::MidCalib);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(
        pos_lut_state(&mut rebooted),
        state::LIVE,
        "the old table loads"
    );
    assert_eq!(array(&rebooted, s), mg90_a());
    assert_eq!(data_flags(&mut rebooted), 0);
}

/// Flash rot or a save from another layout: the image does not load, and
/// its tables go with it - the identity runs under the CALIB_* reason, the
/// board's never-stamped defaults under the saved gains. The older sound
/// slot, when one stands, loads whole.
#[apply(matrix)]
fn corrupt_or_stale_calib_image_boots_identity_under_its_reason(baud_idx: u8) {
    type Rot = fn(&RamStore, Slot);
    let rots: [(Rot, u8); 2] = [
        (|f, s| f.corrupt_slot(ImageKind::Calib, s), CALIB_CORRUPT),
        (|f, s| f.stale_slot(ImageKind::Calib, s), CALIB_STALE),
    ];
    for (rot, reason) in rots {
        let flash = RamStore::leak();
        let mut sim = sim(baud_idx);
        let s = mg90_servo(&mut sim, flash);
        go_live(&mut sim, s);
        assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
        // the newest image (the table) in B rots; the identity in A is
        // older and still sound
        rot(flash, Slot::B);
        let mut rebooted = support::sim(baud_idx);
        let s = rebooted.add_servo_with_store(ID5, flash);
        assert_eq!(pos_lut_state(&mut rebooted), state::IDENTITY);
        assert_eq!(array(&rebooted, s), ZERO);
        assert_eq!(data_flags(&mut rebooted), 0, "the older save, whole");
        rot(flash, Slot::A);
        let mut again = support::sim(baud_idx);
        let s = again.add_servo_with_store(ID5, flash);
        assert_eq!(pos_lut_state(&mut again), state::IDENTITY);
        assert_eq!(array(&again, s), ZERO);
        assert_eq!(
            data_flags(&mut again),
            reason | STAMP_MISMATCH | PLANT_UNSET
        );
    }
}

// The kernel with the table LIVE, against the plant rig

/// An outward OpenLoop push from rest at raw `pos` against soft limits
/// `(min, max)`: whether the endstop brakes it.
fn endstop_blocks(lut: Option<&[i16; POINTS]>, pos: u16, soft: (i32, i32), duty: i16) -> bool {
    let sh = Shared::new();
    seed(&sh);
    if let Some(k) = lut {
        lut_live(&sh, k);
    }
    sh.table.with_mut(|t| {
        t.config.pos_limits.pos_min_soft_counts = soft.0;
        t.config.pos_limits.pos_max_soft_counts = soft.1;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = duty;
        t.control.lifecycle.torque_enable = true;
    });
    let mut k = kernel();
    let f = Plant::new(pos).step(0);
    for _ in 0..2 * DECIM_MED {
        k.on_tick(f, &sh);
    }
    matches!(last_cmd(&k), MotorCmd::Brake)
}

/// The soft limits sit inside the identity insets (mg90-a: 432 and 3626
/// against stops 209/3849, zero points up to raw 544 and from 3520), so the
/// endstop brake trips at the same raw count with the table LIVE as
/// without it; mid-travel the wall is a linearized count, and raw 2048
/// (2081 linearized) is already past a wall at 2060.
#[test]
fn endstop_trips_at_the_same_raw_counts_under_a_live_lut() {
    let k = mg90_a();
    let soft = (432, 3626);
    for lut in [None, Some(&k)] {
        assert!(endstop_blocks(lut, 3626, soft, 4000), "{:?}", lut.is_some());
        assert!(
            !endstop_blocks(lut, 3625, soft, 4000),
            "{:?}",
            lut.is_some()
        );
        assert!(endstop_blocks(lut, 432, soft, -4000), "{:?}", lut.is_some());
        assert!(
            !endstop_blocks(lut, 433, soft, -4000),
            "{:?}",
            lut.is_some()
        );
    }
    assert_eq!(interp_q4(3626, &k), 3626 << 4);
    assert_eq!(interp_q4(432, &k), 432 << 4);
    assert!(!endstop_blocks(None, 2048, (432, 2060), 4000));
    assert!(endstop_blocks(Some(&k), 2048, (432, 2060), 4000));
    assert_eq!(interp_q4(2048, &k) >> 4, 2081);
}

/// An always-armed sink; the kernel owns it, so the recording is shared.
struct RecTel(Rc<RefCell<Vec<TelSample>>>);
impl TelStream for RecTel {
    fn active(&self) -> bool {
        true
    }
    fn on_tick(&mut self, sample: &TelSample) {
        self.0.borrow_mut().push(*sample);
    }
}

/// TEL `pos_lin` is the Q4 word the kernel controls on: `interp_q4(pos)`
/// over the live table on every sample, at rest and across an OpenLoop
/// traverse both ways, `pos << 4` beside it once the table is gone.
#[test]
fn tel_pos_lin_is_interp_q4_of_pos_on_every_sample() {
    let k = mg90_a();
    let sh = Shared::new();
    seed(&sh);
    lut_live(&sh, &k);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.torque_enable = true;
    });
    let rec = Rc::new(RefCell::new(Vec::new()));
    let mut kn = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        RecTel(rec.clone()),
        TIMING,
    );
    let mut plant = Plant::new(1200);
    let mut duty = 0i16;
    for t in 0..12_000u32 {
        match t {
            2_000 => sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 6000),
            7_000 => sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -6000),
            11_000 => sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 0),
            _ => {}
        }
        kn.on_tick(plant.step(duty), &sh);
        duty = duty_of(kn.io.motor.last.expect("a motor write happened"));
    }
    let samples = std::mem::take(&mut *rec.borrow_mut());
    assert_eq!(samples.len(), 12_000);
    let (lo, hi) = samples
        .iter()
        .fold((u16::MAX, 0), |(lo, hi), s| (lo.min(s.pos), hi.max(s.pos)));
    let back = samples.last().expect("samples").pos;
    assert!(
        hi >= lo + 300 && back + 100 < hi,
        "traversed {lo}..{hi} and back to {back}"
    );
    for (i, s) in samples.iter().enumerate() {
        assert_eq!(s.pos_lin_q4, interp_q4(s.pos, &k), "sample {i}: {s:?}");
    }
    assert!(
        samples.iter().any(|s| s.pos_lin_q4 != s.pos << 4),
        "the table moved something"
    );

    sh.with_pos_lut_mut(|a| a.fill(0));
    sh.table
        .with_mut(|t| t.control.pos_lut.pos_lut_state = state::IDENTITY);
    for _ in 0..DECIM_MED {
        kn.on_tick(plant.step(0), &sh);
    }
    for s in rec.borrow().iter() {
        assert_eq!(s.pos_lin_q4, s.pos << 4, "{s:?}");
    }
}

/// A page past the last or an unknown command is a Validation nack: the
/// write stages nothing, so no command runs and the state stands.
#[apply(matrix)]
fn out_of_range_page_or_command_is_refused(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, RamStore::leak());
    let k = mg90_a();
    let mut w = vec![PAGES as u8, cmd::STORE];
    w.extend(page_bytes(&k, 0));
    assert_eq!(write(&mut sim, POS_LUT_PAGE, &w), ResultCode::Validation);
    assert_eq!(
        write(&mut sim, POS_LUT_CMD, &[cmd::MAX + 1]),
        ResultCode::Validation
    );
    assert_eq!(pos_lut_state(&mut sim), state::IDENTITY);
    assert_eq!(array(&sim, s), ZERO);
    assert_eq!(
        write(&mut sim, POS_LUT_STATE, &[state::LIVE]),
        ResultCode::Access
    );
    assert_eq!(pos_lut_state(&mut sim), state::IDENTITY);
}
