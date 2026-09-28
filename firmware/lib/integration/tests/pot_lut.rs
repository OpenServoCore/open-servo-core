//! The pot LUT window over the wire (`pot_lut` module): STORE/FETCH/COMMIT
//! round trips at every baud, every refusal leaving the identity behind,
//! the stamp checkpoint a COMMIT runs, and the table's own flash image
//! through SAVE, reboot, torn saves, rot and FACTORY. The mg90-a table
//! below is the same 256 knots the core unit tests carry (bringup
//! captures/mg90/pot-lut-mg90-a-grid.json), pinned to the Python reference
//! by CRC.

use osc_integration::sim::{
    ImageKind, RamStore, Sim, Source, WireFrame, assert_valid, instruction, status,
};
use osc_protocol::crc::osc_crc_continue;
use osc_protocol::wire::{MgmtOp, Opcode, ResultCode};
use osc_servo_core::data_state::{CALIB_VIRGIN, CONFIG_VIRGIN, PLANT_UNSET, STAMP_MISMATCH};
use osc_servo_core::persist::{LutImage, Slot};
use osc_servo_core::pot_lut::{INTERVALS, KNOTS, PAGE_KNOTS, PAGES, cmd, interp_q4, state};
use osc_servo_core::regions::calib::addr::motor::{KE_VPC_Q, RECIP_KE_Q};
use osc_servo_core::regions::calib::addr::pot::{RAW_MAX, RAW_MIN};
use osc_servo_core::regions::calib::addr::stamp::PLANT_STAMP;
use osc_servo_core::regions::control::addr::lifecycle::TORQUE_ENABLE;
use osc_servo_core::regions::control::addr::pot_lut::{LUT_CMD, LUT_KNOTS, LUT_PAGE, LUT_STATE};
use osc_servo_core::regions::telemetry::addr::mode::DATA_FLAGS;
use osc_servo_core::stamp::compute;
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{matrix, sim};

const ID5: u8 = 5;
const MG90_A_MIN: u16 = 209;
const MG90_A_MAX: u16 = 3849;
const MG90_A: [i16; INTERVALS] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 4, -1, -5, -10, -16, -21, -23, -21, -14, -12, -11, -11, -14, //
    -22, -25, -28, -28, -25, -27, -28, -29, -31, -30, -22, -23, -24, -20, -18, -20, //
    -21, -29, -32, -35, -34, -33, -34, -33, -28, -27, -25, -25, -27, -30, -28, -25, //
    -28, -24, -23, -23, -25, -23, -6, 3, 2, -2, -6, -8, -8, -3, -3, -7, //
    -12, -14, -16, -17, -17, -12, 2, 15, 19, 16, 15, 17, 22, 25, 28, 30, //
    28, 29, 29, 28, 25, 21, 19, 16, 14, 13, 12, 11, 6, 10, 20, 31, //
    33, 38, 42, 42, 39, 35, 30, 26, 23, 22, 23, 24, 22, 26, 27, 30, //
    34, 38, 41, 44, 42, 42, 42, 41, 43, 44, 45, 44, 43, 44, 45, 44, //
    42, 42, 45, 47, 53, 59, 66, 71, 70, 70, 69, 65, 60, 56, 51, 47, //
    44, 45, 43, 43, 42, 40, 39, 38, 39, 37, 35, 34, 33, 30, 27, 23, //
    20, 18, 14, 9, 7, 8, 13, 16, 17, 18, 16, 13, 7, 6, 5, 6, //
    3, 2, 7, 10, 10, 8, 13, 12, 9, 6, 2, -1, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
];
const MG90_A_Q4_CRC: u16 = 0x8F97;
const ZERO: [i16; KNOTS] = [0; KNOTS];

fn mg90_a() -> [i16; KNOTS] {
    let mut k = ZERO;
    k[..INTERVALS].copy_from_slice(&MG90_A);
    k
}

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

fn lut_state(sim: &mut Sim) -> u8 {
    read_byte(sim, LUT_STATE)
}

fn data_flags(sim: &mut Sim) -> u8 {
    read_byte(sim, DATA_FLAGS)
}

fn set_torque(sim: &mut Sim, on: bool) {
    write_ok(sim, TORQUE_ENABLE, &[on as u8]);
}

/// The host's stamp over the set it intends: what the servo holds now,
/// under the knots it expects the kernel to apply.
fn stamp(sim: &mut Sim, s: usize, knots: Option<&[i16; INTERVALS]>) {
    let v = sim.servo_table(s, |t| compute(t, knots));
    write_ok(sim, PLANT_STAMP, &v.to_le_bytes());
}

fn page_bytes(k: &[i16; KNOTS], page: usize) -> Vec<u8> {
    k[page * PAGE_KNOTS..][..PAGE_KNOTS]
        .iter()
        .flat_map(|c| c.to_le_bytes())
        .collect()
}

/// One 66 B WRITE: page, STORE and the page's knots; the state it left.
fn store(sim: &mut Sim, page: usize, k: &[i16; KNOTS]) -> u8 {
    let mut w = vec![page as u8, cmd::STORE];
    w.extend(page_bytes(k, page));
    write_ok(sim, LUT_PAGE, &w);
    lut_state(sim)
}

fn store_all(sim: &mut Sim, k: &[i16; KNOTS]) {
    for page in 0..PAGES {
        assert_eq!(store(sim, page, k), state::LOADING, "page {page}");
    }
}

fn commit(sim: &mut Sim) -> u8 {
    write_ok(sim, LUT_CMD, &[cmd::COMMIT]);
    lut_state(sim)
}

fn fetch(sim: &mut Sim, page: usize) -> Vec<u8> {
    write_ok(sim, LUT_PAGE, &[page as u8, cmd::FETCH]);
    read(sim, LUT_KNOTS, 2 * PAGE_KNOTS as u16)
}

fn array(sim: &Sim, s: usize) -> [i16; KNOTS] {
    sim.servo_pot_lut(s, |k| *k)
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
    assert_eq!(lut_state(sim), state::IDENTITY);
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
    let window = read(&mut sim, LUT_PAGE, 2 + 2 * PAGE_KNOTS as u16 + 1);
    assert!(window.iter().all(|&b| b == 0), "{window:02x?}");
    assert_eq!(lut_state(&mut sim), state::IDENTITY);
    assert_eq!(fetch(&mut sim, PAGES - 1), vec![0; 2 * PAGE_KNOTS]);
    assert_eq!(array(&sim, s), ZERO);
}

/// The `osc lut write` sequence: eight STOREs, a COMMIT, LIVE, the stamp
/// checkpoint refusing closed loop until a restamp over the live array,
/// and a knot-for-knot FETCH readback.
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
        read_byte(&mut sim, LUT_CMD),
        cmd::NONE,
        "the command cleared"
    );
    assert_eq!(
        data_flags(&mut sim),
        STAMP_MISMATCH,
        "the hashed knots changed: run osc ident"
    );
    stamp(&mut sim, s, Some(&MG90_A));
    assert_eq!(data_flags(&mut sim), 0);
    for page in 0..PAGES {
        assert_eq!(fetch(&mut sim, page), page_bytes(&k, page), "page {page}");
    }
    assert_eq!(lut_state(&mut sim), state::LIVE, "fetch moves nothing");
    // an all-zero table LIVE hashes like the identity
    store_all(&mut sim, &ZERO);
    assert_eq!(data_flags(&mut sim), STAMP_MISMATCH, "left LIVE");
    assert_eq!(commit(&mut sim), state::LIVE);
    assert_eq!(array(&sim, s), ZERO);
    assert_eq!(
        data_flags(&mut sim),
        STAMP_MISMATCH,
        "the stamp covered the mg90-a knots"
    );
    stamp(&mut sim, s, None);
    assert_eq!(data_flags(&mut sim), 0);
}

#[apply(matrix)]
fn commit_rejects_ends_and_shape_and_leaves_identity(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, RamStore::leak());
    // a nonzero knot inside the low inset of stop 209
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
        vec![0; 2 * PAGE_KNOTS],
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
    assert_eq!(lut_state(&mut sim), state::IDENTITY);
    assert_eq!(array(&sim, s), ZERO, "the pages are gone");
    let img = flash.lut_slot(Slot::B).expect("the second save lands in B");
    let parsed = LutImage::parse(&img).expect("stored lut image parses");
    assert!(parsed.knots.iter().all(|&b| b == 0));
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(lut_state(&mut rebooted), state::IDENTITY);
    assert_eq!(array(&rebooted, s), ZERO);
    assert_eq!(data_flags(&mut rebooted), 0);

    let mut shape = k;
    shape[100] = shape[99] - 16;
    store_all(&mut rebooted, &shape);
    assert_eq!(commit(&mut rebooted), state::REJECT_SHAPE);
    assert_eq!(mgmt(&mut rebooted, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(lut_state(&mut rebooted), state::IDENTITY);
    assert_eq!(array(&rebooted, s), ZERO);
    assert_eq!(data_flags(&mut rebooted), 0);
    let mut again = support::sim(baud_idx);
    let s = again.add_servo_with_store(ID5, flash);
    assert_eq!(lut_state(&mut again), state::IDENTITY);
    assert_eq!(array(&again, s), ZERO);
}

// The LUT image: SAVE, reboot, torn saves, rot, FACTORY

const FRESH: u8 = CONFIG_VIRGIN | CALIB_VIRGIN | STAMP_MISMATCH | PLANT_UNSET;

/// The mg90-a table loaded, LIVE and stamped on `s`.
fn go_live(sim: &mut Sim, s: usize) {
    store_all(sim, &mg90_a());
    assert_eq!(commit(sim), state::LIVE);
    stamp(sim, s, Some(&MG90_A));
    assert_eq!(data_flags(sim), 0);
}

fn knots_of(img: &[u8]) -> [i16; INTERVALS] {
    let parsed = LutImage::parse(img).expect("stored lut image parses");
    let mut k = [0; INTERVALS];
    for (d, s) in k.iter_mut().zip(parsed.knots.as_chunks::<2>().0) {
        *d = i16::from_le_bytes(*s);
    }
    k
}

/// The per-servo switch's last step: one SAVE persists CONFIG, CALIB and
/// the LUT; the reboot loads it LIVE, knot for knot, with every reason
/// clear; FACTORY wipes it back to the virgin identity.
#[apply(matrix)]
fn lut_survives_save_and_reboot_until_factory(baud_idx: u8) {
    let flash = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, flash);
    go_live(&mut sim, s);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    assert_eq!(data_flags(&mut sim), 0);
    assert_eq!(lut_state(&mut sim), state::LIVE);
    let img = flash.lut_slot(Slot::B).expect("the second save lands in B");
    assert_eq!(LutImage::parse(&img).expect("parses").seq, 2);
    assert_eq!(knots_of(&img), MG90_A);

    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(lut_state(&mut rebooted), state::LIVE);
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
    assert!(flash.lut_slot(Slot::A).is_none() && flash.lut_slot(Slot::B).is_none());
    let mut fresh = support::sim(baud_idx);
    let s = fresh.add_servo_with_store(ID5, flash);
    assert_eq!(lut_state(&mut fresh), state::IDENTITY);
    assert_eq!(array(&fresh, s), ZERO);
    assert_eq!(data_flags(&mut fresh), FRESH);
}

/// A power cut after the CALIB image and before the LUT image, both ways
/// round: a stamp over the knots beside the old identity image, and a
/// stamp over the identity beside the old table. Either mix boots the
/// mismatch; a tear that changed nothing covered is harmless.
#[apply(matrix)]
fn torn_save_between_calib_and_lut_boots_stamp_mismatch(baud_idx: u8) {
    // the new stamp covers the knots, the old image is the identity
    let flash = RamStore::leak();
    let mut sim = sim(baud_idx);
    let s = mg90_servo(&mut sim, flash);
    go_live(&mut sim, s);
    flash.fail_after(ImageKind::Calib);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    assert!(
        flash.lut_slot(Slot::B).is_none(),
        "the lut image never landed"
    );
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(lut_state(&mut rebooted), state::IDENTITY);
    assert_eq!(array(&rebooted, s), ZERO);
    assert_eq!(data_flags(&mut rebooted), STAMP_MISMATCH);

    // the table SAVEd, then dropped to the identity and restamped: the new
    // stamp covers the identity, the old image still holds the table
    let flash = RamStore::leak();
    let mut sim = support::sim(baud_idx);
    let s = mg90_servo(&mut sim, flash);
    go_live(&mut sim, s);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    store_all(&mut sim, &ZERO);
    assert_eq!(commit(&mut sim), state::LIVE);
    stamp(&mut sim, s, None);
    assert_eq!(data_flags(&mut sim), 0);
    flash.fail_after(ImageKind::Calib);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(lut_state(&mut rebooted), state::LIVE, "the old table loads");
    assert_eq!(array(&rebooted, s), mg90_a());
    assert_eq!(data_flags(&mut rebooted), STAMP_MISMATCH);

    // nothing covered moved: the mix is the same set
    let flash = RamStore::leak();
    let mut sim = support::sim(baud_idx);
    let s = mg90_servo(&mut sim, flash);
    go_live(&mut sim, s);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
    flash.fail_after(ImageKind::Calib);
    assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Hardware);
    let mut rebooted = support::sim(baud_idx);
    let s = rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(lut_state(&mut rebooted), state::LIVE);
    assert_eq!(array(&rebooted, s), mg90_a());
    assert_eq!(data_flags(&mut rebooted), 0);
}

/// Flash rot or a table from another grid version: the image does not
/// load, the identity runs, and the stamp (which covered the knots) is
/// what says so - the LUT has no reason bit of its own. Losing an identity
/// image loses nothing.
#[apply(matrix)]
fn corrupt_or_stale_lut_image_boots_identity_under_the_stamp(baud_idx: u8) {
    let rots: [fn(&RamStore); 2] = [
        |f| f.corrupt_slot(ImageKind::Lut, Slot::A),
        |f| f.stale_slot(ImageKind::Lut, Slot::A),
    ];
    for rot in rots {
        let flash = RamStore::leak();
        let mut sim = sim(baud_idx);
        let s = mg90_servo(&mut sim, flash);
        go_live(&mut sim, s);
        assert_eq!(mgmt(&mut sim, MgmtOp::Save), ResultCode::Ok);
        // the newest image (the table) in B rots; the identity in A is
        // older and still sound
        rot(flash);
        flash.corrupt_slot(ImageKind::Lut, Slot::B);
        let mut rebooted = support::sim(baud_idx);
        let s = rebooted.add_servo_with_store(ID5, flash);
        assert_eq!(lut_state(&mut rebooted), state::IDENTITY);
        assert_eq!(array(&rebooted, s), ZERO);
        assert_eq!(data_flags(&mut rebooted), STAMP_MISMATCH);
        // the sound older slot alone loads the identity the same way
        flash.erase(ImageKind::Lut);
        rot(flash);
        let mut again = support::sim(baud_idx);
        let s = again.add_servo_with_store(ID5, flash);
        assert_eq!(lut_state(&mut again), state::IDENTITY);
        assert_eq!(array(&again, s), ZERO);
        assert_eq!(data_flags(&mut again), STAMP_MISMATCH);
    }

    let flash = RamStore::leak();
    let mut sim = sim(baud_idx);
    mg90_servo(&mut sim, flash);
    flash.corrupt_slot(ImageKind::Lut, Slot::A);
    let mut rebooted = support::sim(baud_idx);
    rebooted.add_servo_with_store(ID5, flash);
    assert_eq!(lut_state(&mut rebooted), state::IDENTITY);
    assert_eq!(
        data_flags(&mut rebooted),
        0,
        "a lost identity image is no loss"
    );
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
    assert_eq!(write(&mut sim, LUT_PAGE, &w), ResultCode::Validation);
    assert_eq!(
        write(&mut sim, LUT_CMD, &[cmd::MAX + 1]),
        ResultCode::Validation
    );
    assert_eq!(lut_state(&mut sim), state::IDENTITY);
    assert_eq!(array(&sim, s), ZERO);
    assert_eq!(
        write(&mut sim, LUT_STATE, &[state::LIVE]),
        ResultCode::Access
    );
    assert_eq!(lut_state(&mut sim), state::IDENTITY);
}
