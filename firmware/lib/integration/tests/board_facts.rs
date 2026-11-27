//! What osc-ident reads off a servo instead of a host board file: every
//! board fact its steps and rules take, published read-only in CALIB and
//! equal to the board file the image is built from (protocol sec 5.5).

use control_table::descriptor::FieldDesc;
use osc_integration::sim::{RamStore, Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::table::{STATUS_FLAG_CALIB_DEFAULTS, STATUS_FLAGS};
use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::persist::{CALIB_LEN, Slot};
use osc_servo_core::regions::calib::addr::motor::R_Q12;
use osc_servo_core::regions::{CALIB_BASE_ADDR, ControlTable};
use toml::de::{DeTable, DeValue};

const ID: u8 = 1;
/// The settle table's words in CALIB: 0 past the board's last band.
const SETTLE_WORDS: usize = 24;

fn board_file() -> String {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../boards/osc-dev-v006/board.toml"
    );
    std::fs::read_to_string(path).unwrap_or_else(|e| panic!("reading {path}: {e}"))
}

fn at<'a, 'i>(t: &'a DeTable<'i>, key: &str) -> &'a DeValue<'i> {
    let mut table = t;
    let mut keys = key.split('.').peekable();
    while let Some(k) = keys.next() {
        let v = table
            .get(k)
            .unwrap_or_else(|| panic!("board.toml states no {key}"))
            .get_ref();
        if keys.peek().is_none() {
            return v;
        }
        let DeValue::Table(next) = v else {
            panic!("board.toml: {key} is not a table path");
        };
        table = next;
    }
    unreachable!("a key has a last part")
}

fn word(v: &DeValue<'_>, key: &str) -> u16 {
    let DeValue::Integer(i) = v else {
        panic!("board.toml: {key} is no integer");
    };
    let n = i64::from_str_radix(&i.as_str().replace('_', ""), i.radix())
        .unwrap_or_else(|e| panic!("board.toml: {key}: {e}"));
    n as u16
}

/// The bytes the servo must publish for `field`, from the board file.
fn expected(t: &DeTable<'_>, field: &str) -> Vec<u8> {
    let u16le = |key: &str| word(at(t, key), key).to_le_bytes().to_vec();
    match field {
        "ntc_site" => match at(t, "calibration.ntc.site") {
            DeValue::String(s) if s == "Air" => vec![1],
            DeValue::String(s) if s == "Can" => vec![2],
            other => panic!("board.toml: calibration.ntc.site {other:?}"),
        },
        "bridge_min_ticks" => u16le("calibration.bridge_min_ticks"),
        "i_settle_start_ticks" => u16le("calibration.i_settle_gain.start_ticks"),
        "i_settle_q15" => {
            let key = "calibration.i_settle_gain.q15";
            let DeValue::Array(words) = at(t, key) else {
                panic!("board.toml: {key} is no array");
            };
            assert!(words.len() <= SETTLE_WORDS, "settle table over CALIB's");
            let mut b: Vec<u8> = words
                .iter()
                .flat_map(|w| word(w.get_ref(), key).to_le_bytes())
                .collect();
            b.resize(2 * SETTLE_WORDS, 0);
            b
        }
        "oc_trip_ma" => u16le("protection.oc_trip_ma"),
        "oc_trip_us" => u16le("protection.oc_trip_us"),
        "rail_source_mohm" => u16le("protection.rail_source_mohm"),
        k => u16le(&format!("characterization.{k}")),
    }
}

/// The CALIB fields osc-ident's steps and rules read, beside the sense
/// block it already reads: the image's constants with no other register
/// (the bridge's shortest pulse, the settle table), the protection
/// defaults the board seeds CONFIG from, where the NTC sits, and the
/// board's stage 1 characterization.
const FACTS: [&str; 15] = [
    "ntc_site",
    "bridge_min_ticks",
    "i_settle_start_ticks",
    "i_settle_q15",
    "oc_trip_ma",
    "oc_trip_us",
    "rail_source_mohm",
    "window_scale_q15",
    "terminal_offset_q4",
    "brake_loop_mohm",
    "crest_sample_ticks",
    "shunt_on_lag_q4",
    "tap_on_lag_q4",
    "off_lag_q4",
    "shunt_settle_q4",
];

fn read_at(sim: &mut Sim, addr: u16, width: u16, name: &str) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let n = width.to_le_bytes();
    sim.host_send(&instruction(ID, Opcode::Read, 0, &[a[0], a[1], n[0], n[1]]));
    let frames = sim.run();
    let replies: Vec<&WireFrame> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "{name}: one reply");
    assert_valid(replies[0]);
    let (inst, payload) = status(replies[0]);
    assert_eq!(inst.result(), Some(ResultCode::Ok), "{name}");
    payload.to_vec()
}

fn read(sim: &mut Sim, f: &FieldDesc) -> Vec<u8> {
    read_at(sim, f.addr, f.width, f.name)
}

/// Every fact read off `sim` against the board file.
fn assert_publishes_the_board_file(sim: &mut Sim) {
    let fields: Vec<(&str, Option<&FieldDesc>)> = FACTS
        .iter()
        .map(|&n| (n, ControlTable::FIELDS.iter().find(|d| d.name == n)))
        .collect();
    let missing: Vec<&str> = fields
        .iter()
        .filter(|(_, d)| d.is_none())
        .map(|(n, _)| *n)
        .collect();
    assert!(
        missing.is_empty(),
        "the table publishes no {missing:?}: osc-ident would need a host board file for them"
    );
    let text = board_file();
    let t = DeTable::parse(&text).expect("the board file parses");
    let t = t.get_ref();
    for (name, d) in fields {
        let d = d.expect("checked");
        assert!(!d.writable, "{name}: a board fact, read-only");
        assert_eq!(read(sim, d), expected(t, name), "{name}");
    }
}

#[test]
fn the_servo_publishes_every_board_fact_ident_reads() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(ID);
    assert_publishes_the_board_file(&mut sim);
}

/// The block sits in what was CALIB's reserved tail and no RW field moved,
/// so the image version stands: an image saved before the block (its tail
/// all zero) still loads, and the install stamp fills the block over it.
#[test]
fn a_calib_image_saved_before_the_board_facts_block_loads_with_them() {
    let mut calib = [0u8; CALIB_LEN];
    calib[(R_Q12 - CALIB_BASE_ADDR) as usize..][..2].copy_from_slice(&4016u16.to_le_bytes());
    let store = RamStore::leak();
    store.inject_calib(Slot::A, 1, &calib);
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo_with_store(ID, store);
    let flags = read_at(&mut sim, STATUS_FLAGS, 1, "status_flags")[0];
    assert_eq!(flags & STATUS_FLAG_CALIB_DEFAULTS, 0, "the image loaded");
    assert_eq!(read_at(&mut sim, R_Q12, 2, "r_q12"), 4016u16.to_le_bytes());
    assert_publishes_the_board_file(&mut sim);
}
