use std::path::Path;

use osc_board_file::{emit_board_config, generate};

const DEV: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../boards/osc-dev-v006/board.toml"
);

/// The dev board's literal, as rustc reads it in the app build: a change
/// here is a change to what every osc-dev-v006 image is built from.
#[test]
fn the_dev_v006_board_file_emits_its_literal() {
    let g = emit_board_config(Path::new(DEV)).expect("the dev board file is well formed");
    assert_eq!(g.config, include_str!("osc-dev-v006.config.txt"));
    assert_eq!(g.compat, "\"osc-dev-v006\"");
}

const MIN: &str = "[board]\ncompat = \"b\"\nhw_rev = 1\n";

fn refused(text: &str) -> String {
    generate(text, "board.toml")
        .expect_err("the generator refuses the file")
        .to_string()
}

#[test]
fn a_well_formed_file_emits_one_expression() {
    let g = generate(
        &format!("{MIN}[defaults]\nid = 0x10\nbaud = \"B500000\"\n"),
        "b.toml",
    )
    .unwrap();
    assert_eq!(
        g.config,
        "::osc_servo_ch32::prelude::BoardConfig {\n    \
         hw_rev: 1, // b.toml:3 board.hw_rev\n    \
         defaults: ::osc_servo_ch32::prelude::ConfigDefaults { // b.toml:4 defaults\n        \
         id: 16, // b.toml:5 defaults.id\n        \
         baud: ::osc_servo_ch32::prelude::BaudRate::B500000, // b.toml:6 defaults.baud\n        \
         response_deadline_us: ::osc_servo_ch32::prelude::DEFAULT_RESPONSE_DEADLINE_US,\n    \
         },\n    \
         model: ::osc_servo_ch32::prelude::MODEL_OSC_SERVO,\n}\n"
    );
    assert_eq!(g.compat, "\"b\"");
}

#[test]
fn a_syntax_error_names_its_line() {
    let e = refused(&format!("{MIN}[protection\n"));
    assert!(e.starts_with("board.toml:4:"), "{e}");
}

#[test]
fn a_float_is_refused_at_its_key() {
    assert_eq!(
        refused(&format!("{MIN}[calibration]\nvdd_mv = 3.3\n")),
        "board.toml:5:10: calibration.vdd_mv: integers only: the firmware's units are whole mA, mV, ohm, us, ticks"
    );
}

#[test]
fn a_string_is_refused_where_no_variant_is_named() {
    assert_eq!(
        refused(&format!("{MIN}[calibration]\nvdd_mv = \"3300\"\n")),
        "board.toml:5:10: calibration.vdd_mv: a string where the board config takes a number or a flag: strings only name a variant"
    );
}

#[test]
fn a_variant_must_be_an_identifier() {
    assert_eq!(
        refused(&format!("{MIN}[wiring.drv_en]\npin = \"P C3\"\n")),
        "board.toml:5:7: wiring.drv_en.pin: `P C3` is no variant name: write it as the firmware spells it"
    );
    assert_eq!(
        refused(&format!(
            "{MIN}[wiring.sensors]\nvmotor = [\"A5\", \"6\"]\n"
        )),
        "board.toml:5:17: wiring.sensors.vmotor: `6` is no variant name: write it as the firmware spells it"
    );
}

#[test]
fn an_array_is_refused_where_one_value_is_taken() {
    assert_eq!(
        refused(&format!("{MIN}[protection]\noc_trip_ma = [1, 2]\n")),
        "board.toml:5:14: protection.oc_trip_ma: an array where the board config takes one value"
    );
}

#[test]
fn the_pair_and_the_table_hold_their_own_kinds() {
    assert_eq!(
        refused(&format!("{MIN}[wiring.sensors]\nvmotor = [5, 6]\n")),
        "board.toml:5:11: wiring.sensors.vmotor: the pair names two variants, as strings"
    );
    assert_eq!(
        refused(&format!(
            "{MIN}[calibration.i_settle_gain]\nq15 = [1, \"2\"]\n"
        )),
        "board.toml:5:11: calibration.i_settle_gain.q15: the table holds integers only"
    );
}

#[test]
fn a_table_must_name_a_struct() {
    assert_eq!(
        refused(&format!("{MIN}[wiring.motor]\nin1 = 2\n")),
        "board.toml:4:1: wiring.motor: the board config holds no table by this name"
    );
    assert_eq!(
        refused(&format!("{MIN}[defaults]\nbaud = {{ rate = 1 }}\n")),
        "board.toml:5:8: defaults.baud: a table where the board config takes one value"
    );
}

#[test]
fn a_generator_owned_key_is_refused() {
    assert_eq!(
        refused(&format!("{MIN}[defaults]\nresponse_deadline_us = 100\n")),
        "board.toml:5:1: defaults.response_deadline_us: the generator writes this field from `DEFAULT_RESPONSE_DEADLINE_US`: leave it out"
    );
    assert_eq!(
        refused(&format!("{MIN}model = 7\n")),
        "board.toml:4:1: board.model: the generator writes this field from `MODEL_OSC_SERVO`: leave it out"
    );
}

#[test]
fn the_compatibility_key_is_required() {
    assert_eq!(
        refused("[board]\nhw_rev = 1\n"),
        "board.toml:1:1: board.compat: missing: the app and the bootloader take the tinyboot compatibility key from it"
    );
    assert_eq!(
        refused("[board]\ncompat = 1\n"),
        "board.toml:2:10: board.compat: the tinyboot compatibility key is a string"
    );
}

#[test]
fn a_missing_file_names_its_path() {
    let e = emit_board_config(Path::new("/nonexistent/board.toml"))
        .expect_err("no file")
        .to_string();
    assert!(
        e.starts_with("/nonexistent/board.toml: reading the board file:"),
        "{e}"
    );
}

/// The sim's dev-v006 board, as osc-integration includes it.
#[test]
fn the_dev_v006_board_file_emits_its_sim_mirror() {
    let m = osc_board_file::emit_sim_mirror(Path::new(DEV), "DEV_V006")
        .expect("the dev board file is well formed");
    assert_eq!(m, include_str!("osc-dev-v006.mirror.txt"));
}

#[test]
fn the_mirror_names_a_key_it_lacks() {
    let e = osc_board_file::sim_mirror(MIN, "board.toml", "B")
        .expect_err("the mirror reads the sense chain")
        .to_string();
    assert_eq!(
        e,
        "board.toml:1:1: calibration.shunt_r_mohm: missing: the sim's mirror of the board reads it"
    );
}

#[test]
fn the_mirror_refuses_what_the_image_refuses() {
    let dev = std::fs::read_to_string(DEV).unwrap();
    let bad = dev.replace("vdd_mv = 3300", "vdd_mv = 3.3");
    let e = osc_board_file::sim_mirror(&bad, "board.toml", "DEV_V006")
        .expect_err("a float is refused")
        .to_string();
    assert!(e.contains("calibration.vdd_mv: integers only"), "{e}");
}

#[test]
fn the_mirror_takes_the_ntc_site_as_a_variant() {
    let dev = std::fs::read_to_string(DEV).unwrap();
    let bad = dev.replace("site = \"Air\"", "site = 1");
    assert_ne!(bad, dev);
    let e = osc_board_file::sim_mirror(&bad, "board.toml", "DEV_V006")
        .expect_err("a number is no site")
        .to_string();
    assert!(
        e.ends_with("calibration.ntc.site: the sim's mirror takes a variant name here"),
        "{e}"
    );
}
