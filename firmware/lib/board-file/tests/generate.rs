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
