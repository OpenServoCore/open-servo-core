//! The osc-dev-v006 board file holds the values the sim mirrors as that
//! board's: the sense chain, the window floors, the settle table, the
//! protection defaults and the boot seed's comms and travel. The board
//! image is built from the same file, so the sim and the chip seed one
//! board.

use osc_integration::sim::{
    DEV_V006_BEMF_MIN_TICKS, DEV_V006_I_SETTLE_GAIN, DEV_V006_PROTECTION, DEV_V006_SENSE,
    DEV_V006_SENSE_EXT,
};
use osc_servo_core::regions::config::vmotor_counts;

const BOARD_FILE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../boards/osc-dev-v006/board.toml"
);

fn value<'a>(t: &'a toml::Table, path: &str) -> &'a toml::Value {
    let mut keys = path.split('.');
    let first = keys.next().unwrap_or_default();
    let mut v = t
        .get(first)
        .unwrap_or_else(|| panic!("{path}: no `{first}`"));
    for k in keys {
        v = v.get(k).unwrap_or_else(|| panic!("{path}: no `{k}`"));
    }
    v
}

fn int(t: &toml::Table, path: &str) -> i64 {
    value(t, path)
        .as_integer()
        .unwrap_or_else(|| panic!("{path}: not an integer"))
}

fn word<'a>(t: &'a toml::Table, path: &str) -> &'a str {
    value(t, path)
        .as_str()
        .unwrap_or_else(|| panic!("{path}: not a string"))
}

#[test]
fn the_dev_v006_board_file_holds_the_mirrored_values() {
    let text =
        std::fs::read_to_string(BOARD_FILE).unwrap_or_else(|e| panic!("reading {BOARD_FILE}: {e}"));
    let t: toml::Table = text.parse().expect("the board file is TOML");

    let s = &DEV_V006_SENSE;
    assert_eq!(int(&t, "calibration.shunt_r_mohm"), s.shunt_r_mohm as i64);
    assert_eq!(
        int(&t, "wiring.current_sense.gain_milli"),
        s.gain_milli as i64
    );
    assert_eq!(
        int(&t, "calibration.vmotor_divider.top_ohm"),
        s.vmotor_div_top as i64
    );
    assert_eq!(
        int(&t, "calibration.vmotor_divider.bot_ohm"),
        s.vmotor_div_bot as i64
    );
    assert_eq!(int(&t, "calibration.vdd_mv"), s.vdd_mv as i64);
    assert_eq!(
        int(&t, "calibration.i_window_min_ticks"),
        s.i_window_min_ticks as i64
    );
    assert_eq!(
        int(&t, "calibration.v_window_min_ticks"),
        s.v_window_min_ticks as i64
    );

    let x = &DEV_V006_SENSE_EXT;
    assert_eq!(
        int(&t, "calibration.vbus_divider.top_ohm"),
        x.vbus_div_top_ohm as i64
    );
    assert_eq!(
        int(&t, "calibration.vbus_divider.bot_ohm"),
        x.vbus_div_bot_ohm as i64
    );
    assert_eq!(
        int(&t, "calibration.ntc.pullup_ohm"),
        x.ntc_pullup_ohm as i64
    );
    assert_eq!(int(&t, "calibration.ntc.r25_ohm"), x.ntc_r25_ohm as i64);
    assert_eq!(int(&t, "calibration.ntc.beta"), x.ntc_beta as i64);
    assert_eq!(int(&t, "calibration.rail_drop_mv"), x.rail_drop_mv as i64);
    assert_eq!(
        int(&t, "calibration.vmotor_bias.top_ohm"),
        x.vmotor_bias_top_ohm as i64
    );
    assert_eq!(
        int(&t, "calibration.vmotor_bias.bot_ohm"),
        x.vmotor_bias_bot_ohm as i64
    );
    let vdd = int(&t, "calibration.vdd_mv") as u16;
    let bias = vmotor_counts(
        vdd,
        int(&t, "calibration.vmotor_bias.top_ohm") as u32,
        int(&t, "calibration.vmotor_bias.bot_ohm") as u32,
        vdd,
    );
    assert_eq!(
        bias, x.vmotor_bias_nom_counts,
        "the nominal bias follows the file"
    );

    assert_eq!(
        int(&t, "calibration.bemf_window_min_ticks"),
        DEV_V006_BEMF_MIN_TICKS as i64
    );
    let g = &DEV_V006_I_SETTLE_GAIN;
    assert_eq!(
        int(&t, "calibration.i_settle_gain.start_ticks"),
        g.start_ticks as i64
    );
    let q15: Vec<i64> = value(&t, "calibration.i_settle_gain.q15")
        .as_array()
        .expect("q15 is an array")
        .iter()
        .map(|v| v.as_integer().expect("q15 words are integers"))
        .collect();
    let mirror: Vec<i64> = g.q15.iter().map(|&w| w as i64).collect();
    assert_eq!(q15, mirror);

    let p = &DEV_V006_PROTECTION;
    assert_eq!(int(&t, "protection.i_cont_ma"), p.i_cont_ma as i64);
    assert_eq!(
        int(&t, "protection.stall_release_ma"),
        p.stall_release_ma as i64
    );
    assert_eq!(int(&t, "protection.oc_trip_ma"), p.oc_trip_ma as i64);
    assert_eq!(int(&t, "protection.oc_trip_us"), p.oc_trip_us as i64);
    assert_eq!(int(&t, "protection.thermal_tau_s"), p.thermal_tau_s as i64);
    assert_eq!(int(&t, "protection.cutoff_cc"), p.cutoff_cc as i64);
    assert_eq!(
        int(&t, "protection.v_undervolt_mv"),
        p.v_undervolt_mv as i64
    );
    assert_eq!(
        int(&t, "protection.rail_source_mohm"),
        p.rail_source_mohm as i64
    );
    assert_eq!(
        value(&t, "protection.drive_polarity").as_bool(),
        Some(p.drive_polarity)
    );

    // The boot seed `class_defaults` pins (`the_dev_v006_boot_seed_is_pinned`).
    assert_eq!(int(&t, "defaults.id"), 1);
    assert_eq!(word(&t, "defaults.baud"), "B1000000");
    assert_eq!(int(&t, "defaults.pos_min_phys_counts"), 0);
    assert_eq!(int(&t, "defaults.pos_max_phys_counts"), 4095);
}
