//! The refusals of a profile or a policy: what the run cannot do without
//! the value first, then the file and the field that lacks it.

use crate::nameplate::VoltRange;

fn no_usable(consequence: &str, file: &str, field: &str, what: &str) -> String {
    format!("{consequence}: the {file} has no usable {field} ({what})")
}

pub fn gear_class() -> String {
    no_usable(
        "Without the gear material the run cannot tell whether the gears may press the stops",
        "profile",
        "gear_class",
        "metal, plastic or unknown",
    )
}

pub fn gear_ratio() -> String {
    no_usable(
        "Without the gear ratio the run cannot turn motor turns into horn degrees",
        "profile",
        "gear_ratio",
        "motor revolutions per output revolution, counted teeth",
    )
}

pub fn rated_travel_deg() -> String {
    no_usable(
        "The run sets the soft limits to the rated travel and paces its first moves by it",
        "profile",
        "rated_travel_deg",
        "the output's travel as the maker rates it, deg, over 0 and at most 360",
    )
}

/// An output travel in degrees the tool takes: over 0 and at most a turn.
pub fn within_a_turn(deg: f64) -> bool {
    deg.is_finite() && deg > 0.0 && deg <= 360.0
}

pub const RETIRED_TRAVEL_DEG: &str = "the profile states travel_deg, the stop-to-stop travel, which this tool now measures: take it out and enter rated_travel_deg, the travel the maker's sheet states (180 for a servo sold as 180 deg)";

pub const RETIRED_SOFT_MARGIN_SHARE: &str = "the policy states motion.soft_margin_share, which this tool no longer reads: take it out, and state motion.soft_travel_deg, the soft window's width in output deg centered between the stops, only to narrow or widen it from the profile's rated travel";

pub const RETIRED_ANGLE_CW_STOP_DEG: &str = "the policy states motion.angle_cw_stop_deg, which this tool no longer reads: 0 deg is the servo's center of travel; to move it, state motion.center_offset_deg, the degrees counterclockwise of the center";

pub fn center_offset_deg(deg: f64) -> String {
    format!(
        "An offset that is no number cannot place 0 deg: the policy's motion.center_offset_deg is {deg}; state the degrees counterclockwise of the center of travel, or leave it out for the center"
    )
}

pub fn soft_travel_deg(deg: f64) -> String {
    format!(
        "A soft window of no width or of more than one turn cannot be centered between the stops: the policy's motion.soft_travel_deg is {deg}; state one over 0 and at most 360, or leave it out for the profile's rated travel"
    )
}

pub fn count_rises_ccw() -> String {
    no_usable(
        "Without it the run cannot tell which stop is which, and every angle would read mirrored",
        "profile",
        "count_rises_ccw",
        "whether the count rises as the horn turns counterclockwise, facing it",
    )
}

pub const COUNTS_MISSING: &str = "Without both counts the run cannot measure the angle table: the profile has no usable brushes and rotor_segments (the motor's brush count and its commutator segments, counted on a teardown or stated by the maker; their product is the ripple events a motor revolution): enter both";

pub const RETIRED_RIPPLE_PER_REV: &str = "the profile states ripple_per_rev, which this tool no longer reads: take it out and enter brushes and rotor_segments, the motor's brush count and its commutator segments (a two-brush, three-segment motor: 2 and 3)";

pub fn operating_v() -> String {
    no_usable(
        "The run needs the maker's range to choose which rating applies",
        "profile",
        "operating_v",
        "the operating voltage range, min and max",
    )
}

fn rows(what: &str, op: Option<VoltRange>, run_v: Option<f64>) -> String {
    match op {
        Some(op) => format!(
            "{what} inside the operating range {} to {} V, nearest the {} V it runs on",
            op.min,
            op.max,
            run_v.unwrap_or(0.0)
        ),
        None => format!("{what} inside the operating range, nearest the voltage it runs on"),
    }
}

pub fn no_load(op: Option<VoltRange>, run_v: Option<f64>) -> String {
    no_usable(
        "The speed ceiling that protects the brushes cannot be set",
        "profile",
        "no_load",
        &rows("a no-load speed row", op, run_v),
    )
}

pub fn stall(op: Option<VoltRange>, run_v: Option<f64>) -> String {
    no_usable(
        "The current limit cannot be set, and the servo would keep the limit it holds",
        "profile",
        "stall",
        &format!(
            "{}; its current is optional",
            rows("a stall row", op, run_v)
        ),
    )
}

pub fn run_v() -> String {
    no_usable(
        "The run needs the voltage the servo runs on to choose which of the maker's ratings applies",
        "policy",
        "protection.run_v",
        "the voltage the servo will run on, V; run_v in a version 0 nameplate",
    )
}

pub fn speed_ceiling() -> String {
    no_usable(
        "Without a speed ceiling nothing keeps the motor under the speed that wears its brushes out",
        "policy",
        "protection.speed_ceiling",
        "the share of the stated no-load speed it may turn at, [numerator, denominator], over 0 and at most 1; derate_speed in a version 0 nameplate",
    )
}

pub fn current_limit() -> String {
    no_usable(
        "Without a current limit a blocked horn draws up to the full stall current",
        "policy",
        "protection.current_limit",
        "the share of the stall current, the stated volts over R, over 0 and at most 1; derate_other in a version 0 nameplate",
    )
}

/// No heat model: the run cannot price a drive's heat or write the winding
/// estimate, so it is refused before the bus.
pub fn thermal() -> String {
    no_usable(
        "The run prices the heat of every drive and writes the winding temperature estimate from the motor's heat model",
        "profile",
        "thermal",
        "fast_tau_s, fast_gain_c_per_w, slow_tau_s and slow_gain_c_per_w: each node's time constant, s, and its rise over the board NTC per watt of copper loss, C/W",
    ) + ": enter all four"
}
