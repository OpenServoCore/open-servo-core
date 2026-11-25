//! What the user types from the maker's sheet, once per servo type, and
//! the few answers about this unit no measurement gives.

use serde::Deserialize;
use serde::de::IgnoredAny;

#[derive(Clone, Debug, Default, Deserialize)]
pub struct Nameplate {
    pub name: String,
    #[serde(default)]
    pub document: Option<String>,
    pub gear_class: Option<GearClass>,
    /// Motor revolutions per output revolution, counted teeth.
    pub gear_ratio: Option<f64>,
    /// Whether the count rises as the horn turns counterclockwise, facing
    /// it: which stop is the counterclockwise end.
    pub count_rises_ccw: Option<bool>,
    /// The output's travel from stop to stop, deg, as the sheet states it or
    /// the user measured it.
    pub travel_deg: Option<f64>,
    pub operating_v: Option<VoltRange>,
    #[serde(default)]
    pub no_load: Vec<NoLoad>,
    #[serde(default)]
    pub stall: Vec<Stall>,
    /// The motor's brushes, counted on a teardown or stated by the maker.
    pub brushes: Option<u32>,
    /// The motor's commutator segments, the rotor's slots on these motors,
    /// counted on a teardown or stated by the maker.
    pub rotor_segments: Option<u32>,
    /// A field this tool no longer reads: its presence is refused, naming
    /// the two counts to enter instead.
    #[serde(default, rename = "ripple_per_rev")]
    retired_ripple_per_rev: Option<IgnoredAny>,
    /// The voltage the servo will run on: the stated row nearest it binds.
    pub run_v: Option<f64>,
    /// The share of the stated no-load speed the servo may ever turn at,
    /// `[numerator, denominator]`.
    pub derate_speed: Option<[u32; 2]>,
    /// The share any other value of the sheet is taken at.
    pub derate_other: Option<f64>,
}

/// The row the maker states at the stated voltage nearest the one the
/// servo runs on (`run_v`), the lower of two as near, of the rows inside
/// the operating range; with a single operating voltage, the row at it.
/// None when no one row binds.
pub fn stated<T: Copy>(
    op: VoltRange,
    run_v: Option<f64>,
    rows: &[T],
    volts_of: fn(&T) -> f64,
) -> Option<T> {
    let inside: Vec<T> = rows
        .iter()
        .copied()
        .filter(|r| (op.min..=op.max).contains(&volts_of(r)))
        .collect();
    let volts = if op.min == op.max {
        op.min
    } else {
        let run = run_v?;
        let near = |v: f64| ((v - run).abs() * 1e9).round_ties_even();
        inside
            .iter()
            .map(volts_of)
            .min_by(|a, b| near(*a).total_cmp(&near(*b)).then(a.total_cmp(b)))?
    };
    let mut at = inside.iter().filter(|r| volts_of(r) == volts);
    match (at.next(), at.next()) {
        (Some(row), None) => Some(*row),
        _ => None,
    }
}

impl Nameplate {
    /// The no-load row that binds ([`stated`]).
    pub fn speed_row(&self) -> Option<NoLoad> {
        stated(self.operating_v?, self.run_v, &self.no_load, |r| r.at_v)
    }

    /// The stall row that binds ([`stated`]).
    pub fn stall_row(&self) -> Option<Stall> {
        stated(self.operating_v?, self.run_v, &self.stall, |r| r.at_v)
    }

    /// Ripple events a motor revolution, the ruler the speed and the table
    /// are counted in: brushes times rotor segments (a two-brush,
    /// three-segment motor: 6). Refused with the line to show the user
    /// when either count is missing or 0.
    pub fn events_per_rev(&self) -> Result<u32, String> {
        match (self.brushes, self.rotor_segments) {
            (Some(b), Some(s)) if b > 0 && s > 0 => b.checked_mul(s).ok_or_else(|| {
                format!("the nameplate's brushes {b} times rotor_segments {s} is too large")
            }),
            _ => Err(COUNTS_MISSING.into()),
        }
    }

    /// A user's nameplate as the steps load it: every value they need is
    /// there, else refused naming the first missing one. A stated stall
    /// current is optional: the rules take stated V over the seated R.
    pub fn check_user(&self) -> Result<(), String> {
        let missing = |field: &str, what: &str| {
            Err(format!(
                "the nameplate has no usable {field} ({what}): enter it from the maker's sheet or your own measurement"
            ))
        };
        if self.retired_ripple_per_rev.is_some() {
            return Err(RETIRED_RIPPLE_PER_REV.into());
        }
        if self.gear_class.is_none() {
            return missing("gear_class", "plastic, metal or unknown");
        }
        if !self.gear_ratio.is_some_and(|r| r > 0.0) {
            return missing("gear_ratio", "motor revolutions per output revolution");
        }
        if !self.travel_deg.is_some_and(|t| t > 0.0 && t <= 360.0) {
            return missing("travel_deg", "the output's travel from stop to stop, deg");
        }
        if self.count_rises_ccw.is_none() {
            return missing(
                "count_rises_ccw",
                "whether the count rises as the horn turns counterclockwise, facing it",
            );
        }
        self.events_per_rev()?;
        let Some(op) = self.operating_v.filter(|o| o.min > 0.0 && o.min <= o.max) else {
            return missing("operating_v", "the operating voltage range, min and max");
        };
        if !self.run_v.is_some_and(|v| v > 0.0) {
            return missing("run_v", "the voltage the servo will run on");
        }
        if self
            .speed_row()
            .is_none_or(|r| !(r.seconds > 0.0 && r.per_deg > 0.0))
        {
            return missing(
                "no_load",
                &format!(
                    "a no-load speed row inside the operating range {} to {} V, nearest the {} V it runs on",
                    op.min,
                    op.max,
                    self.run_v.unwrap_or(0.0)
                ),
            );
        }
        if self.derate_speed.is_none_or(|[n, d]| n == 0 || n > d) {
            return missing(
                "derate_speed",
                "the share of the stated speed it may turn at, [numerator, denominator]",
            );
        }
        if !self.derate_other.is_some_and(|d| d > 0.0 && d <= 1.0) {
            return missing(
                "derate_other",
                "the share any other stated value is taken at",
            );
        }
        Ok(())
    }
}

pub const COUNTS_MISSING: &str = "the nameplate has no usable brushes and rotor_segments (the motor's brush count and its commutator segments, counted on a teardown or stated by the maker; their product is the ripple events a motor revolution): enter both";
pub const RETIRED_RIPPLE_PER_REV: &str = "the nameplate states ripple_per_rev, which this tool no longer reads: take it out and enter brushes and rotor_segments, the motor's brush count and its commutator segments (a two-brush, three-segment motor: 2 and 3)";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GearClass {
    Unknown,
    Plastic,
    Metal,
}

impl GearClass {
    /// The byte the firmware stores (client calib.rs GearClass).
    pub fn byte(self) -> u8 {
        match self {
            GearClass::Unknown => 0,
            GearClass::Plastic => 1,
            GearClass::Metal => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
pub struct VoltRange {
    pub min: f64,
    pub max: f64,
}

/// The sheet's no-load speed as makers state it: `seconds` per `per_deg`.
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct NoLoad {
    pub at_v: f64,
    pub seconds: f64,
    pub per_deg: f64,
}

/// A stated stall: the voltage, and the current where the sheet gives one.
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct Stall {
    pub at_v: f64,
    #[serde(default)]
    pub current_a: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bench() -> Nameplate {
        serde_json::from_str(include_str!("../testdata/mg90-d/nameplate.json")).expect("nameplate")
    }

    /// The bench nameplate passes; each required value taken out is refused
    /// naming it. The stall current is not one.
    #[test]
    fn a_nameplate_missing_a_required_field_is_refused_naming_it() {
        assert_eq!(bench().check_user(), Ok(()));
        type Take = fn(&mut Nameplate);
        let cases: [(&str, Take); 12] = [
            ("gear_class", |n| n.gear_class = None),
            ("gear_ratio", |n| n.gear_ratio = None),
            ("travel_deg", |n| n.travel_deg = None),
            ("count_rises_ccw", |n| n.count_rises_ccw = None),
            ("brushes and rotor_segments", |n| n.brushes = None),
            ("brushes and rotor_segments", |n| n.rotor_segments = None),
            ("brushes and rotor_segments", |n| n.brushes = Some(0)),
            ("operating_v", |n| n.operating_v = None),
            ("run_v", |n| n.run_v = None),
            ("no_load", |n| n.no_load.clear()),
            ("derate_speed", |n| n.derate_speed = None),
            ("derate_other", |n| n.derate_other = None),
        ];
        for (field, take) in cases {
            let mut n = bench();
            take(&mut n);
            let why = n.check_user().expect_err(field);
            assert!(
                why.starts_with(&format!("the nameplate has no usable {field} (")),
                "{field}: {why}"
            );
        }
        let mut n = bench();
        n.stall.clear();
        assert_eq!(n.check_user(), Ok(()), "a stall current is optional");
        let mut n = bench();
        n.no_load = vec![NoLoad {
            at_v: 9.0,
            seconds: 0.1,
            per_deg: 60.0,
        }];
        assert!(n.check_user().expect_err("outside").contains("no_load"));
    }

    /// The bench motor's two brushes and three segments: six events a
    /// revolution, the notebooks' `Motor.ripple_per_rev` for the mg90-d.
    #[test]
    fn the_events_a_revolution_are_brushes_times_rotor_segments() {
        let n = bench();
        assert_eq!((n.brushes, n.rotor_segments), (Some(2), Some(3)));
        assert_eq!(n.events_per_rev(), Ok(6));
        let mut n = bench();
        n.rotor_segments = Some(0);
        assert_eq!(n.events_per_rev(), Err(COUNTS_MISSING.into()));
    }

    /// A nameplate from before the two counts, stating ripple_per_rev, is
    /// refused saying what to enter instead, even with the counts beside it.
    #[test]
    fn a_nameplate_stating_ripple_per_rev_is_refused_naming_the_counts() {
        let mut v: serde_json::Value =
            serde_json::from_str(include_str!("../testdata/mg90-d/nameplate.json")).expect("json");
        v["ripple_per_rev"] = 6.into();
        let n: Nameplate = serde_json::from_value(v.clone()).expect("parses");
        assert_eq!(n.check_user(), Err(RETIRED_RIPPLE_PER_REV.into()));
        let o = v.as_object_mut().expect("object");
        o.remove("brushes");
        o.remove("rotor_segments");
        let n: Nameplate = serde_json::from_value(v).expect("parses");
        let why = n.check_user().expect_err("refused");
        assert!(
            why.contains("ripple_per_rev") && why.contains("brushes and rotor_segments"),
            "{why}"
        );
    }
}
