//! The tune's schedule: the speed gain each try installs, a fixed share
//! under the try before, from the rules' starting gain down; and what the
//! closed-loop check's verdict on a try says to do next. Only a set the
//! check refuses on its scores is tried again lower: a lower gain is no
//! answer to a fault, a loop that drove the shaft on its own, a bus error
//! or an interrupt. The sequence on the servo (rules, a volatile install,
//! the check, a try at a time) is the client's.

use serde_json::{Value, json};

use crate::scorecard as sc;
use crate::steps::{Check, Verdict};

/// Each try's gain over the try before's.
pub const SHARE: f64 = 0.8;
pub const TRIES: u32 = 6;
/// The checks a lower speed gain answers.
pub const SCORES: [&str; 4] = [sc::OVERSHOOT, sc::QUIET, sc::REST, sc::FOLLOW];

/// The gain of try `n`, from 0.
pub fn gain(start: f64, n: u32) -> f64 {
    start * SHARE.powi(n as i32)
}

pub fn schedule(start: f64, tries: u32) -> Vec<f64> {
    (0..tries).map(|n| gain(start, n)).collect()
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Next {
    /// The check accepted the set: it is the tune's.
    Accept,
    /// Refused on its scores alone: the next try, lower.
    Lower,
    /// Anything else ends the tune, said why.
    Stop(String),
}

/// The checks that failed, by name.
pub fn failed(checks: &[Check]) -> Vec<String> {
    checks
        .iter()
        .filter(|c| c.verdict == Verdict::Fail)
        .map(|c| c.name.clone())
        .collect()
}

/// What a try's check says to do next, from its verdict (the way it
/// ended, as its result names it), the checks it failed and why.
pub fn next(verdict: &str, failed_on: &[String], why: Option<&str>) -> Next {
    match verdict {
        "accepted" => Next::Accept,
        "refused"
            if !failed_on.is_empty() && failed_on.iter().all(|f| SCORES.contains(&f.as_str())) =>
        {
            Next::Lower
        }
        _ => Next::Stop(format!(
            "the check was {verdict}{}{}: a lower gain is no answer to that",
            if failed_on.is_empty() {
                String::new()
            } else {
                format!(", failing {}", failed_on.join(", "))
            },
            why.map_or(String::new(), |w| format!(" ({w})"))
        )),
    }
}

/// Whether a check of the final set, on the cruise's friction line, backs
/// the gain off to the tune's next try: refused on rest error alone. One
/// accepted check at a high gain does not predict the next one there (the
/// hold's thrown park lands a draw off the goal), so the final check goes
/// on down the tune's schedule; a refusal on any other line ends as the
/// tune's own refusals do.
pub fn backs_off(verdict: &str, failed_on: &[String]) -> bool {
    verdict == "refused" && failed_on.len() == 1 && failed_on[0] == sc::REST
}

fn largest(moves: &[Value], f: impl Fn(&Value) -> Option<f64>) -> Option<f64> {
    moves
        .iter()
        .filter_map(f)
        .filter(|v| v.is_finite())
        .fold(None, |w, v| Some(w.map_or(v, |w: f64| w.max(v))))
}

/// The worst of the check's scores over its moves: overshoot as a share
/// of its bound, the most driven hold's share, the largest rest error and
/// following error. Null where no move was scored.
pub fn worst(result: &Value) -> Value {
    let moves = result["moves"].as_array().map_or(&[][..], Vec::as_slice);
    let n = |m: &Value, k: &str| m.get(k).and_then(Value::as_f64);
    json!({
        "overshoot_share_of_bound": largest(moves, |m| Some(n(m, "overshoot_deg")? / n(m, "overshoot_bound_deg")?)),
        "tail_driven_share": largest(moves, |m| n(m, "tail_driven_share")),
        "rest_error_deg": largest(moves, |m| n(m, "rest_error_deg")),
        "follow_peak_counts": largest(moves, |m| n(m, "follow_peak_counts")),
        "screen_counts": result["bounds"]["screen_counts"],
        "moves_scored": moves.len(),
    })
}

/// [`worst`] in a line.
pub fn worst_said(w: &Value) -> String {
    let n = |k: &str| w[k].as_f64();
    let pct = |v: Option<f64>| v.map_or("none".into(), |v| format!("{:.0}%", 100.0 * v));
    if w["moves_scored"].as_u64().unwrap_or(0) == 0 {
        return "no move scored".into();
    }
    format!(
        "worst overshoot {} of its bound, most driven hold {}, rest error {} deg, following error {} counts (screen {})",
        pct(n("overshoot_share_of_bound")),
        pct(n("tail_driven_share")),
        n("rest_error_deg").map_or("none".into(), |v| format!("{v:.2}")),
        n("follow_peak_counts").map_or("none".into(), |v| format!("{v:.1}")),
        n("screen_counts").map_or("none".into(), |v| format!("{v:.0}"))
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_try_is_four_fifths_of_the_one_before() {
        let s = schedule(0.5, TRIES);
        assert_eq!(s.len(), 6);
        assert_eq!(s[0], 0.5);
        for (k, pair) in s.windows(2).enumerate() {
            assert!((pair[1] / pair[0] - SHARE).abs() < 1e-12, "{k}: {pair:?}");
        }
        assert!((s[3] - 0.5 * 0.512).abs() < 1e-12);
        assert!(schedule(0.5, 0).is_empty());
    }

    #[test]
    fn only_a_refusal_on_the_scores_alone_goes_lower() {
        let names = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(next("accepted", &[], None), Next::Accept);
        for f in SCORES {
            assert_eq!(next("refused", &names(&[f]), None), Next::Lower, "{f}");
        }
        assert_eq!(
            next(
                "refused",
                &names(&[sc::OVERSHOOT, sc::FOLLOW, sc::REST]),
                None
            ),
            Next::Lower
        );
        let stops = [
            ("refused", names(&[sc::NO_FAULT])),
            ("refused", names(&[sc::OVERSHOOT, sc::NO_FAULT])),
            ("refused", names(&[sc::FIRST_ENABLE])),
            ("refused", names(&[sc::SMALL_STEPS])),
            ("refused", names(&["torque off at the end"])),
            ("refused", vec![]),
            ("aborted", names(&[sc::REST])),
            ("aborted", vec![]),
            ("interrupted", vec![]),
        ];
        for (verdict, f) in stops {
            match next(verdict, &f, Some("why")) {
                Next::Stop(w) => assert!(w.contains(verdict) && w.contains("why"), "{w}"),
                other => panic!("{verdict} {f:?}: {other:?}"),
            }
        }
    }

    #[test]
    fn only_a_final_check_refused_on_rest_error_alone_backs_off() {
        let names = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert!(backs_off("refused", &names(&[sc::REST])));
        for f in [
            names(&[]),
            names(&[sc::OVERSHOOT]),
            names(&[sc::QUIET]),
            names(&[sc::FOLLOW]),
            names(&[sc::REST, sc::OVERSHOOT]),
            names(&[sc::REST, sc::NO_FAULT]),
            names(&[sc::FIRST_ENABLE]),
        ] {
            assert!(!backs_off("refused", &f), "{f:?}");
        }
        for verdict in ["accepted", "aborted", "interrupted"] {
            assert!(!backs_off(verdict, &names(&[sc::REST])), "{verdict}");
        }
    }

    #[test]
    fn the_worst_scores_are_over_every_move() {
        let r = json!({
            "bounds": {"screen_counts": 199.0},
            "moves": [
                {"overshoot_deg": 0.5, "overshoot_bound_deg": 1.0, "tail_driven_share": 0.01,
                 "rest_error_deg": 0.2, "follow_peak_counts": 30.0},
                {"overshoot_deg": 3.0, "overshoot_bound_deg": 2.0, "tail_driven_share": null,
                 "rest_error_deg": 0.1, "follow_peak_counts": 80.0},
            ]
        });
        let w = worst(&r);
        assert_eq!(w["overshoot_share_of_bound"], 1.5);
        assert_eq!(w["tail_driven_share"], 0.01);
        assert_eq!(w["rest_error_deg"], 0.2);
        assert_eq!(w["follow_peak_counts"], 80.0);
        assert_eq!(w["moves_scored"], 2);
        assert!(worst_said(&w).contains("150%"), "{}", worst_said(&w));
        assert_eq!(worst_said(&worst(&json!({}))), "no move scored");
    }
}
