//! The table step's planner, pure: a free rung's length against its
//! runway and the burst, a stop rung's against the permit, the short-rung
//! cut, the fast passes' duty and bound, the rows a code, the stops step's
//! result it reads and the checks after a rung. The step on the simulated
//! servo is tested next to it (`steps::table`), on the crate's test kit.

use osc_measure::capture::rows::TelFrame;
use osc_measure::envelope::{BURST_MAX_S, DRIVE_DEADMAN_S, FREE_MARGIN, GUARD_SHARE, PERMIT_MAX_S};
use osc_measure::steps::runway::COAST_TAU_S;
use osc_measure::steps::seek::WINDOW_S;
use osc_measure::steps::table::{
    Band, FIRST_FAST_SHARE, FromStops, NEAR_STOP_COUNTS, RUNG_MIN_S, Reach, RungRead, Schedule,
    Speeds, after, another_trip, fast_bound_from, fast_predicted_cps, fast_q15, histogram_rows,
    plan_rung, plan_stop_rung, reach, rows_a_code, stamp_ticks, still_window_s, thin_carries,
};
use serde_json::json;

const FS: f64 = 20_000.0;

/// The bench pair's stops.
fn band() -> Band {
    Band::of_stops(110.0, 3730.0)
}

#[test]
fn the_guards_are_the_envelopes_and_a_fast_pass_starts_at_its_near_one() {
    let b = band();
    let g = GUARD_SHARE * 3620.0;
    assert_eq!((b.lo, b.hi), (110.0 + g, 3730.0 - g));
    assert_eq!((b.near(1), b.near(-1)), (b.lo, b.hi));
    assert_eq!((b.far(1), b.far(-1)), (b.hi, b.lo));
    assert_eq!(b.runway(1, 500.0), b.hi - 500.0);
    assert_eq!(b.runway(-1, 3000.0), 3000.0 - b.lo);
    assert!(b.holds(b.lo) && b.holds(b.hi) && !b.holds(b.hi + 0.5));
}

/// From every start and at every speed seen, both ways: the rung's whole
/// bound travel, its rows, the dead-man after them and the coast, ends at
/// or short of the far guard, and its rows and dead-man fit one burst.
#[test]
fn a_rung_is_never_planned_past_the_far_guard_at_its_bound_speed() {
    let b = band();
    for dir in [1i8, -1] {
        for k in 0..200 {
            let start = b.lo + (b.hi - b.lo) * k as f64 / 200.0;
            for fastest in [150.0, 400.0, 730.0, 1200.0, 3000.0, 9000.0] {
                for s in [1.0, 0.85, 0.7] {
                    let p = plan_rung(&b, dir, start, fastest, s, FS);
                    assert_eq!(p.bound_speed, FREE_MARGIN * fastest);
                    assert_eq!(p.bound_coast, p.bound_speed * COAST_TAU_S);
                    if !p.fits() {
                        continue;
                    }
                    let end = start + dir as f64 * p.travel_bound();
                    assert!(
                        dir as f64 * (b.far(dir) - end) >= -1e-9,
                        "{dir} from {start} at {fastest}: {}",
                        p.said()
                    );
                    assert!(p.t_s + DRIVE_DEADMAN_S <= BURST_MAX_S, "{}", p.said());
                    assert_eq!(p.rows as f64 / FS, p.t_s);
                }
            }
        }
    }
}

/// T is the lesser of the runway's and the burst's: on the bench's
/// first rung the burst binds, near the far guard the runway does.
#[test]
fn the_rung_is_the_lesser_of_the_runway_and_the_burst() {
    let b = band();
    let p = plan_rung(&b, 1, b.near(1), 730.0, 1.0, FS);
    let bs = FREE_MARGIN * 730.0;
    let runway = b.hi - b.near(1);
    assert!((p.t_runway_s - ((runway - bs * COAST_TAU_S) / bs - DRIVE_DEADMAN_S)).abs() < 1e-12);
    assert_eq!(p.t_burst_s, BURST_MAX_S - DRIVE_DEADMAN_S);
    assert!(p.t_runway_s > p.t_burst_s);
    assert!((p.t_s - p.t_burst_s).abs() <= 1.0 / FS);
    let p = plan_rung(&b, -1, b.lo + 1200.0, 730.0, 1.0, FS);
    assert!(p.t_runway_s < p.t_burst_s);
    assert!((p.t_s - p.t_runway_s).abs() <= 1.0 / FS, "{}", p.said());
}

/// The first fast pass runs at half its window.
#[test]
fn a_share_cuts_the_window() {
    let b = band();
    let full = plan_rung(&b, 1, b.near(1), 1400.0, 1.0, FS);
    let p = plan_rung(&b, 1, b.near(1), 1400.0, FIRST_FAST_SHARE, FS);
    let want = FIRST_FAST_SHARE * full.t_runway_s.min(full.t_burst_s);
    assert!((p.t_s - want).abs() <= 1.0 / FS, "{}", p.said());
    assert_eq!(p.share, FIRST_FAST_SHARE);
}

/// A rung under 150 ms is left out: at the start where T reaches 150 ms
/// it fits, a count nearer the far guard it does not; no speed seen
/// leaves nothing planned.
#[test]
fn a_rung_under_150_ms_is_left_out() {
    let b = band();
    let bs = FREE_MARGIN * 730.0;
    let runway = (RUNG_MIN_S + DRIVE_DEADMAN_S) * bs + bs * COAST_TAU_S;
    let start = b.hi - runway - 0.1;
    assert!(plan_rung(&b, 1, start, 730.0, 1.0, FS).fits());
    let p = plan_rung(&b, 1, start + 1.0, 730.0, 1.0, FS);
    assert!(!p.fits() && p.t_s < RUNG_MIN_S, "{}", p.said());
    assert!(!plan_rung(&b, 1, start, 730.0, 0.85, FS).fits());
    assert!(!plan_rung(&b, 1, b.hi + 10.0, 730.0, 1.0, FS).fits());
    assert!(!plan_rung(&b, 1, b.near(1), 0.0, 1.0, FS).fits());
}

fn stops(legs: serde_json::Value) -> serde_json::Value {
    json!({
        "verdict": "accepted",
        "stops": {"low": {"median_counts": 108.5}, "high": {"median_counts": 3729.25}, "legs": legs},
        "drives": {"approach_q15": 4000},
    })
}

/// The stops step's approach legs each way, their mean speeds the first
/// bound; a result with no approach one way, or not accepted, refuses.
#[test]
fn the_stops_result_gives_the_stops_the_duty_and_the_approach_speeds() {
    let legs = json!([
        {"n": 1, "kind": "approach", "toward": "low", "from_counts": 2000.0, "to_counts": 108.0, "secs": 2.0},
        {"n": 2, "kind": "leave", "toward": "high", "from_counts": 108.0, "to_counts": 260.0, "secs": 0.3},
        {"n": 3, "kind": "approach", "toward": "high", "from_counts": 260.0, "to_counts": 3730.0, "secs": 5.0},
    ]);
    let s = FromStops::of(&stops(legs)).expect("reads");
    assert_eq!((s.lo, s.hi, s.approach_q15), (108.5, 3729.25, 4000));
    assert_eq!(s.raw(), (109, 3729));
    assert_eq!(s.legs.len(), 2);
    assert_eq!((s.legs[0].dir, s.legs[0].speed_cps()), (-1, 946.0));
    assert_eq!((s.legs[1].dir, s.legs[1].speed_cps()), (1, 694.0));
    let mut f = Speeds::of_legs(&s.legs);
    assert_eq!((f.get(1), f.get(-1)), (694.0, 946.0));
    assert_eq!(f.from(1), "approach leg 3");
    f.feed(1, 650.0, "rung 2".into());
    f.feed(1, 720.0, "rung 3".into());
    assert_eq!((f.get(1), f.from(1)), (720.0, "rung 3"));
    assert_eq!(f.slowest(1), Some(&(650.0, "rung 2".to_string())));

    let one_way = json!([
        {"n": 1, "kind": "approach", "toward": "low", "from_counts": 2000.0, "to_counts": 108.0, "secs": 2.0},
    ]);
    let why = FromStops::of(&stops(one_way)).expect_err("one way").0;
    assert!(why.contains("no approach leg up"), "{why}");
    let mut refused = stops(json!([]));
    refused["verdict"] = json!("refused");
    let why = FromStops::of(&refused).expect_err("refused").0;
    assert!(why.contains("accepted stops step"), "{why}");
}

/// After a rung: one that did not turn, one past a guard and one faster
/// than its bound each end the step; one inside passes.
#[test]
fn the_checks_after_a_rung() {
    let b = band();
    let p = plan_rung(&b, 1, b.near(1), 730.0, 1.0, FS);
    let r = RungRead {
        start: p.start,
        end: 2500.0,
        drive_travel: 2200.0,
        travel: 2500.0 - p.start,
        speed_cps: 0.95 * p.bound_speed,
    };
    assert_eq!(after(&p, &r, &b, "rung 1"), Ok(()));
    let still = RungRead {
        drive_travel: 5.0,
        ..r
    };
    assert!(
        after(&p, &still, &b, "rung 1")
            .expect_err("still")
            .contains("did not turn")
    );
    let past = RungRead {
        end: b.hi + 1.0,
        ..r
    };
    let why = after(&p, &past, &b, "rung 1").expect_err("past");
    assert!(why.contains("outside the guards"), "{why}");
    let fast = RungRead {
        speed_cps: 1.01 * p.bound_speed,
        ..r
    };
    let why = after(&p, &fast, &b, "rung 1").expect_err("fast");
    assert!(why.contains("faster than its bound"), "{why}");
}

/// A rung's time on by the servo's stamps across their wrap, the host's
/// clock picking the lap: a 3.15 s rung held a further second reads
/// 4.15 s, not the 0.87 s the plain difference of u16 stamps gives.
#[test]
fn a_long_rungs_stamps_read_across_their_wrap() {
    let at = |ticks: u32| (1000u32 + ticks) as u16;
    assert_eq!(at(83_000).wrapping_sub(1000), 17_464);
    assert_eq!(stamp_ticks(1000, at(83_000), 82_000.0), 83_000.0);
    assert_eq!(stamp_ticks(1000, at(63_010), 63_500.0), 63_010.0);
    assert_eq!(
        stamp_ticks(60_000, 60_000u16.wrapping_add(500), 300.0),
        500.0
    );
    assert_eq!(stamp_ticks(5, 5, 200.0), 0.0);
}

/// A stop rung covers the counts to the stop at the slowest speed seen
/// over the free margin, spins up and reads a still window of whole polls,
/// inside the permit's time less the dead-man.
#[test]
fn a_stop_rung_reaches_the_stop_inside_the_permit() {
    let p = plan_stop_rung(1, 3500.0, 3600.0, 900.0, 800.0, FS);
    assert!(still_window_s() >= WINDOW_S && still_window_s() < WINDOW_S + 0.02);
    let want = 100.0 / (800.0 / FREE_MARGIN) + 0.1 + still_window_s();
    assert!((p.t_s - want).abs() <= 1.0 / FS, "{}", p.said());
    assert_eq!(
        (p.runway, p.far_guard, p.bound_speed),
        (100.0, 3600.0, FREE_MARGIN * 900.0)
    );
    let far = plan_stop_rung(-1, 2000.0, 100.0, 900.0, 800.0, FS);
    assert!((far.t_s - (PERMIT_MAX_S - DRIVE_DEADMAN_S)).abs() <= 1.0 / FS);
    assert!(far.t_s + DRIVE_DEADMAN_S <= PERMIT_MAX_S);
    // already on the stop: the spin-up and the still window alone
    let on = plan_stop_rung(-1, 100.0, 100.0, 900.0, 800.0, FS);
    assert!((on.t_s - (0.1 + still_window_s())).abs() <= 1.0 / FS);
}

fn rows(pos: &[f64]) -> Vec<TelFrame> {
    pos.iter()
        .enumerate()
        .map(|(k, p)| TelFrame {
            tick: k as u64,
            pos: Some(p.round() as u16),
            duty_counts: Some(146),
            ..TelFrame::default()
        })
        .collect()
}

/// The latest still window against the stop: on it within 10 counts,
/// short of it, past it, or still moving.
#[test]
fn a_stop_rung_reads_reached_short_past_or_moving() {
    let n = (still_window_s() * FS) as usize;
    let still = |at: f64| rows(&vec![at; n + 100]);
    assert_eq!(
        reach(&still(3605.0), 1, 3600.0, 3.0, FS),
        Reach::Reached(3605.0)
    );
    assert_eq!(
        reach(&still(3580.0), 1, 3600.0, 3.0, FS),
        Reach::Short(3580.0)
    );
    assert_eq!(
        reach(&still(3620.0), 1, 3600.0, 3.0, FS),
        Reach::Past(3620.0)
    );
    assert_eq!(
        reach(&still(120.0), -1, 100.0, 3.0, FS),
        Reach::Short(120.0)
    );
    assert_eq!(reach(&still(80.0), -1, 100.0, 3.0, FS), Reach::Past(80.0));
    let moving: Vec<f64> = (0..n + 100).map(|k| 3400.0 + k as f64 * 0.05).collect();
    assert_eq!(reach(&rows(&moving), 1, 3600.0, 3.0, FS), Reach::Moving);
    assert_eq!(
        reach(&still(3600.0)[..n - 1], 1, 3600.0, 3.0, FS),
        Reach::Moving
    );
}

/// The histogram drops rows within 32 counts of a stop; the rows a code
/// are the median of the rest over the codes between less stage 1's
/// 15-count margins.
#[test]
fn the_rows_a_code_leave_out_the_counts_by_the_stops() {
    let codes: Vec<u16> = (110..=3729).flat_map(|c| [c, c]).collect();
    let (inside, lo, hi) = histogram_rows(&codes, 110, 3729);
    assert_eq!(
        (lo, hi),
        (
            110 + NEAR_STOP_COUNTS as u16,
            3729 - NEAR_STOP_COUNTS as u16
        )
    );
    assert_eq!(inside.len(), 2 * (hi - lo + 1) as usize);
    assert_eq!(rows_a_code(&codes, 110, 3729), 2.0);
    assert_eq!(rows_a_code(&[], 110, 3729), 0.0);
    // a median, not a mean: a third of the codes read 10x leave it at 2
    let mut heavy = codes.clone();
    heavy.extend((500..1700).flat_map(|c| [c; 18]));
    assert_eq!(rows_a_code(&heavy, 110, 3729), 2.0);
}

/// The plan's slow round trips, then one more while a carry reads thin,
/// at most the extra allowed.
#[test]
fn a_thin_carry_adds_a_slow_round_trip_up_to_the_extra() {
    let s = Schedule::PLAN;
    assert!(another_trip(&s, 0, &[]) && another_trip(&s, 1, &[]));
    assert!(!another_trip(&s, 2, &[]));
    assert!(another_trip(&s, 2, &[2303]) && another_trip(&s, 3, &[2303]));
    assert!(!another_trip(&s, 4, &[2303]));
    // 120 rows a code everywhere but 60 over the 64 codes under carry 9
    let mut codes: Vec<u16> = (110..=3729).flat_map(|c| [c; 120]).collect();
    codes.retain(|&c| !(2495..2559).contains(&c));
    codes.extend((2495..2559).flat_map(|c| [c; 60]));
    let thin = thin_carries(&codes, 110, 3729);
    assert_eq!(thin, [2559]);
    assert!(rows_a_code(&codes, 110, 3729) >= 100.0);
}

/// One fifth of the rated speed by the nameplate's speed per volt: one
/// fifth of its stated voltage; the first fast pass is bounded by the
/// prediction, later ones by what the fast passes ran.
#[test]
fn the_fast_passes_run_at_a_fifth_of_the_stated_volts() {
    assert_eq!(fast_q15(4.8, 5.0), (0.96 / 5.0 * 32768.0f64).floor() as i16);
    assert_eq!(fast_predicted_cps(13_333.0), 0.2 * 13_333.0);
    assert_eq!(fast_bound_from([None, None], 1, 2667.0), 2667.0);
    assert_eq!(fast_bound_from([Some(2100.0), None], 1, 2667.0), 2100.0);
    assert_eq!(fast_bound_from([Some(2100.0), None], -1, 2667.0), 2667.0);
    assert_eq!(fast_bound_from([Some(3000.0), None], -1, 2667.0), 3000.0);
    assert_eq!(
        fast_bound_from([Some(2100.0), Some(1900.0)], -1, 2667.0),
        1900.0
    );
}

#[test]
fn the_plans_schedule() {
    assert_eq!(
        Schedule::PLAN,
        Schedule {
            slow_round_trips: 2,
            fast_round_trips: 2,
            extra_slow_max: 2
        }
    );
}

/// The bench's stop rungs (`testdata/table/stop-rungs.csv.gz`): pressed
/// on the low stop, and twice on the high stop where the pot reads about
/// +/-6 counts row to row; read at the poll spacing as the stops step
/// reads its seats, all three reach their stop. The same rows cut to the
/// last 0.15 s before the high stop, still climbing, read Moving.
#[test]
fn the_bench_stop_rungs_read_reached() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/testdata/table/stop-rungs.csv.gz"
    );
    let gz = std::fs::read(path).expect("fixture");
    let text =
        String::from_utf8(osc_measure::capture::meta::gunzip(&gz).expect("gzip")).expect("utf8");
    let mut lines = text.lines();
    assert_eq!(lines.next(), Some("capture,tick,pos"));
    let mut by: std::collections::BTreeMap<String, Vec<TelFrame>> = Default::default();
    for l in lines {
        let v: Vec<&str> = l.split(',').collect();
        by.entry(v[0].into()).or_default().push(TelFrame {
            tick: v[1].parse().expect("tick"),
            pos: Some(v[2].parse().expect("pos")),
            duty_counts: Some(152),
            ..TelFrame::default()
        });
    }
    for (k, dir, stop) in [("07", -1, 108.0), ("13", 1, 3726.0), ("15", 1, 3726.0)] {
        let r = reach(&by[k], dir, stop, 3.0, FS);
        assert!(
            matches!(r, Reach::Reached(m) if (m - stop).abs() <= 10.0),
            "{k}: {r:?}"
        );
    }
    let climb = &by["13"];
    let top = climb
        .iter()
        .position(|f| f.pos.is_some_and(|p| p >= 3700))
        .expect("reaches the top");
    assert_eq!(reach(&climb[..top], 1, 3726.0, 3.0, FS), Reach::Moving);
}
