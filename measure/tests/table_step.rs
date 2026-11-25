//! The table step's planner, pure: a rung's length against its runway and
//! the burst, the stagger, the short-rung cut, the stops step's result it
//! reads and the checks after a rung. The step on the simulated servo is
//! tested next to it (`steps::table`), on the crate's test kit.

use osc_measure::envelope::{BURST_MAX_S, DRIVE_DEADMAN_S, FREE_MARGIN, GUARD_SHARE};
use osc_measure::steps::runway::COAST_TAU_S;
use osc_measure::steps::table::{
    self, Band, FromStops, RUNG_MIN_S, RungRead, STAGGER, START_INSIDE_COUNTS, after, plan_rung,
    share, stamp_ticks,
};
use serde_json::json;

const FS: f64 = 20_000.0;

/// The bench pair's stops.
fn band() -> Band {
    Band::of_stops(110.0, 3730.0)
}

#[test]
fn the_guards_are_the_envelopes_and_a_pass_starts_inside_its_near_one() {
    let b = band();
    let g = GUARD_SHARE * 3620.0;
    assert_eq!((b.lo, b.hi), (110.0 + g, 3730.0 - g));
    assert_eq!(b.start(1), b.lo + START_INSIDE_COUNTS);
    assert_eq!(b.start(-1), b.hi - START_INSIDE_COUNTS);
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
    let p = plan_rung(&b, 1, b.start(1), 730.0, 1.0, FS);
    let bs = FREE_MARGIN * 730.0;
    let runway = b.hi - b.start(1);
    assert!((p.t_runway_s - ((runway - bs * COAST_TAU_S) / bs - DRIVE_DEADMAN_S)).abs() < 1e-12);
    assert_eq!(p.t_burst_s, BURST_MAX_S - DRIVE_DEADMAN_S);
    assert!(p.t_runway_s > p.t_burst_s);
    assert!((p.t_s - p.t_burst_s).abs() <= 1.0 / FS);
    let p = plan_rung(&b, -1, b.lo + 1200.0, 730.0, 1.0, FS);
    assert!(p.t_runway_s < p.t_burst_s);
    assert!((p.t_s - p.t_runway_s).abs() <= 1.0 / FS, "{}", p.said());
}

#[test]
fn the_first_rung_of_each_round_trip_is_staggered() {
    assert_eq!(STAGGER, [1.0, 0.85, 0.7]);
    for (trip, s) in STAGGER.iter().enumerate() {
        assert_eq!(share(trip, 0), *s);
        assert_eq!(share(trip, 1), 1.0);
        assert_eq!(share(trip, 2), 1.0);
    }
    let b = band();
    let full = plan_rung(&b, 1, b.start(1), 1400.0, 1.0, FS);
    for s in STAGGER {
        let p = plan_rung(&b, 1, b.start(1), 1400.0, s, FS);
        let want = s * full.t_runway_s.min(full.t_burst_s);
        assert!((p.t_s - want).abs() <= 1.0 / FS, "{s}: {}", p.said());
        assert_eq!(p.share, s);
    }
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
    assert!(!plan_rung(&b, 1, b.start(1), 0.0, 1.0, FS).fits());
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
    let f = table::Fastest::of_legs(&s.legs);
    assert_eq!((f.get(1), f.get(-1)), (694.0, 946.0));
    assert_eq!(f.from(1), "approach leg 3");

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
    let p = plan_rung(&b, 1, b.start(1), 730.0, 1.0, FS);
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
