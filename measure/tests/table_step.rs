//! The table step's planner, pure: a free rung's length against its
//! runway and the burst, a stop rung's against the permit, the short-rung
//! cut, the fast passes' duty and bound, the rows a code, the stops step's
//! result it reads and the checks after a rung. The step on the simulated
//! servo is tested next to it (`steps::table`), on the crate's test kit.

use osc_measure::capture::rows::TelFrame;
use osc_measure::envelope::{BURST_MAX_S, DRIVE_DEADMAN_S, FREE_MARGIN, GUARD_SHARE, PERMIT_MAX_S};
use osc_measure::fit::postable::{self, Seed};
use osc_measure::fit::stopcontact;
use osc_measure::steps::runway::COAST_TAU_S;
use osc_measure::steps::seek::WINDOW_S;
use osc_measure::steps::table::{
    self, Band, FIRST_FAST_SHARE, FromStops, HoleRead, NextStop, PassKind, RUNG_MIN_S, Reach,
    RungKind, RungRead, RungRows, RungSegment, STOP_RUNGS_MAX, Schedule, Speeds, after,
    after_stop_rung, another_trip, fast_bound_from, fast_predicted_cps, fast_q15, histogram_rows,
    hole_reads, pass_reach, plan_rung, plan_stop_rung, reach, rows_a_code, rung_holes, stamp_ticks,
    still_at, still_window_s, stop_q15, thin_carries,
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
        "drives": {"approach_q15": 4000, "step_q15": 1329, "ceiling_q15": 8046},
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
    assert_eq!(
        (s.lo, s.hi, s.approach_q15, s.step_q15, s.ceiling_q15),
        (108.5, 3729.25, 4000, 1329, 8046)
    );
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
    let mut no_step = stops(legs_both());
    no_step["drives"]
        .as_object_mut()
        .expect("drives")
        .remove("step_q15");
    let why = FromStops::of(&no_step).expect_err("no step").0;
    assert!(why.contains("drives.step_q15"), "{why}");
}

fn legs_both() -> serde_json::Value {
    json!([
        {"n": 1, "kind": "approach", "toward": "low", "from_counts": 2000.0, "to_counts": 108.0, "secs": 2.0},
        {"n": 3, "kind": "approach", "toward": "high", "from_counts": 260.0, "to_counts": 3730.0, "secs": 5.0},
    ])
}

/// The stop rungs' duty on the bench runs of record, Q15 from each stops
/// result: where the sign was proven low the approach sits at the 0.6 x
/// ceiling floor and the stop rungs run at 0.75 x the ceiling (0.697 V to
/// 0.871 V, 0.683 V to 0.852 V); where it was proven a step higher the
/// one-step cap holds (6066 + 1332 over 0.75 x 8585); never under the
/// approach, never over the ceiling.
#[test]
fn the_stop_rungs_drive_at_three_quarters_of_the_ceiling_at_most_a_step_over_the_approach() {
    assert_eq!(stop_q15(4827, 1329, 8046), 6034);
    assert_eq!(stop_q15(4726, 1329, 7865), 5898);
    assert_eq!(stop_q15(6066, 1332, 8585), 6438);
    assert_eq!(stop_q15(6066, 100, 8585), 6166);
    assert_eq!(stop_q15(7000, 1329, 8046), 7000);
    assert_eq!(stop_q15(8046, 1329, 8046), 8046);
}

/// After a rung: one that did not turn, one past a guard and one past
/// its priced travel inside the guards each end the step; one inside
/// passes, its pot speed not judged.
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
        speed_cps: 1.3 * p.bound_speed,
        ..r
    };
    assert_eq!(after(&p, &fast, &b, "rung 1"), Ok(()));
    let over = 1.01 * p.travel_bound();
    let far = RungRead {
        end: p.start + over,
        travel: over,
        ..r
    };
    assert!(b.holds(far.end), "{}", far.end);
    let why = after(&p, &far, &b, "rung 1").expect_err("far");
    assert!(why.contains("past its priced travel"), "{why}");
}

/// The bench servo's first free rung up after leaving a low stop read at
/// 4, stops 4 and 3730: the leave rung's pot slope 1084 counts/s, then
/// this rung from 2907 for 7493 rows read 1370 counts/s, 1.26x, its tail
/// over 3058..3369 where the pot reads 1.24x the counts a commutation
/// cycle of the middle, the rotor at 0.98x the leave rung's speed. It ran
/// 493 counts of the 711 it was priced at and rested at 3400, 218 inside
/// the far guard: accepted. Ended past that guard, it is refused.
#[test]
fn a_rung_read_fast_by_a_steep_pot_stretch_inside_its_price_passes() {
    let b = Band::of_stops(4.0, 3730.0);
    let p = plan_rung(&b, 1, 2907.0, 1084.38, 1.0, FS);
    assert!(p.rows.abs_diff(7493) <= 1, "{}", p.rows);
    assert!((p.travel_bound() - 711.0).abs() < 1.0, "{}", p.said());
    let r = RungRead {
        start: 2907.0,
        end: 3400.0,
        drive_travel: 462.0,
        travel: 493.0,
        speed_cps: 1369.58,
    };
    assert!(r.speed_cps > p.bound_speed, "{}", p.said());
    assert_eq!(after(&p, &r, &b, "rung 5"), Ok(()));
    let past = RungRead {
        end: b.hi + 1.0,
        travel: b.hi + 1.0 - 2907.0,
        ..r
    };
    let why = after(&p, &past, &b, "rung 5").expect_err("past");
    assert!(why.contains("outside the guards"), "{why}");
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

/// `rows` with the shunt at 1300 counts, stepping to 1300 + `rise` from
/// row 3000.
fn with_shunt(mut rows: Vec<TelFrame>, rise: u16) -> Vec<TelFrame> {
    for (k, f) in rows.iter_mut().enumerate() {
        f.current_raw = Some(if k < 3000 { 1300 } else { 1300 + rise });
    }
    rows
}

/// The latest still window, wherever the pot reads it: with the shunt
/// risen onto the stop reached, with no rise short, never still moving.
/// Nothing is judged against where the stops step read the stop.
#[test]
fn a_stop_rung_reads_reached_short_or_moving() {
    let n = 6000;
    let still = |at: f64, rise: u16| with_shunt(rows(&vec![at; n]), rise);
    for at in [3605.0, 3580.0, 120.0, 4.0] {
        assert_eq!(reach(&still(at, 76), 3.0, FS), Reach::Reached(at));
        assert_eq!(reach(&still(at, 20), 3.0, FS), Reach::Short(at));
    }
    let moving: Vec<f64> = (0..n).map(|k| 3400.0 + k as f64 * 0.05).collect();
    assert_eq!(
        reach(&with_shunt(rows(&moving), 76), 3.0, FS),
        Reach::Moving
    );
    let short = (still_window_s() * FS) as usize - 1;
    assert_eq!(reach(&still(3600.0, 76)[..short], 3.0, FS), Reach::Moving);
}

/// The histogram counts every code it is given over the range they read;
/// the rows a code are the median over the codes between less stage 1's
/// 15-count margins.
#[test]
fn the_rows_a_code_are_the_median_over_the_codes_read() {
    let codes: Vec<u16> = (110..=3729).flat_map(|c| [c, c]).collect();
    let (inside, lo, hi) = histogram_rows(&codes);
    assert_eq!((lo, hi), (110, 3729));
    assert_eq!(inside.len(), codes.len());
    assert_eq!(rows_a_code(&codes), 2.0);
    assert_eq!(rows_a_code(&[]), 0.0);
    // a median, not a mean: a third of the codes read 10x leave it at 2
    let mut heavy = codes.clone();
    heavy.extend((500..1700).flat_map(|c| [c; 18]));
    assert_eq!(rows_a_code(&heavy), 2.0);
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
    let thin = thin_carries(&codes);
    assert_eq!(thin, [2559]);
    assert!(rows_a_code(&codes) >= 100.0);
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
/// reads its seats, all three read still at their stop. The same rows cut
/// to the last 0.15 s before the high stop, still climbing, read no still
/// window.
#[test]
fn the_bench_stop_rungs_read_still() {
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
    for (k, stop) in [("07", 108.0), ("13", 3726.0), ("15", 3726.0)] {
        let r = still_at(&by[k], 3.0, FS);
        assert!(r.is_some_and(|m| (m - stop).abs() <= 10.0), "{k}: {r:?}");
    }
    let climb = &by["13"];
    let top = climb
        .iter()
        .position(|f| f.pos.is_some_and(|p| p >= 3700))
        .expect("reaches the top");
    assert_eq!(still_at(&climb[..top], 3.0, FS), None);
}

/// A stop rung that starts pressed, after one in its pass that rose onto
/// the stop and ended still moving on the pressed pot's noise, has no free
/// level to rise from: still, it is on the stop (the recorded run's rungs
/// 16 and 17, 21 and 22, 25 and 26). With no rise before it in the pass
/// it is short.
#[test]
fn a_stop_rung_pressed_from_its_start_is_on_the_stop_after_a_rise() {
    assert_eq!(pass_reach(Reach::Short(6.0), true), Reach::Reached(6.0));
    assert_eq!(pass_reach(Reach::Short(6.0), false), Reach::Short(6.0));
    assert_eq!(pass_reach(Reach::Moving, true), Reach::Moving);
    assert_eq!(pass_reach(Reach::Reached(4.0), false), Reach::Reached(4.0));
}

/// After a stop rung: still with the rise, done wherever the pot reads;
/// still with no rise, one more stop rung at the same duty even past the
/// count, then blocked; still moving, another up to the count, then
/// unreached.
#[test]
fn a_stop_rung_short_of_the_stop_takes_one_more() {
    let (on, short) = (Reach::Reached(4.0), Reach::Short(119.0));
    assert_eq!(after_stop_rung(&on, 1, 0), NextStop::Done);
    assert_eq!(after_stop_rung(&short, 1, 0), NextStop::Another);
    assert_eq!(
        after_stop_rung(&short, STOP_RUNGS_MAX, 0),
        NextStop::Another
    );
    assert_eq!(after_stop_rung(&short, 2, 1), NextStop::Blocked);
    assert_eq!(after_stop_rung(&on, STOP_RUNGS_MAX + 1, 1), NextStop::Done);
    assert_eq!(after_stop_rung(&Reach::Moving, 1, 0), NextStop::Another);
    assert_eq!(
        after_stop_rung(&Reach::Moving, STOP_RUNGS_MAX, 0),
        NextStop::Unreached
    );
}

/// A low-stop fixture's rows (`testdata/table/*-low-stop.csv.gz`) and its
/// reference reading (`low-stop.expected.json`).
fn low_stop(name: &str) -> (Vec<TelFrame>, serde_json::Value) {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/table/");
    let gz = std::fs::read(format!("{dir}{name}.csv.gz")).expect("fixture");
    let text =
        String::from_utf8(osc_measure::capture::meta::gunzip(&gz).expect("gzip")).expect("utf8");
    let mut lines = text.lines();
    assert_eq!(lines.next(), Some("tick,pos,duty_counts,current_raw"));
    let rows = lines
        .map(|l| {
            let v: Vec<&str> = l.split(',').collect();
            TelFrame {
                tick: v[0].parse().expect("tick"),
                pos: Some(v[1].parse().expect("pos")),
                duty_counts: Some(v[2].parse().expect("duty")),
                current_raw: Some(v[3].parse().expect("current")),
                ..TelFrame::default()
            }
        })
        .collect();
    let exp: serde_json::Value = serde_json::from_slice(
        &std::fs::read(format!("{dir}low-stop.expected.json")).expect("json"),
    )
    .expect("expected");
    (rows, exp[name].clone())
}

fn num(v: &serde_json::Value) -> f64 {
    v.as_f64().expect("number")
}

/// A stop fixture's counts a cycle from its run's table
/// (`testdata/table/sources.json`).
fn per_cycle(name: &str, key: &str) -> f64 {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/testdata/table/");
    let s: serde_json::Value =
        serde_json::from_slice(&std::fs::read(format!("{dir}sources.json")).expect("sources"))
            .expect("json");
    num(&s[format!("{name}.csv.gz")][key]["value"])
}

/// A stop fixture's drive rows, its pot and its shunt as numbers.
fn drive(name: &str) -> (Vec<TelFrame>, Vec<f64>) {
    let (rows, _) = low_stop(name);
    let (_, d) = table::drive_rows(&rows);
    let pos = d.iter().map(|f| f64::from(f.pos.expect("pos"))).collect();
    (d.to_vec(), pos)
}

/// One stop, two sittings (oscnb `stopcontact` on the same rows): the pot
/// rests 96 counts apart, the ripple puts first contact 0.6 cycles apart,
/// 48.93 and 49.53 cycles past the pot's 300 crossing (contact-analysis:
/// 49.3 +/- 1.28 between sittings). At the contact row the clean pot reads
/// 108 and the dead one 3. Judged against the other sitting's pot seat,
/// the stop would read moved or short.
#[test]
fn the_low_stop_is_where_the_ripple_meets_it_not_where_the_pot_rests() {
    let read = |name: &str| {
        let (d, pos) = drive(name);
        let per = per_cycle(name, "counts_per_cycle");
        let c = table::stop_contact(&d, -1, &|_, _| per, FS).expect("a contact");
        assert!(c.valid(), "{name}: rise {}", c.rise.counts());
        let past = stopcontact::cycles_past(&c, &pos, 300.0).expect("300");
        let rest = pos[pos.len() - 200..].iter().sum::<f64>() / 200.0;
        (c.rise.counts(), past, rest, pos[c.row])
    };
    let (clean_rise, clean, clean_rest, clean_pot) = read("clean-low-stop");
    let (dead_rise, dead, dead_rest, dead_pot) = read("dead-low-stop");
    assert!((clean_rise - 79.6).abs() <= 0.1 && (dead_rise - 75.4).abs() <= 0.1);
    assert_eq!(
        [(clean * 100.0).round(), (dead * 100.0).round()],
        [4893.0, 4953.0]
    );
    assert!((clean - dead).abs() <= 1.5 && (clean_rest - dead_rest).abs() > 90.0);
    assert_eq!((clean_pot, dead_pot), (108.0, 3.0));
}

/// The fast high-stop rung of a bench run: its pot runs about 1100
/// counts/s over the free stretch, so at its table's 4.31 counts a cycle
/// the line sits near 259 Hz, and the ripple counts 67.02 cycles from the
/// pot's 3480 to contact (a fixed 100..300 Hz band held a 131 Hz line on
/// these rows and counted 33.0). Through a table at the 4.03 counts a
/// cycle its placement samples read, the track holds; seeded at twice the
/// counts a cycle it runs on the 131 Hz line, reads twice the table's
/// counts a cycle and is named.
#[test]
fn a_fast_rung_is_counted_on_the_line_its_free_stretch_predicts() {
    let (d, pos) = drive("fast-high-stop");
    let per = per_cycle("fast-high-stop", "counts_per_cycle");
    let c = table::stop_contact(&d, 1, &|_, _| per, FS).expect("a contact");
    assert!(c.valid() && (c.f_seed - 259.4).abs() <= 0.5, "{}", c.f_seed);
    let past = stopcontact::cycles_past(&c, &pos, 3480.0).expect("3480");
    assert_eq!((past * 100.0).round(), 6702.0);
    let codes: Vec<u16> = d.iter().map(|f| f.pos.expect("pos")).collect();
    let table = per_cycle("fast-high-stop", "table_counts_per_cycle");
    let p = stopcontact::placed(&c, &codes, &|x| x / table, None, None).expect("placed");
    assert!(
        p.tracked() && (p.counts_per_cycle - 4.09).abs() <= 0.01,
        "{p:?}"
    );
    let wrong = table::stop_contact(&d, 1, &|_, _| 2.0 * per, FS).expect("a contact");
    assert!((wrong.f_seed - 131.2).abs() <= 0.5, "{}", wrong.f_seed);
    let off = stopcontact::placed(&wrong, &codes, &|x| x / table, None, None).expect("placed");
    assert!(
        !off.tracked() && (off.counts_per_cycle / table - 2.0).abs() <= 0.15,
        "{off:?}"
    );
}

/// A low-stop rung read for pot holes as the table step reads it: the
/// whole drive, no near-stop cut, on the line the table read was seeded on
/// (here the one the tool recorded; a constant 3.65 counts a cycle, the
/// fast rungs' about, seeds the same line). Every cycle is the reference's
/// (bringup `kb/scratch/pot-holes` `potholes.py`), and against the run's
/// typical and line the dead low end flags cycles 50 and 51, one hole.
#[test]
fn the_dead_low_end_flags_the_cycles_into_the_stop() {
    let (rows, e) = low_stop("dead-low-stop");
    let read = hole_read(&rows, &e);
    let want = e["cycles"].as_array().expect("cycles");
    assert_eq!(read.cycles.len(), want.len());
    for (c, w) in read.cycles.iter().zip(want) {
        assert_eq!(c.k as u64, w[0].as_u64().expect("k"));
        for (got, w) in [c.from, c.to, c.counts()]
            .into_iter()
            .zip(&w.as_array().expect("cycle")[1..])
        {
            assert!(
                (got - num(w)).abs() <= 6e-4,
                "cycle {}: {got} against {w}",
                c.k
            );
        }
    }
    let (m, line) = (num(&e["run_typical_this_pass"]), num(&e["run_line"]));
    assert!((m - 3.8557).abs() < 1e-4 && (line - 7.7261).abs() < 1e-4);
    let flagged: Vec<(usize, f64)> = read
        .cycles
        .iter()
        .filter(|c| c.counts() / m > line)
        .map(|c| (c.k, c.counts() / m))
        .collect();
    assert_eq!(flagged.len(), 2, "{flagged:?}");
    assert_eq!(flagged[0].0, 50);
    assert!((flagged[0].1 - 20.39).abs() < 0.005, "{flagged:?}");
    assert_eq!(flagged[1].0, 51);
    assert!((flagged[1].1 - 9.42).abs() < 0.005, "{flagged:?}");
    let holes = rung_holes(&read, m, line);
    assert_eq!(holes.len(), 1, "{holes:?}");
    let h = &holes[0];
    assert!(
        (h.from - 5.529).abs() < 1e-3 && (h.to - 120.473).abs() < 1e-3,
        "{h:?}"
    );
    assert_eq!(
        (h.cycles, &h.rungs[..], &h.dirs[..]),
        (2, &[3][..], &[-1][..])
    );
    assert!((h.r_max - 20.39).abs() < 0.005, "{h:?}");
}

/// The same rung on a clean pot, against its run's typical and line: no
/// cycle flags; the cycles pressed on the stop read under typical.
#[test]
fn a_clean_low_end_flags_nothing() {
    let (rows, e) = low_stop("clean-low-stop");
    let read = hole_read(&rows, &e);
    assert_eq!(
        read.cycles.len(),
        e["cycles"].as_array().expect("cycles").len()
    );
    let (m, line) = (num(&e["run_typical_this_pass"]), num(&e["run_line"]));
    assert!((m - 3.6434).abs() < 1e-4 && (line - 8.3988).abs() < 1e-4);
    assert_eq!(rung_holes(&read, m, line), []);
    assert!(
        read.cycles
            .iter()
            .filter(|c| c.lo() < 101.0)
            .all(|c| c.counts() < m)
    );
}

/// [`hole_reads`] over one rung the table read, with its line.
fn hole_read(rows: &[TelFrame], e: &serde_json::Value) -> HoleRead {
    let slots: [u8; 16] = e["slots"]
        .as_array()
        .expect("slots")
        .iter()
        .map(|v| v.as_u64().expect("slot") as u8)
        .collect::<Vec<_>>()
        .try_into()
        .expect("16 slots");
    let line = num(&e["tool_line_hz"]);
    let (pos, cur, duty): (Vec<u16>, Vec<u16>, Vec<i16>) = (
        rows.iter().map(|f| f.pos.expect("pos")).collect(),
        rows.iter()
            .map(|f| f.current_raw.expect("current"))
            .collect(),
        rows.iter().map(|f| f.duty_counts.expect("duty")).collect(),
    );
    let near = |_: f64| num(&e["const_seed_counts_a_cycle"]);
    let seeded = postable::segment_seeded(&pos, &cur, &duty, &slots, FS, Seed::Near(&near))
        .expect("segment");
    assert_eq!(seeded.f_seed, line);
    assert_eq!(seeded.f_seed, num(&e["const_seed_line_hz"]));
    let table = RungSegment {
        n: 3,
        pass: 1,
        pass_kind: PassKind::Slow,
        kind: RungKind::Stop,
        dir: -1,
        read: Ok(seeded),
        ratio: None,
        speed_cps: None,
    };
    let rung = RungRows {
        n: 3,
        pass: 1,
        pass_kind: PassKind::Slow,
        kind: RungKind::Stop,
        dir: -1,
        rows,
    };
    let (mut reads, not_read) = hole_reads(&[rung], &[table], &slots, FS);
    assert_eq!((reads.len(), not_read), (1, vec![]));
    let read = reads.remove(0);
    assert!(read.cycles.len() >= 3);
    read
}
