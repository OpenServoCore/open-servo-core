//! What two protection screens of a servo must clear, read off its rows:
//! the largest speed the observer reads on a still shaft, which the stall
//! speed sits over, and the worst run of pot steps the sensor screen
//! tests, which the pot jump sits over (notebooks `oscnb/screens.py`).
//!
//! The stall detector counts a press only while the observer's speed stays
//! under the stall speed on every MEDIUM period, so the bound has to sit
//! over the speed the observer reads with the loop driving a shaft that
//! does not move: [`still_speed_cps`], the largest |omega_hat| over the
//! rows of the still holds. The rows carry `omega_hat_cps` as the servo
//! streams it, raw csQ16. A wide-code hold is not a still hold: the pot
//! flips across the skipped span there, and the observer's speed with it.
//! Each hold is read from [`STILL_SKIP_TICKS`] past its first row: a burst
//! of rows that starts on a still shaft disturbs the pot over its first
//! rows, and the observer reads 100 to 200 count/s there with no current.
//!
//! The sensor screen takes the raw pot's step from one RAIL period to the
//! next, once a MEDIUM period, and faults when a number of steps in a row
//! each pass its bound. What a healthy pot must clear is therefore the
//! level that `run` steps in a row all pass: the worst over the rows of the
//! least of `run` successive steps ([`step_run`]). The rows are FAST
//! ticks, so the RAIL samples of every phase of the period are scanned,
//! and a run never spans a hole in the ticks. A RAIL sample inside one of
//! the run's pot holes ([`Holes`]) is left out, and no run of its phase
//! spans it, since the bound is what a healthy pot must clear.

use serde_json::Value;

use crate::capture::rows::TelFrame;
use crate::rules::units::FAST_TICKS_PER_MEDIUM;

/// The steps in a row the sensor screen's statistic reads: the screen's
/// own count.
pub const SCREEN_RUN: usize = 4;

const DECIM_MED: usize = FAST_TICKS_PER_MEDIUM as usize;
/// The raw csQ16 of `omega_hat_cps` over count/s.
const Q16: f64 = 65536.0;
const FAST_HZ: u64 = 20_000;
/// The opening of a still hold the still speed skips, ms, and in FAST
/// ticks.
pub const STILL_SKIP_MS: u64 = 10;
pub const STILL_SKIP_TICKS: u64 = STILL_SKIP_MS * FAST_HZ / 1000;

/// The largest |omega_hat| over the rows of the still holds `holds`,
/// count/s, each hold read from [`STILL_SKIP_TICKS`] past its first row;
/// none where no row read carries a speed.
pub fn still_speed_cps(holds: &[&[TelFrame]]) -> Option<f64> {
    holds
        .iter()
        .flat_map(|h| {
            let first = h.iter().map(|f| f.tick).min().unwrap_or(0);
            h.iter()
                .filter(move |f| f.tick - first >= STILL_SKIP_TICKS)
                .filter_map(|f| f.omega_hat_cps)
        })
        .map(|w| (w as i64).unsigned_abs())
        .max()
        .map(|w| w as f64 * (1.0 / Q16))
}

/// The run's pot holes: raw counts, each from its lower end to its upper
/// one, both inside, and where they were read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Holes {
    pub ranges: Vec<(f64, f64)>,
    pub source: String,
}

impl Holes {
    /// Refused where a hole runs from its upper end down.
    pub fn new(ranges: Vec<(f64, f64)>, source: String) -> Result<Self, String> {
        let wrong: Vec<String> = ranges
            .iter()
            .filter(|(lo, hi)| lo > hi)
            .map(|(lo, hi)| format!("{lo}..{hi}"))
            .collect();
        if !wrong.is_empty() {
            return Err(format!(
                "a pot hole runs from its lower end to its upper one, and {} does not",
                wrong.join(", ")
            ));
        }
        Ok(Holes { ranges, source })
    }

    /// No hole, and why.
    pub fn none(why: impl Into<String>) -> Self {
        Holes {
            ranges: Vec::new(),
            source: why.into(),
        }
    }

    /// The pot holes of a table step's result `table`, kept at `file`:
    /// `holes.pot.holes[]` `from_counts` and `to_counts`; none, said, for a
    /// result written before the table step judged holes.
    pub fn of_table(table: &Value, file: &str) -> Result<Self, String> {
        let Some(list) = table.pointer("/holes/pot/holes") else {
            return Ok(Holes::none(format!(
                "{file} holds no pot holes block, a table step from before the holes: no pot hole left out"
            )));
        };
        let list = list
            .as_array()
            .ok_or_else(|| format!("{file} holes.pot.holes is no list"))?;
        let mut ranges = Vec::new();
        for (k, h) in list.iter().enumerate() {
            let end = |key: &str| {
                h[key]
                    .as_f64()
                    .ok_or_else(|| format!("{file} holes.pot.holes[{k}] holds no {key}"))
            };
            ranges.push((end("from_counts")?, end("to_counts")?));
        }
        let n = ranges.len();
        Holes::new(
            ranges,
            format!(
                "the {n} pot hole{} of {file}",
                if n == 1 { "" } else { "s" }
            ),
        )
        .map_err(|e| format!("{file}: {e}"))
    }

    fn inside(&self, raw: u16) -> bool {
        let r = raw as f64;
        self.ranges.iter().any(|&(lo, hi)| r >= lo && r <= hi)
    }
}

/// The worst level `run` successive steps all pass: the most of the least
/// over each window of `run`; none for fewer steps.
fn worst_run(steps: &[u32], run: usize) -> Option<u32> {
    steps
        .windows(run)
        .map(|w| w.iter().copied().min().unwrap_or(0))
        .max()
}

/// The worst level `run` successive RAIL-period pot steps all pass, raw
/// counts, over every RAIL phase of every stretch of successive ticks of
/// every burst `segments` holds, the samples inside a hole of `holes` left
/// out: each cuts the runs of its own phase alone. None where no stretch
/// holds `run` steps.
pub fn step_run(segments: &[&[TelFrame]], run: usize, holes: &Holes) -> Option<u32> {
    let mut worst = None;
    let mut piece: Vec<u16> = Vec::new();
    let mut close = |piece: &mut Vec<u16>| {
        let steps: Vec<u32> = piece
            .windows(2)
            .map(|w| w[0].abs_diff(w[1]) as u32)
            .collect();
        if let Some(level) = worst_run(&steps, run) {
            worst = worst.max(Some(level));
        }
        piece.clear();
    };
    for seg in segments {
        let rows: Vec<(u64, u16)> = seg.iter().filter_map(|f| Some((f.tick, f.pos?))).collect();
        for stretch in rows.chunk_by(|a, b| b.0 == a.0 + 1) {
            for phase in 0..DECIM_MED {
                for &(_, p) in stretch.iter().skip(phase).step_by(DECIM_MED) {
                    if holes.inside(p) {
                        close(&mut piece);
                    } else {
                        piece.push(p);
                    }
                }
                close(&mut piece);
            }
        }
    }
    worst
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rows(pos: &[u16], from: u64) -> Vec<TelFrame> {
        pos.iter()
            .enumerate()
            .map(|(k, &p)| TelFrame {
                tick: from + k as u64,
                pos: Some(p),
                ..TelFrame::default()
            })
            .collect()
    }

    fn holes(r: &[(f64, f64)]) -> Holes {
        Holes::new(r.to_vec(), "the test's".into()).expect("holes")
    }

    fn none() -> Holes {
        Holes::none("none")
    }

    /// A quiet pot at 2000, and at RAIL phase 3 the samples of five periods
    /// in a row read 2009, 1997, 2004, 1994 and 2024: steps of 9, 12, 7, 10,
    /// 30 and, back to 2000, 24.
    #[test]
    fn the_step_run_is_the_worst_level_successive_rail_period_steps_all_pass() {
        let mut pos = vec![2000u16; 200];
        for (k, level) in [2009u16, 1997, 2004, 1994, 2024].into_iter().enumerate() {
            pos[3 + 10 * (5 + k)] = level;
        }
        let r = rows(&pos, 0);
        assert_eq!(step_run(&[&r], 1, &none()), Some(30));
        assert_eq!(step_run(&[&r], 2, &none()), Some(24));
        assert_eq!(step_run(&[&r], SCREEN_RUN, &none()), Some(7));
    }

    /// A hole of one tick in the fourth period leaves three steps of 20 on
    /// each side: no run of four passes 20, and the quiet phases read 0; the
    /// same rows cut into two bursts at the hole read the same.
    #[test]
    fn a_run_never_spans_a_hole_in_the_ticks_or_two_segments() {
        let mut pos = vec![2000u16; 80];
        for k in 0..8 {
            pos[10 * k] = 2000 + 20 * k as u16;
        }
        let whole = rows(&pos, 0);
        assert_eq!(step_run(&[&whole], 4, &none()), Some(20));
        let holed: Vec<TelFrame> = whole.iter().filter(|f| f.tick != 35).cloned().collect();
        assert_eq!(
            (
                step_run(&[&holed], 4, &none()),
                step_run(&[&holed], 3, &none())
            ),
            (Some(0), Some(20))
        );
        let (a, b) = whole.split_at(35);
        assert_eq!(
            (step_run(&[a, b], 4, &none()), step_run(&[a, b], 3, &none())),
            (Some(0), Some(20))
        );
    }

    #[test]
    fn too_few_steps_read_nothing() {
        assert_eq!(step_run(&[&rows(&[2000; 30], 0)], 4, &none()), None);
        assert_eq!(step_run(&[&rows(&[2000; 41], 0)], 4, &none()), Some(0));
    }

    /// A spin, 7 counts a RAIL period on every phase, and the dead low end
    /// of a worn track: the reading glides from 129 to the floor at 4 by 21
    /// or 22 counts a RAIL period, then rests there.
    #[test]
    fn the_rows_inside_a_pot_hole_are_left_out() {
        let spin: Vec<u16> = (0..300u16).map(|k| 371 + k * 7 / 10).collect();
        let glide: Vec<u16> = (0..300i32)
            .map(|k| (129 - k * 215 / 100).max(4) as u16)
            .collect();
        let (a, b) = (rows(&spin, 0), rows(&glide, 300));
        assert_eq!(step_run(&[&a, &b], 4, &none()), Some(21));
        // with the hole of the dead low end, raw 4 to 121, the glide's
        // samples are left out and the spin reads
        let low = holes(&[(4.0, 121.0)]);
        assert_eq!(step_run(&[&a, &b], 4, &low), Some(7));
        // the glide alone: of each phase at most its first sample lies over
        // the hole, and no step is left
        assert_eq!(step_run(&[&b], 4, &low), None);
        // a hole away from the rows leaves them as they read
        assert_eq!(
            step_run(&[&a, &b], 4, &holes(&[(2500.0, 2600.0)])),
            Some(21)
        );
    }

    /// Steps of 20 on RAIL phase 0 from 2000, eight samples, the other
    /// phases quiet at 2500.
    #[test]
    fn a_sample_in_a_hole_cuts_the_run_of_its_own_phase_alone() {
        let mut pos = vec![2500u16; 80];
        for k in 0..8 {
            pos[10 * k] = 2000 + 20 * k as u16;
        }
        let alone = rows(&pos, 0);
        // the hole takes phase 0's fourth sample, 2060, and leaves three
        // steps after it: no run of four passes 20
        let one = holes(&[(2060.0, 2060.0)]);
        assert_eq!(step_run(&[&alone], 4, &one), Some(0));
        assert_eq!(step_run(&[&alone], 3, &one), Some(20));
        // a hole's ends are both inside it
        assert_eq!(
            step_run(&[&alone], 4, &holes(&[(2050.0, 2059.0)])),
            Some(20)
        );
        assert_eq!(step_run(&[&alone], 4, &holes(&[(2041.0, 2060.0)])), Some(0));
        // the same steps on phase 5 from 3000, whose samples lie outside the
        // hole, still hold runs of four
        for k in 0..8 {
            pos[5 + 10 * k] = 3000 + 20 * k as u16;
        }
        assert_eq!(step_run(&[&rows(&pos, 0)], 4, &one), Some(20));
    }

    #[test]
    fn a_hole_whose_ends_are_the_wrong_way_round_is_refused() {
        let e = Holes::new(vec![(121.0, 4.0)], "x".into()).expect_err("refused");
        assert!(
            e.contains("a pot hole runs from its lower end to its upper one"),
            "{e}"
        );
    }

    /// A table result's holes block, every hole's ends; a result written
    /// before the holes holds none, and says so.
    #[test]
    fn the_holes_are_read_off_a_table_result() {
        let t = serde_json::json!({"holes": {"pot": {"holes": [
            {"from_counts": 6.75, "to_counts": 118.8, "counts": 112.05}]}}});
        let h = Holes::of_table(&t, "results/table.json").expect("holes");
        assert_eq!(h.ranges, vec![(6.75, 118.8)]);
        assert_eq!(h.source, "the 1 pot hole of results/table.json");
        let old = Holes::of_table(&serde_json::json!({"step": "table"}), "t.json").expect("old");
        assert!(old.ranges.is_empty());
        assert!(old.source.contains("no pot holes block"), "{}", old.source);
        let wrong = serde_json::json!({"holes": {"pot": {"holes": [
            {"from_counts": 9.0, "to_counts": 3.0}]}}});
        assert!(Holes::of_table(&wrong, "t.json").is_err());
    }

    /// Rows as a loop burst streams them, one tick after another: each
    /// MEDIUM period's speed, count/s, as raw csQ16 over its ten FAST ticks.
    fn loop_rows(speeds: &[f64], from: u64) -> Vec<TelFrame> {
        speeds
            .iter()
            .flat_map(|v| std::iter::repeat_n((v * Q16).round() as i32, DECIM_MED))
            .enumerate()
            .map(|(k, w)| TelFrame {
                tick: from + k as u64,
                omega_hat_cps: Some(w),
                ..TelFrame::default()
            })
            .collect()
    }

    /// The MEDIUM periods of a hold's first 10 ms, the opening the still
    /// speed skips.
    const OPENING: usize = STILL_SKIP_TICKS as usize / DECIM_MED;

    fn opened(v: &[f64]) -> Vec<f64> {
        let mut s = vec![0.0; OPENING];
        s.extend_from_slice(v);
        s
    }

    #[test]
    fn the_still_speed_is_the_largest_speed_of_the_holds_either_way() {
        let a = loop_rows(&opened(&[3.0, -40.5, 12.0]), 0);
        let b = loop_rows(&opened(&[-115.0, 60.0, 0.25]), 5000);
        assert_eq!(still_speed_cps(&[&a, &b]), Some(115.0));
        assert_eq!(still_speed_cps(&[&a]), Some(40.5));
    }

    #[test]
    fn the_first_10_ms_of_each_hold_are_not_read() {
        assert_eq!(STILL_SKIP_TICKS, 200);
        // a burst start's spike of 180 count/s in a hold's first MEDIUM
        // period, then 60 count/s at most
        let mut first = vec![180.0];
        first.extend(vec![5.0; OPENING - 1]);
        first.extend([60.0, -42.0, 31.0]);
        assert_eq!(still_speed_cps(&[&loop_rows(&first, 0)]), Some(60.0));
        // the last period of the opening, ticks 190 to 199, is still
        // skipped; the first one after it, from tick 200, is read
        let mut late = vec![5.0; OPENING - 1];
        late.extend([180.0, 60.0]);
        assert_eq!(still_speed_cps(&[&loop_rows(&late, 0)]), Some(60.0));
        let mut read = vec![5.0; OPENING];
        read.extend([180.0, 60.0]);
        assert_eq!(still_speed_cps(&[&loop_rows(&read, 7)]), Some(180.0));
        // each hold is skipped from its own first row
        let mut second = vec![-195.0];
        second.extend(vec![5.0; OPENING - 1]);
        second.push(-48.0);
        let (a, b) = (loop_rows(&first, 0), loop_rows(&second, 900));
        assert_eq!(still_speed_cps(&[&a, &b]), Some(60.0));
        assert_eq!(still_speed_cps(&[&b]), Some(48.0));
    }

    #[test]
    fn a_hold_with_no_speed_read_gives_none() {
        let mut r = loop_rows(&[1.0; OPENING + 1], 0);
        for f in &mut r {
            f.omega_hat_cps = None;
        }
        assert_eq!(still_speed_cps(&[&r]), None);
        // a hold no longer than its opening has no row read
        assert_eq!(still_speed_cps(&[&loop_rows(&[250.0; OPENING], 0)]), None);
    }
}
