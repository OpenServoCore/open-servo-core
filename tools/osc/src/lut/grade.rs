//! The graph and the grade: what a table does to the pot's local gain,
//! interval by interval, and an advisory letter from nb09's numbers. The
//! firmware judges physics only (identity at the stops, monotone, under
//! 16x); quality is the operator's call, and this is what they look at.

use std::fmt::Write;

use osc_ident::lut::{GRID, GridLut, INTERVALS};
use serde::Serialize;

/// Local gain is judged over this many raw counts, as nb09 did.
pub(crate) const WINDOW: u16 = 25;
/// Grade A: every window within this band of nominal (the mg90-a grid
/// table: 0.61..1.88x; nb09's dense curve 0.53..1.53x).
pub(crate) const SMOOTH: (f64, f64) = (0.5, 2.0);
/// Grade B: within this band (the SG90 class swings ~6x across 20 mV bins).
pub(crate) const COARSE: (f64, f64) = (0.25, 4.0);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub(crate) enum Grade {
    A,
    B,
    C,
}

impl Grade {
    pub(crate) fn text(self) -> &'static str {
        match self {
            Grade::A => "smooth",
            Grade::B => "coarse track",
            Grade::C => "suspect",
        }
    }

    fn of(min: f64, max: f64) -> Grade {
        if min >= SMOOTH.0 && max <= SMOOTH.1 {
            Grade::A
        } else if min >= COARSE.0 && max <= COARSE.1 {
            Grade::B
        } else {
            Grade::C
        }
    }
}

/// One interval's gain, x nominal, at the raw count it starts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub(crate) struct Interval {
    pub raw: u16,
    pub gain: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub(crate) struct Windows {
    pub counts: u16,
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Debug, Serialize)]
pub(crate) struct Report {
    pub stops: [u16; 2],
    pub nonzero: usize,
    /// Raw counts from the first nonzero point to the last.
    pub span: Option<[u16; 2]>,
    pub max_abs: i16,
    pub steepest: Option<Interval>,
    pub shallowest: Option<Interval>,
    pub windows: Option<Windows>,
    pub grade: Option<Grade>,
    #[serde(skip)]
    gains: [f64; INTERVALS],
    /// The intervals the table touches: first nonzero point - 1 ..= last.
    #[serde(skip)]
    band: Option<(usize, usize)>,
}

impl Report {
    pub(crate) fn new(lut: &GridLut, stops: (u16, u16)) -> Report {
        let k = &lut.points;
        let mut gains = [1.0; INTERVALS];
        for (i, g) in gains.iter_mut().enumerate() {
            *g = (GRID as i32 + k[i + 1] as i32 - k[i] as i32) as f64 / GRID as f64;
        }
        let nonzero: Vec<usize> = (0..INTERVALS).filter(|&i| k[i] != 0).collect();
        let mut r = Report {
            stops: [stops.0, stops.1],
            nonzero: nonzero.len(),
            span: None,
            max_abs: k.iter().map(|c| c.abs()).max().unwrap_or(0),
            steepest: None,
            shallowest: None,
            windows: None,
            grade: None,
            gains,
            band: None,
        };
        let (Some(&first), Some(&last)) = (nonzero.first(), nonzero.last()) else {
            return r;
        };
        r.span = Some([raw(first), raw(last)]);
        let band = (first.saturating_sub(1), last);
        r.band = Some(band);
        let at = |i: usize| Interval {
            raw: raw(i),
            gain: gains[i],
        };
        let mut steep = at(band.0);
        let mut shallow = steep;
        for (i, &g) in gains.iter().enumerate().take(band.1 + 1).skip(band.0) {
            if g > steep.gain {
                steep = at(i);
            }
            if g < shallow.gain {
                shallow = at(i);
            }
        }
        r.steepest = Some(steep);
        r.shallowest = Some(shallow);
        let (lo, hi) = (raw(band.0), raw(band.1 + 1));
        let (mut min, mut max) = (f64::INFINITY, f64::NEG_INFINITY);
        for r0 in lo..=hi - WINDOW {
            let g = (lut.counts(r0 + WINDOW) - lut.counts(r0)) / WINDOW as f64;
            min = min.min(g);
            max = max.max(g);
        }
        r.windows = Some(Windows {
            counts: WINDOW,
            min,
            max,
        });
        r.grade = Some(Grade::of(min, max));
        r
    }

    /// Every interval as one character of gain, 64 per row, so the whole
    /// travel is four rows.
    pub(crate) fn chart(&self) -> String {
        const PER_ROW: usize = 64;
        let mut s = String::new();
        s.push_str("local gain per 16-count interval, x nominal:");
        for (c, label) in LEVELS.iter().zip(LEVEL_TEXT) {
            let _ = write!(s, "  {c} {label}");
        }
        s.push('\n');
        for (row, gains) in self.gains.chunks(PER_ROW).enumerate() {
            let bar: String = gains.iter().map(|&g| level(g)).collect();
            let _ = writeln!(s, "{:>5} |{bar}|", row * PER_ROW * GRID as usize);
        }
        s
    }

    /// The numbers under the chart, one line each.
    pub(crate) fn summary(&self) -> String {
        let mut s = String::new();
        let _ = write!(s, "stops {}..{}; ", self.stops[0], self.stops[1]);
        match self.span {
            Some([a, z]) => {
                let _ = writeln!(
                    s,
                    "knots: {} nonzero at raw {a}..{z}, |c| up to {}",
                    self.nonzero, self.max_abs
                );
            }
            None => s.push_str("knots: none nonzero (identity)\n"),
        }
        if let (Some(steep), Some(shallow), Some(w), Some(g)) =
            (self.steepest, self.shallowest, self.windows, self.grade)
        {
            let at = |i: Interval| format!("{}..{}", i.raw, i.raw + GRID);
            let _ = writeln!(
                s,
                "interval gain: steepest {:.2}x at raw {}, shallowest {:.2}x at raw {}",
                steep.gain,
                at(steep),
                shallow.gain,
                at(shallow)
            );
            let (band0, band1) = self.band.unwrap_or((0, 0));
            let _ = writeln!(
                s,
                "{}-count windows over raw {}..{}: {:.2}x .. {:.2}x",
                w.counts,
                raw(band0),
                raw(band1 + 1),
                w.min,
                w.max
            );
            let _ = writeln!(
                s,
                "grade {g:?} ({}): every window within {}..{}x is A, within {}..{}x is B, beyond is C; advisory only, the firmware accepts anything under 16x and the graph decides",
                g.text(),
                SMOOTH.0,
                SMOOTH.1,
                COARSE.0,
                COARSE.1
            );
        }
        s
    }
}

fn raw(k: usize) -> u16 {
    (k * GRID as usize) as u16
}

const LEVELS: [char; 8] = ['_', '.', ':', '-', '=', '+', '*', '#'];
const LEVEL_TEXT: [&str; 8] = [
    "<0.5", "0.5-0.7", "0.7-0.9", "0.9-1.1", "1.1-1.4", "1.4-2", "2-4", ">=4",
];
const LEVEL_EDGES: [f64; 7] = [0.5, 0.7, 0.9, 1.1, 1.4, 2.0, 4.0];

fn level(gain: f64) -> char {
    let i = LEVEL_EDGES.iter().filter(|&&e| gain >= e).count();
    LEVELS[i]
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_ident::lut::Image;

    const COMMITTED: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../ident/testdata/lut/pos-lut-mg90-a-grid.json"
    ));

    fn mg90_a() -> (GridLut, (u16, u16)) {
        let img: Image = serde_json::from_str(COMMITTED).unwrap();
        (img.lut().unwrap(), (img.raw_min, img.raw_max))
    }

    #[test]
    fn mg90_a_grades_a_with_the_notebooks_numbers() {
        let (lut, stops) = mg90_a();
        let r = Report::new(&lut, stops);
        assert_eq!(r.stops, [209, 3849]);
        assert_eq!((r.nonzero, r.span), (185, Some([560, 3504])));
        assert_eq!(r.max_abs, 71);
        let steep = r.steepest.unwrap();
        assert_eq!(steep.raw, 1360);
        assert!((steep.gain - 2.0625).abs() < 1e-9, "{}", steep.gain);
        let w = r.windows.unwrap();
        assert_eq!(w.counts, 25);
        assert!((w.min - 0.6125).abs() < 1e-9, "{}", w.min);
        assert!((w.max - 1.8825).abs() < 1e-9, "{}", w.max);
        assert_eq!(r.grade, Some(Grade::A));
        let text = r.summary();
        assert!(
            text.contains("knots: 185 nonzero at raw 560..3504, |c| up to 71"),
            "{text}"
        );
        assert!(
            text.contains("steepest 2.06x at raw 1360..1376, shallowest 0.50x at raw 752..768"),
            "{text}"
        );
        assert!(
            text.contains("25-count windows over raw 544..3520: 0.61x .. 1.88x"),
            "{text}"
        );
        assert!(text.contains("grade A (smooth)"), "{text}");
        let chart = r.chart();
        assert_eq!(chart.lines().count(), 5);
        assert!(chart.lines().nth(1).unwrap().starts_with("    0 |"));
        assert!(chart.lines().nth(4).unwrap().starts_with(" 3072 |"));
        assert!(chart.is_ascii());
        // the steepest interval, 1360 = row 1 column 21, reads as 2-4x
        let row1 = chart.lines().nth(2).unwrap();
        assert_eq!(row1.as_bytes()[7 + 21], b'*', "{row1}");
        let json = serde_json::to_value(&r).unwrap();
        assert_eq!(json["grade"], "A");
        assert_eq!(json["windows"]["counts"], 25);
        assert!(json.get("gains").is_none());
    }

    #[test]
    fn identity_has_nothing_to_grade() {
        let r = Report::new(&GridLut::IDENTITY, (209, 3849));
        assert_eq!((r.nonzero, r.span, r.grade), (0, None, None));
        assert!(r.summary().contains("none nonzero (identity)"));
        assert!(r.chart().lines().nth(1).unwrap().contains(&"-".repeat(64)));
    }

    #[test]
    fn grade_thresholds_and_levels() {
        assert_eq!(Grade::of(0.5, 2.0), Grade::A);
        assert_eq!(Grade::of(0.49, 2.0), Grade::B);
        assert_eq!(Grade::of(0.5, 4.0), Grade::B);
        assert_eq!(Grade::of(0.24, 1.0), Grade::C);
        assert_eq!(Grade::of(1.0, 4.01), Grade::C);
        assert_eq!(level(0.1), '_');
        assert_eq!(level(1.0), '-');
        assert_eq!(level(2.0), '*');
        assert_eq!(level(16.0), '#');
        // a coarse SG90-class step: one 6x interval
        let mut lut = GridLut::IDENTITY;
        lut.points[100] = 80;
        for (i, k) in (101..).zip((1..=5).rev()) {
            lut.points[i] = k * 15;
        }
        let r = Report::new(&lut, (5, 4095));
        assert_eq!(r.steepest.unwrap().raw, 1584);
        assert_eq!(r.grade, Some(Grade::C));
    }
}
