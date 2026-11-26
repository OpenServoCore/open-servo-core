//! `table`: the position table the servo holds, read as `install` and
//! `check` read it, and graded; nothing written. The graph and the grade say
//! what the table does to the pot's local gain, interval by interval, with
//! an advisory letter from nb09's numbers: the firmware judges physics only
//! (identity at the stops, monotone, under 16x), quality is the operator's
//! call.

use std::fmt::Write;

use anyhow::{Result, anyhow};
use osc_client::data_state::DataState;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::pos_lut::{self, INTERVALS, PosLut};
use osc_client::{Client, Id};
use osc_measure::state;
use osc_servo_core::pos_lut::{GRID, POINTS, Reject, interp_q4, validate};
use serde::Serialize;

/// The firmware's shape bound: an interval's gain under this many times
/// nominal (`pos_lut::validate`).
const GAIN_MAX: i32 = 16;
/// Local gain is judged over this many raw counts, as nb09 did.
const WINDOW: u16 = 25;
/// Grade A: every window within this band of nominal (the mg90-a grid
/// table: 0.61..1.88x; nb09's dense curve 0.53..1.53x).
const SMOOTH: (f64, f64) = (0.5, 2.0);
/// Grade B: within this band (the SG90 class swings ~6x across 20 mV bins).
const COARSE: (f64, f64) = (0.25, 4.0);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Grade {
    A,
    B,
    C,
}

impl Grade {
    fn text(self) -> &'static str {
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
pub struct Interval {
    pub raw: u16,
    pub gain: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Windows {
    pub counts: u16,
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct Report {
    pub stops: [u16; 2],
    pub nonzero: usize,
    /// Raw counts from the first nonzero point to the last.
    pub span: Option<[u16; 2]>,
    pub max_abs: i16,
    pub steepest: Option<Interval>,
    pub shallowest: Option<Interval>,
    pub windows: Option<Windows>,
    pub grade: Option<Grade>,
    /// The intervals the table touches: first nonzero point - 1 ..= last.
    #[serde(skip)]
    band: Option<(usize, usize)>,
}

impl Report {
    pub fn new(k: &[i16; POINTS], stops: (u16, u16)) -> Report {
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
        let counts = |raw: u16| interp_q4(raw, k) as f64 / GRID as f64;
        let (lo, hi) = (raw(band.0), raw(band.1 + 1));
        let (mut min, mut max) = (f64::INFINITY, f64::NEG_INFINITY);
        for r0 in lo..=hi - WINDOW {
            let g = (counts(r0 + WINDOW) - counts(r0)) / WINDOW as f64;
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

    /// The numbers, one line each.
    pub fn summary(&self) -> String {
        let mut s = String::new();
        let _ = write!(s, "stops {}..{}; ", self.stops[0], self.stops[1]);
        match self.span {
            Some([a, z]) => {
                let _ = writeln!(
                    s,
                    "calibration points: {} nonzero at raw {a}..{z}, |c| up to {}",
                    self.nonzero, self.max_abs
                );
            }
            None => s.push_str("calibration points: none nonzero (identity)\n"),
        }
        if let (Some(steep), Some(shallow), Some(w), Some(g)) =
            (self.steepest, self.shallowest, self.windows, self.grade)
        {
            let at = |i: Interval| format!("{}..{}", i.raw, i.raw + GRID as u16);
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
    (k * GRID) as u16
}

/// The servo's table, its stops and its data state, as read.
pub struct Table {
    id: Id,
    lut: PosLut,
    stops: (u16, u16),
    data: DataState,
}

#[derive(Serialize)]
struct Shown<'a> {
    state: u8,
    state_name: Option<&'static str>,
    live: bool,
    #[serde(flatten)]
    report: Report,
    points: &'a [i16],
}

pub async fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<Table> {
    let s = state::read(c, id, d).await?;
    let stop = |name: &str| -> Result<u16> {
        let f = d
            .field(name)
            .ok_or_else(|| anyhow!("no {name} in {}", d.model))?;
        match s.bytes(f) {
            Some(&[lo, hi]) => Ok(u16::from_le_bytes([lo, hi])),
            _ => Err(anyhow!("{name} is not two bytes")),
        }
    };
    Ok(Table {
        id,
        stops: (stop("raw_min")?, stop("raw_max")?),
        lut: s.lut,
        data: s.data,
    })
}

impl Table {
    fn grid(&self) -> [i16; POINTS] {
        let mut points = [0i16; POINTS];
        points[..INTERVALS].copy_from_slice(&self.lut.points);
        points
    }

    fn report(&self) -> Report {
        Report::new(&self.grid(), self.stops)
    }

    /// The data state's one-line warning, when it names a reason.
    pub fn warning(&self) -> Option<String> {
        let msg = self.data.message()?;
        Some(format!(
            "warning: id {}: {} - {msg}",
            self.id.as_byte(),
            self.data.names()
        ))
    }

    /// The table and its report as JSON.
    pub fn json(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(&Shown {
            state: self.lut.state,
            state_name: self.lut.state_name(),
            live: self.lut.live(),
            report: self.report(),
            points: &self.lut.points,
        })
    }

    /// State, stops, nonzero calibration points, grade, and why the
    /// firmware would refuse the points against the servo's stops.
    pub fn text(&self) -> String {
        let state = match pos_lut::state::name(self.lut.state) {
            Some(n) => n.to_string(),
            None => format!("state {}", self.lut.state),
        };
        let mut s = format!(
            "id {}: lut {state}{}\n",
            self.id.as_byte(),
            if self.lut.live() {
                ""
            } else {
                " (the kernel applies the identity)"
            }
        );
        s.push_str(&self.report().summary());
        if let Some(why) = explain(&self.grid(), self.stops) {
            let _ = writeln!(s, "against the servo's stops: {why}");
        }
        s
    }
}

/// Why the firmware would refuse `k` against `stops`, in its own terms.
fn explain(k: &[i16; POINTS], stops: (u16, u16)) -> Option<String> {
    let (raw_min, raw_max) = stops;
    match validate(k, raw_min, raw_max) {
        Ok(()) => None,
        Err(Reject::Ends) => {
            let lo = (raw_min as usize).div_ceil(GRID);
            let hi = raw_max as usize / GRID;
            let at = (0..POINTS)
                .find(|&i| (i <= lo || i >= hi) && k[i] != 0)
                .unwrap_or(0);
            Some(format!(
                "REJECT_ENDS: calibration point {at} (raw {}) is {} inside the identity inset of stops {raw_min}..{raw_max} (calibration points <= {lo} and >= {hi} must be 0); measure the table again against these stops (osc-ident run)",
                at * GRID,
                k[at]
            ))
        }
        Err(Reject::Shape) => {
            let at = k
                .windows(2)
                .position(|w| {
                    let d = GRID as i32 + w[1] as i32 - w[0] as i32;
                    !(1..GAIN_MAX * GRID as i32).contains(&d)
                })
                .unwrap_or(0);
            let d = GRID as i32 + k[at + 1] as i32 - k[at] as i32;
            Some(format!(
                "REJECT_SHAPE: interval at raw {}..{} has gain {:.2}x nominal (must be within 1/{GRID}..{GAIN_MAX}x); not a pot, measure the table again (osc-ident run)",
                at * GRID,
                (at + 1) * GRID,
                d as f64 / GRID as f64
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures_lite::future::block_on;
    use osc_client::BaudRate;
    use osc_client::fake::FakePipe;
    use osc_measure::install;
    use osc_measure::set::{SetFile, TableFile};

    const IDENTITY: [i16; POINTS] = [0; POINTS];

    /// The mg90-a grid table the notebook's nb09 numbers were read on.
    fn mg90_a() -> ([i16; POINTS], (u16, u16)) {
        let img: serde_json::Value = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../client/testdata/lut/pos-lut-mg90-a-grid.json"
        )))
        .unwrap();
        let mut k = [0i16; POINTS];
        let points = img["points"].as_array().unwrap();
        assert_eq!(points.len(), INTERVALS);
        for (d, p) in k.iter_mut().zip(points) {
            *d = p.as_i64().unwrap() as i16;
        }
        let stop = |name: &str| img[name].as_u64().unwrap() as u16;
        (k, (stop("raw_min"), stop("raw_max")))
    }

    #[test]
    fn mg90_a_grades_a_with_the_notebooks_numbers() {
        let (k, stops) = mg90_a();
        let r = Report::new(&k, stops);
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
            text.contains("calibration points: 185 nonzero at raw 560..3504, |c| up to 71"),
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
        let json = serde_json::to_value(&r).unwrap();
        assert_eq!(json["grade"], "A");
        assert_eq!(json["windows"]["counts"], 25);
        assert!(json.get("band").is_none());
    }

    #[test]
    fn identity_has_nothing_to_grade() {
        let r = Report::new(&IDENTITY, (209, 3849));
        assert_eq!((r.nonzero, r.span, r.grade), (0, None, None));
        assert!(r.summary().contains("none nonzero (identity)"));
    }

    #[test]
    fn grade_thresholds() {
        assert_eq!(Grade::of(0.5, 2.0), Grade::A);
        assert_eq!(Grade::of(0.49, 2.0), Grade::B);
        assert_eq!(Grade::of(0.5, 4.0), Grade::B);
        assert_eq!(Grade::of(0.24, 1.0), Grade::C);
        assert_eq!(Grade::of(1.0, 4.01), Grade::C);
        // a coarse SG90-class step: one 6x interval
        let mut k = IDENTITY;
        k[100] = 80;
        for (i, c) in (101..).zip((1..=5).rev()) {
            k[i] = c * 15;
        }
        let r = Report::new(&k, (5, 4095));
        assert_eq!(r.steepest.unwrap().raw, 1584);
        assert_eq!(r.grade, Some(Grade::C));
    }

    #[test]
    fn explain_names_the_offending_point_or_interval() {
        let mut k = IDENTITY;
        assert_eq!(explain(&k, (209, 3849)), None);
        k[14] = 1;
        let why = explain(&k, (209, 3849)).unwrap();
        assert!(
            why.starts_with("REJECT_ENDS: calibration point 14 (raw 224) is 1 inside"),
            "{why}"
        );
        assert!(why.contains("calibration points <= 14 and >= 240"), "{why}");
        k[14] = 0;
        k[100] = 16;
        let why = explain(&k, (209, 3849)).unwrap();
        assert!(
            why.starts_with("REJECT_SHAPE: interval at raw 1600..1616 has gain 0.00x"),
            "{why}"
        );
        k[100] = 15;
        assert_eq!(explain(&k, (209, 3849)), None);
    }

    /// On a simulated servo holding the set of record and its table, the
    /// read gives the table's points, its state and the set's stops, in the
    /// JSON shape the bench scripts read.
    #[test]
    fn reads_the_installed_table_and_writes_nothing() {
        let testdata = |rel: &str| {
            std::fs::read_to_string(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../measure/testdata/mg90-d")
                    .join(rel),
            )
            .expect("read")
        };
        let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
        pipe.seed_calibrated(0);
        let mut c = block_on(Client::connect(pipe)).expect("connect");
        let id = Id::new(1);
        let d = crate::descriptor_builtin().expect("descriptor");

        let fresh = block_on(read(&mut c, id, &d)).expect("read");
        assert!(!fresh.lut.live());
        assert!(fresh.text().contains("none nonzero (identity)"));

        let record = SetFile::parse(&testdata("constants.json")).expect("the set of record");
        let table = TableFile::parse(&testdata("pos_lut.json")).expect("table");
        block_on(install::install(
            &mut c,
            id,
            &d,
            &record,
            Some(&table.points),
            &mut Vec::new(),
        ))
        .expect("the servo takes the set of record");
        let before = block_on(state::read(&mut c, id, &d)).expect("state");

        let t = block_on(read(&mut c, id, &d)).expect("read");
        let shown: serde_json::Value = serde_json::from_str(&t.json().unwrap()).unwrap();
        let points: Vec<i16> = shown["points"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| p.as_i64().unwrap() as i16)
            .collect();
        assert_eq!(points, table.points.to_vec());
        assert_eq!(shown["state_name"], "LIVE");
        assert_eq!(shown["live"], true);
        assert_eq!(shown["state"], pos_lut::state::LIVE);
        let stops = [&shown["stops"][0], &shown["stops"][1]].map(|v| v.as_u64().unwrap() as u16);
        assert_eq!(
            (stops[0], stops[1]),
            t.stops,
            "the stops are the servo's raw_min..raw_max"
        );
        for key in [
            "nonzero",
            "span",
            "max_abs",
            "steepest",
            "shallowest",
            "windows",
            "grade",
        ] {
            assert!(shown.get(key).is_some(), "{key} in {shown}");
        }
        let text = t.text();
        assert!(text.starts_with("id 1: lut LIVE\nstops "), "{text}");
        assert!(!text.contains("against the servo's stops"), "{text}");

        let after = block_on(state::read(&mut c, id, &d)).expect("state");
        assert_eq!(after.raw, before.raw, "nothing written");
        assert_eq!(after.lut.points, before.lut.points);
    }
}
