//! First contact at an end stop, read off the motor's ripple (oscnb
//! `stopcontact`).
//!
//! A stop rung drives the shaft onto one end stop open loop, at one duty,
//! with the rows at the control tick. Until the shaft meets the stop the
//! motor turns freely and its current carries the commutation ripple, one
//! cycle per commutation event, so the ripple counts the rotor's turn with
//! no pot. At the stop the rotor slows and the shunt rises to its seated
//! level. First contact is the row where the shunt's 5 ms box has made half
//! that rise, and its place is the rung's ripple cycle at that row.
//!
//! The pot near an end stop is not always the shaft: pressed into a stop it
//! can read anywhere from its seat to its blind end at one rotor angle, so
//! nothing here reads a stop off the pot. The pot enters only upstream of
//! the contact, where it is sound: its speed over the free stretch seeds
//! the ripple, and the rung's samples from its first crossing to
//! [`CUT_CYCLES`] before contact place the rung's ripple count in the frame
//! of a fitted table, the way one rung's offset enters a pooled table fit.
//! A rung whose track reads counts a cycle off the table's has followed
//! another line, and is left out of its stop, named.

use osc_servo_core::adc::ADC_WIDE_SLOTS;

use super::widecodes::compensated;
use super::{interp, median};
use crate::ripple::{TRACK_CYCLES, line_freq, tracked_crossings};

/// s after the drive's edge before a rung's rows are read: the spin-up.
pub const SPIN_UP_S: f64 = 0.1;
/// s after the spin-up the free shunt level and the ripple's seed are read
/// over.
pub const FREE_S: f64 = 0.04;
/// Rows (5 ms at 20 kHz), the shunt's box.
pub const BOX_ROWS: usize = 100;
/// s, the rung's last stretch the seated level is read over.
pub const SEATED_S: f64 = 0.1;
/// s at the rung's very end left out of the seated level.
pub const TAIL_S: f64 = 0.005;
/// First contact at this share of the rise from free to seated.
pub const RISE_SHARE: f64 = 0.5;
/// Raw counts, the least rise a contact is read on: a rung that ends short
/// of its stop.
pub const MIN_RISE_COUNTS: f64 = 30.0;
/// The share either side of the line the free stretch predicts that the
/// ripple's seed is looked for in. A fixed band can hold another line than
/// the fundamental and miss the fundamental itself on a fast rung.
pub const SEED_SHARE: f64 = 0.3;
/// The share a rung's own counts a cycle may stand off the table's over the
/// samples that place it: a track on another line reads a multiple of the
/// table's, and the rung is left out of its stop.
pub const CHECK_SHARE: f64 = 0.15;
/// Cycles before contact a rung's samples stop: the rise from its foot to
/// 50 % takes about as long as 50 to 90 %, 2.6 +/- 1.1 cycles, and a pot
/// that leaves the shaft at the stop starts to fall up to 2.3 cycles before
/// the 50 % row. 4 cycles clears both at the mean plus 1 sd.
pub const CUT_CYCLES: f64 = 4.0;
/// Rows a placement sample, and the pot's centered mean at it.
pub const SAMPLE_ROWS: usize = 16;
/// A rung this far off its stop's median, cycles, is named.
pub const ODD_CYCLES: f64 = 4.0;
/// K, the least distance from a soft limit to first contact, deg: the high
/// contact moved 0.58 deg within one day of sittings and the hold band is
/// 0.2..0.3 deg, 0.9 deg together, rounded up.
pub const HOLD_MARGIN_DEG: f64 = 1.0;
/// On a low-ratio train, where [`CUT_CYCLES`] is more than 1 deg, K stays
/// this many cuts, so a soft limit lies on samples a table was fitted on.
pub const HOLD_CUT_RATIO: f64 = 1.25;

/// The shunt's levels on one stop rung: free over [`FREE_S`] after the
/// spin-up, seated over the last [`SEATED_S`] less [`TAIL_S`], and the
/// first row after the free stretch at [`RISE_SHARE`] of the rise.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Rise {
    pub free: f64,
    pub seated: f64,
    pub row: Option<usize>,
}

impl Rise {
    /// Raw counts from the free shunt level to the seated one.
    pub fn counts(&self) -> f64 {
        self.seated - self.free
    }

    /// Whether the rung reached its stop: a rise of [`MIN_RISE_COUNTS`].
    pub fn valid(&self) -> bool {
        self.counts() >= MIN_RISE_COUNTS
    }
}

/// One rung's first contact. `row` is the contact row, `cycle` the rung's
/// ripple cycle there, counted from the end of the spin-up; `crossings`
/// the ripple's upward crossings, rows, the cycles are read on, and `lead`
/// the cycle at the spin-up's end.
#[derive(Clone, Debug, PartialEq)]
pub struct Contact {
    pub dir: i8,
    pub row: usize,
    pub cycle: f64,
    pub rise: Rise,
    pub f_seed: f64,
    pub crossings: Vec<f64>,
    pub lead: f64,
    pub start: usize,
}

impl Contact {
    pub fn valid(&self) -> bool {
        self.rise.valid()
    }

    /// The rung's ripple cycle at `row`, from the spin-up's end.
    pub fn cycle_at(&self, row: f64) -> f64 {
        let tc = &self.crossings;
        let index: Vec<f64> = (0..tc.len()).map(|k| k as f64).collect();
        interp(row, tc, &index, 0.0, (tc.len() - 1) as f64) + self.lead
    }

    /// The first row whose cycle lies past [`CUT_CYCLES`] before contact:
    /// the rows a table may take end there.
    pub fn cut_row(&self) -> usize {
        let last = self.cycle - CUT_CYCLES;
        (0..self.row)
            .find(|&r| self.cycle_at(r as f64) > last)
            .unwrap_or(self.row)
    }
}

/// numpy's `convolve(x, ones(n) / n, mode="same")`: the mean of `n` rows
/// from `n / 2` before each, the rows past the ends read as 0.
fn boxed(x: &[f64], n: usize) -> Vec<f64> {
    let mut sum = vec![0.0; x.len() + 1];
    for (i, v) in x.iter().enumerate() {
        sum[i + 1] = sum[i] + v;
    }
    (0..x.len())
        .map(|i| {
            let a = i.saturating_sub(n / 2);
            let b = (i + n.div_ceil(2)).min(x.len());
            (sum[b] - sum[a]) / n as f64
        })
        .collect()
}

/// The mean of `n` rows centered on each, fewer at the ends.
pub fn centered_mean(x: &[f64], n: usize) -> Vec<f64> {
    let mut sum = vec![0.0; x.len() + 1];
    for (i, v) in x.iter().enumerate() {
        sum[i + 1] = sum[i] + v;
    }
    (0..x.len())
        .map(|i| {
            let (a, b) = (i.saturating_sub(n / 2), (i + n.div_ceil(2)).min(x.len()));
            (sum[b] - sum[a]) / (b - a) as f64
        })
        .collect()
}

/// The shunt's [`Rise`] on one rung's driven raw shunt rows; None when the
/// rows are too short to read.
pub fn rise(current: &[f64], fs: f64) -> Option<Rise> {
    let start = (SPIN_UP_S * fs).round() as usize;
    let free_rows = (FREE_S * fs).round() as usize;
    let (seated_rows, tail_rows) = (
        (SEATED_S * fs).round() as usize,
        (TAIL_S * fs).round() as usize,
    );
    if current.len() < start + free_rows + seated_rows {
        return None;
    }
    let b = boxed(current, BOX_ROWS);
    let free = median(&b[start..start + free_rows])?;
    let seated = median(&b[b.len() - seated_rows..b.len() - tail_rows])?;
    let after = start + free_rows;
    let row = b[after..]
        .iter()
        .position(|&v| v >= free + RISE_SHARE * (seated - free))
        .map(|k| after + k);
    Some(Rise { free, seated, row })
}

/// The ripple line the free stretch predicts, Hz: the pot's speed over
/// [`FREE_S`] after the spin-up, its centered means at the two ends, over
/// the counts a cycle `counts_per_cycle` gives between those two readings.
/// None for a pot that does not move.
pub fn seed_hz(pos: &[f64], counts_per_cycle: &dyn Fn(f64, f64) -> f64, fs: f64) -> Option<f64> {
    let start = (SPIN_UP_S * fs).round() as usize;
    let free_rows = (FREE_S * fs).round() as usize;
    let end = (start + free_rows + SAMPLE_ROWS).min(pos.len());
    if end < start + free_rows {
        return None;
    }
    let m = centered_mean(&pos[..end], SAMPLE_ROWS);
    let (a, b) = (m[start], m[start + free_rows - 1]);
    if a == b {
        return None;
    }
    let per = counts_per_cycle(a, b);
    (per > 0.0).then(|| (b - a).abs() / ((free_rows - 1) as f64 / fs) / per)
}

/// The `counts_per_cycle` of a table whose angle in cycles at a pot reading
/// is `angle`: the counts between two readings over the cycles the table
/// puts between them.
pub fn table_counts_per_cycle(angle: &dyn Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    (b - a).abs() / (angle(b) - angle(a)).abs()
}

/// The contact of one stop rung toward `dir` (+1 the pot rising), from its
/// driven rows' raw shunt `current` and raw pot `pos`. The ripple is seeded
/// at the line the free stretch's pot speed predicts at `counts_per_cycle`
/// ([`seed_hz`]), looked for within [`SEED_SHARE`] of it, and tracked from
/// the spin-up's end. None when the rows are too short to read, the pot
/// does not move, the spectrum has no line there or the shunt never makes
/// half its rise.
pub fn read(
    current: &[f64],
    pos: &[f64],
    dir: i8,
    counts_per_cycle: &dyn Fn(f64, f64) -> f64,
    fs: f64,
) -> Option<Contact> {
    let start = (SPIN_UP_S * fs).round() as usize;
    let free_rows = (FREE_S * fs).round() as usize;
    let r = rise(current, fs)?;
    let predicted = seed_hz(pos, counts_per_cycle, fs)?;
    let f_seed = line_freq(
        &current[start..start + free_rows],
        (1.0 - SEED_SHARE) * predicted,
        (1.0 + SEED_SHARE) * predicted,
        fs,
    )?;
    let tc = tracked_crossings(
        current,
        start,
        (current.len() - 1) as f64,
        f_seed,
        TRACK_CYCLES,
        fs,
    );
    if tc.len() < 5 {
        return None;
    }
    let row = r.row?;
    let lead = (tc[0] - start as f64) * f_seed / fs;
    let mut c = Contact {
        dir,
        row,
        cycle: 0.0,
        rise: r,
        f_seed,
        crossings: tc,
        lead,
        start,
    };
    c.cycle = c.cycle_at(row as f64);
    Some(c)
}

/// Cycles from the first row the pot `pos` reads past `code` toward the
/// stop to the contact: the reference read, on a pot sound at `code`.
pub fn cycles_past(c: &Contact, pos: &[f64], code: f64) -> Option<f64> {
    let past = pos
        .iter()
        .position(|&p| if c.dir < 0 { p <= code } else { p >= code })?;
    Some(c.cycle - c.cycle_at(past as f64))
}

/// A contact in a table's frame: `angle`, cycles, the table's angle at
/// first contact; `offset` the rung's ripple count's offset in that frame,
/// `samples` how many set it and `residual_sd` their scatter about it,
/// cycles. `counts_per_cycle` is the rung's own over the samples, and
/// `table_counts_per_cycle` the table's over the same travel.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Placed {
    pub angle: f64,
    pub offset: f64,
    pub samples: usize,
    pub residual_sd: f64,
    pub counts_per_cycle: f64,
    pub table_counts_per_cycle: f64,
}

impl Placed {
    /// Whether the rung's track is on the table's line: its counts a cycle
    /// within [`CHECK_SHARE`] of the table's.
    pub fn tracked(&self) -> bool {
        (self.counts_per_cycle / self.table_counts_per_cycle - 1.0).abs() <= CHECK_SHARE
    }
}

/// `c` in the frame of a table, `angle` the table's angle in cycles at a
/// pot reading, rising with the pot. The rung's samples run from its first
/// ripple crossing to [`CUT_CYCLES`] before contact, the pot's centered
/// mean over [`SAMPLE_ROWS`] at each (its wide codes compensated with
/// `slots` when given), and only those inside `span`, the stretch the
/// table was fitted on, when given. Along the rung the table's angle is the
/// offset plus `dir` x the rung's cycle, so the offset is the mean of the
/// samples' angle less `dir` x cycle, the least-squares offset of one rung
/// against a fixed table. None under two samples.
pub fn placed(
    c: &Contact,
    pos: &[u16],
    angle: &dyn Fn(f64) -> f64,
    slots: Option<&[u8; ADC_WIDE_SLOTS]>,
    span: Option<(f64, f64)>,
) -> Option<Placed> {
    let p: Vec<f64> = pos
        .iter()
        .map(|&v| slots.map_or(v as f64, |s| compensated(v, s)))
        .collect();
    let m = centered_mean(&p, SAMPLE_ROWS);
    let first = c.crossings.first()?.ceil() as usize;
    let rows: Vec<(f64, f64)> = (first..c.row.min(m.len()))
        .step_by(SAMPLE_ROWS)
        .map(|r| (m[r], c.cycle_at(r as f64)))
        .filter(|&(_, cyc)| cyc <= c.cycle - CUT_CYCLES)
        .filter(|&(x, _)| span.is_none_or(|(lo, hi)| x >= lo && x <= hi))
        .collect();
    if rows.len() < 2 {
        return None;
    }
    let d = c.dir as f64;
    let at: Vec<f64> = rows.iter().map(|&(x, _)| angle(x)).collect();
    let off: Vec<f64> = rows
        .iter()
        .zip(&at)
        .map(|(&(_, cyc), a)| a - d * cyc)
        .collect();
    let n = off.len() as f64;
    let offset = off.iter().sum::<f64>() / n;
    let sd = (off.iter().map(|o| (o - offset).powi(2)).sum::<f64>() / n).sqrt();
    let (first, last) = (rows[0], rows[rows.len() - 1]);
    let travel = (last.0 - first.0).abs();
    Some(Placed {
        angle: offset + d * c.cycle,
        offset,
        samples: rows.len(),
        residual_sd: sd,
        counts_per_cycle: travel / (last.1 - first.1).abs(),
        table_counts_per_cycle: travel / (at[at.len() - 1] - at[0]).abs(),
    })
}

/// One stop from its rungs' contacts, cycles: the median, the spread
/// (max - min) and the sd over the rungs kept, the rungs more than
/// [`ODD_CYCLES`] off the median and the rungs left out, each by its index
/// in the list given.
#[derive(Clone, Debug, PartialEq)]
pub struct Stop {
    pub contact: f64,
    pub spread: f64,
    pub sd: f64,
    pub n: usize,
    pub odd: Vec<usize>,
    pub left_out: Vec<usize>,
}

/// The [`Stop`] of one end from each rung's contact angle, cycles. A rung
/// `kept` marks false, one whose track failed its check, is left out of
/// the median and named. No other rung is dropped: one far off the median
/// is named. None when no rung is kept.
pub fn stop(angles: &[f64], kept: &[bool]) -> Option<Stop> {
    let k: Vec<f64> = angles
        .iter()
        .zip(kept)
        .filter(|(_, k)| **k)
        .map(|(a, _)| *a)
        .collect();
    let m = median(&k)?;
    let (lo, hi) = k
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &v| {
            (l.min(v), h.max(v))
        });
    Some(Stop {
        contact: m,
        spread: hi - lo,
        sd: super::sd(&k, 1).unwrap_or(0.0),
        n: k.len(),
        odd: (0..angles.len())
            .filter(|&i| kept[i] && (angles[i] - m).abs() > ODD_CYCLES)
            .collect(),
        left_out: (0..angles.len()).filter(|&i| !kept[i]).collect(),
    })
}

/// Output degrees a ripple cycle: one motor turn is `events_per_rev`
/// cycles, one output turn `gear_ratio` motor turns.
pub fn deg_per_cycle(events_per_rev: u32, gear_ratio: f64) -> f64 {
    360.0 / (events_per_rev as f64 * gear_ratio)
}

/// The physical travel, deg: contact to contact, cycles, at the train's
/// degrees a cycle.
pub fn travel_deg(low: f64, high: f64, events_per_rev: u32, gear_ratio: f64) -> f64 {
    (high - low) * deg_per_cycle(events_per_rev, gear_ratio)
}

/// K, deg: the least distance from a soft limit to first contact, at
/// `per_cycle` output degrees a ripple cycle.
pub fn hold_margin_deg(per_cycle: f64) -> f64 {
    HOLD_MARGIN_DEG.max(HOLD_CUT_RATIO * CUT_CYCLES * per_cycle)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 20_000.0;
    const LINE_HZ: f64 = 200.0;
    const FREE: f64 = 1310.0;
    const SEATED: f64 = 1386.0;
    const RIPPLE: f64 = 6.0;

    /// A rung onto a stop: the ripple at [`LINE_HZ`] on the free level, then
    /// the shunt stepping to [`SEATED`] with the rotor still at
    /// `contact_row`.
    fn synthetic(rows: usize, contact_row: usize) -> Vec<f64> {
        (0..rows)
            .map(|k| {
                let t = k.min(contact_row) as f64 / FS;
                if k < contact_row {
                    FREE + RIPPLE * (2.0 * std::f64::consts::PI * LINE_HZ * t).sin()
                } else {
                    SEATED
                }
            })
            .collect()
    }

    fn per(v: f64) -> impl Fn(f64, f64) -> f64 {
        move |_, _| v
    }

    #[test]
    fn a_rung_reads_its_contact_at_half_the_rise_and_counts_cycles_from_the_spin_up() {
        let pos: Vec<f64> = (0..13000).map(|k| k as f64 * 0.038).collect();
        let c = read(&synthetic(13000, 9000), &pos, 1, &per(3.8), FS).expect("a contact");
        assert!(c.row.abs_diff(9000) <= 5, "{}", c.row);
        assert!((c.rise.free - FREE).abs() <= 0.5 && (c.rise.seated - SEATED).abs() < 1e-9);
        assert!((c.cycle - (c.row as f64 - 2000.0) / FS * LINE_HZ).abs() <= 0.05);
        assert!((c.cycle - 70.0).abs() <= 0.1, "{}", c.cycle);
    }

    #[test]
    fn a_rung_that_ends_short_of_its_stop_reads_no_contact() {
        let free = synthetic(13000, usize::MAX);
        let moving: Vec<f64> = (0..free.len()).map(|k| 470.0 - k as f64 * 0.038).collect();
        let c = read(&free, &moving, -1, &per(3.8), FS).expect("read");
        assert!(!c.valid());
        assert!(read(&free[..4000], &moving[..4000], -1, &per(3.8), FS).is_none());
        assert!(read(&free, &vec![470.0; free.len()], -1, &per(3.8), FS).is_none());
    }

    #[test]
    fn a_contact_placed_in_a_tables_frame_is_the_rungs_offset_plus_its_cycles() {
        let current = synthetic(13000, 9000);
        let fall: Vec<f64> = (0..current.len())
            .map(|k| 470.0 - k as f64 * 0.038)
            .collect();
        let c = read(&current, &fall, -1, &per(3.8), FS).expect("a contact");
        let pos: Vec<u16> = (0..current.len())
            .map(|k| {
                let cyc = c.cycle_at(k as f64);
                (470.0 - 3.8 * cyc + 4.0 * (cyc / 20.0).sin()).round() as u16
            })
            .collect();
        let angle = |x: f64| (x - 4.0 * ((470.0 - x) / 3.8 / 20.0).sin()) / 3.8;
        let p = placed(&c, &pos, &angle, None, None).expect("placed");
        assert!(
            (p.angle - (470.0 - 3.8 * c.cycle) / 3.8).abs() <= 0.1,
            "{p:?}"
        );
        assert!(p.residual_sd < 0.2);
        let mut dead = pos.clone();
        for v in &mut dead[c.row - 2 * (FS / LINE_HZ).round() as usize..] {
            *v = 4;
        }
        assert_eq!(
            placed(&c, &dead, &angle, None, None).expect("placed").angle,
            p.angle
        );
        let inside = placed(&c, &pos, &angle, None, Some((200.0, 400.0))).expect("placed");
        assert!(inside.samples < p.samples && (inside.angle - p.angle).abs() <= 0.1);
    }

    #[test]
    fn a_stop_is_its_rungs_median_with_the_spread_and_a_far_rung_named() {
        let s = stop(&[27.31, 27.20, 27.35, 32.0, 27.38], &[true; 5]).expect("a stop");
        assert_eq!(
            (s.contact, s.n, &s.odd[..], &s.left_out[..]),
            (27.35, 5, &[3][..], &[][..])
        );
        assert!((s.spread - (32.0 - 27.20)).abs() < 1e-12);
        let s = stop(
            &[27.31, 27.20, 27.35, 13.0, 27.38],
            &[true, true, true, false, true],
        )
        .expect("a stop");
        assert!((s.contact - 27.33).abs() < 1e-12);
        assert_eq!((s.n, &s.odd[..], &s.left_out[..]), (4, &[][..], &[3][..]));
        assert!((s.spread - (27.38 - 27.20)).abs() < 1e-12);
        assert_eq!(stop(&[], &[]), None);
        assert_eq!(stop(&[27.3], &[false]), None);
    }

    #[test]
    fn the_travel_and_the_hold_margin_are_in_the_trains_degrees() {
        let per = deg_per_cycle(6, 308.0533);
        assert!((per - 0.1948).abs() < 5e-5);
        assert!((travel_deg(27.31, 965.08, 6, 308.0533) - 937.77 * per).abs() < 1e-9);
        assert_eq!(hold_margin_deg(per), 1.0);
        assert!((hold_margin_deg(deg_per_cycle(6, 30.0)) - 10.0).abs() < 1e-12);
    }
}
