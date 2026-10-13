//! The watch: what a drive streams while it waits. Every pause of a drive
//! is a TEL burst of `current`, `duty` and `vdiff`, and `pos` when the line
//! carries it too, chained past [`WATCH_BURST_MS`]. The host folds each
//! 16-row window of it into floor means and the current's extremes: the
//! safety envelope judges every row and every window, the fits take every
//! window, and the next region read carries the latest ([`Windows`]). The
//! fold reads the decoded rows' fields by name; which fields stream, and in
//! what layout, is the driver's selection and the protocol's decoder.
//!
//! The fields are the kernel's own: `current` and `vdiff` hold the last
//! valid window through the ones the shunt or the taps cannot read, the
//! current reads 0 from the first tick nothing drives, and `duty` is the
//! compare count the bridge applied (protocol sec 5.6).

use crate::frame::{TelFrame, TelemetrySnapshot};
use crate::limits::PERMIT_REFRESH_MS;

/// Fast ticks one window spans, as a shift: the mean is `sum >> 4`, which
/// floors toward -inf.
pub const WINDOW_SHIFT: u32 = 4;
pub const WINDOW_TICKS: u16 = 1 << WINDOW_SHIFT;

/// The longest watch burst, ms: the host is silent through a burst, so a
/// held stall permit is rewritten between them, and a guard trip waits at
/// most this long for the line.
pub const WATCH_BURST_MS: u32 = PERMIT_REFRESH_MS;

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Window {
    pub i_mean: i16,
    pub i_min: i16,
    pub i_max: i16,
    pub vdiff_mean: i16,
    /// PWM compare counts.
    pub duty_mean: i16,
}

/// The window of the first [`WINDOW_TICKS`] rows, None unless they are
/// consecutive ticks carrying `current`, `vdiff` and `duty`: a lost frame
/// or a row the servo dropped leaves no window rather than a short one.
pub fn fold(rows: &[TelFrame]) -> Option<Window> {
    let rows = rows.get(..WINDOW_TICKS as usize)?;
    let (mut i_sum, mut v_sum, mut d_sum) = (0i32, 0i32, 0i32);
    let (mut i_min, mut i_max) = (i16::MAX, i16::MIN);
    for (k, r) in rows.iter().enumerate() {
        if r.tick != rows[0].tick + k as u64 {
            return None;
        }
        let (i, v, d) = (r.current?, r.vdiff?, r.duty_counts?);
        i_sum += i as i32;
        v_sum += v as i32;
        d_sum += d as i32;
        i_min = i_min.min(i);
        i_max = i_max.max(i);
    }
    // sixteen i16 samples: each mean is an i16
    let mean = |sum: i32| (sum >> WINDOW_SHIFT) as i16;
    Some(Window {
        i_mean: mean(i_sum),
        i_min,
        i_max,
        vdiff_mean: mean(v_sum),
        duty_mean: mean(d_sum),
    })
}

/// Whole windows of rows in `ms` at `tick_hz`.
pub fn watch_rows(ms: f64, tick_hz: f64) -> u16 {
    let rows = (ms * tick_hz / 1000.0).clamp(0.0, u16::MAX as f64) as u16;
    rows & !(WINDOW_TICKS - 1)
}

/// One window of a watch.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Watched {
    pub window: Window,
    /// The window's number: the servo tick of its last row, in windows.
    pub seq: u64,
    /// When its last row was sampled, ms on the host's clock.
    pub t_ms: f64,
    /// Its last row's pot reading, raw counts, when the watch carries it.
    pub pos: Option<u16>,
}

/// One watch burst as the host folded it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Watch {
    /// Every whole window, aligned to the burst's first row.
    pub windows: Vec<Watched>,
    /// The pot's extremes over every row, when the watch carries it.
    pub pos: Option<(u16, u16)>,
    /// Rows asked for that never arrived: frames the line lost, rows the
    /// servo dropped.
    pub lost: u16,
    /// A frame flagged a fault: the servo latched one.
    pub alert: bool,
    /// The full scale of the duty counts.
    pub pwm_period_ticks: u16,
}

impl Watch {
    /// Fold `frames`, a watch burst of `rows` rows, whose
    /// last row the host received at `t_end_ms`, from a servo ticking at
    /// `tick_hz`.
    pub fn fold(
        frames: &[TelFrame],
        rows: u16,
        alert: bool,
        t_end_ms: f64,
        tick_hz: f64,
        pwm_period_ticks: u16,
    ) -> Self {
        let mut w = Self {
            alert,
            pwm_period_ticks,
            ..Self::default()
        };
        let (Some(first), Some(last)) = (frames.first(), frames.last()) else {
            w.lost = rows;
            return w;
        };
        let span = (last.tick - first.tick + 1) as usize;
        w.lost = (rows as usize).max(span).saturating_sub(frames.len()) as u16;
        let tick_ms = 1000.0 / tick_hz;
        let end_tick = (t_end_ms / tick_ms).round() as u64;
        for chunk in frames.chunks(WINDOW_TICKS as usize) {
            let (Some(window), Some(r)) = (fold(chunk), chunk.last()) else {
                continue;
            };
            let behind = last.tick - r.tick;
            w.windows.push(Watched {
                window,
                seq: end_tick.saturating_sub(behind) >> WINDOW_SHIFT,
                t_ms: t_end_ms - behind as f64 * tick_ms,
                pos: r.pos,
            });
        }
        w.pos = frames
            .iter()
            .filter_map(|f| f.pos)
            .fold(None, |m, p| match m {
                None => Some((p, p)),
                Some((lo, hi)) => Some((lo.min(p), hi.max(p))),
            });
        w
    }
}

/// The latest window watched, for the region reads that follow: a read
/// with no window since the last repeats it, `agg_seq` and all, so a
/// consumer that keys on `agg_seq` takes no new sample from it.
#[derive(Clone, Debug, Default)]
pub struct Windows {
    last: Option<(u16, Window)>,
    pwm_period_ticks: u16,
}

impl Windows {
    /// `pwm_period_ticks`: the servo's PWM period, the full scale of its
    /// duty counts.
    pub fn new(pwm_period_ticks: u16) -> Self {
        Self {
            last: None,
            pwm_period_ticks,
        }
    }

    pub fn watched(&mut self, w: &Watch) {
        if let Some(x) = w.windows.last() {
            self.last = Some((x.seq as u16, x.window));
        }
    }

    pub fn attach(&self, snap: &mut TelemetrySnapshot) {
        let (seq, w) = self.last.unwrap_or_default();
        snap.agg_seq = seq;
        snap.i_mean_counts = w.i_mean;
        snap.i_min_counts = w.i_min;
        snap.i_max_counts = w.i_max;
        snap.vdiff_mean = w.vdiff_mean;
        snap.duty_mean_counts = w.duty_mean;
        snap.pwm_period_ticks = self.pwm_period_ticks;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::duty_counts;

    const ARR: u16 = 1200;

    /// The servo's own aggregate as it shipped, verbatim but for the
    /// output struct: the reference the host fold must reproduce.
    mod firmware {
        pub const IDENT_SHIFT: u32 = 4;
        const IDENT_WINDOW: u8 = 1 << IDENT_SHIFT;

        #[derive(Debug, PartialEq)]
        pub struct IdentOut {
            pub i_mean_counts: i16,
            pub i_min_counts: i16,
            pub i_max_counts: i16,
            pub vdiff_mean: i16,
            pub duty_mean_q15: i16,
        }

        pub struct IdentAgg {
            ctr: u8,
            i_sum: i32,
            i_min: i16,
            i_max: i16,
            vdiff_sum: i32,
            duty_sum: i32,
        }

        fn mean_i16(sum: i32) -> i16 {
            let m = sum >> IDENT_SHIFT;
            m.clamp(i16::MIN as i32, i16::MAX as i32) as i16
        }

        impl IdentAgg {
            pub const fn new() -> Self {
                Self {
                    ctr: 0,
                    i_sum: 0,
                    i_min: i16::MAX,
                    i_max: i16::MIN,
                    vdiff_sum: 0,
                    duty_sum: 0,
                }
            }

            pub fn sample(&mut self, i: i16, vdiff: i16, duty: i16) -> Option<IdentOut> {
                self.i_sum += i as i32;
                self.i_min = self.i_min.min(i);
                self.i_max = self.i_max.max(i);
                self.vdiff_sum += vdiff as i32;
                self.duty_sum += duty as i32;
                self.ctr += 1;
                if self.ctr < IDENT_WINDOW {
                    return None;
                }
                let out = IdentOut {
                    i_mean_counts: mean_i16(self.i_sum),
                    i_min_counts: self.i_min,
                    i_max_counts: self.i_max,
                    vdiff_mean: mean_i16(self.vdiff_sum),
                    duty_mean_q15: mean_i16(self.duty_sum),
                };
                *self = Self::new();
                Some(out)
            }
        }
    }

    /// One kernel tick as both paths see it: the aggregate folded the
    /// commanded Q15 duty, the stream carries the compare count the bridge
    /// applied for it.
    #[derive(Copy, Clone)]
    struct Tick {
        i: i16,
        vdiff: i16,
        duty_q15: i16,
    }

    fn row(tick: u64, t: Tick) -> TelFrame {
        TelFrame {
            tick,
            current: Some(t.i),
            vdiff: Some(t.vdiff),
            duty_counts: Some(duty_counts(t.duty_q15, ARR)),
            ..Default::default()
        }
    }

    struct Lcg(u64);

    impl Lcg {
        fn next(&mut self) -> u32 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 33) as u32
        }

        fn i16_in(&mut self, lo: i32, hi: i32) -> i16 {
            (lo + (self.next() % (hi - lo + 1) as u32) as i32) as i16
        }
    }

    /// The windows a run of ticks gives on both paths: every one the
    /// aggregate published beside the stream's fold of the same 16 rows.
    fn both(ticks: &[Tick]) -> Vec<(firmware::IdentOut, Window, bool)> {
        let mut agg = firmware::IdentAgg::new();
        let rows: Vec<TelFrame> = ticks
            .iter()
            .enumerate()
            .map(|(k, &t)| row(k as u64, t))
            .collect();
        let mut out = Vec::new();
        for (k, t) in ticks.iter().enumerate() {
            if let Some(o) = agg.sample(t.i, t.vdiff, t.duty_q15) {
                let w = fold(&rows[k + 1 - WINDOW_TICKS as usize..]).expect("whole");
                let start = k + 1 - WINDOW_TICKS as usize;
                let steady = ticks[start..=k]
                    .iter()
                    .all(|u| u.duty_q15 == ticks[start].duty_q15);
                out.push((o, w, steady));
            }
        }
        out
    }

    /// The equivalence: fed the same ticks, the host fold publishes the
    /// aggregate's current and differential bit for bit, and its duty is
    /// the floor mean of the counts the bridge applied. Over a steady
    /// window, the only kind a fit takes, that is the aggregate's mean
    /// quantized as the bridge quantizes it.
    #[test]
    fn the_host_fold_is_the_firmware_aggregate() {
        let mut rng = Lcg(0x5eed);
        let mut ticks = Vec::new();
        for segment in 0..600 {
            let duty_q15 = match segment % 5 {
                0 => 0,
                1 => rng.i16_in(-32767, 32767),
                _ => rng.i16_in(-12000, 12000),
            };
            // a quarter of the segments slew and dip under the limiter
            let dips = segment % 4 == 0;
            for _ in 0..1 + rng.next() % 200 {
                let duty_q15 = if dips && rng.next().is_multiple_of(7) {
                    duty_q15 / 2
                } else {
                    duty_q15
                };
                ticks.push(Tick {
                    i: rng.i16_in(-8190, 8190),
                    vdiff: rng.i16_in(-4095, 4095),
                    duty_q15,
                });
            }
        }
        let pairs = both(&ticks);
        assert!(pairs.len() > 1000, "{} windows", pairs.len());
        let mut steady = 0;
        for (k, (o, w, is_steady)) in pairs.iter().enumerate() {
            assert_eq!(
                (
                    o.i_mean_counts,
                    o.i_min_counts,
                    o.i_max_counts,
                    o.vdiff_mean
                ),
                (w.i_mean, w.i_min, w.i_max, w.vdiff_mean),
                "window {k}"
            );
            let start = k * WINDOW_TICKS as usize;
            let counts: i32 = ticks[start..start + WINDOW_TICKS as usize]
                .iter()
                .map(|t| duty_counts(t.duty_q15, ARR) as i32)
                .sum();
            assert_eq!(w.duty_mean as i32, counts >> WINDOW_SHIFT, "window {k}");
            if *is_steady {
                assert_eq!(w.duty_mean, duty_counts(o.duty_mean_q15, ARR), "window {k}");
                steady += 1;
            }
        }
        assert!(
            steady > 1000 && pairs.len() - steady > 1000,
            "{steady} steady windows"
        );
    }

    /// The edges the aggregate's own tests pinned: the floor toward -inf,
    /// the extremes at full scale, and a window that forgets the last one.
    #[test]
    fn the_fold_floors_and_keeps_the_extremes() {
        let t = |i, vdiff, duty_q15| Tick { i, vdiff, duty_q15 };
        let mut ticks = vec![t(0, 0, 0); 15];
        // 137 is the least Q15 the bridge applies as 5 counts
        ticks.push(t(-5, -5, -137));
        ticks.extend([t(i16::MIN, i16::MIN, -32768); 16]);
        ticks.extend([t(i16::MAX, i16::MAX, 32767); 16]);
        ticks.extend((0..15).map(|n| t(100 + n, 50, 8000)));
        ticks.push(t(-60, 50, 8000));
        for (o, w, _) in both(&ticks) {
            assert_eq!(
                (
                    o.i_mean_counts,
                    o.i_min_counts,
                    o.i_max_counts,
                    o.vdiff_mean
                ),
                (w.i_mean, w.i_min, w.i_max, w.vdiff_mean)
            );
        }
        let w = |k: usize| fold(&both_rows(&ticks)[k * 16..]).unwrap();
        assert_eq!((w(0).i_mean, w(0).vdiff_mean, w(0).duty_mean), (-1, -1, -1));
        assert_eq!(
            (w(1).i_mean, w(1).i_min, w(1).i_max),
            (i16::MIN, i16::MIN, i16::MIN)
        );
        assert_eq!(w(1).duty_mean, -(ARR as i16));
        assert_eq!((w(2).i_mean, w(2).i_max), (i16::MAX, i16::MAX));
        // (100 + .. + 114 - 60) / 16 = 1545 / 16
        assert_eq!((w(3).i_mean, w(3).i_min, w(3).i_max), (96, -60, 114));
        assert_eq!(w(3).duty_mean, duty_counts(8000, ARR));
    }

    fn both_rows(ticks: &[Tick]) -> Vec<TelFrame> {
        ticks
            .iter()
            .enumerate()
            .map(|(k, &t)| row(k as u64, t))
            .collect()
    }

    #[test]
    fn a_broken_or_short_burst_forms_no_window() {
        let t = Tick {
            i: 7,
            vdiff: 9,
            duty_q15: 4000,
        };
        let rows: Vec<TelFrame> = (0..16).map(|k| row(k, t)).collect();
        assert!(fold(&rows).is_some());
        assert_eq!(fold(&rows[..15]), None, "short");
        let mut hole = rows.clone();
        for r in &mut hole[9..] {
            r.tick += 1;
        }
        assert_eq!(fold(&hole), None, "a dropped row");
        let mut bare = rows;
        bare[3].vdiff = None;
        assert_eq!(fold(&bare), None, "a row without the field");
    }

    /// A watch numbers every whole window on the servo clock from the time
    /// its last row arrived, pairs it with that row's pot reading, and keeps
    /// the pot's extremes over every row.
    #[test]
    fn a_watch_folds_every_window_on_the_servo_clock() {
        let t = |k: u64| Tick {
            i: 100 + k as i16,
            vdiff: 2000,
            duty_q15: 8000,
        };
        let rows: Vec<TelFrame> = (0..48)
            .map(|k| TelFrame {
                pos: Some(1000 + k as u16),
                ..row(k, t(k))
            })
            .collect();
        // 20 kHz: a tick is 50 us; the last row lands at tick 20_000
        let w = Watch::fold(&rows, 48, false, 1000.0, 20_000.0, ARR);
        assert_eq!(w.lost, 0);
        assert_eq!(w.pos, Some((1000, 1047)));
        let seq: Vec<u64> = w.windows.iter().map(|x| x.seq).collect();
        assert_eq!(seq, [(20_000 - 32) >> 4, (20_000 - 16) >> 4, 20_000 >> 4]);
        assert_eq!(w.windows[0].t_ms, 1000.0 - 32.0 * 0.05);
        assert_eq!(w.windows[1].pos, Some(1031));
        // (116 + .. + 131) / 16
        assert_eq!(w.windows[1].window.i_mean, 123);
        assert_eq!(w.windows[2].window.duty_mean, duty_counts(8000, ARR));
    }

    /// Rows that never arrived are counted, and a window they break is
    /// left out: a frame lost mid-burst, one lost at the end, a row the
    /// servo dropped.
    #[test]
    fn a_watch_counts_the_rows_it_lacks() {
        let t = Tick {
            i: 7,
            vdiff: 9,
            duty_q15: 4000,
        };
        let rows: Vec<TelFrame> = (0..48).map(|k| row(k, t)).collect();
        let fold = |r: &[TelFrame]| Watch::fold(r, 48, false, 10.0, 20_000.0, ARR);
        let mid: Vec<TelFrame> = rows[..16].iter().chain(&rows[32..]).copied().collect();
        assert_eq!((fold(&mid).lost, fold(&mid).windows.len()), (16, 2));
        assert_eq!(
            (fold(&rows[..32]).lost, fold(&rows[..32]).windows.len()),
            (16, 2)
        );
        let mut dropped = rows.clone();
        for r in &mut dropped[20..] {
            r.tick += 1;
        }
        assert_eq!((fold(&dropped).lost, fold(&dropped).windows.len()), (1, 2));
        assert_eq!(fold(&[]).lost, 48);
    }

    /// A read carries the latest window watched; one with no window since
    /// repeats it, number and all.
    #[test]
    fn a_read_without_a_window_repeats_the_last() {
        let t = Tick {
            i: 300,
            vdiff: -1200,
            duty_q15: -8000,
        };
        let rows: Vec<TelFrame> = (0..32).map(|k| row(k, t)).collect();
        let mut ws = Windows::new(ARR);
        let mut snap = TelemetrySnapshot::default();
        ws.attach(&mut snap);
        assert_eq!(
            (snap.agg_seq, snap.i_mean_counts, snap.pwm_period_ticks),
            (0, 0, ARR)
        );
        ws.watched(&Watch::fold(&rows, 32, false, 800.0, 20_000.0, ARR));
        ws.attach(&mut snap);
        let first = snap;
        assert_eq!(first.agg_seq, 1000);
        assert_eq!((first.i_mean_counts, first.vdiff_mean), (300, -1200));
        assert!(first.duty_is(-8000));
        ws.watched(&Watch::fold(&rows[..9], 32, false, 900.0, 20_000.0, ARR));
        let mut next = TelemetrySnapshot::default();
        ws.attach(&mut next);
        assert_eq!(
            (next.agg_seq, next.i_mean_counts, next.duty_mean_counts),
            (first.agg_seq, 300, first.duty_mean_counts)
        );
    }

    #[test]
    fn whole_windows_fit_the_pause() {
        assert_eq!(watch_rows(2.0, 20_000.0), 32);
        assert_eq!(watch_rows(2.0, 20_100.0), 32);
        assert_eq!(watch_rows(0.7, 20_000.0), 0);
        // 250 ms is 5000 rows, 312 whole windows
        assert_eq!(watch_rows(WATCH_BURST_MS as f64, 20_000.0), 4992);
    }
}
