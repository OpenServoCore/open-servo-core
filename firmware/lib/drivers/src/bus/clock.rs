//! Clock-discipline sub-driver (`docs/osc-native-protocol.md` sec 9.3): MGMT CAL
//! break-train measurement, the differential drift tracker, and the trim-loop
//! gating between them.
//!
//! Pure state machine: the composite owns the ring and deadline providers and
//! drives this with plain ticks, cursors, and ring-derived values; the one
//! deadline the CAL train needs (its watchdog, then the post-train hunt) is
//! returned for the composite's mux to arm.

use super::ring_wrap;
use super::trim::TrimLoop;

/// Trim measurement accept gate (sec 9.3), right-shift of the nominal span
/// (1/16 ~ 6%): wider than any legal clock offset (the HSITRIM throw is
/// +/-3.4%), far under a missed/spurious break or a real inter-frame pause.
/// Gates CAL gaps and drift chain-pairs alike.
pub(super) const TRIM_GATE_SHIFT: u32 = 4;

/// CAL train watchdog, in announced gaps: a train silent this long is
/// abandoned -- no decision -- and the framer resumes (a suspended resolver
/// needs a deadline backstop like any busy-wait).
const CAL_WATCHDOG_GAPS: u32 = 2;

/// Chain-pairs per drift window -- a decision every second or two at
/// hot-loop rates, against thermal drift that moves over minutes. The first
/// window after a restart is the seam baseline (sec 9.3): a power of two, so
/// its mean divides by shift on the divider-less chip build.
const DRIFT_WINDOW_PAIRS: u8 = 128;

/// A window reading past this is not thermal: the loop holds the residual
/// against the anchor near zero, and HSI tempco cannot open thousands of ppm
/// of it within one window - a host seam shift or a garbage window,
/// discarded; the anchor stands, the next CAL re-anchors.
const DRIFT_SANITY_PPM: u32 = 8_000;

/// A live MGMT CAL break train (sec 9.3): the host's crystal spaces the breaks,
/// and break-wake stamps measure that ruler with the local clock. Both
/// stamps of every gap are the SAME ISR flavor, so entry latency cancels in
/// the difference; what survives is clock skew plus sub-us jitter the
/// per-gap gate and the gap sum average out.
struct CalRun {
    gap_ticks: u32,
    gaps_left: u8,
    /// Announced gap count -- the >=-half validity bar at train end.
    total: u8,
    valid: u8,
    last_break: u32,
    err: i32,
    span: u32,
}

/// What the wire delivered between two break stamps (sec 9.3): the drift
/// tracker pairs the stamps only around exactly ONE CRC-verified SILENT
/// instruction -- the one frame shape whose break-to-break span is
/// host-clocked end to end. Anything solicited puts a responder's
/// turnaround (its clock, not the host's) inside the span, and a reply gap
/// can slip under the 1/16 gate at 1M.
#[derive(Copy, Clone)]
enum VerifiedSpan {
    None,
    One { footprint: u16, silent: bool },
    Many,
}

pub struct ClockTracker {
    // MGMT CAL (sec 9.3): a dispatched-but-not-started train (the announce),
    // the live train, and a completed measurement awaiting the main loop.
    pub(super) pending_cal: Option<(u16, u8)>,
    cal: Option<CalRun>,
    cal_ready: bool,
    // Differential drift tracker (sec 9.3): the last break stamp + ring
    // cursor, the one silent instruction verified since it, the seam
    // baseline (mean pair error of the first window after a restart - the
    // CAL anchor), and the current window.
    drift_prev: Option<(u32, u16)>,
    drift_seen: VerifiedSpan,
    drift_base: Option<i32>,
    drift_win_err: i32,
    drift_win_span: u32,
    drift_win_n: u8,
    drift_ready: bool,
    // Completed-measurement handoff to the main loop's `poll_clock_trim`:
    // one (err, span) at a time, owned by whichever ready flag is set.
    cadence_err: i32,
    cadence_span: u32,
    trim: TrimLoop,
}

impl ClockTracker {
    pub const fn new(step_ppm: u32) -> Self {
        Self {
            pending_cal: None,
            cal: None,
            cal_ready: false,
            drift_prev: None,
            drift_seen: VerifiedSpan::None,
            drift_base: None,
            drift_win_err: 0,
            drift_win_span: 0,
            drift_win_n: 0,
            drift_ready: false,
            cadence_err: 0,
            cadence_span: 0,
            trim: TrimLoop::new(step_ppm),
        }
    }

    /// A live or announced CAL train: breaks are ruler marks, not traffic.
    pub fn cal_active(&self) -> bool {
        self.cal.is_some() || self.pending_cal.is_some()
    }

    /// One CAL ruler mark (sec 9.3): a break the composite classified from
    /// ring data (its 0x00 rang fresh), so a wake that rang nothing never
    /// gets here. The stamp is the CALLER's `now`, read at service entry
    /// before any other work -- every gap's two ends then carry the same
    /// entry path, and its latency cancels in the difference. Returns the
    /// framer deadline to arm: the train's watchdog while it runs, the
    /// pend-on-past hunt at its end.
    pub fn on_cal_break(&mut self, now: u32, ticks_per_us: u32) -> Option<u32> {
        if let Some((gap_us, gaps)) = self.pending_cal.take() {
            self.restart();
            // Train start: the first break after the announce opens gap 1.
            let gap_ticks = (gap_us as u32).wrapping_mul(ticks_per_us);
            self.cal = Some(CalRun {
                gap_ticks,
                gaps_left: gaps,
                total: gaps,
                valid: 0,
                last_break: now,
                err: 0,
                span: 0,
            });
            return Some(cal_watchdog_at(now, gap_ticks));
        }
        let (finished, gap_ticks) = {
            let Some(cal) = &mut self.cal else {
                return None; // SAFETY: caller guards; a bare entry changes nothing
            };
            let delta = now.wrapping_sub(cal.last_break);
            cal.last_break = now;
            let err = delta.wrapping_sub(cal.gap_ticks) as i32;
            if err.unsigned_abs() <= cal.gap_ticks >> TRIM_GATE_SHIFT {
                cal.err = cal.err.wrapping_add(err);
                cal.span = cal.span.wrapping_add(cal.gap_ticks);
                cal.valid += 1;
            }
            cal.gaps_left = cal.gaps_left.saturating_sub(1);
            (cal.gaps_left == 0, cal.gap_ticks)
        };
        if !finished {
            return Some(cal_watchdog_at(now, gap_ticks));
        }
        if let Some(c) = self.cal.take() {
            // >= half the announced gaps measured clean, or no decision -- a
            // mangled train yields nothing rather than something.
            if c.valid as u32 * 2 >= c.total as u32 {
                self.cadence_err = c.err;
                self.cadence_span = c.span;
                self.cal_ready = true;
            }
        }
        // Pend-on-past: hunt the train's break bytes off the ring now.
        Some(now)
    }

    /// The watchdog fired: the train (or a dangling announce) dies, no
    /// decision.
    pub fn abandon_cal(&mut self) {
        self.cal = None;
        self.pending_cal = None;
    }

    /// Drift chain-pair (sec 9.3): adjacent break stamps bracketing one
    /// silent verified instruction measure `seam + drift*span` -- the host's
    /// queuing seam is unknown but stationary, so the mean pair error over
    /// the first window after a CAL IS the seam (baseline), and every later
    /// window reads its shift from that one anchor. Anything constant -
    /// seam, detector latch offset - dies in the subtraction; only changes
    /// survive, and the sanity band catches the non-thermal ones. The
    /// steps this tracker applies stay IN the measurement: a window reads
    /// the clock's residual against the anchor, never an increment against
    /// the tracker's own last decision, so window noise cannot integrate
    /// into a walk - a step a noisy window took, the next window reads
    /// back out (DES pin `noisy_silent_flood_holds_the_cal_anchor`;
    /// re-baselining per decision walked the bench trim -1 -> +13 on plain
    /// flood food). `stamp` is the break detector's hardware latch
    /// (transport sec 8): a wake's entry lag never enters a pair.
    pub fn on_drift_break(&mut self, stamp: u32, cursor: u16, len: usize, tpb: u32) {
        let prev = self.drift_prev.replace((stamp, cursor));
        let seen = core::mem::replace(&mut self.drift_seen, VerifiedSpan::None);
        crate::bench::trim_probe(|p| p.stamps += 1);
        let (t1, c1) = match prev {
            Some(pair) => pair,
            None => {
                crate::bench::trim_probe(|p| p.no_prev += 1);
                return;
            }
        };
        let footprint = match seen {
            VerifiedSpan::One {
                footprint,
                silent: true,
            } => footprint,
            VerifiedSpan::One { silent: false, .. } => {
                crate::bench::trim_probe(|p| p.unsilent += 1);
                return;
            }
            VerifiedSpan::None => {
                crate::bench::trim_probe(|p| p.span_none += 1);
                return;
            }
            VerifiedSpan::Many => {
                crate::bench::trim_probe(|p| p.span_many += 1);
                return;
            }
        };
        // Byte-exactness: the pair's ring span must be exactly the verified
        // frame -- anything else intervened (a status, garble, an echo).
        if len == 0 || ring_wrap(cursor as usize + len - c1 as usize, len) as u16 != footprint {
            crate::bench::trim_probe(|p| p.inexact += 1);
            return;
        }
        let span = (footprint as u32).wrapping_mul(tpb);
        let err = stamp.wrapping_sub(t1).wrapping_sub(span) as i32;
        if err.unsigned_abs() > span >> TRIM_GATE_SHIFT {
            crate::bench::trim_probe(|p| p.gated += 1);
            return; // a real pause (inter-chain gap), not a seam
        }
        crate::bench::trim_probe(|p| {
            if self.drift_base.is_none() {
                p.base_pairs += 1;
            } else {
                p.win_pairs += 1;
            }
        });
        let base = self.drift_base.unwrap_or(0);
        self.drift_win_err = self.drift_win_err.wrapping_add(err.wrapping_sub(base));
        self.drift_win_span = self.drift_win_span.saturating_add(span);
        self.drift_win_n += 1;
        if self.drift_win_n < DRIFT_WINDOW_PAIRS {
            return;
        }
        if self.drift_base.is_none() {
            self.drift_base = Some(self.drift_win_err / DRIFT_WINDOW_PAIRS as i32);
        } else if !self.cal_ready && !self.drift_ready {
            crate::bench::trim_probe(|p| {
                p.verdicts += 1;
                p.verdict_err = self.drift_win_err;
                p.verdict_span = self.drift_win_span;
            });
            self.cadence_err = self.drift_win_err;
            self.cadence_span = self.drift_win_span;
            self.drift_ready = true;
        }
        self.drift_win_err = 0;
        self.drift_win_span = 0;
        self.drift_win_n = 0;
    }

    /// The earliest tick a stamp for the break at `cursor` can carry: the
    /// previous stamp plus the ring distance between the two breaks in wire
    /// time, less the gate (the wire delivers no byte in under a byte-time,
    /// and a legal clock offset stays inside the gate). A stamp before it
    /// belongs to no break at `cursor` - an orphan (a parked low's re-fire,
    /// a CRC-failed frame's break) for the caller to skip. None without a
    /// previous stamp: nothing bounds the first.
    pub fn stamp_floor(&self, cursor: u16, len: usize, tpb: u32) -> Option<u32> {
        let (t1, c1) = self.drift_prev?;
        if len == 0 {
            return None;
        }
        let span = (ring_wrap(cursor as usize + len - c1 as usize, len) as u32).wrapping_mul(tpb);
        Some(t1.wrapping_add(span - (span >> TRIM_GATE_SHIFT)))
    }

    /// The drift pair is still open for its one verified frame -- the shape
    /// classification is only worth computing for that first frame.
    pub fn pair_open(&self) -> bool {
        matches!(self.drift_seen, VerifiedSpan::None)
    }

    /// A verified frame between breaks -- the drift tracker's qualification
    /// record. Only exactly-one counts, and only silent shapes pair.
    pub fn note_verified(&mut self, footprint: u16, silent: bool) {
        self.drift_seen = match self.drift_seen {
            VerifiedSpan::None => VerifiedSpan::One { footprint, silent },
            _ => VerifiedSpan::Many,
        };
    }

    /// Tracker restart: baseline, window, and pair continuity all drop --
    /// after a CAL train (the ruler is the new anchor, and the train broke
    /// pair continuity), a rate change, or a rescue. Never after a drift
    /// decision: the baseline IS the anchor the tracker holds the clock to.
    pub fn restart(&mut self) {
        self.drift_prev = None;
        self.drift_seen = VerifiedSpan::None;
        self.drift_base = None;
        self.drift_win_err = 0;
        self.drift_win_span = 0;
        self.drift_win_n = 0;
        self.drift_ready = false;
    }

    /// Drain a completed measurement -- a CAL train (absolute) or a drift
    /// window (anchor-relative) - through the trim loop.
    pub fn poll(&mut self) -> Option<i8> {
        if self.cal_ready {
            crate::bench::trim_probe(|p| p.poll_cal += 1);
            self.cal_ready = false;
            let (err, span) = (self.cadence_err, self.cadence_span);
            self.cadence_err = 0;
            self.cadence_span = 0;
            // The clock is freshly measured either way the decision goes:
            // the next window captures the seam baseline against it.
            self.restart();
            return self.trim.on_cal(err, span);
        }
        if !self.drift_ready {
            return None;
        }
        self.drift_ready = false;
        let (err, span) = (self.cadence_err, self.cadence_span);
        self.cadence_err = 0;
        self.cadence_span = 0;
        if span == 0 {
            return None; // defensive: an empty window decides nothing
        }
        // trunc-ppm > DRIFT_SANITY_PPM <=> |err|*1e6 >= (SANITY+1)*span:
        // multiply-compare only; the quotient itself is bench diagnostics.
        crate::bench::trim_probe(|p| {
            p.poll_drift += 1;
            p.poll_ppm = super::trim::ppm(err, span);
        });
        if err.unsigned_abs() as u64 * 1_000_000 >= (DRIFT_SANITY_PPM as u64 + 1) * span as u64 {
            crate::bench::trim_probe(|p| p.sanity_drop += 1);
            return None; // not thermal -- seam shift or garbage, discarded
        }
        // The anchor stands across the apply: the next window reads the
        // residual this decision left, against the same baseline.
        self.trim.on_drift(err, span)
    }
}

/// The train's silence bound: `on_deadline` reads an expiring framer
/// slot during a live train as "the train died" and abandons it.
fn cal_watchdog_at(now: u32, gap_ticks: u32) -> u32 {
    now.wrapping_add(gap_ticks.wrapping_mul(CAL_WATCHDOG_GAPS))
}
