//! Clock-discipline sub-driver (`docs/osc-native-protocol.md` sec 9.3): the
//! MGMT CAL break-train ruler and the trim loop it feeds.
//!
//! Pure state machine: the composite owns the deadline provider and drives
//! this with plain ticks; the one deadline the CAL train needs (its
//! watchdog, then the post-train hunt) is returned for the composite's mux
//! to arm.

use super::trim::TrimLoop;

/// CAL per-gap accept gate (sec 9.3), right-shift of the announced gap
/// (1/16 ~ 6%): wider than any legal clock offset (the HSITRIM throw is
/// +/-3.4%), far under a missed or spurious break.
const TRIM_GATE_SHIFT: u32 = 4;

/// CAL train watchdog, in announced gaps: a train silent this long is
/// abandoned -- no decision -- and the framer resumes (a suspended resolver
/// needs a deadline backstop like any busy-wait).
const CAL_WATCHDOG_GAPS: u32 = 2;

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

pub struct ClockDiscipline {
    // MGMT CAL (sec 9.3): a dispatched-but-not-started train (the announce),
    // the live train, and a completed measurement (err, span) awaiting the
    // main loop's `poll_clock_trim`.
    pub(super) pending_cal: Option<(u16, u8)>,
    cal: Option<CalRun>,
    measured: Option<(i32, u32)>,
    trim: TrimLoop,
}

impl ClockDiscipline {
    pub const fn new(step_ppm: u32) -> Self {
        Self {
            pending_cal: None,
            cal: None,
            measured: None,
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
    /// pend-on-past hunt at its end. Out of line: a cold path, which
    /// inlined grew the TIM2 and SysTick vectors by ~200 B each.
    #[cfg_attr(target_arch = "riscv32", inline(never))]
    pub fn on_cal_break(&mut self, now: u32, ticks_per_us: u32) -> Option<u32> {
        if let Some((gap_us, gaps)) = self.pending_cal.take() {
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
                self.measured = Some((c.err, c.span));
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

    /// Drain a completed CAL measurement through the trim loop.
    pub fn poll(&mut self) -> Option<i8> {
        let (err, span) = self.measured.take()?;
        crate::bench::trim_probe(|p| p.poll_cal += 1);
        self.trim.on_cal(err, span)
    }
}

/// The train's silence bound: `on_deadline` reads an expiring framer
/// slot during a live train as "the train died" and abandons it.
fn cal_watchdog_at(now: u32, gap_ticks: u32) -> u32 {
    now.wrapping_add(gap_ticks.wrapping_mul(CAL_WATCHDOG_GAPS))
}

#[cfg(test)]
mod tests;
