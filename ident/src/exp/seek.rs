//! How a seek tells a stop from a blocked shaft. A shaft that comes to rest
//! is at a stop only when it rests where the stop is; one that never
//! travelled, or rests anywhere else, is blocked (or the pot is not
//! reading), and the whole run ends there: a mid-travel jam looks exactly
//! like a stop to a seek that judges by stillness alone.
//!
//! Escalation - raising the duty while the shaft has not moved - exists
//! for two cases. Leaving a stop costs more than holding one: a drive that
//! starts at a stop and points away from it ([`leaves_stop`]) may raise by
//! [`SEEK_STEP_Q15`]. And a sticky spot in the train can hold a shaft the
//! same duty moves everywhere else: a still shaft [`STOP_CLEAR`] or more
//! from both stops ([`clear_of_stops`]) meets nothing but friction, so the
//! jam check and the stop finder raise there too, by their own smaller
//! step, up to their own cap, whichever way they drive. Near a stop,
//! unless leaving it, the duty is fixed: a raise there would meet the
//! stop.

use super::AbortReason;

/// Stillness: a shaft that moves no more than `STALL_EPS` counts a poll
/// over `STALL_POLLS` polls is at rest.
pub const STALL_EPS: u16 = 3;
pub const STALL_POLLS: u32 = 8;

/// Counts of travel before a seek believes it has gone somewhere. A few
/// counts of elastic wind-up at a stop pass any stillness threshold, and a
/// seek that mistakes wind-up for travel declares the stop it is leaning
/// on to be the one it was sent to find (bench: a ladder ran eleven rungs
/// against the wrong stop that way).
pub const SEEK_TRAVEL_MIN: u16 = 100;

/// Breakout escalation per stalled window while leaving a stop: 5% of full
/// scale.
pub const SEEK_STEP_Q15: i16 = 1638;

/// How close to a pot stop a shaft at rest counts as at it, counts.
pub const STOP_TOL: u16 = 150;

/// The stop a drive of sign `dir` ends at: positive duty moves the pot up.
pub fn stop_for(dir: i8, stops: (u16, u16)) -> u16 {
    if dir > 0 { stops.1 } else { stops.0 }
}

/// A drive of sign `dir` from `pos` leaves a stop: it starts within
/// [`STOP_TOL`] of one and points away from it. Unknown stops never allow
/// it.
pub fn leaves_stop(pos: u16, dir: i8, stops: Option<(u16, u16)>) -> bool {
    stops.is_some_and(|s| pos.abs_diff(stop_for(-dir, s)) <= STOP_TOL)
}

/// How far from both stops a still shaft must be for a raised duty to meet
/// nothing but the mechanism's own friction, counts.
pub const STOP_CLEAR: u16 = 300;

/// `pos` is [`STOP_CLEAR`] or more from both stops. With the stops unknown
/// nothing says otherwise.
pub fn clear_of_stops(pos: u16, stops: Option<(u16, u16)>) -> bool {
    stops.is_none_or(|(lo, hi)| {
        pos >= lo.saturating_add(STOP_CLEAR) && pos.saturating_add(STOP_CLEAR) <= hi
    })
}

/// A seek of sign `dir` that started at `start` has come to rest at `pos`.
/// With the stops known it must rest within [`STOP_TOL`] of the one it was
/// driving at (already being there counts); with them unknown it must have
/// travelled [`SEEK_TRAVEL_MIN`]. Anything else is a blocked shaft.
pub fn at_stop(
    start: u16,
    pos: u16,
    dir: i8,
    stops: Option<(u16, u16)>,
) -> Result<(), AbortReason> {
    let arrived = match stops {
        Some(s) => pos.abs_diff(stop_for(dir, s)) <= STOP_TOL,
        None => pos.abs_diff(start) >= SEEK_TRAVEL_MIN,
    };
    if arrived {
        Ok(())
    } else {
        Err(blocked(start, pos))
    }
}

pub fn blocked(start: u16, pos: u16) -> AbortReason {
    AbortReason::Blocked {
        pos,
        moved: pos.abs_diff(start),
    }
}

/// A drive watched for travel, window by window: a shaft whose position
/// spans at most `stall_eps` x `stall_polls` counts over `stall_polls` polls
/// is still. The span over the window, not stillness poll to poll - a
/// couple of counts of ADC jitter at a rail reset a consecutive count
/// forever - and not the net move either: a shaft coasting the wrong way
/// when the drive starts turns round inside a window and nets nothing.
#[derive(Copy, Clone, Debug)]
pub struct Watch {
    start: u16,
    span: (u16, u16),
    polls: u32,
    window: u32,
    drift: u16,
}

impl Watch {
    pub fn new(start: u16, stall_eps: u16, stall_polls: u32) -> Self {
        Self {
            start,
            span: (start, start),
            polls: 0,
            window: stall_polls.max(1),
            drift: stall_eps.saturating_mul(stall_polls as u16),
        }
    }

    pub fn start(&self) -> u16 {
        self.start
    }

    /// True once a whole window passed without the shaft moving.
    pub fn still(&mut self, pos: u16) -> bool {
        self.polls += 1;
        self.span = (self.span.0.min(pos), self.span.1.max(pos));
        if !self.polls.is_multiple_of(self.window) {
            return false;
        }
        let still = self.span.1 - self.span.0 <= self.drift;
        self.span = (pos, pos);
        still
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STOPS: Option<(u16, u16)> = Some((209, 3849));

    #[test]
    fn a_mid_travel_rest_is_not_a_stop() {
        // a mid-travel jam: 2048 is nowhere near either stop
        assert_eq!(
            at_stop(2051, 2048, 1, STOPS),
            Err(AbortReason::Blocked {
                pos: 2048,
                moved: 3
            })
        );
        // travelled a long way, still short of the stop
        assert!(at_stop(2051, 3000, 1, STOPS).is_err());
        assert_eq!(at_stop(2051, 3800, 1, STOPS), Ok(()));
        assert_eq!(at_stop(2051, 250, -1, STOPS), Ok(()));
        // the wrong stop is no stop
        assert!(at_stop(2051, 250, 1, STOPS).is_err());
        // already there
        assert_eq!(at_stop(3840, 3845, 1, STOPS), Ok(()));
        // unknown stops: travel is all there is to judge by
        assert!(at_stop(2051, 2048, 1, None).is_err());
        assert_eq!(at_stop(2051, 3000, 1, None), Ok(()));
    }

    #[test]
    fn clear_of_the_stops_is_300_counts_from_both() {
        assert!(clear_of_stops(2048, STOPS));
        assert!(clear_of_stops(509, STOPS) && clear_of_stops(3549, STOPS));
        assert!(!clear_of_stops(508, STOPS) && !clear_of_stops(3550, STOPS));
        assert!(clear_of_stops(100, None));
    }

    #[test]
    fn only_a_drive_off_a_stop_leaves_it() {
        assert!(leaves_stop(220, 1, STOPS));
        assert!(!leaves_stop(220, -1, STOPS), "toward the stop it sits at");
        assert!(leaves_stop(3800, -1, STOPS));
        assert!(!leaves_stop(3800, 1, STOPS));
        assert!(!leaves_stop(2048, 1, STOPS) && !leaves_stop(2048, -1, STOPS));
        assert!(!leaves_stop(220, 1, None));
    }

    #[test]
    fn stillness_is_net_travel_over_a_window() {
        let mut w = Watch::new(2048, STALL_EPS, STALL_POLLS);
        // jitter inside the window's drift never counts as travel
        let still: Vec<bool> = (0..8).map(|k| w.still(2048 + (k % 2) * 3)).collect();
        assert_eq!(
            still,
            [false, false, false, false, false, false, false, true]
        );
        let moving: Vec<bool> = (1..=8).map(|k| w.still(2051 + k * 4)).collect();
        assert!(!moving[7], "32 counts over the window is travel");
        // coasting up, turning round, driven back down: nets nothing
        let turning: Vec<bool> = [10, 25, 35, 38, 35, 25, 10, 0]
            .iter()
            .map(|d| w.still(2083 + d))
            .collect();
        assert!(!turning[7], "a turn inside the window is not a rest");
    }
}
