//! What a drive's current says about a shaft the pot sees still. A stalled
//! rotor draws its ohmic current for as long as the drive lasts; a turning
//! one sags as its back-EMF rises, within the mechanical rise of tens of
//! ms; an open winding draws nothing. The current is the window means a
//! watch streams, read against a reference: the ohmic current by the
//! winding R the servo carries, else the early windows of the first watch
//! after the step, inside the mechanical rise. The first window after a
//! step can read up to [`OVER_READ`] high, so it is left out and the
//! reference is allowed that much.

use crate::window::Watch;

/// A late current at or over this share of the reference: the rotor is
/// stalled.
pub const STALL_RATIO: f64 = 0.8;
/// At or under this share: it turns.
pub const SAG_RATIO: f64 = 0.6;
/// How far over its current the first watch after a step may read.
pub const OVER_READ: f64 = 1.3;

/// Windows of the first watch after a step left out of the reference, and
/// the windows after them it is the mean of.
pub const BLANK_WINDOWS: usize = 1;
pub const EARLY_WINDOWS: usize = 3;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Felt {
    Stalled,
    Turning,
    /// No current flows.
    Dead,
    Unclear,
}

/// The early reference from the first watch after a step, counts.
pub fn early(w: &Watch) -> Option<f64> {
    mean(
        w.windows
            .get(BLANK_WINDOWS..BLANK_WINDOWS + EARLY_WINDOWS)?,
    )
}

/// A watch's current, counts: the mean over its windows.
pub fn late(w: &Watch) -> Option<f64> {
    mean(&w.windows)
}

fn mean(ws: &[crate::window::Watched]) -> Option<f64> {
    if ws.is_empty() {
        return None;
    }
    let sum: f64 = ws
        .iter()
        .map(|x| x.window.i_mean.unsigned_abs() as f64)
        .sum();
    Some(sum / ws.len() as f64)
}

/// The shaft the pot saw still under a drive whose last watch read `late`:
/// against `ohmic`, the stall current by the stored R, else against
/// `early`. `pinned`: the limiter held the duty under the goal, which on a
/// still shaft is a stall whatever the ratio. Under `i_open` nothing flows.
pub fn feel(late: f64, early: Option<f64>, ohmic: Option<f64>, pinned: bool, i_open: f64) -> Felt {
    if pinned {
        return Felt::Stalled;
    }
    if late < i_open {
        return Felt::Dead;
    }
    let (reference, stall) = match (ohmic, early) {
        (Some(i), _) => (i, STALL_RATIO),
        (None, Some(i)) => (i, STALL_RATIO / OVER_READ),
        (None, None) => return Felt::Unclear,
    };
    if reference <= 0.0 {
        return Felt::Unclear;
    }
    let ratio = late / reference;
    if ratio >= stall {
        Felt::Stalled
    } else if ratio <= SAG_RATIO {
        Felt::Turning
    } else {
        Felt::Unclear
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::bench_mg90;
    use super::*;
    use crate::regs::control;

    /// Felt on the bench MG90 after 60 ms at 15% on 2S, with no R stored:
    /// locked, free with a dead pot, and open, the first watch reading 1.3x
    /// over. The second poll would have missed the turning rotor: by 40 ms
    /// it has sagged most of the way, and against it the late read stays
    /// over the stall ratio.
    #[test]
    fn felt_from_the_early_windows() {
        let felt = |jam: bool, open: bool| {
            let mut s = bench_mg90(3204);
            s.step_over_read = 1.3;
            s.current_limit = None;
            s.pos = 2000.0;
            s.jam = jam.then_some(2000.0);
            s.open = open;
            s.write(control::TORQUE_ENABLE, 1);
            s.write(control::GOAL_DUTY, 4915);
            let first = s.watch_folded(400);
            let second = s.watch_folded(400);
            let third = s.watch_folded(400);
            let l = late(&third).unwrap();
            let got = feel(l, early(&first), None, false, 33.0);
            (got, l / late(&second).unwrap())
        };
        assert_eq!(felt(true, false).0, Felt::Stalled);
        let (turning, vs_second) = felt(false, false);
        assert_eq!(turning, Felt::Turning);
        assert!(vs_second > STALL_RATIO, "{vs_second}");
        assert_eq!(felt(false, true).0, Felt::Dead);
    }

    #[test]
    fn the_stored_r_needs_no_allowance() {
        assert_eq!(feel(260.0, None, Some(300.0), false, 33.0), Felt::Stalled);
        assert_eq!(feel(220.0, None, Some(300.0), false, 33.0), Felt::Unclear);
        assert_eq!(feel(220.0, Some(300.0), None, false, 33.0), Felt::Stalled);
        assert_eq!(feel(120.0, None, Some(300.0), false, 33.0), Felt::Turning);
        assert_eq!(feel(120.0, None, Some(300.0), true, 33.0), Felt::Stalled);
        assert_eq!(feel(120.0, None, None, false, 33.0), Felt::Unclear);
    }
}
