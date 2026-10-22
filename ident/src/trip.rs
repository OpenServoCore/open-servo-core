//! The collision trip from a verify recording. `stall_tau_trip_counts`
//! boots off; the kernel folds on a tau_d spike past it (control-theory,
//! the collision), so it must clear the largest tau_d the observer carries
//! through an ordinary move on the identified model. Verify velocity's
//! legs are that move: free travel inside the guard on the written gains.
//! Verify current stalls against the stops on purpose, and a stop's
//! reaction IS a disturbance, so its rows are not a floor.

use std::fmt;

/// The trip over the floor. A spike from a block mid travel carries the
/// loop's whole push; the floor is model error on a free shaft.
pub const TRIP_MARGIN: u16 = 2;
/// Reads a floor needs: fewer leave a leg's transient unseen.
pub const TRIP_MIN_ROWS: usize = 100;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Trip {
    /// The largest |tau_d| over the rows, counts.
    pub floor: u16,
    pub n: usize,
    /// `stall_tau_trip_counts` to write.
    pub trip: u16,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TripRefusal {
    TooFewRows {
        n: usize,
    },
    /// Every read was zero: the observer holds tau_d at zero while the
    /// motor model is unset.
    ModelUnset {
        n: usize,
    },
    /// The margin over the floor leaves no trip under off.
    NoRoom {
        floor: u16,
    },
}

impl fmt::Display for TripRefusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TripRefusal::TooFewRows { n } => {
                write!(
                    f,
                    "{n} reads under torque, {TRIP_MIN_ROWS} needed for a floor"
                )
            }
            TripRefusal::ModelUnset { n } => write!(
                f,
                "tau_d read zero in all {n} reads: the observer holds it at zero while the \
                 motor model is unset, so there is no floor"
            ),
            TripRefusal::NoRoom { floor } => write!(
                f,
                "a floor of {floor} counts times {TRIP_MARGIN} leaves no trip under off"
            ),
        }
    }
}

impl std::error::Error for TripRefusal {}

/// The floor and trip over `tau_d` reads taken while the servo drove.
pub fn trip(tau_d: impl IntoIterator<Item = i16>) -> Result<Trip, TripRefusal> {
    let (mut floor, mut n) = (0u16, 0usize);
    for t in tau_d {
        floor = floor.max(t.unsigned_abs());
        n += 1;
    }
    if n < TRIP_MIN_ROWS {
        return Err(TripRefusal::TooFewRows { n });
    }
    if floor == 0 {
        return Err(TripRefusal::ModelUnset { n });
    }
    match floor.checked_mul(TRIP_MARGIN) {
        Some(t) if t < u16::MAX => Ok(Trip { floor, n, trip: t }),
        _ => Err(TripRefusal::NoRoom { floor }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rows(n: usize, peak: i16) -> Vec<i16> {
        (0..n)
            .map(|i| if i == n / 2 { peak } else { (i % 7) as i16 - 3 })
            .collect()
    }

    #[test]
    fn the_trip_is_the_margin_over_the_largest_magnitude() {
        let t = trip(rows(240, -41)).unwrap();
        assert_eq!(
            t,
            Trip {
                floor: 41,
                n: 240,
                trip: 82
            }
        );
    }

    #[test]
    fn too_few_reads_refuse() {
        assert_eq!(
            trip(rows(TRIP_MIN_ROWS - 1, 40)),
            Err(TripRefusal::TooFewRows {
                n: TRIP_MIN_ROWS - 1
            })
        );
        assert!(trip(rows(TRIP_MIN_ROWS, 40)).is_ok());
    }

    #[test]
    fn an_unset_model_reads_zero_and_refuses() {
        assert_eq!(
            trip(std::iter::repeat_n(0, 300)),
            Err(TripRefusal::ModelUnset { n: 300 })
        );
    }

    #[test]
    fn a_floor_without_room_under_off_refuses() {
        let r = trip(rows(200, i16::MIN));
        assert_eq!(r, Err(TripRefusal::NoRoom { floor: 32768 }));
        let edge = trip(rows(200, 32767)).unwrap();
        assert_eq!(edge.trip, 65534);
    }
}
