//! Park: drive the horn back to a centre count open-loop, brake it to rest,
//! then torque off, so the next run starts from a still shaft at mid travel
//! instead of a stop. It drives at the caller's seek duty and never raises
//! it: a shaft that comes to rest on the way is blocked, and the park gives
//! up, torque off.

use std::sync::atomic::Ordering;

use anyhow::{Result, bail};
use osc_ident::exp::AbortReason;
use osc_ident::exp::seek::{self, STALL_EPS, STALL_POLLS, Watch};
use osc_ident::regs::control;

use super::Aborted;
use super::pump::{Lease, STOP};
use super::servo::Servo;
use crate::sweep::brake_to_rest;

/// Counts either side of the centre that count as parked; the bench
/// centring script's band.
const PARK_TOL: u16 = 60;
/// Poll budget: 300 x `PARK_POLL_MS` bounds the drive at 6 s.
const PARK_POLLS: u32 = 300;
const PARK_POLL_MS: u32 = 20;

/// Signed drive of `duty_q15` toward `center`, or None when `pos` is
/// already within `PARK_TOL` of it.
fn park_duty(pos: u16, center: u16, duty_q15: i16) -> Option<i32> {
    if pos.abs_diff(center) <= PARK_TOL {
        return None;
    }
    let mag = duty_q15 as i32;
    Some(if pos < center { mag } else { -mag })
}

/// Done once within `PARK_TOL` short of the centre or anywhere past it: the
/// drive never reverses, so an overshoot ends the park instead of hunting.
fn arrived(pos: u16, center: u16, duty: i32) -> bool {
    (pos as i32 - center as i32) * duty.signum() >= -(PARK_TOL as i32)
}

/// One poll of the drive: Ok(true) arrived, Ok(false) still going, Err on a
/// shaft at rest short of the centre.
fn poll(watch: &mut Watch, pos: u16, center: u16, duty: i32) -> Result<bool, AbortReason> {
    if arrived(pos, center, duty) {
        return Ok(true);
    }
    if watch.still(pos) {
        return Err(seek::blocked(watch.start(), pos));
    }
    Ok(false)
}

/// Re-centre the horn, then zero the duty and torque off whatever happened.
/// Reaching the poll budget short of the band is not an error: the shaft is
/// left where it got to.
pub(crate) fn park<S: Servo>(s: &mut S, center: u16, duty_q15: i16) -> Result<()> {
    let start = s.snapshot()?.pos;
    println!("park: pos {start} -> centre {center}");
    let stopped = || STOP.load(Ordering::SeqCst);
    let drove = (|| -> Result<()> {
        let Some(duty) = park_duty(start, center, duty_q15) else {
            return Ok(());
        };
        if stopped() {
            bail!("interrupted");
        }
        s.write(control::MODE, 0)?;
        s.write(control::TORQUE_ENABLE, 1)?;
        s.write(control::GOAL_DUTY, duty)?;
        let mut watch = Watch::new(start, STALL_EPS, STALL_POLLS);
        for _ in 0..PARK_POLLS {
            s.sleep(PARK_POLL_MS);
            if stopped() {
                bail!("interrupted");
            }
            match poll(&mut watch, s.snapshot()?.pos, center, duty) {
                // braked, not left to coast: the jam check that may follow
                // judges a shaft that moves as one its duty moved
                Ok(true) => return brake_to_rest(s, &mut Lease::new(false), duty.signum() as i8),
                Ok(false) => {}
                Err(reason) => {
                    return Err(Aborted {
                        what: "park",
                        reason,
                    }
                    .into());
                }
            }
        }
        Ok(())
    })();
    let zeroed = s.write(control::GOAL_DUTY, 0);
    let off = s.write(control::TORQUE_ENABLE, 0);
    drove?;
    zeroed?;
    off
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench servo's seek by its stored R: 15% of 32767.
    const SEEK: i16 = 4915;

    #[test]
    fn inside_the_band_does_not_drive() {
        assert_eq!(park_duty(2029, 2029, SEEK), None);
        assert_eq!(park_duty(2029 - PARK_TOL, 2029, SEEK), None);
        assert_eq!(park_duty(2029 + PARK_TOL, 2029, SEEK), None);
    }

    #[test]
    fn outside_the_band_drives_toward_the_centre() {
        assert_eq!(park_duty(209, 2029, SEEK), Some(4915));
        assert_eq!(park_duty(2029 - PARK_TOL - 1, 2029, SEEK), Some(4915));
        assert_eq!(park_duty(3849, 2029, SEEK), Some(-4915));
        assert_eq!(park_duty(2029 + PARK_TOL + 1, 2029, SEEK), Some(-4915));
        assert_eq!(park_duty(3849, 2029, 5242), Some(-5242), "the duty given");
    }

    /// A shaft that does not move - jammed, or a pot that is not reading -
    /// ends the park within one stillness window; a moving one never does.
    #[test]
    fn park_gives_up_on_a_blocked_shaft() {
        let duty = park_duty(2600, 2029, SEEK).unwrap();
        let mut watch = Watch::new(2600, STALL_EPS, STALL_POLLS);
        let polls: Vec<_> = (0..STALL_POLLS)
            .map(|k| poll(&mut watch, 2600 + (k % 2) as u16, 2029, duty))
            .collect();
        assert!(
            polls[..STALL_POLLS as usize - 1]
                .iter()
                .all(|p| *p == Ok(false))
        );
        assert_eq!(
            polls.last(),
            Some(&Err(AbortReason::Blocked {
                pos: 2601,
                moved: 1
            }))
        );
        let msg = Aborted {
            what: "park",
            reason: AbortReason::Blocked {
                pos: 2601,
                moved: 1,
            },
        }
        .to_string();
        assert_eq!(
            msg,
            "park aborted: the shaft is blocked or the position sensor is not reading (pos 2601, \
             moved 1 counts); a blocked shaft cannot be centred, so torque is off and the shaft \
             stays where it stopped"
        );

        let mut watch = Watch::new(2600, STALL_EPS, STALL_POLLS);
        let mut pos = 2600;
        let mut done = false;
        for _ in 0..100 {
            pos -= 15;
            match poll(&mut watch, pos, 2029, duty) {
                Ok(true) => {
                    done = true;
                    break;
                }
                Ok(false) => {}
                Err(e) => panic!("a moving shaft read as blocked: {e}"),
            }
        }
        assert!(done && pos.abs_diff(2029) <= PARK_TOL);
    }

    #[test]
    fn arrives_at_the_near_band_edge_or_past_the_centre() {
        // driving up: short of the band keeps going, the band edge and any
        // overshoot stop
        assert!(!arrived(2029 - PARK_TOL - 1, 2029, 4915));
        assert!(arrived(2029 - PARK_TOL, 2029, 4915));
        assert!(arrived(3000, 2029, 4915));
        // driving down mirrors it
        assert!(!arrived(2029 + PARK_TOL + 1, 2029, -4915));
        assert!(arrived(2029 + PARK_TOL, 2029, -4915));
        assert!(arrived(1000, 2029, -4915));
    }
}
