//! Park: drive the horn back to a centre count open-loop, then torque off,
//! so the next run starts from mid travel instead of a stop. A shaft that
//! comes to rest on the way is blocked: the park gives up, torque off.

use std::sync::atomic::Ordering;
use std::time::Duration;

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::AbortReason;
use osc_ident::exp::seek::{self, STALL_EPS, STALL_POLLS, Watch};
use osc_ident::regs::control;

use super::Aborted;
use super::pump::{STOP, read_snapshot, write_reg};
use crate::sweep::pct_q15;

/// Counts either side of the centre that count as parked; the bench
/// centring script's band.
const PARK_TOL: u16 = 60;
/// Park drive, percent of full scale; the same script's duty.
const PARK_DUTY_PCT: u8 = 15;
/// Poll budget: 300 x `PARK_POLL` bounds the drive at 6 s.
const PARK_POLLS: u32 = 300;
const PARK_POLL: Duration = Duration::from_millis(20);

/// Signed drive toward `center`, or None when `pos` is already within
/// `PARK_TOL` of it.
fn park_duty(pos: u16, center: u16) -> Option<i32> {
    if pos.abs_diff(center) <= PARK_TOL {
        return None;
    }
    let mag = pct_q15(PARK_DUTY_PCT) as i32;
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
pub(crate) fn park(c: &mut Client<NusbPipe>, id: Id, center: u16) -> Result<()> {
    let start = read_snapshot(c, id)?.pos;
    println!("park: pos {start} -> centre {center}");
    let stopped = || STOP.load(Ordering::SeqCst);
    let drove = (|| -> Result<()> {
        let Some(duty) = park_duty(start, center) else {
            return Ok(());
        };
        if stopped() {
            bail!("interrupted");
        }
        write_reg(c, id, control::MODE, 0)?;
        write_reg(c, id, control::TORQUE_ENABLE, 1)?;
        write_reg(c, id, control::GOAL_DUTY, duty)?;
        let mut watch = Watch::new(start, STALL_EPS, STALL_POLLS);
        for _ in 0..PARK_POLLS {
            std::thread::sleep(PARK_POLL);
            if stopped() {
                bail!("interrupted");
            }
            match poll(&mut watch, read_snapshot(c, id)?.pos, center, duty) {
                Ok(true) => break,
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
    let zeroed = write_reg(c, id, control::GOAL_DUTY, 0);
    let off = write_reg(c, id, control::TORQUE_ENABLE, 0);
    drove?;
    zeroed?;
    off
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inside_the_band_does_not_drive() {
        assert_eq!(park_duty(2029, 2029), None);
        assert_eq!(park_duty(2029 - PARK_TOL, 2029), None);
        assert_eq!(park_duty(2029 + PARK_TOL, 2029), None);
    }

    #[test]
    fn outside_the_band_drives_toward_the_centre() {
        // 15% of 32767, the value the bench script wrote
        assert_eq!(park_duty(209, 2029), Some(4915));
        assert_eq!(park_duty(2029 - PARK_TOL - 1, 2029), Some(4915));
        assert_eq!(park_duty(3849, 2029), Some(-4915));
        assert_eq!(park_duty(2029 + PARK_TOL + 1, 2029), Some(-4915));
    }

    /// A shaft that does not move - jammed, or a pot that is not reading -
    /// ends the park within one stillness window; a moving one never does.
    #[test]
    fn park_gives_up_on_a_blocked_shaft() {
        let duty = park_duty(2600, 2029).unwrap();
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
