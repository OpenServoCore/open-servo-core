//! A committed drive capture: the duty and the TEL burst's arm staged
//! under HOLD and applied by one COMMIT, so the burst opens on the tick of
//! the commit and the duty's edge lands inside it, at the first medium
//! tick after (protocol sec 5.6); the rows ahead of the edge are the
//! shaft at rest. The burst is read to LAST, then the drive ends by a zero
//! write, or by a second committed pair, the zero and a new arm, whose
//! burst opens on the tick the duty goes to zero.
//!
//! Every such drive runs under the short bus dead-man
//! [`DRIVE_DEADMAN_S`], written before the commit, the step's own written
//! back after the zero, as [`super::polarity`]'s pulses do: a streaming
//! burst holds the dead-man, so it runs from the burst's LAST frame and
//! cuts a drive the host stalls in after it. A drive is priced at its rows
//! and that dead-man ([`priced_s`]).

use osc_client::pipe::Pipe;
use osc_servo_core::kernel::phases::loops::faults::BIT_DEADMAN;
use serde_json::{Map, Value};

use super::polarity::STALLED;
use super::{Halt, Rig, Taken, faulted, read_u8, read_u16, write};
use crate::capture::meta;
use crate::capture::rows::Layout;
use crate::capture::tel::{self, Burst, StreamCfg};
use crate::envelope::DRIVE_DEADMAN_S;
use crate::state::field;

/// The time a drive of `rows` rows is priced at, s: its rows and the
/// dead-man after them.
pub fn priced_s(rows: u16, tick_hz: f64) -> f64 {
    rows as f64 / tick_hz + DRIVE_DEADMAN_S
}

/// The dead-men a drive runs under, ticks: the step's own and the drive's.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Deadmen {
    pub step_ticks: u16,
    pub drive_ticks: u16,
}

/// How a drive ends once its burst is read.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum End {
    /// A plain zero-duty write.
    Zero,
    /// The zero and the arm of a burst of this many rows, one commit.
    Coast(u16),
}

/// One committed drive.
pub struct Drive<'a> {
    pub layout: Layout,
    pub duty_q15: i16,
    pub rows: u16,
    pub end: End,
    /// What the bursts' meta carries as their plan.
    pub plan: &'a Value,
}

/// A drive as received: its burst, the coast's, and the host's time from
/// the commit to the zero.
#[derive(Clone, Debug)]
pub struct Driven {
    pub drive: Burst,
    pub coast: Option<Burst>,
    pub on_s: f64,
}

impl Driven {
    /// Why its bursts are not whole, a phrase a reason.
    pub fn lost(&self) -> Vec<String> {
        let mut why = self.drive.lost();
        if let Some(c) = &self.coast {
            why.extend(c.lost().into_iter().map(|w| format!("coast: {w}")));
        }
        why
    }
}

fn keep(
    taken: &mut Vec<Taken>,
    servo: &Map<String, Value>,
    b: &Burst,
    duty: i16,
    plan: &Value,
    t: (f64, f64),
) {
    taken.push(Taken {
        nn: taken.len() as u32 + 1,
        burst: b.clone(),
        cmd_duty_q15: duty,
        meta: meta::tel(servo, b, plan.clone(), t),
    });
}

/// `d` on the servo: the drive's dead-man, the duty and the arm under one
/// COMMIT, the burst read to LAST, the drive's end, the step's dead-man
/// back, then the rows the servo dropped and the faults read. Every burst
/// goes into `taken`, whatever it held. A dead-man latched refuses with
/// [`STALLED`], any other fault aborts.
pub(crate) async fn drive<P: Pipe>(
    rig: &mut Rig<'_, P>,
    stream: &StreamCfg,
    dm: &Deadmen,
    d: &Drive<'_>,
    taken: &mut Vec<Taken>,
    servo: &Map<String, Value>,
) -> Result<Driven, Halt> {
    let goal = field(rig.d, "goal_duty")?.addr;
    let drops0 = read_u16(rig, "tel_drop_count").await?;
    write(rig, "bus_deadman_ticks", &dm.drive_ticks.to_le_bytes()).await?;
    let duty = d.duty_q15.to_le_bytes();
    let t0 = (rig.clock)();
    let mut drive = tel::stream_committed(
        rig.c,
        rig.id,
        rig.d,
        stream,
        d.layout,
        d.rows,
        &[(goal, &duty)],
    )
    .await?;
    let t1 = (rig.clock)();
    let (coast, on_s, t2) = match d.end {
        End::Zero => {
            write(rig, "goal_duty", &[0, 0]).await?;
            let t = (rig.clock)();
            (None, t - t0, t)
        }
        End::Coast(rows) => {
            let t = (rig.clock)();
            let c = tel::stream_committed(
                rig.c,
                rig.id,
                rig.d,
                stream,
                d.layout,
                rows,
                &[(goal, &[0, 0])],
            )
            .await?;
            (Some(c), t - t0, (rig.clock)())
        }
    };
    write(rig, "bus_deadman_ticks", &dm.step_ticks.to_le_bytes()).await?;
    let dropped = read_u16(rig, "tel_drop_count").await?.wrapping_sub(drops0);
    let faults = read_u8(rig, "fault_flags").await?;
    if coast.is_none() {
        drive.dropped = dropped;
    }
    keep(taken, servo, &drive, d.duty_q15, d.plan, (t0, t1));
    let coast = coast.map(|mut c| {
        c.dropped = dropped;
        keep(taken, servo, &c, 0, d.plan, (t1, t2));
        c
    });
    if faults & BIT_DEADMAN != 0 {
        return Err(Halt::Refused(STALLED.into()));
    }
    faulted(rig, taken, &drive, faults).await?;
    if let Some(c) = &coast {
        faulted(rig, taken, c, faults).await?;
    }
    Ok(Driven { drive, coast, on_s })
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicBool;

    use futures_lite::future::block_on;
    use serde_json::json;

    use super::*;
    use crate::sink::MemSink;
    use crate::steps::testkit::{self, ID, Log, OnSent, Sent, addr, descriptor};

    const DM: Deadmen = Deadmen {
        step_ticks: 10_000,
        drive_ticks: 1_000,
    };

    /// `end` driven at +8191 for 1010 rows on the simulated servo, torque
    /// on in OpenLoop at zero duty under the step's dead-man; the log from
    /// the drive's first write.
    fn run(end: End, on_sent: OnSent) -> (Result<Driven, Halt>, Vec<Sent>, Vec<Taken>) {
        let d = descriptor();
        let (mut c, log): (_, Log) = testkit::servo(2041, on_sent);
        let w = |c: &mut osc_client::Client<testkit::Spy>, name: &str, b: &[u8]| {
            block_on(c.write(ID, addr(&d, name), b)).expect(name)
        };
        let open_loop = crate::steps::mode_value(&d, "OpenLoop").expect("mode");
        w(&mut c, "mode", &[open_loop]);
        w(&mut c, "goal_duty", &[0, 0]);
        w(&mut c, "bus_deadman_ticks", &DM.step_ticks.to_le_bytes());
        w(&mut c, "torque_enable", &[1]);
        let from = log.borrow().len();
        let stream = block_on(StreamCfg::read(&mut c, ID, &d)).expect("cfg");
        let mut sink = MemSink::default();
        let clock = testkit::clock();
        let stop = AtomicBool::new(false);
        let mut taken = Vec::new();
        let plan = json!({"rung": "test"});
        let r = {
            let mut rig = Rig {
                c: &mut c,
                id: ID,
                d: &d,
                sink: &mut sink,
                stop: &stop,
                clock: &clock,
                tell: &crate::steps::untold,
            };
            block_on(drive(
                &mut rig,
                &stream,
                &DM,
                &Drive {
                    layout: Layout::Raw,
                    duty_q15: 8191,
                    rows: 1010,
                    end,
                    plan: &plan,
                },
                &mut taken,
                &json!({}).as_object().cloned().unwrap_or_default(),
            ))
        };
        let sent = log.borrow()[from..].to_vec();
        (r, sent, taken)
    }

    fn arm(d: &osc_client::descriptor::Descriptor, rows: u16) -> Sent {
        let mut data = Layout::Raw.chunks().to_le_bytes().to_vec();
        data.extend(rows.to_le_bytes());
        Sent::Write {
            addr: addr(d, "chunk0_addr"),
            data,
            hold: true,
        }
    }

    fn put(d: &osc_client::descriptor::Descriptor, name: &str, data: &[u8], hold: bool) -> Sent {
        Sent::Write {
            addr: addr(d, name),
            data: data.to_vec(),
            hold,
        }
    }

    /// The drive's dead-man, the duty and the arm both held and one COMMIT
    /// carrying them, the burst read whole, the zero, the step's dead-man
    /// back; no duty is written plain before the commit.
    #[test]
    fn a_drive_rides_one_commit_and_ends_on_a_zero_write() {
        let d = descriptor();
        let (r, sent, taken) = run(End::Zero, testkit::quiet());
        let r = r.expect("drives");
        assert_eq!(
            sent,
            [
                put(
                    &d,
                    "bus_deadman_ticks",
                    &DM.drive_ticks.to_le_bytes(),
                    false
                ),
                put(&d, "goal_duty", &8191i16.to_le_bytes(), true),
                arm(&d, 1010),
                Sent::Commit,
                put(&d, "goal_duty", &[0, 0], false),
                put(&d, "bus_deadman_ticks", &DM.step_ticks.to_le_bytes(), false),
            ]
        );
        assert!(r.lost().is_empty(), "{:?}", r.lost());
        assert_eq!(r.drive.rows.len(), 1010);
        assert!(r.coast.is_none() && r.on_s > 0.0);
        assert_eq!(taken.len(), 1);
        assert_eq!(taken[0].cmd_duty_q15, 8191);
    }

    /// A coast end: the zero and a second arm held and committed together,
    /// then the step's dead-man; both bursts kept.
    #[test]
    fn a_coast_end_commits_the_zero_with_its_own_arm() {
        let d = descriptor();
        let (r, sent, taken) = run(End::Coast(200), testkit::quiet());
        let r = r.expect("drives");
        assert_eq!(
            sent,
            [
                put(
                    &d,
                    "bus_deadman_ticks",
                    &DM.drive_ticks.to_le_bytes(),
                    false
                ),
                put(&d, "goal_duty", &8191i16.to_le_bytes(), true),
                arm(&d, 1010),
                Sent::Commit,
                put(&d, "goal_duty", &[0, 0], true),
                arm(&d, 200),
                Sent::Commit,
                put(&d, "bus_deadman_ticks", &DM.step_ticks.to_le_bytes(), false),
            ]
        );
        assert_eq!(r.coast.as_ref().map(|c| c.rows.len()), Some(200));
        assert!(r.lost().is_empty(), "{:?}", r.lost());
        assert_eq!(
            taken.iter().map(|t| t.cmd_duty_q15).collect::<Vec<_>>(),
            [8191, 0]
        );
    }

    /// A dead-man latched in the drive refuses with the stall sentence,
    /// after the zero and the step's dead-man went out; the burst is kept.
    #[test]
    fn a_latched_dead_man_refuses_with_the_stall_sentence() {
        let latch: OnSent = Box::new(|s, pipe| {
            if *s == Sent::Commit {
                pipe.sim_mut()
                    .servo_table_mut(0, |t| t.telemetry.common.fault_flags = BIT_DEADMAN);
            }
        });
        let d = descriptor();
        let (r, sent, taken) = run(End::Zero, latch);
        assert_eq!(r.err(), Some(Halt::Refused(STALLED.into())));
        assert_eq!(
            sent.last(),
            Some(&put(
                &d,
                "bus_deadman_ticks",
                &DM.step_ticks.to_le_bytes(),
                false
            ))
        );
        assert_eq!(taken.len(), 1);
    }

    #[test]
    fn a_drive_is_priced_at_its_rows_and_the_dead_man() {
        assert!((priced_s(1010, 20_000.0) - (0.0505 + DRIVE_DEADMAN_S)).abs() < 1e-12);
    }
}
