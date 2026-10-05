//! The servo a drive talks to: its table over the bus for everything that
//! moves nothing, and the four moves a drive makes - a write, a telemetry
//! read, a TEL burst, a wait - on the one clock the servo moves on. On the
//! bus that clock is the wall; a test servo keeps its own. Every TEL burst
//! carries the rows the servo dropped from it, read off its table.

use std::time::{Duration, Instant};

use anyhow::Result;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_ident::frame::{TelFrame, TelemetrySnapshot};
use osc_ident::regs::{Reg, control, telemetry};

use super::pump::{BurstStats, exchange_tel_burst, read_stamped, write_reg};
use super::snapshot::read_u16;

pub(crate) trait Servo {
    type P: Pipe;

    /// The table, for reads and writes that move nothing.
    fn client(&mut self) -> &mut Client<Self::P>;
    fn id(&self) -> Id;
    /// Ms on the clock the moves run on.
    fn now_ms(&self) -> f64;
    fn write(&mut self, reg: Reg, value: i32) -> Result<()>;
    /// One telemetry snapshot, stamped `now_ms` at its read.
    fn snapshot(&mut self) -> Result<TelemetrySnapshot>;
    /// One TEL burst of `samples` fast ticks under `mask`, the `goal` write
    /// applied in the instant the burst starts. Drives call [`Servo::stream`].
    fn burst(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        mask: u16,
    ) -> Result<(Vec<TelFrame>, BurstStats)>;
    fn sleep(&mut self, ms: u32);

    /// [`Servo::burst`], its stats carrying the rows the servo dropped from
    /// it: `tel_drop_count` after it less before it. Firmware without the
    /// health block reads 0 there, so it drops none.
    fn stream(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        mask: u16,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        let id = self.id();
        let before = read_u16(self.client(), id, telemetry::TEL_DROP_COUNT)?;
        let (frames, mut stats) = self.burst(samples, goal, mask)?;
        let after = read_u16(self.client(), id, telemetry::TEL_DROP_COUNT)?;
        stats.rows_dropped = after.wrapping_sub(before);
        Ok((frames, stats))
    }
}

impl<S: Servo + ?Sized> Servo for &mut S {
    type P = S::P;

    fn client(&mut self) -> &mut Client<S::P> {
        (**self).client()
    }

    fn id(&self) -> Id {
        (**self).id()
    }

    fn now_ms(&self) -> f64 {
        (**self).now_ms()
    }

    fn write(&mut self, reg: Reg, value: i32) -> Result<()> {
        (**self).write(reg, value)
    }

    fn snapshot(&mut self) -> Result<TelemetrySnapshot> {
        (**self).snapshot()
    }

    fn burst(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        mask: u16,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        (**self).burst(samples, goal, mask)
    }

    fn sleep(&mut self, ms: u32) {
        (**self).sleep(ms)
    }
}

/// A client, owned or borrowed.
pub(crate) trait AsClient {
    type P: Pipe;
    fn get(&mut self) -> &mut Client<Self::P>;
}

impl<P: Pipe> AsClient for Client<P> {
    type P = P;
    fn get(&mut self) -> &mut Client<P> {
        self
    }
}

impl<P: Pipe> AsClient for &mut Client<P> {
    type P = P;
    fn get(&mut self) -> &mut Client<P> {
        self
    }
}

/// The servo on the bus, on the wall clock.
pub(crate) struct Wire<C> {
    c: C,
    id: Id,
    t0: Instant,
}

impl<C: AsClient> Wire<C> {
    pub(crate) fn new(c: C, id: Id) -> Self {
        Self {
            c,
            id,
            t0: Instant::now(),
        }
    }
}

impl<C: AsClient> Servo for Wire<C> {
    type P = C::P;

    fn client(&mut self) -> &mut Client<C::P> {
        self.c.get()
    }

    fn id(&self) -> Id {
        self.id
    }

    fn now_ms(&self) -> f64 {
        self.t0.elapsed().as_secs_f64() * 1000.0
    }

    fn write(&mut self, reg: Reg, value: i32) -> Result<()> {
        write_reg(self.c.get(), self.id, reg, value)
    }

    fn snapshot(&mut self) -> Result<TelemetrySnapshot> {
        read_stamped(self.c.get(), self.id, self.t0)
    }

    fn burst(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        mask: u16,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        exchange_tel_burst(self.c.get(), self.id, samples, goal, mask)
    }

    fn sleep(&mut self, ms: u32) {
        std::thread::sleep(Duration::from_millis(ms as u64));
    }
}

/// Run `f`, then force the servo safe (duty and goals zero, torque, stall
/// permit, TEL and the ident aggregate off) whether it succeeded, failed,
/// or was ctrl-c'd.
pub(crate) fn guard<S: Servo, T>(s: &mut S, f: impl FnOnce(&mut S) -> Result<T>) -> Result<T> {
    let r = f(s);
    for (reg, v) in SAFE {
        let _ = s.write(reg, v);
    }
    r
}

/// What leaves a servo safe, in the order it is written.
pub(crate) const SAFE: [(Reg, i32); 8] = [
    (control::GOAL_DUTY, 0),
    (control::GOAL_CURRENT, 0),
    (control::GOAL_VELOCITY, 0),
    (control::TORQUE_ENABLE, 0),
    (control::STALL_PERMIT, 0),
    (control::TEL_COUNT, 0),
    (control::TEL_MASK, 0),
    (control::IDENT_AGG, 0),
];

/// The bench servo in-process: its table is the in-process servo stack's,
/// its motion osc-ident's test servo, driven on the bench bus's timing.
#[cfg(test)]
pub(crate) mod bench {
    use osc_client::BaudRate;
    use osc_client::fake::{FakePipe, TelSample};
    use osc_ident::exp::testkit::{Bus, FakeServo, bench_mg90};
    use osc_ident::regs::{calib, config};

    use super::*;
    use crate::capture::Supply;
    use crate::sweep::Decay;

    /// The in-process servo stack, calibrated and identified like the bench
    /// servo: limit 280, stall yield 168 released under 84, soft limits
    /// 432..3626 inside stops 232..3849, its rail ADC reading `vbus_raw`
    /// (1961 is 7.9 V, 1090 4.39 V).
    pub(crate) fn table(vbus_raw: u16) -> (Client<FakePipe>, Id) {
        let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
        pipe.seed_calibrated(0);
        let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
        rail(&mut c, vbus_raw);
        for (reg, v) in [
            (config::CURRENT_LIMIT_COUNTS, 280),
            (config::STALL_RESPONSE, 1),
            (config::STALL_YIELD_COUNTS, 168),
            (config::STALL_RELEASE_COUNTS, 84),
            (config::POS_MIN_PHYS_COUNTS, 232),
            (config::POS_MAX_PHYS_COUNTS, 3849),
            (config::POS_MIN_SOFT_COUNTS, 432),
            (config::POS_MAX_SOFT_COUNTS, 3626),
            (calib::RAW_MIN, 232),
            (calib::RAW_MAX, 3849),
        ] {
            set(&mut c, id, reg, v);
        }
        // the sim runs no kernel to publish it
        c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.limits.window_floor_q15 = 4356;
        });
        (c, id)
    }

    /// Every later read of the rail ADC sees `vbus_raw`.
    pub(crate) fn rail(c: &mut Client<FakePipe>, vbus_raw: u16) {
        c.pipe_mut().set_track(
            0,
            vec![TelSample {
                pos: 2029,
                current: 0,
                current_trough: 0,
                duty_q15: 0,
                vdiff: 0,
                vbus: 3204,
                current_raw: 0,
                vmotor_a: 0,
                vmotor_b: 0,
                vbus_raw,
                ntc_raw: 0,
                pos_lin_q4: 0,
                window_valid: false,
                fault: false,
            }],
        );
    }

    pub(crate) fn torque(c: &mut Client<FakePipe>, id: Id) -> u8 {
        c.read(id, control::TORQUE_ENABLE.addr, 1).unwrap()[0]
    }

    pub(crate) fn set(c: &mut Client<FakePipe>, id: Id, reg: Reg, v: i32) {
        write_reg(c, id, reg, v).unwrap();
    }

    /// The bench MG90 on `supply`: its table in the servo stack, its shaft
    /// in the test servo at mid travel, every move charged the bench bus's
    /// time.
    pub(crate) struct Bench {
        pub(crate) c: Client<FakePipe>,
        id: Id,
        pub(crate) servo: FakeServo,
        bus: Bus,
        /// Ticks past a stream's first sample at its goal that the applied
        /// duty last comes to it: the limiter takes it back for the 4
        /// ticks before. 0 never chatters.
        pub(crate) chatter: u64,
        /// A load that comes on late in a long rung: from this goal up,
        /// the limiter holds the applied duty 1000 under the goal from this
        /// many ticks into a stream on. None never does.
        pub(crate) late_load: Option<(i16, usize)>,
        /// Under fast decay the shaft stays still under this duty and the
        /// limiter never holds it: while the decay is fast the breakaway
        /// rises to it and the current limit is lifted. None drives as
        /// under slow decay.
        pub(crate) fast_breakaway_q15: Option<i16>,
        /// The breakaway and current limit a fast decay write set aside.
        slow: Option<(i16, Option<u16>)>,
        decay: u16,
    }

    impl Bench {
        pub(crate) fn mg90(supply: Supply) -> Self {
            let (vbus_raw, vbus) = match supply {
                Supply::TwoS => (1961, 3204),
                Supply::Usb => (1090, 1780),
            };
            let (mut c, id) = table(vbus_raw);
            let d = crate::state::descriptor(&mut c, id).unwrap();
            let decay = crate::descriptor::field(&d, "openloop_decay").unwrap().addr;
            let mut servo = bench_mg90(vbus);
            servo.ends = (232.0, 3849.0);
            servo.pos = 2029.0;
            // the tick rate the table declares
            servo.f_med = 2000.0;
            Self {
                c,
                id,
                servo,
                bus: Bus::BENCH,
                chatter: 0,
                late_load: None,
                fast_breakaway_q15: None,
                slow: None,
                decay,
            }
        }

        /// Charge `txns` transactions of `ms` in all.
        fn busy(&mut self, ms: f64, txns: f64) {
            let lost = txns * self.servo.ticks_lost_per_txn;
            self.servo.busy_ms(ms, lost);
        }
    }

    impl Servo for Bench {
        type P = FakePipe;

        fn client(&mut self) -> &mut Client<FakePipe> {
            &mut self.c
        }

        fn id(&self) -> Id {
            self.id
        }

        fn now_ms(&self) -> f64 {
            self.servo.t_ms
        }

        fn write(&mut self, reg: Reg, value: i32) -> Result<()> {
            let half = self.bus.write_ms / 2.0;
            self.busy(half, 0.5);
            self.servo.write(reg, value);
            if reg.addr == self.decay
                && let Some(q15) = self.fast_breakaway_q15
            {
                let s = &mut self.servo;
                if value == Decay::Fast as i32 {
                    self.slow.get_or_insert((s.breakaway_q15, s.current_limit));
                    (s.breakaway_q15, s.current_limit) = (q15, None);
                } else if let Some(slow) = self.slow.take() {
                    (s.breakaway_q15, s.current_limit) = slow;
                }
            }
            self.busy(half, 0.5);
            Ok(())
        }

        fn snapshot(&mut self) -> Result<TelemetrySnapshot> {
            let half = self.bus.read_ms / 2.0;
            self.busy(half, 1.0);
            let snap = self.servo.read();
            self.busy(half, 1.0);
            Ok(snap)
        }

        /// The rows the test servo drops land in the table's
        /// `tel_drop_count`, where [`Servo::stream`] reads them.
        fn burst(
            &mut self,
            samples: u16,
            goal: Option<(Reg, i32)>,
            _mask: u16,
        ) -> Result<(Vec<TelFrame>, BurstStats)> {
            self.busy(2.0 * self.bus.write_ms, 3.0);
            if let Some((reg, value)) = goal {
                self.servo.write(reg, value);
            }
            let mut frames = Vec::new();
            self.servo.stream(samples, &mut frames);
            let dropped = self.servo.rows_dropped;
            self.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
                let h = &mut t.telemetry.health;
                h.tel_drop_count = h.tel_drop_count.wrapping_add(dropped);
            });
            if let Some((_, goal)) = goal.filter(|_| self.chatter > 0) {
                let goal = goal as i16;
                if let Some(k) = frames.iter().position(|f| f.duty_q15 == Some(goal)) {
                    let last = (k as u64 + self.chatter) as usize;
                    for f in frames.iter_mut().take(last).skip(last.saturating_sub(4)) {
                        f.duty_q15 = Some(goal - goal.signum() * 128);
                    }
                }
            }
            if let Some((from, at)) = self.late_load
                && let Some((_, goal)) = goal
                && goal.unsigned_abs() >= from.unsigned_abs() as u32
            {
                let held = (goal - goal.signum() * 1000) as i16;
                for f in frames.iter_mut().skip(at) {
                    f.duty_q15 = Some(held);
                }
            }
            let stats = BurstStats {
                frames: frames.len().div_ceil(16),
                samples: frames.len(),
                holes: 0,
                garble: 0,
                rows_dropped: 0,
            };
            Ok((frames, stats))
        }

        fn sleep(&mut self, ms: u32) {
            self.servo.advance_ms(ms as f64 * self.bus.pause_scale);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::bench::Bench;
    use super::*;
    use crate::capture::Supply;

    /// A stream reads the rows the servo dropped from it off the table: the
    /// counter's change across it, through its wrap.
    #[test]
    fn a_stream_carries_the_rows_the_servo_dropped_from_it() {
        let mut b = Bench::mg90(Supply::TwoS);
        let (_, st) = b.stream(200, None, 0x1cd).unwrap();
        assert_eq!(st.rows_dropped, 0);
        b.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.health.tel_drop_count = u16::MAX - 1;
        });
        b.servo.rows_dropped = 3;
        let (_, st) = b.stream(200, None, 0x1cd).unwrap();
        assert_eq!(st.rows_dropped, 3);
        let id = b.id();
        assert_eq!(
            read_u16(&mut b.c, id, telemetry::TEL_DROP_COUNT).unwrap(),
            1
        );
    }
}
