//! The servo a drive talks to: its table over the bus for everything that
//! moves nothing, and the four moves a drive makes - a write, a telemetry
//! read, a TEL burst, a wait - on the one clock the servo moves on. On the
//! bus that clock is the wall; a test servo keeps its own.

use std::time::{Duration, Instant};

use anyhow::Result;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_ident::frame::{TelFrame, TelemetrySnapshot};
use osc_ident::regs::{Reg, control};

use super::pump::{BurstStats, exchange_tel_burst, read_stamped, write_reg};

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
    /// applied in the instant the burst starts.
    fn stream(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        mask: u16,
    ) -> Result<(Vec<TelFrame>, BurstStats)>;
    fn sleep(&mut self, ms: u32);
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

    fn stream(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        mask: u16,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        (**self).stream(samples, goal, mask)
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

    fn stream(
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
/// permit and TEL off) whether it succeeded, failed, or was ctrl-c'd.
pub(crate) fn guard<S: Servo, T>(s: &mut S, f: impl FnOnce(&mut S) -> Result<T>) -> Result<T> {
    let r = f(s);
    for (reg, v) in SAFE {
        let _ = s.write(reg, v);
    }
    r
}

/// What leaves a servo safe, in the order it is written.
pub(crate) const SAFE: [(Reg, i32); 7] = [
    (control::GOAL_DUTY, 0),
    (control::GOAL_CURRENT, 0),
    (control::GOAL_VELOCITY, 0),
    (control::TORQUE_ENABLE, 0),
    (control::STALL_PERMIT, 0),
    (control::TEL_COUNT, 0),
    (control::TEL_MASK, 0),
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

    /// The in-process servo stack, calibrated and identified like the bench
    /// servo: limit 280, stall yield 168 released under 84, soft limits
    /// 432..3626 inside stops 232..3849, its rail ADC reading `vbus_raw`
    /// (3922 is 7.9 V, 2180 4.39 V).
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
    }

    impl Bench {
        pub(crate) fn mg90(supply: Supply) -> Self {
            let (vbus_raw, vbus) = match supply {
                Supply::TwoS => (3922, 3204),
                Supply::Usb => (2180, 1780),
            };
            let (c, id) = table(vbus_raw);
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

        fn stream(
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
            let stats = BurstStats {
                frames: frames.len().div_ceil(16),
                samples: frames.len(),
                holes: 0,
                garble: 0,
            };
            Ok((frames, stats))
        }

        fn sleep(&mut self, ms: u32) {
            self.servo.advance_ms(ms as f64 * self.bus.pause_scale);
        }
    }
}
