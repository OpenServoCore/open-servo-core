//! What the steps' tests on the simulated servo share: a pipe that logs
//! every write and COMMIT and hands each to a hook, the servo resting on a
//! track, and the bench servo's inputs.

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use futures_lite::future::block_on;
use osc_client::descriptor::Descriptor;
use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
use osc_client::pipe::{Pipe, PipeError};
use osc_client::{BaudRate, Client, Id, Inst, Opcode};

use crate::Board;
use crate::inputs::ThermalInputs;
use crate::nameplate::Nameplate;
use crate::state::field;

const OSC_EXCHANGE: u8 = 0x20;
pub const ID: Id = Id::new(1);

/// A write or a COMMIT as it went on the bus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Sent {
    Write {
        addr: u16,
        data: Vec<u8>,
        hold: bool,
    },
    Commit,
}

pub type Log = Rc<RefCell<Vec<Sent>>>;
pub type OnSent = Box<dyn FnMut(&Sent, &mut FakePipe)>;

pub struct Spy {
    inner: FakePipe,
    log: Log,
    on_sent: OnSent,
}

impl Spy {
    pub fn inner_mut(&mut self) -> &mut FakePipe {
        &mut self.inner
    }
}

impl Pipe for Spy {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        if let [
            _,
            _,
            OSC_EXCHANGE,
            _,
            _,
            _,
            _,
            inst,
            _,
            _,
            _,
            _,
            payload @ ..,
        ] = bytes
        {
            let inst = Inst(*inst);
            let sent = match inst.opcode() {
                Some(Opcode::Write) if payload.len() >= 2 => Some(Sent::Write {
                    addr: u16::from_le_bytes([payload[0], payload[1]]),
                    data: payload[2..].to_vec(),
                    hold: inst.hold(),
                }),
                Some(Opcode::Commit) => Some(Sent::Commit),
                _ => None,
            };
            if let Some(s) = sent {
                self.log.borrow_mut().push(s.clone());
                (self.on_sent)(&s, &mut self.inner);
            }
        }
        self.inner.send(bytes).await
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        self.inner.recv().await
    }

    async fn pause(&mut self, d: Duration) {
        self.inner.pause(d).await
    }
}

pub fn descriptor() -> Descriptor {
    Descriptor::parse(
        &std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../descriptors/osc-servo/0.1.json"
        ))
        .expect("descriptor"),
    )
    .expect("parses")
}

pub fn addr(d: &Descriptor, name: &str) -> u16 {
    field(d, name).expect(name).addr
}

/// The shaft at rest at `pos`, the zero at 1206.
pub fn still_at(pos: u16) -> Vec<TelSample> {
    (0..4096u32).map(|i| rest_row(pos, i)).collect()
}

fn rest_row(pos: u16, i: u32) -> TelSample {
    TelSample {
        tick: TelemetryTick {
            _scan: [0; 10],
            pos,
            vref_raw: 1490,
            vbus_raw: 1241,
            ntc_raw: 2010,
            current_raw: 1206,
            vmotor_a: 773,
            vmotor_b: 772,
            duty_ticks: 0,
            current_trough: 1206,
            i_tick_counts: 0,
            vdiff_counts: 0,
            vmotor_a_trough: 773,
            vmotor_b_trough: 772,
            tick_flags: 0,
            sample_tick: i,
        },
        vbus_counts: 2000,
    }
}

/// The calibrated simulated servo at rest at `pos` on a 3 Mbaud bus.
pub fn servo(pos: u16, on_sent: OnSent) -> (Client<Spy>, Log) {
    let mut inner = FakePipe::new(BaudRate::B3000000, &[1]);
    inner.seed_calibrated(0);
    inner.set_track(0, still_at(pos));
    let log = Log::default();
    let c = block_on(Client::connect(Spy {
        inner,
        log: log.clone(),
        on_sent,
    }))
    .expect("connect");
    (c, log)
}

pub fn quiet() -> OnSent {
    Box::new(|_, _| {})
}

/// A host clock that moves 11 ms a look.
pub fn clock() -> impl Fn() -> f64 {
    let t = Cell::new(0.0);
    move || {
        t.set(t.get() + 0.011);
        t.get()
    }
}

pub fn nameplate() -> Nameplate {
    serde_json::from_str(include_str!("../../testdata/mg90-d/nameplate.json")).expect("nameplate")
}

pub fn board() -> Board {
    serde_json::from_str(include_str!("../../boards/osc-dev-v006.json")).expect("board")
}

pub fn thermal() -> ThermalInputs {
    serde_json::from_str(include_str!("../../testdata/mg90-d/thermal.json")).expect("thermal")
}

/// A brushed motor behind the simulated servo's pot, the least the table
/// step reads: the shaft's turn, a first-order spin-up and coast, a pot
/// that reads fast over one bend, a converter that sticks at chosen
/// carries, and a winding current that rides a ripple line at the
/// commutations' rate. Positions are straight counts, the pot's own where
/// it has no bend.
#[derive(Clone, Debug)]
pub struct Motor {
    /// The free speed at full duty, counts/s.
    pub cps_full: f64,
    /// Commutation cycles a straight count: the ripple line is this times
    /// the speed.
    pub cycles_per_count: f64,
    pub rise_tau_s: f64,
    pub coast_tau_s: f64,
    /// Where the train stops.
    pub stops: (f64, f64),
    /// The bend's middle and half width, and the counts it adds across it.
    pub bend: (f64, f64, f64),
    /// Each carry the converter sticks at, and the codes over it it skips.
    pub wide: Vec<(u16, u16)>,
    /// The current's zero, its level at full duty and the ripple's
    /// amplitude, counts.
    pub zero: f64,
    pub level: f64,
    pub ripple: f64,
    pub pwm_arr: f64,
}

/// Rows of the shaft's old state ahead of a committed duty: the burst
/// opens on the commit's tick, the duty lands a medium tick later.
pub const EDGE_ROWS: usize = 10;
/// Rows a track holds the shaft still after it settles: longer than any
/// wait between two writes, since a track loops.
const HOLD_ROWS: usize = 200_000;
const TRACK_MAX_ROWS: usize = 1_200_000;
const TICK_HZ: f64 = 20_000.0;

impl Motor {
    /// The pot's reading at straight count `x`, before the converter.
    pub fn pot(&self, x: f64) -> f64 {
        let (mid, w, extra) = self.bend;
        x + extra / 2.0 * (1.0 + ((x - mid) / w).tanh())
    }

    /// The code the converter reads at `x`.
    pub fn code(&self, x: f64) -> u16 {
        let c = self.pot(x).floor().clamp(0.0, 4095.0) as u16;
        self.wide
            .iter()
            .find(|(k, s)| (*k..=k + s).contains(&c))
            .map_or(c, |(k, _)| *k)
    }

    pub fn sample(&self, x: f64, q: i16, k: usize) -> TelSample {
        let d = q as f64 / 32768.0;
        let phase = 2.0 * std::f64::consts::PI * self.cycles_per_count * x;
        let i = (self.zero + self.level * d.abs() + self.ripple * phase.sin()).round() as u16;
        let mut s = rest_row(self.code(x), k as u32);
        s.tick.current_raw = i;
        s.tick.current_trough = i;
        s.tick.duty_ticks = (d * self.pwm_arr).round() as i16;
        s.tick.sample_tick = k as u32;
        s
    }

    /// The shaft from `x` at `v` counts/s under duty `q` until it settles
    /// or presses a stop, then held there; `edge` rows of duty `was`
    /// ahead. With `zero_after`, the duty goes to zero that many rows in
    /// and the shaft coasts.
    pub fn run(
        &self,
        x: f64,
        v: f64,
        was: i16,
        q: i16,
        edge: usize,
        zero_after: Option<usize>,
    ) -> (Vec<TelSample>, Vec<(f64, f64)>) {
        let (mut x, mut v) = (x, v);
        let mut rows = Vec::new();
        let mut states = Vec::new();
        for k in 0..edge {
            rows.push(self.sample(x, was, k));
            states.push((x, v));
        }
        let dt = 1.0 / TICK_HZ;
        let mut q = q;
        while rows.len() < TRACK_MAX_ROWS {
            if zero_after.is_some_and(|n| rows.len() >= edge + n) {
                q = 0;
            }
            let target = q as f64 / 32768.0 * self.cps_full;
            let tau = if q != 0 {
                self.rise_tau_s
            } else {
                self.coast_tau_s
            };
            v += (target - v) * dt / tau;
            x += v * dt;
            let (lo, hi) = self.stops;
            let pressed = (x <= lo && target <= 0.0) || (x >= hi && target >= 0.0);
            if x <= lo || x >= hi {
                x = x.clamp(lo, hi);
                v = 0.0;
            }
            rows.push(self.sample(x, q, rows.len()));
            states.push((x, v));
            if (q == 0 && v.abs() < 0.05) || (q != 0 && pressed && zero_after.is_none()) {
                break;
            }
        }
        for _ in 0..HOLD_ROWS {
            rows.push(self.sample(x, q, rows.len()));
            states.push((x, 0.0));
        }
        (rows, states)
    }
}

/// The motor's state on the simulated servo: where the shaft is along the
/// track it plays, and a duty and a burst's rows held for their COMMIT.
pub struct Plant {
    pub motor: Motor,
    states: Vec<(f64, f64)>,
    staged: Option<i16>,
    armed: Option<u16>,
    duty: i16,
}

impl Plant {
    /// Where the shaft is now, straight counts, and its speed.
    pub fn now(&self, pipe: &mut FakePipe) -> (f64, f64) {
        let row = pipe.sim_mut().track_row(0).unwrap_or(0);
        self.states
            .get(row % self.states.len().max(1))
            .copied()
            .unwrap_or((self.motor.stops.0, 0.0))
    }

    fn apply(&mut self, pipe: &mut FakePipe, q: i16, edge: usize, armed: Option<u16>) {
        let (x, v) = self.now(pipe);
        let zero_after = armed.filter(|_| q != 0).map(usize::from);
        let (rows, states) = self.motor.run(x, v, self.duty, q, edge, zero_after);
        self.states = states;
        self.duty = if zero_after.is_some() { 0 } else { q };
        pipe.set_track(0, rows);
    }
}

/// The calibrated simulated servo with `motor` behind its pot, the shaft
/// at rest at straight count `x`: a duty written plain applies at once, a
/// held one at its COMMIT, torque off coasts. A committed drive armed with
/// a burst ends with the burst's rows, as on silicon the host's zero
/// follows the burst's LAST frame: the fake adapter plays each stream's
/// whole window out before the host's next command. `also` sees every write and COMMIT
/// first.
pub fn motor_servo(
    motor: Motor,
    x: f64,
    mut also: OnSent,
) -> (Client<Spy>, Log, Rc<RefCell<Plant>>) {
    let d = descriptor();
    let (goal, torque, chunk0) = (
        addr(&d, "goal_duty"),
        addr(&d, "torque_enable"),
        addr(&d, "chunk0_addr"),
    );
    let (rows, states) = motor.run(x, 0.0, 0, 0, 0, None);
    let plant = Rc::new(RefCell::new(Plant {
        motor,
        states,
        staged: None,
        armed: None,
        duty: 0,
    }));
    let p = plant.clone();
    let on_sent: OnSent = Box::new(move |s, pipe| {
        also(s, pipe);
        let mut p = p.borrow_mut();
        match s {
            Sent::Write { addr, data, hold } if *addr == goal && data.len() == 2 => {
                let q = i16::from_le_bytes([data[0], data[1]]);
                if *hold {
                    p.staged = Some(q);
                } else {
                    p.apply(pipe, q, 0, None);
                }
            }
            Sent::Write {
                addr,
                data,
                hold: true,
            } if *addr == chunk0 && data.len() >= 10 => {
                p.armed = Some(u16::from_le_bytes([data[8], data[9]]));
            }
            Sent::Write { addr, data, .. } if *addr == torque && data[..] == [0] => {
                p.apply(pipe, 0, 0, None)
            }
            Sent::Commit => {
                let armed = p.armed.take();
                if let Some(q) = p.staged.take() {
                    p.apply(pipe, q, EDGE_ROWS, armed);
                }
            }
            _ => {}
        }
    });
    let mut inner = FakePipe::new(BaudRate::B3000000, &[1]);
    inner.seed_calibrated(0);
    inner.set_track(0, rows);
    let log = Log::default();
    let c = block_on(Client::connect(Spy {
        inner,
        log: log.clone(),
        on_sent,
    }))
    .expect("connect");
    (c, log, plant)
}
