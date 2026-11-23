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
    (0..4096u32)
        .map(|i| TelSample {
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
        })
        .collect()
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
