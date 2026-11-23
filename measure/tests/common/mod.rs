//! The rig the set-path tests share: the client's simulated servo behind a
//! pipe that logs what goes on the bus, and the mg90-d inputs.

#![allow(dead_code)]

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;
use std::time::Duration;

use futures_lite::future::block_on;
use osc_client::descriptor::Descriptor;
use osc_client::fake::FakePipe;
use osc_client::pipe::{Pipe, PipeError};
use osc_client::{BaudRate, Client, Id, Inst, MgmtOp, Opcode, mgmt};
use osc_measure::capture::rows::TelFrame;
use osc_measure::inputs::{Measured, ShelfRecord, ThermalInputs, Tuning};
use osc_measure::install::{self, Step};
use osc_measure::set::{self, FromRules, SetFile, TableFile, file_sha1};
use osc_measure::sink::{Sink, SinkError};
use osc_measure::{Board, Inputs, Nameplate, Policy, build};
use serde::de::DeserializeOwned;

pub const ID: Id = Id::new(1);
/// CONTROL's first address: below it are CONFIG and CALIB, what a set holds.
pub const CONTROL: u16 = 0x180;
const OSC_EXCHANGE: u8 = 0x20;

pub fn path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

pub fn text(rel: &str) -> String {
    std::fs::read_to_string(path(rel)).unwrap_or_else(|e| panic!("{rel}: {e}"))
}

pub fn load<T: DeserializeOwned>(rel: &str) -> T {
    serde_json::from_str(&text(rel)).unwrap_or_else(|e| panic!("{rel}: {e}"))
}

pub fn descriptor() -> Descriptor {
    Descriptor::parse(&text("../descriptors/osc-servo/0.1.json")).expect("descriptor")
}

pub fn record() -> SetFile {
    SetFile::parse(&text("testdata/mg90-d/constants.json")).expect("the set of record loads")
}

pub fn table() -> TableFile {
    TableFile::parse(&text("testdata/mg90-d/pos_lut.json")).expect("the table loads")
}

pub fn inputs() -> Inputs {
    let mut measured: Measured = load("testdata/mg90-d/measured.json");
    measured.shelf = Some(load::<ShelfRecord>("testdata/mg90-d/shelf.json"));
    Inputs {
        board: load::<Board>("boards/osc-dev-v006.json"),
        measured,
        nameplate: load::<Nameplate>("testdata/mg90-d/nameplate.json"),
        policy: load::<Policy>("testdata/mg90-d/policy.json"),
        thermal: load::<ThermalInputs>("testdata/mg90-d/thermal.json"),
        tuning: Tuning::default(),
    }
}

#[derive(serde::Deserialize)]
struct Slots {
    slots: Vec<u8>,
}

/// The set the rules give for `inputs` over `table`, as the CLI writes it.
pub fn rules_set(inputs: &Inputs, table: &TableFile) -> SetFile {
    let slots: Slots = load("testdata/mg90-d/wide-codes.json");
    let files = vec![(
        "measured".to_string(),
        "measured.json".to_string(),
        file_sha1(text("testdata/mg90-d/measured.json").as_bytes()),
    )];
    let (set, _) = set::from_rules(
        &build(inputs),
        &FromRules {
            pair: "mg90-d__dev-v006",
            nameplate: &inputs.nameplate,
            counts_per_deg: inputs.measured.table.as_ref().map(|t| t.counts_per_deg),
            table: Some(table),
            slots: Some(&slots.slots),
            files: &files,
        },
        &descriptor(),
    )
    .expect("the rules give a set");
    set
}

/// What the bus and the files saw, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Ev {
    Read { addr: u16, len: u16 },
    Write { addr: u16, len: u16 },
    Mgmt(u8),
    File(String),
}

pub type Log = Rc<RefCell<Vec<Ev>>>;

/// The fake adapter, every instruction it sends logged.
pub struct Spy {
    pub inner: FakePipe,
    pub log: Log,
}

impl Pipe for Spy {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        if let [
            _,
            _,
            OSC_EXCHANGE,
            _,
            _,
            _flags,
            _id,
            inst,
            _,
            _,
            _,
            _,
            payload @ ..,
        ] = bytes
        {
            let word = |b: &[u8]| u16::from_le_bytes([b[0], b[1]]);
            let ev = match Inst(*inst).opcode() {
                Some(Opcode::Read) => Some(Ev::Read {
                    addr: word(payload),
                    len: word(&payload[2..]),
                }),
                Some(Opcode::Write) => Some(Ev::Write {
                    addr: word(payload),
                    len: payload.len() as u16 - 2,
                }),
                Some(Opcode::Mgmt) => Some(Ev::Mgmt(payload[0])),
                _ => None,
            };
            assert_ne!(
                ev,
                Some(Ev::Mgmt(MgmtOp::Factory as u8)),
                "the tool sent FACTORY"
            );
            self.log.borrow_mut().extend(ev);
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

pub fn writes_below(log: &Log, end: u16) -> Vec<(u16, u16)> {
    log.borrow()
        .iter()
        .filter_map(|e| match *e {
            Ev::Write { addr, len } if addr < end => Some((addr, len)),
            _ => None,
        })
        .collect()
}

/// A servo as it leaves the calibration bench (the client's seed): SAVEd,
/// stamped, data state clean.
pub fn seeded() -> (Client<Spy>, Log) {
    let mut inner = FakePipe::new(BaudRate::B1000000, &[1]);
    inner.seed_calibrated(0);
    let log = Log::default();
    let c = block_on(Client::connect(Spy {
        inner,
        log: log.clone(),
    }))
    .expect("connect");
    (c, log)
}

/// Every protection field of `set`: what a test names to take a seeded
/// servo onto the set of record.
pub fn every_protection(set: &SetFile) -> Vec<String> {
    set.values()
        .iter()
        .filter(|(n, _)| set::looser(n).is_some())
        .map(|(n, _)| n.to_string())
        .collect()
}

/// A servo that holds the set of record, saved.
pub fn on_record() -> (Client<Spy>, Log) {
    let (mut c, log) = seeded();
    let d = descriptor();
    let rec = record();
    let mut steps = Vec::new();
    block_on(install::install(
        &mut c,
        ID,
        &d,
        &rec,
        Some(&table().points),
        &every_protection(&rec),
        &mut steps,
    ))
    .expect("the set of record installs");
    block_on(mgmt::save(&mut c, ID)).expect("save");
    log.borrow_mut().clear();
    (c, log)
}

pub fn wrote(steps: &[Step]) -> Vec<String> {
    steps
        .iter()
        .filter_map(|s| match s {
            Step::Field { name, .. } => Some(name.clone()),
            Step::Slots { .. } => Some("slots".into()),
            Step::Table { .. } => Some("table".into()),
            _ => None,
        })
        .collect()
}

/// Files kept in memory, each write logged beside the bus.
pub struct LogSink {
    pub files: std::collections::BTreeMap<String, Vec<u8>>,
    pub log: Log,
    pub fail: bool,
}

impl Sink for LogSink {
    fn write(&mut self, path: &str, bytes: &[u8]) -> Result<(), SinkError> {
        if self.fail {
            return Err(SinkError(format!("{path}: the disk is full")));
        }
        self.log.borrow_mut().push(Ev::File(path.to_string()));
        self.files.insert(path.to_string(), bytes.to_vec());
        Ok(())
    }

    fn read(&mut self, path: &str) -> Result<Vec<u8>, SinkError> {
        self.files
            .get(path)
            .cloned()
            .ok_or_else(|| SinkError(format!("{path}: no such file")))
    }
}

/// An osc-lab sweep.csv as its segments: (seg, commanded duty, rows).
pub fn sweep(text: &str) -> Vec<(u32, i16, Vec<TelFrame>)> {
    let mut lines = text.lines();
    let head: Vec<&str> = lines.next().expect("header").split(',').collect();
    let col = |name: &str| head.iter().position(|h| *h == name).expect(name);
    let [seg, cmd, tick, valid, pos, trough, duty, crest, va, vb] = [
        "seg",
        "cmd_duty_q15",
        "tick",
        "window_valid",
        "pos",
        "current_trough",
        "duty_counts",
        "current_raw",
        "vmotor_a",
        "vmotor_b",
    ]
    .map(col);
    let mut out: Vec<(u32, i16, Vec<TelFrame>)> = Vec::new();
    for line in lines {
        let c: Vec<&str> = line.split(',').collect();
        let int = |k: usize| (!c[k].is_empty()).then(|| c[k].parse::<i64>().expect(line));
        let row = TelFrame {
            tick: int(tick).expect("tick") as u64,
            window_valid: int(valid) == Some(1),
            pos: int(pos).map(|v| v as u16),
            current_trough: int(trough).map(|v| v as u16),
            duty_counts: int(duty).map(|v| v as i16),
            current_raw: int(crest).map(|v| v as u16),
            vmotor_a: int(va).map(|v| v as u16),
            vmotor_b: int(vb).map(|v| v as u16),
            ..TelFrame::default()
        };
        let (s, d) = (int(seg).expect("seg") as u32, int(cmd).expect("cmd") as i16);
        match out.last_mut() {
            Some((ls, _, rows)) if *ls == s => rows.push(row),
            _ => out.push((s, d, vec![row])),
        }
    }
    out
}
