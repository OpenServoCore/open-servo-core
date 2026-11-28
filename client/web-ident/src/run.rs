//! The CLI's `run` verb over any pipe, into memory: the arguments admitted
//! before the bus is touched, the servo reached and named, [`of::run`],
//! and the reboot that clears the unsaved bit after it, every line told to
//! the feed as it is said. The files are [`of::run`]'s own, so the run
//! folder is the CLI's byte for byte. [`resume`] is `run --resume` on the
//! same terms, the folder's files given, and both tell the page each file
//! as the run writes or moves it.

use std::cell::RefCell;
use std::future::Future;
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;

use anyhow::{Context, Result, bail};
use osc_client::descriptor::{Descriptor, Registry, Selection};
use osc_client::pipe::Pipe;
use osc_client::{BaudRate, Client, Error, Id, mgmt};
use osc_measure::run::steps::{StepInputs, clear_unsaved};
use osc_measure::run::{self as of, Host, Progress, RunArgs, Store, servo_name};
use osc_measure::sink::MemSink;
use serde::Serialize;

/// The table layouts this module knows: the CLI's, from the checked-in
/// export.
const DESCRIPTORS: &[&str] = &[include_str!("../../../descriptors/osc-servo/0.1.json")];

/// The name a run's records carry.
pub const TOOL: &str = concat!("osc-client-web-ident ", env!("CARGO_PKG_VERSION"));

/// What a page gives a run: the CLI's arguments, and the files they name
/// by their paths in the store.
#[derive(Clone, Debug)]
pub struct Given {
    pub id: u8,
    pub nameplate: String,
    pub policy: String,
    pub tries: u32,
    /// `None` finds the bus rate, as the CLI's `--baud auto`.
    pub baud: Option<BaudRate>,
    pub files: Vec<(String, Vec<u8>)>,
}

/// What a run leaves.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Outcome {
    /// Every file the run wrote, by its path from the store's root
    /// (`<uid>/<UTC>/run.json`), in path order.
    pub files: Vec<(String, Vec<u8>)>,
    /// Every line told to the feed, in order.
    pub lines: Vec<String>,
    /// Whether the tune accepted a set.
    pub accepted: bool,
    /// Why the verb ended short, as the CLI prints it after `refused: `.
    pub refused: Option<String>,
}

impl Outcome {
    /// The run folder `<uid>/<UTC>`, once one was named.
    pub fn root(&self) -> Option<String> {
        let (path, _) = self.files.first()?;
        let mut parts = path.splitn(3, '/');
        Some(format!("{}/{}", parts.next()?, parts.next()?))
    }
}

/// The host a page runs on: its feed, its clock, its stop flag, and where
/// each file goes as the run stores it.
pub struct Page<'a> {
    pub feed: &'a dyn Fn(&Progress),
    /// Unix seconds.
    pub clock: &'a dyn Fn() -> f64,
    pub stop: &'a AtomicBool,
    pub store: &'a dyn Fn(Stored<'_>),
}

/// A change to the run's store, told as it happens, by paths from the
/// store's root: replayed in order onto the files the run was given, they
/// give the run folder the outcome holds.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum Stored<'a> {
    /// `bytes`, whole, as the file at `path`, over one that is there.
    Write {
        path: &'a str,
        #[serde(with = "serde_bytes")]
        bytes: &'a [u8],
    },
    /// The file or the folder at `from`, whole, to `to`.
    Move { from: &'a str, to: &'a str },
}

/// The store of a page that keeps no mirror.
pub fn unmirrored(_: Stored<'_>) {}

/// A memory store that tells each write and move once it is done. Reads
/// answer from memory at once, as a run needs.
struct Mirror<'a> {
    files: MemSink,
    tell: &'a dyn Fn(Stored<'_>),
}

impl Store for Mirror<'_> {
    fn read_file(&mut self, path: &Path) -> std::result::Result<Vec<u8>, String> {
        self.files.read_file(path)
    }

    fn write_file(&mut self, path: &Path, bytes: &[u8]) -> std::result::Result<(), String> {
        self.files.write_file(path, bytes)?;
        (self.tell)(Stored::Write {
            path: &path.display().to_string(),
            bytes,
        });
        Ok(())
    }

    fn make_dirs(&mut self, path: &Path) -> std::result::Result<(), String> {
        self.files.make_dirs(path)
    }

    fn exists(&self, path: &Path) -> bool {
        self.files.exists(path)
    }

    fn entries(&self, path: &Path) -> std::result::Result<Vec<(String, bool)>, String> {
        self.files.entries(path)
    }

    fn canonical(&self, path: &Path) -> std::result::Result<PathBuf, String> {
        self.files.canonical(path)
    }

    fn rename(&mut self, from: &Path, to: &Path) -> std::result::Result<(), String> {
        self.files.rename(from, to)?;
        (self.tell)(Stored::Move {
            from: &from.display().to_string(),
            to: &to.display().to_string(),
        });
        Ok(())
    }
}

/// The run on the pipe `open` yields: nothing opened when the arguments
/// are refused. The client comes back where one was connected, for the
/// caller to close its device.
pub async fn run<P: Pipe>(
    open: impl Future<Output = Result<P>>,
    given: &Given,
    page: &Page<'_>,
) -> (Outcome, Option<Client<P>>) {
    session(open, given, None, page).await
}

/// The CLI's `run --resume` of the run folder `folder` (`<uid>/<UTC>`),
/// whose files are among `given.files`, on the pipe `open` yields: the
/// tune's tries are the folder's, and every refusal the folder's files
/// give comes before the pipe is opened, the folder untouched. The
/// outcome's files are the folder whole, as the resume leaves it.
pub async fn resume<P: Pipe>(
    open: impl Future<Output = Result<P>>,
    given: &Given,
    folder: &str,
    page: &Page<'_>,
) -> (Outcome, Option<Client<P>>) {
    session(open, given, Some(folder), page).await
}

/// What a resume of `folder` would do, decided from `given.files` alone
/// at `now`, as [`resume`] decides it before the pipe is opened; the
/// refusal in the tool's words where it would refuse.
pub fn plan(given: &Given, folder: &str, now: f64) -> Result<of::resume::Plan> {
    let mut store = MemSink::default();
    for (path, bytes) in &given.files {
        store.files.insert(key(path), bytes.clone());
    }
    let a = RunArgs {
        nameplate: Path::new(&given.nameplate),
        policy: Path::new(&given.policy),
        out: Path::new(folder),
        tries: folder_tries(&mut store, folder),
    };
    let inputs = of::admit(&mut store, &a, &mut Vec::new())?;
    of::resume::plan(&mut store, Path::new(folder), &a, &inputs, now)
}

/// The tune's tries a run folder records, as the CLI's `--resume` takes
/// them.
fn folder_tries(store: &mut MemSink, folder: &str) -> u32 {
    store
        .read_file(&Path::new(folder).join("run.json"))
        .ok()
        .and_then(|b| serde_json::from_slice::<serde_json::Value>(&b).ok())
        .and_then(|d| d["given"]["tries"].as_u64())
        .map_or(osc_measure::tune::TRIES, |t| t as u32)
}

async fn session<P: Pipe>(
    open: impl Future<Output = Result<P>>,
    given: &Given,
    folder: Option<&str>,
    page: &Page<'_>,
) -> (Outcome, Option<Client<P>>) {
    let said = RefCell::new(Vec::new());
    let feed = |p: &Progress| {
        if let Progress::Lines { lines } = p {
            said.borrow_mut().extend(lines.iter().cloned());
        }
        (page.feed)(p);
    };
    let tell = |lines: Vec<String>| {
        if !lines.is_empty() {
            feed(&Progress::Lines { lines });
        }
    };
    let mut store = Mirror {
        files: MemSink::default(),
        tell: page.store,
    };
    for (path, bytes) in &given.files {
        store.files.files.insert(key(path), bytes.clone());
    }
    let tries = match folder {
        Some(f) => folder_tries(&mut store.files, f),
        None => given.tries,
    };
    let a = RunArgs {
        nameplate: Path::new(&given.nameplate),
        policy: Path::new(&given.policy),
        out: Path::new(folder.unwrap_or("")),
        tries,
    };
    let mut notes = Vec::new();
    let admitted = of::admit(&mut store, &a, &mut notes);
    tell(notes);
    let ended = |refused: Option<String>, accepted: bool, store: &MemSink| Outcome {
        files: match folder {
            Some(f) => under(store, f),
            None => written(store, given),
        },
        lines: said.borrow().clone(),
        accepted,
        refused,
    };
    let inputs = match admitted {
        Ok(i) => i,
        Err(e) => return (ended(Some(format!("{e:#}")), false, &store.files), None),
    };
    if let Some(f) = folder
        && let Err(e) = of::resume::plan(&mut store, Path::new(f), &a, &inputs, (page.clock)())
    {
        return (ended(Some(format!("{e:#}")), false, &store.files), None);
    }
    let (mut c, d) = match connect(open, given, &tell).await {
        Ok(cd) => cd,
        Err(e) => return (ended(Some(format!("{e:#}")), false, &store.files), None),
    };
    let ran = on_bus(
        &mut c, &d, given, &a, &inputs, folder, &mut store, &feed, page,
    )
    .await;
    let (accepted, refused) = match ran {
        Ok((accepted, more)) => {
            tell(more);
            (accepted, None)
        }
        Err((e, more)) => {
            tell(more);
            (false, Some(format!("{e:#}")))
        }
    };
    (ended(refused, accepted, &store.files), Some(c))
}

fn key(path: &str) -> String {
    Path::new(path).display().to_string()
}

/// The files of `store` the run wrote: every one but the inputs given.
fn written(store: &MemSink, given: &Given) -> Vec<(String, Vec<u8>)> {
    let inputs: Vec<String> = given.files.iter().map(|(p, _)| key(p)).collect();
    store
        .files
        .iter()
        .filter(|(k, _)| !inputs.contains(k))
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect()
}

/// The files of `store` in the run folder `folder`.
fn under(store: &MemSink, folder: &str) -> Vec<(String, Vec<u8>)> {
    let head = format!("{}/", key(folder));
    store
        .files
        .iter()
        .filter(|(k, _)| k.starts_with(&head))
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect()
}

/// The CLI's `connect`: the session, the client's resend on, the bus rate
/// found or set, the servo pinged (once more on a timeout) and its
/// descriptor chosen.
async fn connect<P: Pipe>(
    open: impl Future<Output = Result<P>>,
    given: &Given,
    tell: &dyn Fn(Vec<String>),
) -> Result<(Client<P>, Descriptor)> {
    let mut c = Client::connect(open.await?).await?;
    c.set_resend(true);
    match given.baud {
        None => match mgmt::find_bus_baud(&mut c).await? {
            Some(rate) => tell(vec![format!("bus at {} baud", rate.as_hz())]),
            None => bail!("no servo answers at any rate"),
        },
        Some(rate) => c.host_baud(rate).await?,
    }
    let id = Id::new(given.id);
    let ping = match c.ping(id).await {
        Err(Error::Timeout { .. }) => c.ping(id).await,
        p => p,
    }
    .with_context(|| format!("servo {} does not answer", id.as_byte()))?;
    let mut reg = Registry::new();
    for s in DESCRIPTORS {
        reg.push(Descriptor::parse(s)?);
    }
    let d = match reg.select(ping.model, ping.fw) {
        Selection::Exact(d) | Selection::OlderMinor(d) => d.clone(),
        _ => bail!(
            "no descriptor of this tool is the servo's (model {:#06x}, fw {:#06x})",
            ping.model,
            ping.fw
        ),
    };
    Ok((c, d))
}

type Ran = std::result::Result<(bool, Vec<String>), (anyhow::Error, Vec<String>)>;

/// The CLI's `run` verb past `connect`: the servo named, its state read,
/// the run (the resume of `folder` where one is given), then the reboot
/// that clears unsaved; the lines of that reboot with the outcome, as the
/// CLI's `ended` joins them.
#[allow(clippy::too_many_arguments)]
async fn on_bus<P: Pipe>(
    c: &mut Client<P>,
    d: &Descriptor,
    given: &Given,
    a: &RunArgs<'_>,
    inputs: &StepInputs,
    folder: Option<&str>,
    store: &mut Mirror<'_>,
    feed: &dyn Fn(&Progress),
    page: &Page<'_>,
) -> Ran {
    let id = Id::new(given.id);
    let named = async {
        let servo = servo_name(c, id).await?;
        let before = osc_measure::state::read(c, id, d).await?;
        anyhow::Ok((servo, before))
    };
    let (servo, before) = named.await.map_err(|e| (e, Vec::new()))?;
    let mut host = Host {
        files: store as &mut dyn Store,
        clock: page.clock,
        stop: page.stop,
        feed,
        tool: TOOL,
    };
    let ran = match folder {
        Some(f) => of::resume::resume(c, id, d, &mut host, a, inputs, Path::new(f), &servo).await,
        None => of::run(c, id, d, &mut host, a, inputs, &servo).await,
    };
    let cleared = clear_unsaved(c, id, d, &before).await;
    match (ran, cleared) {
        (Ok((_, accepted)), Ok(more)) => Ok((accepted, more)),
        (Ok(_), Err(e)) => Err((
            e.context("the command ended, but the reboot to clear unsaved"),
            Vec::new(),
        )),
        (Err(e), Ok(more)) => Err((e, more)),
        (Err(e), Err(r)) => Err((e, vec![format!("the reboot to clear unsaved: {r:#}")])),
    }
}

#[cfg(all(test, feature = "fake"))]
mod tests {
    use std::cell::Cell;
    use std::sync::atomic::Ordering;

    use futures_lite::future::block_on;
    use osc_client::fake::{FakePipe, uid_of};
    use osc_client::mgmt::Uid;
    use serde_json::Value;

    use super::*;

    const ID: u8 = 1;

    fn testdata(name: &str) -> Vec<u8> {
        let p = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../measure/testdata/mg90-d")
            .join(name);
        std::fs::read(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
    }

    /// The mg90-d profile, its policy and its heat model, where a page keeps
    /// them.
    fn given() -> Given {
        Given {
            id: ID,
            nameplate: "profiles/mg90-d/nameplate.json".into(),
            policy: "profiles/mg90-d/policy.json".into(),
            tries: osc_measure::tune::TRIES,
            baud: None,
            files: ["nameplate.json", "policy.json", "thermal.json"]
                .iter()
                .map(|f| (format!("profiles/mg90-d/{f}"), testdata(f)))
                .collect(),
        }
    }

    /// A clock that reads a quarter second later each time, from the same
    /// instant on every run.
    fn clock() -> impl Fn() -> f64 {
        let t = Cell::new(1_764_000_000.0);
        move || {
            let now = t.get();
            t.set(now + 0.25);
            now
        }
    }

    async fn opened() -> Result<FakePipe> {
        Ok(crate::fake::pipe(ID))
    }

    /// The entry on the simulated servo with a feed that keeps what it is
    /// told, raising the stop flag as the step `stop_at` starts.
    fn entry(stop_at: Option<&str>) -> (Outcome, Vec<Progress>, Client<FakePipe>) {
        let told = RefCell::new(Vec::new());
        let stop = AtomicBool::new(false);
        let feed = |p: &Progress| {
            if matches!(p, Progress::Started { step } if Some(*step) == stop_at) {
                stop.store(true, Ordering::SeqCst);
            }
            told.borrow_mut().push(p.clone());
        };
        let clock = clock();
        let page = Page {
            feed: &feed,
            clock: &clock,
            stop: &stop,
            store: &unmirrored,
        };
        let (o, c) = block_on(run(opened(), &given(), &page));
        (o, told.into_inner(), c.expect("connected"))
    }

    fn manifest(o: &Outcome) -> Value {
        let (_, bytes) = o
            .files
            .iter()
            .find(|(p, _)| p.ends_with("/run.json"))
            .expect("run.json");
        serde_json::from_slice(bytes).expect("json")
    }

    /// The CLI's `run` verb as `main` runs it, step for step, into a memory
    /// store holding the same files: the files `osc-ident`'s parity test
    /// pins to the CLI's disk, byte for byte.
    fn cli_verb() -> MemSink {
        let g = given();
        let mut store = MemSink::default();
        for (p, b) in &g.files {
            store.files.insert(p.clone(), b.clone());
        }
        let a = RunArgs {
            nameplate: Path::new(&g.nameplate),
            policy: Path::new(&g.policy),
            out: Path::new(""),
            tries: g.tries,
        };
        let inputs = of::admit(&mut store, &a, &mut Vec::new()).expect("admitted");
        block_on(async {
            let mut c = Client::connect(crate::fake::pipe(ID))
                .await
                .expect("connect");
            c.set_resend(true);
            mgmt::find_bus_baud(&mut c)
                .await
                .expect("baud")
                .expect("found");
            let ping = c.ping(Id::new(ID)).await.expect("ping");
            let mut reg = Registry::new();
            reg.push(Descriptor::parse(DESCRIPTORS[0]).expect("descriptor"));
            let Selection::Exact(d) = reg.select(ping.model, ping.fw) else {
                panic!("the descriptor is the simulated servo's");
            };
            let d = d.clone();
            let servo = servo_name(&mut c, Id::new(ID)).await.expect("named");
            let before = osc_measure::state::read(&mut c, Id::new(ID), &d)
                .await
                .expect("state");
            let stop = AtomicBool::new(false);
            let clock = clock();
            let mut host = Host {
                files: &mut store,
                clock: &clock,
                stop: &stop,
                feed: &of::quiet,
                tool: TOOL,
            };
            of::run(&mut c, Id::new(ID), &d, &mut host, &a, &inputs, &servo)
                .await
                .expect("runs");
            clear_unsaved(&mut c, Id::new(ID), &d, &before)
                .await
                .expect("clears");
        });
        store
    }

    /// The simulated servo runs no shunt burst, so the run stops at the
    /// probe: the entry hands back the files the CLI's run verb writes, path
    /// for path and byte for byte, in the servo's folder under its UID, and
    /// no input given among them.
    #[test]
    fn the_entry_writes_what_the_cli_run_verb_writes() {
        let (o, _, _) = entry(None);
        assert_eq!(o.refused, None, "{:?}", o.lines);
        assert!(!o.accepted);
        let cli: Vec<(String, Vec<u8>)> = written(&cli_verb(), &given());
        let names: Vec<&str> = o.files.iter().map(|(n, _)| n.as_str()).collect();
        for f in ["run.json", "inputs/board.json", "results/probe.json"] {
            assert!(names.iter().any(|n| n.ends_with(f)), "{f}: {names:?}");
        }
        assert!(names.iter().any(|n| n.contains("/captures/rest/")));
        assert!(!names.iter().any(|n| n.starts_with("profiles/")));
        assert_eq!(o.files, cli, "the same files");
        let uid = format!("{:?}", Uid(uid_of(ID)));
        let root = o.root().expect("a run folder");
        assert!(root.starts_with(&format!("{uid}/")), "{root}");
        assert!(names.iter().all(|n| n.starts_with(&format!("{root}/"))));
        let m = manifest(&o);
        assert_eq!(m["tool"], TOOL, "{m}");
        assert_eq!(m["steps"]["probe"]["state"], "aborted", "{m}");
        assert_eq!(m["gather"]["state"], "not run", "{m}");
    }

    /// The feed is told each step's start and end as `run.json` records
    /// them, and every line in order: the files read, the bus found, the
    /// steps', and last the reboot that clears unsaved; the outcome's lines
    /// are those.
    #[test]
    fn the_feed_is_told_each_step_and_every_line() {
        let (o, told, _) = entry(None);
        let ends: Vec<(&str, &str)> = told
            .iter()
            .filter_map(|p| match p {
                Progress::Ended { step, state, .. } => Some((*step, state.as_str())),
                _ => None,
            })
            .collect();
        assert_eq!(
            ends,
            [
                ("rest", "accepted"),
                ("probe", "aborted"),
                ("gather", "not run"),
                ("tune", "not run"),
                ("cruise", "not run"),
                ("final", "not run")
            ]
        );
        let Progress::Lines { lines: read } = &told[0] else {
            panic!("the files read are told first: {:?}", told[0]);
        };
        assert!(
            read.last()
                .is_some_and(|l| l.starts_with("heat model: from profiles/mg90-d/thermal.json")),
            "{read:?}"
        );
        assert_eq!(
            told[1],
            Progress::Lines {
                lines: vec!["bus at 3000000 baud".into()]
            }
        );
        let lines: Vec<String> = told
            .iter()
            .filter_map(|p| match p {
                Progress::Lines { lines } => Some(lines.clone()),
                _ => None,
            })
            .flatten()
            .collect();
        assert_eq!(lines, o.lines);
        let last = o.lines.last().expect("a line");
        assert!(
            last.starts_with("rebooted to clear unsaved")
                && last.ends_with("now unsaved no, 0 differences"),
            "{last}"
        );
        assert_eq!(
            told.last(),
            Some(&Progress::Lines {
                lines: vec![last.clone()]
            })
        );
        let mut step = "";
        let mut units = Vec::new();
        for p in &told {
            match p {
                Progress::Started { step: s } => step = s,
                Progress::Ended { .. } => step = "",
                Progress::Unit {
                    step: s, at, of, ..
                } => {
                    assert_eq!(*s, step, "a unit between its step's start and end");
                    units.push((*s, *at, *of));
                }
                _ => {}
            }
        }
        assert_eq!(units, [("rest", 1, 2), ("rest", 2, 2), ("probe", 1, 17)]);
    }

    /// A page has no command line: the run's lines name no command of the
    /// CLI's, and the run that stops at the probe says so without the one
    /// that runs it again.
    #[test]
    fn the_lines_name_no_command() {
        let (o, _, _) = entry(None);
        assert!(
            !o.lines.iter().any(|l| l.contains("osc-ident --id")),
            "{:?}",
            o.lines
        );
        let stops = "the run stops: not every step was accepted, so nothing is gathered or tuned";
        assert!(o.lines.iter().any(|l| l == stops), "{:?}", o.lines);
    }

    /// The stop raised as the probe starts, as Cancel raises it: the probe
    /// ends through its way out, torque off and read back, `run.json`
    /// records it interrupted, and the run stops there with its files kept.
    #[test]
    fn a_stop_ends_the_run_through_its_way_out() {
        let (o, told, mut c) = entry(Some("probe"));
        let m = manifest(&o);
        let why = "interrupted: the step stopped between rungs";
        assert_eq!(m["steps"]["rest"]["state"], "accepted", "{m}");
        assert_eq!(m["steps"]["probe"]["state"], "interrupted", "{m}");
        assert_eq!(m["steps"]["probe"]["why"], why, "{m}");
        assert!(told.contains(&Progress::Ended {
            step: "probe",
            state: "interrupted".into(),
            why: Some(why.into()),
        }));
        let out = o
            .lines
            .iter()
            .rposition(|l| l == "the way out:")
            .expect("the probe's way out");
        assert!(
            o.lines[out..]
                .iter()
                .any(|l| l == "  torque_enable 0, read back 0"),
            "{:?}",
            &o.lines[out..]
        );
        let d = Descriptor::parse(DESCRIPTORS[0]).expect("descriptor");
        let torque = d.field("torque_enable").expect("torque").addr;
        assert_eq!(block_on(c.read(Id::new(ID), torque, 1)).expect("read"), [0]);
        assert_eq!(o.refused, None);
    }

    /// A change as a test keeps it: a write as `(path, None, bytes)`, a
    /// move as `(from, Some(to), [])`.
    type Op = (String, Option<String>, Vec<u8>);
    type Ops = RefCell<Vec<Op>>;
    type Files = Vec<(String, Vec<u8>)>;

    /// A page's store callback that keeps each change.
    fn recorder(ops: &Ops) -> impl Fn(Stored<'_>) + '_ {
        move |s| {
            ops.borrow_mut().push(match s {
                Stored::Write { path, bytes } => (path.into(), None, bytes.to_vec()),
                Stored::Move { from, to } => (from.into(), Some(to.into()), Vec::new()),
            });
        }
    }

    /// The changes replayed in order onto `files`, as a page mirrors them.
    fn replay(files: &[(String, Vec<u8>)], ops: &[Op]) -> Files {
        let mut m = MemSink::default();
        m.files.extend(files.iter().cloned());
        for (path, to, bytes) in ops {
            match to {
                None => m.write_file(Path::new(path), bytes).expect("write"),
                Some(to) => m.rename(Path::new(path), Path::new(to)).expect("move"),
            }
        }
        m.files.into_iter().collect()
    }

    /// The run on the simulated servo from the clock at `t0`, its store's
    /// changes kept, the stop raised as the step `stop_at` starts.
    fn mirrored(
        stop_at: Option<&str>,
        resume_of: Option<(&str, &Files)>,
        t0: f64,
    ) -> (Outcome, Vec<Op>, Vec<Progress>) {
        let ops = Ops::default();
        let told = RefCell::new(Vec::new());
        let stop = AtomicBool::new(false);
        let feed = |p: &Progress| {
            if matches!(p, Progress::Started { step } if Some(*step) == stop_at) {
                stop.store(true, Ordering::SeqCst);
            }
            told.borrow_mut().push(p.clone());
        };
        let t = Cell::new(t0);
        let clock = || {
            let now = t.get();
            t.set(now + 0.25);
            now
        };
        let (o, _) = {
            let store = recorder(&ops);
            let page = Page {
                feed: &feed,
                clock: &clock,
                stop: &stop,
                store: &store,
            };
            match resume_of {
                None => block_on(run(opened(), &given(), &page)),
                Some((folder, files)) => {
                    let mut g = given();
                    g.files.extend(files.iter().cloned());
                    block_on(resume(opened(), &g, folder, &page))
                }
            }
        };
        (o, ops.into_inner(), told.into_inner())
    }

    /// Each file goes to the page as the run writes it: the changes
    /// replayed into an empty store are the outcome's files, and `run.json`
    /// records a step accepted only once that step's result was told.
    #[test]
    fn the_store_tells_each_file_as_it_is_written() {
        let (o, ops, _) = mirrored(None, None, T0);
        assert!(!ops.is_empty());
        assert_eq!(replay(&[], &ops), o.files);
        let root = o.root().expect("a run folder");
        let result = format!("{root}/results/rest.json");
        let told_result = ops.iter().position(|(p, _, _)| *p == result).expect("told");
        let accepted = ops
            .iter()
            .position(|(p, _, b)| {
                p.ends_with("/run.json")
                    && serde_json::from_slice::<Value>(b)
                        .is_ok_and(|m| m["steps"]["rest"]["state"] == "accepted")
            })
            .expect("run.json records the rest accepted");
        assert!(told_result < accepted, "{told_result} {accepted}");
    }

    const T0: f64 = 1_764_000_000.0;

    /// The run folder's files with the host clock's stamps zeroed, the
    /// segments' records and their files left out: what a stopped and
    /// resumed run shares with one that ran unbroken.
    fn unsegmented(files: &[(String, Vec<u8>)]) -> Files {
        fn mask(v: &mut Value) {
            match v {
                Value::Object(o) => {
                    o.remove("segments");
                    o.remove("segment");
                    for (k, x) in o.iter_mut() {
                        if ["host_t_start", "host_t_end", "started", "ended"].contains(&k.as_str())
                            && x.is_number()
                        {
                            *x = 0.into();
                        } else {
                            mask(x);
                        }
                    }
                }
                Value::Array(a) => a.iter_mut().for_each(mask),
                _ => {}
            }
        }
        files
            .iter()
            .filter(|(p, _)| !p.contains("/segments/"))
            .map(|(p, b)| match serde_json::from_slice::<Value>(b) {
                Ok(mut v) if p.ends_with(".json") => {
                    mask(&mut v);
                    (p.clone(), serde_json::to_vec_pretty(&v).expect("json"))
                }
                _ => (p.clone(), b.clone()),
            })
            .collect()
    }

    /// The run stopped in the probe, as Cancel stops it, then resumed
    /// from the files the page mirrored: the rest stays the first
    /// segment's, the stopped probe's result moves under `segments/1/`, the
    /// probe runs again whole, and the folder ends as an unbroken run's
    /// does, the host's stamps, the segments and the probe's readings of
    /// the sim's noise aside; the resume's changes replayed onto the folder
    /// give the outcome's files.
    #[test]
    fn a_run_stopped_in_the_probe_resumes_from_its_mirror() {
        let (whole, _, _) = mirrored(None, None, T0);
        let (stopped, ops, _) = mirrored(Some("probe"), None, T0);
        let root = stopped.root().expect("a run folder");
        assert_eq!(manifest(&stopped)["steps"]["probe"]["state"], "interrupted");
        let folder = replay(&[], &ops);
        assert_eq!(folder, stopped.files);

        let (resumed, more, told) = mirrored(None, Some((&root, &folder)), T0 + 900.0);
        assert_eq!(resumed.refused, None, "{:#?}", resumed.lines);
        assert_eq!(resumed.root().as_deref(), Some(root.as_str()));
        assert_eq!(replay(&folder, &more), resumed.files);
        let m = manifest(&resumed);
        assert_eq!(m["segments"][1]["from"], "probe", "{m}");
        assert_eq!(m["steps"]["rest"]["segment"], 1, "{m}");
        assert_eq!(m["steps"]["probe"]["segment"], 2, "{m}");
        assert_eq!(m["steps"]["probe"]["state"], "aborted", "{m}");
        let names: Vec<&str> = resumed.files.iter().map(|(p, _)| p.as_str()).collect();
        let moved = format!("{root}/segments/1/results/probe.json");
        assert!(names.contains(&moved.as_str()), "{names:#?}");
        assert!(
            more.iter()
                .any(|(f, to, _)| *f == format!("{root}/results/probe.json")
                    && to.as_deref() == Some(moved.as_str())),
            "the move is told"
        );
        let started: Vec<&str> = told
            .iter()
            .filter_map(|p| match p {
                Progress::Started { step } => Some(*step),
                _ => None,
            })
            .collect();
        assert_eq!(started, ["resume", "probe"]);
        // The probe ran again later in sim time, so its readings of the sim's
        // noise move by a count or so; every other byte is the unbroken run's.
        let probe = format!("{root}/results/probe.json");
        let readings = [
            ("rail_v", 5.0 / 1241.0 * 1.5),
            ("position_before_counts", 1.5),
            ("position_after_counts", 1.5),
        ];
        let read = |files: &mut Files| -> Value {
            let at = files.iter().position(|(p, _)| *p == probe).expect("probe");
            let mut v: Value = serde_json::from_slice(&files[at].1).expect("json");
            let o = v.as_object_mut().expect("an object");
            let kept = readings
                .iter()
                .map(|(k, _)| (k.to_string(), o.remove(*k).unwrap_or_default()))
                .collect();
            files[at].1 = serde_json::to_vec(&v).expect("json");
            kept
        };
        let (mut again, mut once) = (unsegmented(&resumed.files), unsegmented(&whole.files));
        let (r, w) = (read(&mut again), read(&mut once));
        for (k, within) in readings {
            let (r, w) = (r[k].as_f64().expect(k), w[k].as_f64().expect(k));
            assert!((r - w).abs() <= within, "{k}: {r} against {w}");
        }
        assert_eq!(again, once);
    }

    /// Cancel as the run's first step starts, as a page's Cancel before the
    /// run gets going does, then Resume right after on mg90-d's own heat
    /// model: the rest step left no result, so the resume goes on from it
    /// in segment 2, ends its own step accepted, and waits for the winding
    /// as a restart into the probe does, minutes the sim pauses through.
    #[test]
    fn a_run_stopped_in_its_first_step_resumes_from_the_rest_step() {
        let (stopped, ops, _) = mirrored(Some("rest"), None, T0);
        let root = stopped.root().expect("a run folder");
        let m = manifest(&stopped);
        assert_eq!(m["steps"]["rest"]["state"], "interrupted", "{m}");
        let folder = replay(&[], &ops);
        assert!(
            !folder
                .iter()
                .any(|(p, _)| p.ends_with("/results/rest.json")),
            "the rest step wrote no result"
        );
        let ended = m["segments"][0]["ended"].as_f64().expect("ended");

        let (resumed, more, told) = mirrored(None, Some((&root, &folder)), ended + 1.0);
        assert_eq!(resumed.refused, None, "{:#?}", resumed.lines);
        assert_eq!(replay(&folder, &more), resumed.files);
        let ends: Vec<(&str, &str)> = told
            .iter()
            .filter_map(|p| match p {
                Progress::Ended { step, state, .. } => Some((*step, state.as_str())),
                _ => None,
            })
            .collect();
        assert_eq!(ends[..2], [("resume", "accepted"), ("rest", "accepted")]);
        let m = manifest(&resumed);
        let s = &m["segments"][1];
        assert_eq!((&s["n"], &s["from"]), (&2.into(), &"rest".into()), "{m}");
        let cooled = s["cooled_s"].as_f64().expect("cooled");
        // the resume reads the clock a beat or so after it is handed
        let (most, least) = (
            of::resume::cooling_s(128.92, 1.0),
            of::resume::cooling_s(128.92, 2.0),
        );
        assert!(
            cooled > 60.0 && (least..=most).contains(&cooled),
            "{cooled} not in {least}..{most}"
        );
        assert_eq!(m["steps"]["rest"]["segment"], 2, "{m}");
        assert_eq!(m["steps"]["rest"]["state"], "accepted", "{m}");
        assert_eq!(m["steps"]["probe"]["segment"], 2, "{m}");
        let rest = format!("{root}/results/rest.json");
        assert!(resumed.files.iter().any(|(p, _)| *p == rest));
    }

    /// The folder's own refusals come before the pipe is opened, in the
    /// tool's words, the folder untouched; `plan` says the same.
    #[test]
    fn a_resume_the_folder_refuses_opens_no_pipe() {
        let (whole, _, _) = mirrored(None, None, T0);
        let root = whole.root().expect("a run folder");
        let mut g = given();
        let mut folder = whole.files.clone();
        folder.retain(|(p, _)| !p.ends_with("/run.json"));
        g.files.extend(folder);
        let words = format!("{root}: no run.json: not a run folder");
        let e = plan(&g, &root, T0).expect_err("refused");
        assert_eq!(format!("{e:#}"), words);
        let stop = AtomicBool::new(false);
        let clock = clock();
        let ops = Ops::default();
        let store = recorder(&ops);
        let page = Page {
            feed: &of::quiet,
            clock: &clock,
            stop: &stop,
            store: &store,
        };
        let opened = Cell::new(false);
        let open = async {
            opened.set(true);
            Ok(crate::fake::pipe(ID))
        };
        let (o, c) = block_on(resume(open, &g, &root, &page));
        assert!(!opened.get() && c.is_none());
        assert_eq!(o.refused.as_deref(), Some(words.as_str()));
        assert!(ops.borrow().is_empty(), "nothing written");

        let mut g = given();
        g.files.extend(whole.files.iter().cloned());
        let p = plan(&g, &root, T0 + 900.0).expect("plans");
        assert_eq!(p.from, "probe");
        assert_eq!(p.n, 2);
    }

    /// Arguments the CLI refuses before the bus are refused here before
    /// the pipe is opened: no session, no file written.
    #[test]
    fn a_refused_argument_opens_no_pipe() {
        let mut g = given();
        g.tries = 0;
        let stop = AtomicBool::new(false);
        let clock = clock();
        let page = Page {
            feed: &of::quiet,
            clock: &clock,
            stop: &stop,
            store: &unmirrored,
        };
        let opened = Cell::new(false);
        let open = async {
            opened.set(true);
            Ok(crate::fake::pipe(ID))
        };
        let (o, c) = block_on(run(open, &g, &page));
        assert!(!opened.get() && c.is_none());
        assert_eq!(o.refused.as_deref(), Some("--tries is at least 1"));
        assert!(o.files.is_empty() && o.root().is_none());
    }
}
