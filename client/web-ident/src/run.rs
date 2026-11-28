//! The CLI's `run` verb over any pipe, into memory: the arguments admitted
//! before the bus is touched, the servo reached and named, [`of::run`],
//! and the reboot that clears the unsaved bit after it, every line told to
//! the feed as it is said. The files are [`of::run`]'s own, so the run
//! folder is the CLI's byte for byte.

use std::cell::RefCell;
use std::future::Future;
use std::path::Path;
use std::sync::atomic::AtomicBool;

use anyhow::{Context, Result, bail};
use osc_client::descriptor::{Descriptor, Registry, Selection};
use osc_client::pipe::Pipe;
use osc_client::{BaudRate, Client, Error, Id, mgmt};
use osc_measure::run::steps::{StepInputs, clear_unsaved};
use osc_measure::run::{self as of, Host, Progress, RunArgs, Store, servo_name};
use osc_measure::sink::MemSink;

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

/// The host a page runs on: its feed, its clock, its stop flag.
pub struct Page<'a> {
    pub feed: &'a dyn Fn(&Progress),
    /// Unix seconds.
    pub clock: &'a dyn Fn() -> f64,
    pub stop: &'a AtomicBool,
}

/// The run on the pipe `open` yields: nothing opened when the arguments
/// are refused. The client comes back where one was connected, for the
/// caller to close its device.
pub async fn run<P: Pipe>(
    open: impl Future<Output = Result<P>>,
    given: &Given,
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
    let mut store = MemSink::default();
    for (path, bytes) in &given.files {
        store.files.insert(key(path), bytes.clone());
    }
    let a = RunArgs {
        nameplate: Path::new(&given.nameplate),
        policy: Path::new(&given.policy),
        out: Path::new(""),
        tries: given.tries,
    };
    let mut notes = Vec::new();
    let admitted = of::admit(&mut store, &a, &mut notes);
    tell(notes);
    let ended = |refused: Option<String>, accepted: bool, store: &MemSink| Outcome {
        files: written(store, given),
        lines: said.borrow().clone(),
        accepted,
        refused,
    };
    let inputs = match admitted {
        Ok(i) => i,
        Err(e) => return (ended(Some(format!("{e:#}")), false, &store), None),
    };
    let (mut c, d) = match connect(open, given, &tell).await {
        Ok(cd) => cd,
        Err(e) => return (ended(Some(format!("{e:#}")), false, &store), None),
    };
    let ran = on_bus(&mut c, &d, given, &a, &inputs, &mut store, &feed, page).await;
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
    (ended(refused, accepted, &store), Some(c))
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
/// the run, then the reboot that clears unsaved; the lines of that reboot
/// with the outcome, as the CLI's `ended` joins them.
#[allow(clippy::too_many_arguments)]
async fn on_bus<P: Pipe>(
    c: &mut Client<P>,
    d: &Descriptor,
    given: &Given,
    a: &RunArgs<'_>,
    inputs: &StepInputs,
    store: &mut MemSink,
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
    let ran = of::run(c, id, d, &mut host, a, inputs, &servo).await;
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
