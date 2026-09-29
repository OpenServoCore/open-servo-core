//! `osc capture session`: the campaign. The capture front, a discarded
//! warm-up, then per capture the front and the jam check again and each
//! recording up to RECORDING_TRIES times, the pack read at rest before every
//! try, the horn parked at centre and torqued off between recordings and at
//! the end of every run. A blocked shaft ends the run where it stopped, with
//! no retry and no park. Only a landed `.csv.gz` marks a recording done, so a
//! rerun resumes where the last one stopped.

use std::ffi::OsString;
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::ops::RangeInclusive;
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Result, anyhow, bail};
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::pipe::PipeError;
use osc_client::{Id, LinkError};

use super::envelope::{Envelope, civil_date};
use super::front::{self, Front, Proved};
use super::plan::{self, Plan};
use super::procs::Procedure;
use super::store::{Capture, CaptureMeta, Decl, PosLutFile, Store};
use super::verdict::verdict;
use super::{RULE, RUNG_TRIES, SETTLE_MS, Supply, WINDOW_MS};
use crate::rig::battery::{self, read_pack_mv};
use crate::rig::plant::{self, Snapshot};
use crate::rig::pump::{self, STOP, read_snapshot};
use crate::rig::servo::{Wire, guard};
use crate::rig::{blocked, centre, park};
use crate::sweep::{self, Cfg, Dirs};

/// `osc capture session` args.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Servo key; the dataset dir is `<root>/<servo>__<supply>__limit`.
    #[arg(long)]
    servo: String,
    /// Supply the servo runs on.
    #[arg(long, value_enum)]
    supply: Supply,
    /// Captures to run, `n` or `a..b` inclusive; defaults to the procedure's
    /// count. Captures already landed are skipped.
    #[arg(long, value_parser = super::parse_captures)]
    captures: Option<RangeInclusive<u32>>,
    /// Skip the discarded warm-up capture.
    #[arg(long)]
    no_warmup: bool,
    /// Re-record the captures in range even when they already landed.
    #[arg(long)]
    redo: bool,
    /// Dataset root; defaults to notebooks/telemetry in this git checkout.
    #[arg(long)]
    root: Option<PathBuf>,
}

const EXPERIMENT: &str = "session";
/// Tries per recording; an adapter loss does not spend one.
const RECORDING_TRIES: u32 = 5;
/// Adapter losses one recording survives, each redoing it from the start; a
/// link that keeps dropping stops the run instead of looping on it.
const RECORDING_LOSSES: u32 = 3;
/// How long a lost adapter gets to come back.
const RECONNECT: Duration = Duration::from_secs(30);
const RECONNECT_POLL: Duration = Duration::from_secs(2);
/// Rest after a rejected try.
const RETRY_REST: Duration = Duration::from_secs(5);
/// A rebooted servo's boot, before the next command.
const REBOOT_WAIT: Duration = Duration::from_secs(3);

/// Entry from `osc capture session`.
pub(crate) fn run(a: &Args, baud: String, id: u8) -> Result<()> {
    let root = match &a.root {
        Some(r) => r.clone(),
        None => super::default_root()?,
    };
    let dir = super::dataset_dir(&root, &a.servo, a.supply);
    let (p, source) = Procedure::load()?;
    let env = Envelope::load(&dir)?;
    if env.supply != a.supply {
        bail!(
            "{}/envelope.toml was measured on {}, not {}",
            dir.display(),
            env.supply.as_str(),
            a.supply.as_str()
        );
    }
    let store = Store::new(dir.clone());
    let range = a.captures.clone().unwrap_or(1..=p.captures);
    let names: Vec<&str> = p.recording.iter().map(|r| r.name.as_str()).collect();
    let todo = todo(range.clone(), &names, a.redo, |n, r| {
        store.landed(EXPERIMENT, n, r)
    });
    // Every plan expands before the servo moves, so a bad envelope or
    // procedure refuses here.
    let jobs = todo
        .iter()
        .map(|(n, left)| {
            let plans = plan::expand(&p, &env, *n)?
                .into_iter()
                .filter(|pl| left.contains(&pl.recording.as_str()))
                .collect();
            Ok((*n, plans))
        })
        .collect::<Result<Vec<(u32, Vec<Plan>)>>>()?;
    let warmup = match todo.first() {
        Some(&(first, _)) if p.warmup && !a.no_warmup => {
            Some((first, plan::expand(&p, &env, first)?))
        }
        _ => None,
    };
    let tag = dir.file_name().map_or_else(
        || dir.display().to_string(),
        |f| f.to_string_lossy().into_owned(),
    );
    let dataset_name = tag.clone();
    let mut log = Log::open(tag);
    log.line(format_args!("procedure: {source}"));
    if jobs.is_empty() {
        log.line(format_args!(
            "captures {}..{} all landed; --redo re-records them",
            range.start(),
            range.end()
        ));
        return Ok(());
    }
    let queued: Vec<String> = jobs.iter().map(|(n, _)| n.to_string()).collect();
    log.line(format_args!(
        "session: captures {}..{}, to run: {}",
        range.start(),
        range.end(),
        queued.join(" ")
    ));

    pump::install_ctrlc();
    let mut c = crate::rig::connect(&baud)?;
    let env_path = dir.join("envelope.toml");
    let front = front::read(&mut c, Id::new(id), a.supply)?;
    front.check_envelope(&env, &env_path)?;
    let decl = Decl {
        servo: a.servo.clone(),
        supply: a.supply,
        rule: RULE,
        current_limit_counts: Some(front.lim.i_lim),
        captured: civil_date(now()),
        notes: None,
    };
    if declare(&store, &decl)? {
        log.line(format_args!("wrote {}", dir.join("dataset.toml").display()));
    }
    let mut s = Session {
        c: Some(c),
        baud,
        id: Id::new(id),
        p: &p,
        env: &env,
        store: &store,
        supply: a.supply,
        log: &mut log,
        dataset: dataset_name,
        plant: None,
        env_path,
        front,
        proved: None,
    };
    let r = s.campaign(warmup.as_ref(), &jobs, a.redo);
    s.recheck_plant();
    s.finish(r.as_ref().err());
    match r {
        Ok(()) => {
            s.log.line("session DONE");
            Ok(())
        }
        Err(stop) => {
            s.log.line(format_args!("session STOPPED: {stop}"));
            std::process::exit(stop.code())
        }
    }
}

/// Why a campaign stopped short.
#[derive(Debug)]
enum Stop {
    /// A recording ran out of tries.
    Failed(String),
    AdapterLost,
    Battery(String),
    /// Nothing may drive the shaft again, not even to park it.
    Blocked(String),
    Interrupted,
    Error(anyhow::Error),
}

impl Stop {
    /// The bench scripts' codes, so chained runs keep their meaning; 130 is
    /// the shell's for ctrl-c.
    fn code(&self) -> i32 {
        match self {
            Stop::Failed(_) | Stop::Error(_) => 1,
            Stop::AdapterLost => 2,
            Stop::Battery(_) => 3,
            Stop::Blocked(_) => front::BLOCKED_EXIT,
            Stop::Interrupted => 130,
        }
    }
}

impl fmt::Display for Stop {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Stop::Failed(m) => write!(f, "{m}"),
            Stop::AdapterLost => f.write_str("adapter lost - replug and rerun"),
            Stop::Battery(m) => write!(f, "battery: {m}"),
            Stop::Blocked(m) => write!(f, "{m}"),
            Stop::Interrupted => f.write_str("interrupted"),
            Stop::Error(e) => write!(f, "{e:#}"),
        }
    }
}

/// What an error from the servo means for the run.
#[derive(Debug, PartialEq, Eq)]
enum Kind {
    Interrupted,
    /// The link is gone or poisoned: reconnect.
    Lost,
    /// A drive found the shaft blocked: nothing drives it again.
    Blocked,
    /// The servo side: a reject, a timeout, a seek that gave up.
    Other,
}

fn kind(e: &anyhow::Error, stopped: bool) -> Kind {
    if stopped {
        return Kind::Interrupted;
    }
    let lost = e.chain().any(|c| {
        c.downcast_ref::<PipeError>().is_some()
            || matches!(
                c.downcast_ref::<osc_client::Error>(),
                Some(osc_client::Error::Pipe(_) | osc_client::Error::Link(LinkError::Desync(_)))
            )
    });
    if lost {
        Kind::Lost
    } else if blocked(e) {
        Kind::Blocked
    } else {
        Kind::Other
    }
}

fn stopped() -> bool {
    STOP.load(Ordering::SeqCst)
}

#[derive(Debug)]
enum Outcome {
    Accepted { segs: usize },
    Rejected(String),
    Lost(String),
    Blocked(String),
    Interrupted,
}

fn outcome_of(e: &anyhow::Error) -> Outcome {
    match kind(e, stopped()) {
        Kind::Interrupted => Outcome::Interrupted,
        Kind::Lost => Outcome::Lost(format!("{e:#}")),
        Kind::Blocked => Outcome::Blocked(format!("{e:#}")),
        Kind::Other => Outcome::Rejected(format!("{e:#}")),
    }
}

#[derive(Debug, PartialEq, Eq)]
enum Next {
    Done,
    Retry,
    Reconnect,
    Failed,
    AdapterLost,
    Blocked(String),
    Interrupted,
}

/// One recording's try accounting: a reject spends a try, an adapter loss
/// redoes the try without spending it, a blocked shaft ends the run.
#[derive(Default)]
struct Tries {
    rejected: u32,
    lost: u32,
}

impl Tries {
    /// The try about to run, counting from 1.
    fn attempt(&self) -> u32 {
        self.rejected + 1
    }

    fn after(&mut self, o: &Outcome) -> Next {
        match o {
            Outcome::Accepted { .. } => Next::Done,
            Outcome::Interrupted => Next::Interrupted,
            Outcome::Blocked(why) => Next::Blocked(why.clone()),
            Outcome::Rejected(_) => {
                self.rejected += 1;
                if self.rejected < RECORDING_TRIES {
                    Next::Retry
                } else {
                    Next::Failed
                }
            }
            Outcome::Lost(_) => {
                self.lost += 1;
                if self.lost <= RECORDING_LOSSES {
                    Next::Reconnect
                } else {
                    Next::AdapterLost
                }
            }
        }
    }
}

/// Per capture in `range`, the recordings still to land, in procedure order;
/// a capture with none left is skipped. `redo` re-records every one.
fn todo<'a>(
    range: RangeInclusive<u32>,
    names: &[&'a str],
    redo: bool,
    landed: impl Fn(u32, &str) -> bool,
) -> Vec<(u32, Vec<&'a str>)> {
    range
        .filter_map(|n| {
            let left: Vec<&str> = names
                .iter()
                .copied()
                .filter(|r| redo || !landed(n, r))
                .collect();
            (!left.is_empty()).then_some((n, left))
        })
        .collect()
}

/// Writes dataset.toml on the first run; a later run must name the same
/// servo and supply, under the same drive rule and current limit. True when
/// written.
fn declare(store: &Store, decl: &Decl) -> Result<bool> {
    if decl.save(store)? {
        return Ok(true);
    }
    let have = Decl::load(store)?;
    if (have.servo.as_str(), have.supply) != (decl.servo.as_str(), decl.supply) {
        bail!(
            "dataset.toml names {} on {}, not {} on {}",
            have.servo,
            have.supply.as_str(),
            decl.servo,
            decl.supply.as_str()
        );
    }
    if have.rule != decl.rule {
        bail!(
            "dataset.toml declares rule {}, this capture runs under {}: a dataset never mixes \
             drive rules",
            have.rule.as_str(),
            decl.rule.as_str()
        );
    }
    if have.current_limit_counts != decl.current_limit_counts {
        let counts = |l: Option<u16>| l.map_or("no".to_string(), |l| l.to_string());
        bail!(
            "dataset.toml declares a current limit of {} counts, the servo holds {}: a dataset \
             never mixes current limits",
            counts(have.current_limit_counts),
            counts(decl.current_limit_counts)
        );
    }
    Ok(false)
}

/// A session recording: the plan's schedule both ways at the procedure's
/// pacing, between the envelope's guards, the seeks at the jam check's duty.
fn cfg(p: &Procedure, env: &Envelope, plan: &Plan, proved: &Proved) -> Cfg {
    Cfg {
        steps: plan.schedule.clone(),
        dirs: Dirs::Both,
        decay: plan.decay,
        window_ms: WINDOW_MS,
        rest_ms: p.rest_ms,
        baseline_ms: p.baseline_ms,
        seek_duty_pct: proved.seek_pct(),
        seek_cap_pct: proved.cap_pct(),
        settle_ms: SETTLE_MS,
        stall: false,
        static_load: false,
        guard: (env.limits.guard[0], env.limits.guard[1]),
        stops: Some((env.limits.phys[0], env.limits.phys[1])),
        tel_mask: p.tel_mask,
        rung_tries: RUNG_TRIES,
    }
}

/// Sleep in slices that honour ctrl-c.
fn pause(d: Duration) -> Result<(), Stop> {
    let end = Instant::now() + d;
    loop {
        if stopped() {
            return Err(Stop::Interrupted);
        }
        let left = end.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Ok(());
        }
        std::thread::sleep(left.min(Duration::from_millis(100)));
    }
}

fn now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

/// UTC `YYYY-MM-DDTHH:MM:SSZ` of a unix time.
fn stamp(secs: u64) -> String {
    let t = secs % 86_400;
    format!(
        "{}T{:02}:{:02}:{:02}Z",
        civil_date(secs),
        t / 3600,
        t / 60 % 60,
        t % 60
    )
}

fn log_path(state: Option<OsString>, home: Option<OsString>) -> Option<PathBuf> {
    let base = match state.filter(|v| !v.is_empty()) {
        Some(s) => PathBuf::from(s),
        None => PathBuf::from(home?).join(".local").join("state"),
    };
    Some(base.join("osc").join("capture.log"))
}

/// Progress lines to stdout, each appended to the run log too.
struct Log {
    tag: String,
    file: Option<File>,
}

impl Log {
    fn open(tag: String) -> Self {
        let path = log_path(std::env::var_os("XDG_STATE_HOME"), std::env::var_os("HOME"));
        let file = path.and_then(|p| {
            let opened = p
                .parent()
                .map_or(Ok(()), std::fs::create_dir_all)
                .and_then(|()| OpenOptions::new().create(true).append(true).open(&p));
            match opened {
                Ok(f) => {
                    println!("log: {}", p.display());
                    Some(f)
                }
                Err(e) => {
                    eprintln!("warning: no run log at {}: {e}", p.display());
                    None
                }
            }
        });
        Self { tag, file }
    }

    fn line(&mut self, msg: impl fmt::Display) {
        let l = format!("{} [{}] {msg}", stamp(now()), self.tag);
        println!("{l}");
        if let Some(f) = &mut self.file {
            let _ = writeln!(f, "{l}");
        }
    }
}

struct Session<'a> {
    /// None once a lost link failed to come back.
    c: Option<Client<NusbPipe>>,
    baud: String,
    id: Id,
    p: &'a Procedure,
    env: &'a Envelope,
    store: &'a Store,
    supply: Supply,
    log: &'a mut Log,
    /// `<servo>__<supply>`, what the dataset's table image is named for.
    dataset: String,
    /// The plant as read at the start, for the end-of-run recheck.
    plant: Option<Snapshot>,
    env_path: PathBuf,
    /// The servo as the front last read it.
    front: Front,
    /// What the last jam check proved; None until one did, and while the
    /// next one runs.
    proved: Option<Proved>,
}

impl Session<'_> {
    fn client(&mut self) -> Result<&mut Client<NusbPipe>, Stop> {
        self.c.as_mut().ok_or(Stop::AdapterLost)
    }

    fn campaign(
        &mut self,
        warmup: Option<&(u32, Vec<Plan>)>,
        jobs: &[(u32, Vec<Plan>)],
        redo: bool,
    ) -> Result<(), Stop> {
        let fw = self.on_servo(|c, id| Ok(c.identity(id)?.fw))?;
        if fw != self.env.fw {
            self.log.line(format_args!(
                "warning: envelope measured on fw {}, servo runs fw {fw}: rerun osc capture \
                 pilot if the drive changed",
                self.env.fw
            ));
        }
        self.record_plant(fw)?;
        if let Some((n, plans)) = warmup {
            self.prove()?;
            self.log
                .line(format_args!("warm-up: capture-{n}'s plan, discarded"));
            let cap = Capture::warmup(self.store, EXPERIMENT, *n).map_err(Stop::Error)?;
            for plan in plans {
                self.recording(&cap, plan, "warm-up")?;
                self.park()?;
            }
            drop(cap);
            self.cooldown()?;
        }
        for (i, (n, plans)) in jobs.iter().enumerate() {
            self.prove()?;
            let order: Vec<String> = plans
                .iter()
                .map(|pl| {
                    let b: Vec<&str> = pl.blocks.iter().map(|b| b.name.as_str()).collect();
                    format!("{} ({})", pl.recording, b.join(" "))
                })
                .collect();
            self.log
                .line(format_args!("capture-{n}: {}", order.join(", ")));
            let cap = Capture::open(self.store, EXPERIMENT, *n).map_err(Stop::Error)?;
            let r = self.capture(&cap, plans, redo, &format!("capture-{n}"));
            if r.is_err() {
                // Only succeeds when nothing landed: an empty capture dir
                // would read as a capture to the notebooks.
                let _ = std::fs::remove_dir(cap.dir());
            }
            r?;
            self.log.line(format_args!("capture-{n} landed"));
            if i + 1 < jobs.len() {
                self.cooldown()?;
            }
        }
        Ok(())
    }

    fn capture(
        &mut self,
        cap: &Capture,
        plans: &[Plan],
        redo: bool,
        label: &str,
    ) -> Result<(), Stop> {
        // Unlanded up front, so a redo cut short reads as incomplete instead
        // of mixing old recordings with new.
        if redo {
            for plan in plans {
                cap.discard(&plan.recording).map_err(Stop::Error)?;
            }
        }
        for plan in plans {
            self.recording(cap, plan, label)?;
            self.park()?;
        }
        Ok(())
    }

    fn recording(&mut self, cap: &Capture, plan: &Plan, label: &str) -> Result<(), Stop> {
        let name = &plan.recording;
        let mut tries = Tries::default();
        loop {
            if stopped() {
                return Err(Stop::Interrupted);
            }
            self.gate()?;
            let proved = self.proved()?;
            let cfg = cfg(self.p, self.env, plan, &proved);
            let attempt = tries.attempt();
            let outcome = self.attempt(cap, plan, &cfg, &proved, attempt)?;
            match &outcome {
                Outcome::Accepted { segs } => self.log.line(format_args!(
                    "  {label}/{name}: {segs} segs clean (try {attempt})"
                )),
                Outcome::Rejected(why) => self
                    .log
                    .line(format_args!("  REJECT {label}/{name} try {attempt}: {why}")),
                Outcome::Lost(why) => self
                    .log
                    .line(format_args!("  ADAPTER GONE during {label}/{name}: {why}")),
                Outcome::Blocked(why) => self
                    .log
                    .line(format_args!("  BLOCKED {label}/{name}: {why}")),
                Outcome::Interrupted => {
                    self.log.line(format_args!("  {label}/{name}: interrupted"))
                }
            }
            match tries.after(&outcome) {
                Next::Done => return Ok(()),
                Next::Retry => {
                    self.park()?;
                    pause(RETRY_REST)?;
                }
                Next::Reconnect => self.reconnect()?,
                Next::Failed => {
                    return Err(Stop::Failed(format!(
                        "{label}/{name} rejected {RECORDING_TRIES} times"
                    )));
                }
                Next::AdapterLost => return Err(Stop::AdapterLost),
                Next::Blocked(why) => return Err(Stop::Blocked(why)),
                Next::Interrupted => return Err(Stop::Interrupted),
            }
        }
    }

    /// One try, judged: accepted into the capture or rejected with why.
    fn attempt(
        &mut self,
        cap: &Capture,
        plan: &Plan,
        cfg: &Cfg,
        proved: &Proved,
        attempt: u32,
    ) -> Result<Outcome, Stop> {
        let (id, supply) = (self.id, self.supply);
        let drive = self.front.drive(proved);
        let c = self.client()?;
        let meta = match sweep::meta(c, id, cfg, drive) {
            Ok(m) => m,
            Err(e) => return Ok(outcome_of(&e)),
        };
        let mut t = cap.begin(&plan.recording, &meta).map_err(Stop::Error)?;
        let judged = (|| -> Result<Result<usize, String>> {
            let rec = sweep::record(&mut Wire::new(&mut *c, id), cfg, |g| t.on_seg(g))?;
            if let Err(why) = verdict(&rec, cfg) {
                return Ok(Err(why));
            }
            let s = read_snapshot(c, id)?;
            Ok(match s.fault_flags {
                0 => Ok(rec.segments.len()),
                f => Err(format!(
                    "servo faulted: flags {f:#04x} code {}",
                    s.fault_code
                )),
            })
        })();
        match judged {
            Ok(Ok(segs)) => {
                t.accept(&CaptureMeta {
                    supply,
                    plan,
                    attempt,
                })
                .map_err(Stop::Error)?;
                Ok(Outcome::Accepted { segs })
            }
            Ok(Err(why)) => {
                t.reject().map_err(Stop::Error)?;
                Ok(Outcome::Rejected(why))
            }
            Err(e) => {
                t.reject().map_err(Stop::Error)?;
                Ok(outcome_of(&e))
            }
        }
    }

    /// The plant the captures are made under, logged, and its table kept
    /// with the dataset while one is LIVE. Every recording's meta names
    /// the table by crc; the dataset holds the points once.
    fn record_plant(&mut self, fw: u16) -> Result<(), Stop> {
        let (dataset, source) = (
            self.dataset.clone(),
            format!(
                "servo id {} fw {fw}, read at session start",
                self.id.as_byte()
            ),
        );
        let (snap, image) = self.on_servo(|c, id| {
            let d = crate::state::descriptor(c, id)?;
            let snap = Snapshot::read(c, id, &d)?;
            let image = snap
                .lut
                .live()
                .then(|| plant::stops(c, id, &d).map(|s| snap.lut.image(s, &dataset, &source)))
                .transpose()?;
            Ok((snap, image))
        })?;
        self.log.line(format_args!("plant: {}", snap.line()));
        if let Some(image) = image {
            match self.store.save_pos_lut(&image).map_err(Stop::Error)? {
                PosLutFile::Written => self.log.line("wrote pos-lut.json"),
                PosLutFile::Same => {}
                PosLutFile::Differs(crc) => self.log.line(format_args!(
                    "warning: pos-lut.json holds table {crc}, the servo runs {}: the dataset \
                     will mix tables (osc capture check flags it)",
                    image["lut_crc"].as_str().unwrap_or_default()
                )),
            }
        }
        self.plant = Some(snap);
        Ok(())
    }

    /// The plant read again after the run: a table lost to a reboot, a
    /// restamp or a data-state change mid-session means the captures do
    /// not all describe the same servo.
    fn recheck_plant(&mut self) {
        let Some(before) = self.plant.take() else {
            return;
        };
        let Some(c) = self.c.as_mut() else {
            self.log
                .line("no adapter: the plant was not re-read at the end");
            return;
        };
        let id = self.id;
        let after = crate::state::descriptor(c, id).and_then(|d| Snapshot::read(c, id, &d));
        match after {
            Ok(after) if before.same(&after) => {}
            Ok(after) => self.log.line(format_args!(
                "warning: plant changed during the session: was {}, now {}",
                before.line(),
                after.line()
            )),
            Err(e) => self
                .log
                .line(format_args!("plant not re-read at the end: {e:#}")),
        }
    }

    /// Refuse a pack near empty, read at rest; a supply with no gate
    /// configured is not read.
    fn gate(&mut self) -> Result<(), Stop> {
        let p = self.p;
        let Some(g) = p.supply.get(&self.supply) else {
            return Ok(());
        };
        let pack = self.on_servo(read_pack_mv)?;
        match battery::gate(Some(g), pack) {
            battery::Verdict::Ok => {
                self.log
                    .line(format_args!("battery: pack {} mV", pack.unwrap_or(0)));
                Ok(())
            }
            battery::Verdict::Warn(m) => {
                self.log.line(format_args!("battery WARNING: {m}"));
                Ok(())
            }
            battery::Verdict::Refuse(m) => Err(Stop::Battery(m)),
        }
    }

    fn cooldown(&mut self) -> Result<(), Stop> {
        let s = self.p.cooldown_s;
        self.log.line(format_args!("cooldown {s} s, then reboot"));
        pause(Duration::from_secs(s.into()))?;
        self.on_servo(|c, id| Ok(c.reboot(id)?))?;
        pause(REBOOT_WAIT)
    }

    /// Park at the jam check's duty; a shaft not proven free stays put.
    fn park(&mut self) -> Result<(), Stop> {
        let Some(p) = self.proved else {
            return Ok(());
        };
        let (center, duty) = (self.env.limits.center, p.seek_q15());
        self.on_servo(|c, id| guard(&mut Wire::new(c, id), |s| park::park(s, center, duty)))
    }

    fn proved(&self) -> Result<Proved, Stop> {
        self.proved
            .ok_or_else(|| Stop::Error(anyhow!("no jam check proved the shaft free")))
    }

    /// The front and the jam check again, on a servo a cooldown or a lost
    /// link may have rebooted: its RAM settings are the saved ones again.
    fn prove(&mut self) -> Result<(), Stop> {
        self.proved = None;
        let (supply, env, path) = (self.supply, self.env, self.env_path.clone());
        let (front, proved) = self.on_servo(|c, id| prove(c, id, supply, env, &path))?;
        self.proved_by(front, proved);
        Ok(())
    }

    fn proved_by(&mut self, front: Front, proved: Proved) {
        self.log.line(format_args!(
            "jam check: the shaft moves at {:.1}%, seeks at {}%, raised off a stop up to {}%",
            proved.moved * 100.0,
            proved.seek_pct(),
            proved.cap_pct()
        ));
        self.front = front;
        self.proved = Some(proved);
    }

    /// `f` on the servo; an adapter loss reconnects and runs it again.
    fn on_servo<T>(
        &mut self,
        mut f: impl FnMut(&mut Client<NusbPipe>, Id) -> Result<T>,
    ) -> Result<T, Stop> {
        let id = self.id;
        for _ in 0..=RECORDING_LOSSES {
            let e = match f(self.client()?, id) {
                Ok(v) => return Ok(v),
                Err(e) => e,
            };
            match kind(&e, stopped()) {
                Kind::Interrupted => return Err(Stop::Interrupted),
                Kind::Blocked => return Err(Stop::Blocked(format!("{e:#}"))),
                Kind::Other => return Err(Stop::Error(e)),
                Kind::Lost => {
                    self.log.line(format_args!("  ADAPTER GONE: {e:#}"));
                    self.reconnect()?;
                }
            }
        }
        Err(Stop::AdapterLost)
    }

    /// Drop the dead link and reopen it within RECONNECT, then reboot the
    /// servo, read it again and prove the shaft free. A servo whose host
    /// vanished mid-rung may have driven on to the soft limit; the reboot and
    /// the jam check, which ends at mid travel, start the redo clean.
    fn reconnect(&mut self) -> Result<(), Stop> {
        self.proved = None;
        // The old handle holds the interface claim; the new open needs it.
        self.c = None;
        self.log.line(format_args!(
            "  reconnecting for up to {} s",
            RECONNECT.as_secs()
        ));
        let t0 = Instant::now();
        let c = loop {
            match crate::rig::connect(&self.baud) {
                Ok(c) => break c,
                Err(e) if t0.elapsed() >= RECONNECT => {
                    self.log.line(format_args!("  no adapter: {e:#}"));
                    return Err(Stop::AdapterLost);
                }
                Err(_) => pause(RECONNECT_POLL)?,
            }
        };
        self.c = Some(c);
        self.log.line(format_args!(
            "  adapter back after {} s: reboot, jam check, redo",
            t0.elapsed().as_secs()
        ));
        let lost = |e: anyhow::Error| match kind(&e, stopped()) {
            Kind::Interrupted => Stop::Interrupted,
            Kind::Lost => Stop::AdapterLost,
            Kind::Blocked => Stop::Blocked(format!("{e:#}")),
            Kind::Other => Stop::Error(e),
        };
        let (id, supply, env) = (self.id, self.supply, self.env);
        let r = self.client()?.reboot(id);
        r.map_err(|e| lost(e.into()))?;
        pause(REBOOT_WAIT)?;
        let path = self.env_path.clone();
        let (front, proved) = prove(self.client()?, id, supply, env, &path).map_err(lost)?;
        self.proved_by(front, proved);
        Ok(())
    }

    /// Every run ends at centre, torque off and the permit clear. Ctrl-c is
    /// cleared first so the park drives; a second ctrl-c cuts the drive
    /// short and still torques off.
    fn finish(&mut self, stop: Option<&Stop>) {
        STOP.store(false, Ordering::SeqCst);
        let (id, center) = (self.id, self.env.limits.center);
        let duty = end_park(stop, self.proved.as_ref());
        let Some(c) = self.c.as_mut() else {
            self.log
                .line("no adapter: the servo was left where it stopped");
            return;
        };
        let r = guard(&mut Wire::new(c, id), |s| match duty {
            Some(duty) => park::park(s, center, duty),
            None => Ok(()),
        });
        if let Err(e) = r {
            self.log.line(format_args!("park failed: {e:#}"));
        }
        if duty.is_none() {
            self.log
                .line("the shaft was left where it stopped, torque off");
        }
    }
}

/// The duty the run's last park drives at: none after a blocked shaft,
/// which nothing may drive again, or before a jam check proved it free.
fn end_park(stop: Option<&Stop>, proved: Option<&Proved>) -> Option<i16> {
    match stop {
        Some(Stop::Blocked(_)) => None,
        _ => proved.map(Proved::seek_q15),
    }
}

/// The front, the envelope it must match and the jam check, on the servo.
fn prove(
    c: &mut Client<NusbPipe>,
    id: Id,
    supply: Supply,
    env: &Envelope,
    env_path: &Path,
) -> Result<(Front, Proved)> {
    let front = front::read(c, id, supply)?;
    front.check_envelope(env, env_path)?;
    let proved = front.jam_check(|exp| centre::drive(&mut Wire::new(c, id), exp))?;
    Ok((front, proved))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::envelope;
    use crate::capture::store::fixture::tmp;
    use crate::capture::verdict::expected_segments;
    use crate::sweep::Decay;
    use osc_client::ResultCode;
    use osc_ident::exp::AbortReason;
    use osc_ident::exp::testkit::{FakeServo, bench_mg90, pump};
    use osc_ident::limits::{CLASS_R_MIN, ServoLimits, q15_floor};
    use osc_ident::regs::control;

    #[test]
    fn todo_skips_landed_and_resumes_half_done() {
        let landed = |n: u32, r: &str| n == 1 || (n == 2 && r == "slow");
        let t = todo(1..=4, &["slow", "fast"], false, landed);
        assert_eq!(
            t,
            [
                (2, vec!["fast"]),
                (3, vec!["slow", "fast"]),
                (4, vec!["slow", "fast"])
            ]
        );
        assert!(todo(1..=1, &["slow", "fast"], false, landed).is_empty());
    }

    #[test]
    fn redo_reruns_every_recording() {
        let t = todo(2..=3, &["slow", "fast"], true, |_, _| true);
        assert_eq!(t, [(2, vec!["slow", "fast"]), (3, vec!["slow", "fast"])]);
    }

    fn rejected() -> Outcome {
        Outcome::Rejected("2 holes".into())
    }

    fn lost() -> Outcome {
        Outcome::Lost("pipe: gone".into())
    }

    #[test]
    fn rejects_spend_tries_until_the_last() {
        let mut t = Tries::default();
        for k in 1..RECORDING_TRIES {
            assert_eq!(t.attempt(), k);
            assert_eq!(t.after(&rejected()), Next::Retry);
        }
        assert_eq!(t.attempt(), RECORDING_TRIES);
        assert_eq!(t.after(&rejected()), Next::Failed);
    }

    #[test]
    fn an_adapter_loss_redoes_the_try_without_spending_it() {
        let mut t = Tries::default();
        assert_eq!(t.after(&rejected()), Next::Retry);
        assert_eq!(t.after(&lost()), Next::Reconnect);
        assert_eq!(t.after(&lost()), Next::Reconnect);
        assert_eq!(t.attempt(), 2);
        assert_eq!(t.after(&Outcome::Accepted { segs: 5 }), Next::Done);
        // the accepted recording is try 2 whatever the losses
        assert_eq!(t.attempt(), 2);
    }

    #[test]
    fn a_link_that_keeps_dropping_stops_the_run() {
        let mut t = Tries::default();
        for _ in 0..RECORDING_LOSSES {
            assert_eq!(t.after(&lost()), Next::Reconnect);
        }
        assert_eq!(t.after(&lost()), Next::AdapterLost);
        assert_eq!(t.attempt(), 1);
    }

    #[test]
    fn interrupt_stops_at_once() {
        let mut t = Tries::default();
        assert_eq!(t.after(&Outcome::Interrupted), Next::Interrupted);
    }

    #[test]
    fn exit_codes_match_the_bench_scripts() {
        let codes: Vec<i32> = [
            Stop::Failed("x".into()),
            Stop::Error(anyhow::anyhow!("x")),
            Stop::AdapterLost,
            Stop::Battery("low".into()),
            Stop::Interrupted,
        ]
        .iter()
        .map(Stop::code)
        .collect();
        assert_eq!(codes, [1, 1, 2, 3, 130]);
        assert_eq!(
            Stop::AdapterLost.to_string(),
            "adapter lost - replug and rerun"
        );
    }

    #[test]
    fn transport_errors_are_losses_servo_errors_are_not() {
        use osc_client::Error as E;
        let wrapped = |e: E| anyhow::Error::from(e).context("telemetry read");
        let io = || E::Pipe(PipeError::Io("device gone".into()));
        assert_eq!(kind(&wrapped(io()), false), Kind::Lost);
        assert_eq!(
            kind(&wrapped(E::Pipe(PipeError::Stalled)), false),
            Kind::Lost
        );
        assert_eq!(
            kind(&wrapped(E::Link(LinkError::Desync("seq".into()))), false),
            Kind::Lost
        );
        let open = anyhow::Error::from(PipeError::Io("no adapter".into()));
        assert_eq!(kind(&open, false), Kind::Lost);
        assert_eq!(kind(&wrapped(E::Timeout { slot: 0 }), false), Kind::Other);
        assert_eq!(
            kind(&wrapped(E::Servo(ResultCode::Range)), false),
            Kind::Other
        );
        assert_eq!(
            kind(&anyhow::anyhow!("seek did not reach"), false),
            Kind::Other
        );
        // ctrl-c wins: the error it cut short is not a finding
        assert_eq!(kind(&wrapped(io()), true), Kind::Interrupted);
    }

    #[test]
    fn log_path_prefers_xdg_state() {
        let p = |x: Option<&str>, h: Option<&str>| log_path(x.map(Into::into), h.map(Into::into));
        assert_eq!(
            p(Some("/s"), Some("/h")),
            Some(PathBuf::from("/s/osc/capture.log"))
        );
        assert_eq!(
            p(Some(""), Some("/h")),
            Some(PathBuf::from("/h/.local/state/osc/capture.log"))
        );
        assert_eq!(
            p(None, Some("/h")),
            Some(PathBuf::from("/h/.local/state/osc/capture.log"))
        );
        assert_eq!(p(None, None), None);
    }

    #[test]
    fn stamp_is_utc_iso() {
        assert_eq!(stamp(0), "1970-01-01T00:00:00Z");
        assert_eq!(stamp(1_758_758_400 + 3_723), "2025-09-25T01:02:03Z");
        assert_eq!(stamp(1_758_758_400 + 86_399), "2025-09-25T23:59:59Z");
    }

    #[test]
    fn declare_writes_once_and_refuses_another_servo() {
        let root = tmp("declare");
        let store = Store::new(root.join("mg90-a__2s__limit"));
        let decl = |servo: &str, supply| Decl {
            servo: servo.into(),
            supply,
            rule: RULE,
            current_limit_counts: Some(280),
            captured: "2026-09-25".into(),
            notes: None,
        };
        assert!(declare(&store, &decl("mg90-a", Supply::TwoS)).unwrap());
        let later = Decl {
            captured: "2026-09-26".into(),
            ..decl("mg90-a", Supply::TwoS)
        };
        assert!(!declare(&store, &later).unwrap());
        assert_eq!(Decl::load(&store).unwrap().captured, "2026-09-25");
        let e = declare(&store, &decl("sg90-a", Supply::TwoS)).unwrap_err();
        assert!(
            e.to_string()
                .contains("names mg90-a on 2s, not sg90-a on 2s"),
            "{e}"
        );
        assert!(declare(&store, &decl("mg90-a", Supply::Usb)).is_err());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn declare_refuses_another_rule() {
        let root = tmp("declare-rule");
        // a dataset captured before the servo limited open-loop current
        let free = Store::new(root.join("mg90-a__2s"));
        std::fs::create_dir_all(root.join("mg90-a__2s")).unwrap();
        std::fs::write(
            root.join("mg90-a__2s/dataset.toml"),
            "servo = \"mg90-a\"\nsupply = \"2s\"\ncaptured = \"2026-09-26\"\n",
        )
        .unwrap();
        let limit = |counts| Decl {
            servo: "mg90-a".into(),
            supply: Supply::TwoS,
            rule: RULE,
            current_limit_counts: Some(counts),
            captured: "2026-09-29".into(),
            notes: None,
        };
        assert_eq!(
            declare(&free, &limit(280)).unwrap_err().to_string(),
            "dataset.toml declares rule free, this capture runs under limit: a dataset never \
             mixes drive rules"
        );

        let store = Store::new(root.join("mg90-a__2s__limit"));
        assert!(declare(&store, &limit(280)).unwrap());
        assert!(!declare(&store, &limit(280)).unwrap());
        assert_eq!(
            declare(&store, &limit(300)).unwrap_err().to_string(),
            "dataset.toml declares a current limit of 280 counts, the servo holds 300: a \
             dataset never mixes current limits"
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn cfg_runs_the_plan_both_ways_between_the_envelope_guards() {
        let p = Procedure::parse(include_str!("session.toml")).unwrap();
        let env = envelope::mg90_every_window();
        let plans = plan::expand(&p, &env, 2).unwrap();
        let c = cfg(&p, &env, &plans[1], &proved(0.13));
        assert_eq!(c.steps, plans[1].schedule);
        assert_eq!(c.decay, Decay::Fast);
        assert_eq!(c.dirs, Dirs::Both);
        assert_eq!(c.guard, (532, 3526));
        // the bench session's pacing
        assert_eq!((c.rest_ms, c.baseline_ms, c.tel_mask), (1500, 1000, 0x1cd));
        assert_eq!((c.seek_duty_pct, c.seek_cap_pct), (15, 15));
        assert!(!c.stall && !c.static_load);
        assert_eq!(
            expected_segments(c.baseline_ms > 0, c.dirs.signs().len(), c.steps.len()),
            41
        );
    }

    /// The bench servo's plan once a jam check moved the shaft at `moved`.
    fn proved(moved: f64) -> Proved {
        let f = front::bench();
        Proved {
            moved,
            plan: f.lim.stall_plan(f.sc.r_vpc(CLASS_R_MIN), Some(moved)),
        }
    }

    /// The test servo at mid travel, at rest: the bench MG90 on 2S.
    fn mg90() -> FakeServo {
        let mut s = bench_mg90(3204);
        s.pos = 2029.0;
        s
    }

    fn jam_check(front: &Front, s: &mut FakeServo) -> Result<Proved> {
        front.jam_check(|exp| {
            pump(exp, s, 100_000);
            Ok(())
        })
    }

    /// The jam check on the bench servo starts at the class-safe 9.5% and
    /// raises by 2.5% while the shaft stays still: its breakaway, 13%, gives
    /// at 14.5%. Every seek, its breakout ceiling and the park drive by that:
    /// 2% over it, capped at the 15.5% whose stall draws the limit - 15%
    /// in whole percent - by the stored 7270; under the cap by a winding
    /// of 9065, 16% with a ceiling of 19%.
    #[test]
    fn seeks_drive_at_the_duty_that_moved_the_shaft() {
        let front = front::bench();
        let mut s = mg90();
        let breakaway = s.breakaway_q15 as f64 / 32767.0;
        let p = jam_check(&front, &mut s).unwrap();
        assert!(!s.torque, "the jam check ends torque off");
        let step = osc_ident::exp::centre::NUDGE_STEP_Q15 as f64 / 32767.0;
        assert!(
            p.moved >= breakaway && p.moved - step < breakaway,
            "moved at {}",
            p.moved
        );
        assert!((p.plan.seek - (p.moved + 0.02).min(p.plan.stop_cap)).abs() < 1e-12);
        assert_eq!((p.seek_pct(), p.cap_pct()), (15, 15));

        let session = Procedure::parse(include_str!("session.toml")).unwrap();
        let env = envelope::mg90_every_window();
        for plan in plan::expand(&session, &env, 1).unwrap() {
            let c = cfg(&session, &env, &plan, &p);
            assert_eq!((c.seek_duty_pct, c.seek_cap_pct), (15, 15));
        }
        assert_eq!(end_park(None, Some(&p)), Some(4915));
        let drive = front.drive(&p);
        assert_eq!(drive["seek_q15"], 4915);
        assert_eq!(drive["moved_q15"], q15_floor(p.moved));
        assert_eq!(drive["stop_cap_q15"], 4915);

        // the seek moves the shaft from rest where the duty under the one
        // that moved it did not
        for (q15, moves) in [(p.seek_q15(), true), (q15_floor(p.moved - step), false)] {
            let mut s = mg90();
            s.write(control::TORQUE_ENABLE, 1);
            s.write(control::GOAL_DUTY, q15 as i32);
            s.advance(200);
            assert_eq!((s.pos - 2029.0).abs() > 100.0, moves, "{q15} q15");
        }

        let wound = Front {
            lim: ServoLimits {
                r_q12: 9065,
                ..front.lim
            },
            ..front
        };
        let p = jam_check(&wound, &mut mg90()).unwrap();
        assert!(p.plan.seek < p.plan.stop_cap);
        assert_eq!((p.seek_pct(), p.cap_pct()), (16, 19));
        let c = cfg(
            &session,
            &env,
            &plan::expand(&session, &env, 1).unwrap()[0],
            &p,
        );
        assert_eq!((c.seek_duty_pct, c.seek_cap_pct), (16, 19));
        assert_eq!(end_park(None, Some(&p)), Some(5242));
    }

    /// A shaft locked at mid travel: the jam check raises until the limit
    /// holds a still shaft and calls it blocked, torque off. Nothing retries
    /// it, nothing parks it, and the run exits 4; a seek that finds it
    /// blocked mid capture ends the run the same way.
    #[test]
    fn a_blocked_shaft_ends_the_session_unparked() {
        let mut s = mg90();
        s.jam = Some(2029.0);
        let e = jam_check(&front::bench(), &mut s).unwrap_err();
        assert!(blocked(&e), "{e:#}");
        assert!(!s.torque && !s.permit_live());
        assert_eq!(s.pos, 2029.0);

        let o = outcome_of(&e);
        let Outcome::Blocked(why) = &o else {
            panic!("{o:?}");
        };
        assert!(
            why.starts_with("the jam check aborted: the shaft is blocked"),
            "{why}"
        );
        let mut t = Tries::default();
        assert_eq!(t.after(&o), Next::Blocked(why.clone()));
        assert_eq!(t.attempt(), 1, "no try spent, none left to retry");

        let stop = Stop::Blocked(why.clone());
        assert_eq!(stop.code(), 4);
        assert_eq!(stop.to_string(), *why);
        let p = proved(0.13);
        assert_eq!(end_park(Some(&stop), Some(&p)), None);
        assert_eq!(
            end_park(Some(&Stop::Failed("x".into())), Some(&p)),
            Some(p.seek_q15())
        );
        assert_eq!(end_park(Some(&Stop::Interrupted), None), None);

        let seek: anyhow::Error = crate::rig::Aborted {
            what: "the seek",
            reason: AbortReason::Blocked {
                pos: 1900,
                moved: 2,
            },
        }
        .into();
        assert_eq!(kind(&seek.context("slow"), false), Kind::Blocked);
    }
}
