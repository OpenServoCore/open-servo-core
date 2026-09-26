//! `osc capture session`: the campaign. Battery gate, a discarded warm-up,
//! then per capture each recording up to RECORDING_TRIES times, the horn
//! parked at centre and torqued off between recordings and at the end of
//! every run. Only a landed `.csv.gz` marks a recording done, so a rerun
//! resumes where the last one stopped.

use std::ffi::OsString;
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::ops::RangeInclusive;
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Result, bail};
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::pipe::PipeError;
use osc_client::{Id, LinkError};

use super::battery::{self, read_pack_mv};
use super::envelope::{Envelope, civil_date};
use super::plan::{self, Plan};
use super::procs::Procedure;
use super::store::{Capture, CaptureMeta, Decl, Store};
use super::verdict::verdict;
use super::{RUNG_TRIES, SEEK_CAP_PCT, SETTLE_MS, Supply, WINDOW_MS};
use crate::rig::park;
use crate::rig::pump::{self, STOP, read_snapshot};
use crate::sweep::{self, Cfg, Dirs};

/// `osc capture session` args.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Servo key; the dataset dir is `<root>/<servo>__<supply>`.
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
    let wrote = declare(
        &store,
        &Decl {
            servo: a.servo.clone(),
            supply: a.supply,
            captured: civil_date(now()),
            notes: None,
        },
    )?;

    let tag = dir.file_name().map_or_else(
        || dir.display().to_string(),
        |f| f.to_string_lossy().into_owned(),
    );
    let mut log = Log::open(tag);
    log.line(format_args!("procedure: {source}"));
    if wrote {
        log.line(format_args!("wrote {}", dir.join("dataset.toml").display()));
    }
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
    let c = crate::rig::connect(&baud)?;
    let mut s = Session {
        c: Some(c),
        baud,
        id: Id::new(id),
        p: &p,
        env: &env,
        store: &store,
        supply: a.supply,
        log: &mut log,
    };
    let r = s.campaign(warmup.as_ref(), &jobs, a.redo);
    s.finish();
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
    if lost { Kind::Lost } else { Kind::Other }
}

fn stopped() -> bool {
    STOP.load(Ordering::SeqCst)
}

#[derive(Debug)]
enum Outcome {
    Accepted { segs: usize },
    Rejected(String),
    Lost(String),
    Interrupted,
}

fn outcome_of(e: &anyhow::Error) -> Outcome {
    match kind(e, stopped()) {
        Kind::Interrupted => Outcome::Interrupted,
        Kind::Lost => Outcome::Lost(format!("{e:#}")),
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
    Interrupted,
}

/// One recording's try accounting: a reject spends a try, an adapter loss
/// redoes the try without spending it.
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
/// servo and supply. True when written.
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
    Ok(false)
}

/// A session recording: the plan's schedule both ways at the procedure's
/// pacing, between the envelope's guards.
fn cfg(p: &Procedure, env: &Envelope, plan: &Plan) -> Cfg {
    Cfg {
        steps: plan.schedule.clone(),
        dirs: Dirs::Both,
        decay: plan.decay,
        window_ms: WINDOW_MS,
        rest_ms: p.rest_ms,
        baseline_ms: p.baseline_ms,
        seek_duty_pct: p.seek_pct,
        seek_cap_pct: SEEK_CAP_PCT,
        settle_ms: SETTLE_MS,
        stall: false,
        static_load: false,
        guard: (env.limits.guard[0], env.limits.guard[1]),
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
        if let Some((n, plans)) = warmup {
            self.gate()?;
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
            self.gate()?;
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
        let cfg = cfg(self.p, self.env, plan);
        let name = &plan.recording;
        let mut tries = Tries::default();
        loop {
            if stopped() {
                return Err(Stop::Interrupted);
            }
            let attempt = tries.attempt();
            let outcome = self.attempt(cap, plan, &cfg, attempt)?;
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
        attempt: u32,
    ) -> Result<Outcome, Stop> {
        let (id, supply) = (self.id, self.supply);
        let c = self.client()?;
        let meta = match sweep::meta(c, id, cfg) {
            Ok(m) => m,
            Err(e) => return Ok(outcome_of(&e)),
        };
        let mut t = cap.begin(&plan.recording, &meta).map_err(Stop::Error)?;
        let judged = (|| -> Result<Result<usize, String>> {
            let rec = sweep::record(c, id, cfg, |s| t.on_seg(s))?;
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

    fn park(&mut self) -> Result<(), Stop> {
        let center = self.env.limits.center;
        self.on_servo(|c, id| park::park(c, id, center))
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
    /// servo and park. A servo whose host vanished mid-rung may have driven
    /// on to the soft limit; the reboot and park start the redo clean.
    fn reconnect(&mut self) -> Result<(), Stop> {
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
            "  adapter back after {} s: reboot, park, redo",
            t0.elapsed().as_secs()
        ));
        let lost = |e: anyhow::Error| match kind(&e, stopped()) {
            Kind::Interrupted => Stop::Interrupted,
            Kind::Lost => Stop::AdapterLost,
            Kind::Other => Stop::Error(e),
        };
        let (id, center) = (self.id, self.env.limits.center);
        let r = self.client()?.reboot(id);
        r.map_err(|e| lost(e.into()))?;
        pause(REBOOT_WAIT)?;
        park::park(self.client()?, id, center).map_err(lost)
    }

    /// Every run ends at centre, torque off. Ctrl-c is cleared first so the
    /// park drives; a second ctrl-c cuts the drive short and still torques
    /// off.
    fn finish(&mut self) {
        STOP.store(false, Ordering::SeqCst);
        let (id, center) = (self.id, self.env.limits.center);
        match self.c.as_mut() {
            Some(c) => {
                if let Err(e) = park::park(c, id, center) {
                    self.log.line(format_args!("park failed: {e:#}"));
                }
            }
            None => self
                .log
                .line("no adapter: the servo was left where it stopped"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::envelope;
    use crate::capture::store::fixture::tmp;
    use crate::capture::verdict::expected_segments;
    use crate::sweep::Decay;
    use osc_client::ResultCode;

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
        let store = Store::new(root.join("mg90-a__2s"));
        let decl = |servo: &str, supply| Decl {
            servo: servo.into(),
            supply,
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
    fn cfg_runs_the_plan_both_ways_between_the_envelope_guards() {
        let p = Procedure::parse(include_str!("session.toml")).unwrap();
        let mut env = envelope::mg90();
        env.windows_ms = crate::capture::pilot::windows(env.v_ss.used(), env.limits.runway);
        let plans = plan::expand(&p, &env, 2).unwrap();
        let c = cfg(&p, &env, &plans[1]);
        assert_eq!(c.steps, plans[1].schedule);
        assert_eq!(c.decay, Decay::Fast);
        assert_eq!(c.dirs, Dirs::Both);
        assert_eq!(c.guard, (532, 3526));
        // the bench session's pacing
        assert_eq!(
            (c.rest_ms, c.baseline_ms, c.seek_duty_pct, c.tel_mask),
            (1500, 1000, 15, 0x1cd)
        );
        assert!(!c.stall && !c.static_load);
        assert_eq!(
            expected_segments(c.baseline_ms > 0, c.dirs.signs().len(), c.steps.len()),
            41
        );
    }
}
