//! The identification run in the browser: osc-measure's whole run, the
//! CLI's `run` verb step for step ([`run`]), over the osc-adapter by WebUSB,
//! the run folder handed back as files. A wasm package of its own, loaded
//! only on the run screen, so every other page keeps osc-client-web alone.
//!
//! The two modules hold separate memories and cannot share a client, so the
//! device changes hands inside the bus manager's exclusive turn: the page's
//! `OscClient.release()` closes it and hands it over, `IdentRun.run` opens
//! it, runs and closes it, and `OscClient.reclaim()` opens it again. `run`
//! refuses a device that is still open, so the bus has one owner at a time.
//!
//! The run's files stay in memory: a store the run reads and writes as it
//! goes must answer at once, and OPFS answers a page only through promises.
//! They come back with the outcome, each at its path from the store's root
//! (`<uid>/<UTC>/...`) with the bytes the CLI writes, for the page to copy
//! into OPFS at the same paths. A page closed mid-run leaves the servo as a
//! killed CLI does: each drive ends under the bus dead-man, and the run's
//! volatile writes stay until a reboot.

#[cfg(feature = "fake")]
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
mod fake;
pub mod run;

#[cfg(target_arch = "wasm32")]
use std::cell::Cell;
#[cfg(target_arch = "wasm32")]
use std::sync::atomic::{AtomicBool, Ordering};

use osc_client::BaudRate as WireBaud;
use serde::{Deserialize, Serialize};
use tsify::Tsify;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(typescript_custom_section)]
const PROGRESS: &str = r#"
/** What a run tells its feed as it goes, each step by its key in run.json. */
export type Progress =
  | { event: "started"; step: string }
  | { event: "try"; step: string; at: number; of: number }
  | { event: "unit"; step: string; at: number; of: number; what: string }
  | { event: "lines"; lines: string[] }
  | { event: "ended"; step: string; state: string; why: string | null };
"#;

/// A bus rate, as osc-client-web names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub enum BaudRate {
    B500000,
    B1000000,
    B2000000,
    B3000000,
}

impl From<BaudRate> for WireBaud {
    fn from(r: BaudRate) -> Self {
        match r {
            BaudRate::B500000 => WireBaud::B500000,
            BaudRate::B1000000 => WireBaud::B1000000,
            BaudRate::B2000000 => WireBaud::B2000000,
            BaudRate::B3000000 => WireBaud::B3000000,
        }
    }
}

/// A file by its path in the run's store.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Tsify)]
pub struct RunFile {
    pub path: String,
    #[tsify(type = "Uint8Array")]
    pub bytes: serde_bytes::ByteBuf,
}

/// The CLI's `run` arguments, and the files they name.
#[derive(Debug, Clone, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct RunArgs {
    /// The servo's bus id (`--id`).
    pub id: u8,
    /// The profile's path in `files` (`--nameplate`).
    pub nameplate: String,
    /// The policy's path in `files` (`--policy`); its file name gives the
    /// policy's key, as on the command line.
    pub policy: String,
    /// The tune's tries at most (`--tries`), 6 when not given.
    #[tsify(optional)]
    pub tries: Option<u32>,
    /// The bus rate (`--baud`); found when not given.
    #[tsify(optional)]
    pub baud: Option<BaudRate>,
    /// The profile, the policy and, beside a version 0 nameplate, its
    /// `thermal.json`.
    pub files: Vec<RunFile>,
}

impl From<RunArgs> for run::Given {
    fn from(a: RunArgs) -> Self {
        run::Given {
            id: a.id,
            nameplate: a.nameplate,
            policy: a.policy,
            tries: a.tries.unwrap_or(osc_measure::tune::TRIES),
            baud: a.baud.map(Into::into),
            files: a
                .files
                .into_iter()
                .map(|f| (f.path, f.bytes.into_vec()))
                .collect(),
        }
    }
}

/// What a run leaves.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Tsify)]
pub struct RunOutcome {
    /// The run folder `<uid>/<UTC>`, once one was named.
    #[tsify(optional)]
    pub root: Option<String>,
    /// Every file the run wrote, at its path from the store's root
    /// (`<uid>/<UTC>/run.json`), with the bytes the CLI writes.
    pub files: Vec<RunFile>,
    /// Every line told to the feed, in order: what the CLI prints.
    pub lines: Vec<String>,
    /// Whether the tune accepted a set.
    pub accepted: bool,
    /// Why the run ended short, as the CLI prints it after `refused: `.
    #[tsify(optional)]
    pub refused: Option<String>,
}

impl From<run::Outcome> for RunOutcome {
    fn from(o: run::Outcome) -> Self {
        RunOutcome {
            root: o.root(),
            files: o
                .files
                .into_iter()
                .map(|(path, bytes)| RunFile {
                    path,
                    bytes: serde_bytes::ByteBuf::from(bytes),
                })
                .collect(),
            lines: o.lines,
            accepted: o.accepted,
            refused: o.refused,
        }
    }
}

/// One identification run. Made before the run starts, so the page's
/// Cancel reaches its stop flag while the run holds the bus; a handle runs
/// once.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
#[derive(Default)]
pub struct IdentRun {
    stop: AtomicBool,
    used: Cell<bool>,
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
impl IdentRun {
    #[wasm_bindgen(constructor)]
    pub fn new() -> IdentRun {
        IdentRun::default()
    }

    /// Raise the stop flag, as Ctrl-C raises the CLI's: the step stops
    /// between rungs, a burst in flight runs out, and the step ends through
    /// its way out, torque off. A stop raised before `run` stops the run at
    /// its first step.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }

    #[wasm_bindgen(getter)]
    pub fn stopped(&self) -> bool {
        self.stop.load(Ordering::SeqCst)
    }

    /// The run on `device`, which the page's client has released (a device
    /// still open is refused before anything is sent). `onProgress` is told
    /// each `Progress` as it happens; `clock` gives unix seconds, the
    /// page's own `Date.now() / 1000` when not given. The device is closed
    /// when the run ends, for `OscClient.reclaim()`. Resolves on every
    /// run, a refused one too, with what it wrote.
    pub async fn run(
        &self,
        device: osc_client::webusb::UsbDevice,
        args: tsify::Ts<RunArgs>,
        #[wasm_bindgen(js_name = onProgress, unchecked_param_type = "(p: Progress) => void")]
        on_progress: &js_sys::Function,
        #[wasm_bindgen(unchecked_optional_param_type = "() => number")] clock: Option<
            js_sys::Function,
        >,
    ) -> Result<tsify::Ts<RunOutcome>, JsError> {
        use osc_client::webusb::{WebUsbPipe, close_device};
        if device.opened() {
            return Err(JsError::new(
                "run: the device is open: the page's client still holds the bus; release it first",
            ));
        }
        let given = self.take(args)?;
        let open = async {
            WebUsbPipe::open(device.clone())
                .await
                .map_err(anyhow::Error::from)
        };
        let feed = feed(on_progress);
        let clock = clock_of(clock);
        let (o, c) = self.go(open, &given, &feed, &clock).await;
        drop(c);
        let mut o = RunOutcome::from(o);
        if let Err(e) = close_device(&device).await {
            let line = format!("closing the device: {e}");
            feed(&osc_measure::run::Progress::Lines {
                lines: vec![line.clone()],
            });
            o.lines.push(line);
        }
        Ok(tsify::Tsify::into_ts(&o)?)
    }

    /// `run` on one simulated servo at rest, at `args.id`, with osc-client-web's
    /// simulated UID for that id. No device, no user gesture; the sim runs
    /// no motor, so the run stops at the probe. Each exchange and each pause
    /// gives the page a turn, as a device's transfers do, so the page draws
    /// its progress and takes a Cancel as the run goes.
    #[cfg(feature = "fake")]
    #[wasm_bindgen(js_name = runFake)]
    pub async fn run_fake(
        &self,
        args: tsify::Ts<RunArgs>,
        #[wasm_bindgen(js_name = onProgress, unchecked_param_type = "(p: Progress) => void")]
        on_progress: &js_sys::Function,
        #[wasm_bindgen(unchecked_optional_param_type = "() => number")] clock: Option<
            js_sys::Function,
        >,
    ) -> Result<tsify::Ts<RunOutcome>, JsError> {
        let given = self.take(args)?;
        let id = given.id;
        let open = async move { Ok(fake::Yielding(fake::pipe(id))) };
        let (o, _) = self
            .go(open, &given, &feed(on_progress), &clock_of(clock))
            .await;
        Ok(tsify::Tsify::into_ts(&RunOutcome::from(o))?)
    }
}

#[cfg(target_arch = "wasm32")]
impl IdentRun {
    fn take(&self, args: tsify::Ts<RunArgs>) -> Result<run::Given, JsError> {
        if self.used.replace(true) {
            return Err(JsError::new(
                "run: this handle has run; another run takes a new IdentRun",
            ));
        }
        Ok(args.to_rust()?.into())
    }

    async fn go<P: osc_client::pipe::Pipe>(
        &self,
        open: impl std::future::Future<Output = anyhow::Result<P>>,
        given: &run::Given,
        feed: &dyn Fn(&osc_measure::run::Progress),
        clock: &dyn Fn() -> f64,
    ) -> (run::Outcome, Option<osc_client::Client<P>>) {
        let page = run::Page {
            feed,
            clock,
            stop: &self.stop,
        };
        run::run(open, given, &page).await
    }
}

/// The page's callback as the run's feed, each event as its JSON; a
/// callback that throws cannot stop the run, so its throw is dropped.
#[cfg(target_arch = "wasm32")]
fn feed(f: &js_sys::Function) -> impl Fn(&osc_measure::run::Progress) + '_ {
    move |p| {
        let json = serde_wasm_bindgen::Serializer::json_compatible();
        if let Ok(v) = p.serialize(&json) {
            let _ = f.call1(&JsValue::NULL, &v);
        }
    }
}

/// The page's clock, or `Date.now()` where it gives no number.
#[cfg(target_arch = "wasm32")]
fn clock_of(f: Option<js_sys::Function>) -> impl Fn() -> f64 {
    move || {
        f.as_ref()
            .and_then(|f| f.call0(&JsValue::NULL).ok())
            .and_then(|v| v.as_f64())
            .filter(|t| t.is_finite())
            .unwrap_or_else(|| js_sys::Date::now() / 1000.0)
    }
}
