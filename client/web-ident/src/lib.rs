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
//! A page runs it in a dedicated worker (`worker.js`, driven from the page
//! by `host.js`): a hidden page's timers are throttled, a dedicated worker's
//! are not, and a pause stretched past the dead-man stops the run. The
//! worker finds the lent device again with `findDevice`, since a
//! `USBDevice` cannot be posted to it.
//!
//! The run's files stay in memory: a store the run reads and writes as it
//! goes must answer at once, and OPFS answers a page only through promises.
//! Each write and move is told to `onStore` as it happens, by its path from
//! the store's root (`<uid>/<UTC>/...`) with the bytes the CLI writes, for
//! the page to mirror into OPFS in order; the outcome carries the folder
//! whole as well. A page closed mid-run leaves the servo as a killed CLI
//! does: each drive ends under the bus dead-man, and the run's volatile
//! writes stay until a reboot; the mirror holds the folder as far as the
//! page applied it, and `resume` goes on from it.
//!
//! `save` is the CLI's `save --set` of a run folder's accepted final set,
//! then its `check --set`, on the same terms: the page's "Save this set".

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
  | { event: "try"; step: string; at: number; of: number; percent: number }
  | { event: "unit"; step: string; at: number; of: number; what: string }
  | { event: "lines"; lines: string[] }
  | { event: "ended"; step: string; state: string; why: string | null };

/** A change to the run's store as it happens, by paths from the store's root. */
export type Stored =
  | { op: "write"; path: string; bytes: Uint8Array }
  | { op: "move"; from: string; to: string };
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

/// The CLI's `run --resume` arguments, and the files they name: the
/// profile and the policy, and every file of the run folder.
#[derive(Debug, Clone, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct ResumeArgs {
    /// The servo's bus id (`--id`).
    pub id: u8,
    /// The profile's path in `files` (`--nameplate`).
    pub nameplate: String,
    /// The policy's path in `files` (`--policy`).
    pub policy: String,
    /// The bus rate (`--baud`); found when not given.
    #[tsify(optional)]
    pub baud: Option<BaudRate>,
    /// The run folder `<uid>/<UTC>` (`--resume`); the tune's tries are its
    /// own.
    pub folder: String,
    /// The profile, the policy, and the run folder's files at their paths
    /// from the store's root (`<uid>/<UTC>/run.json`).
    pub files: Vec<RunFile>,
}

impl ResumeArgs {
    fn given(self) -> (run::Given, String) {
        let args = RunArgs {
            id: self.id,
            nameplate: self.nameplate,
            policy: self.policy,
            tries: None,
            baud: self.baud,
            files: self.files,
        };
        (args.into(), self.folder)
    }
}

/// What a resume would do, decided from the folder's files alone.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct ResumePlan {
    /// The step it restarts at, by its key in `run.json`.
    pub from: String,
    /// The new segment's number in `run.json`.
    pub segment: u32,
    /// The tune's tries, the run's.
    pub tries: u32,
    /// The wait for the winding to cool before the restart, s.
    pub cool_s: f64,
    /// Whether the high stop is pressed once first, to check it seats where
    /// it did.
    pub reseat: bool,
    /// Why it restarts where it does, a line each.
    pub why: Vec<String>,
    /// The stopped attempt's files, each to its place under `segments/`.
    pub moves: Vec<Moved>,
}

/// A path in the run folder moved to another.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Tsify)]
pub struct Moved {
    pub from: String,
    pub to: String,
}

impl From<osc_measure::run::resume::Plan> for ResumePlan {
    fn from(p: osc_measure::run::resume::Plan) -> Self {
        ResumePlan {
            from: p.from.into(),
            segment: p.n,
            tries: p.tries,
            cool_s: p.cool_s,
            reseat: p.reseat,
            why: p.why,
            moves: p
                .moves
                .into_iter()
                .map(|(from, to)| Moved { from, to })
                .collect(),
        }
    }
}

/// What a resume of `args.folder` would do at `now` (unix seconds; the
/// page's clock when not given), from its files alone: nothing opened,
/// nothing sent. Throws the refusal `resume` would end on before the bus,
/// in the tool's words.
#[wasm_bindgen(js_name = planResume)]
pub fn plan_resume(
    args: tsify::Ts<ResumeArgs>,
    now: Option<f64>,
) -> Result<tsify::Ts<ResumePlan>, JsError> {
    let (given, folder) = args.to_rust()?.given();
    let now = now.unwrap_or_else(now_s);
    let p = run::plan(&given, &folder, now).map_err(|e| JsError::new(&format!("{e:#}")))?;
    Ok(tsify::Tsify::into_ts(&ResumePlan::from(p))?)
}

#[cfg(target_arch = "wasm32")]
fn now_s() -> f64 {
    js_sys::Date::now() / 1000.0
}

#[cfg(not(target_arch = "wasm32"))]
fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// What a run leaves.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Tsify)]
pub struct RunOutcome {
    /// The run folder `<uid>/<UTC>`, once one was named.
    #[tsify(optional)]
    pub root: Option<String>,
    /// Every file of the run folder, at its path from the store's root
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

/// What "Save this set" is given: the CLI's `save --set` of a run
/// folder's accepted final set, and the folder's files it names.
#[derive(Debug, Clone, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct SaveArgs {
    /// The servo's bus id (`--id`).
    pub id: u8,
    /// The bus rate (`--baud`); found when not given.
    #[tsify(optional)]
    pub baud: Option<BaudRate>,
    /// The run folder `<uid>/<UTC>` whose final set is stored.
    pub folder: String,
    /// The folder's files at their paths from the store's root: at least
    /// `run.json`, the final set file it names and, for a LIVE table,
    /// `final/inputs/pos_lut.json`.
    pub files: Vec<RunFile>,
}

impl From<SaveArgs> for run::SaveGiven {
    fn from(a: SaveArgs) -> Self {
        run::SaveGiven {
            id: a.id,
            baud: a.baud.map(Into::into),
            folder: a.folder,
            files: a
                .files
                .into_iter()
                .map(|f| (f.path, f.bytes.into_vec()))
                .collect(),
        }
    }
}

/// What a save leaves.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct SaveOutcome {
    /// The lines `osc-ident save` prints, then `osc-ident check`'s.
    pub lines: Vec<String>,
    /// The save's own folder: its backup of the stored state and the state
    /// after the SAVE, at their paths from the store's root.
    pub files: Vec<RunFile>,
    /// Whether the SAVE landed and the servo booted as the set.
    pub saved: bool,
    /// The check's differences against the set, once it ran.
    #[tsify(optional)]
    pub differences: Option<Vec<String>>,
    /// The bus rate the servo now boots at.
    #[tsify(optional)]
    pub boots_at: Option<BaudRate>,
    /// Why the save ended short, as the CLI prints it after `refused: `.
    #[tsify(optional)]
    pub refused: Option<String>,
}

impl From<WireBaud> for BaudRate {
    fn from(r: WireBaud) -> Self {
        match r {
            WireBaud::B500000 => BaudRate::B500000,
            WireBaud::B1000000 => BaudRate::B1000000,
            WireBaud::B2000000 => BaudRate::B2000000,
            WireBaud::B3000000 => BaudRate::B3000000,
        }
    }
}

impl From<run::SaveOutcome> for SaveOutcome {
    fn from(o: run::SaveOutcome) -> Self {
        SaveOutcome {
            lines: o.lines,
            files: o
                .files
                .into_iter()
                .map(|(path, bytes)| RunFile {
                    path,
                    bytes: serde_bytes::ByteBuf::from(bytes),
                })
                .collect(),
            saved: o.saved,
            differences: o.differences,
            boots_at: o.boots_at.map(Into::into),
            refused: o.refused,
        }
    }
}

/// The device a page lent, by the `USBDevice` fields of the same names,
/// for a worker to find its own object for it (`findDevice`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct DeviceKey {
    pub vendor_id: u16,
    pub product_id: u16,
    #[tsify(optional)]
    pub serial_number: Option<String>,
}

#[cfg(target_arch = "wasm32")]
impl From<DeviceKey> for osc_client::webusb::DeviceKey {
    fn from(k: DeviceKey) -> Self {
        osc_client::webusb::DeviceKey {
            vendor_id: k.vendor_id,
            product_id: k.product_id,
            serial: k.serial_number,
        }
    }
}

/// The device `key` names among those this site was granted, found
/// without a prompt, so it works in a worker: a `USBDevice` cannot be posted
/// to one. Refused when none matches, or several do.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(js_name = findDevice)]
pub async fn find_device(
    key: tsify::Ts<DeviceKey>,
) -> Result<osc_client::webusb::UsbDevice, JsError> {
    let key = key.to_rust()?.into();
    osc_client::webusb::find_device(&key)
        .await
        .map_err(JsError::from)
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
    /// each `Progress` as it happens, `onStore` each change to the run's
    /// files; `clock` gives unix seconds, the page's own `Date.now() /
    /// 1000` when not given. The device is closed when the run ends, for
    /// `OscClient.reclaim()`. Resolves on every run, a refused one too,
    /// with what it wrote.
    pub async fn run(
        &self,
        device: osc_client::webusb::UsbDevice,
        args: tsify::Ts<RunArgs>,
        #[wasm_bindgen(js_name = onProgress, unchecked_param_type = "(p: Progress) => void")]
        on_progress: &js_sys::Function,
        #[wasm_bindgen(js_name = onStore, unchecked_optional_param_type = "(s: Stored) => void")]
        on_store: Option<js_sys::Function>,
        #[wasm_bindgen(unchecked_optional_param_type = "() => number")] clock: Option<
            js_sys::Function,
        >,
    ) -> Result<tsify::Ts<RunOutcome>, JsError> {
        let given = self.take(args.to_rust()?.into())?;
        self.on_device(device, &given, None, on_progress, on_store, clock)
            .await
    }

    /// The CLI's `run --resume` of `args.folder` on `device`, as `run` runs:
    /// the folder's own refusals come before the device is opened, then the
    /// checks against what the run began on, before anything drives. The
    /// outcome's files are the folder whole, as the resume leaves it.
    pub async fn resume(
        &self,
        device: osc_client::webusb::UsbDevice,
        args: tsify::Ts<ResumeArgs>,
        #[wasm_bindgen(js_name = onProgress, unchecked_param_type = "(p: Progress) => void")]
        on_progress: &js_sys::Function,
        #[wasm_bindgen(js_name = onStore, unchecked_optional_param_type = "(s: Stored) => void")]
        on_store: Option<js_sys::Function>,
        #[wasm_bindgen(unchecked_optional_param_type = "() => number")] clock: Option<
            js_sys::Function,
        >,
    ) -> Result<tsify::Ts<RunOutcome>, JsError> {
        let (given, folder) = args.to_rust()?.given();
        let given = self.take(given)?;
        self.on_device(device, &given, Some(&folder), on_progress, on_store, clock)
            .await
    }

    /// The CLI's `save --set` of the run folder's accepted final set, then
    /// its `check --set`, on `device`, as `run` runs: the folder's refusals
    /// come before the device is opened, a servo that is not the run's is
    /// refused before anything is written. The servo is rebooted onto its
    /// stored state and that state backed up, the set installed, checked,
    /// SAVEd with the bus rate it runs at, rebooted and read back. The
    /// device is closed when the save ends.
    pub async fn save(
        &self,
        device: osc_client::webusb::UsbDevice,
        args: tsify::Ts<SaveArgs>,
        #[wasm_bindgen(js_name = onProgress, unchecked_param_type = "(p: Progress) => void")]
        on_progress: &js_sys::Function,
        #[wasm_bindgen(js_name = onStore, unchecked_optional_param_type = "(s: Stored) => void")]
        on_store: Option<js_sys::Function>,
        #[wasm_bindgen(unchecked_optional_param_type = "() => number")] clock: Option<
            js_sys::Function,
        >,
    ) -> Result<tsify::Ts<SaveOutcome>, JsError> {
        use osc_client::webusb::{WebUsbPipe, close_device};
        self.once()?;
        let given: run::SaveGiven = args.to_rust()?.into();
        if device.opened() {
            return Err(JsError::new(
                "save: the device is open: the page's client still holds the bus; release it first",
            ));
        }
        let open = async {
            WebUsbPipe::open(device.clone())
                .await
                .map_err(anyhow::Error::from)
        };
        let feed = feed(on_progress);
        let page = run::Page {
            feed: &feed,
            clock: &clock_of(clock),
            stop: &self.stop,
            store: &stored(on_store),
        };
        let (o, c) = run::save(open, &given, &page).await;
        drop(c);
        let mut o = SaveOutcome::from(o);
        if let Err(e) = close_device(&device).await {
            let line = format!("closing the device: {e}");
            feed(&osc_measure::run::Progress::Lines {
                lines: vec![line.clone()],
            });
            o.lines.push(line);
        }
        Ok(tsify::Tsify::into_ts(&o)?)
    }

    /// `save` on the simulated servo of `runFake`: calibrated, at rest, its
    /// stored settings the seed's.
    #[cfg(feature = "fake")]
    #[wasm_bindgen(js_name = saveFake)]
    pub async fn save_fake(
        &self,
        args: tsify::Ts<SaveArgs>,
        #[wasm_bindgen(js_name = onProgress, unchecked_param_type = "(p: Progress) => void")]
        on_progress: &js_sys::Function,
        #[wasm_bindgen(js_name = onStore, unchecked_optional_param_type = "(s: Stored) => void")]
        on_store: Option<js_sys::Function>,
        #[wasm_bindgen(unchecked_optional_param_type = "() => number")] clock: Option<
            js_sys::Function,
        >,
    ) -> Result<tsify::Ts<SaveOutcome>, JsError> {
        self.once()?;
        let given: run::SaveGiven = args.to_rust()?.into();
        let id = given.id;
        let open = async move { Ok(fake::Yielding(fake::pipe(id))) };
        let feed = feed(on_progress);
        let page = run::Page {
            feed: &feed,
            clock: &clock_of(clock),
            stop: &self.stop,
            store: &stored(on_store),
        };
        let (o, _) = run::save(open, &given, &page).await;
        Ok(tsify::Tsify::into_ts(&SaveOutcome::from(o))?)
    }

    /// `run` on one simulated servo at rest, at `args.id`, with osc-client-web's
    /// simulated UID for that id. No device, no user gesture; the sim runs
    /// no motor, so the run stops at the probe. Each exchange and each pause
    /// gives the event loop a turn, as a device's transfers do, so a page
    /// draws its progress, and a page or a worker takes a Cancel, as the run
    /// goes.
    #[cfg(feature = "fake")]
    #[wasm_bindgen(js_name = runFake)]
    pub async fn run_fake(
        &self,
        args: tsify::Ts<RunArgs>,
        #[wasm_bindgen(js_name = onProgress, unchecked_param_type = "(p: Progress) => void")]
        on_progress: &js_sys::Function,
        #[wasm_bindgen(js_name = onStore, unchecked_optional_param_type = "(s: Stored) => void")]
        on_store: Option<js_sys::Function>,
        #[wasm_bindgen(unchecked_optional_param_type = "() => number")] clock: Option<
            js_sys::Function,
        >,
    ) -> Result<tsify::Ts<RunOutcome>, JsError> {
        let given = self.take(args.to_rust()?.into())?;
        self.on_fake(&given, None, on_progress, on_store, clock)
            .await
    }

    /// `resume` on the simulated servo of `runFake`, as it is when a page
    /// opens again: the same servo, its stored settings, at rest.
    #[cfg(feature = "fake")]
    #[wasm_bindgen(js_name = resumeFake)]
    pub async fn resume_fake(
        &self,
        args: tsify::Ts<ResumeArgs>,
        #[wasm_bindgen(js_name = onProgress, unchecked_param_type = "(p: Progress) => void")]
        on_progress: &js_sys::Function,
        #[wasm_bindgen(js_name = onStore, unchecked_optional_param_type = "(s: Stored) => void")]
        on_store: Option<js_sys::Function>,
        #[wasm_bindgen(unchecked_optional_param_type = "() => number")] clock: Option<
            js_sys::Function,
        >,
    ) -> Result<tsify::Ts<RunOutcome>, JsError> {
        let (given, folder) = args.to_rust()?.given();
        let given = self.take(given)?;
        self.on_fake(&given, Some(&folder), on_progress, on_store, clock)
            .await
    }
}

#[cfg(target_arch = "wasm32")]
impl IdentRun {
    fn take(&self, given: run::Given) -> Result<run::Given, JsError> {
        self.once()?;
        Ok(given)
    }

    fn once(&self) -> Result<(), JsError> {
        if self.used.replace(true) {
            return Err(JsError::new(
                "run: this handle has run; another run takes a new IdentRun",
            ));
        }
        Ok(())
    }

    async fn on_device(
        &self,
        device: osc_client::webusb::UsbDevice,
        given: &run::Given,
        folder: Option<&str>,
        on_progress: &js_sys::Function,
        on_store: Option<js_sys::Function>,
        clock: Option<js_sys::Function>,
    ) -> Result<tsify::Ts<RunOutcome>, JsError> {
        use osc_client::webusb::{WebUsbPipe, close_device};
        if device.opened() {
            return Err(JsError::new(
                "run: the device is open: the page's client still holds the bus; release it first",
            ));
        }
        let open = async {
            WebUsbPipe::open(device.clone())
                .await
                .map_err(anyhow::Error::from)
        };
        let feed = feed(on_progress);
        let (o, c) = self
            .go(
                open,
                given,
                folder,
                &feed,
                &stored(on_store),
                &clock_of(clock),
            )
            .await;
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

    #[cfg(feature = "fake")]
    async fn on_fake(
        &self,
        given: &run::Given,
        folder: Option<&str>,
        on_progress: &js_sys::Function,
        on_store: Option<js_sys::Function>,
        clock: Option<js_sys::Function>,
    ) -> Result<tsify::Ts<RunOutcome>, JsError> {
        let id = given.id;
        let open = async move { Ok(fake::Yielding(fake::pipe(id))) };
        let (o, _) = self
            .go(
                open,
                given,
                folder,
                &feed(on_progress),
                &stored(on_store),
                &clock_of(clock),
            )
            .await;
        Ok(tsify::Tsify::into_ts(&RunOutcome::from(o))?)
    }

    async fn go<P: osc_client::pipe::Pipe>(
        &self,
        open: impl std::future::Future<Output = anyhow::Result<P>>,
        given: &run::Given,
        folder: Option<&str>,
        feed: &dyn Fn(&osc_measure::run::Progress),
        store: &dyn Fn(run::Stored<'_>),
        clock: &dyn Fn() -> f64,
    ) -> (run::Outcome, Option<osc_client::Client<P>>) {
        let page = run::Page {
            feed,
            clock,
            stop: &self.stop,
            store,
        };
        match folder {
            Some(f) => run::resume(open, given, f, &page).await,
            None => run::run(open, given, &page).await,
        }
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
            .unwrap_or_else(now_s)
    }
}

/// The page's store callback, each change as a `Stored` object with its
/// bytes in a `Uint8Array` of their own; a callback that throws cannot stop
/// the run, so its throw is dropped.
#[cfg(target_arch = "wasm32")]
fn stored(f: Option<js_sys::Function>) -> impl Fn(run::Stored<'_>) {
    move |s| {
        let Some(f) = f.as_ref() else { return };
        let o = js_sys::Object::new();
        let set = |k: &str, v: &JsValue| {
            let _ = js_sys::Reflect::set(&o, &k.into(), v);
        };
        match s {
            run::Stored::Write { path, bytes } => {
                set("op", &"write".into());
                set("path", &path.into());
                set("bytes", &js_sys::Uint8Array::from(bytes).into());
            }
            run::Stored::Move { from, to } => {
                set("op", &"move".into());
                set("from", &from.into());
                set("to", &to.into());
            }
        }
        let _ = f.call1(&JsValue::NULL, &o);
    }
}
