//! The module as a page calls it, in wasm: the simulated servo's run
//! through `IdentRun.runFake`, told to a JS callback, its files handed
//! back. `wasm-pack test --node`.
#![cfg(all(target_arch = "wasm32", feature = "fake"))]

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use std::collections::BTreeMap;

use osc_client_web_ident::{
    DeviceKey, IdentRun, ResumeArgs, ResumePlan, RunArgs, RunFile, RunOutcome, find_device,
    plan_resume,
};
use tsify::Tsify;
use wasm_bindgen::prelude::*;
use wasm_bindgen_test::*;

fn args() -> RunArgs {
    let file = |name: &str, bytes: &[u8]| RunFile {
        path: format!("profiles/mg90-d/{name}"),
        bytes: serde_bytes::ByteBuf::from(bytes.to_vec()),
    };
    RunArgs {
        id: 1,
        nameplate: "profiles/mg90-d/nameplate.json".into(),
        policy: "profiles/mg90-d/policy.json".into(),
        tries: None,
        baud: None,
        files: vec![
            file(
                "nameplate.json",
                include_bytes!("../../../measure/testdata/mg90-d/nameplate.json"),
            ),
            file(
                "policy.json",
                include_bytes!("../../../measure/testdata/mg90-d/policy.json"),
            ),
            file(
                "thermal.json",
                include_bytes!("../../../measure/testdata/mg90-d/thermal.json"),
            ),
        ],
    }
}

fn get(v: &JsValue, key: &str) -> JsValue {
    js_sys::Reflect::get(v, &key.into()).expect("a property")
}

/// The events told of `kind`, each as `[step, state, why]`.
fn events(told: &js_sys::Array, kind: &str) -> Vec<(String, Option<String>, JsValue)> {
    told.iter()
        .filter(|p| get(p, "event").as_string().as_deref() == Some(kind))
        .map(|p| {
            (
                get(&p, "step").as_string().unwrap_or_default(),
                get(&p, "state").as_string(),
                get(&p, "why"),
            )
        })
        .collect()
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_name = setInterval)]
    fn set_interval(f: &js_sys::Function, ms: i32) -> JsValue;
    #[wasm_bindgen(js_name = clearInterval)]
    fn clear_interval(id: &JsValue);
}

/// The last step start or end told, as `started probe`.
fn last_mark(told: &js_sys::Array) -> Option<String> {
    told.iter()
        .filter_map(|p| {
            let event = get(&p, "event").as_string()?;
            (event == "started" || event == "ended").then(|| {
                format!(
                    "{event} {}",
                    get(&p, "step").as_string().unwrap_or_default()
                )
            })
        })
        .next_back()
}

/// A page's timer: `at` runs at every turn the run gives the page, until
/// the ticker is dropped.
struct Ticker {
    id: JsValue,
    _tick: Closure<dyn FnMut()>,
}

impl Ticker {
    fn every_turn(at: impl FnMut() + 'static) -> Ticker {
        let tick = Closure::<dyn FnMut()>::new(at);
        let id = set_interval(tick.as_ref().unchecked_ref(), 0);
        Ticker { id, _tick: tick }
    }
}

impl Drop for Ticker {
    fn drop(&mut self) {
        clear_interval(&self.id);
    }
}

fn manifest(o: &RunOutcome) -> serde_json::Value {
    let f = o
        .files
        .iter()
        .find(|f| f.path.ends_with("/run.json"))
        .expect("run.json");
    serde_json::from_slice(&f.bytes).expect("json")
}

/// The run on the simulated servo stops at the probe: the callback is told
/// each step's start, its units and its end as JSON (a `why` of none as
/// null) and every
/// line the outcome carries; the outcome holds `run.json` in the servo's
/// run folder; the handle runs once.
#[wasm_bindgen_test]
async fn a_page_run_tells_its_callback_and_hands_back_its_folder() {
    let told = js_sys::Array::new();
    let keep = told.clone();
    let push = Closure::<dyn Fn(JsValue)>::new(move |p: JsValue| {
        keep.push(&p);
    });
    let h = IdentRun::new();
    let o: RunOutcome = h
        .run_fake(
            args().into_ts().expect("args"),
            push.as_ref().unchecked_ref(),
            None,
            None,
        )
        .await
        .map_err(JsValue::from)
        .expect("runs")
        .to_rust()
        .expect("outcome");
    let started: Vec<String> = events(&told, "started").into_iter().map(|e| e.0).collect();
    assert_eq!(started, ["rest", "probe"]);
    let ended = events(&told, "ended");
    assert_eq!(ended[0].0, "rest");
    assert_eq!(ended[0].1.as_deref(), Some("accepted"));
    assert!(ended[0].2.is_null(), "{:?}", ended[0].2);
    assert_eq!(
        (ended[1].0.as_str(), ended[1].1.as_deref()),
        ("probe", Some("aborted"))
    );
    assert!(ended[1].2.as_string().is_some());
    let units: Vec<(String, f64, f64, String)> = told
        .iter()
        .filter(|p| get(p, "event").as_string().as_deref() == Some("unit"))
        .map(|p| {
            (
                get(&p, "step").as_string().unwrap_or_default(),
                get(&p, "at").as_f64().unwrap_or_default(),
                get(&p, "of").as_f64().unwrap_or_default(),
                get(&p, "what").as_string().unwrap_or_default(),
            )
        })
        .collect();
    assert_eq!(
        units,
        [
            ("rest".to_string(), 1.0, 2.0, "burst".to_string()),
            ("rest".to_string(), 2.0, 2.0, "burst".to_string()),
            ("probe".to_string(), 1.0, 17.0, "burst".to_string())
        ]
    );
    let lines: Vec<String> = told
        .iter()
        .filter(|p| get(p, "event").as_string().as_deref() == Some("lines"))
        .flat_map(|p| {
            js_sys::Array::from(&get(&p, "lines"))
                .iter()
                .map(|l| l.as_string().expect("a line"))
                .collect::<Vec<_>>()
        })
        .collect();
    assert_eq!(lines, o.lines);
    let root = o.root.clone().expect("a run folder");
    assert!(
        o.files
            .iter()
            .all(|f| f.path.starts_with(&format!("{root}/")))
    );
    let m = manifest(&o);
    assert_eq!(m["steps"]["probe"]["state"], "aborted", "{m}");
    assert_eq!(o.refused, None);
    let again = h
        .run_fake(
            args().into_ts().expect("args"),
            push.as_ref().unchecked_ref(),
            None,
            None,
        )
        .await;
    assert!(again.is_err(), "a handle runs once");
}

/// Cancel from the page's callback as the probe starts: the probe ends
/// interrupted, its way out run, and the run stops there.
#[wasm_bindgen_test]
async fn a_stop_from_the_page_ends_the_step_through_its_way_out() {
    let h = Rc::new(IdentRun::new());
    let stop = h.clone();
    let cancel = Closure::<dyn Fn(JsValue)>::new(move |p: JsValue| {
        if get(&p, "event").as_string().as_deref() == Some("started")
            && get(&p, "step").as_string().as_deref() == Some("probe")
        {
            stop.stop();
        }
    });
    let o: RunOutcome = h
        .run_fake(
            args().into_ts().expect("args"),
            cancel.as_ref().unchecked_ref(),
            None,
            None,
        )
        .await
        .map_err(JsValue::from)
        .expect("runs")
        .to_rust()
        .expect("outcome");
    assert!(h.stopped());
    let m = manifest(&o);
    assert_eq!(m["steps"]["probe"]["state"], "interrupted", "{m}");
    assert!(
        o.lines
            .iter()
            .any(|l| l == "  torque_enable 0, read back 0"),
        "{:?}",
        o.lines
    );
}

/// The run gives the page turns as it goes: a timer the page set fires
/// between the rest's start and its end, and again in the probe, before
/// the run resolves.
#[wasm_bindgen_test]
async fn the_page_gets_turns_while_the_run_goes() {
    let told = js_sys::Array::new();
    let keep = told.clone();
    let push = Closure::<dyn Fn(JsValue)>::new(move |p: JsValue| {
        keep.push(&p);
    });
    let seen = Rc::new(RefCell::new(Vec::new()));
    let (look, note) = (told.clone(), seen.clone());
    let ticker = Ticker::every_turn(move || {
        note.borrow_mut().push(last_mark(&look));
    });
    let h = IdentRun::new();
    let o: RunOutcome = h
        .run_fake(
            args().into_ts().expect("args"),
            push.as_ref().unchecked_ref(),
            None,
            None,
        )
        .await
        .map_err(JsValue::from)
        .expect("runs")
        .to_rust()
        .expect("outcome");
    drop(ticker);
    assert_eq!(manifest(&o)["steps"]["probe"]["state"], "aborted");
    let seen = seen.take();
    for mark in ["started rest", "started probe"] {
        assert!(
            seen.iter().any(|m| m.as_deref() == Some(mark)),
            "no turn after `{mark}` before the step ended: {seen:?}"
        );
    }
}

/// Cancel from a page timer, not from the run's callback: raised at a turn
/// inside the rest, it lands at the rest's next rung, the rest ends
/// interrupted through its way out, and the run stops there.
#[wasm_bindgen_test]
async fn a_stop_from_a_page_timer_lands_mid_step() {
    let told = js_sys::Array::new();
    let keep = told.clone();
    let push = Closure::<dyn Fn(JsValue)>::new(move |p: JsValue| {
        keep.push(&p);
    });
    let h = Rc::new(IdentRun::new());
    let raised = Rc::new(Cell::new(false));
    let (look, stop, flag) = (told.clone(), h.clone(), raised.clone());
    let ticker = Ticker::every_turn(move || {
        if !flag.get() && last_mark(&look).as_deref() == Some("started rest") {
            stop.stop();
            flag.set(true);
        }
    });
    let o: RunOutcome = h
        .run_fake(
            args().into_ts().expect("args"),
            push.as_ref().unchecked_ref(),
            None,
            None,
        )
        .await
        .map_err(JsValue::from)
        .expect("runs")
        .to_rust()
        .expect("outcome");
    drop(ticker);
    assert!(raised.get() && h.stopped(), "the timer raised the stop");
    let m = manifest(&o);
    assert_eq!(m["steps"]["rest"]["state"], "interrupted", "{m}");
    assert_eq!(
        m["steps"]["rest"]["why"], "interrupted: the step stopped between rungs",
        "{m}"
    );
    let started: Vec<String> = events(&told, "started").into_iter().map(|e| e.0).collect();
    assert_eq!(started, ["rest"]);
    let said = |head: &str| o.lines.iter().any(|l| l.starts_with(head));
    assert!(
        said("rest row: ") && !said("raw row: "),
        "the stop lands between the rest's two rows: {:?}",
        o.lines
    );
    let out = o
        .lines
        .iter()
        .position(|l| l == "the way out:")
        .expect("the rest's way out");
    assert!(
        o.lines[out..]
            .iter()
            .any(|l| l == "  torque_enable 0, read back 0"),
        "{:?}",
        o.lines
    );
}

#[wasm_bindgen(inline_js = r#"
export function grant(devices) {
    const nav = globalThis.navigator ?? (globalThis.navigator = {});
    const usb = { getDevices: async () => devices };
    Object.defineProperty(nav, "usb", { value: usb, configurable: true, writable: true });
}
export function device(vendorId, productId, serialNumber, name) {
    return { vendorId, productId, serialNumber, name };
}
"#)]
extern "C" {
    /// Stands in for the site's granted devices behind `navigator.usb`.
    fn grant(devices: &js_sys::Array);
    fn device(vendor: u16, product: u16, serial: Option<String>, name: &str) -> JsValue;
}

fn key(serial: Option<&str>) -> tsify::Ts<DeviceKey> {
    DeviceKey {
        vendor_id: 0x1209,
        product_id: 0x0001,
        serial_number: serial.map(Into::into),
    }
    .into_ts()
    .expect("key")
}

/// A worker finds the lent adapter among the site's devices by its vendor,
/// product and serial number, and refuses a key that names none, or one
/// that two devices share.
#[wasm_bindgen_test]
async fn a_worker_finds_the_lent_device_by_its_key() {
    let all = js_sys::Array::of3(
        &device(0x1209, 0x0001, Some("A1".into()), "first"),
        &device(0x1209, 0x0001, Some("B2".into()), "second"),
        &device(0x2e8a, 0x0001, Some("B2".into()), "other vendor"),
    );
    grant(&all);
    let found: JsValue = find_device(key(Some("B2"))).await.expect("found").into();
    assert_eq!(get(&found, "name").as_string().as_deref(), Some("second"));

    let gone = find_device(key(Some("C3"))).await.err().map(JsValue::from);
    let why = gone.map(|e| get(&e, "message").as_string().unwrap_or_default());
    assert!(
        why.as_deref().is_some_and(|w| w.contains("not among")),
        "{why:?}"
    );

    grant(&js_sys::Array::of2(
        &device(0x1209, 0x0001, None, "first"),
        &device(0x1209, 0x0001, None, "second"),
    ));
    let twins = find_device(key(None)).await.err().map(JsValue::from);
    let why = twins.map(|e| get(&e, "message").as_string().unwrap_or_default());
    assert!(
        why.as_deref().is_some_and(|w| w.contains("several")),
        "{why:?}"
    );
}

/// A run folder as a page's mirror holds it: each change the run told,
/// replayed in order onto `files`.
fn replay(files: &mut BTreeMap<String, Vec<u8>>, told: &js_sys::Array) {
    for s in told.iter() {
        match get(&s, "op").as_string().as_deref() {
            Some("write") => {
                let bytes = js_sys::Uint8Array::from(get(&s, "bytes")).to_vec();
                files.insert(get(&s, "path").as_string().expect("a path"), bytes);
            }
            Some("move") => {
                let (from, to) = (
                    get(&s, "from").as_string().expect("from"),
                    get(&s, "to").as_string().expect("to"),
                );
                let below = format!("{from}/");
                let moved: Vec<String> = files
                    .keys()
                    .filter(|k| **k == from || k.starts_with(&below))
                    .cloned()
                    .collect();
                assert!(!moved.is_empty(), "{from} is there to move");
                for k in moved {
                    let bytes = files.remove(&k).expect("there");
                    files.insert(format!("{to}{}", &k[from.len()..]), bytes);
                }
            }
            op => panic!("an unknown change {op:?}"),
        }
    }
}

fn keep(into: &js_sys::Array) -> Closure<dyn Fn(JsValue)> {
    let into = into.clone();
    Closure::<dyn Fn(JsValue)>::new(move |v: JsValue| {
        into.push(&v);
    })
}

fn outcome_files(o: &RunOutcome) -> BTreeMap<String, Vec<u8>> {
    o.files
        .iter()
        .map(|f| (f.path.clone(), f.bytes.to_vec()))
        .collect()
}

/// The folder's files by their paths in it, the host clock's stamps zeroed
/// and the segments' records and files left out.
fn unsegmented(files: &BTreeMap<String, Vec<u8>>) -> BTreeMap<String, Vec<u8>> {
    fn mask(v: &mut serde_json::Value) {
        match v {
            serde_json::Value::Object(o) => {
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
            serde_json::Value::Array(a) => a.iter_mut().for_each(mask),
            _ => {}
        }
    }
    let in_folder = |p: &str| p.splitn(3, '/').nth(2).unwrap_or_default().to_string();
    files
        .iter()
        .map(|(p, b)| (in_folder(p), b))
        .filter(|(p, _)| !p.starts_with("segments/"))
        .map(
            |(p, b)| match serde_json::from_slice::<serde_json::Value>(b) {
                Ok(mut v) if p.ends_with(".json") => {
                    mask(&mut v);
                    (p, serde_json::to_vec(&v).expect("json"))
                }
                _ => (p, b.clone()),
            },
        )
        .collect()
}

fn resume_args(folder: &str, files: &BTreeMap<String, Vec<u8>>) -> ResumeArgs {
    let a = args();
    let mut all = a.files;
    all.extend(files.iter().map(|(path, bytes)| RunFile {
        path: path.clone(),
        bytes: serde_bytes::ByteBuf::from(bytes.clone()),
    }));
    ResumeArgs {
        id: a.id,
        nameplate: a.nameplate,
        policy: a.policy,
        baud: None,
        folder: folder.into(),
        files: all,
    }
}

/// Cancel as the probe starts, the folder as the page mirrored it from
/// `onStore`, then `resumeFake` on it a quarter of an hour later: the
/// mirror held the folder whole,
/// the resume goes on from the probe in segment 2, its own changes
/// replayed onto the folder give its outcome, and the folder ends as an
/// unbroken run's, the host's stamps, the segments and the probe's
/// readings of the sim's noise aside.
#[wasm_bindgen_test]
async fn a_stopped_page_run_resumes_from_its_mirror() {
    let quiet = Closure::<dyn Fn(JsValue)>::new(|_: JsValue| {});
    let whole: RunOutcome = IdentRun::new()
        .run_fake(
            args().into_ts().expect("args"),
            quiet.as_ref().unchecked_ref(),
            None,
            None,
        )
        .await
        .map_err(JsValue::from)
        .expect("runs")
        .to_rust()
        .expect("outcome");

    let h = Rc::new(IdentRun::new());
    let stop = h.clone();
    let cancel = Closure::<dyn Fn(JsValue)>::new(move |p: JsValue| {
        if get(&p, "event").as_string().as_deref() == Some("started")
            && get(&p, "step").as_string().as_deref() == Some("probe")
        {
            stop.stop();
        }
    });
    let changes = js_sys::Array::new();
    let mirror = keep(&changes);
    let stopped: RunOutcome = h
        .run_fake(
            args().into_ts().expect("args"),
            cancel.as_ref().unchecked_ref(),
            Some(mirror.as_ref().unchecked_ref::<js_sys::Function>().clone()),
            None,
        )
        .await
        .map_err(JsValue::from)
        .expect("runs")
        .to_rust()
        .expect("outcome");
    let mut folder = BTreeMap::new();
    replay(&mut folder, &changes);
    assert_eq!(folder, outcome_files(&stopped));
    let root = stopped.root.clone().expect("a run folder");

    let plan: ResumePlan = plan_resume(resume_args(&root, &folder).into_ts().expect("args"), None)
        .map_err(JsValue::from)
        .expect("plans")
        .to_rust()
        .expect("plan");
    assert_eq!((plan.from.as_str(), plan.segment), ("probe", 2));

    let told = js_sys::Array::new();
    let feed = keep(&told);
    let more = js_sys::Array::new();
    let mirror = keep(&more);
    // A quarter of an hour on: the winding has cooled, so the sim spends no
    // simulated minutes waiting for it.
    let later = Closure::<dyn Fn() -> f64>::new(|| js_sys::Date::now() / 1000.0 + 900.0);
    let resumed: RunOutcome = IdentRun::new()
        .resume_fake(
            resume_args(&root, &folder).into_ts().expect("args"),
            feed.as_ref().unchecked_ref(),
            Some(mirror.as_ref().unchecked_ref::<js_sys::Function>().clone()),
            Some(later.as_ref().unchecked_ref::<js_sys::Function>().clone()),
        )
        .await
        .map_err(JsValue::from)
        .expect("resumes")
        .to_rust()
        .expect("outcome");
    assert_eq!(resumed.refused, None, "{:#?}", resumed.lines);
    replay(&mut folder, &more);
    assert_eq!(folder, outcome_files(&resumed));
    let m = manifest(&resumed);
    assert_eq!(m["segments"][1]["from"], "probe", "{m}");
    assert_eq!(m["steps"]["probe"]["segment"], 2, "{m}");
    assert!(folder.contains_key(&format!("{root}/segments/1/results/probe.json")));
    let started: Vec<String> = events(&told, "started").into_iter().map(|e| e.0).collect();
    assert_eq!(started, ["resume", "probe"]);

    let probe = "results/probe.json".to_string();
    let (mut again, mut once) = (unsegmented(&folder), unsegmented(&outcome_files(&whole)));
    for (k, within) in [
        ("rail_v", 5.0 / 1241.0 * 1.5),
        ("position_before_counts", 1.5),
        ("position_after_counts", 1.5),
    ] {
        let take = |files: &mut BTreeMap<String, Vec<u8>>| {
            let b = files.get_mut(&probe).expect("probe");
            let mut v: serde_json::Value = serde_json::from_slice(b).expect("json");
            let x = v[k].as_f64().expect(k);
            v.as_object_mut().expect("object").remove(k);
            *b = serde_json::to_vec(&v).expect("json");
            x
        };
        let (r, w) = (take(&mut again), take(&mut once));
        assert!((r - w).abs() <= within, "{k}: {r} against {w}");
    }
    assert_eq!(again, once);
}

/// A folder a resume refuses on its files alone: `planResume` throws the
/// tool's words, and `resumeFake` ends refused in them, nothing told to the
/// mirror.
#[wasm_bindgen_test]
async fn a_resume_the_folder_refuses_is_refused_in_the_tool_s_words() {
    let quiet = Closure::<dyn Fn(JsValue)>::new(|_: JsValue| {});
    let whole: RunOutcome = IdentRun::new()
        .run_fake(
            args().into_ts().expect("args"),
            quiet.as_ref().unchecked_ref(),
            None,
            None,
        )
        .await
        .map_err(JsValue::from)
        .expect("runs")
        .to_rust()
        .expect("outcome");
    let root = whole.root.clone().expect("a run folder");
    let mut folder = outcome_files(&whole);
    folder.retain(|p, _| !p.ends_with("/run.json"));
    let words = format!("{root}: no run.json: not a run folder");
    let thrown = plan_resume(resume_args(&root, &folder).into_ts().expect("args"), None)
        .err()
        .map(JsValue::from)
        .map(|e| get(&e, "message").as_string().unwrap_or_default());
    assert_eq!(thrown.as_deref(), Some(words.as_str()));
    let more = js_sys::Array::new();
    let mirror = keep(&more);
    let o: RunOutcome = IdentRun::new()
        .resume_fake(
            resume_args(&root, &folder).into_ts().expect("args"),
            quiet.as_ref().unchecked_ref(),
            Some(mirror.as_ref().unchecked_ref::<js_sys::Function>().clone()),
            None,
        )
        .await
        .map_err(JsValue::from)
        .expect("resolves")
        .to_rust()
        .expect("outcome");
    assert_eq!(o.refused.as_deref(), Some(words.as_str()));
    assert_eq!(more.length(), 0);
}
