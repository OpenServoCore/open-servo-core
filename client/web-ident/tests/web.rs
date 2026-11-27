//! The module as a page calls it, in wasm: the simulated servo's run
//! through `IdentRun.runFake`, told to a JS callback, its files handed
//! back. `wasm-pack test --node`.
#![cfg(all(target_arch = "wasm32", feature = "fake"))]

use std::rc::Rc;

use osc_client_web_ident::{IdentRun, RunArgs, RunFile, RunOutcome};
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

fn manifest(o: &RunOutcome) -> serde_json::Value {
    let f = o
        .files
        .iter()
        .find(|f| f.path.ends_with("/run.json"))
        .expect("run.json");
    serde_json::from_slice(&f.bytes).expect("json")
}

/// The run on the simulated servo stops at the probe: the callback is told
/// each step's start and end as JSON (a `why` of none as null) and every
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
