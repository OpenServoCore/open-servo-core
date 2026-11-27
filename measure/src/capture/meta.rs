//! A capture as a run keeps it: `captures/<step>/NN-name.csv.gz`, the rows
//! exactly as received in osc-lab's CSV, and `NN-name.meta.json` beside
//! it: the row layout, the servo's sense block, the rung's plan, the host
//! times, and what the burst lost. gzip is pure Rust (miniz), so the
//! browser build writes the same bytes.

use std::io::Write as _;

use flate2::Compression;
use flate2::write::GzEncoder;
use osc_client::descriptor::Descriptor;
use serde_json::{Map, Value, json};

use super::burst::{self, Capture};
use super::rows::{chunks_pairs, chunks_text, csv};
use super::tel::{Burst, Line};
use crate::sink::{Sink, SinkError};
use crate::state::State;

/// The sense block a capture's counts convert through, read off the servo.
pub const SENSE: [&str; 14] = [
    "shunt_r_mohm",
    "gain_milli",
    "vmotor_div_top",
    "vmotor_div_bot",
    "vdd_mv",
    "vbus_div_top_ohm",
    "vbus_div_bot_ohm",
    "ntc_pullup_ohm",
    "ntc_r25_ohm",
    "ntc_beta",
    "vmotor_bias_nom_counts",
    "rail_drop_mv",
    "vmotor_bias_top_ohm",
    "vmotor_bias_bot_ohm",
];

/// What every capture's meta opens with, as osc-lab's does: the servo, its
/// tick rate, the PWM period its duty counts are of, the plant it ran and
/// its sense constants; and the image.
pub fn servo(state: &State, d: &Descriptor, line: &Line) -> Map<String, Value> {
    let num = |name: &str| d.field(name).map_or(Value::Null, |f| state.text(f));
    let sense: Map<String, Value> = SENSE.iter().map(|n| (n.to_string(), num(n))).collect();
    let mut m = Map::new();
    m.insert("model".into(), num("model_number"));
    m.insert("fw".into(), num("firmware_version"));
    m.insert("capability_flags".into(), line.caps.into());
    m.insert("image".into(), line.image().name().into());
    m.insert("tick_hz".into(), num("tick_hz"));
    m.insert("pwm_period_ticks".into(), num("pwm_period_ticks"));
    m.insert("i_window_min_ticks".into(), num("i_window_min_ticks"));
    m.insert(
        "plant".into(),
        json!({
            "lut_state": state.table_state(),
            "servo": state.hash(d),
            "data_state": state.data_said(),
        }),
    );
    m.insert("sense".into(), Value::Object(sense));
    m
}

/// A TEL capture's meta: the servo's block, the row, the plan the rung ran
/// under, the host's clock at the arm and after the LAST frame, and every
/// count of what the burst lost.
pub fn tel(servo: &Map<String, Value>, b: &Burst, plan: Value, host_s: (f64, f64)) -> Value {
    let mut m = servo.clone();
    let chunks = b.layout.chunks();
    m.insert("row".into(), b.layout.name().into());
    m.insert("tel_chunks".into(), json!(chunks_pairs(&chunks)));
    m.insert("tel_chunks_text".into(), chunks_text(&chunks).into());
    m.insert("rows_armed".into(), b.armed.into());
    m.insert("rows".into(), b.rows.len().into());
    m.insert("rows_dropped".into(), b.dropped.into());
    m.insert("frames".into(), b.frames.into());
    m.insert("seq_holes".into(), b.holes.into());
    m.insert("tick_gaps".into(), b.tick_gaps().into());
    m.insert("skipped".into(), b.skipped.into());
    m.insert("garble".into(), b.garble.into());
    m.insert("laps".into(), b.laps.into());
    m.insert("timed_out".into(), b.timed_out.into());
    m.insert("alert".into(), b.alert.into());
    m.insert(
        "first_stamp".into(),
        b.first_stamp.map_or(Value::Null, Value::from),
    );
    m.insert("lost".into(), json!(b.lost()));
    m.insert("plan".into(), plan);
    m.insert("host_t_start".into(), host_s.0.into());
    m.insert("host_t_end".into(), host_s.1.into());
    Value::Object(m)
}

/// A shunt burst's meta: the servo's block, the burst's header and its
/// pre-arm reads, the plan it ran under and the host's clock around it.
pub fn burst(servo: &Map<String, Value>, cap: &Capture, plan: Value, host_s: (f64, f64)) -> Value {
    let mut m = servo.clone();
    let h = &cap.meta;
    m.insert("capture".into(), "shunt burst".into());
    m.insert("samples".into(), cap.samples.len().into());
    m.insert(
        "header".into(),
        json!({
            "pre_q15": h.pre_q15, "step_q15": h.step_q15, "step_index": h.step_index,
            "start_cnt": h.start_cnt, "pwm_arr": h.pwm_arr, "start_dir": h.start_dir,
            "restore_dir": h.restore_dir, "chans": h.chans, "frame_len": h.frame_len,
        }),
    );
    m.insert(
        "pre_arm".into(),
        json!({
            "vbus_raw": h.vbus_raw, "bias": h.bias, "vmotor_bias": h.vmotor_bias, "pos": h.pos,
            "seated": h.seated, "drive_polarity": h.drive_polarity,
        }),
    );
    m.insert("plan".into(), plan);
    m.insert("host_t_start".into(), host_s.0.into());
    m.insert("host_t_end".into(), host_s.1.into());
    Value::Object(m)
}

/// Where burst `k` of `step` lands: osc-lab's `burst-K.csv`, plain so
/// `oscnb.bursts.load` reads the folder, and its meta beside it.
pub fn burst_paths(step: &str, k: u32) -> (String, String) {
    let base = format!("captures/{step}/burst-{k}");
    (format!("{base}.csv"), format!("{base}.meta.json"))
}

/// Keep one shunt burst: its codes as osc-lab writes them, then its meta.
pub fn keep_burst(
    sink: &mut dyn Sink,
    step: &str,
    k: u32,
    cap: &Capture,
    meta: &Value,
) -> Result<(String, String), SinkError> {
    let (csv, m) = burst_paths(step, k);
    sink.write(&csv, burst::to_csv(cap).as_bytes())?;
    let mut text = serde_json::to_vec_pretty(meta).map_err(|e| SinkError(e.to_string()))?;
    text.push(b'\n');
    sink.write(&m, &text)?;
    Ok((sink.shown(&csv), sink.shown(&m)))
}

/// Deterministic gzip: no name, no time, so equal rows are equal bytes.
pub fn gzip(bytes: &[u8]) -> Vec<u8> {
    let mut e = GzEncoder::new(Vec::new(), Compression::default());
    // a Vec never refuses a write
    let _ = e.write_all(bytes);
    e.finish().unwrap_or_default()
}

pub fn gunzip(bytes: &[u8]) -> std::io::Result<Vec<u8>> {
    use std::io::Read as _;
    let mut out = Vec::new();
    flate2::read::GzDecoder::new(bytes).read_to_end(&mut out)?;
    Ok(out)
}

/// Where capture `nn` named `name` of `step` lands: the rows, the meta.
pub fn paths(step: &str, nn: u32, name: &str) -> (String, String) {
    let base = format!("captures/{step}/{nn:02}-{name}");
    (format!("{base}.csv.gz"), format!("{base}.meta.json"))
}

/// Keep one TEL capture: its rows as received, gzipped, then its meta.
pub fn keep(
    sink: &mut dyn Sink,
    step: &str,
    nn: u32,
    b: &Burst,
    cmd_duty_q15: i16,
    meta: &Value,
) -> Result<(String, String), SinkError> {
    let (rows, m) = paths(step, nn, b.layout.name());
    sink.write(
        &rows,
        &gzip(csv(b.layout, 0, cmd_duty_q15, &b.rows).as_bytes()),
    )?;
    let mut text = serde_json::to_vec_pretty(meta).map_err(|e| SinkError(e.to_string()))?;
    text.push(b'\n');
    sink.write(&m, &text)?;
    Ok((sink.shown(&rows), sink.shown(&m)))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A committed `burst-0.csv` kept through a sink is the same bytes, and
    /// its meta carries the header.
    #[test]
    fn a_kept_burst_is_byte_compatible_with_osc_lab() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/captures/burst-0.csv"
        ))
        .expect("fixture");
        let cap = burst::from_csv(&text).expect("parses");
        let mut sink = crate::sink::MemSink::default();
        let m = burst(&Map::new(), &cap, json!({"rung": "burst 0"}), (1.0, 2.0));
        keep_burst(&mut sink, "probe", 0, &cap, &m).expect("kept");
        assert!(sink.files["captures/probe/burst-0.csv"] == text.as_bytes());
        let back: Value =
            serde_json::from_slice(&sink.files["captures/probe/burst-0.meta.json"]).expect("meta");
        assert_eq!(back["header"]["pwm_arr"], cap.meta.pwm_arr);
        assert_eq!(back["plan"]["rung"], "burst 0");
    }

    #[test]
    fn gzip_is_deterministic_and_round_trips() {
        let a = gzip(b"seg,tick\n0,1\n");
        assert_eq!(a, gzip(b"seg,tick\n0,1\n"));
        assert_eq!(gunzip(&a).expect("gunzip"), b"seg,tick\n0,1\n");
        assert_eq!(
            paths("rest", 2, "raw"),
            (
                "captures/rest/02-raw.csv.gz".into(),
                "captures/rest/02-raw.meta.json".into()
            )
        );
    }
}
