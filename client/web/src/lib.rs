//! osc-client for the browser: a wasm-bindgen layer over [`osc_client`]
//! with every boundary type declared through tsify. The WebUSB-backed
//! [`OscClient`] exists on wasm32 only (the pipe it wraps is target-gated
//! in osc-client); the descriptor surface and the pure helpers build
//! everywhere so native `cargo test` covers them.
//!
//! `OscClient.fake(ids)` opens the same client over a simulated adapter and
//! fleet instead of a device, so the app and its browser tests run with no
//! hardware. It lives behind the `fake` feature, on by default so the
//! shipped package carries it; each servo's UID is derived from its id, so
//! the same roster always discovers the same UIDs.

#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
mod boot;
#[cfg(target_arch = "wasm32")]
mod client;
mod descriptor;
#[cfg(feature = "fake")]
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
mod fake;
mod types;
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
mod uid;

use tsify::{Ts, Tsify};
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
pub use client::{OscClient, request_device};
pub use descriptor::{Descriptor, Registry};
pub use types::*;

/// The osc-adapter's USB vendor id.
#[wasm_bindgen]
pub fn vid() -> u16 {
    osc_client::pipe::VID
}

/// The osc-adapter's USB product id.
#[wasm_bindgen]
pub fn pid() -> u16 {
    osc_client::pipe::PID
}

/// Split a packed `firmware_version` (protocol sec 5.4) into
/// `[major, minor, patch]`.
#[wasm_bindgen(js_name = unpackVersion)]
pub fn unpack_version(fw: u16) -> Result<Ts<Version>, JsError> {
    Ok(version(fw).into_ts()?)
}

fn version(fw: u16) -> Version {
    let (major, minor, patch) = osc_protocol::version::unpack_version(fw);
    Version(major, minor, patch)
}

/// A `.bin`'s tinyboot tail (compatibility key, version, size), or why
/// it has none.
#[wasm_bindgen(js_name = parseImage)]
pub fn parse_image(bytes: &[u8]) -> Result<Ts<ImageCheck>, JsError> {
    let check = ImageCheck::from(osc_client::boot::Image::parse(bytes.to_vec()));
    Ok(check.into_ts()?)
}

/// The reasons set in a `data_flags` byte, most urgent first.
#[wasm_bindgen(js_name = dataReasons)]
pub fn data_reasons(flags: u8) -> Result<Vec<Ts<DataReason>>, JsError> {
    Ok(osc_client::data_state::reasons(flags)
        .into_iter()
        .map(|r| DataReason::from(r).into_ts())
        .collect::<Result<_, _>>()?)
}

/// A `fault_code` by name (`"data"` for a refused closed-loop enable);
/// undefined for a code this build does not know.
#[wasm_bindgen(js_name = faultName)]
pub fn fault_name(code: u8) -> Option<String> {
    osc_client::data_state::fault::name(code).map(String::from)
}

/// A `pos_lut_state` by name (`"LIVE"` while the kernel applies the table);
/// undefined for a value this build does not know.
#[wasm_bindgen(js_name = posLutStateName)]
pub fn pos_lut_state_name(state: u8) -> Option<String> {
    osc_client::pos_lut::state::name(state).map(String::from)
}

/// The protocol default response deadline (protocol sec 7), us.
#[wasm_bindgen(js_name = defaultResponseDeadlineUs)]
pub fn default_response_deadline_us() -> u16 {
    osc_protocol::wire::DEFAULT_RESPONSE_DEADLINE_US
}

/// The longest TEL row, bytes (protocol sec 5.6).
#[wasm_bindgen(js_name = telRowBytesMax)]
pub fn tel_row_bytes_max() -> u32 {
    osc_protocol::tel::ROW_BYTES_MAX as u32
}

/// The chunks a TEL row copies at most.
#[wasm_bindgen(js_name = telChunksMax)]
pub fn tel_chunks_max() -> u32 {
    osc_protocol::tel::CHUNKS_MAX as u32
}

/// The one rate a TEL burst streams at.
#[wasm_bindgen(js_name = telBaud)]
pub fn tel_baud() -> Result<Ts<BaudRate>, JsError> {
    Ok(BaudRate::from(osc_protocol::tel::BAUD).into_ts()?)
}

/// Whether `chunks` make a row a burst arms with: even, inside the table,
/// the first nonempty, at most `telRowBytesMax()` in all.
#[wasm_bindgen(js_name = telChunksValid)]
pub fn tel_chunks_valid(
    #[wasm_bindgen(unchecked_param_type = "TelChunk[]")] chunks: JsValue,
) -> Result<bool, JsError> {
    let chunks: Vec<TelChunk> = serde_wasm_bindgen::from_value(chunks)?;
    Ok(tel_chunks(&chunks).is_some_and(|c| c.valid()))
}

/// The descriptor fields a row of `chunks` carries, each where it sits in
/// the row (a field wholly inside one chunk), in table order.
#[wasm_bindgen(js_name = telColumns)]
pub fn tel_columns(
    d: &Descriptor,
    #[wasm_bindgen(unchecked_param_type = "TelChunk[]")] chunks: JsValue,
) -> Result<Vec<Ts<TelColumn>>, JsError> {
    let chunks: Vec<TelChunk> = serde_wasm_bindgen::from_value(chunks)?;
    let c =
        tel_chunks(&chunks).ok_or_else(|| JsError::new("telColumns: more chunks than a row"))?;
    Ok(columns(d, c)
        .into_iter()
        .map(|c| c.into_ts())
        .collect::<Result<_, _>>()?)
}

/// The descriptor fields a row of `c` carries, in table order.
fn columns(d: &Descriptor, c: osc_protocol::tel::Chunks) -> Vec<TelColumn> {
    use osc_client::descriptor::Kind;
    d.inner()
        .fields
        .iter()
        .filter(|f| matches!(f.width, 1 | 2 | 4) && f.kind != Kind::Bytes)
        .filter_map(|f| {
            let offset = c.offset(f.addr, f.width)? as u16;
            Some(TelColumn {
                name: f.name.clone(),
                offset,
                width: f.width,
                signed: f.kind == Kind::Int,
            })
        })
        .collect()
}

/// One TEL stream frame payload split into head and rows (protocol sec
/// 5.6); undefined when it cannot be a frame of `rowBytes` rows.
#[wasm_bindgen(js_name = telStreamFrame)]
pub fn tel_stream_frame(
    #[wasm_bindgen(js_name = rowBytes)] row_bytes: u32,
    payload: &[u8],
) -> Result<Option<Ts<TelStreamFrame>>, JsError> {
    Ok(
        osc_protocol::tel::StreamFrame::parse(row_bytes as usize, payload)
            .map(|f| TelStreamFrame::from(f).into_ts())
            .transpose()?,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tel_stream_frames_split_head_and_rows() {
        use osc_protocol::tel::StreamFrame;
        let p = [3, 1, 0x10, 0x00, 0x00, 0x01, 0x01, 0x01];
        let f = TelStreamFrame::from(StreamFrame::parse(2, &p).unwrap());
        assert_eq!((f.seq, f.last, f.tick), (3, true, 16));
        assert_eq!(f.rows.as_slice(), [0x00, 0x01, 0x01, 0x01]);
        assert!(StreamFrame::parse(4, &p[..6]).is_none());
    }

    /// A row's columns: the fields wholly inside a chunk, at their row
    /// offsets, signed where the descriptor says.
    #[test]
    fn tel_columns_follow_the_chunks() {
        let d = Descriptor::parse(include_str!("../../../descriptors/osc-servo/0.1.json")).unwrap();
        let tick = d.field("duty_ticks").unwrap().addr;
        let vbus = d.field("vbus_counts").unwrap().addr;
        let chunks = [
            TelChunk { addr: tick, len: 8 },
            TelChunk { addr: vbus, len: 2 },
        ];
        let c = tel_chunks(&chunks).unwrap();
        assert!(c.valid());
        let cols: Vec<(String, u16, bool)> = columns(&d, c)
            .into_iter()
            .map(|c| (c.name, c.offset, c.signed))
            .collect();
        let want = [
            ("duty_ticks", 0, true),
            ("current_trough", 2, false),
            ("i_tick_counts", 4, true),
            ("vdiff_counts", 6, true),
            ("vbus_counts", 8, false),
        ];
        assert_eq!(cols.len(), want.len());
        for (got, (name, offset, signed)) in cols.iter().zip(want) {
            assert_eq!(got, &(name.to_string(), offset, signed));
        }
        let wide = [TelChunk {
            addr: tick,
            len: 14,
        }];
        assert!(!tel_chunks(&wide).unwrap().valid());
        assert!(tel_chunks(&[chunks[0]; 3]).is_none());
    }

    #[test]
    fn pos_lut_mirrors_state_and_points() {
        use osc_client::pos_lut::{self, INTERVALS};
        let mut points = [0i16; INTERVALS];
        points[40] = -7;
        let l = PosLut::from(pos_lut::PosLut {
            state: pos_lut::state::LIVE,
            points,
        });
        assert_eq!((l.state, l.live), (pos_lut::state::LIVE, true));
        assert_eq!(l.state_name.as_deref(), Some("LIVE"));
        assert_eq!(l.points.len(), INTERVALS);
        assert_eq!(l.points[40], -7);
        let l = PosLut::from(pos_lut::PosLut {
            state: pos_lut::state::REJECT_ENDS,
            points,
        });
        assert!(!l.live);
        assert_eq!(pos_lut_state_name(l.state).as_deref(), Some("REJECT_ENDS"));
        assert_eq!(pos_lut_state_name(9), None);
    }

    #[test]
    fn unpack_version_passes_through() {
        let fw = osc_protocol::version::pack_version(1, 2, 3);
        assert_eq!(version(fw), Version(1, 2, 3));
        assert_eq!(version(0xFFFF), Version(31, 31, 63));
    }

    #[test]
    fn data_state_mirrors_name_text_and_verdicts() {
        use osc_client::data_state::{self, CALIB_STALE, PLANT_UNSET, STAMP_MISMATCH, fault};
        let s = DataState::from(data_state::DataState {
            flags: CALIB_STALE | STAMP_MISMATCH | PLANT_UNSET,
            fault_code: fault::DATA,
        });
        let names: Vec<&str> = s.reasons.iter().map(|r| r.name.as_str()).collect();
        assert_eq!(names, ["CALIB_STALE", "STAMP_MISMATCH", "PLANT_UNSET"]);
        assert_eq!(s.fault.as_deref(), Some("data"));
        assert!(s.message.unwrap().starts_with("closed loop refused: "));
        assert!(s.open_loop);
        assert!(!s.closed_loop);
        let clean = DataState::from(data_state::DataState {
            flags: 0,
            fault_code: fault::NONE,
        });
        assert!(clean.reasons.is_empty());
        assert_eq!(clean.message, None);
        assert!(clean.closed_loop);
        assert_eq!(fault_name(fault::STALL).as_deref(), Some("stall"));
        assert_eq!(fault_name(200), None);
    }

    #[test]
    fn vendor_ids_match_the_pipe() {
        assert_eq!((vid(), pid()), (0x1209, 0x0001));
    }
}
