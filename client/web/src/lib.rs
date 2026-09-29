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

#[cfg(target_arch = "wasm32")]
mod client;
mod descriptor;
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

#[cfg(test)]
mod tests {
    use super::*;

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
