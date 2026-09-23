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
#[cfg(feature = "fake")]
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
mod fake_seed;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unpack_version_passes_through() {
        let fw = osc_protocol::version::pack_version(1, 2, 3);
        assert_eq!(version(fw), Version(1, 2, 3));
        assert_eq!(version(0xFFFF), Version(31, 31, 63));
    }

    #[test]
    fn vendor_ids_match_the_pipe() {
        assert_eq!((vid(), pid()), (0x1209, 0x0001));
    }
}
