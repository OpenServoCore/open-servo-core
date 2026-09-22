//! Browser backend over WebUSB: the same vendor device as [`crate::nusb`],
//! IF0, 1209:0001, from a page or worker. Bindings are hand-written: web-sys
//! keeps its WebUSB types behind `--cfg=web_sys_unstable_apis`, which every
//! downstream build would have to set in rustflags.

use std::future::poll_fn;
use std::pin::Pin;

use js_sys::Promise;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::JsFuture;

use crate::pipe::{PID, Pipe, PipeError, VID};

// WebUSB takes the endpoint number and derives the address from the
// method (transferIn: `endpointNumber | 0x80`), so IN 0x81 is endpoint 1.
const EP_OUT: u8 = 1;
const EP_IN: u8 = 1;
const IN_CAP: u32 = 4096;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_name = USBDevice, typescript_type = "USBDevice")]
    pub type UsbDevice;
    #[wasm_bindgen(js_namespace = ["navigator", "usb"], js_name = requestDevice, catch)]
    fn request_device_js(opts: &JsValue) -> Result<Promise, JsValue>;
    #[wasm_bindgen(method)]
    fn open(this: &UsbDevice) -> Promise;
    #[wasm_bindgen(method, js_name = selectConfiguration)]
    fn select_configuration(this: &UsbDevice, v: u8) -> Promise;
    #[wasm_bindgen(method, js_name = claimInterface)]
    fn claim_interface(this: &UsbDevice, n: u8) -> Promise;
    #[wasm_bindgen(method, js_name = transferIn)]
    fn transfer_in(this: &UsbDevice, ep: u8, len: u32) -> Promise;
    #[wasm_bindgen(method, js_name = transferOut)]
    fn transfer_out(this: &UsbDevice, ep: u8, data: &[u8]) -> Promise;
    #[wasm_bindgen(method)]
    fn close(this: &UsbDevice) -> Promise;

    type UsbInTransferResult;
    #[wasm_bindgen(method, getter)]
    fn data(this: &UsbInTransferResult) -> Option<js_sys::DataView>;
    #[wasm_bindgen(method, getter)]
    fn status(this: &UsbInTransferResult) -> String;

    type UsbOutTransferResult;
    #[wasm_bindgen(method, getter)]
    fn status(this: &UsbOutTransferResult) -> String;
}

fn js_err(e: JsValue) -> PipeError {
    PipeError::Io(match e.dyn_ref::<js_sys::Error>() {
        Some(e) => format!("{}: {}", e.name(), e.message()),
        None => format!("{e:?}"),
    })
}

async fn settle(p: Promise) -> Result<JsValue, PipeError> {
    JsFuture::from(p).await.map_err(js_err)
}

fn check(status: String) -> Result<(), PipeError> {
    if status == "ok" {
        Ok(())
    } else {
        Err(PipeError::Io(format!("transfer status {status}")))
    }
}

/// Prompt the user for an osc-adapter (must run from a user gesture).
pub async fn request_device() -> Result<UsbDevice, PipeError> {
    let filter = js_sys::Object::new();
    js_sys::Reflect::set(&filter, &"vendorId".into(), &VID.into()).map_err(js_err)?;
    js_sys::Reflect::set(&filter, &"productId".into(), &PID.into()).map_err(js_err)?;
    let opts = js_sys::Object::new();
    let filters = js_sys::Array::of1(&filter);
    js_sys::Reflect::set(&opts, &"filters".into(), &filters).map_err(js_err)?;
    let dev = settle(request_device_js(&opts).map_err(js_err)?).await?;
    Ok(dev.unchecked_into())
}

pub struct WebUsbPipe {
    device: UsbDevice,
    // Dropping a JsFuture does not cancel the transfer; the browser would
    // deliver its bytes to nobody, so a guard-lapsed recv resumes it.
    pending: Option<JsFuture>,
}

impl WebUsbPipe {
    /// Open, configure and claim a device from [`request_device`].
    pub async fn open(device: UsbDevice) -> Result<Self, PipeError> {
        settle(device.open()).await?;
        settle(device.select_configuration(1)).await?;
        settle(device.claim_interface(0)).await?;
        Ok(Self {
            device,
            pending: None,
        })
    }

    pub async fn close(self) -> Result<(), PipeError> {
        settle(self.device.close()).await.map(drop)
    }
}

impl Pipe for WebUsbPipe {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        let res = settle(self.device.transfer_out(EP_OUT, bytes)).await?;
        check(res.unchecked_into::<UsbOutTransferResult>().status())
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        let (device, pending) = (&self.device, &mut self.pending);
        let res = poll_fn(|cx| {
            let f =
                pending.get_or_insert_with(|| JsFuture::from(device.transfer_in(EP_IN, IN_CAP)));
            Pin::new(f).poll(cx)
        })
        .await;
        self.pending = None;
        let res = res.map_err(js_err)?.unchecked_into::<UsbInTransferResult>();
        check(res.status())?;
        let dv = res
            .data()
            .ok_or_else(|| PipeError::Io("transferIn result without data".into()))?;
        let bytes = js_sys::Uint8Array::new_with_byte_offset_and_length(
            &dv.buffer(),
            dv.byte_offset() as u32,
            dv.byte_length() as u32,
        );
        Ok(bytes.to_vec())
    }
}
