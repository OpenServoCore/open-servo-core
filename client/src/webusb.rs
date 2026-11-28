//! Browser backend over WebUSB: the same vendor device as [`crate::nusb`],
//! IF0, 1209:0001, from a page or worker. Bindings are hand-written: web-sys
//! keeps its WebUSB types behind `--cfg=web_sys_unstable_apis`, which every
//! downstream build would have to set in rustflags.

use js_sys::Promise;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::JsFuture;

use osc_host::link::record::USB_REQ_OPEN_SESSION;

use crate::in_queue::InQueue;
use crate::pipe::{OLD_ADAPTER, PID, Pipe, PipeError, VID};

// WebUSB takes the endpoint number and derives the address from the
// method (transferIn: `endpointNumber | 0x80`), so IN 0x81 is endpoint 1.
const EP_OUT: u8 = 1;
const EP_IN: u8 = 1;
// The native pipe's sizing (`nusb` IN_DEPTH): each IN transfer completes on
// a short packet, one record (~200 B at TEL rates, about 0.8 ms each), so
// IN_DEPTH queued cover ~100 ms of the page not draining before the
// adapter's TX queue fills and its UART RX ring laps. With one transfer
// queued, every completion-to-reissue round trip through the page's event
// loop left the endpoint unpolled.
//
// Results are taken oldest first, and that is arrival order: the queued
// transfers share one bulk pipe, which the host controller services FIFO;
// WebUSB transferIn() settles each promise from a task queued on the USB
// task source as its transfer completes, and an event loop runs one
// source's tasks in queue order.
const IN_CAP: u32 = 4096;
const IN_DEPTH: usize = 128;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_name = USBDevice, typescript_type = "USBDevice")]
    #[derive(Clone)]
    pub type UsbDevice;
    #[wasm_bindgen(js_namespace = ["navigator", "usb"], js_name = requestDevice, catch)]
    fn request_device_js(opts: &JsValue) -> Result<Promise, JsValue>;
    #[wasm_bindgen(method)]
    fn open(this: &UsbDevice) -> Promise;
    #[wasm_bindgen(method, js_name = selectConfiguration)]
    fn select_configuration(this: &UsbDevice, v: u8) -> Promise;
    #[wasm_bindgen(method, js_name = claimInterface)]
    fn claim_interface(this: &UsbDevice, n: u8) -> Promise;
    #[wasm_bindgen(method, js_name = controlTransferOut)]
    fn control_transfer_out(this: &UsbDevice, setup: &JsValue) -> Promise;
    #[wasm_bindgen(method, js_name = transferIn)]
    fn transfer_in(this: &UsbDevice, ep: u8, len: u32) -> Promise;
    #[wasm_bindgen(method, js_name = transferOut)]
    fn transfer_out(this: &UsbDevice, ep: u8, data: &[u8]) -> Promise;
    #[wasm_bindgen(method)]
    fn close(this: &UsbDevice) -> Promise;
    /// Whether a page holds the device open: an owner that finds it so
    /// would share the bus with whoever holds it.
    #[wasm_bindgen(method, getter)]
    pub fn opened(this: &UsbDevice) -> bool;

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

/// Before any bulk traffic: a previous page may have left records queued,
/// and `selectConfiguration` sends nothing when the configuration is
/// already active.
async fn open_session(device: &UsbDevice) -> Result<(), PipeError> {
    let setup = js_sys::Object::new();
    for (k, v) in [
        ("requestType", JsValue::from("vendor")),
        ("recipient", "interface".into()),
        ("request", USB_REQ_OPEN_SESSION.into()),
        ("value", 0.into()),
        ("index", 0.into()),
    ] {
        js_sys::Reflect::set(&setup, &k.into(), &v).map_err(js_err)?;
    }
    let res = settle(device.control_transfer_out(&setup)).await?;
    match res
        .unchecked_into::<UsbOutTransferResult>()
        .status()
        .as_str()
    {
        "stall" => Err(PipeError::Io(OLD_ADAPTER.into())),
        s => check(s.to_string()),
    }
}

pub struct WebUsbPipe {
    device: UsbDevice,
    // A pipe dropped unclosed leaves its transfers to the device's close,
    // which every owner runs before the next opens it.
    rx: InQueue<JsFuture>,
}

impl WebUsbPipe {
    /// Open, configure and claim a device from [`request_device`].
    pub async fn open(device: UsbDevice) -> Result<Self, PipeError> {
        settle(device.open()).await?;
        settle(device.select_configuration(1)).await?;
        settle(device.claim_interface(0)).await?;
        open_session(&device).await?;
        let mut rx = InQueue::new(IN_DEPTH);
        rx.fill(in_transfer(&device));
        Ok(Self { device, rx })
    }

    /// `transferIn` calls issued since open: IN_DEPTH at open, then one per
    /// result taken, so this counts transfers, not recv calls.
    pub fn transfers_in(&self) -> u64 {
        self.rx.issued()
    }

    /// The device this pipe holds, for the owner the bus goes to next.
    pub fn device(&self) -> &UsbDevice {
        &self.device
    }

    pub async fn close(mut self) -> Result<(), PipeError> {
        close_device(&self.device).await?;
        // close() rejects every transfer still scheduled with an AbortError;
        // settling them here leaves none of this session's in flight once
        // the device goes to its next owner.
        for t in self.rx.drain() {
            let _ = t.await;
        }
        Ok(())
    }
}

fn in_transfer(device: &UsbDevice) -> impl FnMut() -> JsFuture + '_ {
    || JsFuture::from(device.transfer_in(EP_IN, IN_CAP))
}

/// Close `device` with no pipe over it: a client whose GET_INFO failed
/// dropped its pipe, and the device must still be closed before another
/// owner opens it.
pub async fn close_device(device: &UsbDevice) -> Result<(), PipeError> {
    settle(device.close()).await.map(drop)
}

impl Pipe for WebUsbPipe {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        let res = settle(self.device.transfer_out(EP_OUT, bytes)).await?;
        check(res.unchecked_into::<UsbOutTransferResult>().status())
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        let res = self.rx.next(in_transfer(&self.device)).await;
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
