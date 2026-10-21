//! USB bulk backend over nusb (pure rust, no libusb): the osc-adapter's
//! vendor device, IF0, 1209:0001.

use std::time::Duration;

use nusb::transfer::{Control, ControlType, Queue, Recipient, RequestBuffer, TransferError};
use osc_host::link::record::USB_REQ_OPEN_SESSION;

use crate::pipe::{OLD_ADAPTER, PID, Pipe, PipeError, VID};

const EP_OUT: u8 = 0x01;
const EP_IN: u8 = 0x81;
// Each IN transfer completes on a short packet, so it carries one record
// (~200 B at TEL rates), not IN_CAP: the queue holds IN_DEPTH records, about
// 0.8 ms each. 128 cover a ~100 ms client stall; past that the adapter's TX
// queue fills and its UART RX ring laps (one lap = 1024 B of frames lost).
const IN_CAP: usize = 4096;
const IN_DEPTH: usize = 128;

pub struct NusbPipe {
    interface: nusb::Interface,
    rx: Queue<RequestBuffer>,
}

impl NusbPipe {
    /// Find and claim the first osc-adapter on the bus.
    pub fn open() -> Result<Self, PipeError> {
        let io = |e: String| PipeError::Io(e);
        let di = nusb::list_devices()
            .map_err(|e| io(e.to_string()))?
            .find(|d| d.vendor_id() == VID && d.product_id() == PID)
            .ok_or_else(|| io(format!("no osc-adapter ({VID:04x}:{PID:04x}) on the bus")))?;
        let device = di.open().map_err(|e| io(e.to_string()))?;
        // Vendor-class device: no OS driver configures it, so pick config 1
        // ourselves before claiming (WinUSB does it for us and refuses the
        // call). A refusal here is a dead or busy device.
        #[cfg(not(windows))]
        device
            .set_configuration(1)
            .map_err(|e| io(format!("set_configuration(1): {e}")))?;
        let interface = device.claim_interface(0).map_err(|e| io(e.to_string()))?;
        open_session(&interface)?;
        let mut rx = interface.bulk_in_queue(EP_IN);
        while rx.pending() < IN_DEPTH {
            rx.submit(RequestBuffer::new(IN_CAP));
        }
        Ok(Self { interface, rx })
    }
}

impl Pipe for NusbPipe {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        self.interface
            .bulk_out(EP_OUT, bytes.to_vec())
            .await
            .into_result()
            .map_err(|e| transfer_error(EP_OUT, e))?;
        Ok(())
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        let done = self.rx.next_complete().await;
        self.rx.submit(RequestBuffer::new(IN_CAP));
        done.into_result().map_err(|e| transfer_error(EP_IN, e))
    }
}

/// Before any bulk traffic: a previous client may have left records queued.
fn open_session(interface: &nusb::Interface) -> Result<(), PipeError> {
    let control = Control {
        control_type: ControlType::Vendor,
        recipient: Recipient::Interface,
        request: USB_REQ_OPEN_SESSION,
        value: 0,
        index: 0,
    };
    match interface.control_out_blocking(control, &[], Duration::from_secs(1)) {
        Ok(_) => Ok(()),
        Err(TransferError::Stall) => Err(PipeError::Io(OLD_ADAPTER.into())),
        Err(e) => Err(transfer_error(0, e)),
    }
}

/// nusb folds every OS status it has no variant for into a bare "unknown
/// error"; on macOS that includes the transaction error a dropping USB link
/// produces (kIOReturnNotResponding), which Linux reports as a fault.
fn transfer_error(ep: u8, e: TransferError) -> PipeError {
    let dir = if ep & 0x80 != 0 { "IN" } else { "OUT" };
    let what = match e {
        TransferError::Unknown => {
            "transfer failed with an OS status nusb does not name (a USB link fault such \
             as a transaction error; macOS logs the status under subsystem com.apple.usb)"
                .to_string()
        }
        e => e.to_string(),
    };
    PipeError::Io(format!("usb {dir} endpoint {ep:#04x}: {what}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unnamed_transfer_status_names_the_endpoint_and_the_link() {
        let PipeError::Io(m) = transfer_error(EP_IN, TransferError::Unknown) else {
            panic!("io error expected");
        };
        assert!(m.starts_with("usb IN endpoint 0x81: "), "{m}");
        assert!(m.contains("USB link fault"), "{m}");
        assert!(!m.contains("unknown error"), "{m}");
    }

    #[test]
    fn a_named_transfer_status_keeps_its_name() {
        let e = transfer_error(EP_OUT, TransferError::Disconnected);
        assert_eq!(
            e,
            PipeError::Io("usb OUT endpoint 0x01: device disconnected".into())
        );
    }
}
