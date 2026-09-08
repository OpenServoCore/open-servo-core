//! USB bulk backend over nusb (pure rust, no libusb): the osc-adapter's
//! vendor device, IF0, 1209:0001.

use nusb::transfer::{Queue, RequestBuffer};

use crate::pipe::{Pipe, PipeError};

pub const VID: u16 = 0x1209;
pub const PID: u16 = 0x0001;
const EP_OUT: u8 = 0x01;
const EP_IN: u8 = 0x81;
// IN_DEPTH x IN_CAP = 32 kB queued with the xHCI: ~128 ms of the 255 kB/s
// TEL stream with no IN gap while the client thread is busy. A gap past a
// few ms fills the adapter's TX queue and laps its UART RX ring.
const IN_CAP: usize = 4096;
const IN_DEPTH: usize = 8;

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
            .map_err(|e| PipeError::Io(e.to_string()))?;
        Ok(())
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        let done = self.rx.next_complete().await;
        self.rx.submit(RequestBuffer::new(IN_CAP));
        done.into_result().map_err(|e| PipeError::Io(e.to_string()))
    }
}
