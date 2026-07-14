//! USB bulk backend over nusb (pure rust, no libusb): the osc-adapter's
//! vendor device, IF0, 1209:0001.

use nusb::transfer::RequestBuffer;

use crate::pipe::{Pipe, PipeError};

pub const VID: u16 = 0x1209;
pub const PID: u16 = 0x0001;
const EP_OUT: u8 = 0x01;
const EP_IN: u8 = 0x81;
const IN_CAP: usize = 512;

pub struct NusbPipe {
    interface: nusb::Interface,
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
        // ourselves before claiming (harmless if already configured).
        let _ = device.set_configuration(1);
        let interface = device.claim_interface(0).map_err(|e| io(e.to_string()))?;
        Ok(Self { interface })
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
        self.interface
            .bulk_in(EP_IN, RequestBuffer::new(IN_CAP))
            .await
            .into_result()
            .map_err(|e| PipeError::Io(e.to_string()))
    }
}
