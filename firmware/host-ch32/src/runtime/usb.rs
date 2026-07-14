//! Polled USBHS vendor-class device: EP0 control FSM + one bulk pair
//! (EP1 OUT = pipe bytes in, EP1 IN = records out). No interrupt vector --
//! the main loop calls [`UsbDevice::poll`] and the controller's `int_busy`
//! auto-NAK holds the host off while a flag waits for service. No class
//! machinery: descriptors, SET_ADDRESS/SET_CONFIGURATION, and raw bulk
//! bytes are the whole surface (the link layer owns framing).
//!
//! Identity: 1209:0001 = pid.codes' testing PID, pre-release only; the
//! permanent-PID switch must bump bcdDevice (Windows caches MS OS
//! descriptors per VID/PID/bcdDevice). MS OS 2.0 rides the BOS so WinUSB
//! binds driverless; macOS/Linux ignore it.

use core::cell::SyncUnsafeCell;

use ch32_metapac::usbhs::vals::{EpRxResponse, EpTog, EpTxResponse, SpeedType, UsbToken};

use crate::hal::usbhs::{dregs, regs};

pub const VID: u16 = 0x1209;
pub const PID: u16 = 0x0001;

const EP0_MPS: usize = 64;
const BULK_MPS_HS: usize = 512;
const BULK_MPS_FS: usize = 64;

/// MS OS 2.0 vendor code (arbitrary, echoed from the BOS platform blob).
const MS_VENDOR_CODE: u8 = 0x20;

#[repr(align(4))]
struct DmaBuf<const N: usize>([u8; N]);

static EP0_BUF: SyncUnsafeCell<DmaBuf<EP0_MPS>> = SyncUnsafeCell::new(DmaBuf([0; EP0_MPS]));
static EP1_RX: SyncUnsafeCell<DmaBuf<BULK_MPS_HS>> = SyncUnsafeCell::new(DmaBuf([0; BULK_MPS_HS]));
static EP1_TX: SyncUnsafeCell<DmaBuf<BULK_MPS_HS>> = SyncUnsafeCell::new(DmaBuf([0; BULK_MPS_HS]));

// --- descriptors -----------------------------------------------------------

const DESC_DEVICE: u8 = 1;
const DESC_CONFIG: u8 = 2;
const DESC_STRING: u8 = 3;
const DESC_QUALIFIER: u8 = 6;
const DESC_OTHER_SPEED: u8 = 7;
const DESC_BOS: u8 = 0x0F;

#[rustfmt::skip]
const DEVICE_DESC: [u8; 18] = [
    18, DESC_DEVICE,
    0x10, 0x02,             // bcdUSB 2.10 (BOS-capable)
    // Class at the interface, 00 here: a device-level vendor class gets no
    // OS composite driver, so nothing ever configures it (measured).
    0x00, 0x00, 0x00,
    EP0_MPS as u8,
    (VID & 0xFF) as u8, (VID >> 8) as u8,
    (PID & 0xFF) as u8, (PID >> 8) as u8,
    0x00, 0x01,             // bcdDevice 1.00
    1, 2, 3,                // manufacturer / product / serial strings
    1,                      // one configuration
];

#[rustfmt::skip]
const QUALIFIER_DESC: [u8; 10] = [
    10, DESC_QUALIFIER,
    0x00, 0x02,             // bcdUSB 2.00
    0x00, 0x00, 0x00,
    EP0_MPS as u8,
    1, 0,
];

/// Config + interface + two bulk endpoints; `mps` is speed-dependent.
fn config_desc(dst: &mut [u8], mps: usize) -> usize {
    #[rustfmt::skip]
    let desc: [u8; 32] = [
        9, DESC_CONFIG, 32, 0,
        1,                  // one interface
        1,                  // bConfigurationValue
        0,
        0x80,               // bus powered
        250,                // 500 mA: the adapter feeds the DUT's 3V3 rail
        // interface 0: osc-host, vendor class
        9, 4, 0, 0, 2, 0xFF, 0x00, 0x00, 4,
        // EP1 OUT bulk
        7, 5, 0x01, 0x02, (mps & 0xFF) as u8, (mps >> 8) as u8, 0,
        // EP1 IN bulk
        7, 5, 0x81, 0x02, (mps & 0xFF) as u8, (mps >> 8) as u8, 0,
    ];
    dst[..32].copy_from_slice(&desc);
    32
}

/// BOS with the MS OS 2.0 platform capability (UUID per Microsoft spec).
#[rustfmt::skip]
const BOS_DESC: [u8; 33] = [
    5, DESC_BOS, 33, 0, 1,
    28, 0x10, 0x05, 0x00,
    // MS OS 2.0 platform capability UUID D8DD60DF-4589-4CC7-9CD2-659D9E648A9F
    0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06, // Windows 8.1+
    (MS_OS_DESC_LEN & 0xFF) as u8, (MS_OS_DESC_LEN >> 8) as u8,
    MS_VENDOR_CODE,
    0,                      // no alternate enumeration
];

const MS_OS_DESC_LEN: usize = 10 + 20 + 132;

/// MS OS 2.0 descriptor set: header + WINUSB compatible ID + the
/// DeviceInterfaceGUIDs registry property clients enumerate by.
fn ms_os_desc(dst: &mut [u8]) -> usize {
    let mut n = 0;
    let mut put = |bytes: &[u8]| {
        dst[n..n + bytes.len()].copy_from_slice(bytes);
        n += bytes.len();
    };
    // set header
    put(&[10, 0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x06]);
    put(&(MS_OS_DESC_LEN as u16).to_le_bytes());
    // compatible ID: WINUSB
    put(&[20, 0, 0x03, 0x00]);
    put(b"WINUSB\0\0");
    put(&[0; 8]);
    // registry property: DeviceInterfaceGUIDs = REG_MULTI_SZ {GUID}
    put(&(132u16).to_le_bytes());
    put(&[0x04, 0x00, 0x07, 0x00, 42, 0]);
    for c in "DeviceInterfaceGUIDs\0".bytes() {
        put(&[c, 0]);
    }
    put(&(80u16).to_le_bytes());
    for c in "{9db9fbf2-52f6-4e87-9d3d-6e8e29ef6b5e}\0\0".bytes() {
        put(&[c, 0]);
    }
    debug_assert!(n == MS_OS_DESC_LEN);
    n
}

const STR_MANUFACTURER: &str = "OpenServoCore";
const STR_PRODUCT: &str = "osc-adapter-wchlinke";
const STR_INTERFACE: &str = "osc-host";

/// ESIG 96-bit UID (RM: 0x1FFFF7E8) -- the serial string source.
const UID_BASE: *const u32 = 0x1FFF_F7E8 as *const u32;

// --- device state -----------------------------------------------------------

#[derive(PartialEq)]
enum Ep0 {
    Idle,
    /// Streaming `stage[pos..len]` to the host, 64 B per IN.
    DataIn {
        pos: usize,
        len: usize,
    },
    /// Zero-length OUT closes an IN data phase.
    StatusOut,
    /// Zero-length IN closes a no-data request.
    StatusIn,
}

pub struct UsbDevice {
    ep0: Ep0,
    ep0_tog: bool,
    address_pending: Option<u8>,
    configured: bool,
    tx_busy: bool,
    tx_tog: bool,
    rx_tog: bool,
    rx_pending: Option<usize>,
    stage: [u8; 256],
    serial: [u8; 24],
}

impl UsbDevice {
    pub fn new() -> Self {
        let mut serial = [0u8; 24];
        for w in 0..3 {
            // SAFETY: ESIG is a read-only factory region, always mapped.
            let word = unsafe { UID_BASE.add(w).read_volatile() };
            for (i, item) in serial[w * 8..w * 8 + 8].iter_mut().enumerate() {
                let nibble = (word >> (28 - 4 * i)) & 0xF;
                *item = b"0123456789abcdef"[nibble as usize];
            }
        }
        Self {
            ep0: Ep0::Idle,
            ep0_tog: true,
            address_pending: None,
            configured: false,
            tx_busy: false,
            tx_tog: false,
            rx_tog: false,
            rx_pending: None,
            stage: [0; 256],
            serial,
        }
    }

    /// One EP0 DMA buffer serves both directions; control transfers are
    /// strictly sequential so the reuse is safe.
    fn ep0_buf() -> &'static mut [u8; EP0_MPS] {
        // SAFETY: main-loop-only access (this module has no ISR).
        let buf: &'static mut DmaBuf<EP0_MPS> = unsafe { &mut *EP0_BUF.get() };
        &mut buf.0
    }

    /// Current bulk packet ceiling (per negotiated speed).
    pub fn mps(&self) -> usize {
        if regs().speed_type().read().speed_type() == SpeedType::HIGHSPEED {
            BULK_MPS_HS
        } else {
            BULK_MPS_FS
        }
    }

    pub fn configured(&self) -> bool {
        self.configured
    }

    /// Bytes of the last completed EP1 OUT, until `rx_consume`.
    pub fn rx(&self) -> Option<&[u8]> {
        self.rx_pending.map(|n| {
            // SAFETY: EP1 RX is NAK-parked while `rx_pending` holds (re-armed
            // only in rx_consume), so the DMA side is quiescent under the view.
            let buf: &DmaBuf<BULK_MPS_HS> = unsafe { &*EP1_RX.get() };
            &buf.0[..n]
        })
    }

    pub fn rx_consume(&mut self) {
        if self.rx_pending.take().is_some() {
            dregs().ep_rx_ctrl(1).modify(|w| {
                w.set_mask_uep_r_res(EpRxResponse::ACK);
            });
        }
    }

    pub fn tx_ready(&self) -> bool {
        self.configured && !self.tx_busy
    }

    /// Queue one IN packet (<= current bulk MPS; the caller chunks).
    pub fn tx(&mut self, bytes: &[u8]) {
        debug_assert!(self.tx_ready() && bytes.len() <= self.mps());
        // SAFETY: tx_busy=false means the controller is not reading the
        // buffer (last IN completed or never armed).
        let buf: &mut DmaBuf<BULK_MPS_HS> = unsafe { &mut *EP1_TX.get() };
        buf.0[..bytes.len()].copy_from_slice(bytes);
        let d = dregs();
        d.ep_t_len(1).write(|w| w.set_len(bytes.len() as u16));
        d.ep_tx_ctrl(1).modify(|w| {
            w.set_mask_uep_t_tog(if self.tx_tog {
                EpTog::DATA1
            } else {
                EpTog::DATA0
            });
            w.set_mask_uep_t_res(EpTxResponse::ACK);
        });
        self.tx_busy = true;
    }

    /// Service pending controller flags; at most one flag per call. Call
    /// freely from the main loop.
    pub fn poll(&mut self) {
        let r = regs();
        let flag = r.int_fg().read();

        if flag.bus_rst() {
            self.on_bus_reset();
            r.int_fg().write(|w| w.set_bus_rst(true));
            return;
        }
        if flag.setup_act() {
            self.on_setup();
            r.int_fg().write(|w| {
                w.set_setup_act(true);
                w.set_transfer(true);
            });
            return;
        }
        if flag.transfer() {
            let st = r.int_st().read();
            match st.endp() {
                0 => self.on_ep0_transfer(st.token()),
                1 => self.on_ep1_transfer(st.token(), st.tog_ok()),
                _ => {}
            }
            r.int_fg().write(|w| w.set_transfer(true));
            return;
        }
        if flag.suspend() {
            r.int_fg().write(|w| w.set_suspend(true));
        }
    }

    fn on_bus_reset(&mut self) {
        let d = dregs();
        regs().dev_ad().write(|w| w.set_addr(0));
        d.ep0_dma().write_value(Self::ep0_buf().as_ptr() as u32);
        d.ep_max_len(0).write(|w| w.set_len(EP0_MPS as u16));
        d.ep_rx_ctrl(0)
            .write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
        d.ep_tx_ctrl(0)
            .write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
        // EP1 parks until SET_CONFIGURATION re-enables it.
        d.ep_config()
            .write_value(ch32_metapac::usbhs::regs::EpConfig::default());
        d.ep_tx_ctrl(1)
            .write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
        d.ep_rx_ctrl(1)
            .write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
        self.ep0 = Ep0::Idle;
        self.address_pending = None;
        self.configured = false;
        self.tx_busy = false;
        self.rx_pending = None;
    }

    // --- control pipe -------------------------------------------------------

    fn on_setup(&mut self) {
        let d = dregs();
        d.ep_rx_ctrl(0)
            .write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
        d.ep_tx_ctrl(0)
            .write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));

        let mut setup = [0u8; 8];
        setup.copy_from_slice(&Self::ep0_buf()[..8]);
        let req_type = setup[0];
        let request = setup[1];
        let value = u16::from_le_bytes([setup[2], setup[3]]);
        let index = u16::from_le_bytes([setup[4], setup[5]]);
        let length = u16::from_le_bytes([setup[6], setup[7]]) as usize;

        // A SETUP mid-transfer aborts the old one (control-pipe law).
        self.ep0 = Ep0::Idle;

        let reply: Option<usize> = match (req_type, request) {
            // GET_DESCRIPTOR
            (0x80, 6) => self.load_descriptor((value >> 8) as u8, (value & 0xFF) as u8),
            // GET_STATUS (device/interface/endpoint)
            (0x80..=0x82, 0) => {
                self.stage[..2].copy_from_slice(&[0, 0]);
                Some(2)
            }
            // SET_ADDRESS: applied after the status stage (controller law).
            (0x00, 5) => {
                self.address_pending = Some((value & 0x7F) as u8);
                Some(0)
            }
            // SET_CONFIGURATION
            (0x00, 9) => match value {
                0 => {
                    self.configured = false;
                    Some(0)
                }
                1 => {
                    self.configure_ep1();
                    Some(0)
                }
                _ => None,
            },
            // GET_CONFIGURATION
            (0x80, 8) => {
                self.stage[0] = self.configured as u8;
                Some(1)
            }
            // GET_INTERFACE / SET_INTERFACE (one interface, one alt)
            (0x81, 10) => {
                self.stage[0] = 0;
                Some(1)
            }
            (0x01, 11) if value == 0 => Some(0),
            // CLEAR_FEATURE(ENDPOINT_HALT): reset the named EP1 toggle.
            (0x02, 1) if value == 0 => {
                match index as u8 {
                    0x01 => self.rx_tog = false,
                    0x81 => self.tx_tog = false,
                    _ => {}
                }
                Some(0)
            }
            // MS OS 2.0 descriptor set request
            (0xC0, MS_VENDOR_CODE) if index == 7 => Some(ms_os_desc(&mut self.stage)),
            _ => None,
        };

        match reply {
            None => self.ep0_stall(),
            Some(0) if length == 0 => {
                // No-data request: zero-length status IN, DATA1.
                let d = dregs();
                d.ep_t_len(0).write(|w| w.set_len(0));
                d.ep_tx_ctrl(0).write(|w| {
                    w.set_mask_uep_t_tog(EpTog::DATA1);
                    w.set_mask_uep_t_res(EpTxResponse::ACK);
                });
                self.ep0 = Ep0::StatusIn;
            }
            Some(n) => {
                self.ep0 = Ep0::DataIn {
                    pos: 0,
                    len: n.min(length),
                };
                self.ep0_tog = true;
                self.ep0_send_chunk();
            }
        }
    }

    fn load_descriptor(&mut self, kind: u8, idx: u8) -> Option<usize> {
        match kind {
            DESC_DEVICE => {
                self.stage[..18].copy_from_slice(&DEVICE_DESC);
                Some(18)
            }
            DESC_CONFIG => {
                let mps = self.mps();
                Some(config_desc(&mut self.stage, mps))
            }
            DESC_QUALIFIER => {
                self.stage[..10].copy_from_slice(&QUALIFIER_DESC);
                Some(10)
            }
            // Other-speed operation is real but unadvertised; stalling the
            // descriptor is spec-legal and hosts fall back cleanly.
            DESC_OTHER_SPEED => None,
            DESC_BOS => {
                self.stage[..33].copy_from_slice(&BOS_DESC);
                Some(33)
            }
            DESC_STRING => {
                let n = match idx {
                    0 => {
                        self.stage[2..4].copy_from_slice(&[0x09, 0x04]); // en-US
                        4
                    }
                    1 => self.utf16_string(STR_MANUFACTURER.as_bytes()),
                    2 => self.utf16_string(STR_PRODUCT.as_bytes()),
                    3 => {
                        let serial = self.serial;
                        self.utf16_string(&serial)
                    }
                    4 => self.utf16_string(STR_INTERFACE.as_bytes()),
                    _ => return None,
                };
                self.stage[0] = n as u8;
                self.stage[1] = DESC_STRING;
                Some(n)
            }
            _ => None,
        }
    }

    /// ASCII -> UTF-16LE string descriptor body; returns the total length.
    fn utf16_string(&mut self, ascii: &[u8]) -> usize {
        for (i, &c) in ascii.iter().enumerate() {
            self.stage[2 + 2 * i] = c;
            self.stage[3 + 2 * i] = 0;
        }
        2 + 2 * ascii.len()
    }

    fn ep0_send_chunk(&mut self) {
        let Ep0::DataIn { pos, len } = self.ep0 else {
            return;
        };
        let n = (len - pos).min(EP0_MPS);
        Self::ep0_buf()[..n].copy_from_slice(&self.stage[pos..pos + n]);
        let d = dregs();
        d.ep_t_len(0).write(|w| w.set_len(n as u16));
        d.ep_tx_ctrl(0).write(|w| {
            w.set_mask_uep_t_tog(if self.ep0_tog {
                EpTog::DATA1
            } else {
                EpTog::DATA0
            });
            w.set_mask_uep_t_res(EpTxResponse::ACK);
        });
        self.ep0 = Ep0::DataIn { pos: pos + n, len };
        self.ep0_tog = !self.ep0_tog;
    }

    fn ep0_stall(&mut self) {
        let d = dregs();
        d.ep_tx_ctrl(0)
            .write(|w| w.set_mask_uep_t_res(EpTxResponse::STALL));
        d.ep_rx_ctrl(0)
            .write(|w| w.set_mask_uep_r_res(EpRxResponse::STALL));
        self.ep0 = Ep0::Idle;
    }

    fn on_ep0_transfer(&mut self, token: UsbToken) {
        let d = dregs();
        match token {
            UsbToken::IN => match self.ep0 {
                Ep0::DataIn { pos, len } if pos < len => self.ep0_send_chunk(),
                Ep0::DataIn { .. } => {
                    // Data phase done: arm the zero-length status OUT.
                    d.ep_tx_ctrl(0)
                        .write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
                    d.ep_rx_ctrl(0).write(|w| {
                        w.set_mask_uep_r_tog(EpTog::DATA1);
                        w.set_mask_uep_r_res(EpRxResponse::ACK);
                    });
                    self.ep0 = Ep0::StatusOut;
                }
                Ep0::StatusIn => {
                    d.ep_tx_ctrl(0)
                        .write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
                    if let Some(addr) = self.address_pending.take() {
                        regs().dev_ad().write(|w| w.set_addr(addr));
                    }
                    self.ep0 = Ep0::Idle;
                }
                _ => {}
            },
            UsbToken::OUT if self.ep0 == Ep0::StatusOut => {
                d.ep_rx_ctrl(0)
                    .write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
                self.ep0 = Ep0::Idle;
            }
            _ => {}
        }
    }

    // --- bulk pair ----------------------------------------------------------

    fn configure_ep1(&mut self) {
        let d = dregs();
        let mps = self.mps();
        // SAFETY: static DMA buffers, addresses stable for the program.
        d.ep_rx_dma(0).write_value(EP1_RX.get() as u32);
        d.ep_tx_dma(0).write_value(EP1_TX.get() as u32);
        d.ep_max_len(1).write(|w| w.set_len(mps as u16));
        // Bulk is EpType reset default; buf_mod stays single-buffer.
        d.ep_config().modify(|w| {
            w.set_t_en(0, true);
            w.set_r_en(0, true);
        });
        d.ep_rx_ctrl(1).write(|w| {
            w.set_mask_uep_r_tog(EpTog::DATA0);
            w.set_mask_uep_r_res(EpRxResponse::ACK);
        });
        d.ep_tx_ctrl(1).write(|w| {
            w.set_mask_uep_t_tog(EpTog::DATA0);
            w.set_mask_uep_t_res(EpTxResponse::NAK);
        });
        self.rx_tog = false;
        self.tx_tog = false;
        self.tx_busy = false;
        self.rx_pending = None;
        self.configured = true;
    }

    fn on_ep1_transfer(&mut self, token: UsbToken, tog_ok: bool) {
        let d = dregs();
        match token {
            UsbToken::OUT => {
                if !tog_ok {
                    // Duplicate packet (lost ACK): keep our toggle, stay
                    // ACK-armed; the host's retry lands with the right one.
                    return;
                }
                let n = regs().rx_len().read() as usize;
                self.rx_tog = !self.rx_tog;
                d.ep_rx_ctrl(1).modify(|w| {
                    w.set_mask_uep_r_tog(if self.rx_tog {
                        EpTog::DATA1
                    } else {
                        EpTog::DATA0
                    });
                    w.set_mask_uep_r_res(EpRxResponse::NAK);
                });
                self.rx_pending = Some(n);
            }
            UsbToken::IN => {
                self.tx_tog = !self.tx_tog;
                d.ep_tx_ctrl(1)
                    .modify(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
                self.tx_busy = false;
            }
            _ => {}
        }
    }
}

impl Default for UsbDevice {
    fn default() -> Self {
        Self::new()
    }
}
