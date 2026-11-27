//! The adapter client as a JS class. One command in flight is the adapter
//! contract, so the wrapped client sits in a RefCell and an overlapping
//! call fails with "busy" rather than queueing.
//!
//! `release` and `reclaim` hand the device to another wasm module and take
//! it back (the identification run's): the modules hold separate memories,
//! so they cannot share a client, and one owner of the bus at a time means
//! this client is closed while the other holds the device.

// The RefMut is held across the await on purpose: it IS the busy gate.
#![allow(clippy::await_holding_refcell_ref)]

use std::cell::{Cell, RefCell, RefMut};
use std::time::Duration;

use osc_client::boot::Image;
use osc_client::descriptor as desc;
use osc_client::pipe::Pipe;
use osc_client::webusb::{UsbDevice, WebUsbPipe, close_device};
use osc_client::{
    Client, Error, Id, Inst, Opcode, Outcome, ResultCode, common, mgmt, pos_lut, store_state,
};
use osc_protocol::build;
use tsify::{Ts, Tsify};
use wasm_bindgen::prelude::*;

#[cfg(feature = "fake")]
use osc_client::fake::FakePipe;

use crate::descriptor::Descriptor;
#[cfg(feature = "fake")]
use crate::types::FakeServo;
use crate::types::{
    Alive, BaudRate, BootRow, DeadlineRewrite, Found, Health, Identity, InstallVerdict, LinkInfo,
    Ping, PosLut, Rails, StoreState, TelBurst, TelChunk, tel_chunks,
};

/// Floor under `setGuard`: below this the watchdog would fire inside a
/// healthy exchange and every command would look like a dead adapter.
const MIN_GUARD_MS: f64 = 10.0;

/// Prompt for an osc-adapter; must run from a user gesture.
#[wasm_bindgen(js_name = requestDevice)]
pub async fn request_device() -> Result<UsbDevice, JsError> {
    Ok(osc_client::webusb::request_device().await?)
}

/// The open transport. [`Client`] is generic over it, so a command differs
/// only in which arm it lands in.
enum Backend {
    Usb(Client<WebUsbPipe>),
    // Boxed: the sim rig behind it dwarfs a usb client, and every
    // OscClient would otherwise carry that footprint.
    #[cfg(feature = "fake")]
    Fake(Box<Client<FakePipe>>),
}

/// Where the client stands: open on a backend, its device lent to another
/// module, or closed for good.
enum State {
    Open(Backend),
    Lent(Lent),
    Closed,
}

/// What `release` keeps for `reclaim`.
enum Lent {
    Usb {
        device: UsbDevice,
        guard: Duration,
    },
    // The simulated fleet lives in this module, so nothing else can open
    // it; it is set aside so the client still answers "released".
    #[cfg(feature = "fake")]
    Fake(Box<Client<FakePipe>>),
}

/// Run one command body against whichever backend is open, so each command
/// stays a single line and the two transports never drift apart.
macro_rules! cmd {
    ($self:expr, |$c:ident| $body:expr) => {{
        let mut g = $self.backend()?;
        match &mut *g {
            Backend::Usb($c) => $body,
            #[cfg(feature = "fake")]
            Backend::Fake(b) => {
                let $c = &mut **b;
                $body
            }
        }
    }};
}

#[wasm_bindgen]
pub struct OscClient {
    inner: RefCell<State>,
    info: Cell<osc_client::session::LinkInfo>,
}

#[wasm_bindgen]
impl OscClient {
    /// Open, configure and claim the device, then GET_INFO.
    pub async fn connect(device: UsbDevice) -> Result<OscClient, JsError> {
        let pipe = WebUsbPipe::open(device).await?;
        let c = Client::connect(pipe).await?;
        Ok(OscClient {
            info: Cell::new(c.info()),
            inner: RefCell::new(State::Open(Backend::Usb(c))),
        })
    }

    pub async fn close(&self) -> Result<(), JsError> {
        let mut g = self.inner.try_borrow_mut().map_err(|_| busy())?;
        match std::mem::replace(&mut *g, State::Closed) {
            State::Open(Backend::Usb(c)) => Ok(c.into_pipe().close().await?),
            #[cfg(feature = "fake")]
            State::Open(Backend::Fake(_)) => Ok(()),
            // The device is the other module's until reclaim.
            State::Lent(_) => Ok(()),
            State::Closed => Err(closed()),
        }
    }

    /// Close the device for another module to open, keeping this client:
    /// every command answers "released" until `reclaim`. Resolves to the
    /// device, or undefined for the simulated adapter, which no other
    /// module can open.
    #[wasm_bindgen(unchecked_return_type = "USBDevice | undefined")]
    pub async fn release(&self) -> Result<JsValue, JsError> {
        let mut g = self.inner.try_borrow_mut().map_err(|_| busy())?;
        match std::mem::replace(&mut *g, State::Closed) {
            State::Open(Backend::Usb(c)) => {
                let guard = c.guard();
                let pipe = c.into_pipe();
                let device = pipe.device().clone();
                *g = State::Lent(Lent::Usb {
                    device: device.clone(),
                    guard,
                });
                pipe.close().await?;
                Ok(device.into())
            }
            #[cfg(feature = "fake")]
            State::Open(Backend::Fake(c)) => {
                *g = State::Lent(Lent::Fake(c));
                Ok(JsValue::UNDEFINED)
            }
            s @ State::Lent(_) => {
                *g = s;
                Err(released())
            }
            State::Closed => Err(closed()),
        }
    }

    /// Open the device `release` lent again: a new session (GET_INFO), the
    /// guard as it was, then the bus rate found. Resolves to that rate,
    /// undefined when no servo answers at any rate; the client is open
    /// either way.
    pub async fn reclaim(&self) -> Result<Option<Ts<BaudRate>>, JsError> {
        let mut g = self.inner.try_borrow_mut().map_err(|_| busy())?;
        let lent = match std::mem::replace(&mut *g, State::Closed) {
            State::Lent(l) => l,
            s @ State::Open(_) => {
                *g = s;
                return Err(JsError::new("reclaim: the client holds the bus"));
            }
            State::Closed => return Err(closed()),
        };
        match lent {
            Lent::Usb { device, guard } => {
                let pipe = match WebUsbPipe::open(device.clone()).await {
                    Ok(p) => p,
                    Err(e) => {
                        *g = State::Lent(Lent::Usb { device, guard });
                        return Err(e.into());
                    }
                };
                let mut c = match Client::connect(pipe).await {
                    Ok(c) => c,
                    Err(e) => {
                        let _ = close_device(&device).await;
                        *g = State::Lent(Lent::Usb { device, guard });
                        return Err(e.into());
                    }
                };
                c.set_guard(guard);
                self.info.set(c.info());
                *g = State::Open(Backend::Usb(c));
            }
            #[cfg(feature = "fake")]
            Lent::Fake(c) => *g = State::Open(Backend::Fake(c)),
        }
        drop(g);
        match cmd!(self, |c| mgmt::find_bus_baud(c).await?) {
            Some(r) => Ok(Some(BaudRate::from(r).into_ts()?)),
            None => Ok(None),
        }
    }

    #[wasm_bindgen(js_name = linkInfo)]
    pub fn link_info(&self) -> Result<Ts<LinkInfo>, JsError> {
        Ok(LinkInfo::from(self.info.get()).into_ts()?)
    }

    /// Client-side watchdog on pipe delivery, for every later command.
    #[wasm_bindgen(js_name = setGuard)]
    pub fn set_guard(&self, ms: f64) -> Result<(), JsError> {
        if !(ms.is_finite() && ms >= MIN_GUARD_MS) {
            return Err(JsError::new(&format!(
                "setGuard: the guard must be at least {MIN_GUARD_MS} ms"
            )));
        }
        let guard = Duration::from_micros((ms * 1_000.0) as u64);
        cmd!(self, |c| c.set_guard(guard));
        Ok(())
    }

    /// `transferIn` calls the transport has issued since open; the
    /// simulated adapter moves no bytes over USB and reports 0.
    #[wasm_bindgen(js_name = transfersIn)]
    pub fn transfers_in(&self) -> Result<f64, JsError> {
        Ok(match &mut *self.backend()? {
            Backend::Usb(c) => c.pipe_mut().transfers_in() as f64,
            #[cfg(feature = "fake")]
            Backend::Fake(_) => 0.0,
        })
    }

    pub async fn rails(&self) -> Result<Ts<Rails>, JsError> {
        Ok(Rails::from(cmd!(self, |c| c.rails().await?)).into_ts()?)
    }

    /// Drive both rails; resolves to the acked state.
    #[wasm_bindgen(js_name = setRails)]
    pub async fn set_rails(&self, v3v3: bool, v5: bool) -> Result<Ts<Rails>, JsError> {
        Ok(Rails::from(cmd!(self, |c| c.set_rails(v3v3, v5).await?)).into_ts()?)
    }

    #[wasm_bindgen(js_name = busPresent)]
    pub async fn bus_present(&self) -> Result<bool, JsError> {
        Ok(cmd!(self, |c| mgmt::bus_present(c).await?))
    }

    /// Probe the rates for the bus; the host stays at the found rate.
    #[wasm_bindgen(js_name = findBusBaud)]
    pub async fn find_bus_baud(&self) -> Result<Option<Ts<BaudRate>>, JsError> {
        match cmd!(self, |c| mgmt::find_bus_baud(c).await?) {
            Some(r) => Ok(Some(BaudRate::from(r).into_ts()?)),
            None => Ok(None),
        }
    }

    /// Host-side UART rate only; `setBaud` migrates a fleet.
    #[wasm_bindgen(js_name = hostBaud)]
    pub async fn host_baud(&self, rate: Ts<BaudRate>) -> Result<(), JsError> {
        let rate: BaudRate = rate.to_rust()?;
        cmd!(self, |c| c.host_baud(rate.into()).await?);
        Ok(())
    }

    #[wasm_bindgen(unchecked_return_type = "Found[]")]
    pub async fn discover(&self) -> Result<JsValue, JsError> {
        let found = cmd!(self, |c| mgmt::discover(c).await?);
        list(found.iter().map(Found::from))
    }

    pub async fn ping(&self, id: u8) -> Result<Ts<Ping>, JsError> {
        Ok(Ping::from(cmd!(self, |c| c.ping(Id::new(id)).await?)).into_ts()?)
    }

    pub async fn identity(&self, id: u8) -> Result<Ts<Identity>, JsError> {
        let v = cmd!(self, |c| common::identity(c, Id::new(id)).await?);
        Ok(Identity::from(v).into_ts()?)
    }

    pub async fn health(&self, id: u8) -> Result<Ts<Health>, JsError> {
        let v = cmd!(self, |c| common::health(c, Id::new(id)).await?);
        Ok(Health::from(v).into_ts()?)
    }

    #[wasm_bindgen(js_name = clearCounters)]
    pub async fn clear_counters(&self, id: u8) -> Result<(), JsError> {
        cmd!(self, |c| common::clear_counters(c, Id::new(id)).await?);
        Ok(())
    }

    /// `status_flags`, `refused_modes` and `fault_code` decoded: the
    /// images on their defaults, unsaved changes and the modes the belt
    /// refuses, with the operator lines.
    #[wasm_bindgen(js_name = storeState)]
    pub async fn store_state(&self, id: u8, d: &Descriptor) -> Result<Ts<StoreState>, JsError> {
        let s = cmd!(self, |c| store_state::read(c, Id::new(id), d.inner())
            .await?);
        Ok(StoreState::from(s).into_ts()?)
    }

    /// The position table as the servo holds it: every page FETCHed, with
    /// `pos_lut_state`.
    #[wasm_bindgen(js_name = readPosLut)]
    pub async fn read_pos_lut(&self, id: u8, d: &Descriptor) -> Result<Ts<PosLut>, JsError> {
        let l = cmd!(self, |c| pos_lut::read(c, Id::new(id), d.inner()).await?);
        Ok(PosLut::from(l).into_ts()?)
    }

    /// `pos_lut_state` alone.
    #[wasm_bindgen(js_name = posLutState)]
    pub async fn pos_lut_state(&self, id: u8, d: &Descriptor) -> Result<u8, JsError> {
        Ok(cmd!(self, |c| pos_lut::state(c, Id::new(id), d.inner()).await?))
    }

    /// STORE the 256 points, COMMIT and read them back: resolves once the
    /// table is LIVE point for point (IDENTITY when every point is zero),
    /// rejects with the servo's reason (ends, shape, torque). Torque must
    /// be off. The servo reads unsaved after it until SAVE.
    #[wasm_bindgen(js_name = writePosLut)]
    pub async fn write_pos_lut(
        &self,
        id: u8,
        d: &Descriptor,
        #[wasm_bindgen(unchecked_param_type = "number[]")] points: Vec<i16>,
    ) -> Result<(), JsError> {
        let points: [i16; pos_lut::INTERVALS] = points.try_into().map_err(|k: Vec<i16>| {
            JsError::new(&format!(
                "writePosLut: {} points expected, got {}",
                pos_lut::INTERVALS,
                k.len()
            ))
        })?;
        cmd!(self, |c| pos_lut::write(c, Id::new(id), d.inner(), &points)
            .await?);
        Ok(())
    }

    /// Back to IDENTITY: an all-zero table STOREd and COMMITted.
    #[wasm_bindgen(js_name = clearPosLut)]
    pub async fn clear_pos_lut(&self, id: u8, d: &Descriptor) -> Result<(), JsError> {
        cmd!(self, |c| pos_lut::clear(c, Id::new(id), d.inner()).await?);
        Ok(())
    }

    pub async fn read(&self, id: u8, addr: u16, count: u16) -> Result<Vec<u8>, JsError> {
        Ok(cmd!(self, |c| c.read(Id::new(id), addr, count).await?))
    }

    pub async fn write(&self, id: u8, addr: u16, data: &[u8]) -> Result<(), JsError> {
        cmd!(self, |c| c.write(Id::new(id), addr, data).await?);
        Ok(())
    }

    /// HOLD-staged write: applied by the next `commit`.
    #[wasm_bindgen(js_name = writeHold)]
    pub async fn write_hold(&self, id: u8, addr: u16, data: &[u8]) -> Result<(), JsError> {
        cmd!(self, |c| c.write_hold(Id::new(id), addr, data).await?);
        Ok(())
    }

    /// Broadcast COMMIT: every held write applies in the same instant.
    pub async fn commit(&self) -> Result<(), JsError> {
        cmd!(self, |c| c.commit().await?);
        Ok(())
    }

    /// Broadcast ASSIGN: the UID's owner takes `new_id`.
    pub async fn assign(
        &self,
        uid: &str,
        #[wasm_bindgen(js_name = newId)] new_id: u8,
    ) -> Result<(), JsError> {
        let uid = crate::uid::parse(uid).map_err(|e| JsError::new(&e))?;
        cmd!(self, |c| mgmt::assign(c, &uid, Id::new(new_id)).await?);
        Ok(())
    }

    /// Fleet baud migration, servo-first; resolves to the reunion roster.
    #[wasm_bindgen(js_name = setBaud, unchecked_return_type = "Alive[]")]
    pub async fn set_baud(
        &self,
        #[wasm_bindgen(unchecked_param_type = "number[]")] ids: Vec<u8>,
        rate: Ts<BaudRate>,
    ) -> Result<JsValue, JsError> {
        let rate: BaudRate = rate.to_rust()?;
        let ids: Vec<Id> = ids.into_iter().map(Id::new).collect();
        let roster = cmd!(self, |c| mgmt::set_baud(c, &ids, rate.into()).await?);
        list(roster.iter().map(|&(id, alive)| Alive {
            id: id.as_byte(),
            alive,
        }))
    }

    /// Make `us` the bus's response deadline: the adapter mirror, then each
    /// of `ids` that holds another value; resolves to the servos rewritten.
    #[wasm_bindgen(js_name = syncResponseDeadline, unchecked_return_type = "DeadlineRewrite[]")]
    pub async fn sync_response_deadline(
        &self,
        #[wasm_bindgen(unchecked_param_type = "number[]")] ids: Vec<u8>,
        us: u16,
    ) -> Result<JsValue, JsError> {
        let ids: Vec<Id> = ids.into_iter().map(Id::new).collect();
        let rewritten = cmd!(self, |c| common::sync_response_deadline(c, &ids, us)
            .await?);
        list(rewritten.iter().map(|&(id, held)| DeadlineRewrite {
            id: id.as_byte(),
            held,
        }))
    }

    pub async fn save(&self, id: u8) -> Result<(), JsError> {
        cmd!(self, |c| mgmt::save(c, Id::new(id)).await?);
        Ok(())
    }

    pub async fn reboot(&self, id: u8) -> Result<(), JsError> {
        cmd!(self, |c| mgmt::reboot(c, Id::new(id)).await?);
        Ok(())
    }

    pub async fn factory(&self, id: u8) -> Result<(), JsError> {
        cmd!(self, |c| mgmt::factory(c, Id::new(id)).await?);
        Ok(())
    }

    /// One TEL burst (protocol sec 5.6): a stream-tagged write of the
    /// chunks and `tel_count`, one span, arms the burst and collects it
    /// within `window_us`. The registers' addresses come from `d`. The bus
    /// must run at `telBaud()`: below it the servo acks and never streams.
    #[wasm_bindgen(js_name = telBurst)]
    pub async fn tel_burst(
        &self,
        id: u8,
        d: &Descriptor,
        #[wasm_bindgen(unchecked_param_type = "TelChunk[]")] chunks: JsValue,
        count: u16,
        #[wasm_bindgen(js_name = windowUs)] window_us: u32,
    ) -> Result<Ts<TelBurst>, JsError> {
        let chunks: Vec<TelChunk> = serde_wasm_bindgen::from_value(chunks)?;
        let c = tel_chunks(&chunks)
            .filter(|c| c.valid())
            .ok_or_else(|| JsError::new("telBurst: the chunks make no row"))?;
        let first = d.field("chunk0_addr")?;
        let count_f = d.field("tel_count")?;
        let span = c.to_le_bytes().len() as u16;
        if count_f.addr != first.addr + span {
            return Err(JsError::new(
                "telBurst: tel_count does not follow the chunks",
            ));
        }
        let mut arm = c.to_le_bytes().to_vec();
        arm.extend(desc::encode(count_f, &desc::Value::Uint(count as u64))?);
        let id = Id::new(id);
        let window = Duration::from_micros(window_us as u64);
        let out = cmd!(self, |c| burst(c, id, (first.addr, &arm), window).await?);
        Ok(out.into_ts()?)
    }

    /// Every roster servo at the bus rate to its bootloader at `rate`;
    /// resolves to the roster. The bus must be in osc mode and stays there.
    /// A servo whose reboot did not land stays in its app and is missing
    /// from `bootList`.
    #[wasm_bindgen(js_name = bootEnter, unchecked_return_type = "Found[]")]
    pub async fn boot_enter(&self, rate: Ts<BaudRate>) -> Result<JsValue, JsError> {
        let rate: BaudRate = rate.to_rust()?;
        let roster = cmd!(self, |c| crate::boot::enter(c, rate).await?);
        list(roster.into_iter())
    }

    /// The bus to tinyboot at `rate`, then every waiting bootloader with
    /// its Info, by uid. The bus stays in tinyboot mode for `bootInstall`
    /// and `bootResetAll`.
    #[wasm_bindgen(js_name = bootList)]
    pub async fn boot_list(&self, rate: Ts<BaudRate>) -> Result<Vec<Ts<BootRow>>, JsError> {
        let rate: BaudRate = rate.to_rust()?;
        let rows = cmd!(self, |c| crate::boot::list(c, rate).await?);
        Ok(rows
            .iter()
            .map(BootRow::into_ts)
            .collect::<Result<_, _>>()?)
    }

    /// Install a `.bin` on the bootloader `uid` (a `BootRow.uid`): refused
    /// unerased when foreign or too large, else erase, write, verify; no
    /// reset. `onProgress(written, total)` runs after each acked write, in
    /// image bytes. Resolves to the device's verdict.
    #[wasm_bindgen(js_name = bootInstall)]
    pub async fn boot_install(
        &self,
        uid: &str,
        image: &[u8],
        #[wasm_bindgen(
            js_name = onProgress,
            unchecked_param_type = "(written: number, total: number) => void"
        )]
        on_progress: &js_sys::Function,
    ) -> Result<Ts<InstallVerdict>, JsError> {
        let uid = crate::uid::boot_parse(uid).map_err(|e| JsError::new(&e))?;
        let image = Image::parse(image.to_vec())?;
        // A throwing callback cannot stop a write in flight; it is ignored.
        let progress = |written: u32, total: u32| {
            let _ = on_progress.call2(&JsValue::NULL, &written.into(), &total.into());
        };
        let v = cmd!(self, |c| crate::boot::install(c, &uid, &image, progress)
            .await?);
        Ok(v.into_ts()?)
    }

    /// Every bootloader back to its app at once, then the bus to osc at
    /// `rate`; resolves once the apps are up, so `discover` finds them.
    #[wasm_bindgen(js_name = bootResetAll)]
    pub async fn boot_reset_all(&self, rate: Ts<BaudRate>) -> Result<(), JsError> {
        let rate: BaudRate = rate.to_rust()?;
        cmd!(self, |c| crate::boot::reset_all(c, rate).await?);
        Ok(())
    }
}

/// The simulated adapter: the production link server and servo stacks over
/// the DES sim, in the wasm module. Sim time, so every window resolves at
/// once and a run is deterministic.
#[cfg(feature = "fake")]
#[wasm_bindgen]
impl OscClient {
    /// A fleet of `ids` on a simulated bus, already past GET_INFO. No hardware,
    /// no user gesture; each servo's UID is derived from its id, and every
    /// servo boots calibrated, identified and saved (`storeState` loaded).
    pub async fn fake(
        #[wasm_bindgen(unchecked_param_type = "number[]")] ids: Vec<u8>,
    ) -> Result<OscClient, JsError> {
        let fleet = ids
            .into_iter()
            .map(|id| FakeServo {
                id,
                ..Default::default()
            })
            .collect();
        Self::fake_fleet(fleet).await
    }

    /// `fake` with per-servo options: an optional recorded track (the sim
    /// plays it back at the fast-tick rate, so bursts and the live
    /// telemetry registers show captured data instead of the synthetic
    /// ramp; columns must be non-empty and equal in length), `virgin`
    /// for a factory-fresh servo that has never been calibrated,
    /// identified or saved, `therm` ("on" or "off") for a calibrated
    /// servo's winding estimate, `responseDeadlineUs` for
    /// a servo holding another deadline than the protocol default,
    /// `bootloader` for a board that carries only its bootloader, and
    /// `failNext` ("erase" or "write") for a bootloader that fails its next
    /// such command.
    #[wasm_bindgen(js_name = fakeWithTracks)]
    pub async fn fake_with_tracks(
        #[wasm_bindgen(unchecked_param_type = "FakeServo[]")] fleet: JsValue,
    ) -> Result<OscClient, JsError> {
        Self::fake_fleet(serde_wasm_bindgen::from_value(fleet)?).await
    }

    async fn fake_fleet(fleet: Vec<FakeServo>) -> Result<OscClient, JsError> {
        let pipe = crate::fake::pipe(fleet).map_err(|e| JsError::new(&e))?;
        let c = Client::connect(pipe).await?;
        Ok(OscClient {
            info: Cell::new(c.info()),
            inner: RefCell::new(State::Open(Backend::Fake(Box::new(c)))),
        })
    }
}

/// The burst choreography, written once for every backend.
async fn burst<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    arm: (u16, &[u8]),
    window: Duration,
) -> Result<TelBurst, JsError> {
    let mut p = vec![0u8; arm.1.len() + 4];
    let n = build::write(&mut p, arm.0, arm.1).ok_or(Error::Servo(ResultCode::Limit))?;
    let inst = Inst::instruction(Opcode::Write, 0);
    // The pipe guard must outlast the whole burst (see exchange_stream),
    // and the caller's own guard has to survive the detour.
    let prev = c.guard();
    c.set_guard(window + Duration::from_secs(1));
    let reply = c.exchange_stream(id, inst, &p[..n], window).await;
    c.set_guard(prev);
    let reply = reply?;
    if let Some(ack) = &reply.ack
        && ack.result != Some(ResultCode::Ok)
    {
        return Err(JsError::new(&format!(
            "stream arm answered {:?}",
            ack.result
        )));
    }
    Ok(TelBurst {
        frames: reply
            .frames
            .into_iter()
            .map(|f| serde_bytes::ByteBuf::from(f.payload))
            .collect(),
        complete: matches!(reply.outcome, Outcome::Complete),
        tick: reply.tick,
        statuses: reply.statuses,
        garble: reply.garble,
        trailing: reply.trailing,
    })
}

impl OscClient {
    fn backend(&self) -> Result<RefMut<'_, Backend>, JsError> {
        let g = self.inner.try_borrow_mut().map_err(|_| busy())?;
        let lent = matches!(*g, State::Lent(_));
        RefMut::filter_map(g, |s| match s {
            State::Open(b) => Some(b),
            _ => None,
        })
        .map_err(|_| if lent { released() } else { closed() })
    }
}

fn busy() -> JsError {
    JsError::new("busy: a command is in flight")
}

fn released() -> JsError {
    JsError::new("released: another module holds the bus until reclaim")
}

fn closed() -> JsError {
    JsError::new("closed")
}

fn list<T: serde::Serialize>(items: impl Iterator<Item = T>) -> Result<JsValue, JsError> {
    let v: Vec<T> = items.collect();
    Ok(serde_wasm_bindgen::to_value(&v)?)
}
