//! The adapter client as a JS class. One command in flight is the adapter
//! contract, so the wrapped client sits in a RefCell and an overlapping
//! call fails with "busy" rather than queueing.

// The RefMut is held across the await on purpose: it IS the busy gate.
#![allow(clippy::await_holding_refcell_ref)]

use std::cell::{RefCell, RefMut};
use std::time::Duration;

use osc_client::boot::Image;
use osc_client::descriptor as desc;
use osc_client::pipe::Pipe;
use osc_client::webusb::{UsbDevice, WebUsbPipe};
use osc_client::{
    Client, Error, Id, Inst, Opcode, Outcome, ResultCode, common, data_state, mgmt, pos_lut, stamp,
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
    Alive, BaudRate, BootRow, DataState, DeadlineRewrite, Found, Health, Identity, InstallVerdict,
    LinkInfo, Ping, PosLut, Rails, StampVerdict, TelBurst, TelChunk, tel_chunks,
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
    inner: RefCell<Option<Backend>>,
    info: osc_client::session::LinkInfo,
}

#[wasm_bindgen]
impl OscClient {
    /// Open, configure and claim the device, then GET_INFO.
    pub async fn connect(device: UsbDevice) -> Result<OscClient, JsError> {
        let pipe = WebUsbPipe::open(device).await?;
        let c = Client::connect(pipe).await?;
        Ok(OscClient {
            info: c.info(),
            inner: RefCell::new(Some(Backend::Usb(c))),
        })
    }

    pub async fn close(&self) -> Result<(), JsError> {
        let b = self
            .inner
            .try_borrow_mut()
            .map_err(|_| busy())?
            .take()
            .ok_or_else(closed)?;
        match b {
            Backend::Usb(c) => Ok(c.into_pipe().close().await?),
            #[cfg(feature = "fake")]
            Backend::Fake(_) => Ok(()),
        }
    }

    #[wasm_bindgen(js_name = linkInfo)]
    pub fn link_info(&self) -> Result<Ts<LinkInfo>, JsError> {
        Ok(LinkInfo::from(self.info).into_ts()?)
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

    /// `data_flags` and `fault_code` decoded: the reasons closed loop is
    /// refused, most urgent first, with the operator line.
    #[wasm_bindgen(js_name = dataState)]
    pub async fn data_state(&self, id: u8, d: &Descriptor) -> Result<Ts<DataState>, JsError> {
        let s = cmd!(self, |c| data_state::read(c, Id::new(id), d.inner())
            .await?);
        Ok(DataState::from(s).into_ts()?)
    }

    /// The stored `plant_stamp` beside the stamp the live covered set
    /// computes to (the descriptor's `stamp` recipe), over the position table
    /// points the kernel applies.
    #[wasm_bindgen(js_name = plantStamp)]
    pub async fn plant_stamp(&self, id: u8, d: &Descriptor) -> Result<Ts<StampVerdict>, JsError> {
        let v = cmd!(self, |c| stamp::verdict(c, Id::new(id), d.inner()).await?);
        Ok(StampVerdict::from(v).into_ts()?)
    }

    /// Stamp the set the servo holds now; resolves to the stamp written.
    /// The servo verifies it only with torque off - under torque it lands
    /// unverified and STAMP_MISMATCH waits for the next torque-off
    /// checkpoint.
    pub async fn restamp(&self, id: u8, d: &Descriptor) -> Result<u16, JsError> {
        Ok(cmd!(self, |c| stamp::restamp(c, Id::new(id), d.inner()).await?))
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
    /// table is LIVE point for point, rejects with the servo's reason (ends,
    /// shape, torque). Torque must be off. Never stamps: the COMMIT
    /// checkpoint leaves STAMP_MISMATCH until the set is re-identified.
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

    /// The identity as a LIVE all-zero table.
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
    /// servo boots calibrated, identified and stamped (`dataState` clean).
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
    /// identified or saved, `therm` ("on", "off" or "derating") for a
    /// calibrated servo's winding thermometer, `responseDeadlineUs` for
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
            info: c.info(),
            inner: RefCell::new(Some(Backend::Fake(Box::new(c)))),
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
        RefMut::filter_map(g, Option::as_mut).map_err(|_| closed())
    }
}

fn busy() -> JsError {
    JsError::new("busy: a command is in flight")
}

fn closed() -> JsError {
    JsError::new("closed")
}

fn list<T: serde::Serialize>(items: impl Iterator<Item = T>) -> Result<JsValue, JsError> {
    let v: Vec<T> = items.collect();
    Ok(serde_wasm_bindgen::to_value(&v)?)
}
