//! The adapter client as a JS class. One command in flight is the adapter
//! contract, so the wrapped client sits in a RefCell and an overlapping
//! call fails with "busy" rather than queueing.

// The RefMut is held across the await on purpose: it IS the busy gate.
#![allow(clippy::await_holding_refcell_ref)]

use std::cell::{RefCell, RefMut};
use std::time::Duration;

use osc_client::descriptor as desc;
use osc_client::pipe::Pipe;
use osc_client::webusb::{UsbDevice, WebUsbPipe};
use osc_client::{Client, Error, Id, Inst, Opcode, Outcome, ResultCode, common, mgmt};
use osc_protocol::build;
use tsify::{Ts, Tsify};
use wasm_bindgen::prelude::*;

#[cfg(feature = "fake")]
use osc_client::fake::{FakePipe, TelSample};
#[cfg(feature = "fake")]
use osc_protocol::wire::UID_LEN;

use crate::descriptor::Descriptor;
use crate::types::{Alive, BaudRate, Found, Health, Identity, LinkInfo, Ping, Rails, TelBurst};
#[cfg(feature = "fake")]
use crate::types::{FakeServo, Track};

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
    /// Open, configure and claim the device, then HELLO.
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

    /// One TEL burst (protocol sec 5.6): write `tel_mask`, then a
    /// stream-tagged write of `tel_count` arms the burst and collects it
    /// within `window_us`. The registers' addresses come from `d`.
    #[wasm_bindgen(js_name = telBurst)]
    pub async fn tel_burst(
        &self,
        id: u8,
        d: &Descriptor,
        mask: u16,
        count: u16,
        #[wasm_bindgen(js_name = windowUs)] window_us: u32,
    ) -> Result<Ts<TelBurst>, JsError> {
        let mask_f = d.field("tel_mask")?;
        let count_f = d.field("tel_count")?;
        let mask_b = desc::encode(mask_f, &desc::Value::Uint(mask as u64))?;
        let count_b = desc::encode(count_f, &desc::Value::Uint(count as u64))?;
        let id = Id::new(id);
        let window = Duration::from_micros(window_us as u64);
        let out = cmd!(self, |c| burst(
            c,
            id,
            (mask_f.addr, &mask_b),
            (count_f.addr, &count_b),
            window
        )
        .await?);
        Ok(out.into_ts()?)
    }
}

/// The simulated adapter: the production link server and servo stacks over
/// the DES sim, in the wasm module. Sim time, so every window resolves at
/// once and a run is deterministic.
#[cfg(feature = "fake")]
#[wasm_bindgen]
impl OscClient {
    /// A fleet of `ids` on a simulated bus, already HELLOed. No hardware,
    /// no user gesture; each servo's UID is derived from its id.
    pub async fn fake(
        #[wasm_bindgen(unchecked_param_type = "number[]")] ids: Vec<u8>,
    ) -> Result<OscClient, JsError> {
        Self::fake_fleet(ids, Vec::new()).await
    }

    /// `fake` with an optional recorded track per servo: the sim plays
    /// each track back at the fast-tick rate, so bursts and the live
    /// telemetry registers show captured data instead of the synthetic
    /// ramp. A track's columns must be non-empty and equal in length.
    #[wasm_bindgen(js_name = fakeWithTracks)]
    pub async fn fake_with_tracks(
        #[wasm_bindgen(unchecked_param_type = "FakeServo[]")] fleet: JsValue,
    ) -> Result<OscClient, JsError> {
        let fleet: Vec<FakeServo> = serde_wasm_bindgen::from_value(fleet)?;
        let mut ids = Vec::with_capacity(fleet.len());
        let mut tracks = Vec::with_capacity(fleet.len());
        for s in fleet {
            ids.push(s.id);
            tracks.push(s.track.map(track_rows).transpose()?);
        }
        Self::fake_fleet(ids, tracks).await
    }

    /// `tracks` pairs with `ids` by index; shorter (or empty) leaves the
    /// rest on the synthetic samples.
    async fn fake_fleet(
        ids: Vec<u8>,
        tracks: Vec<Option<Vec<TelSample>>>,
    ) -> Result<OscClient, JsError> {
        if ids.is_empty() {
            return Err(JsError::new("fake: the fleet needs at least one id"));
        }
        for (i, id) in ids.iter().enumerate() {
            if ids[..i].contains(id) {
                return Err(JsError::new(&format!("fake: duplicate id {id}")));
            }
        }
        // The rate a servo leaves the factory at, so the fleet answers
        // before any migration.
        let mut pipe = FakePipe::new(osc_client::BaudRate::B1000000, &ids);
        for (i, &id) in ids.iter().enumerate() {
            pipe.sim_mut().seed_servo_uid(i, uid(id));
            seed_calibrated(&mut pipe, i);
        }
        for (i, track) in tracks.into_iter().enumerate() {
            if let Some(track) = track {
                pipe.set_track(i, track);
            }
        }
        let c = Client::connect(pipe).await?;
        Ok(OscClient {
            info: c.info(),
            inner: RefCell::new(Some(Backend::Fake(Box::new(c)))),
        })
    }
}

/// Overwrite servo `i`'s calibration with the recorded table
/// ([`crate::fake_seed`]): the sim seeds a blank one, which the app reads as
/// uncalibrated and renders in raw counts. Sense fields the sim already
/// matches stay as it seeded them.
#[cfg(feature = "fake")]
fn seed_calibrated(pipe: &mut FakePipe, i: usize) {
    use crate::fake_seed as s;

    pipe.sim_mut().servo_table_mut(i, |t| {
        t.calib.pot_lut.raw_min = s::RAW_MIN;
        t.calib.pot_lut.raw_max = s::RAW_MAX;
        t.calib.pot_lut.lut_corr = s::LUT_CORR;
        t.calib.kinematics.angle_min_cdeg = s::ANGLE_MIN_CDEG;
        t.calib.kinematics.angle_max_cdeg = s::ANGLE_MAX_CDEG;
        t.calib.kinematics.gear_ratio_centi = s::GEAR_RATIO_CENTI;
        t.calib.sense.shunt_r_mohm = s::SHUNT_R_MOHM;
        t.calib.sense.vmotor_div_top = s::VMOTOR_DIV_TOP;
        t.calib.sense.vmotor_div_bot = s::VMOTOR_DIV_BOT;
        t.calib.sense_ext.vbus_div_top_ohm = s::VBUS_DIV_TOP_OHM;
        t.calib.sense_ext.vmotor_bias_nom_counts = s::VMOTOR_BIAS_NOM_COUNTS;
        t.config.pos_limits.pos_min_phys_counts = s::POS_MIN_PHYS_COUNTS;
        t.config.pos_limits.pos_max_phys_counts = s::POS_MAX_PHYS_COUNTS;
        t.config.pos_limits.pos_min_soft_counts = s::POS_MIN_SOFT_COUNTS;
        t.config.pos_limits.pos_max_soft_counts = s::POS_MAX_SOFT_COUNTS;
        t.config.limits.drive_polarity = s::DRIVE_POLARITY;
    });
}

/// Columnar track to the sim's row form.
#[cfg(feature = "fake")]
fn track_rows(t: Track) -> Result<Vec<TelSample>, JsError> {
    let n = t.pos.len();
    if n == 0 {
        return Err(JsError::new("fakeWithTracks: empty track"));
    }
    let lens = [
        t.current.len(),
        t.current_trough.len(),
        t.duty_q15.len(),
        t.vdiff.len(),
        t.vbus.len(),
        t.current_raw.len(),
        t.vmotor_a.len(),
        t.vmotor_b.len(),
        t.vbus_raw.len(),
        t.ntc_raw.len(),
        t.window_valid.len(),
    ];
    if lens.iter().any(|&l| l != n) {
        return Err(JsError::new(
            "fakeWithTracks: track columns differ in length",
        ));
    }
    Ok((0..n)
        .map(|i| TelSample {
            pos: t.pos[i],
            current: t.current[i],
            current_trough: t.current_trough[i],
            duty_q15: t.duty_q15[i],
            vdiff: t.vdiff[i],
            vbus: t.vbus[i],
            current_raw: t.current_raw[i],
            vmotor_a: t.vmotor_a[i],
            vmotor_b: t.vmotor_b[i],
            vbus_raw: t.vbus_raw[i],
            ntc_raw: t.ntc_raw[i],
            window_valid: t.window_valid[i] != 0,
            fault: false,
        })
        .collect())
}

/// splitmix64 over the id: the same roster always discovers the same UIDs,
/// and they carry a real part's entropy rather than a run of zeros.
#[cfg(feature = "fake")]
fn uid(id: u8) -> [u8; UID_LEN] {
    let mut s = 0x05C0_DE00_0000_0000 | id as u64;
    let mut out = [0u8; UID_LEN];
    for w in out.chunks_mut(8) {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        w.copy_from_slice(&(z ^ (z >> 31)).to_le_bytes());
    }
    out
}

/// The burst choreography, written once for every backend.
async fn burst<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    mask: (u16, &[u8]),
    count: (u16, &[u8]),
    window: Duration,
) -> Result<TelBurst, JsError> {
    c.write(id, mask.0, mask.1).await?;
    let mut p = vec![0u8; count.1.len() + 4];
    let n = build::write(&mut p, count.0, count.1).ok_or(Error::Servo(ResultCode::Limit))?;
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
