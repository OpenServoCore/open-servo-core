//! The adapter client as a JS class. One command in flight is the adapter
//! contract, so the wrapped client sits in a RefCell and an overlapping
//! call fails with "busy" rather than queueing.

// The RefMut is held across the await on purpose: it IS the busy gate.
#![allow(clippy::await_holding_refcell_ref)]

use std::cell::{RefCell, RefMut};
use std::time::Duration;

use osc_client::descriptor as desc;
use osc_client::webusb::{UsbDevice, WebUsbPipe};
use osc_client::{
    Client, DEFAULT_GUARD, Error, Id, Inst, Opcode, Outcome, ResultCode, common, mgmt,
};
use osc_protocol::build;
use tsify::{Ts, Tsify};
use wasm_bindgen::prelude::*;

use crate::descriptor::Descriptor;
use crate::types::{Alive, BaudRate, Found, Health, Identity, LinkInfo, Ping, Rails, TelBurst};

/// Prompt for an osc-adapter; must run from a user gesture.
#[wasm_bindgen(js_name = requestDevice)]
pub async fn request_device() -> Result<UsbDevice, JsError> {
    Ok(osc_client::webusb::request_device().await?)
}

#[wasm_bindgen]
pub struct OscClient {
    inner: RefCell<Option<Client<WebUsbPipe>>>,
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
            inner: RefCell::new(Some(c)),
        })
    }

    pub async fn close(&self) -> Result<(), JsError> {
        let c = self
            .inner
            .try_borrow_mut()
            .map_err(|_| busy())?
            .take()
            .ok_or_else(closed)?;
        Ok(c.into_pipe().close().await?)
    }

    #[wasm_bindgen(js_name = linkInfo)]
    pub fn link_info(&self) -> Result<Ts<LinkInfo>, JsError> {
        Ok(LinkInfo::from(self.info).into_ts()?)
    }

    pub async fn rails(&self) -> Result<Ts<Rails>, JsError> {
        let mut c = self.client()?;
        Ok(Rails::from(c.rails().await?).into_ts()?)
    }

    /// Drive both rails; resolves to the acked state.
    #[wasm_bindgen(js_name = setRails)]
    pub async fn set_rails(&self, v3v3: bool, v5: bool) -> Result<Ts<Rails>, JsError> {
        let mut c = self.client()?;
        Ok(Rails::from(c.set_rails(v3v3, v5).await?).into_ts()?)
    }

    #[wasm_bindgen(js_name = busPresent)]
    pub async fn bus_present(&self) -> Result<bool, JsError> {
        let mut c = self.client()?;
        Ok(mgmt::bus_present(&mut c).await?)
    }

    /// Probe the rates for the bus; the host stays at the found rate.
    #[wasm_bindgen(js_name = findBusBaud)]
    pub async fn find_bus_baud(&self) -> Result<Option<Ts<BaudRate>>, JsError> {
        let mut c = self.client()?;
        match mgmt::find_bus_baud(&mut c).await? {
            Some(r) => Ok(Some(BaudRate::from(r).into_ts()?)),
            None => Ok(None),
        }
    }

    /// Host-side UART rate only; `setBaud` migrates a fleet.
    #[wasm_bindgen(js_name = hostBaud)]
    pub async fn host_baud(&self, rate: Ts<BaudRate>) -> Result<(), JsError> {
        let rate: BaudRate = rate.to_rust()?;
        let mut c = self.client()?;
        Ok(c.host_baud(rate.into()).await?)
    }

    #[wasm_bindgen(unchecked_return_type = "Found[]")]
    pub async fn discover(&self) -> Result<JsValue, JsError> {
        let mut c = self.client()?;
        let found = mgmt::discover(&mut c).await?;
        list(found.iter().map(Found::from))
    }

    pub async fn ping(&self, id: u8) -> Result<Ts<Ping>, JsError> {
        let mut c = self.client()?;
        Ok(Ping::from(c.ping(Id::new(id)).await?).into_ts()?)
    }

    pub async fn identity(&self, id: u8) -> Result<Ts<Identity>, JsError> {
        let mut c = self.client()?;
        Ok(Identity::from(common::identity(&mut c, Id::new(id)).await?).into_ts()?)
    }

    pub async fn health(&self, id: u8) -> Result<Ts<Health>, JsError> {
        let mut c = self.client()?;
        Ok(Health::from(common::health(&mut c, Id::new(id)).await?).into_ts()?)
    }

    #[wasm_bindgen(js_name = clearCounters)]
    pub async fn clear_counters(&self, id: u8) -> Result<(), JsError> {
        let mut c = self.client()?;
        Ok(common::clear_counters(&mut c, Id::new(id)).await?)
    }

    pub async fn read(&self, id: u8, addr: u16, count: u16) -> Result<Vec<u8>, JsError> {
        let mut c = self.client()?;
        Ok(c.read(Id::new(id), addr, count).await?)
    }

    pub async fn write(&self, id: u8, addr: u16, data: &[u8]) -> Result<(), JsError> {
        let mut c = self.client()?;
        Ok(c.write(Id::new(id), addr, data).await?)
    }

    /// HOLD-staged write: applied by the next `commit`.
    #[wasm_bindgen(js_name = writeHold)]
    pub async fn write_hold(&self, id: u8, addr: u16, data: &[u8]) -> Result<(), JsError> {
        let mut c = self.client()?;
        Ok(c.write_hold(Id::new(id), addr, data).await?)
    }

    /// Broadcast COMMIT: every held write applies in the same instant.
    pub async fn commit(&self) -> Result<(), JsError> {
        let mut c = self.client()?;
        Ok(c.commit().await?)
    }

    /// Broadcast ASSIGN: the UID's owner takes `new_id`.
    pub async fn assign(
        &self,
        uid: &str,
        #[wasm_bindgen(js_name = newId)] new_id: u8,
    ) -> Result<(), JsError> {
        let uid = crate::uid::parse(uid).map_err(|e| JsError::new(&e))?;
        let mut c = self.client()?;
        Ok(mgmt::assign(&mut c, &uid, Id::new(new_id)).await?)
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
        let mut c = self.client()?;
        let roster = mgmt::set_baud(&mut c, &ids, rate.into()).await?;
        list(roster.iter().map(|&(id, alive)| Alive {
            id: id.as_byte(),
            alive,
        }))
    }

    pub async fn save(&self, id: u8) -> Result<(), JsError> {
        let mut c = self.client()?;
        Ok(mgmt::save(&mut c, Id::new(id)).await?)
    }

    pub async fn reboot(&self, id: u8) -> Result<(), JsError> {
        let mut c = self.client()?;
        Ok(mgmt::reboot(&mut c, Id::new(id)).await?)
    }

    pub async fn factory(&self, id: u8) -> Result<(), JsError> {
        let mut c = self.client()?;
        Ok(mgmt::factory(&mut c, Id::new(id)).await?)
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
        let mut c = self.client()?;
        c.write(id, mask_f.addr, &mask_b).await?;
        let mut p = vec![0u8; count_b.len() + 4];
        let n =
            build::write(&mut p, count_f.addr, &count_b).ok_or(Error::Servo(ResultCode::Limit))?;
        let inst = Inst::instruction(Opcode::Write, 0);
        let window = Duration::from_micros(window_us as u64);
        // The pipe guard must outlast the whole burst (see exchange_stream).
        c.set_guard(window + Duration::from_secs(1));
        let reply = c.exchange_stream(id, inst, &p[..n], window).await;
        c.set_guard(DEFAULT_GUARD);
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
        }
        .into_ts()?)
    }
}

impl OscClient {
    fn client(&self) -> Result<RefMut<'_, Client<WebUsbPipe>>, JsError> {
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
