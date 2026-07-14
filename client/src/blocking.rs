//! Blocking facade: every async method wrapped in `block_on`, one for one.
//! The async core stays the single implementation.

use std::time::Duration;

use futures_lite::future::block_on;
use osc_protocol::wire::{BaudRate, Id, Inst};

use crate::client::{Chain, Ping, Reply};
use crate::common::{self, Health, Identity};
use crate::cyclic::Cycle;
use crate::error::Error;
use crate::mgmt::{self, CalTrace, Uid};
use crate::pipe::Pipe;
use crate::session::LinkInfo;

pub struct Client<P: Pipe>(crate::Client<P>);

impl<P: Pipe> Client<P> {
    pub fn connect(pipe: P) -> Result<Self, Error> {
        Ok(Self(block_on(crate::Client::connect(pipe))?))
    }

    pub fn info(&self) -> LinkInfo {
        self.0.info()
    }

    /// Reach the transport (the fake backend's sim hooks live there).
    pub fn pipe_mut(&mut self) -> &mut P {
        self.0.pipe_mut()
    }

    pub fn set_guard(&mut self, guard: Duration) {
        self.0.set_guard(guard);
    }

    pub fn ticks(&self, span: u32) -> Duration {
        self.0.ticks(span)
    }

    pub fn exchange(&mut self, id: Id, inst: Inst, payload: &[u8]) -> Result<Reply, Error> {
        block_on(self.0.exchange(id, inst, payload))
    }

    /// Quiet bus time (sec 8 pacing): wall time on hardware, sim time on
    /// the fake adapter.
    pub fn pause(&mut self, d: Duration) {
        block_on(self.0.pause(d));
    }

    pub fn ping(&mut self, id: Id) -> Result<Ping, Error> {
        block_on(self.0.ping(id))
    }

    pub fn read(&mut self, id: Id, addr: u16, count: u16) -> Result<Vec<u8>, Error> {
        block_on(self.0.read(id, addr, count))
    }

    pub fn write(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<(), Error> {
        block_on(self.0.write(id, addr, data))
    }

    pub fn write_hold(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<(), Error> {
        block_on(self.0.write_hold(id, addr, data))
    }

    pub fn write_noreply(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<(), Error> {
        block_on(self.0.write_noreply(id, addr, data))
    }

    pub fn gread(&mut self, ids: &[Id], addr: u16, count: u16) -> Result<Chain, Error> {
        block_on(self.0.gread(ids, addr, count))
    }

    pub fn gwrite(&mut self, addr: u16, count: u8, pairs: &[(Id, &[u8])]) -> Result<(), Error> {
        block_on(self.0.gwrite(addr, count, pairs))
    }

    pub fn gwrite_hold(
        &mut self,
        addr: u16,
        count: u8,
        pairs: &[(Id, &[u8])],
    ) -> Result<(), Error> {
        block_on(self.0.gwrite_hold(addr, count, pairs))
    }

    pub fn commit(&mut self) -> Result<(), Error> {
        block_on(self.0.commit())
    }

    pub fn rescue(&mut self) -> Result<(), Error> {
        block_on(self.0.rescue())
    }

    pub fn host_baud(&mut self, rate: BaudRate) -> Result<(), Error> {
        block_on(self.0.host_baud(rate))
    }

    pub fn set_response_deadline(&mut self, us: u16) -> Result<(), Error> {
        block_on(self.0.set_response_deadline(us))
    }

    pub fn set_rails(&mut self, v3v3: bool, v5: bool) -> Result<(), Error> {
        block_on(self.0.set_rails(v3v3, v5))
    }

    pub fn enter_bootloader(&mut self) -> Result<(), Error> {
        block_on(self.0.enter_bootloader())
    }

    pub fn discover(&mut self) -> Result<Vec<Uid>, Error> {
        block_on(mgmt::discover(&mut self.0))
    }

    pub fn assign(&mut self, uid: &Uid, new_id: Id) -> Result<(), Error> {
        block_on(mgmt::assign(&mut self.0, uid, new_id))
    }

    pub fn cal(&mut self, trains: u8, gap_us: u16, gaps: u8) -> Result<(), Error> {
        block_on(mgmt::cal(&mut self.0, trains, gap_us, gaps))
    }

    pub fn save(&mut self, id: Id) -> Result<(), Error> {
        block_on(mgmt::save(&mut self.0, id))
    }

    pub fn reboot(&mut self, id: Id) -> Result<(), Error> {
        block_on(mgmt::reboot(&mut self.0, id))
    }

    pub fn factory(&mut self, id: Id) -> Result<(), Error> {
        block_on(mgmt::factory(&mut self.0, id))
    }

    pub fn rescue_sweep(&mut self, ids: &[Id]) -> Result<Vec<(Id, bool)>, Error> {
        block_on(mgmt::rescue_sweep(&mut self.0, ids))
    }

    pub fn set_baud(&mut self, ids: &[Id], rate: BaudRate) -> Result<Vec<(Id, bool)>, Error> {
        block_on(mgmt::set_baud(&mut self.0, ids, rate))
    }

    pub fn cal_verify(
        &mut self,
        ids: &[Id],
        trains: u8,
        gap_us: u16,
        gaps: u8,
    ) -> Result<Vec<CalTrace>, Error> {
        block_on(mgmt::cal_verify(&mut self.0, ids, trains, gap_us, gaps))
    }

    pub fn identity(&mut self, id: Id) -> Result<Identity, Error> {
        block_on(common::identity(&mut self.0, id))
    }

    pub fn health(&mut self, id: Id) -> Result<Health, Error> {
        block_on(common::health(&mut self.0, id))
    }

    pub fn clear_counters(&mut self, id: Id) -> Result<(), Error> {
        block_on(common::clear_counters(&mut self.0, id))
    }

    pub fn gread_profile(&mut self, ids: &[Id], slot: u8) -> Result<Chain, Error> {
        block_on(self.0.gread_profile(ids, slot))
    }

    /// One [`Cycle`] exchange (see `cyclic`): held group writes, COMMIT,
    /// telemetry chain.
    pub fn step(&mut self, cycle: &Cycle, payloads: &[&[u8]]) -> Result<Chain, Error> {
        block_on(cycle.step(&mut self.0, payloads))
    }
}
