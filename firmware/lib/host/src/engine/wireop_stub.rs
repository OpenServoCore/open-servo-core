//! `wireop` without the `bench` feature: the names the engine calls, every
//! body a no-op, so the engine's call sites carry no cfg.

use core::marker::PhantomData;

use crate::traits::Providers;

use super::HostBus;

pub(super) struct Wire<P>(PhantomData<P>);

impl<P: Providers> Wire<P> {
    pub(super) fn new() -> Self {
        Self(PhantomData)
    }

    pub(super) fn busy(&self) -> bool {
        false
    }

    pub(super) fn take_done(&mut self) -> Option<u32> {
        None
    }
}

impl<P: Providers> HostBus<P> {
    pub(super) fn wire_tx_complete(&mut self) {}

    pub(super) fn wire_deadline(&mut self) {}

    pub(super) fn wire_poll(&mut self, _now: u32) {}
}
